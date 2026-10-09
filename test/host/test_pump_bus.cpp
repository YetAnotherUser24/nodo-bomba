/**
 * @file test_pump_bus.cpp
 * @brief Host conformance test for the peristaltic pump + AS5600 device model.
 *
 * Runs `src/pump_sim.cpp` on a deterministic clock and asserts the contract the dosing loop
 * relies on: deadband, duty->rpm scaling, encoder revolution accumulation with 12-bit wrap,
 * and the stuck-encoder / no-flow faults.
 *
 * Build: g++ -std=c++17 -I src test/host/test_pump_bus.cpp src/pump_sim.cpp -o /tmp/test_pump_bus
 */

#include "pump_sim.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static void advance(PumpSim& sim, uint32_t& t, uint32_t ms, uint32_t stepMs = 10) {
  for (uint32_t e = 0; e < ms; e += stepMs) {
    t += stepMs;
    sim.tick(t);
  }
}

int main() {
  PumpSimSpec spec;
  spec.maxRpm = 120.0f;
  spec.minDutyPercent = 10.0f;
  spec.ticksPerRev = 4096.0f;

  PumpSim sim(spec);
  uint32_t t = 1000;
  sim.tick(t);

  // 1. At rest.
  assert(sim.rpm() == 0.0f);
  assert(sim.revolutions() == 0.0f);

  // 2. Deadband: below minDutyPercent the head does not turn.
  sim.setDutyPercent(5.0f);
  advance(sim, t, 2000);
  assert(sim.rpm() == 0.0f);

  // 3. At full duty the head reaches its rated speed.
  sim.setDutyPercent(100.0f);
  advance(sim, t, 3000);
  assert(sim.rpm() > spec.maxRpm * 0.95f);
  assert(sim.rpm() <= spec.maxRpm + 0.001f);

  // 4. Revolutions accumulate and the angle wraps within 12 bits.
  const float revAfterRun = sim.revolutions();
  assert(revAfterRun > 1.0f);
  assert(sim.rawAngle() <= 0x0FFF);

  // 5. The 12-bit angle wraps (verify by integrating enough revolutions).
  assert(sim.rawAngle() == ((uint16_t)((int64_t)(sim.revolutions() * spec.ticksPerRev) & 0x0FFF)));

  // 6. Duty is clamped to [0, 100].
  sim.setDutyPercent(250.0f);
  assert(sim.duty() == 100.0f);

  // 7. Stuck encoder freezes the angle while duty is commanded.
  sim.setFault(PumpSimFault::StuckEncoder);
  const uint16_t stuckAngle = sim.rawAngle();
  const float stuckRev = sim.revolutions();
  sim.setDutyPercent(100.0f);
  advance(sim, t, 2000);
  assert(sim.rawAngle() == stuckAngle);
  assert(sim.revolutions() == stuckRev);

  // 8. No-flow: the motor spins but delivers no revolutions.
  PumpSim nf(spec);
  uint32_t tn = 0;
  nf.tick(tn);
  nf.setFault(PumpSimFault::NoFlow);
  nf.setDutyPercent(100.0f);
  advance(nf, tn, 2000);
  assert(nf.revolutions() == 0.0f);

  // 9. Data-driven fault schedule fires once at its timestamp.
  PumpSim scheduled(spec);
  scheduled.addFaultEvent({5000u, PumpSimFault::StuckEncoder});
  uint32_t ts = 0;
  scheduled.tick(ts);
  advance(scheduled, ts, 4000);
  assert(scheduled.fault() == PumpSimFault::None);
  advance(scheduled, ts, 2000);
  assert(scheduled.fault() == PumpSimFault::StuckEncoder);

  std::printf("test_pump_bus: all assertions passed\n");
  return 0;
}

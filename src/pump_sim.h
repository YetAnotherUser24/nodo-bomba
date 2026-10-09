#pragma once
/**
 * @file pump_sim.h
 * @brief Declarative model of the peristaltic pump head + AS5600 encoder. Host-testable.
 *
 * The sim models the *device*: duty -> steady-state rpm with a first-order spin-up, and an
 * encoder angle that advances at that rpm (4096 ticks/rev, 12-bit wrap). Faults are data-driven.
 */

#include <stdint.h>

enum class PumpSimFault : uint8_t {
  None = 0,
  StuckEncoder,  // angle frozen while the motor is commanded
  NoFlow,        // motor spins but delivers no revolutions (slipping head)
};

struct PumpSimFaultEvent {
  uint32_t atMs;
  PumpSimFault kind;
};

struct PumpSimSpec {
  float maxRpm = 120.0f;        // Head speed at 100% duty
  float minDutyPercent = 10.0f; // Below this the head does not turn
  float spinUpTauS = 0.15f;     // First-order motor time constant (s)
  float ticksPerRev = 4096.0f;  // AS5600 12-bit resolution
};

class PumpSim {
public:
  explicit PumpSim(const PumpSimSpec& spec = PumpSimSpec());

  void reset();
  void setSpec(const PumpSimSpec& spec) { _spec = spec; }

  void setDutyPercent(float percent);
  void tick(uint32_t nowMs);

  uint16_t rawAngle() const { return _angle; }
  float rpm() const { return _rpm; }
  float duty() const { return _duty; }
  float revolutions() const { return _rev; }
  PumpSimFault fault() const { return _fault; }

  void addFaultEvent(const PumpSimFaultEvent& ev);
  void clearFaultSchedule() { _faultCount = 0; }
  void setFault(PumpSimFault kind) { _fault = kind; }

  static constexpr uint8_t kMaxFaults = 8;

private:
  PumpSimSpec _spec;
  float _duty = 0.0f;
  float _rpm = 0.0f;
  float _rev = 0.0f;
  uint16_t _angle = 0;
  uint32_t _lastMs = 0;
  PumpSimFault _fault = PumpSimFault::None;
  uint32_t _faultFired = 0;
  PumpSimFaultEvent _faults[kMaxFaults] = {};
  uint8_t _faultCount = 0;
};

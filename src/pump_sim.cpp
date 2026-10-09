#include "pump_sim.h"

PumpSim::PumpSim(const PumpSimSpec& spec) : _spec(spec) { reset(); }

void PumpSim::reset() {
  _duty = 0.0f;
  _rpm = 0.0f;
  _rev = 0.0f;
  _angle = 0;
  _lastMs = 0;
  _fault = PumpSimFault::None;
  _faultFired = 0;
}

void PumpSim::setDutyPercent(float percent) {
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;
  _duty = percent;
}

void PumpSim::addFaultEvent(const PumpSimFaultEvent& ev) {
  if (_faultCount < kMaxFaults) _faults[_faultCount++] = ev;
}

void PumpSim::tick(uint32_t nowMs) {
  if (_lastMs == 0) {
    _lastMs = nowMs;
    return;
  }
  float dt = (float)(nowMs - _lastMs) / 1000.0f;
  _lastMs = nowMs;
  if (dt <= 0.0f) return;
  if (dt > 0.5f) dt = 0.5f;

  for (uint8_t i = 0; i < _faultCount; ++i) {
    const bool fired = (_faultFired & (1u << i)) != 0;
    if (!fired && nowMs >= _faults[i].atMs) {
      _faultFired |= (1u << i);
      _fault = _faults[i].kind;
    }
  }

  // Duty -> steady-state rpm (with deadband).
  float targetRpm = 0.0f;
  if (_fault != PumpSimFault::NoFlow && _fault != PumpSimFault::StuckEncoder) {
    if (_duty >= _spec.minDutyPercent) {
      targetRpm = (_duty / 100.0f) * _spec.maxRpm;
    }
  }

  const float alpha = dt / (_spec.spinUpTauS + dt);
  _rpm += (targetRpm - _rpm) * alpha;
  if (_rpm < 0.1f && targetRpm == 0.0f) _rpm = 0.0f;

  // Integrate revolutions unless the encoder is stuck or flow is lost.
  if (_fault != PumpSimFault::StuckEncoder && _fault != PumpSimFault::NoFlow) {
    _rev += (_rpm / 60.0f) * dt;
    float ticks = _rev * _spec.ticksPerRev;
    _angle = (uint16_t)((int64_t)ticks & 0x0FFF);  // 12-bit wrap
  }
}

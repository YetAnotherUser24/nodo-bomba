/**
 * @file pump_bus_virtual.cpp
 * @brief Virtual `IPwmActuator` + `IRotaryEncoder`: binds PumpSim to the node interface.
 *
 * Build: selected by `-DPUMP_VIRTUAL=1` (env `esp32dev-virtual`). The dosing loop above it is
 * the production controller, unchanged.
 */

#include "pump_bus.h"
#include "pump_sim.h"

#include <Arduino.h>

namespace {

class VirtualActuator : public IPwmActuator {
public:
  void init() override {
    _sim.reset();
    Serial.println("[PumpBus/Virtual] Simulated pump head + AS5600 active.");
  }
  void setDutyPercent(float percent) override {
    _duty = percent;
    _sim.setDutyPercent(percent);
  }
  float dutyPercent() const override { return _duty; }
  void tick(uint32_t nowMs) { _sim.tick(nowMs); }
  uint16_t angle() const { return _sim.rawAngle(); }

private:
  PumpSim _sim;
  float _duty = 0.0f;
};

class VirtualEncoder : public IRotaryEncoder {
public:
  explicit VirtualEncoder(VirtualActuator& act) : _act(act) {}
  void init() override {}
  void tick(uint32_t nowMs) override { _act.tick(nowMs); }
  uint16_t readRawAngle() override {
    // The real AS5600 is sampled on demand, so the model must advance here too
    // (the node only calls readRawAngle(), never tick()).
    _act.tick(millis());
    return _act.angle();
  }

private:
  VirtualActuator& _act;
};

VirtualActuator g_actuator;
VirtualEncoder g_encoder(g_actuator);

}  // namespace

IPwmActuator& pumpActuator() { return g_actuator; }
IRotaryEncoder& pumpEncoder() { return g_encoder; }

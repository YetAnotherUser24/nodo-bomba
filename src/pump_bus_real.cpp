/**
 * @file pump_bus_real.cpp
 * @brief Real `IPwmActuator` (LEDC MOSFET) + `IRotaryEncoder` (AS5600 over I2C).
 *
 * Extracted from the original `main.cpp`: duty PWM write and the 12-bit AS5600 raw-angle read.
 * No control logic lives here.
 */

#include "pump_bus.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
constexpr uint16_t AS5600_TICKS_PER_REV = 4096;

class LedcPwmActuator : public IPwmActuator {
public:
  void init() override {
    ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ_HZ, PUMP_PWM_RES_BITS);
    ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CHANNEL);
    setDutyPercent(0.0f);
  }

  void setDutyPercent(float percent) override {
    percent = constrain(percent, 0.0f, 100.0f);
    _duty = percent;
    const uint32_t maxDuty = (1u << PUMP_PWM_RES_BITS) - 1u;
    const uint32_t duty = (uint32_t)((percent / 100.0f) * maxDuty);
    ledcWrite(PUMP_PWM_CHANNEL, duty);
  }

  float dutyPercent() const override { return _duty; }

private:
  float _duty = 0.0f;
};

class As5600Encoder : public IRotaryEncoder {
public:
  void init() override {}

  uint16_t readRawAngle() override {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_REG);
    if (Wire.endTransmission(false) != 0) return _last;
    if (Wire.requestFrom(AS5600_ADDR, 2) != 2) return _last;
    const uint16_t hi = Wire.read();
    const uint16_t lo = Wire.read();
    _last = ((hi & 0x0F) << 8) | lo;  // 12-bit
    return _last;
  }

private:
  uint16_t _last = 0;
};

LedcPwmActuator g_actuator;
As5600Encoder g_encoder;

}  // namespace

IPwmActuator& pumpActuator() { return g_actuator; }
IRotaryEncoder& pumpEncoder() { return g_encoder; }

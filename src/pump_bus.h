#pragma once
/**
 * @file pump_bus.h
 * @brief Device-boundary interface for the peristaltic dosing pump (MOSFET PWM + AS5600).
 *
 * The dosing loop talks to the pump only through this seam. The real build binds it to LEDC PWM
 * and the AS5600 I2C encoder; the virtual build binds it to a duty->flow + encoder model.
 * `main.cpp` is identical in both builds.
 */

#include <stdint.h>

class IPwmActuator {
public:
  virtual ~IPwmActuator() = default;
  /** Initialize the PWM stage (idempotent). */
  virtual void init() = 0;
  /** Set duty in percent [0, 100]. */
  virtual void setDutyPercent(float percent) = 0;
  /** Last commanded duty (%). */
  virtual float dutyPercent() const = 0;
};

class IRotaryEncoder {
public:
  virtual ~IRotaryEncoder() = default;
  /** Initialize the encoder bus (idempotent). */
  virtual void init() = 0;
  /** Raw 12-bit angle (0..4095). */
  virtual uint16_t readRawAngle() = 0;
  /** Advance the model (virtual only; no-op on hardware). */
  virtual void tick(uint32_t nowMs) {}
};

/** Compile-time factories. Each build provides exactly one definition of each. */
IPwmActuator& pumpActuator();
IRotaryEncoder& pumpEncoder();

#pragma once
/**
 * @file pump_state.h
 * @brief The pump's observable state, shared between the two cores.
 *
 * This used to live inside `main.cpp`. It moved here because the cloud transport has to
 * read it, and `main.cpp` is compiled as a standalone translation unit (`build_src_filter`
 * lists sources explicitly rather than globbing), so nothing else could see it.
 *
 * The alternative was a second, parallel struct in the transport module. That is exactly
 * the kind of duplication that drifts: a field gets added to one and not the other, and
 * the failure is a silently missing telemetry column rather than a compile error.
 *
 * Only the *shape* lives here. The mutexes and the accessors stay in `main.cpp`, because
 * they are about how the cores share this, not about what it is.
 */

#include <stdint.h>

/** Dosing mode. Serialised to schema v2 as a string, not a number. */
enum PumpMode : uint8_t { PUMP_IDLE = 0, PUMP_DOSE = 1, PUMP_MANUAL_ON = 2 };

struct PumpState {
  float dosed_ml;
  float target_ml;
  float rpm;
  float duty_percent;
  uint8_t mode;
  uint32_t status_code;
  bool dose_completed;
  bool dose_timed_out;
};

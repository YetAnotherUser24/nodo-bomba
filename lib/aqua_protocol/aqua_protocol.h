#pragma once
/**
 * @file aqua_protocol.h
 * @brief Schema v2 telemetry envelope, shared by every node.
 *
 * This is the firmware half of `docs/PROTOCOL.md`. The gateway validates every message
 * against `gateway/aqua_gateway/mqtt/envelope.py`; this file must produce exactly what that
 * parser accepts, and `test_protocol.cpp` proves it by feeding the output to the real parser.
 *
 * ## Why a hand-rolled serializer instead of ArduinoJson
 *
 * The rest of the firmware uses `StaticJsonDocument`. Shared code cannot:
 *
 *   * **Version coupling.** `nodo-sensor-od` vendors ArduinoJson 7, `nodo-aerador` pulls 6.x
 *     via `lib_deps`. `StaticJsonDocument` was *removed* in 7. One header that compiles against
 *     both is not possible without `#if` version soup in every node.
 *   * **A hard byte budget.** `StaticJsonDocument<N>` fails at runtime with `NoMemory` once the
 *     payload outgrows N, and the failure is only observable by checking the return of every
 *     insert. Writing into a caller-owned buffer with an explicit capacity makes overflow a
 *     single checkable flag, and the caller decides what to drop.
 *   * **Host testability.** No Arduino APIs and no third-party headers, so the conformance
 *     test compiles with plain `g++` and can be fed straight into the Python parser.
 *
 * The cost is that string escaping is ours to get right. The only free-form string in an
 * envelope is `exp` (an experiment id), and it is escaped properly; field names and enum
 * values are compile-time constants from the registry and never need escaping.
 *
 * ## Allocation
 *
 * Zero. No `new`, no `malloc`, no `String`. Every buffer is caller-owned. This matters: the
 * node streams 24/7 and repeated malloc/free fragments the ESP32 heap (AGENTS.md §6.1).
 *
 * ## Usage
 *
 * ```cpp
 * char payload[AQUA_ENVELOPE_MAX_BYTES];
 * aqua::EnvelopeBuilder env(aqua::Role::Aerator, bootId, payload, sizeof(payload));
 * env.begin();
 * env.addSample(sample, sampleCount);
 * const size_t len = env.finish(tDevMs, aqua::TimeSource::EspNow, rssi);
 * if (len == 0) { ++dropped; return; }        // overflowed; the counter is the alarm
 * mqtt.publish(aqua::telemetryTopic(aqua::Role::Aerator), payload, len, false);
 * ```
 */

#include <stdint.h>
#include <stddef.h>

namespace aqua {

// ---------------------------------------------------------------------------
// Roles
// ---------------------------------------------------------------------------

/**
 * Node roles. The string values are the MQTT topic segment and the `role` field, and they
 * must match the keys in `gateway/aqua_gateway/schema/roles.yaml` exactly.
 */
enum class Role : uint8_t {
  OdSensor = 0,
  Aerator = 1,
  Mixer = 2,
  Pump = 3,
};

/** @return role name, or "" if the enum value is not a known role. */
const char* roleName(Role role);

/**
 * @return the canonical node id (UUID) for a role, or "" if unknown.
 *
 * Hardcoded because it is identity, not configuration: the gateway's registry uses the same
 * values, and a node that publishes under the wrong id is indistinguishable from a different
 * node. `test_protocol.cpp` checks these against `roles.yaml`.
 */
const char* nodeId(Role role);

// ---------------------------------------------------------------------------
// Time provenance
// ---------------------------------------------------------------------------

/**
 * Where the node's wall clock came from. Ordered best -> worst; the gateway maps this to a
 * trust score and flags exports whose samples came from a low-trust source.
 *
 * A node must never claim a better source than it has. Claiming `Ntp` while actually
 * extrapolating from uptime silently corrupts the experiment record, and the whole point of
 * storing `t_src` is that the export can tell the difference afterwards.
 */
enum class TimeSource : uint8_t {
  Ntp = 0,        ///< synced from the gateway's retained `aqua/time`
  EspNow = 1,     ///< received from the OD logger, which owns the DS3231
  Rtc = 2,        ///< local DS3231
  Monotonic = 3,  ///< extrapolated from uptime: ordered, but not anchored
};

const char* timeSourceName(TimeSource source);

/**
 * Epoch seconds below which a clock reading is not a real time: 2020-01-01T00:00:00Z.
 *
 * A node whose RTC or SNTP sync failed reports 1970, and a clock that has never been set is
 * worse than one that admits it. Everything that reads a clock compares against this first, so
 * the threshold is defined once. The four nodes previously each declared their own copy, and one
 * of them had drifted to `1600000000` -- which the pump's own comment did not describe, saying
 * instead that the pre-2020 case is the one reported as `monotonic`.
 */
constexpr uint64_t kPlausibleEpochS = 1577836800ULL;

/** @return true when `unixMs` is a plausible epoch-millisecond reading. */
bool plausibleEpochMs(long long unixMs);

/**
 * Carry a time broadcast forward by uptime, in epoch milliseconds.
 *
 * Tier 3 of `docs/TIME-SYNC.md`: with an AP and a broker but no internet, the retained
 * `aqua/time` message is the only thing that can say what time it is. Extrapolating from it
 * keeps samples stamped with something ordered and nearly right, which is a far better answer
 * than a node stamping every sample 0.
 *
 * Takes plain integers rather than reading `millis()` itself, so the arithmetic is testable on
 * the host — including the wrap, which is invisible until the node has been up for 49 days.
 *
 * @param anchorMs    epoch ms from the broadcast
 * @param anchorAtMs  uptime at which the broadcast arrived
 * @param nowMs       current uptime
 */
uint64_t extrapolateEpochMs(uint64_t anchorMs, uint32_t anchorAtMs, uint32_t nowMs);

/**
 * Whether a command's deadline has already passed.
 *
 * The gateway stamps `deadline_ms` when it issues a command. Acting after it has expired is not
 * neutral: it moves an actuator for a reason nobody is waiting on any more.
 *
 * Two conditions answer "not expired". A deadline of `<= 0` means there is nothing to enforce,
 * and a clock that is not plausible means this node is in no position to judge. The second is
 * checked with `plausibleEpochMs` rather than against zero, so a caller cannot disable the check
 * by passing an unanchored reading that happens not to be exactly 0.
 *
 * Shared rather than repeated per node: the four copies had already diverged, and the 256-byte
 * JSON probe that silently disabled the check on all four had to be corrected four times.
 *
 * @param deadlineMs the command's deadline, epoch ms
 * @param nowMs      best-known epoch ms; implausible values mean "no anchored clock"
 */
bool deadlinePassed(long long deadlineMs, uint64_t nowMs);

// ---------------------------------------------------------------------------
// Topics
// ---------------------------------------------------------------------------

/**
 * Envelope buffer size.
 *
 * Measured, not guessed: the widest role (the aerator) costs ~270 bytes per sample, so a
 * full batch of `AQUA_ENVELOPE_BATCH_LIMIT` needs ~6.8 KB plus the ~150-byte header.
 * `test_protocol.cpp` asserts that the declared limit actually fits in the declared buffer —
 * an undersized buffer does not fail loudly, it makes the node drop every full batch, which
 * looks exactly like a network problem.
 *
 * Allocate this **statically**, not on the stack. 8 KB will not fit in the default ESP32 task
 * stack, and a stack overflow here reboots the node in the middle of an experiment.
 */
constexpr size_t AQUA_ENVELOPE_MAX_BYTES = 8192;

/**
 * Samples per envelope.
 *
 * The protocol allows 200 (`MAX_BATCH`), but a 200-sample aerator envelope is ~54 KB, which no
 * ESP32 MQTT client will send happily. 25 matches the previous firmware's batch size: 5 s of
 * data at the 5 Hz experiment cadence.
 */
constexpr size_t AQUA_ENVELOPE_BATCH_LIMIT = 25;

/**
 * Write `aqua/tlm/<role>` into `out`.
 * @return false if the buffer is too small (nothing is written).
 */
bool telemetryTopic(Role role, char* out, size_t capacity);

/** Write `aqua/cmd/<role>`. Subscribed to by the node. */
bool commandTopic(Role role, char* out, size_t capacity);

/** Write `aqua/ack/<role>`. */
bool ackTopic(Role role, char* out, size_t capacity);

/**
 * Write `aqua/presence/<node_id>`.
 *
 * Publish RETAINED on connect, and set it as the MQTT Last Will. The broker then tells the
 * gateway the moment a node drops, instead of the gateway inferring it a minute later from a
 * stale timestamp — which is what the previous heartbeat heuristic did.
 */
bool presenceTopic(Role role, char* out, size_t capacity);

/** Write `aqua/event/<role>`. */
bool eventsTopic(Role role, char* out, size_t capacity);

/** Retained time broadcast from the gateway. Payload carries `t_dev_ms` and `t_src`. */
constexpr const char* AQUA_TIME_TOPIC = "aqua/time";

/**
 * Reserved per-sample key carrying that sample's own device timestamp, epoch ms UTC.
 *
 * A batch spans time and one envelope-level ``t_dev_ms`` cannot describe every sample in
 * it. A node sampling at 5 Hz batches five samples a second; with only the envelope
 * timestamp those five rows land on one instant and a plot collapses them onto a single
 * point. The gateway reads this key per sample when present, so it is part of the format
 * rather than a registry field -- which also means it must not be reported as unknown.
 */
constexpr const char* AQUA_SAMPLE_TIME_FIELD = "t_dev_ms";

// ---------------------------------------------------------------------------
// Sample values
// ---------------------------------------------------------------------------

/**
 * One telemetry field in a sample.
 *
 * Tagged rather than variant so that a struct array can be built as a static const table —
 * "what this node publishes" is data, not code, which is what makes it reviewable against
 * `roles.yaml`.
 */
struct FieldValue {
  enum class Kind : uint8_t { Number, Boolean, Text };

  const char* name;
  Kind kind;
  double number;      ///< Kind::Number
  bool boolean;       ///< Kind::Boolean
  const char* str;    ///< Kind::Text (enum members; never needs escaping)
  uint8_t precision;  ///< Kind::Number only, and only for floats

  static FieldValue num(const char* name, double value, uint8_t precision = 3);
  /** Integer fields carry no precision; they are emitted without a decimal point. */
  static FieldValue integer(const char* name, long value);
  /**
   * 64-bit integer, for values a 32-bit ``long`` cannot hold.
   *
   * Exists because of ``t_dev_ms``: epoch milliseconds is ~1.79e12, and ``long`` is 32
   * bits on ESP32, so routing it through ``integer()`` truncates it silently to something
   * around 1.7e9 -- which every consumer would accept as a plausible timestamp. A
   * ``double`` holds integers exactly to 2^53, so nothing is lost on the way out.
   */
  static FieldValue integer64(const char* name, long long value);
  static FieldValue flag(const char* name, bool value);
  static FieldValue text(const char* name, const char* value);
};

// ---------------------------------------------------------------------------
// Envelope builder
// ---------------------------------------------------------------------------

/**
 * Builds one v2 envelope into a caller-owned buffer.
 *
 * Not thread-safe, and not intended to be: the cloud worker owns it on Core 0. The shared
 * telemetry snapshot is copied out under a critical section before it gets here.
 */
class EnvelopeBuilder {
 public:
  /**
   * @param role     publishing role; must match the topic it is sent on
   * @param bootId   random per boot, stable for the session. With `seq` it forms the
   *                 gateway's idempotency key, so a QoS 1 redelivery is deduplicated
   *                 rather than stored twice.
   * @param buffer   caller-owned destination
   * @param capacity bytes available at `buffer`
   */
  EnvelopeBuilder(Role role, uint32_t bootId, char* buffer, size_t capacity);

  /**
   * Experiment id to tag the batch with, or nullptr for none.
   * Must be called before `begin()`: the field is written into the envelope prefix.
   */
  void setExperiment(const char* experimentId);

  /**
   * Write the envelope prefix.
   *
   * @param sequence monotonic counter for this boot, starting at 1. Together with `bootId` it
   *                 forms the gateway's `(node_id, boot_id, seq)` idempotency key, and it is
   *                 also what the gateway's gap detector compares against the previous message.
   *                 It must therefore **increase by one per envelope** and must not restart
   *                 within a boot. A constant here makes every message after the first look like
   *                 a duplicate.
   *
   * The number is claimed at `begin()` rather than at `finish()` so that a build which overflows
   * still consumes one. Burning a number on a dropped message is the correct behaviour: the
   * gateway then reports a sequence *gap*, which is exactly the signal you want when firmware is
   * throwing payloads away.
   *
   * @return false if the buffer cannot hold even an empty envelope.
   */
  bool begin(uint32_t sequence);

  /**
   * Append one sample object.
   *
   * @param values non-null array of `count` fields
   * @return false once the buffer is full. The builder keeps its overflowed state; the
   *         caller should stop adding samples and call `finish()`, which will return 0.
   */
  bool addSample(const FieldValue* values, size_t count);

  /**
   * Close the envelope and record the time.
   *
   * @param tDevMs device clock at capture, Unix epoch milliseconds **UTC**. The gateway
   *               rejects 0 or negative values, so a node with no clock must sync first or
   *               publish with `TimeSource::Monotonic` and a monotonic-derived value.
   * @param source provenance of `tDevMs`
   * @param rssi   dBm, or `AQUA_RSSI_ABSENT` when the node has no radio metric (mains-powered
   *               nodes report none, and a fabricated 0 would look like an impossible signal)
   * @return payload length in bytes, or 0 if the envelope overflowed and must be discarded.
   */
  size_t finish(uint64_t tDevMs, TimeSource source, int rssi = AQUA_RSSI_ABSENT);

  /** Bytes written so far. */
  size_t length() const { return length_; }

  /** True once any write has failed. Read it to distinguish "no samples" from "too many". */
  bool overflowed() const { return overflowed_; }

  /** Samples accepted. */
  size_t sampleCount() const { return sampleCount_; }

  /** The sequence number reserved by `begin()`. */
  uint32_t sequence() const { return sequence_; }

  /** Sentinel for "this node has no RSSI to report". */
  static constexpr int AQUA_RSSI_ABSENT = INT32_MIN;

 private:
  bool raw(const char* text, size_t count);
  bool rawChar(char c);
  bool rawString(const char* text);  ///< with JSON escaping

  Role role_;
  uint32_t bootId_;
  uint32_t sequence_;
  const char* experiment_ = nullptr;
  char* buffer_;
  size_t capacity_;
  size_t length_;
  size_t sampleCount_;
  bool overflowed_;
  bool began_;
};

// ---------------------------------------------------------------------------
// Field names, per role
// ---------------------------------------------------------------------------
// These mirror `roles.yaml` exactly. `tools/check-firmware-schema.py` fails the build if they
// drift, so a rename in the registry cannot silently break firmware.

namespace od_sensor_fields {
constexpr const char* kDoMgL = "od_do_mg_l";
constexpr const char* kDoRaw = "od_do_raw";
constexpr const char* kSatPct = "od_sat_pct";
constexpr const char* kSatRaw = "od_sat_raw";
constexpr const char* kTempC = "od_temp_c";
constexpr const char* kTempRaw = "od_temp_raw";
constexpr const char* kAux1Raw = "od_aux1_raw";
constexpr const char* kAux2Raw = "od_aux2_raw";
constexpr const char* kBatteryV = "pwr_battery_v";
constexpr const char* kRtcTempC = "rtc_temp_c";
constexpr const char* kRtcHealth = "rtc_health";
constexpr const char* kDevStatus = "dev_status";
}  // namespace od_sensor_fields

namespace aerator_fields {
constexpr const char* kRpm = "drv_rpm";
constexpr const char* kRpmTarget = "drv_rpm_target";
constexpr const char* kVbusV = "drv_vbus_v";
constexpr const char* kIbusA = "drv_ibus_a";
constexpr const char* kPowerW = "drv_power_w";
constexpr const char* kTorqueNm = "drv_torque_nm";
constexpr const char* kIqA = "drv_iq_a";
constexpr const char* kRunning = "drv_running";
constexpr const char* kStatus = "drv_status";
constexpr const char* kCtlMode = "ctl_mode";
constexpr const char* kCtlFailsafe = "ctl_failsafe";
constexpr const char* kPeerDoMgL = "peer_do_mg_l";
constexpr const char* kPeerTempC = "peer_temp_c";

/** `ctl_mode` enum members, sent as text so an unknown value is rejected rather than
 *  silently mapped to a mode index that means something else. */
constexpr const char* kModePid = "pid";
constexpr const char* kModeFuzzy = "fuzzy";
constexpr const char* kModeManual = "manual";
}  // namespace aerator_fields

namespace mixer_fields {
constexpr const char* kRpm = "drv_rpm";
constexpr const char* kRpmTarget = "drv_rpm_target";
constexpr const char* kRadS = "drv_rad_s";
constexpr const char* kRadSTarget = "drv_rad_s_target";
constexpr const char* kDutyPct = "drv_duty_pct";
constexpr const char* kRunning = "drv_running";
constexpr const char* kStall = "drv_stall";
constexpr const char* kStatus = "drv_status";
}  // namespace mixer_fields

namespace pump_fields {
constexpr const char* kDosedMl = "pump_dosed_ml";
constexpr const char* kTargetMl = "pump_target_ml";
constexpr const char* kRpm = "pump_rpm";
constexpr const char* kMode = "pump_mode";
constexpr const char* kDoseCompleted = "pump_dose_completed";
constexpr const char* kDoseTimedOut = "pump_dose_timed_out";
constexpr const char* kStatus = "drv_status";

constexpr const char* kModeIdle = "idle";
constexpr const char* kModeDose = "dose";
constexpr const char* kModeManual = "manual";
}  // namespace pump_fields

}  // namespace aqua

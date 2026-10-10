/**
 * @file aqua_link.cpp
 * @brief Schema-v2 MQTT transport for the dosing pump.
 *
 * Structure mirrors the OD logger's transport so the two nodes can be read side by side.
 * The differences are the role, the field set, and the fact that this node's cloud code
 * lives in a FreeRTOS task on core 0 rather than in the main loop — which actually suits
 * this module better, since "call `aquaLinkLoop()` from one task only" is satisfied by
 * construction here instead of by convention.
 */

#include "aqua_link.h"

#include "config.h"  // before the guard: AQUA_TRANSPORT_MQTT is resolved here

#if AQUA_TRANSPORT_MQTT

#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_random.h>
#include <sys/time.h>
#include <ArduinoJson.h>

#include "aqua_protocol.h"

// No shared logging header in this repo; `main.cpp` logs with Serial.printf directly and
// this module follows it rather than introducing a second convention.

/* ------------------------------- Timebase ------------------------------- */

/**
 * The pump has no RTC. Its only clock is SNTP, which is why the legacy code guarded its
 * timestamps with `tv_sec < 1600000000` and sent `millis()` when that failed.
 *
 * Schema v2 makes the ambiguity explicit via `t_src` rather than guessing, so the
 * pre-2020 case is reported as `monotonic` — a value that is honest about being relative
 * — instead of an epoch number that decodes to 1970 and looks like a real instant.
 */
static constexpr uint64_t kPlausibleEpochS = 1600000000ULL;

static uint64_t nowUnixMs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static aqua::TimeSource timeSourceNow() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec >= kPlausibleEpochS ? aqua::TimeSource::Ntp
                                                 : aqua::TimeSource::Monotonic;
}

/* ------------------------------- Statics ------------------------------- */

/** One queued snapshot. A copy, not a reference: core 1 keeps writing while this is held. */
struct Queued {
  PumpState state;
  uint64_t unixMs;
};

static WiFiClient s_net;
static PubSubClient s_mqtt(s_net);

/** Caller-owned envelope buffer. Static, so the node's heap footprint does not move. */
static char s_buffer[aqua::AQUA_ENVELOPE_MAX_BYTES];
static aqua::EnvelopeBuilder s_builder(aqua::Role::Pump, 0, s_buffer, sizeof(s_buffer));

static Queued s_batch[aqua::AQUA_ENVELOPE_BATCH_LIMIT];
static size_t s_queued = 0;

static uint32_t s_sequence = 0;
static uint32_t s_dropped = 0;
static uint64_t s_lastUnixMs = 0;
static uint32_t s_nextAttemptMs = 0;
static bool s_presencePublished = false;
static bool s_configured = false;

/**
 * Experiment the samples and events in flight belong to, or empty for none.
 *
 * A dose is a physical act with a volume attached, so which run it belonged to is the one
 * thing that cannot be reconstructed later. Held as a copy because the caller's buffer may be
 * reused between queueing and flushing.
 */
static char s_experimentId[40] = {0};

/* ------------------------------- Topics -------------------------------- */

static bool presenceTopic(char* out, size_t capacity) {
  return aqua::presenceTopic(aqua::Role::Pump, out, capacity);
}

static void publishPresence(bool online, const char* reason) {
  char topic[64];
  if (!presenceTopic(topic, sizeof(topic))) return;

  char payload[192];
  snprintf(payload, sizeof(payload),
           "{\"v\":2,\"node_id\":\"%s\",\"role\":\"pump\",\"online\":%s,\"reason\":\"%s\","
           "\"boot_id\":%lu}",
           aqua::nodeId(aqua::Role::Pump), online ? "true" : "false", reason,
           (unsigned long)s_builder.sequence());

  // Retained, because presence is state rather than an event: a dashboard that connects
  // later must still learn the node exists.
  s_mqtt.publish(topic, payload, /*retained=*/true);
}

/* ------------------------------ Inbound -------------------------------- */

/**
 * Has this command's deadline already passed?
 *
 * The gateway stamps `deadline_ms` when it issues a command, and nothing on the device ever
 * read it. A command that arrived late -- queued while the session was down, or redelivered --
 * was carried out exactly as if it had just been requested. An operator's intent expires, and
 * acting after it does is not neutral: it moves a pump for a reason nobody is waiting on.
 *
 * Enforced here rather than in the handler, so all four nodes behave the same.
 *
 * Skipped outright when the device clock is not anchored (before 2020). Comparing an epoch
 * deadline against an unanchored clock would reject *every* command, which is a worse failure
 * than the one being fixed.
 *
 * @return true when the command is stale and must not be applied
 */
static bool commandExpired(const char* payload, size_t length) {
  // 768, not 256. The command envelope is ~230 bytes with a nested `target`, and a document
  // too small to hold it does not truncate -- it fails to parse. That failure used to return
  // false here, which silently disabled the whole check while every layer still looked right.
  // A probe that cannot read the deadline must say so rather than pretend there isn't one.
  StaticJsonDocument<768> probe;
  const DeserializationError err = deserializeJson(probe, payload, length);
  if (err) {
    Serial.printf("[AQUA] deadline probe could not parse (%s); applying\n", err.c_str());
    return false;
  }
  if (!probe["deadline_ms"].is<long long>()) return false;

  const long long deadlineMs = probe["deadline_ms"].as<long long>();
  if (deadlineMs <= 0) return false;  // no deadline to enforce

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const long long nowMs = (long long)tv.tv_sec * 1000LL + (long long)(tv.tv_usec / 1000);
  if (nowMs < (long long)kPlausibleEpochS * 1000LL) return false;
  if (nowMs <= deadlineMs) return false;

  const char* cmdId = probe["cmd_id"].is<const char*>() ? probe["cmd_id"].as<const char*>() : "";
  Serial.printf("[AQUA] command %s expired %lld ms ago; rejected\n", cmdId, nowMs - deadlineMs);
  if (cmdId[0] != '\0') aquaLinkAck(cmdId, false, 0.0f);
  return true;
}

static void onMessage(char* topic, uint8_t* payload, unsigned int length) {
  (void)topic;
  // Not NUL-terminated by contract. `pumpHandleMqttCommand` is given the length and must
  // use it; treating this as a C string would read past the end of the buffer.
  if (commandExpired((const char*)payload, (size_t)length)) return;
  pumpHandleMqttCommand((const char*)payload, (size_t)length);
}

/* ------------------------------- Connect ------------------------------- */

static bool connectOnce() {
  char willTopic[64];
  if (!presenceTopic(willTopic, sizeof(willTopic))) return false;

  char willPayload[160];
  snprintf(willPayload, sizeof(willPayload),
           "{\"v\":2,\"node_id\":\"%s\",\"role\":\"pump\",\"online\":false,"
           "\"reason\":\"lwt\"}",
           aqua::nodeId(aqua::Role::Pump));

  const char* user = AQUA_MQTT_USER[0] ? AQUA_MQTT_USER : nullptr;
  const char* pass = AQUA_MQTT_PASSWORD[0] ? AQUA_MQTT_PASSWORD : nullptr;

  // The will is what makes a node that dies mid-dose visible immediately, instead of
  // after the dashboard's staleness timeout. It is the whole reason presence is worth
  // doing over MQTT at all.
  if (!s_mqtt.connect(AQUA_MQTT_CLIENT_ID, user, pass, willTopic, /*willQos=*/1,
                      /*willRetain=*/true, willPayload)) {
    return false;
  }

  char cmdTopic[64];
  if (aqua::commandTopic(aqua::Role::Pump, cmdTopic, sizeof(cmdTopic))) {
    s_mqtt.subscribe(cmdTopic, 1);
  }

  // Cleared so the next `loop()` republishes presence for this new session.
  s_presencePublished = false;
  return true;
}

/* ------------------------- Samples -> envelope -------------------------- */

/** Map a mode enum to the string the schema declares. Unknown values go out as `idle`. */
static const char* modeName(uint8_t mode) {
  switch (mode) {
    case PUMP_DOSE: return aqua::pump_fields::kModeDose;
    case PUMP_MANUAL_ON: return aqua::pump_fields::kModeManual;
    default: return aqua::pump_fields::kModeIdle;
  }
}

/**
 * Build the schema-v2 field list for one snapshot.
 *
 * The legacy `postTelemetry()` sent `is_on`, `speed_percent`, `actual_rpm`, `pwm_us` and a
 * `target_rpm` that actually carried `target_ml` — a misnomer that made the dashboard's
 * speed axis meaningless. Schema v2 is volume-oriented, which is what this device is:
 * `pump_dosed_ml` / `pump_target_ml` in mL, and `pump_rpm` for the encoder. The PWM
 * microseconds are not carried over; they were a transport artefact of the old driver,
 * not a measurement.
 *
 * `mode`, `dose_completed` and `dose_timed_out` were never sent at all. They are added
 * here because the state struct already had them, so the dashboard can finally show a
 * dose in progress versus one that hit the anti-overdose timeout.
 *
 * @return the number of fields written
 */
static size_t buildFields(const Queued& entry, aqua::FieldValue* out) {
  const PumpState& st = entry.state;
  size_t n = 0;
  // The sample's own time, not the envelope's. `postTelemetry` queues and flushes immediately,
  // so the common path carries one sample and this looks redundant -- but a flush that cannot
  // reach the broker *keeps* the batch, so an outage accumulates samples that are published
  // together later. At the 3 s running cadence a full batch spans 75 s, and without this every
  // one of those samples would land on the envelope's single instant: the dose curve would
  // flatten to a handful of points exactly when the data matters most.
  out[n++] = aqua::FieldValue::integer64(aqua::AQUA_SAMPLE_TIME_FIELD,
                                         static_cast<long long>(entry.unixMs));
  out[n++] = aqua::FieldValue::num(aqua::pump_fields::kDosedMl, st.dosed_ml, 2);
  out[n++] = aqua::FieldValue::num(aqua::pump_fields::kTargetMl, st.target_ml, 2);
  out[n++] = aqua::FieldValue::num(aqua::pump_fields::kRpm, st.rpm, 2);
  out[n++] = aqua::FieldValue::text(aqua::pump_fields::kMode, modeName(st.mode));
  out[n++] = aqua::FieldValue::flag(aqua::pump_fields::kDoseCompleted, st.dose_completed);
  out[n++] = aqua::FieldValue::flag(aqua::pump_fields::kDoseTimedOut, st.dose_timed_out);
  out[n++] = aqua::FieldValue::integer(aqua::pump_fields::kStatus, (long)st.status_code);
  return n;
}

static_assert(aqua::AQUA_ENVELOPE_BATCH_LIMIT > 0, "batch limit must allow at least one");
/** 8 fields per sample; sized rather than counted inline so a new field is a compile error. */
static aqua::FieldValue s_fields[8];

/* ------------------------------- Public -------------------------------- */

bool aquaLinkBegin() {
  if (s_configured) return true;

  if (AQUA_MQTT_BUFFER_BYTES < aqua::AQUA_ENVELOPE_MAX_BYTES) {
    // A publish larger than the client buffer is refused by PubSubClient at send time,
    // which would show up as telemetry silently never arriving. Fail at setup instead.
    Serial.printf("[AQUA] buffer %d < envelope max %u\n", (int)AQUA_MQTT_BUFFER_BYTES,
                  (unsigned)aqua::AQUA_ENVELOPE_MAX_BYTES);
    return false;
  }

  s_mqtt.setServer(AQUA_MQTT_HOST, AQUA_MQTT_PORT);
  s_mqtt.setBufferSize(AQUA_MQTT_BUFFER_BYTES);
  s_mqtt.setKeepAlive(AQUA_MQTT_KEEPALIVE_S);
  // Bounded explicitly. The library default is 15 s of busy-waiting for a CONNACK, which is
  // long enough to trip a watchdog when the broker accepts the TCP connection and then says
  // nothing. Recovery from a dead broker is the case this path exists for, so it must not be
  // the case that resets the node.
  s_mqtt.setSocketTimeout(AQUA_MQTT_SOCKET_TIMEOUT_S);
  s_mqtt.setCallback(onMessage);

  // Random boot id, so a sequence number that restarts at 1 after a reboot is not
  // mistaken for a duplicate of the previous boot's sample 1.
  s_builder = aqua::EnvelopeBuilder(aqua::Role::Pump, (uint32_t)esp_random(), s_buffer,
                                    sizeof(s_buffer));

  s_configured = true;
  return true;
}

void aquaLinkLoop() {
  // Lazy init, so WiFi coming up by any route (first connect, the task's reconnect, the
  // OTA reboot) leads here without every path having to remember to call aquaLinkBegin().
  if (!s_configured) {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!aquaLinkBegin()) return;
  }

  if (!s_mqtt.connected()) {
    const uint32_t now = millis();
    // Signed-difference compare so this survives the millis() rollover at ~49 days.
    if (static_cast<int32_t>(now - s_nextAttemptMs) >= 0) {
      if (connectOnce()) {
        s_nextAttemptMs = now;
      } else {
        s_nextAttemptMs = now + AQUA_MQTT_RETRY_MS;
      }
    }
    return;
  }

  s_mqtt.loop();

  if (!s_presencePublished) {
    publishPresence(true, "boot");
    s_presencePublished = true;
  }
}

bool aquaLinkConnected() { return s_configured && s_mqtt.connected(); }

void aquaLinkQueue(const PumpState& state) {
  if (s_queued >= aqua::AQUA_ENVELOPE_BATCH_LIMIT) {
    // Refuse the newest rather than evict the oldest. Evicting would silently discard the
    // start of a window; refusing loses one sample and shows up as a sequence gap.
    return;
  }

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint64_t ms = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);

  Queued& entry = s_batch[s_queued];
  entry.state = state;
  entry.unixMs = ms;
  s_lastUnixMs = ms;
  s_queued++;
}

bool aquaLinkFlush() {
  if (s_queued == 0) return true;
  if (!s_mqtt.connected()) return false;

  ++s_sequence;
  // Null rather than an empty string: the format and the gateway both read null as "no
  // experiment", and an empty string would invent a nameless run to group rows under.
  s_builder.setExperiment(s_experimentId[0] != '\0' ? s_experimentId : nullptr);
  s_builder.begin(s_sequence);

  for (size_t i = 0; i < s_queued; i++) {
    const size_t n = buildFields(s_batch[i], s_fields);
    if (!s_builder.addSample(s_fields, n)) {
      // Overflow should be impossible: the batch limit was sized from a measured
      // per-sample cost. If it happens, dropping the whole batch and counting it is
      // better than publishing a truncated envelope that decodes as a short window.
      ++s_dropped;
      s_queued = 0;
      return false;
    }
  }

  const size_t length = s_builder.finish(s_lastUnixMs, timeSourceNow(), WiFi.RSSI());
  if (length == 0 || s_builder.overflowed()) {
    ++s_dropped;
    s_queued = 0;
    return false;
  }

  char topic[64];
  if (!aqua::telemetryTopic(aqua::Role::Pump, topic, sizeof(topic))) return false;
  if (!s_mqtt.publish(topic, (const uint8_t*)s_buffer, length, /*retained=*/false)) {
    // Retain the batch: the caller's state is only cleared on success, so an outage
    // costs latency rather than data.
    return false;
  }

  s_queued = 0;
  return true;
}

size_t aquaLinkQueued() { return s_queued; }
uint32_t aquaLinkDropped() { return s_dropped; }
bool aquaLinkPresencePublished() { return s_presencePublished; }

void aquaLinkSetExperiment(const char* experimentId) {
  if (experimentId == nullptr || experimentId[0] == '\0') {
    s_experimentId[0] = '\0';
    return;
  }
  // Bounded: the id arrives over the wire, and a truncated id is still a usable grouping key
  // whereas an unbounded copy is a buffer overflow.
  snprintf(s_experimentId, sizeof(s_experimentId), "%s", experimentId);
}

/**
 * The experiment id as a JSON fragment: a quoted string, or the literal ``null``.
 *
 * The id arrives over the wire, so it is not assumed safe. Anything outside the characters an
 * identifier has any business containing is rejected rather than escaped -- silently rewriting
 * an id would produce a label that matches nothing in the registry, which is worse than saying
 * the id was unusable. The event is still published, just without attribution, because losing
 * the record of a dose is not an acceptable price for a malformed label.
 */
static const char* experimentJsonFragment(char* scratch, size_t capacity) {
  if (s_experimentId[0] == '\0') return "null";
  for (const char* p = s_experimentId; *p != '\0'; ++p) {
    const char c = *p;
    const bool safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                      (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == ':';
    if (!safe) return "null";
  }
  const int written = snprintf(scratch, capacity, "\"%s\"", s_experimentId);
  if (written <= 0 || (size_t)written >= capacity) return "null";
  return scratch;
}

void aquaLinkEvent(const char* eventType, float volumeMl, float targetMl, const char* status) {
  if (!s_mqtt.connected() || eventType == nullptr) return;

  char topic[64];
  if (!aqua::eventsTopic(aqua::Role::Pump, topic, sizeof(topic))) return;

  static constexpr size_t kEventMax = 320;
  char payload[kEventMax];

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint64_t ms = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);

  // Matches what the gateway's `_handle_event` reads: `event_type`, `node_id`,
  // `experiment_id`, `t_dev_ms`, `detail`. The volumes go under `detail` rather than at
  // the top level, because that is where the gateway looks for type-specific payload.
  char experiment[48];
  const char* experimentField = experimentJsonFragment(experiment, sizeof(experiment));
  const int written = snprintf(
      payload, sizeof(payload),
      "{\"v\":2,\"event_type\":\"%s\",\"node_id\":\"%s\",\"experiment_id\":%s,"
      "\"t_dev_ms\":%llu,\"detail\":{\"volume_ml\":%.2f,\"target_ml\":%.2f,\"status\":\"%s\"}}",
      eventType, aqua::nodeId(aqua::Role::Pump), experimentField, (unsigned long long)ms,
      (double)volumeMl, (double)targetMl, status ? status : "");

  if (written <= 0 || (size_t)written >= sizeof(payload)) {
    // Truncated events must not be published: a correct event with a mangled status is
    // worse than a missing one, because it reads as authoritative.
    Serial.println("[AQUA] event payload too long; not sent");
    return;
  }

  s_mqtt.publish(topic, (const uint8_t*)payload, (size_t)written, /*retained=*/false);
}

void aquaLinkAck(const char* cmdId, bool applied, float dutyPercent) {
  if (!s_mqtt.connected() || cmdId == nullptr || cmdId[0] == '\0') return;

  char topic[64];
  if (!aqua::ackTopic(aqua::Role::Pump, topic, sizeof(topic))) return;

  char payload[256];

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint64_t ms = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);

  // The gateway maps `result` onto an event type and only understands applied / rejected
  // / failed / expired. Anything else is filed as a protocol error, which is the right
  // outcome for an unknown word rather than silently counting it as success.
  const int written = snprintf(payload, sizeof(payload),
                               "{\"cmd_id\":\"%s\",\"result\":\"%s\",\"node_id\":\"%s\",\"t_dev_ms\":%llu,"
                               "\"duty_percent\":%.1f}",
                               cmdId, applied ? "applied" : "rejected", aqua::nodeId(aqua::Role::Pump),
                               (unsigned long long)ms, (double)dutyPercent);
  if (written <= 0 || (size_t)written >= sizeof(payload)) {
    // A truncated acknowledgement is worse than none: the gateway would settle the outbox
    // row on a mangled result. Refusing to send is the honest failure.
    Serial.println("[AQUA] ack payload too long; not sent");
    return;
  }

  // The length is passed explicitly, and that is not tidiness. PubSubClient overloads on the
  // payload pointer type: `publish(topic, const char*, bool)` reads the third argument as
  // `retained`, while `publish(topic, const uint8_t*, unsigned int)` reads it as the length.
  // Casting to `uint8_t*` therefore selects the *length* overload, and passing `false` gives
  // length 0 -- an empty publish. It compiles, the call reads as correct, and the node simply
  // never acknowledges anything: the gateway drops the empty payload on JSON decode and every
  // command sits at 'published' forever. The presence publish above is unaffected only because
  // it passes a `char*`, which selects the other overload.
  if (!s_mqtt.publish(topic, (const uint8_t*)payload, (size_t)written, /*retained=*/false)) {
    Serial.println("[AQUA] ack publish refused");
  }
}

#endif  // AQUA_TRANSPORT_MQTT

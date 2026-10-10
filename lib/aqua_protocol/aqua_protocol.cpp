/**
 * @file aqua_protocol.cpp
 * @brief Schema v2 envelope serialiser. See aqua_protocol.h for the design rationale.
 */

#include "aqua_protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace aqua {
namespace {

// ---------------------------------------------------------------------------
// Registry constants
// ---------------------------------------------------------------------------
// Kept in one place so `tools/check-firmware-schema.py` has exactly one thing to compare
// against roles.yaml.

struct RoleInfo {
  const char* name;
  const char* node_id;
};

constexpr RoleInfo kRoles[] = {
    {"od_sensor", "a0000000-0000-0000-0000-000000000001"},
    {"aerator", "b0000000-0000-0000-0000-000000000002"},
    {"mixer", "c0000000-0000-0000-0000-000000000003"},
    {"pump", "d0000000-0000-0000-0000-000000000004"},
};
constexpr size_t kRoleCount = sizeof(kRoles) / sizeof(kRoles[0]);

const char* kTimeSourceNames[] = {"ntp", "espnow", "rtc", "monotonic"};

/** Index into kRoles, or kRoleCount when the enum value is unknown. */
size_t roleIndex(Role role) {
  const size_t index = static_cast<size_t>(role);
  return index < kRoleCount ? index : kRoleCount;
}

/**
 * Format a number into `out`.
 *
 * `precision == 0` means an integer field and is emitted with no decimal point — the registry
 * declares `drv_status` and `rtc_health` as ints with no precision, and sending `0.000` for
 * those would coerce back to the same value but reads as though it were measured.
 *
 * Non-finite values become `null`. This matters more than it looks: a NaN reaching the wire as
 * `nan` is invalid JSON, and the gateway rejects the **entire envelope** rather than the one
 * field. One bad sensor read would silently discard up to 25 samples. `null` coerces to the
 * registry's "absent", which is what a NaN actually means.
 */
void formatNumber(char* out, size_t capacity, double value, uint8_t precision) {
  if (!isfinite(value)) {
    snprintf(out, capacity, "null");
    return;
  }
  if (precision == 0) {
    // The width here is load-bearing, and `long` is the trap: on the ESP32 `long` is 32 bits,
    // so casting an epoch-millisecond value (~1.79e12) to it saturates at INT32_MAX. It does
    // not raise or warn -- it emits a plausible-looking number from 1970 that every consumer
    // accepts. That is exactly how this reached the bench: the envelope timestamp was correct
    // and only the per-sample one was wrong, so the samples filed under 1970 while the
    // envelope looked healthy. `long long` is exact, because a double holds integers exactly
    // up to 2^53. The assertion fires on the *device* build, where the narrowing would happen;
    // a 64-bit host cannot see the difference, which is why the message is spelled out.
    using IntOut = long long;
    static_assert(sizeof(IntOut) >= 8, "integer fields must not be narrowed to 32 bits");
    const IntOut rounded = static_cast<IntOut>(value >= 0.0 ? value + 0.5 : value - 0.5);
    snprintf(out, capacity, "%lld", rounded);
    return;
  }
  if (precision > 15) precision = 15;
  snprintf(out, capacity, "%.*f", static_cast<int>(precision), value);
}

}  // namespace

// ---------------------------------------------------------------------------
// Registry lookups
// ---------------------------------------------------------------------------

const char* roleName(Role role) {
  const size_t index = roleIndex(role);
  return index < kRoleCount ? kRoles[index].name : "";
}

const char* nodeId(Role role) {
  const size_t index = roleIndex(role);
  return index < kRoleCount ? kRoles[index].node_id : "";
}

const char* timeSourceName(TimeSource source) {
  const size_t index = static_cast<size_t>(source);
  return index < 4 ? kTimeSourceNames[index] : "monotonic";
}

// ---------------------------------------------------------------------------
// Topics
// ---------------------------------------------------------------------------

namespace {

/** `aqua/<prefix>/<role>` — the only shape that varies between topics. */
bool buildTopic(const char* prefix, Role role, char* out, size_t capacity) {
  if (out == nullptr || capacity == 0) return false;
  const char* name = roleName(role);
  if (name[0] == '\0') {
    out[0] = '\0';
    return false;
  }
  const int written = snprintf(out, capacity, "aqua/%s/%s", prefix, name);
  if (written < 0 || static_cast<size_t>(written) >= capacity) {
    out[0] = '\0';
    return false;
  }
  return true;
}

}  // namespace

bool telemetryTopic(Role role, char* out, size_t capacity) { return buildTopic("tlm", role, out, capacity); }
bool commandTopic(Role role, char* out, size_t capacity) { return buildTopic("cmd", role, out, capacity); }
bool ackTopic(Role role, char* out, size_t capacity) { return buildTopic("ack", role, out, capacity); }
bool eventsTopic(Role role, char* out, size_t capacity) { return buildTopic("event", role, out, capacity); }

bool presenceTopic(Role role, char* out, size_t capacity) {
  if (out == nullptr || capacity == 0) return false;
  const char* id = nodeId(role);
  if (id[0] == '\0') {
    out[0] = '\0';
    return false;
  }
  const int written = snprintf(out, capacity, "aqua/presence/%s", id);
  if (written < 0 || static_cast<size_t>(written) >= capacity) {
    out[0] = '\0';
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// FieldValue
// ---------------------------------------------------------------------------

FieldValue FieldValue::num(const char* name, double value, uint8_t precision) {
  FieldValue v{};
  v.name = name;
  v.kind = Kind::Number;
  v.number = value;
  v.precision = precision;
  return v;
}

FieldValue FieldValue::integer(const char* name, long value) {
  FieldValue v{};
  v.name = name;
  v.kind = Kind::Number;
  v.number = static_cast<double>(value);
  v.precision = 0;
  return v;
}

FieldValue FieldValue::integer64(const char* name, long long value) {
  // Same representation as ``integer``: a Number with precision 0, so it is emitted
  // without a decimal point. The input is 64-bit, which is the whole point -- see the
  // header for why a 32-bit ``long`` is not enough for epoch milliseconds.
  FieldValue v{};
  v.name = name;
  v.kind = Kind::Number;
  v.number = static_cast<double>(value);
  v.precision = 0;
  return v;
}

FieldValue FieldValue::flag(const char* name, bool value) {
  FieldValue v{};
  v.name = name;
  v.kind = Kind::Boolean;
  v.boolean = value;
  v.precision = 0;
  return v;
}

FieldValue FieldValue::text(const char* name, const char* value) {
  FieldValue v{};
  v.name = name;
  v.kind = Kind::Text;
  v.str = value;
  v.precision = 0;
  return v;
}

// ---------------------------------------------------------------------------
// EnvelopeBuilder
// ---------------------------------------------------------------------------

EnvelopeBuilder::EnvelopeBuilder(Role role, uint32_t bootId, char* buffer, size_t capacity)
    : role_(role),
      bootId_(bootId),
      sequence_(0),
      buffer_(buffer),
      // One byte is reserved for a NUL terminator, which `finish()` writes. Keeping the
      // payload printable costs one byte and makes a failed envelope diagnosable from a log
      // dump instead of a hex dump.
      capacity_(capacity > 0 ? capacity - 1 : 0),
      length_(0),
      sampleCount_(0),
      overflowed_(capacity == 0 || buffer == nullptr),
      began_(false) {
  if (buffer_ != nullptr && capacity > 0) buffer_[0] = '\0';
}

bool EnvelopeBuilder::raw(const char* text, size_t count) {
  if (overflowed_) return false;
  if (length_ + count > capacity_) {
    overflowed_ = true;
    return false;
  }
  memcpy(buffer_ + length_, text, count);
  length_ += count;
  buffer_[length_] = '\0';
  return true;
}

bool EnvelopeBuilder::rawChar(char c) { return raw(&c, 1); }

bool EnvelopeBuilder::rawString(const char* text) {
  if (overflowed_) return false;
  if (text == nullptr) text = "";

  // Two passes: measure, then commit. Committing first and discovering the shortfall partway
  // leaves the buffer holding invalid JSON, and an invalid envelope is worse than a dropped
  // one: the gateway rejects the whole batch, discarding the samples that were fine.
  size_t needed = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    const unsigned char c = static_cast<unsigned char>(*p);
    if (c == '"' || c == '\\' || c == '\n' || c == '\r' || c == '\t') {
      needed += 2;
    } else if (c < 0x20) {
      needed += 6;  // \u00XX
    } else {
      needed += 1;
    }
  }
  if (length_ + needed > capacity_) {
    overflowed_ = true;
    return false;
  }

  for (const char* p = text; *p != '\0'; ++p) {
    const unsigned char c = static_cast<unsigned char>(*p);
    switch (c) {
      case '"':  buffer_[length_++] = '\\'; buffer_[length_++] = '"';  break;
      case '\\': buffer_[length_++] = '\\'; buffer_[length_++] = '\\'; break;
      case '\n': buffer_[length_++] = '\\'; buffer_[length_++] = 'n';  break;
      case '\r': buffer_[length_++] = '\\'; buffer_[length_++] = 'r';  break;
      case '\t': buffer_[length_++] = '\\'; buffer_[length_++] = 't';  break;
      default:
        if (c < 0x20) {
          // capacity_ leaves one byte spare for the terminator, so +1 is the true room.
          length_ += static_cast<size_t>(
              snprintf(buffer_ + length_, capacity_ - length_ + 1, "\\u%04x", c));
        } else {
          buffer_[length_++] = static_cast<char>(c);
        }
        break;
    }
  }
  buffer_[length_] = '\0';
  return true;
}

bool EnvelopeBuilder::begin(uint32_t sequence) {
  // Unusable buffer: permanent. No amount of retrying makes a null or zero-length buffer work,
  // so this is a programming error and stays one.
  if (buffer_ == nullptr || capacity_ == 0) return false;

  // A builder is deliberately reusable: `begin()` starts a NEW envelope and clears the previous
  // one's state. `began_` and `overflowed_` are per-envelope, not per-builder.
  //
  // This is not a nicety. Leaving `began_` set meant the second envelope was refused, so a node
  // that streams published exactly one envelope and then went silent for as long as it ran --
  // and `begin()` returning false for correct usage is indistinguishable, from the outside, from
  // a node that never sent anything at all. That is precisely how it failed in the field: the
  // dashboard stayed empty, the broker was healthy, and the single envelope that did go out had
  // already been archived.
  began_ = false;
  overflowed_ = false;
  length_ = 0;
  sampleCount_ = 0;
  sequence_ = sequence;
  buffer_[0] = '\0';

  char temp[96];

  if (!raw("{\"v\":2,\"role\":\"", 15)) return false;
  if (!rawString(roleName(role_))) return false;

  if (!raw("\",\"node_id\":\"", 13)) return false;
  if (!rawString(nodeId(role_))) return false;

  if (!raw("\",\"exp\":", 8)) return false;
  if (experiment_ == nullptr || experiment_[0] == '\0') {
    if (!raw("null", 4)) return false;
  } else if (!raw("\"", 1) || !rawString(experiment_) || !raw("\"", 1)) {
    return false;
  }

  snprintf(temp, sizeof(temp), ",\"seq\":%lu,\"boot_id\":%lu,\"samples\":[",
           static_cast<unsigned long>(sequence_), static_cast<unsigned long>(bootId_));
  if (!raw(temp, strlen(temp))) return false;

  began_ = true;
  return true;
}

bool EnvelopeBuilder::addSample(const FieldValue* values, size_t count) {
  if (!began_ || overflowed_) return false;
  if (values == nullptr || count == 0) {
    // An empty sample is meaningless, and the gateway rejects an empty `samples` array
    // outright. Refusing here keeps the failure local to the node that produced it.
    return false;
  }

  if (!rawChar(sampleCount_ == 0 ? '{' : ',')) return false;
  if (sampleCount_ > 0 && !rawChar('{')) return false;

  char number[48];
  bool firstField = true;
  for (size_t i = 0; i < count; ++i) {
    const FieldValue& field = values[i];
    if (field.name == nullptr || field.name[0] == '\0') continue;

    if (!firstField && !rawChar(',')) return false;
    firstField = false;
    if (!rawChar('"') || !rawString(field.name) || !raw("\":", 2)) return false;

    switch (field.kind) {
      case FieldValue::Kind::Number:
        formatNumber(number, sizeof(number), field.number, field.precision);
        if (!raw(number, strlen(number))) return false;
        break;
      case FieldValue::Kind::Boolean:
        if (!raw(field.boolean ? "true" : "false", field.boolean ? 4 : 5)) return false;
        break;
      case FieldValue::Kind::Text:
        if (!rawChar('"') || !rawString(field.str) || !rawChar('"')) return false;
        break;
    }
  }

  if (!rawChar('}')) return false;
  ++sampleCount_;
  return true;
}

size_t EnvelopeBuilder::finish(uint64_t tDevMs, TimeSource source, int rssi) {
  if (!began_ || overflowed_) return 0;
  if (sampleCount_ == 0) {
    // `samples` must be non-empty or the parser raises `empty_samples`.
    return 0;
  }

  char temp[64];
  if (!raw("],\"t_dev_ms\":", 13)) return 0;

  snprintf(temp, sizeof(temp), "%llu", static_cast<unsigned long long>(tDevMs));
  if (!raw(temp, strlen(temp))) return 0;

  if (!raw(",\"t_src\":\"", 10)) return 0;
  if (!rawString(timeSourceName(source))) return 0;
  if (!rawChar('"')) return 0;

  if (rssi != AQUA_RSSI_ABSENT) {
    snprintf(temp, sizeof(temp), ",\"rssi\":%d", rssi);
    if (!raw(temp, strlen(temp))) return 0;
  }

  if (!rawChar('}')) return 0;

  return length_;
}

void EnvelopeBuilder::setExperiment(const char* experimentId) { experiment_ = experimentId; }

}  // namespace aqua

#pragma once
/**
 * @file aqua_link.h
 * @brief Schema-v2 MQTT transport for the dosing pump.
 *
 * Replaces the HTTPS bulk POST, the 1 Hz command poll and the ACK POST in `main.cpp`'s
 * cloud task with a single MQTT session. The dosing loop, the encoder arithmetic, the
 * timeout guard and the SD behaviour are untouched: this is a transport swap, not a
 * control change.
 *
 * ## Why the legacy path is not deleted
 *
 * `main.cpp` keeps every HTTPS code path, selected by `AQUA_TRANSPORT_MQTT` in
 * `config.h`. Reverting a node in the field is then a rebuild rather than a hunt for
 * older firmware. The one exception is the command *application* logic, which is shared
 * rather than duplicated — see `handleCommandObject()` in `main.cpp`. Two copies of the
 * dosing semantics would be free to disagree about what a `dose` action means, and that
 * disagreement would dose the tank.
 *
 * ## Threading
 *
 * This module is NOT thread-safe and does not pretend to be. Every function must be
 * called from the cloud task on core 0, which is also the only caller. The node's state
 * arrives as a copy taken under `main.cpp`'s critical section, so the transport never
 * reads the shared struct directly.
 */

#include <Arduino.h>

#include "config.h"
#include "pump_state.h"

#if AQUA_TRANSPORT_MQTT

/**
 * @brief Configure the MQTT client and set the Last Will.
 *
 * Does not block on a connection: a pump that cannot reach the broker must still dose on
 * command from its own state machine, and still report over serial.
 *
 * @return true if configuration succeeded. False means a programming error (the buffer is
 *         smaller than the envelope limit), not a network problem.
 */
bool aquaLinkBegin();

/**
 * @brief Maintain the connection and dispatch inbound commands. Call every task iteration.
 *
 * A failed connect schedules the next attempt `AQUA_MQTT_RETRY_MS` later instead of
 * spinning, so a broker that is down cannot starve the task.
 */
void aquaLinkLoop();

/** @return true when the broker connection is up and the command subscription is live. */
bool aquaLinkConnected();

/**
 * @brief Queue one state snapshot for the next envelope.
 *
 * Takes the state by value: the caller has already copied it out of the shared struct, and
 * holding a reference across the publish would be a data race on core 1's writes.
 */
void aquaLinkQueue(const PumpState& state);

/**
 * @brief Record which experiment subsequent telemetry and events belong to.
 *
 * A dose is a physical act with a volume attached, so which run it belonged to is the one
 * thing that cannot be reconstructed afterwards. The schema models "no experiment" as JSON
 * null, so passing null or "" clears attribution rather than inventing a run to group rows
 * under.
 *
 * @param experimentId id from `start_experiment`, or null/"" when no run is active
 */
void aquaLinkSetExperiment(const char* experimentId);

/**
 * @brief Publish everything queued as one envelope.
 *
 * On failure the batch is retained, so a broker outage costs latency rather than data.
 *
 * @return true if the envelope was published.
 */
bool aquaLinkFlush();

/** @return samples currently held for the next envelope. */
size_t aquaLinkQueued();

/** @return envelopes discarded because they could not be built. Non-zero is a firmware bug. */
uint32_t aquaLinkDropped();

/**
 * @brief Publish a dosing event (`dose_started`, `dose_completed`, `fault`).
 *
 * Events bypass the sample queue and go out immediately: they are sparse, and a
 * dose-complete that arrives three seconds late because it was waiting behind a batch of
 * idle heartbeats defeats the point of reporting it.
 *
 * @param eventType one of the gateway's known event types, or it is filed as unknown
 * @param volumeMl  volume actually delivered
 * @param targetMl  volume requested
 * @param status    free-text discriminator, e.g. `ok` or `timeout`
 */
void aquaLinkEvent(const char* eventType, float volumeMl, float targetMl, const char* status);

/**
 * @brief Acknowledge a command on `aqua/ack/pump`.
 *
 * The gateway settles the outbox row on this, which is what stops the command being
 * re-served. A pump that doses but never acks looks identical, from the operator's side,
 * to one that ignored the command.
 *
 * @param cmdId         `cmd_id` from the command payload; ignored if empty
 * @param applied       true if the action was carried out
 * @param dutyPercent   the duty in force when the command was applied, echoed so the
 *                      operator can tell a dose that started from one that was refused
 */
void aquaLinkAck(const char* cmdId, bool applied, float dutyPercent);

/** @return true once presence has been published for the current connection. */
bool aquaLinkPresencePublished();

/**
 * @brief Apply a command that arrived over MQTT. **Implemented by `main.cpp`.**
 *
 * The transport deliberately does not know what a `dose` means. It hands the raw payload
 * to the node, which owns the dosing semantics and reuses the same code path the legacy
 * HTTP poll used. Duplicating that logic in the transport would give the two copies a
 * chance to disagree about what a dose request means, and the disagreement would dose
 * the tank.
 *
 * @param payload  the raw publish payload; **not** NUL-terminated
 * @param length   its length in bytes
 */
void pumpHandleMqttCommand(const char* payload, size_t length);

#endif  // AQUA_TRANSPORT_MQTT

/**
 * @file main.cpp
 * @brief AquaControl V4: Nodo Bomba Dosificadora Peristáltica (ESP32).
 *
 * Planta 1 (Desoxigenación química): inyecta volumétricamente Na2SO3 desde el dashboard.
 *
 * Arquitectura:
 *   - Core 1: lazo de dosificación de 100 Hz (encoder AS5600 -> volumen, PWM del cabezal).
 *   - Core 0: Cloud Worker (WiFi + HTTPS): telemetría bulk, polling de comandos, ACK con
 *             instante exacto de ejecución y bitácora de eventos (pump_events).
 *
 * Protocolo de comandos (compatible con el orquestador):
 *   - command_type='set_speed' + payload.action='dose', payload.target_ml  -> dosifica X mL
 *   - payload.action='force_on'  -> giro continuo (purgado)
 *   - payload.action='force_off' | command_type='stop' | 'emergency_stop' -> detener
 *   - payload.action='set_state', payload.state='IDLE' -> detener
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <sys/time.h>
#include "config.h"
#include "pump_bus.h"
#include "pump_state.h"
#include "aqua_link.h"

// Hardware is reached exclusively through the injected bus. Real builds bind it to LEDC PWM +
// the AS5600; virtual builds bind it to a duty->flow + encoder model. The dosing loop is identical.
static IPwmActuator& pumpActuatorRef = pumpActuator();
static IRotaryEncoder& pumpEncoderRef = pumpEncoder();

#if defined(OTA_ENABLED) && OTA_ENABLED
#include <ArduinoOTA.h>
static volatile bool otaInProgress = false;
#endif

/* ----------------------------- AS5600 ------------------------------ */
#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_TICKS_PER_REV 4096.0f

/* --------------------------- Estado ------------------------------------ */
// `enum PumpMode` and `struct PumpState` now live in pump_state.h: this file is its own
// translation unit (build_src_filter names sources explicitly), so the cloud transport
// had no way to see a type it needs to send.

static PumpState s_state;               // Core 1 -> Core 0
static portMUX_TYPE s_stateMux = portMUX_INITIALIZER_UNLOCKED;
static PumpState s_cmd;                 // Core 0 -> Core 1
static portMUX_TYPE s_cmdMux = portMUX_INITIALIZER_UNLOCKED;

static void setState(const PumpState& s) {
  portENTER_CRITICAL(&s_stateMux);
  s_state = s;
  portEXIT_CRITICAL(&s_stateMux);
}
static PumpState getState() {
  PumpState c;
  portENTER_CRITICAL(&s_stateMux);
  c = s_state;
  portEXIT_CRITICAL(&s_stateMux);
  return c;
}
static void setCommand(const PumpState& c) {
  portENTER_CRITICAL(&s_cmdMux);
  s_cmd = c;
  portEXIT_CRITICAL(&s_cmdMux);
}
static PumpState getCommand() {
  PumpState c;
  portENTER_CRITICAL(&s_cmdMux);
  c = s_cmd;
  portEXIT_CRITICAL(&s_cmdMux);
  return c;
}

/* ------------------------- Encoder AS5600 -------------------------- */
static uint16_t readRawAngle(uint16_t fallback) {
  const uint16_t raw = pumpEncoderRef.readRawAngle();
  (void)fallback;
  return raw;
}

/* ---------------------------- Actuador ----------------------------- */
static void applyDuty(float percent) {
  pumpActuatorRef.setDutyPercent(percent);
}

/* --------------------------- Core 1 lazo --------------------------- */
static void dosingLoop() {
  static uint32_t lastTick = 0;
  static uint16_t lastRaw = 0;
  static bool primed = false;
  static uint32_t doseStartMs = 0;

  uint32_t now = millis();
  if (now - lastTick < 10) return; // 100 Hz
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;

  // 1. Encoder -> revoluciones -> volumen
  uint16_t raw = readRawAngle(lastRaw);
  int32_t delta = (int32_t)raw - (int32_t)lastRaw;
  if (delta > (int32_t)(AS5600_TICKS_PER_REV / 2)) delta -= (int32_t)AS5600_TICKS_PER_REV;
  else if (delta < -(int32_t)(AS5600_TICKS_PER_REV / 2)) delta += (int32_t)AS5600_TICKS_PER_REV;
  lastRaw = raw;
  if (!primed) { primed = true; delta = 0; }

  float revDelta = (float)delta / AS5600_TICKS_PER_REV;
  float rpm = (dt > 0.0f) ? (revDelta / dt) * 60.0f : 0.0f;

  PumpState st = getState();
  PumpState cmd = getCommand();

  // 2. Volumen acumulado (solo cuenta avance positivo)
  if (revDelta > 0.0f) st.dosed_ml += revDelta * ML_PER_REV;
  st.rpm = (st.rpm * 0.85f) + (fabsf(rpm) * 0.15f); // EMA suave
  st.mode = cmd.mode;
  st.target_ml = cmd.target_ml;

  // 3. Máquina de estados del actuador
  float duty = 0.0f;
  if (cmd.mode == PUMP_DOSE) {
    if (doseStartMs == 0) doseStartMs = now;
    bool reached = (st.target_ml > 0.0f) && (st.dosed_ml >= st.target_ml);
    bool timeout = (now - doseStartMs) >= DOSE_TIMEOUT_MS;
    if (reached || timeout) {
      duty = 0.0f;
      st.mode = PUMP_IDLE;
      st.dose_completed = reached;
      st.dose_timed_out = timeout;
      st.status_code = reached ? 0 : 2;
      cmd.mode = PUMP_IDLE;
      setCommand(cmd);
      doseStartMs = 0;
    } else {
      duty = DOSE_DUTY_PERCENT;
    }
  } else if (cmd.mode == PUMP_MANUAL_ON) {
    duty = DOSE_DUTY_PERCENT;
    doseStartMs = 0;
  } else {
    doseStartMs = 0;
  }

  st.duty_percent = duty;
  applyDuty(duty);
  setState(st);
}

/* --------------------------- Core 0 nube --------------------------- */
static uint64_t currentRtcMs() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (tv.tv_sec < 1600000000) return 0; // NTP aún no sincronizado
  return (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
}

static void postPumpEvent(const char* eventType, float volumeMl, float targetMl, const char* status) {
#if AQUA_TRANSPORT_MQTT
  // Events go out immediately rather than through the sample queue: they are sparse, and a
  // dose-complete arriving seconds late behind a batch of idle heartbeats defeats the point.
  aquaLinkEvent(eventType, volumeMl, targetMl, status);
  return;
#endif
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(4000);
  if (!http.begin(client, API_EVENTS_PUMP)) return;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Key", DEVICE_KEY);

  JsonDocument doc;
  doc["experiment_id"] = "idle";
  doc["event_type"] = eventType;
  doc["volume_ml"] = round(volumeMl * 100.0f) / 100.0f;
  doc["target_ml"] = targetMl;
  doc["status"] = status;
  uint64_t rtc = currentRtcMs();
  doc["rtc_timestamp_ms"] = (rtc > 0) ? rtc : (uint64_t)millis();

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  Serial.printf("[Cloud] Pump event '%s' (%.2f/%.2f mL) -> HTTP %d\n", eventType, volumeMl, targetMl, code);
  http.end();
}

static void acknowledge(const String& cmdId, bool success, float speedPercent) {
#if AQUA_TRANSPORT_MQTT
  // `speedPercent` is the duty in force. The name is legacy; only the value is passed on.
  aquaLinkAck(cmdId.c_str(), success, speedPercent);
  return;
#else
  if (cmdId.length() == 0) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(4000);
  String url = String(API_ACK_BASE) + "/" + cmdId + "/acknowledge";
  if (!http.begin(client, url)) return;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Key", DEVICE_KEY);

  JsonDocument doc;
  doc["success"] = success;
  doc["actual_speed_percent"] = speedPercent;
  uint64_t rtc = currentRtcMs();
  if (rtc > 0) doc["rtc_timestamp_ms"] = rtc;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  Serial.printf("[Cloud] ACK %s -> HTTP %d\n", cmdId.c_str(), code);
  http.end();
#endif
}

static void postTelemetry() {
  PumpState st = getState();
#if AQUA_TRANSPORT_MQTT
  aquaLinkQueue(st);
  aquaLinkFlush();
  return;
#else
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(4000);
  if (!http.begin(client, API_TELEMETRY_BULK)) return;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Key", DEVICE_KEY);

  JsonDocument doc;
  doc["experiment_id"] = "backend_resolved";
  JsonArray arr = doc["payload"].to<JsonArray>();
  JsonObject o = arr.add<JsonObject>();
  const float rpm = st.rpm;
  const float maxRpm = 100.0f; // Escala nominal del cabezal
  float speedPercent = constrain((rpm / maxRpm) * 100.0f, 0.0f, 100.0f);
  o["is_on"] = (st.duty_percent > 1.0f);
  o["speed_percent"] = round(speedPercent * 10.0f) / 10.0f;
  o["actual_rpm"] = round(rpm * 10.0f) / 10.0f;
  o["target_rpm"] = st.target_ml;
  o["pwm_us"] = (int)round(st.duty_percent / 100.0f * 400.0f) + 1500;
  o["status_code"] = st.status_code;
  o["dosed_ml"] = round(st.dosed_ml * 100.0f) / 100.0f;
  uint64_t rtc = currentRtcMs();
  if (rtc > 0) o["rtc_timestamp_ms"] = rtc;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  if (code < 200 || code >= 300) {
    Serial.printf("[Cloud] Telemetry POST error %d: %s\n", code, http.errorToString(code).c_str());
  }
  http.end();
#endif
}

/* --------------------------- Commands ------------------------------ */

/**
 * Set when the end of a dose has been reported, cleared when a new dose starts.
 *
 * File scope rather than a local of the cloud task, because `applyCommand` — which clears
 * it — now lives outside that function. Only the core-0 cloud task touches it, so this is
 * not new cross-core state.
 */
static bool doseEndReported = true;

/**
 * Apply one command. **The single copy of the dosing semantics.**
 *
 * Both the legacy HTTP poll and the MQTT subscription funnel through here. The legacy
 * envelope nested its arguments under `command:{id, command_type, payload:{...}}`; the v2
 * message puts them at `cmd_id`, `action` and `target`. Translating at the edges is cheap.
 * Keeping two copies of what a `dose` means is not: a divergence between them would
 * silently dose the tank while both looked correct on their own.
 *
 * @param cmdId  echoed in the ack, so the gateway can settle the outbox row
 * @param action the operational verb: emergency_stop, stop, force_off, dose, force_on,
 *               set_state. An unknown verb is acknowledged as rejected, not ignored.
 * @param target the action's arguments
 */
static void applyCommand(const String& cmdId, const String& action, JsonObjectConst target) {
  PumpState cmd = getCommand();
  bool handled = false;
  if (action == "emergency_stop" || action == "stop" || action == "force_off") {
    cmd.mode = PUMP_IDLE;
    setCommand(cmd);
    applyDuty(0.0f);
    handled = true;
    Serial.println("[Cmd] Pump STOP");
  } else if (action == "dose") {
    float targetMl = 0.0f;
    if (target["target_ml"].is<float>()) targetMl = target["target_ml"].as<float>();
    else if (target["volume_ml"].is<float>()) targetMl = target["volume_ml"].as<float>();
    else if (target["target_volume_ml"].is<float>()) targetMl = target["target_volume_ml"].as<float>();
    if (targetMl > 0.0f) {
      PumpState zero = getState();
      zero.dosed_ml = 0.0f;
      zero.dose_completed = false;
      zero.dose_timed_out = false;
      zero.status_code = 0;
      setState(zero);
      cmd.mode = PUMP_DOSE;
      cmd.target_ml = targetMl;
      setCommand(cmd);
      doseEndReported = false;
      handled = true;
      Serial.printf("[Cmd] Dosing %.2f mL\n", targetMl);
      postPumpEvent("dose_started", 0.0f, targetMl, "ok");
    }
  } else if (action == "force_on") {
    cmd.mode = PUMP_MANUAL_ON;
    setCommand(cmd);
    handled = true;
    Serial.println("[Cmd] Pump FORCE ON");
  } else if (action == "stop_experiment") {
    // The pump keeps no experiment state of its own, so "end the experiment" can only mean
    // "stop dosing" -- but it has to be *accepted*. Rejecting it would make an end-of-run
    // transition look like it failed on this role and nowhere else, which reads as a broken
    // node rather than a pump with nothing to stop. The attribution this node does keep is
    // cleared by the adapter, so a rejection would also report the opposite of what happened.
    cmd.mode = PUMP_IDLE;
    setCommand(cmd);
    applyDuty(0.0f);
    handled = true;
    Serial.println("[Cmd] Pump STOP (experiment end)");
  } else if (action == "set_state") {
    String s = target["state"].is<const char*>() ? target["state"].as<String>() : String("");
    // MANUAL_OVERRIDE belongs with IDLE, not apart from it: in both the pump stops dosing
    // under its own control and the operator drives it directly. Rejecting it would mean the
    // orchestrator's transition to manual mode failed on this role and nowhere else, which
    // reads as a broken node rather than a missing branch.
    if (s == "IDLE" || s == "ACTIVE_EXPERIMENT" || s == "MANUAL_OVERRIDE") {
      cmd.mode = PUMP_IDLE;
      setCommand(cmd);
      applyDuty(0.0f);
      handled = true;
    }
  }

  float speed = getState().duty_percent;
  acknowledge(cmdId, handled, speed);
}


#if AQUA_TRANSPORT_MQTT
/**
 * Translate a schema-v2 command and hand it to the shared applier.
 *
 * The v2 message carries `cmd_id`, `action` and `target`. The legacy envelope nested the
 * same information under `command:{id, command_type, payload:{...}}`. Only the shape
 * differs, so this reshapes rather than re-deciding anything — `applyCommand` remains the
 * one place that knows what a dose is.
 *
 * A command with no action is dropped rather than acknowledged: there is nothing to
 * carry out, and acking it would tell the gateway the node acted on an empty verb.
 */
void pumpHandleMqttCommand(const char* payload, size_t length) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, length) != DeserializationError::Ok) {
    Serial.println("[AQUA] command payload was not valid JSON; ignored");
    return;
  }

  String cmdId = doc["cmd_id"].is<const char*>() ? doc["cmd_id"].as<String>() : String("");
  String action = doc["action"].is<const char*>() ? doc["action"].as<String>() : String("");
  if (action.length() == 0) {
    Serial.println("[AQUA] command had no action; ignored");
    return;
  }

  JsonObjectConst target =
      doc["target"].is<JsonObjectConst>() ? doc["target"].as<JsonObjectConst>() : JsonObjectConst();

  const char* exp = doc["exp"].is<const char*>() ? doc["exp"].as<const char*>() : nullptr;

  // Attribution follows the command, because the pump has no run state of its own: the v2
  // message carries the experiment at the envelope level and nothing else on this node has
  // ever known about it. Without this every dose is recorded against no experiment, and a
  // dose is a physical act with a volume attached -- the one thing that cannot be
  // reconstructed after the fact.
  //
  // Clearing is explicit and only on the actions that end a run. A `dose` arriving without an
  // id mid-run must *keep* the current one: clearing on every unnamed command would drop the
  // attribution of exactly the doses that matter, since a UI sending a dose has no reason to
  // repeat the experiment id it already started.
  const String stateArg = target["state"].is<const char*>() ? target["state"].as<String>() : String("");
  const bool endsRun = action == "stop_experiment" ||
                       (action == "set_state" && stateArg != "ACTIVE_EXPERIMENT" &&
                        stateArg != "");
  if (endsRun) {
    aquaLinkSetExperiment(nullptr);
  } else if (exp != nullptr && exp[0] != '\0') {
    aquaLinkSetExperiment(exp);
  }

  applyCommand(cmdId, action, target);
}
#endif

static void cloudWorkerTask(void* param) {
  (void)param;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long lastTelemetry = 0;
  unsigned long lastPoll = 0;
  unsigned long lastWifiRetry = millis();
  bool wasConnected = false;
  bool otaConfigured = false;

  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      wasConnected = false;
      if (millis() - lastWifiRetry > 30000) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(false, false);
        vTaskDelay(pdMS_TO_TICKS(100));
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWifiRetry = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (!wasConnected) {
      Serial.printf("[Cloud] WiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");
      wasConnected = true;
    }

#if defined(OTA_ENABLED) && OTA_ENABLED
    if (!otaConfigured) {
      ArduinoOTA.setHostname(OTA_HOSTNAME);
      if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
      ArduinoOTA.onStart([]() {
        otaInProgress = true;
        PumpState cmd = getCommand();
        cmd.mode = PUMP_IDLE;
        setCommand(cmd);
        applyDuty(0.0f);
        Serial.println("[OTA] Update starting; pump halted.");
      });
      ArduinoOTA.onEnd([]() { otaInProgress = false; });
      ArduinoOTA.onError([](ota_error_t) { otaInProgress = false; });
      ArduinoOTA.begin();
      otaConfigured = true;
    }
    ArduinoOTA.handle();
    if (otaInProgress) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
#endif

    unsigned long now = millis();

#if AQUA_TRANSPORT_MQTT
    // Serviced here rather than at the top of the loop: this task `continue`s before this
    // point while WiFi is down, and there is no point driving an MQTT client without a
    // network. This is also the only task that touches the transport, which is what the
    // module's thread-safety contract requires.
    aquaLinkLoop();
#endif

    // 1. Detección de fin de dosis -> evento pump_completed / fault
    PumpState st = getState();
    if (st.mode == PUMP_IDLE && (st.dose_completed || st.dose_timed_out) && !doseEndReported) {
      postPumpEvent(st.dose_timed_out ? "fault" : "dose_completed", st.dosed_ml, st.target_ml,
                    st.dose_timed_out ? "timeout" : "ok");
      doseEndReported = true;
      PumpState reset = getState();
      reset.dose_completed = false;
      reset.dose_timed_out = false;
      setState(reset);
    }

    // 2. Telemetría
    unsigned long interval = (st.duty_percent > 1.0f) ? TELEMETRY_INTERVAL_MS : TELEMETRY_IDLE_INTERVAL_MS;
    if (now - lastTelemetry >= interval) {
      lastTelemetry = now;
      postTelemetry();
    }

#if !AQUA_TRANSPORT_MQTT
    // 3. Polling de comandos
    // Once MQTT is on, commands are pushed and this 1 Hz GET is pure cost — on the aerator
    // the same poll was ~77% of the node's traffic and almost every response was
    // `has_node: false`.
    if (now - lastPoll >= COMMAND_POLL_INTERVAL_MS) {
      lastPoll = now;
      WiFiClientSecure client;
      client.setInsecure();
      HTTPClient http;
      http.setTimeout(5000);
      if (http.begin(client, API_COMMANDS "?device_key=" DEVICE_KEY)) {
        http.addHeader("X-Device-Key", DEVICE_KEY);
        int code = http.GET();
        if (code == HTTP_CODE_OK) {
          String payload = http.getString();
          JsonDocument doc;
          if (!deserializeJson(doc, payload) && doc["has_command"].as<bool>() && doc["command"].is<JsonObject>()) {
            JsonObject cmdObj = doc["command"].as<JsonObject>();
            String cmdId = cmdObj["id"].is<const char*>() ? cmdObj["id"].as<String>() : String("");
            String cmdType = cmdObj["command_type"].is<const char*>() ? cmdObj["command_type"].as<String>() : String("");
            String action = cmdType;
            if (cmdObj["payload"].is<JsonObject>() && cmdObj["payload"]["action"].is<const char*>()) {
              action = cmdObj["payload"]["action"].as<String>();
            }

            applyCommand(cmdId, action, cmdObj["payload"].is<JsonObjectConst>()
                              ? cmdObj["payload"].as<JsonObjectConst>() : JsonObjectConst());
          }
        } else if (code > 0) {
          Serial.printf("[Cloud] Poll HTTP %d\n", code);
        }
        http.end();
      }
    }
#endif  // !AQUA_TRANSPORT_MQTT

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* ------------------------------ Setup ------------------------------ */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== AquaControl: Bomba Dosificadora Peristáltica (Planta 1) ===");

  // Task watchdog: 60 s, trigger_panic, matching the sensor node's deliberate choice.
  //
  // Without this the Arduino default applies -- 5 s, with the idle tasks on the watch list --
  // and `PubSubClient::connect` busy-waits for a CONNACK using a 15 s socket timeout. A
  // broker that accepts the TCP connection and then answers nothing therefore starves core
  // 0's idle task (the cloud task is pinned there) and the node resets. Measured on the
  // bench: `rst:0xc (RTC_SW_CPU_RST)` twice, ten seconds apart, across a 50 s outage, with
  // the sequence counter restarting each time.
  //
  // That is not a cosmetic reboot. Store-and-forward keeps the pending batch in RAM, so a
  // reset discards it -- and an outage is exactly when the batch is worth keeping. 60 s is
  // comfortably above the 15 s library timeout and still recovers a genuine hang.
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 60000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(60, true);
  esp_task_wdt_add(NULL);
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pumpEncoderRef.init();

  pumpActuatorRef.init();
  applyDuty(0.0f);

  uint16_t probe = readRawAngle(0);
  Serial.printf("[Init] AS5600 raw angle probe: %u (0 constante => revisar I2C)\n", probe);

  PumpState init = {};
  init.mode = PUMP_IDLE;
  init.target_ml = 0.0f;
  setState(init);
  setCommand(init);

  xTaskCreatePinnedToCore(cloudWorkerTask, "CloudWorker", 12288, NULL, 1, NULL, 0);

  Serial.println("[Init] System ready. Dosing loop active on Core 1.\n");
}

/* ------------------------------ Loop ------------------------------- */
void loop() {
  esp_task_wdt_reset();
  dosingLoop();
  delay(2);
}

/**
 * @file config.h
 * @brief Configuración del nodo Bomba Dosificadora Peristáltica (AquaControl V4).
 */

#pragma once

/* ------------------------------- WiFi ------------------------------- */
// Guarded so a credential can be injected at build time; see the note in
// nodo-aerador/src/config.h. An unguarded #define here silently wins over the flag.
#ifndef AQUA_WIFI_SSID
#define AQUA_WIFI_SSID          "WifiDev"
#endif
#ifndef AQUA_WIFI_PASSWORD
#define AQUA_WIFI_PASSWORD      "rpidev24"
#endif
#define WIFI_SSID               AQUA_WIFI_SSID
#define WIFI_PASSWORD           AQUA_WIFI_PASSWORD
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* --------------------------- AquaControl ---------------------------- */
#define API_BASE_URL            "https://tesisutec.vercel.app"
#define API_TELEMETRY_BULK      API_BASE_URL "/api/telemetry/motor_bulk"
#define API_COMMANDS            API_BASE_URL "/api/commands/pending"
#define API_ACK_BASE            API_BASE_URL "/api/commands"
#define API_EVENTS_PUMP         API_BASE_URL "/api/events/pump"
#define DEVICE_KEY              "ESP32_PUMP"
#define TELEMETRY_INTERVAL_MS   3000
#define TELEMETRY_IDLE_INTERVAL_MS 20000
#define COMMAND_POLL_INTERVAL_MS 1000

/* ------------------- AquaControl MQTT (schema v2) ------------------ */
/* Transport selection. Setting this to 0 restores the legacy HTTPS paths below: the
   client setup, the bulk POST, the command poll and the ACK all still compile. This is
   the revert switch, so it must stay a compile-time constant and never become a
   runtime branch. */
#ifndef AQUA_TRANSPORT_MQTT
#define AQUA_TRANSPORT_MQTT       1
#endif
/* Every one of these is overridable with -D at build time, and that is not decoration: a
   bench node has to point at a developer's broker instead of the station's, and the
   alternative is editing this file — a tracked file — before every bench session and
   remembering to revert it afterwards.

   Guarding only `AQUA_TRANSPORT_MQTT` does not achieve that. Skipping that block does not
   skip the rest: the settings below still define themselves, so `-DAQUA_MQTT_HOST=...`
   either loses to the value here or emits a redefinition warning and then loses anyway.
   Each setting needs its own guard. */
/* The Pi's Tailscale name. mDNS would be nicer but the ESP32 resolver for
   `pump-doser.local` is already used for OTA and resolving two names costs more
   than it saves. */
#ifndef AQUA_MQTT_HOST
#define AQUA_MQTT_HOST            "aqua-station.local"
#endif
#ifndef AQUA_MQTT_PORT
#define AQUA_MQTT_PORT            1883
#endif
/* Anonymous is correct on the isolated bench network; the broker is bound to the
   Tailscale interface. If it is ever exposed further, these must become real
   credentials AND the TLS port, not just a password over 1883. */
#ifndef AQUA_MQTT_USER
#define AQUA_MQTT_USER            ""
#endif
#ifndef AQUA_MQTT_PASSWORD
#define AQUA_MQTT_PASSWORD        ""
#endif
#ifndef AQUA_MQTT_KEEPALIVE_S
#define AQUA_MQTT_KEEPALIVE_S     30
#endif
/* Bound for how long the library waits for a broker response. PubSubClient defaults to 15 s
   and busy-waits while it does, so a broker that accepts the connection but never answers
   stalls the reconnect for that long and starves the watchdog. Kept well under the 60 s WDT. */
#ifndef AQUA_MQTT_SOCKET_TIMEOUT_S
#define AQUA_MQTT_SOCKET_TIMEOUT_S 5
#endif
/* Unique per node: two clients sharing a client-id fight over one session and the
   broker disconnects them in a loop. The old 60 s heartbeat heuristic could not tell
   this apart from a healthy node. */
#ifndef AQUA_MQTT_CLIENT_ID
#define AQUA_MQTT_CLIENT_ID       "aqua-pump"
#endif
/* Schema v2 header + one 8-field pump sample is well under 1 KB, but the envelope
   limit is 8 KB and PubSubClient refuses a publish larger than its buffer. Sized to
   match rather than to the current payload, so a batch of samples still fits. */
#ifndef AQUA_MQTT_BUFFER_BYTES
#define AQUA_MQTT_BUFFER_BYTES    8192
#endif
#ifndef AQUA_MQTT_RETRY_MS
#define AQUA_MQTT_RETRY_MS        5000UL
#endif

/* --------------------------- Hardware ------------------------------- */
// Pin PWM del driver MOSFET de la bomba peristáltica de 12 V (verificar en el PCB).
#ifndef PUMP_PWM_PIN
#define PUMP_PWM_PIN            25
#endif
#ifndef PUMP_PWM_CHANNEL
#define PUMP_PWM_CHANNEL        0
#endif
#ifndef PUMP_PWM_FREQ_HZ
#define PUMP_PWM_FREQ_HZ        20000   // 20 kHz: fuera del rango audible
#endif
#ifndef PUMP_PWM_RES_BITS
#define PUMP_PWM_RES_BITS       10      // 0..1023
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN             21
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN             22
#endif

/* ------------------- Calibración de dosificación -------------------- */
// Volumen entregado por revolución del cabezal peristáltico (mL/rev). Calibrar en banco.
#ifndef ML_PER_REV
#define ML_PER_REV              1.00f
#endif
// Duty PWM durante la dosificación (%). El cabezal gira a velocidad constante.
#ifndef DOSE_DUTY_PERCENT
#define DOSE_DUTY_PERCENT       80.0f
#endif
// Tiempo máximo de seguridad por dosis (ms) para evitar sobredosificación por fallo de encoder.
#ifndef DOSE_TIMEOUT_MS
#define DOSE_TIMEOUT_MS         120000UL
#endif

/* ------------------------------- OTA -------------------------------- */
#ifndef OTA_ENABLED
#define OTA_ENABLED             1
#endif
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME            "pump-doser"
#endif
#ifndef OTA_PASSWORD
#define OTA_PASSWORD            ""
#endif

/**
 * @file config.h
 * @brief Configuración del nodo Bomba Dosificadora Peristáltica (AquaControl V4).
 */

#pragma once

/* ------------------------------- WiFi ------------------------------- */
#define WIFI_SSID               "WifiDev"
#define WIFI_PASSWORD           "rpidev24"
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

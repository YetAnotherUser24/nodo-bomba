# Nodo bomba — Dosificación peristáltica (`nodo-bomba`)

> **Repositorio:** [github.com/YetAnotherUser24/nodo-bomba](https://github.com/YetAnotherUser24/nodo-bomba)

Nodo de dosificación volumétrica de la Planta 1. Inyecta la disolución madre de **sulfito de sodio**
($Na_2SO_3$, 100 g/L) que provoca la caída controlada del oxígeno disuelto, e informa el volumen
**realmente** inyectado para que el dashboard pueda graficar la perturbación en el instante correcto.

- **Rol en el sistema:** `pump` (ver `Tesis_webpage/src/lib/deviceRoles.ts`).
- **Placa:** ESP32 Dev Module (ESP32-WROOM-32).
- **Identidad:** resuelta por `device_id` canónico, nunca por `devices.type` (`motor_thruster` es legacy).

---

## 1. Hardware

| Elemento | Detalle |
| :--- | :--- |
| Bomba | Peristáltica 12 V accionada por PWM |
| Sensor de giro | Encoder magnético **AS5600** por I2C |
| Alimentación | Bus de 12 V del banco (ver AGENTS.md regla 12) |
| Relevante | El volumen se calcula por **revoluciones** (mL/rev calibrado), no por tiempo |

Las patillas, la calibración `mL/rev`, el timeout anti-sobredosificación y los endpoints viven en
[`src/config.h`](src/config.h).

---

## 2. Comportamiento

1. La nube encola una orden de dosis (`start_dose` con `target_ml`).
2. El lazo integra las revoluciones del encoder y acumula el volumen dosificado.
3. Al alcanzar el objetivo corta y publica un evento en `/api/events/pump` con el volumen real, el
   objetivo y el estado (`completed` / `timeout`).
4. Un **timeout** corta la dosis si no se alcanza el objetivo: sin él la bomba seguiría girando y
   sobredosificaría el tanque.

La telemetría se envía como lote a `/api/telemetry/motor_bulk` (con `device_id` de rol `pump`) y se
archiva en la tabla `pump_telemetry`.

---

## 3. Compilar y flashear

`default_envs = esp32dev`, así que `pio run` ya construye la placa correcta.

```bash
cd Codigos/nodo-bomba

pio run                       # construye el entorno por defecto (esp32dev)
pio run -t upload             # flashea por USB
pio device monitor -b 115200  # monitor serie

pio run -e esp32dev-ota -t upload   # actualización inalámbrica (pump-doser.local)
```

> **OTA (invariante 13).** Los nodos van en gabinetes sellados, así que el firmware debe poder
> actualizarse sin USB. El OTA se compila con `OTA_ENABLED=1` (por defecto en `src/config.h`) y el
> nombre mDNS es `pump-doser`. `default.csv` ya trae dos ranuras de aplicación, que es lo que el OTA
> necesita.

---

## 4. Comandos en la nube

| Acción | Efecto |
| :--- | :--- |
| `start_dose` (`target_ml`) | Inicia la dosificación hasta el volumen pedido |
| `stop` | Corta el PWM y detiene la bomba |
| `set_state` (`IDLE` / `MANUAL_OVERRIDE`) | Detiene la bomba: una anulación manual aborta cualquier receta |

En `MANUAL_OVERRIDE` las acciones de receta se rechazan con HTTP 409 (ver AGENTS.md regla 18);
la dosificación directa del operador sigue permitida.

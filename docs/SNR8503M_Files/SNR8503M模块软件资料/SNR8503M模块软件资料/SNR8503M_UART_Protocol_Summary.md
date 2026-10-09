# Documentación Técnica del Protocolo UART - Módulo SNR8503M (BLDC Sensorless)

Este documento contiene la especificación completa, verificada e inspeccionada directamente desde los archivos fuente del SDK del SNR8503M (`UR_Ctrl.c`, `UR_Ctrl.h`, `hardware_config.h`, `MC_Parameter.h` y `Global_Variable.h`).

---

## 1. Parámetros del Puerto Serie (Hardware UART)

* **Canal UART**: UART0 (`P1.6` = TXD0, `P1.7` = RXD0)
* **Baud Rate (Velocidad)**: `9600 bps` (Configurable en `MC_Parameter.h` mediante `#define UART0_BaudRate 9600`)
* **Bits de Datos**: 8 bits (`UART_WORDLENGTH_8b`)
* **Bits de Parada**: 1 bit (`UART_STOPBITS_1b`)
* **Paridad**: Ninguna (`None`)
* **Modo de Operación**: Interrupción por recepción (FIFO UART0 a buffer en anillo `rxd_comm0`)

---

## 2. Trama de Comandos (Host -> SNR8503M Módulo) - RX Frame

La longitud fija de la trama de comando recibida por el módulo es de **9 Bytes**.

### Estructura de la Trama (9 Bytes)

| Índice Byte | Campo | Tipo | Descripción / Unidad |
| :---: | :--- | :---: | :--- |
| **Byte 0** | **Cabecera (Header)** | `uint8_t` | Byte fijo de inicio: `0xAA` |
| **Byte 1** | **Velocidad (MSB)** | `uint8_t` | Byte alto del comando de frecuencia eléctrica (`SpeedCMD >> 8`) en **Hz** |
| **Byte 2** | **Velocidad (LSB)** | `uint8_t` | Byte bajo del comando de frecuencia eléctrica (`SpeedCMD & 0xFF`) en **Hz** |
| **Byte 3** | **Potencia Límite (MSB)** | `uint8_t` | Byte alto de la potencia máxima permitida (`PowerCMD >> 8`) en **mW** |
| **Byte 4** | **Potencia Límite (LSB)** | `uint8_t` | Byte bajo de la potencia máxima permitida (`PowerCMD & 0xFF`) en **mW** |
| **Byte 5** | **Reservado** | `uint8_t` | Reservado / Control (`0x00`) |
| **Byte 6** | **Reservado** | `uint8_t` | Reservado (`0x00`) |
| **Byte 7** | **Checksum (CKM)** | `uint8_t` | Suma de los Bytes 0 a 6: `(Byte0 + Byte1 + Byte2 + Byte3 + Byte4 + Byte5 + Byte6) & 0xFF` |
| **Byte 8** | **Fin de Trama (Tail)** | `uint8_t` | Byte fijo de cierre: `0x55` |

### Comandos de Control

1. **Comando de Marcha y Ajuste de Velocidad**:
   - Enviar `SpeedCMD > 0` (en Hz).
   - Establece la referencia de velocidad y el límite de potencia (`PowerCMD` en mW).
   - **Nota**: Si hay alguna falla presente en `sys_error_flg`, enviar un comando de velocidad válido limpia automáticamente la falla.
2. **Comando de Parada (Stop / Off)**:
   - Enviar `SpeedCMD = 0x0000`.

---

## 3. Trama de Telemetría y Respuesta (SNR8503M -> Host) - TX Frame

Al recibir y validar correctamente una trama de comando (encabezado `0xAA`, fin `0x55` y suma de comprobación correcta), el controlador activa `UartResponceFlag = 1` y responde con una trama de telemetría de **16 Bytes**.

### Estructura de la Trama (16 Bytes)

| Índice Byte | Campo | Tipo | Descripción / Unidad |
| :---: | :--- | :---: | :--- |
| **Byte 0** | **Cabecera (Header)** | `uint8_t` | Byte fijo de inicio: `0xAA` |
| **Byte 1** | **Estado del Motor** | `uint8_t` | Estado de la máquina de estados FOC (`eSysState` / `e1M1_MainState`) |
| **Byte 2** | **Velocidad Actual (MSB)** | `uint8_t` | Byte alto de la frecuencia eléctrica actual (`SpeedValue >> 8`) en **Hz** |
| **Byte 3** | **Velocidad Actual (LSB)** | `uint8_t` | Byte bajo de la frecuencia eléctrica actual (`SpeedValue & 0xFF`) en **Hz** |
| **Byte 4** | **Potencia Actual (MSB)** | `uint8_t` | Byte alto de la potencia instantánea (`wPowerValue >> 8`) en **mW** |
| **Byte 5** | **Potencia Actual (LSB)** | `uint8_t` | Byte bajo de la potencia instantánea (`wPowerValue & 0xFF`) en **mW** |
| **Byte 6** | **Reservado** | `uint8_t` | `0x00` |
| **Byte 7** | **Reservado** | `uint8_t` | `0x00` |
| **Byte 8** | **Código de Falla (MSB)**| `uint8_t` | Byte alto del registro de fallas (`stru_Faults.R >> 8`) |
| **Byte 9** | **Código de Falla (LSB)**| `uint8_t` | Byte bajo del registro de fallas (`stru_Faults.R & 0xFF`) |
| **Byte 10** | **Voltaje Bus (MSB)** | `uint8_t` | Byte alto del voltaje de bus DC (`(V_bus * 256) >> 8`) en **V** |
| **Byte 11** | **Voltaje Bus (LSB)** | `uint8_t` | Byte bajo del voltaje de bus DC (`(V_bus * 256) & 0xFF`) en **V** |
| **Byte 12** | **Corriente Bus (MSB)** | `uint8_t` | Byte alto de la corriente de bus DC (`(I_bus * 256) >> 8`) en **A** |
| **Byte 13** | **Corriente Bus (LSB)** | `uint8_t` | Byte bajo de la corriente de bus DC (`(I_bus * 256) & 0xFF`) en **A** |
| **Byte 14** | **Checksum (CKM)** | `uint8_t` | Suma de los Bytes 0 a 13: `(Byte0 + ... + Byte13) & 0xFF` |
| **Byte 15** | **Fin de Trama (Tail)** | `uint8_t` | Byte fijo de cierre: `0x55` |

---

## 4. Máscara de Bits de Fallas (Registro `stru_Faults` / `sys_error_flg`)

Definidos en `Global_Variable.h` y `M1_StateMachine.h`:

| Bit Hex | Nombre de la Falla | Significado |
| :---: | :--- | :--- |
| `0x0001` | `SHORT_ERROR` | Cortocircuito detectado |
| `0x0002` | `LOW_VOL_ERROR` | Voltaje de bus insuficiente (Subvoltaje) |
| `0x0004` | `HIG_VOL_ERROR` | Sobrevoltaje en el bus DC |
| `0x0008` | `BLOCK_ERROR` | Bloqueo / Rotor trabado (Stall) |
| `0x0010` | `DC_OFFSET_ERROR` | Error en offset de corriente continua |
| `0x0020` | `MOS_OVER_ERROR` | Sobretemperatura en los MOSFETs |
| `0x0040` | `MOS_LOW_ERROR` | Bajo nivel de temperatura en MOSFETs |
| `0x0080` | `BAT_OVER_ERROR` | Sobretemperatura en batería |
| `0x0100` | `BAT_LOW_ERROR` | Bajo nivel de temperatura en batería |
| `0x0200` | `OVER_LOAD_ERROR` | Sobrecarga / Sobrecorriente |
| `0x0400` | `PHASE_DROP_ERROR` | Pérdida o desconexión de fase |
| `0x0800` | `MOSFET_ERROR` | Falla en autochequeo de MOSFETs |

---

## 5. Ejemplo Práctico de Envío y Recepción

### Ejemplo: Ajustar velocidad a 100 Hz y Límite de Potencia a 50,000 mW

* **Velocidad**: `100` (`0x0064`)
* **Potencia**: `50000` (`0xC350`)
* **Cálculo Checksum**: `0xAA + 0x00 + 0x64 + 0xC3 + 0x50 + 0x00 + 0x00 = 0x0221` -> Byte bajo = `0x21`

**Trama Hex de Envío (TX Host)**:
```hex
AA 00 64 C3 50 00 00 21 55
```

---

## 6. Archivos de Referencia en el Proyecto

* [`UR_Ctrl.c`](无感/SDK/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/Kernal_Source/UR_Ctrl.c#L141-L260) - Implementación de `UartDealRX()` y `UartDealTX()`.
* [`UR_Ctrl.h`](无感/SDK/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/Include/UR_Ctrl.h) - Estructura de buffer en anillo y prototipos.
* [`MC_Parameter.h`](无感/SDK/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/Include/MC_Parameter.h) - Definición del BaudRate (9600).
* [`Global_Variable.h`](无感/SDK/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418/Include/Global_Variable.h) - Declaración de bits de error y variables globales.

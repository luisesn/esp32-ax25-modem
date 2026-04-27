# esp32-aprs-modem

Módem APRS (AX.25 sobre AFSK Bell-202, 1200 bps) para **ESP32**, basado en ESP-IDF v5.x.

> ⚠️ **Estado: no funcional.** El proyecto compila y arranca, pero la recepción y la transmisión no operan de forma fiable. Los detalles técnicos y el plan de arreglos están en [report.md](report.md).

Este repositorio adapta [LibAPRS-esp32-i2s](https://github.com/handiko/LibAPRS-esp32-i2s) — a su vez fork de [LibAPRS](https://github.com/markqvist/LibAPRS) de markqvist — para funcionar bajo ESP-IDF (no Arduino). Usa el periférico I2S en modo PDM para la salida de audio y ADC one-shot para la entrada.

## Hardware objetivo

- **MCU**: ESP32 (clásico, Xtensa LX6 dual-core).
- **Salida de audio (TX)**: I2S PDM en GPIO 25 (DOUT) y GPIO 26 (CLK).
  Requiere un filtro paso-bajo externo para convertir el PDM en audio analógico hacia la entrada MIC del transceptor.
- **Entrada de audio (RX)**: ADC one-shot en `ADC2_CH0` (GPIO 4 en el código actual — la documentación interna sugiere GPIO 35 / ADC1_CH7; ver [report.md](report.md)).
  Entrada AC-acoplada y polarizada a ~1,65 V desde la salida de altavoz del transceptor.
- **PTT**: GPIO 33, activo en nivel bajo (0 = transmitir).

## Estructura del repositorio

```
esp32-aprs-modem/
├── CMakeLists.txt              proyecto ESP-IDF raíz
├── sdkconfig                   configuración IDF (target = esp32)
├── main/
│   ├── main.c                  app_main — ejemplo mínimo de recepción
│   ├── idf_component.yml       dependencia esp-dsp
│   └── LibAPRS-esp32-i2s/      librería de modulación/demodulación AFSK + AX.25
├── managed_components/         dependencias gestionadas (esp-dsp)
├── CLAUDE.md                   guía de contexto para Claude Code
└── report.md                   informe de problemas y mejoras
```

## Dependencias

- ESP-IDF **≥ v5.1** (usa las nuevas APIs `driver/i2s_pdm.h` y `esp_adc/adc_oneshot.h`; las antiguas `driver/i2s.h` y `driver/adc.h` están deprecated).
- `espressif/esp-dsp` (se resuelve automáticamente vía `idf_component.yml`).

## Compilación y flasheo

```bash
# Exportar el entorno de ESP-IDF previamente (. ./export.sh o export.bat)
idf.py set-target esp32
idf.py build
idf.py -p <PUERTO_SERIE> flash monitor
```

En Windows con el PowerShell/CMD de ESP-IDF, sustituye `<PUERTO_SERIE>` por `COM3`, `COM4`, etc.

## Qué hace el firmware actualmente

1. `APRS_init()` configura I2S PDM para TX y ADC one-shot para RX y arranca la tarea `receive_audio_task`.
2. `APRS_setCallsign("NO0CALL", 1)` fija el indicativo (editar en [main/main.c](main/main.c) para tu propia licencia).
3. `receive_audio_task` muestrea el ADC en bucle, aplica una "squelch" por energía y pasa las muestras al demodulador AFSK; cada vez que llega una trama AX.25 válida se llama a `aprs_msg_callback`.
4. La tarea `processPacket` sondea un flag y *imprimiría* la cabecera del paquete (el volcado del cuerpo está comentado — ver [main.c:59-61](main/main.c#L59-L61)).
5. La transmisión (no ejercitada en `main.c`) está disponible vía `APRS_sendLoc()`, `APRS_sendMsg()`, `APRS_sendPkt()`.

## Uso mínimo

```c
#include "LibAPRS-esp32-i2s/src/LibAPRS.h"

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH  false

void app_main(void) {
    APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
    APRS_setCallsign("NO0CALL", 1);

    // Ejemplo: enviar una baliza de posición
    APRS_setLat("4024.00N");
    APRS_setLon("00342.00W");
    APRS_setSymbol('n');
    APRS_sendLoc("Test beacon", 11);
}
```

## Limitaciones y problemas abiertos

Ver [report.md](report.md). Los principales son:

- Sample rate del ADC **no controlado** (se usa `adc_oneshot_read` en un bucle tight), por lo que la demodulación pierde temporización.
- El RX procesa audio **por ráfagas** en lugar de continuamente → se rompe la sincronización HDLC.
- La salida I2S **PDM** requiere un filtro paso-bajo bien dimensionado; el escalado actual de la muestra (`(sample << 7) + (1<<15)`) genera amplitudes asimétricas.
- Offset DC hardcodeado a **30523** en `process_audio` — era válido para I2S-ADC de 16 bits, pero ahora con `adc_oneshot_read` (12 bits, 0–4095) es incorrecto.
- Se reserva ~**96 KB de RAM** en buffers FFT que solo se usan como telemetría de depuración.
- `freeMemory()` devuelve una constante ficticia; `FakeArduino::Serial` es un stub vacío.

## Indicativo y licencia

Edita [main/main.c:75](main/main.c#L75) con tu propio indicativo antes de transmitir. **Transmitir APRS en la banda amateur requiere licencia de radioaficionado** válida para la región del usuario. El callsign por defecto `NO0CALL` no debe emitirse sin autorización expresa del titular.

## Licencia

El código heredado de LibAPRS mantiene su licencia original (ver [main/LibAPRS-esp32-i2s/LICENSE](main/LibAPRS-esp32-i2s/LICENSE)). Las adaptaciones de este repositorio se distribuyen bajo los mismos términos.

## Créditos

- [Mark Qvist](https://github.com/markqvist) — LibAPRS original.
- [handiko](https://github.com/handiko) — port a ESP32 con I2S.
- Francesco Sacchi — código base de decodificación AX.25.

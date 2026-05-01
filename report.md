# Informe técnico — esp32-aprs-modem

Fecha: 2026-05-01 (actualizado; original 2026-04-17)
Autor: análisis automático (Claude Code)

Este documento lista únicamente los problemas que siguen vigentes en el código actual.
Los problemas históricos ya corregidos se eliminaron de este informe para evitar ruido.

## Estado actual (2026-05-01)

**Build:** binario ~904 KB, 47 % libre. Compilación limpia con `idf.py reconfigure && ninja -C build` (IDF 6.1).

**Infraestructura completada:**
- Control PTT centralizado en `AFSK.cpp` (activo-alto, GPIO 26).
- Conmutación half-duplex I2S0: ADC y DAC nunca activos simultáneamente.
- Cola TX `s_tx_queue` despacha desde `receive_audio_task` (propietaria del mutex ADC).
- FIFO `_locked` variants: `static inline` con `portMUX_TYPE` compartido.
- Indicativo/SSID leídos de `config.json` en arranque (modo KISS y modo APRS).
- `config.json` editable en runtime vía `POST /api/config` sin reflashear SPIFFS.

**Funcionalidades operativas:**
- KISS TNC bidireccional sobre WiFi TCP (port 8001).
- Servidor HTTP port 80: UI web SPA (log APRS, mensajes, posición, config, audio).
- WebSocket `/ws`: audio IMA ADPCM + eventos JSON APRS/ACK/DIGI.
- WAV TCP port 8080 para streaming externo (ffplay, VLC).
- Auto-ACK: detecta mensajes `{NNN}` dirigidos al propio indicativo y responde.
- Digipeater WIDEn-N: aliases múltiples, inserción en path, dedup FNV-1a (TTL 30 s), logging serie + WebSocket badge DIGI.
- Baliza morse CW periódica (configurable tono/WPM/periodo/callsign; one-shot REST).
- Baliza de posición APRS: `POST /api/aprs/beacon` (lat/lon decimal → formato APRS 1.01).
- Gateway IP RFC 1226 (`ax25ip.c`): lwIP custom netif, PID=0xCC, MTU=300, activado con `ip.enabled: true`.

---

## 1. Puntos de código pendientes

### 1.1 `esp-dsp` en `idf_component.yml` sin uso activo

**Archivo:** `main/idf_component.yml`

Los buffers FFT y la dependencia `dsps_*` se eliminaron. `esp-dsp` sigue declarado como dependencia, lo que alarga el build innecesariamente. Sus ficheros `ekf.cpp` / `ekf_imu13states.cpp` requieren parches locales de `#include <cmath>` para compilar (ya aplicados en `managed_components/`).

**Solución**: eliminar la entrada `espressif/esp-dsp` de `idf_component.yml` y borrar el directorio `managed_components/espressif__esp-dsp/`.

**Impacto**: build más rápido, sin cambio funcional.

---

### 1.2 `freeMemory()` en `FakeArduino.cpp`

**Archivo:** `main/LibAPRS-esp32-i2s/src/FakeArduino.cpp`

`freeMemory()` sigue devolviendo la constante `10000000`. No se usa en el código activo: `main.c` (modo APRS) ya llama `esp_get_free_heap_size()` directamente. Impacto: nulo en la práctica. Pendiente por completitud.

**Solución**: sustituir la constante por `(int)esp_get_free_heap_size()` en `FakeArduino.cpp`.

---

### 1.3 `APRS_poll` en modo APRS clásico

**Archivo:** `main/LibAPRS-esp32-i2s/src/AFSK.cpp` (función `receive_audio_task`)

`APRS_poll()` se llama cada 4 muestras decimadas dentro de `receive_audio_task`. En modo KISS esto es correcto (el callback `on_ax25_raw_frame` es rápido). En modo APRS clásico (`TNC_MODE_APRS`), el callback `aprs_msg_callback` hace un `malloc` y una copia de la trama — carga extra en el hot path de RX. Riesgo: si el malloc es lento (heap fragmentado), introduce jitter en el procesamiento de muestras.

**Solución** (solo si se usa modo APRS): mover el callback a una tarea separada con notificación por `xTaskNotify`, similar a `processPacket`. En modo KISS el comportamiento actual es correcto y no debe cambiarse.

---

## 2. Verificación hardware pendiente

| Punto | Estado |
|-------|--------|
| RX con señal RF real (transceptor o SDR) | ⚠️ pendiente |
| TX: datos decodificados por receptor externo | ✅ verificado |
| Gateway IP RFC 1226 en hardware | ⚠️ pendiente |
| Digipeater: retransmisión verificada en hardware | ⚠️ pendiente |

---

## 3. Trampas de implementación documentadas

Las trampas de bajo nivel (I2S0 compartido, timeout ADC, `vTaskDelay(1)`, `TX_SAMPLE_BUFLEN=2048`, GPIO26/PTT/DAC2, polaridad PTT, escritura completa al DMA) están documentadas con detalle en [CLAUDE.md](CLAUDE.md) sección "Convenciones y trampas conocidas".

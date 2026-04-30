# Informe técnico — esp32-aprs-modem

Fecha: 2026-04-30 (actualizado; original 2026-04-17)
Autor: análisis automático (Claude Code)

Este documento lista únicamente los problemas que siguen vigentes en el código actual.
Los problemas históricos ya corregidos se eliminaron de este informe para evitar ruido.

## Addendum de estado actual (2026-04-30)

**Estado del build:** binario 904 KB, 47 % libre. Compilación limpia con `idf.py reconfigure && ninja -C build` (IDF 6.1).

**Infraestructura completada:**
- `ptt.c/.h` eliminados; control PTT centralizado en `AFSK.cpp`.
- `transport_wifi.c` usa config WiFi/AP desde `config.json`.
- FIFO `_locked` variants: `static inline` con `portMUX_TYPE` compartido.
- Indicativo/SSID leídos de `aprs.callsign` / `aprs.ssid` en config.json (modo KISS).

**Nuevas funcionalidades (sesión nocturna 2026-04-30):**
- `audio_stream.c/h` — servidor HTTP port 80 (UI web), WebSocket `/ws` (audio IMA ADPCM + JSON APRS), WAV TCP port 8080, REST `/api/aprs/send` y `/api/me`.
- `index.html` — SPA completa: log APRS, envío de mensajes, streaming de audio, badges PARA MÍ / ACK / TX.
- Auto-ACK: `try_auto_ack()` en `main.c` detecta mensajes dirigidos con `{NNN}`, transmite ACK y notifica al frontend vía WebSocket.
- `ax25ip.c/h` — gateway IP RFC 1226 (lwIP custom netif, PID=0xCC, MTU=300, broadcast QST-0). Activado con `ip.enabled: true` en config.json y `CONFIG_LWIP_IP_FORWARD=y` en sdkconfig.
- `APRS_queue_msg`, `APRS_queue_ack`, `APRS_getCallsign` añadidas a LibAPRS.cpp.

**Único cierre pendiente de hardware:**
- Verificación RX con señal RF real.
- Verificación gateway IP RFC 1226 en hardware.

---

## 1. Estado de temas vigentes del informe

Todos los temas de código que estaban marcados como vigentes quedaron resueltos en esta ronda:

1. `FIFO.h`: `fifo_*_locked` ahora protege con `taskENTER_CRITICAL`/`taskEXIT_CRITICAL` sobre `portMUX_TYPE` compartido.
2. `AFSK.cpp`: `APRS_poll()` se movió fuera del hot path de RX a una tarea dedicada `aprs_poll_task`.
3. `LibAPRS.cpp`: `APRS_init()` ya no usa `malloc`; ahora inicializa un `Afsk` estático.
4. `FakeArduino.cpp`: se implementaron `print/println` para evitar símbolos sin definir si se usa `APRS_printSettings()`.
5. `src.ino`: eliminado del árbol de `main/LibAPRS-esp32-i2s/src/`.
6. `device.h`/`AFSK.h`: eliminadas macros legacy sin uso (`F_CPU`, `FREQUENCY_CORRECTION`, `KEEP_RECORDING_THRESH`, `CPU_FREQ`).

Build validado tras estos cambios: `idf.py build` finaliza correctamente.

---

## 2. Único punto pendiente (requiere hardware)

- **Verificación RX con señal RF real**: no se puede cerrar desde este entorno sin acceso a radio/transceptor/SDR físico.

---

## 3. Resumen ejecutivo actual

- No quedan temas de código abiertos de los que estaban vigentes en este informe.
- El único cierre pendiente es la validación final de RX sobre hardware real.

# Informe técnico — esp32-aprs-modem

Fecha: 2026-04-30 (actualizado; original 2026-04-17)
Autor: análisis automático (Claude Code)

Este documento lista únicamente los problemas que siguen vigentes en el código actual.
Los problemas históricos ya corregidos se eliminaron de este informe para evitar ruido.

## Addendum de estado actual (2026-04-30)

- `idf.py build` verificado en este entorno: compila correctamente (warnings de CMake/IDF no bloqueantes).
- `ptt.c/.h` eliminados como código muerto; el control de PTT quedó centralizado en `AFSK.cpp`.
- `transport_wifi.c` usa configuración WiFi/AP desde `config.json` (`wifi.*`, `ap.*`).
- `main/spiffs_data/config.json` ya incluye sección `ip` (`enabled`, `addr`, `netmask`, `gateway`, `ssid`) para el modo IP nativo planificado.
- `main.c::audio_level_task` añade alarma de nivel: parpadeo rápido del LED RX cuando el pico está fuera de rango (`AUDIO_LEVEL_TOO_LOW/HIGH`).

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

# /flash-spiffs — Flashear solo la partición SPIFFS

Flashea únicamente el binario SPIFFS (index.html + config.json) sin tocar el firmware.
Mucho más rápido que un flash completo cuando solo cambia la UI web o la configuración.

## Partición SPIFFS

- Offset: `0x350000`
- Tamaño: `0xb0000` (704 KB)
- Binario generado: `build/spiffs.bin`

## Pasos

1. Comprueba que `build/spiffs.bin` existe. Si no existe, indica al usuario que ejecute `/build` primero.
2. El argumento de este comando es el puerto serie, ej. `/flash-spiffs /dev/ttyUSB0`.
   - Si no se pasa argumento, intenta detectar el puerto automáticamente con:
     `ls /dev/ttyUSB* /dev/ttyACM* /dev/cu.usbserial* 2>/dev/null`
   - Si hay más de uno, pregunta al usuario cuál usar.
   - Si no hay ninguno, indica que conecte el dispositivo y pase el puerto como argumento.
3. Ejecuta:
   ```
   esptool.py --port <PORT> --baud 921600 write_flash 0x350000 build/spiffs.bin
   ```
4. Reporta éxito o error.

## Cuándo usar

- Cambios solo en `main/spiffs_data/index.html` (UI web)
- Cambios solo en `main/spiffs_data/config.json` (configuración)
- Cualquier combinación de los anteriores sin cambios en código C/C++

Si también hay cambios en código fuente, usar `idf.py flash` completo en su lugar.

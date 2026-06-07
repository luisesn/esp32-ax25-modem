# /build — Compilar firmware ESP32-APRS

Compila el proyecto usando el workaround obligatorio de IDF 6.1 y reporta el tamaño resultante.

## Pasos

1. Ejecuta `idf.py reconfigure` para regenerar `build/config/sdkconfig.cmake` antes de que ninja lo necesite.
2. Ejecuta `ninja -C build` para compilar.
3. Si la compilación falla, muestra el error y detente — no continúes al paso siguiente.
4. Ejecuta `idf.py size` para mostrar el uso de flash y RAM.
5. Ejecuta `idf.py size-components` y muestra los 10 componentes que más flash consumen.
6. Reporta: tamaño binario total, % de flash libre, y si hubo warnings nuevos respecto a la última compilación.

## Notas

- No usar `idf.py build` directamente — lanza ninja demasiado pronto y falla con "FAILED: build.ninja" en build directories recién limpiados.
- Si el usuario pasa un argumento (ej. `--clean`), ejecuta `idf.py fullclean` antes del reconfigure.
- Si hay errores de compilación, no ejecutes `idf.py size`.

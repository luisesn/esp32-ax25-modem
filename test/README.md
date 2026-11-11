# Banco de pruebas en host

Sin ESP-IDF ni hardware: gcc nativo con ASan/UBSan.

```bash
cmake -S test -B build-test && cmake --build build-test -j8
ctest --test-dir build-test --output-on-failure
```

- `unit/test_*.c`: un ejecutable por fichero (kiss, rs/il2p, dedup, config_validate, digipeater, heard). Las fuentes de producción que enlaza cada uno van en `unit/test_X.srcs`. Marco mínimo en `unit/test.h`; `KNOWN_BUG(cond, motivo)` documenta un fallo conocido sin romper ctest (avisa cuando se corrige).
- `stubs/`: sustitutos de esp_log, esp_timer (reloj falso), FreeRTOS y driver/gpio.
- `demod/`: regresión del demodulador RX v2 (`#include "AFSK.cpp"` con stubs) con audio sintético y corpus de WAV reales; ver `demod/README.md`.

## Fallos conocidos
- Demodulador v2: poca tolerancia a deriva de reloj (±0,5 % ya pierde tramas) y cae a 0 con SNR 9 dB (B25 abierto; ver `demod/README.md`).
- Los bugs B23 (kiss FEND tras FESC) y B24 (digipeater WIDE2-1) hallados por este banco están corregidos en v1.11.1.

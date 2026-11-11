# Regresión del demodulador RX v2 en host

`demod_regress` compila el `AFSK.cpp` **real** de producción (vía `#include` en
`harness.cpp`, con stubs de ESP-IDF en `stubs/`) y le alimenta audio: sintético o
capturado. No modifica nada de `main/`. Replica la cadena de `receive_audio_task`
(5 muestras de 12 bit a 48 kHz → FIR `fir5` → AGC/DC `adc_to_s8_v2` → squelch →
`afsk_v2_process` → `rxFifo`) y un decodificador de bytes equivalente a
`aprs_poll_v2` (flags, escapes, CRC-CCITT).

## Uso

```bash
cmake -S test -B build-test-c
cmake --build build-test-c --target demod_regress
./build-test-c/demod/demod_regress synth            # batería sintética con umbrales (ctest: demod_synth)
./build-test-c/demod/demod_regress calibrate        # igual, imprime umbrales sugeridos (90 % de lo medido)
./build-test-c/demod/demod_regress fichero.wav [--expect N]   # lista las tramas (hex + TNC2)
./build-test-c/demod/demod_regress corpus [dir]     # todos los WAV de test/demod/corpus (ctest: demod_corpus)
./build-test-c/demod/demod_regress genwav out.wav [n_tramas] [snr_dB]   # WAV sintético 8 bit 9600 Hz
```

El modo `synth` tarda ~30 s con ASan/UBSan. Los escenarios están en `scenarios.inc`
(tramas enviadas, longitud, nivel, SNR, deriva de reloj, desplazamiento de tonos,
preámbulo, ruido/basura entre tramas...). El resultado es determinista (PRNG propio);
`min_ok` es la suma de tramas **idénticas a las enviadas** sobre 2 semillas, con ~10 %
de margen respecto a lo medido en v1.11.0. Si cambias el demodulador a propósito, ejecuta
`calibrate` y actualiza `scenarios.inc`. Escenarios con umbral 0 (SNR 9 dB, reloj ±2 %)
son informativos: documentan un límite actual, no una exigencia.

## Capturar audio real

Desde el dispositivo (formato WAV mono PCM 8 bit unsigned, 9600 Hz):

```bash
curl -o captura.wav 'http://<ip-del-esp32>/api/rx/capture?secs=30&rate=9600'
```

Cada muestra de 8 bit se pasa al demodulador como `(v-128)*16+2048` (12 bit) y se
repite ×5 para volver a 48 kHz. (El modo de captura toma el audio ya decimado, así
que el FIR de 5 muestras ve valores iguales: no es idéntico al camino ADC, pero sí
representativo para regresiones del resto de la cadena.) También se aceptan WAV mono
de 16 bit y otras frecuencias (remuestreo por vecino más próximo a 48 kHz).

## Añadir al corpus

1. Copia el WAV a `test/demod/corpus/` con un nombre descriptivo:
   `YYYY-MM-DD_radio_vol50_sitio.wav` (indica radio, volumen y escenario).
2. Mira qué decodifica hoy: `demod_regress corpus/fichero.wav`.
3. Crea `corpus/fichero.expect` con el mínimo exigido (algo por debajo de lo obtenido o
   del número real de tramas verificado con otro decodificador):
   ```
   # comentario libre
   min_frames=12
   ```
   (vale también un fichero con solo el número). Sin `.expect` el fichero se decodifica
   pero no falla nunca.
4. Vuelve a configurar CMake (`cmake -S test -B build-test-c`) para que se registre
   `demod_corpus` si el corpus estaba vacío.

Ficheros incluidos: `sintetico_snr15.wav` y `sintetico_snr12.wav` (generados con `genwav`,
sólo ejercitan el camino de ficheros).

## Comparar con Dire Wolf

```bash
atest -B 1200 captura.wav
```

`atest` espera WAV PCM de 8 o 16 bit; si tu fichero da problemas (cabecera, frecuencia),
conviértelo con sox: `sox captura.wav -r 44100 -b 16 captura16.wav` (para 9600 Hz nativos
suele bastar `atest` directo). Compara el número de tramas con el de `demod_regress`: la
diferencia es la medida de calidad del demodulador v2 frente a una referencia (Dire Wolf
con `-P ...` o `-B 1200` según el modo). Usa el recuento de Dire Wolf como `min_frames`
cuando el firmware deba igualarlo.

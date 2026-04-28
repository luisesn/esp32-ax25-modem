# Informe técnico — esp32-aprs-modem

Fecha: 2026-04-28 (actualizado; original 2026-04-17)
Autor: análisis automático (Claude Code)

Este documento detalla los problemas que impiden el correcto funcionamiento del módem y propone mejoras concretas. Los problemas están ordenados por gravedad (bloqueantes → mejoras).

---

## 1. Problemas bloqueantes (impiden que RX/TX funcionen)

### 1.1 El sample rate del ADC de recepción no está controlado
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:636-656](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L636-L656)

```c
for (size_t i = 0; i < TNC_I2S_BUFLEN; i++) {
    int adc_value;
    esp_err_t error = adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_value);
    ...
    buffer[i] = (uint16_t)adc_value;
}
```

El código lee el ADC en un bucle tight usando `adc_oneshot_read`, que es una API **de uso puntual**, no periódico. No existe ningún temporizador ni DMA que garantice los 48 kHz (9600 × OVERSAMPLING=5) que la demodulación asume. El resultado es un muestreo irregular cuya tasa media depende de la carga del CPU, de FreeRTOS, del nivel de logging, etc. Sin reloj de muestreo fijo, la demodulación AFSK **no puede funcionar**: el autocorrelador y el detector de fase viven sobre una base temporal falsa.

**Solución**: sustituir `adc_oneshot_read` por `adc_continuous_read` (ESP-IDF ≥ 5.0) con un ring buffer DMA a 48 kHz, o usar `i2s_std` en modo RX alimentando el ADC vía I2S (modo legacy I2S-ADC, que es lo que el nombre de la librería sugiere pero ya no está implementado aquí).

---

### 1.2 La recepción trabaja por ráfagas, no continuamente
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:630-692](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L630-L692)

El esquema actual es:

1. Graba audio mientras `process_audio` detecte energía (squelch).
2. Cuando baja la energía, **detiene la grabación**.
3. Recorre `audio_buf_full` de una sola vez pasando cada muestra a `AFSK_adc_isr`.
4. Calcula una FFT para telemetría.
5. Vuelve al paso 1.

Problema: entre el paso 3 y el siguiente ciclo de grabación, **el demodulador no recibe muestras**. HDLC depende de un flujo continuo para detectar flags `0x7E` y mantener la sincronización de fase. Además, el "silencio" que dispara el procesamiento suele cortar el final del paquete, que es precisamente la zona con el CRC.

**Solución**: una tarea productora (ADC DMA → ring buffer) y una tarea consumidora que llame a `AFSK_adc_isr` **en cuanto haya una muestra**, sin pausas. Abandonar el concepto de "grabar→procesar" y pasar a streaming.

---

### 1.3 Unidad/canal de ADC incoherentes con la documentación interna
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:66-76](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L66-L76) vs [main/LibAPRS-esp32-i2s/src/device.h:56-59](main/LibAPRS-esp32-i2s/src/device.h#L56-L59)

El código inicializa:
```c
.unit_id = ADC_UNIT_2,
.atten   = ADC_ATTEN_DB_0,
adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, ...);
```

Pero `device.h` dice:
```c
#define I2S_ADC_UNIT     ADC_UNIT_1
#define I2S_ADC_CHANNEL  ADC1_CHANNEL_7   // GPIO 35
// La espri usa como entrada de audio el pin 35 y salida el 25
```

Problemas:
- **ADC2 entra en conflicto con Wi-Fi**. Cualquier activación del radio Wi-Fi hace que `adc_oneshot_read` devuelva `ESP_ERR_INVALID_STATE`.
- `ADC2_CH0` = **GPIO 4** en ESP32, no GPIO 35.
- `ADC_ATTEN_DB_0` limita el rango de entrada a ~0-750 mV; una salida típica de altavoz alcanza 1-2 V. Debe ser `ADC_ATTEN_DB_12` (≈0-3,1 V).

**Solución**: usar ADC1 canal 7 (GPIO 35) con `ADC_ATTEN_DB_12`, y las constantes de `device.h` en lugar de literales.

---

### 1.4 Offset DC incorrecto en `process_audio`
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:600-615](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L600-L615)

```c
average -= 30523; // empirically measured hackily...
average = average / 16;
```

El valor `30523` corresponde al punto medio **de un ADC de 16 bits** (0-65535), que es lo que devolvía el antiguo driver I2S-ADC. Con `adc_oneshot_read` hoy los valores son **0-4095** (12 bits, atenuación 0 dB).

- `average` real: 0 ≤ x ≤ 4095.
- Offset actual esperado: ~2047.
- Tras `-= 30523`, `average` queda en el rango **−30523 a −26428**.
- Dividido por 16 → **−1907 a −1651**, clampeado a **−128** siempre.

Resultado: el demodulador recibe una constante de −128, por lo que nunca verá transiciones y **la recepción no puede producir ni un solo bit correcto**.

**Solución**: calcular el offset de forma dinámica (filtro paso-alto IIR de primer orden o media móvil) o bien usar `(raw - 2048) * 16` cuando el ADC esté en 12 bits + atenuación 12 dB.

---

### 1.5 `AFSK_transmit` saltea bytes
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:182-192](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L182-L192)

```c
void AFSK_transmit(char *buffer, size_t size) {
    fifo_flush(&AFSK_modem->txFifo);
    for (int i=0; i<size; i++) {          // ← incrementa i
        if (fifo_isfull_locked(&AFSK_modem->txFifo)) {
            transmit_audio_i2s(AFSK_modem);
        }
        afsk_putchar(buffer[i++]);         // ← VUELVE a incrementar i
    }
    finish_transmission();
}
```

Doble incremento de `i`: envía los bytes 0, 2, 4… y termina leyendo fuera del buffer. `AFSK_transmit` no se llama desde `main.c` actualmente, pero cualquier uso directo de la API rompería.

**Solución**: eliminar el `i++` de dentro de `afsk_putchar(buffer[i])`.

---

### 1.6 La salida PDM no está bien dimensionada como audio analógico
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:130-163](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L130-L163)

PDM (Pulse-Density Modulation) es un flujo de 1 bit a tasa muy alta que *necesita un filtro RC/Sallen-Key externo* para recuperar la señal analógica. La conversión dentro del loop es:

```c
tx_sample_buf[i] = (sample << 7) + (1<<15);
```

Con `sample` en 0..255:
- `sample << 7` → 0..32640
- `+ 32768`     → 32768..65408

Interpretado como `int16_t` (que es lo que I2S espera en el modo 16-bit): **−32768..−130**, siempre negativo. La "portadora" queda offseteada al extremo inferior del rango. El modulador PDM usará solo la mitad inferior de la escala, lo que reduce drásticamente el SWR/SNR tras el filtro.

Además, el fix del filtro paso-bajo externo no está documentado. En la práctica muchos montajes con este fork del repositorio original dependen de un hardware específico no descrito en el README.

**Solución preferida**: usar el **DAC interno** del ESP32 (GPIO 25 = DAC1) con el driver `driver/dac_continuous.h`, que acepta directamente muestras de 8 bits y produce un voltaje analógico real. Alternativamente, si se mantiene PDM, documentar el filtro externo y centrar la muestra (`(int16_t)((sample - 128) << 8)`).

---

### 1.7 `main.c` no imprime el contenido del paquete recibido
**Archivo**: [main/main.c:44-68](main/main.c#L44-L68)

```c
for (int i = 0; i < incomingPacket.len; i++) {
    //Serial.write(incomingPacket.info[i]);
}
```

El bucle está comentado. Aunque la demodulación funcionara, el usuario no vería el payload — sólo "SRC/DST". Hay que sustituirlo por `putchar(incomingPacket.info[i])` o `printf("%c", ...)`.

---

## 2. Problemas de arquitectura / fiabilidad

### 2.1 Buffers FFT gigantes sin propósito funcional
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:31-34, 672-690](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L31-L34)

```c
static float fft_input[8192];   // 32 KB
static float fft_window[8192];  // 32 KB
static float fft_re[4096];      // 16 KB
static float fft_im[4096];      // 16 KB
// Total: 96 KB de RAM estática
```

Se emplean solo para imprimir `FFT: 1200Hz mag: ... 2200Hz mag: ...`. Como telemetría no aporta nada a la demodulación (ya hay un autocorrelador IIR específico para ese trabajo) y consume ~1/3 de la SRAM disponible del ESP32 (512 KB totales, pero IRAM/DRAM/DIRAM repartidas).

**Solución**: eliminar o condicionar con `#ifdef DEBUG_FFT` y usar `heap_caps_malloc` solo cuando se active. Si se quiere un detector de tono por FFT, bastaría con 256 muestras (~1 KB) para obtener resolución suficiente a 1200/2200 Hz.

---

### 2.2 `freeMemory()` devuelve una constante ficticia
**Archivo**: [main/LibAPRS-esp32-i2s/src/LibAPRS.cpp:319-322](main/LibAPRS-esp32-i2s/src/LibAPRS.cpp#L319-L322)

```c
int freeMemory() {
    // TODO quitar dependencia de esto
    return 10000000;
}
```

Hace que el chequeo de RAM en `aprs_msg_callback` en `main.c` sea siempre verdadero. Con tramas AX.25 de hasta 330 bytes y sin comprobación real, un agotamiento de heap causaría un crash.

**Solución**: implementar con `esp_get_free_heap_size()`.

---

### 2.3 Fugas de memoria
**Archivo**: [main/LibAPRS-esp32-i2s/src/LibAPRS.cpp:55-62](main/LibAPRS-esp32-i2s/src/LibAPRS.cpp#L55-L62)

`APRS_init` hace `malloc(sizeof(Afsk))` que nunca se libera. No es crítico (solo una vez en el arranque), pero es mejor declarar `Afsk` como global estático, evitando la asignación dinámica de 16 KB+ en heap fragmentable.

---

### 2.4 Mezcla de APIs I2S antiguas y nuevas en los includes
**Archivo**: [main/LibAPRS-esp32-i2s/src/device.h:39-52](main/LibAPRS-esp32-i2s/src/device.h#L39-L52)

Define constantes `I2S_BITS_PER_SAMPLE_16BIT`, `I2S_CHANNEL_FMT_ONLY_RIGHT`, `ADC1_CHANNEL_7` (tipos deprecated desde IDF v5), pero el código real usa la nueva API (`I2S_DATA_BIT_WIDTH_16BIT`, `I2S_SLOT_MODE_MONO`, `ADC_CHANNEL_...`). Esas macros viejas pueden emitir warnings y confunden al lector.

**Solución**: actualizar `device.h` a las constantes nuevas (o eliminarlas si no se usan).

---

### 2.5 `TX` y `RX` no son simultáneos ✅ resuelto 2026-04-27
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp)

Resuelto como parte del arreglo del conflicto I2S0 (ver sección 2.10). La conmutación half-duplex (`switch_to_tx` / `switch_to_rx`) y el flag `tx_mode` garantizan que el ADC nunca está activo mientras el DAC transmite.

---

### 2.6 Falta protección de concurrencia entre ISR y tareas
**Archivo**: [main/LibAPRS-esp32-i2s/src/FIFO.h](main/LibAPRS-esp32-i2s/src/FIFO.h), [main/LibAPRS-esp32-i2s/src/AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp)

Las macros `fifo_push_locked` / `fifo_pop_locked` tenían `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` en el fork AVR; aquí están **vacías**. Si RX (tarea con prioridad 10) y TX (tarea de usuario con prioridad 5) se invocan simultáneamente sobre los mismos FIFO (`txFifo`), la estructura puntero-a-puntero puede corromperse en el cambio de contexto.

**Solución**: envolver los acceso críticos con un `portMUX_TYPE` (`taskENTER_CRITICAL(&mux)`) o con un `SemaphoreHandle_t` recursivo.

---

### 2.7 `APRS_poll()` se llama desde la tarea de RX — bloqueo potencial
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp:662-667](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L662-L667)

`APRS_poll` invoca `ax25_poll`, que puede llegar a ejecutar `ctx->hook` (el callback del usuario) síncronamente. Cualquier operación lenta en el callback (impresión, write a flash, Wi-Fi) retrasa el siguiente batch de muestras y empeora el 1.2.

**Solución**: desacoplar. La tarea de RX sólo debería empujar muestras al demodulador; una tarea separada (p. ej. `aprs_dispatch_task`) consumiría bytes del `rxFifo` y ejecutaría los callbacks.

---

### 2.8 `FakeArduino::Serial` es un stub sin implementación
**Archivo**: [main/LibAPRS-esp32-i2s/src/FakeArduino.h](main/LibAPRS-esp32-i2s/src/FakeArduino.h), [FakeArduino.cpp](main/LibAPRS-esp32-i2s/src/FakeArduino.cpp)

Declara métodos `print`/`println` pero **no los define**. Cualquier llamada (p. ej. desde `APRS_printSettings()`) provocará un link error si se intenta usar. Es código heredado que sólo compila porque nadie lo invoca.

**Solución**: o bien implementar los métodos redirigiéndolos a `printf`, o bien eliminar `Serial.*` de `LibAPRS.cpp` y sustituirlo por `printf` directamente.

---

### 2.9 `src.ino` dentro del directorio `src/`
**Archivo**: [main/LibAPRS-esp32-i2s/src/src.ino](main/LibAPRS-esp32-i2s/src/src.ino)

Archivo Arduino residual. No está en el `SRCS` del CMakeLists, pero confunde y algunas herramientas (clangd, VS Code) lo indexan como C++ con errores.

**Solución**: moverlo a `examples/` o eliminarlo.

---

## 2.12 Cola TX entre tareas — violación del mutex `adc_continuous` ✅ resuelto 2026-04-28
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp), [main/main.c](main/main.c)

En modo KISS TNC, `server_task` llamaba directamente a `APRS_send_raw_frame`, que a su vez llama a `switch_to_tx` → `adc_continuous_stop`. El driver `adc_continuous` de ESP-IDF mantiene un mutex interno asociado a la tarea que llamó a `adc_continuous_start`. Invocar `stop` desde una tarea diferente viola ese mutex.

**Solución**: cola FreeRTOS `s_tx_queue` de capacidad 4 × `afsk_tx_frame_t`. `server_task` encola con `afsk_queue_tx_frame()` (no bloqueante). `receive_audio_task` comprueba la cola al inicio de cada iteración y ejecuta `s_tx_fn(data, len)` desde su propio contexto — la misma tarea que llamó a `adc_continuous_start`. `afsk_set_tx_fn(APRS_send_raw_frame)` registra la función real; debe llamarse en `app_main` antes de `APRS_set_raw_hook`.

---

## 2.13 TX hang: DAC no arranca, hambruna de DMA y escritura parcial ✅ resuelto 2026-04-28
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp)

Tres bugs independientes causaban que el PTT quedara pulsado indefinidamente o que la transmisión durara ~10 s:

**Bug A — DAC no arranca**: `vTaskDelay` estaba entre `adc_continuous_stop` y `adc_continuous_deinit` (en lugar de después de `deinit`), `dac_continuous_enable` se llamaba dentro de `transmit_audio_i2s` (después de la primera escritura), y no había priming del DMA. Resultado: `dac_continuous_write(-1)` bloqueaba indefinidamente.

**Solución**: en `switch_to_tx()`, (1) delay de 20 ms movido a después de `deinit`; (2) `dac_continuous_enable` movido aquí; (3) se escribe un descriptor completo de silencio (2048 × `0x80`) para cebar el oscilador I2S antes del audio real. Timeout de `dac_continuous_write` cambiado a 2000 ms con `afsk->sending = false` como salida de emergencia.

**Bug B — hambruna de descriptor DMA**: con `TX_SAMPLE_BUFLEN=320`, cada descriptor duraba 320/48000 s ≈ 6,7 ms. Las tareas WiFi (prioridad 23 > 10 de `receive_audio_task`) podían preemptar durante más tiempo, agotando el descriptor antes de que la tarea pudiera recargar. El semáforo interno `s_dac_wait_to_load_dma_data` expiraba → timeout.

**Solución**: `TX_SAMPLE_BUFLEN` subido de 320 a **2048** bytes (≈42 ms/descriptor). Con `desc_num=8`: pool de ~336 ms, suficiente margen frente a preempciones WiFi.

**Bug C — escritura parcial detiene el DMA**: cuando `AFSK_dac_isr` ponía `sending=false` a mitad del buffer (p.ej. 5 muestras), solo se enviaban 5 bytes al DMA. El DMA los consumía en <0,1 ms y se paraba. Las 20 escrituras de silencio posteriores expiraban individualmente a 500 ms → ~10 s de bloqueo.

**Solución**: `transmit_audio_i2s` siempre rellena el resto del buffer hasta `TX_SAMPLE_BUFLEN` con `0x80` y siempre escribe el buffer completo al DMA. `finish_transmission` simplificado de 20 × 256 bytes a una sola escritura de `TX_SAMPLE_BUFLEN` bytes.

---

## 2.10 Conflicto de hardware I2S0 entre `dac_continuous` y `adc_continuous` ✅ resuelto 2026-04-27
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp)

En el ESP32 clásico, ambos drivers (`dac_continuous` y `adc_continuous`) usan internamente el periférico I2S0 para DMA. Inicializarlos simultáneamente en `AFSK_hw_init` causaba:

```
W i2s_platform: i2s controller 0 has been occupied by dac_dma
E adc_share_hw_ctrl: adc_apb_periph_free called, but s_adc_digi_ctrlr_cnt == 0
abort() at adc_continuous_new_handle
```

`adc_continuous_new_handle` fallaba a mitad de su init (I2S0 ya tomado) e intentaba limpiar llamando a `adc_apb_periph_free` con el contador en 0 → `abort()`.

**Solución implementada**: conmutación half-duplex de I2S0.
- `AFSK_hw_init` arranca solo el ADC.
- `switch_to_tx()`: para + deinit ADC → crea DAC.
- `switch_to_rx()`: elimina DAC → recrea ADC.
- Flag `volatile bool tx_mode` pausa `receive_audio_task` durante TX.
- `adc_continuous_read` usa timeout de 20 ms (no `portMAX_DELAY`) para poder desbloquearse cuando `adc_continuous_stop` se llama.

---

## 2.11 Task WDT — `receive_audio_task` satura CPU sin ceder ✅ resuelto 2026-04-27
**Archivo**: [main/LibAPRS-esp32-i2s/src/AFSK.cpp — receive_audio_task](main/LibAPRS-esp32-i2s/src/AFSK.cpp)

Con prioridad 10 y el ring buffer del ADC siempre lleno (4 × 1024 muestras ≈ 85 ms de pool), `adc_continuous_read` nunca bloqueaba realmente. La tarea corría en bucle cerrado sin dejar correr a `IDLE0` → watchdog a los ~5 s:

```
E task_wdt: IDLE0 (CPU 0) did not reset watchdog in time
Tasks currently running: CPU 0: receive_audio_t
```

**Solución**: `vTaskDelay(1)` al final de cada iteración de procesamiento. Un tick de bloqueo garantiza que el planificador ceda al menos brevemente a `IDLE0`. Con 85 ms de buffer, el impacto en latencia es despreciable.

---

## 3. Problemas menores / deuda técnica

### 3.1 Constantes legacy AVR sin usar
`constants.h` (`m328p`, `m1284p`, `m644p`) y `device.h` (`DAC_DDR`, `DDRB`, `PORTC`, `F_CPU`) son restos del puerto AVR original. Eliminar.

### 3.2 `LibAPRS_vref` y `LibAPRS_open_squelch` no tienen efecto
**Archivos**: [LibAPRS.cpp:11-12](main/LibAPRS-esp32-i2s/src/LibAPRS.cpp#L11-L12), [AX25.cpp:16-17](main/LibAPRS-esp32-i2s/src/AX25.cpp#L16-L17)

Se declaran, se asignan desde `APRS_init`, y sólo se leen en un `if (LibAPRS_open_squelch) { /* LED_RX_ON comentado */ }`. El parámetro `ADC_REFERENCE` tampoco hace nada en el código actual. Documentar o quitar.

### 3.3 `aprs_msg_callback` como variable global volátil implícita
`gotPacket`, `incomingPacket` y `packetData` se comparten entre la tarea RX (que ejecuta el callback) y `processPacket`, sin `volatile` ni barrera de memoria. Funciona por suerte en ESP32 gracias al modelo de memoria TSO, pero es frágil.

**Solución**: usar una `QueueHandle_t` de FreeRTOS para pasar paquetes a `processPacket`.

### 3.4 `APRS_printSettings` usa `Serial.print`/`F(...)`
No produce salida (ver 2.8). Reescribir con `ESP_LOGI` o `printf` si se quiere conservar.

### 3.5 Uso de `printf` en paths críticos
`AFSK_txStart` y `finish_transmission` usan `printf`/`ESP_LOGI` en cada TX. A velocidades de transmisión bajas (1200 bps) esto no es crítico, pero conviene envolver con `ESP_LOGD` para poder silenciarlos en producción.

### 3.6 `TNC_I2S_BUFLEN` — macro heredada sin uso activo
`TNC_I2S_BUFLEN = 48000/8 = 6000` y las constantes relacionadas (`FULL_BUF_LEN`, `KEEP_RECORDING_THRESH`) fueron relevantes en el esquema "grabar→procesar" eliminado. Siguen definidas en `device.h` pero ya no se usan. Eliminar para evitar confusión.

---

## 4. Plan de acción recomendado

### Completado ✅
1. ~~Arreglar offset DC y escalado~~ (1.4) — EMA Q10.10 en `adc_to_s8`.
2. ~~Mover ADC a ADC1_CH7 con atenuación 12 dB~~ (1.3) — `AUDIO_ADC_*` en `device.h`.
3. ~~Arreglar `main.c` para que imprima el payload~~ (1.7) — `printf` con escapes.
4. ~~Sustituir `adc_oneshot_read` por `adc_continuous`~~ con DMA a 48 kHz (1.1).
5. ~~Pasar a streaming RX~~ en lugar de ráfagas (1.2) — `receive_audio_task` continua.
6. ~~Sustituir PDM por DAC interno~~ `dac_continuous` (1.6).
7. ~~Arreglar `AFSK_transmit`~~ (1.5) — eliminado doble incremento.
8. ~~Resolver conflicto I2S0 DAC/ADC~~ (2.10) — conmutación half-duplex.
9. ~~Pausar RX durante TX~~ (2.5) — `tx_mode` flag.
10. ~~Task WDT~~ (2.11) — `vTaskDelay(1)` en `receive_audio_task`.
11. ~~Cola TX entre tareas~~ (2.12) — `s_tx_queue` + `afsk_set_tx_fn`.
12. ~~TX hang / DMA starvation / escritura parcial~~ (2.13) — `TX_SAMPLE_BUFLEN=2048`, padding de silencio, timeout finito.

### Pendiente (orden sugerido)
1. **Verificación en hardware** — flashear y comprobar con `tncattach` + `tcpdump -i tnc0`. Es el paso más importante.
2. **2.2** `freeMemory()` → `esp_get_free_heap_size()` — trivial.
3. **2.6** Proteger FIFOs con `portMUX_TYPE` — necesario si TX y callback pueden solaparse.
4. **2.1** Quitar dependencia `esp-dsp` de `idf_component.yml` (los buffers FFT ya no existen pero el componente sigue declarado — alarga el build).
5. **2.8** Implementar `FakeArduino::Serial` con `printf` o eliminarlo.
6. **3.1 + 3.6** Limpiar constantes legacy AVR y macros sin uso.

> **3.3 ya no aplica**: `aprs_msg_callback` como global implícita fue eliminada al reescribir `main.c` con hooks explícitos (`APRS_set_msg_hook` / `APRS_set_raw_hook`).

---

## 5. Herramientas de diagnóstico sugeridas

- **`idf.py size-components`**: auditar RAM consumida por los buffers FFT (punto 2.1).
- **`idf.py app-trace`**: trazar el timing real del loop ADC (punto 1.1) para confirmar el jitter antes y después del arreglo.
- **`direwolf` o `multimon-ng`** alimentados por una SDR (p. ej. RTL-SDR con Gqrx): decodificar desde el aire el audio generado por el TX para validar la modulación.
- **Analizador lógico** en el GPIO 25 (salida DAC) o GPIO 26 (PTT) para verificar la temporización.
- **Osciloscopio** en la entrada del ADC para medir DC offset real y ajustar el filtro acoplador.

---

## 6. Resumen ejecutivo

El proyecto es funcional en el plano de **alto nivel** (API LibAPRS + AX.25 + HDLC + CRC), pero la **capa física de ESP32** está rota:

| Subsistema               | Estado (2026-04-28)                                     |
|--------------------------|---------------------------------------------------------|
| Codificación AX.25       | ✅ Correcto                                              |
| Modulación AFSK (lógica) | ✅ Correcto                                              |
| Entrada de audio (ADC)   | ✅ `adc_continuous` DMA a 48 kHz, ADC1_CH7 (GPIO 35), DC offset dinámico, streaming continuo |
| Salida de audio (DAC)    | ✅ `dac_continuous` sobre DAC1 (GPIO 25), muestras 8-bit directas |
| Half-duplex TX/RX        | ✅ Conmutación I2S0 entre ADC y DAC; `receive_audio_task` pausa durante TX |
| Task WDT                 | ✅ `vTaskDelay(1)` evita que `receive_audio_task` sature CPU |
| Cola TX entre tareas     | ✅ `s_tx_queue` + dispatch en `receive_audio_task`; respeta mutex ADC |
| Fiabilidad DAC/DMA TX    | ✅ `TX_SAMPLE_BUFLEN=2048` (42 ms/desc × 8 = 336 ms pool); padding silencio; timeout finito |
| TX verificado en HW      | ✅ Datos recibidos y decodificados por receptor externo |
| `main.c` / modo firmware  | ✅ KISS TNC con WiFi TCP (modo APRS consola preservado bajo `#if`) |
| KISS TNC / WiFi TCP       | ✅ Servidor TCP port 8001; compatible con `tncattach` y `direwolf` |
| Capa de transporte        | ✅ Interfaz abstracta `{ init, write }`; WiFi implementado; UART/BT como stubs futuros |
| RX verificado en HW      | ⚠️ Pendiente verificación con señal de RF real |
| Gestión de memoria        | ⚠️ `freeMemory()` sigue siendo constante ficticia (solo afecta modo APRS) |
| Concurrencia              | ⚠️ FIFOs sin protección `portMUX_TYPE` |
| Compilación               | ✅ Pasa `-Werror`; corregido error `IP2STR` con tipos POSIX vs ESP |

El código compila limpio y las rutas de datos TX y RX son arquitectónicamente correctas. La verificación final requiere hardware real (ESP32 + transceptor o SDR) y credenciales WiFi configuradas en `config.h`.

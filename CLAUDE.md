# CLAUDE.md

Guía de contexto para Claude Code al trabajar en este repositorio.

## Resumen del proyecto

Módem APRS (AX.25 sobre AFSK Bell 202, 1200 bps) sobre **ESP32** usando **ESP-IDF v5.x**. Estado actual (2026-04-30): **compila limpio, KISS TNC bidireccional operativo, TX verificado en hardware (datos decodificados por receptor externo), RX pendiente verificación con señal RF real**.

Se basa en una adaptación local de [LibAPRS-esp32-i2s](https://github.com/handiko/LibAPRS-esp32-i2s) (fork de LibAPRS de markqvist para AVR/Arduino), reescrita para usar:
- **DAC continuo** (`dac_continuous`, GPIO 25/DAC1) para la salida de audio.
- **ADC continuo DMA** (`adc_continuous`, ADC1_CH7 / GPIO 35) para la entrada de audio.
- Conmutación half-duplex de I2S0 entre ADC y DAC (ver trampas conocidas).

## Estructura

```
esp32-aprs-modem/
├── CMakeLists.txt              # Proyecto ESP-IDF de nivel superior
├── sdkconfig                   # Configuración IDF (target: esp32)
├── partitions.csv              # Tabla de particiones (NVS + OTA×2 + SPIFFS 704 KB)
├── main/
│   ├── CMakeLists.txt          # Registra .c/.cpp, REQUIRES y crea imagen SPIFFS
│   ├── config.h                # TNC_MODE, KISS_TRANSPORT, TCP port y GPIOs
│   ├── idf_component.yml       # Dependencia esp-dsp declarada pero sin uso activo (pendiente quitar)
│   ├── main.c                  # app_main — bifurca según TNC_MODE (KISS o APRS)
│   ├── kiss.h / kiss.c         # Framing KISS encode/decode, máquina de estados
│   ├── transport.h / .c        # Interfaz abstracta { init, write } para transportes
│   ├── transport_wifi.h / .c   # WiFi STA + servidor TCP KISS (transporte activo)
│   ├── aux_config.h / .c       # Carga/guarda config JSON desde SPIFFS
│   ├── aux_file_management.h/.c# Utilidades de sistema de ficheros SPIFFS
│   ├── spiffs_data/
│   │   └── config.json         # Config inicial (callsign, WiFi, AP, IP) flasheada en SPIFFS
│   └── LibAPRS-esp32-i2s/src/
│       ├── LibAPRS.{h,cpp}     # API APRS de alto nivel (APRS_init, set_raw_hook, send_raw_frame…)
│       ├── AFSK.{h,cpp}        # Modulador/demodulador AFSK, DAC TX, ADC RX
│       ├── AX25.{h,cpp}        # Codificación/decodificación AX.25 + raw_hook para KISS
│       ├── CRC-CCIT.{h,c}      # CRC-CCITT para tramas AX.25
│       ├── HDLC.h              # Flags HDLC (0x7E, 0x7F, AX25_ESC)
│       ├── FIFO.h              # Cola circular inline
│       ├── FakeArduino.{h,cpp} # Stub de `Serial`, `F()`, `_BV()`, cli/sei
│       ├── device.h            # Pines y parámetros (GPIO_PTT_OUT, muestreo, canales)
│       └── constants.h         # Macros m328p, REF_3V3 (herencia AVR — no se usan)
├── managed_components/espressif__esp-dsp/
└── build/                      # artefactos de IDF (no tocar)
```

## Configuración de hardware (según `device.h` y `AFSK.cpp`)

| Señal              | GPIO / Recurso                          |
|--------------------|-----------------------------------------|
| Audio TX (DAC1)    | GPIO 25 (`dac_continuous`, DAC_CHAN_0)  |
| Audio RX (ADC)     | GPIO 35 (ADC1_CH7, `adc_continuous`)   |
| PTT salida         | GPIO 26 (activo alto: 1 = TX, 0 = reposo) — restricción HW; también = DAC2 |
| Trigger audio      | GPIO 37 (sin uso activo)                |

**Parámetros de modulación:**
- Tasa de bits: 1200 bps (Bell 202: 1200 Hz = mark, 2200 Hz = space)
- Sample rate lógico: 9600 Hz (8 muestras/bit)
- Oversampling x5 → sample rate físico DAC/ADC = **48 000 Hz**
- Preamble/tail por defecto: 350 / 50 (unidades de ms·8/bitrate)

## Cómo compilar

```bash
# Desde la raíz del proyecto, con ESP-IDF exportado:
idf.py set-target esp32
idf.py build
idf.py -p <PUERTO> flash monitor
```

## Flujo de ejecución actual

```
app_main (main.c)
  ├── config_load()      ← lee /spiffs/config.json (callsign, WiFi, AP, IP)
  ├── [si TNC_MODE_KISS]
  │     ├── transport_init(&transport_wifi_ops)   ← WiFi STA + TCP server port 8001
  │     ├── kiss_init(on_kiss_frame)
  │     ├── APRS_init → AFSK_init → AFSK_hw_init
  │     │     ├── gpio_config PTT (GPIO26, salida, reposo bajo)
  │     │     ├── xQueueCreate(s_tx_queue, 4)  ← cola de tramas TX
  │     │     └── xTaskCreate(receive_audio_task, prio 10)
  │     │           └── [dentro de la tarea] adc_peripheral_start()
  │     ├── afsk_set_tx_fn(APRS_send_raw_frame) ← registra fn de TX en cola
  │     └── APRS_set_raw_hook(on_ax25_raw_frame)
  ├── [si TNC_MODE_APRS]
  │     ├── APRS_init + APRS_setCallsign + APRS_set_msg_hook
  │     └── xTaskCreate(processPacket, prio 5)
  └── xTaskCreate(audio_level_task, prio 3)   # barra de nivel + alarma LED por rango

receive_audio_task (bucle infinito):
  1) Si s_tx_queue tiene trama pendiente → despacha s_tx_fn(data,len) [TX]
  2) Si tx_mode==true → vTaskDelay(10ms) y repetir   [pausa durante TX]
  3) adc_continuous_read(timeout=20ms)               [DMA ring buffer]
  4) Para cada muestra: decimar x5 → adc_to_s8 → AFSK_adc_isr
  5) APRS_poll() cada 4 muestras lógicas (~2,4 ms)
  6) vTaskDelay(1)                                   [cede CPU a IDLE]

server_task (transport_wifi.c, cuando cliente TCP conectado):
  recibe bytes KISS → kiss_rx_byte() → on_kiss_frame → afsk_queue_tx_frame()
    ↑ NO llama APRS_send_raw_frame directamente (viola mutex ADC)

TX (despachado por receive_audio_task desde s_tx_queue):
  s_tx_fn(data, len) = APRS_send_raw_frame → ax25_sendRaw → AFSK_transmit
  switch_to_tx():
    tx_mode=true → adc_continuous_stop → adc_continuous_deinit
    → vTaskDelay(20ms) → dac_continuous_new_channels → dac_continuous_enable
    → prime silence (TX_SAMPLE_BUFLEN bytes × 0x80)
  transmit_audio_i2s() en bucle: genera muestras AFSK, padding 0x80, escribe
    TX_SAMPLE_BUFLEN bytes al DMA (timeout=2000ms) hasta afsk->sending==false
  finish_transmission():
    escribe TX_SAMPLE_BUFLEN bytes de silencio → gpio PTT=0
    switch_to_rx():
      dac_continuous_del_channels → adc_peripheral_start() → tx_mode=false
```

## Convenciones y trampas conocidas

- **I2S0 compartido entre DAC y ADC**: en ESP32 clásico, `dac_continuous` y `adc_continuous` usan internamente I2S0 para DMA. **No pueden estar activos al mismo tiempo**. La solución implementada es conmutación half-duplex: `switch_to_tx()` deinit el ADC antes de crear el DAC; `switch_to_rx()` elimina el DAC antes de recrear el ADC. No intentar activar ambos simultáneamente.

- **Timeout de `adc_continuous_read`**: se usa `pdMS_TO_TICKS(20)` (no `portMAX_DELAY`) para que `adc_continuous_stop` pueda desbloquear la tarea durante la transición a TX. Con `portMAX_DELAY` la tarea quedaría bloqueada indefinidamente si el ADC se para.

- **`vTaskDelay(1)` en `receive_audio_task`**: necesario aunque parezca un kludge. Con prioridad 10 y el ring buffer DMA siempre lleno (4 descriptores × 1024 muestras ≈ 85 ms de pool), la tarea nunca entraría en BLOCKED y `IDLE0` no correría → watchdog a los 5 s.

- **`adc_continuous_handle_cfg_t::flags`**: es un struct anidado. Inicializar con `= {}`, no con `= 0` (causa `-Werror=missing-field-initializers` en C++).

- **Orden de campos en inicializadores designados C++**: `adc_continuous_config_t` declara `pattern_num` y `adc_pattern` **antes** de `sample_freq_hz`, `conv_mode`, `format`. Los inicializadores designados en C++ deben respetar ese orden.

- **Mezcla C/C++**: `main.c` es C puro; la librería LibAPRS está en `.cpp`. Los símbolos exportados desde `.cpp` llevan `extern "C"`. El callback `aprs_msg_callback` se declara `extern "C"` desde `LibAPRS.cpp` y se define en `main.c` sin wrapper explícito.

- **`FakeArduino`**: emula `Serial.print/println` y `F(...)` para mantener el código heredado. Los métodos son stubs vacíos. `APRS_printSettings()` no produce ninguna salida aunque compile.

- **`sinSample`**: tabla de 128 valores expandida por simetría a 512 × OVERSAMPLING = 2560 puntos. `AFSK_dac_isr` genera muestras 8-bit unsigned (0–255) que `dac_continuous` acepta directamente.

- **Herencia AVR muerta**: `constants.h` (`m328p`, `REF_3V3`), `cli()/sei()` (nop en `FakeArduino`), macros `DAC_DDR`/`DAC_PORT` solo en comentarios. El `#ifdef TARGET_CPU == m328p` en `device.h` define ports AVR que no se usan.

- **`src.ino`** en `LibAPRS-esp32-i2s/src/`: archivo Arduino residual. No está en `CMakeLists.txt` pero clangd/VS Code lo indexan con errores.

- **`freeMemory()`** devuelve `10000000` constante — el chequeo de RAM en `main.c` nunca falla. Pendiente sustituir por `esp_get_free_heap_size()`.

- **`esp-dsp`** sigue declarado en `idf_component.yml` aunque los buffers FFT fueron eliminados. Alarga el build innecesariamente.

- **SPIFFS y `config.json`**: `aux_config.c` lee `/spiffs/config.json` en el arranque. `transport_wifi.c` ya usa `wifi.ssid`/`wifi.password`/`wifi.connect_timeout_s` y fallback AP (`ap.*`) desde JSON. El fichero se flashea automáticamente con `idf.py build` gracias a `spiffs_create_partition_image` en `CMakeLists.txt`. Para modificar la config sin recompilar: editar `main/spiffs_data/config.json` y volver a flashear.

- **`config_load()` devuelve copia**: `aux_config.c` devuelve `cJSON_Duplicate(root, 1)` — el llamador es responsable de liberar el objeto con `config_free_json()`. No usar el puntero después de liberar.

- **Cola TX entre tareas (`s_tx_queue`)**: en modo KISS TNC, `server_task` **no puede** llamar `APRS_send_raw_frame` directamente porque internamente invoca `adc_continuous_stop`, que debe ejecutarse desde la misma tarea FreeRTOS que llamó a `adc_continuous_start` (mutex interno de ESP-IDF). Solución: `QueueHandle_t s_tx_queue` de capacidad 4 × `afsk_tx_frame_t`. `server_task` encola con `afsk_queue_tx_frame()` (no bloqueante); `receive_audio_task` despacha al inicio de cada iteración. **Es obligatorio llamar `afsk_set_tx_fn(APRS_send_raw_frame)` en `app_main` antes de `APRS_set_raw_hook`.**

- **`TX_SAMPLE_BUFLEN=2048` — no reducir**: el descriptor DMA del DAC tiene `buf_size=2048` bytes (≈42 ms a 48 kHz). Con `desc_num=8`, el pool total es ≈336 ms. Las tareas WiFi (prioridad 23) pueden preemptar `receive_audio_task` (prioridad 10) durante decenas de ms. Si el descriptor se consume antes de que la tarea recargue el siguiente, el semáforo `s_dac_wait_to_load_dma_data` expira → `dac_continuous_write` cuelga. Con `TX_SAMPLE_BUFLEN<buf_size` cada escritura llena menos de un descriptor y el margen desaparece.

- **Siempre escribir el buffer completo al DMA**: cuando `AFSK_dac_isr` establece `sending=false` a mitad del buffer, `transmit_audio_i2s` rellena el resto con `0x80` (nivel DC = silencio) y siempre escribe `TX_SAMPLE_BUFLEN` bytes. Escribir un buffer parcial agota el descriptor en <1 ms, el DMA se detiene, y las escrituras de silencio posteriores no consiguen cargar porque el ISR ya no dispara → timeout × N = bloqueo prolongado.

- **`dac_continuous_write` con timeout finito (2000 ms)**: **nunca usar `-1` (portMAX_DELAY)** para este write. Si el DAC se detiene inesperadamente, el timeout permite abortar con `afsk->sending = false` y volver al modo RX. Con timeout infinito, el firmware queda bloqueado con el PTT pulsado indefinidamente.

- **GPIO 26 = PTT = DAC2 (conflicto de recurso)**: GPIO 26 es simultáneamente el pin de PTT y DAC2 del ESP32. `DAC_CHANNEL_MODE_SIMUL` en `switch_to_tx()` puede reconfigurarlo como salida analógica DAC, dejando el GPIO en modo analógico y sacando el PTT del control digital. Mitigación aplicada: llamar `gpio_set_direction(GPIO_PTT_OUT, GPIO_MODE_OUTPUT)` después de `dac_continuous_new_channels()` en `switch_to_tx()` para restaurar el modo digital.

- **Polaridad PTT**: El hardware usa PTT **activo alto** (1 = transmitiendo, 0 = reposo). El control de PTT está centralizado en `AFSK.cpp`. **No invertir** las llamadas a `gpio_set_level`.

## Al editar este proyecto

- **No reintroducir** `adc_oneshot`, PDM, ni el esquema "grabar→procesar": los problemas que causaban están documentados en `report.md` secciones 1.1–1.6.
- **No cambiar GPIO_PTT_OUT a otro pin** — GPIO 26 es restricción de hardware del diseño físico. Definido en `config.h`.
- **No activar DAC y ADC simultáneamente** — ver trampa de I2S0 arriba.
- **Evitar** tocar `AX25.cpp`, `AX25.h`, `HDLC.h`, `CRC-CCIT.c`: código maduro del LibAPRS original que funciona.
- **Al cambiar GPIOs**, actualizar `device.h` y marcar el comentario anterior como obsoleto.
- **Comentarios en español** en código heredado se conservan; los nuevos pueden ir en español o inglés.

## Comandos útiles

```bash
idf.py menuconfig          # ajustar flash size, partición, PSRAM, etc.
idf.py size-components     # auditar consumo RAM/Flash
idf.py monitor             # ver trazas; salir con Ctrl-]
```

## Estado de los problemas conocidos

Ver [report.md](report.md) para el detalle completo y [PROGRESS.md](PROGRESS.md) para el seguimiento.

Resumen rápido:
- Sección 1 (bloqueantes): todos resueltos en código ✅
- Sección 2.5 + 2.10 + 2.11 + 2.12 + 2.13: resueltos en código ✅
- TX verificado en hardware: datos recibidos y decodificados por receptor externo ✅
- Sección 2.2, 2.3, 2.6, 2.7, 2.8: pendientes ⬜
- Verificación RX en hardware real con señal RF: pendiente ⚠️

## Referencias externas

- LibAPRS original (AVR): https://github.com/markqvist/LibAPRS
- LibAPRS-esp32-i2s upstream: https://github.com/handiko/LibAPRS-esp32-i2s
- Especificación AX.25 v2.2: https://www.tapr.org/pdf/AX25.2.2.pdf
- esp-dsp: https://github.com/espressif/esp-dsp

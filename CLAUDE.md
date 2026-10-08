# CLAUDE.md

Guía de contexto para Claude Code al trabajar en este repositorio.

## Resumen del proyecto

Módem APRS (AX.25 sobre AFSK Bell 202, 1200 bps) sobre **ESP32** usando **ESP-IDF v6.1**. Estado actual (2026-10-08): **compila limpio (binario ~928 KB, ~46 % libre de la partición de app de 1,625 MB), KISS TNC bidireccional operativo, TX verificado en hardware, UI web (log APRS, chat con ACK, audio, SSTV, OTA, TUNE…), comandos remotos vía APRS, gateway IP RFC 1226 (TUN) verificado, IL2P con Reed-Solomon, digipeater WIDEn-N, GPS + baliza periódica, display SSD1306, Listen-Before-Talk. Pendiente: verificación de RX con señal RF real y de los módulos con "HW pendiente" (repetidor, IL2P, display)**.

Se basa en una adaptación de [LibAPRS-esp32-i2s](https://github.com/handiko/LibAPRS-esp32-i2s) (fork de LibAPRS de markqvist para AVR/Arduino). El fork propio vive en **https://github.com/luisesn/LibAPRS-esp32-i2s** y está clonado en `main/LibAPRS-esp32-i2s/` como **repo git anidado** (ver "Librería LibAPRS" más abajo). Reescrita para usar:
- **DAC continuo** (`dac_continuous`, GPIO 25/DAC1) para la salida de audio.
- **ADC continuo DMA** (`adc_continuous`, ADC1_CH7 / GPIO 35) para la entrada de audio.
- Conmutación half-duplex de I2S0 entre ADC y DAC (ver trampas conocidas).

## Estructura

```
esp32-aprs-modem/
├── CMakeLists.txt              # Proyecto ESP-IDF de nivel superior
├── sdkconfig                   # Config IDF local (en .gitignore; se regenera desde sdkconfig.defaults)
├── sdkconfig.defaults          # Config reproducible: flash 4 MB, partitions.csv, WiFi/lwIP, WS, IP forward
├── partitions.csv              # NVS + otadata + OTA×2 (1,625 MB c/u) + SPIFFS 704 KB
├── espota.py                   # Cliente OTA por WiFi (protocolo espota, puerto 3232)
├── README.md / PROGRESS.md     # Documentación de usuario / bitácora
├── .claude/commands/           # Skills: /build, /flash-spiffs, /trap-check
├── doc/                        # Notas de diseño y backlog (en .gitignore: no versionado)
├── main/
│   ├── CMakeLists.txt          # Fuentes, REQUIRES, CUSTOM_FRAME_SIZE=600, imagen SPIFFS
│   ├── idf_component.yml       # Dependencia: espressif/cjson (esp-dsp ya eliminado)
│   ├── config.h                # TNC_MODE, KISS_TRANSPORT, KISS_TCP_PORT, GPIOs, REPEATER_ENABLED
│   ├── main.c                  # app_main, on_ax25_raw_frame (dedup, ACK, fan-out), smart_tx_frame
│   ├── kiss.{h,c}              # Framing KISS encode/decode
│   ├── transport.{h,c}         # Interfaz abstracta { init, write }
│   ├── transport_wifi.{h,c}    # WiFi STA (varias redes) + fallback AP + reconexión + servidor TCP KISS
│   ├── audio_stream.{h,c}      # HTTP :80, WebSocket /ws, IMA ADPCM, WAV TCP :8080, ~30 endpoints REST
│   ├── ax25ip.{h,c}            # Gateway IP RFC 1226 / modo TUN (tncattach), PID=0xCC
│   ├── aux_config.{h,c}        # Config JSON en /spiffs/config.json (config_get devuelve copia)
│   ├── aux_file_management.{h,c} # Montaje SPIFFS
│   ├── remote_cmd.{h,c}        # Comandos remotos vía APRS (tx sstv/morse/aprs position, rx sstv list)
│   ├── digipeater.{h,c}        # Digipeater WIDEn-N con dedup FNV-1a (TTL 30 s)
│   ├── morse.{h,c}             # Baliza CW periódica / one-shot
│   ├── sstv.{h,c}              # TX SSTV (Martin M1/M2, Scottie S1, Robot 36/72), JPEG vía TJpgDec en ROM
│   ├── gps.{h,c}               # NMEA por UART2 + tarea de baliza de posición (gps.use_for_beacon)
│   ├── display.{h,c}           # SSD1306 128×64 I2C sin librería externa
│   ├── rf_console.{h,c}        # Consola de texto (TCP y/o UDP, puerto 23) sobre la interfaz RF de ax25ip
│   ├── ota.{h,c} / ota_net.{h,c} # OTA por HTTP (/api/ota/upload) y por espota.py (:3232)
│   ├── delay_tune.{h,c}        # Autoajuste de post_rx_tx_delay_ms por barrido ICMP
│   ├── rx_stats.{h,c}          # Contadores por demodulador (v1/v2): CRC ok/err, flags, overflows
│   ├── il2p.{h,c} / rs_codec.{h,c} # IL2P (FEC Reed-Solomon + scrambler); RS(255,239) y cabecera sobre GF(2⁸)
│   ├── squelch_sf.{h,c}        # Squelch HFNE (Goertzel 3150–4350 Hz); canal ocupado para LBT/monitor
│   ├── repeater.{h,c}          # Repetidor de voz analógico (DESHABILITADO: REPEATER_ENABLED 0)
│   ├── spiffs_data/
│   │   ├── config.json         # Config activa flasheada en SPIFFS (¡contiene credenciales!)
│   │   ├── config.json.normal  # Copia de referencia local (en .gitignore)
│   │   ├── index.html          # UI web completa (~80 KB, ~1500 líneas, un solo fichero)
│   │   └── sstv/               # Imágenes JPEG de usuario (en .gitignore)
│   └── LibAPRS-esp32-i2s/      # ← REPO GIT ANIDADO (luisesn/LibAPRS-esp32-i2s), no submódulo
│       └── src/
│           ├── LibAPRS.{h,cpp} # API de alto nivel (init, queue_msg→int, queue_ack, getCallsign…)
│           ├── AFSK.{h,cpp}    # Mod/demod AFSK (v1 y v2), DAC TX, ADC RX, cola TX, LBT, inhibición post-RX
│           ├── AX25.{h,cpp}    # AX.25 + raw_hook
│           ├── CRC-CCIT.{h,c}, HDLC.h
│           ├── FIFO.h          # Cola circular; variantes _locked (portMUX) son static inline
│           ├── FakeArduino.{h,cpp}, device.h, constants.h  # herencia Arduino/AVR (mayormente muerta)
├── managed_components/         # espressif__cjson (en .gitignore)
└── build/                      # artefactos de IDF (no tocar)
```

## Configuración de hardware (según `device.h` y `AFSK.cpp`)

| Señal              | GPIO / Recurso                          |
|--------------------|-----------------------------------------|
| Audio TX (DAC1)    | GPIO 25 (`dac_continuous`, DAC_CHAN_0)  |
| Audio RX (ADC)     | GPIO 35 (ADC1_CH7, `adc_continuous`)   |
| PTT salida         | GPIO 26 (activo alto: 1 = TX, 0 = reposo) — restricción HW; también = DAC2 |
| Trigger audio      | GPIO 37 (sin uso activo)                |
| LED RX / LED aviso | GPIO 33 (paquete decodificado) / GPIO 23 (nivel de audio fuera de rango) |
| I2C (SSD1306)      | SDA = GPIO 13, SCL = GPIO 19            |
| GPS (UART2)        | RX = GPIO 4, TX = GPIO 16 (según `config.h`) |

**Parámetros de modulación:**
- Tasa de bits: 1200 bps (Bell 202: 1200 Hz = mark, 2200 Hz = space)
- Sample rate lógico: 9600 Hz (8 muestras/bit)
- Oversampling x5 → sample rate físico DAC/ADC = **48 000 Hz**
- Preamble/tail por defecto: 350 / 50 (unidades de ms·8/bitrate)

## Cómo compilar

> **Antes de nada**: la librería `main/LibAPRS-esp32-i2s/` es un repo anidado que el repo principal **no** referencia (sin `.gitmodules`); en un clon limpio hay que traerla aparte (`git clone https://github.com/luisesn/LibAPRS-esp32-i2s main/LibAPRS-esp32-i2s`). La configuración de IDF sale de `sdkconfig.defaults` (verificado: `idf.py reconfigure` desde cero regenera un `sdkconfig` idéntico al local).

```bash
# Desde la raíz del proyecto, con ESP-IDF exportado:
idf.py set-target esp32
idf.py reconfigure   # genera build/config/sdkconfig.cmake antes del primer build
ninja -C build       # o: idf.py build
idf.py -p <PUERTO> flash monitor
```

> **Trampa de primer build**: con IDF 6.1, `idf.py build` lanza ninja demasiado pronto y
> ninja dispara un re-run de CMake que no encuentra `build/config/sdkconfig.cmake` (aún no
> generado) → `FAILED: build.ninja`. Solución: `idf.py reconfigure` primero, luego `ninja -C build`.

## Flujo de ejecución actual

```
app_main (main.c)
  ├── file_management_init() (SPIFFS) + config_load()   ← /spiffs/config.json
  ├── [si TNC_MODE_KISS]
  │     ├── gps_init(cfg) + display_init(cfg)           ← antes del WiFi (la pantalla muestra el estado)
  │     ├── transport_init(&transport_wifi_ops)         ← WiFi STA (lista de redes) / fallback AP,
  │     │                                                  tarea de reconexión, servidor TCP KISS :8001
  │     ├── ax25ip_init(cfg)                            ← gateway IP (si ip.enabled)
  │     ├── kiss_init(on_kiss_frame)
  │     ├── APRS_init → AFSK_init → AFSK_hw_init
  │     │     ├── gpio_config PTT (GPIO26, salida, reposo bajo)
  │     │     ├── xQueueCreate(s_tx_queue, 4)           ← cola de tramas TX
  │     │     ├── xTaskCreate(aprs_poll_task, prio 9)   ← APRS_poll() + il2p_poll() cada tick
  │     │     └── xTaskCreate(receive_audio_task, prio 10, stack 8192)
  │     │           └── [dentro de la tarea] adc_peripheral_start()
  │     ├── APRS_setCallsign() + afsk_set_post_rx_tx_delay_ms()   ← aprs.* de config.json
  │     ├── afsk_set_tx_fn(APRS_send_raw_frame)  ← o smart_tx_frame si il2p.enabled
  │     ├── APRS_set_raw_hook(on_ax25_raw_frame) + il2p_init(en, on_ax25_raw_frame)
  │     ├── digi_init / morse_init / remote_cmd_init / rf_console_init
  │     ├── audio_stream_init()  ← HTTP :80 + WS /ws + WAV :8080 (+ sstv/ota/tune/squelch/repeater REST)
  │     ├── afsk_set_audio_hook(audio_sample_hook)      ← ADPCM, repetidor, squelch HFNE
  │     ├── sstv_init / ota_init / ota_net_init / delay_tune_init / repeater_init / squelch_sf_init
  │     └── afsk_set_channel_busy_fn(channel_is_busy) + afsk_set_dispatch_hook(project_dispatch_hook)
  ├── [si TNC_MODE_APRS]
  │     ├── APRS_init + APRS_setCallsign + APRS_set_msg_hook
  │     └── xTaskCreate(processPacket, prio 5)
  └── xTaskCreate(audio_level_task, prio 3)   # barra de nivel + alarma LED por rango

receive_audio_task (bucle infinito, prio 10):
  1) Si s_tx_queue tiene trama pendiente Y no hay inhibición → despacha s_tx_fn(data,len) [TX]
       · inhibición post-RX: no transmitir hasta post_rx_tx_delay_ms tras la última trama recibida
       · LBT (tx.lbt_enabled): difiere si channel_is_busy(), hasta tx.lbt_max_wait_ms (10 s)
  2) project_dispatch_hook(): morse_check_and_dispatch / sstv_dispatch_if_pending / repeater_dispatch_if_pending
  3) Si tx_mode==true → vTaskDelay(10ms) y repetir   [pausa durante TX]
  4) adc_continuous_read(timeout=20ms)               [DMA ring buffer]
  5) Para cada muestra: decimar x5 → adc_to_s8 → AFSK_adc_isr (demod v1 y, si dual_modem, v2)
     └── audio_hook → audio_stream_q (ADPCM) + repeater_audio_hook + squelch_sf_push_sample
  6) vTaskDelay(1)                                   [cede CPU a IDLE]

aprs_poll_task (AFSK.cpp, prio 9): APRS_poll() + il2p_poll() cada tick, salvo s_rx_paused.
  Aquí se ejecuta el raw_hook (on_ax25_raw_frame): mantener el callback rápido.

audio_stream_task (audio_stream.c, prio 3):
  Lee audio_stream_q → codifica IMA ADPCM (1017 muestras → 512 B)
  → envía frame binario por WebSocket a todos los clientes /ws conectados
  wav_server_task (prio 3): WAV TCP en port 8080 para ffplay/VLC

HTTP server (audio_stream.c, port 80): ~30 endpoints, ver tabla en README.md
  (aprs/send, aprs/beacon, me, config, log, rx/stats, reboot, spiffs/upload, morse, sstv/*,
   ota/upload, repeater/*, squelch/*, tune/*)

on_ax25_raw_frame (main.c, llamado por el raw_hook de AX25 y de IL2P):
  ├── rx_frame_is_duplicate()        → descarta la 2ª copia (doble módem), ventana 300 ms
  ├── afsk_notify_rx_frame()         → reinicia la ventana de inhibición post-RX
  ├── kiss_send_frame()              → host KISS TCP
  ├── try_auto_ack()                 → si va dirigida a nosotros con {NNN}: APRS_queue_ack()
  │     (el ACK se encola ANTES que la respuesta de remote_cmd_handle) + WS {type:ack_sent}
  ├── digi_process_frame()           → digipeater WIDEn-N (encola por afsk_queue_tx_frame)
  ├── ax25ip_rx_frame()              → inyecta en lwIP si PID=0xCC
  └── audio_stream_ws_send_text()    → JSON {type:aprs,...} (+ {type:digipeated,...})

server_task (transport_wifi.c, cliente TCP KISS conectado; TCP keepalive + escritura con buffer):
  recibe bytes KISS → kiss_rx_byte() → on_kiss_frame → afsk_queue_tx_frame()
    (bloquea hasta KISS_TX_ENQUEUE_TIMEOUT_MS = 15 s si la cola está llena: contrapresión)
    ↑ NO llama APRS_send_raw_frame directamente (viola mutex ADC)

TX (despachado por receive_audio_task desde s_tx_queue):
  s_tx_fn(data, len) = APRS_send_raw_frame (o smart_tx_frame: PID 0xCC → IL2P, resto AX.25)
    → ax25_sendRaw → AFSK_transmit
  switch_to_tx():
    tx_mode=true → adc_continuous_stop → adc_continuous_deinit
    → vTaskDelay(20ms) → dac_continuous_new_channels → dac_continuous_enable
    → gpio_set_direction(GPIO_PTT_OUT, OUTPUT)  ← restaura modo digital tras DAC_SIMUL
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

- **`sdkconfig` vs `sdkconfig.defaults`**: `sdkconfig` está en `.gitignore`; la configuración reproducible vive en `sdkconfig.defaults` (flash 4 MB, `partitions.csv`, WS + callback de handshake, `LWIP_IP_FORWARD=y` necesario para `ax25ip.c`, 16 sockets, WiFi 8/8 buffers, WPA3 e IPv6 desactivados). Si cambias un ajuste con `menuconfig`, trasládalo a `sdkconfig.defaults` (`idf.py save-defconfig` muestra las diferencias, pero sobrescribe el fichero: haz copia). Tras `idf.py fullclean` o borrar `sdkconfig`, ejecuta `idf.py reconfigure`. Optimización `-Og` = valor por defecto de IDF (no es una elección local).

- **`pbuf_length(p)` no existe en lwIP**: la longitud total de una cadena pbuf es `p->tot_len`. Usar `pbuf_clen(p)` (cuenta fragmentos) o `p->tot_len` (bytes totales).

- **`idf.py reconfigure` antes del primer ninja**: con IDF 6.1, `idf.py build` en un build directory vacío falla porque ninja dispara un re-run de CMake que intenta incluir `build/config/sdkconfig.cmake` antes de que CMake lo haya generado. Workaround: `idf.py reconfigure && ninja -C build`.

- **SPIFFS y `config.json`**: `aux_config.c` lee `/spiffs/config.json` en el arranque. `transport_wifi.c` usa la lista `wifi[]` (ssid/password/connect_timeout_s, se prueban en orden), reconecta automáticamente y cae a AP (`ap.*`) si falla. El fichero se flashea con `idf.py build` gracias a `spiffs_create_partition_image` (también `/flash-spiffs`). **`main/spiffs_data/config.json` está seguido por git** (a pesar de aparecer en `.gitignore`, que sólo afecta a ficheros no seguidos) y contiene credenciales WiFi reales también en commits anteriores: no hacer commit de credenciales; `git rm --cached` + plantilla sin secretos pendiente. `spiffs_data/sstv/*` sí está ignorado.

- **`doc/` está en `.gitignore`** (`doc/*`): las notas de diseño y backlog (`doc/pending`, `doc/obsolete`, `doc/to-migrate`, `doc/comandos_serial.md`…) no se versionan (sólo hay 2 imágenes seguidas). `report.md` ya no está en la raíz: está en `doc/obsolete/report.md`.

- **Librería `main/LibAPRS-esp32-i2s/` = repo git anidado** (remoto `luisesn/LibAPRS-esp32-i2s`, rama `master`). El repo principal no la rastrea (no hay `.gitmodules`; `git status` la muestra como `??`). Los cambios en `AFSK.*`, `AX25.cpp`, `FIFO.h`, `LibAPRS.cpp` se commitean **dentro** de ese repo (`git -C main/LibAPRS-esp32-i2s ...`). Un clon limpio del proyecto no la trae. Convertirla en submódulo está recomendado (ver `informe_mejoras.md` §2.bis): primero commit + push en la librería, luego `git submodule add`.

- **Listen-Before-Talk e inhibición post-RX** (`AFSK.cpp`, bucle de despacho TX): (a) el TX espera `aprs.post_rx_tx_delay_ms` desde la última trama recibida (`afsk_notify_rx_frame()`); (b) si `tx.lbt_enabled` (por defecto **false**), difiere mientras `channel_is_busy()`, hasta `tx.lbt_max_wait_ms` (10 s). `channel_is_busy()` = `squelch_sf_is_active() && squelch_hfne_is_open()`: **sólo funciona si el squelch HFNE está activo** (modo monitor o repetidor); si no, el canal se considera siempre libre (el valor del squelch estaría congelado). Activar LBT sin activar el monitor no hace nada.

- **Baliza GPS**: `gps.use_for_beacon` + `gps.beacon_period_s` (por defecto 600 s en código; el `config.json` actual usa 60 s). La tarea `gps_beacon` vive en `gps.c`. Para balizas manuales: `POST /api/aprs/beacon` o `tx,aprs,position` por APRS.

- **Consola RF** (`rf_console.c`): escucha en el puerto `console.port` (23) sólo en la interfaz RF de `ax25ip`; `console.tcp` y `console.udp` se activan por separado (por defecto TCP sí, UDP no). Comandos: `help`, `status`, `config`, `quit`. Sin autenticación.

- **`aprs_poll_task` y el `raw_hook`**: `APRS_poll()` ya no corre en `receive_audio_task` sino en `aprs_poll_task` (prio 9), por lo que `on_ax25_raw_frame` se ejecuta en esa tarea. No bloquear en el hook (WS lento, mutex) o se llenan los FIFOs de RX. `FIFO.h` tiene variantes `_locked` (portMUX); el resto de `fifo_*` sin lock se usa en rutas de una sola tarea/ISR — auditar antes de tocar.

- **IL2P**: `il2p.enabled` (por defecto false). Con IL2P activo `smart_tx_frame` enruta PID=0xCC (IP) por IL2P y el resto por AX.25; la RX acepta ambos en paralelo. `rs_codec.c` usa GF(2⁸) tanto para RS(255,239) del payload como para la cabecera (2 raíces); no hay GF(2⁴).

- **Repetidor deshabilitado**: `REPEATER_ENABLED 0` en `config.h` (reservar ~15 KB contiguos de heap, pausa RX y detiene el servidor WAV). Con 0, ni `config.json` ni `/api/repeater/enable` pueden activarlo. Hay presión de heap: vigilar buffers `static` grandes y stacks de tareas.

- **Robustez (informe §3.5)**: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` (requiere reflashear bootloader por cable una vez); `ota_validate_task` (main.c) confirma la imagen a los 30 s. OTA HTTP acepta cabecera opcional `X-SHA256`. `save_config` escribe `config.tmp` y lo renombra (recuperación en `config_load_from_file`). Reconexión WiFi con backoff (5 rondas) antes de caer al AP. Reloj: SNTP al obtener IP + hora GPS (`gps_sync_system_time`). `GET /api/sys` da heap y stacks. Dedup compartido en `dedup.[ch]`; códec en `adpcm.c`.

- **`config_load()` devuelve copia**: `aux_config.c` devuelve `cJSON_Duplicate(root, 1)` — el llamador es responsable de liberar el objeto con `config_free_json()`. No usar el puntero después de liberar.

- **Cola TX entre tareas (`s_tx_queue`)**: en modo KISS TNC, `server_task` **no puede** llamar `APRS_send_raw_frame` directamente porque internamente invoca `adc_continuous_stop`, que debe ejecutarse desde la misma tarea FreeRTOS que llamó a `adc_continuous_start` (mutex interno de ESP-IDF). Solución: `QueueHandle_t s_tx_queue` de capacidad 4 × `afsk_tx_frame_t`. `server_task` encola con `afsk_queue_tx_frame()` (no bloqueante); `receive_audio_task` despacha al inicio de cada iteración. **Es obligatorio llamar `afsk_set_tx_fn(APRS_send_raw_frame)` en `app_main` antes de `APRS_set_raw_hook`.**

- **`TX_SAMPLE_BUFLEN=2048` — no reducir**: el descriptor DMA del DAC tiene `buf_size=2048` bytes (≈42 ms a 48 kHz). Con `desc_num=8`, el pool total es ≈336 ms. Las tareas WiFi (prioridad 23) pueden preemptar `receive_audio_task` (prioridad 10) durante decenas de ms. Si el descriptor se consume antes de que la tarea recargue el siguiente, el semáforo `s_dac_wait_to_load_dma_data` expira → `dac_continuous_write` cuelga. Con `TX_SAMPLE_BUFLEN<buf_size` cada escritura llena menos de un descriptor y el margen desaparece.

- **Siempre escribir el buffer completo al DMA**: cuando `AFSK_dac_isr` establece `sending=false` a mitad del buffer, `transmit_audio_i2s` rellena el resto con `0x80` (nivel DC = silencio) y siempre escribe `TX_SAMPLE_BUFLEN` bytes. Escribir un buffer parcial agota el descriptor en <1 ms, el DMA se detiene, y las escrituras de silencio posteriores no consiguen cargar porque el ISR ya no dispara → timeout × N = bloqueo prolongado.

- **`dac_continuous_write` con timeout finito (2000 ms)**: **nunca usar `-1` (portMAX_DELAY)** para este write. Si el DAC se detiene inesperadamente, el timeout permite abortar con `afsk->sending = false` y volver al modo RX. Con timeout infinito, el firmware queda bloqueado con el PTT pulsado indefinidamente.

- **GPIO 26 = PTT = DAC2 (conflicto de recurso)**: GPIO 26 es simultáneamente el pin de PTT y DAC2 del ESP32. `DAC_CHANNEL_MODE_SIMUL` en `switch_to_tx()` puede reconfigurarlo como salida analógica DAC, dejando el GPIO en modo analógico y sacando el PTT del control digital. Mitigación aplicada: llamar `gpio_set_direction(GPIO_PTT_OUT, GPIO_MODE_OUTPUT)` después de `dac_continuous_new_channels()` en `switch_to_tx()` para restaurar el modo digital.

- **Demodulador v1 desactivado** (`AFSK_RX_V1_ENABLED 0` en `AFSK.h` de la librería): sólo RX v2 (`AFSK_modem_v2` se crea siempre; `rx.dual_modem_enabled`/`rx.active_modem` se ignoran). `AFSK_modem` sigue siendo el modem de TX. `rx_stats_v1` queda a 0; `TNC_MODE_APRS` (`AX25.hook`) no recibe mensajes parseados. Poner a 1 para restaurar el doble módem (entonces aplica lo siguiente).

- **Doble módem RX entrega cada trama dos veces**: con `rx.active_modem = "best"` (valor por defecto), v1 (`ax25_poll`) y v2 (`aprs_poll_v2`) demodulan el mismo audio y llaman por separado a `s_app_raw_hook` con la misma trama, separados por unos ms. Sin filtro eso significa doble inyección en lwIP (`ax25ip`), doble auto-ACK y doble digipetición — en un ping por RF se ve como `DUP!` y dos ciclos TX. El dedup está **en `on_ax25_raw_frame()`** (`rx_frame_is_duplicate()`, ventana `RX_DUP_WINDOW_MS` = 300 ms, comparación byte a byte), que es el punto donde convergen todos los consumidores. No añadir un segundo filtro en LibAPRS (rompería `rx_stats_v1`/`rx_stats_v2`) ni en `ax25ip.c`/`digipeater.c`.

- **Polaridad PTT**: El hardware usa PTT **activo alto** (1 = transmitiendo, 0 = reposo). El control de PTT está centralizado en `AFSK.cpp`. **No invertir** las llamadas a `gpio_set_level`.

## Al editar este proyecto

- **No reintroducir** `adc_oneshot`, PDM, ni el esquema "grabar→procesar": los problemas que causaban están documentados en `doc/obsolete/report.md` secciones 1.1–1.6.
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

## Estado del proyecto

Ver [README.md](README.md) (uso, API, configuración), [PROGRESS.md](PROGRESS.md) (bitácora) y [informe_mejoras.md](informe_mejoras.md) (análisis de mejoras, con anexos de seguridad y pruebas). El informe técnico original está en `doc/obsolete/report.md`.

Resumen rápido (2026-10-08):
- Bloqueantes de la primera auditoría: todos resueltos en código ✅
- KISS TNC bidireccional (WiFi TCP :8001, keepalive, escritura con buffer, contrapresión) ✅
- TX verificado en hardware: datos decodificados por receptor externo ✅
- UI web (`index.html`): log APRS (incluye tráfico de terceros), CHAT con ACK, audio ADPCM, SSTV, OTA, TUNE, CONFIG, STATS ✅
- Comandos remotos vía APRS (`remote_cmd.c`), sin autenticación ✅
- Gateway IP RFC 1226 / TUN (`ax25ip.c`) verificado en hardware ✅
- Dedup de tramas RX por doble módem en `on_ax25_raw_frame()` ✅
- Listen-Before-Talk + inhibición post-RX + autoajuste `post_rx_tx_delay_ms` (TUNE) ✅ (LBT sólo efectivo con squelch HFNE activo)
- GPS + baliza de posición periódica; display SSD1306; digipeater WIDEn-N; baliza CW ✅
- IL2P con Reed-Solomon ✅ (interoperabilidad con Dire Wolf sin verificar)
- OTA por HTTP y por `espota.py`; WiFi multi-red con fallback AP ✅
- Consola RF (TCP/UDP) ✅
- `sdkconfig.defaults` completo (reproducible) ✅
- Build binario: ~928 KB (≈46 % libre de la partición de app) con `idf.py reconfigure && ninja -C build` ✅
- Verificación RX con señal RF real: pendiente ⚠️
- Repetidor de voz (deshabilitado por `REPEATER_ENABLED 0`), display, IL2P: verificación HW pendiente ⚠️
- Librería `LibAPRS-esp32-i2s` como repo anidado sin enlazar (5 ficheros con cambios sin commit) ⬜
- `config.json` con credenciales seguido por git ⬜
- Sin tests ni CI ⬜; FIFOs no `_locked` en rutas multi-tarea sin auditar ⬜

## Referencias externas

- LibAPRS original (AVR): https://github.com/markqvist/LibAPRS
- LibAPRS-esp32-i2s upstream: https://github.com/handiko/LibAPRS-esp32-i2s
- Especificación AX.25 v2.2: https://www.tapr.org/pdf/AX25.2.2.pdf
- Fork propio de la librería: https://github.com/luisesn/LibAPRS-esp32-i2s

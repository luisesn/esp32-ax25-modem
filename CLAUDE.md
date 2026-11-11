# CLAUDE.md

Guía de contexto para Claude Code al trabajar en este repositorio.

## Resumen del proyecto

Módem APRS (AX.25 sobre AFSK Bell 202, 1200 bps) sobre **ESP32** usando **ESP-IDF v6.1**. Estado actual (2026-11-11, firmware v1.13.2): **compila limpio (binario ~1,01 MB, ~41 % libre de la partición de app de 1,625 MB), KISS TNC bidireccional operativo, TX verificado en hardware, UI web (log APRS, chat con ACK, audio, SSTV, OTA, TUNE…), comandos remotos vía APRS, gateway IP RFC 1226 (TUN) verificado, IL2P estándar (v1.8.0, validado en host con los vectores de la spec; sin probar en el aire), digipeater WIDEn-N, GPS + baliza periódica, display SSD1306, Listen-Before-Talk. Pendiente: verificación de RX con señal RF real y de los módulos con "HW pendiente" (repetidor, IL2P, display)**.

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
├── version.txt                 # Versión del firmware (PROJECT_VER); ver "Al editar este proyecto"
├── informe_bugs_funcionalidades.md / informe_modem.md  # Análisis de bugs/funcionalidades y del demodulador (2026-11)
├── test/                       # Banco de pruebas en host (cmake -S test -B build-test; ctest): unit + regresión del demodulador; ver test/README.md
├── server/                     # Servidor de desarrollo en Python (consola serie, flasheo y OTA desde el navegador); ver server/README.md
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
│   ├── audio_stream.{h,c}      # HTTP :80 (servidor), WebSocket /ws, IMA ADPCM, WAV TCP :8080, /api/log, /api/rx/stats
│   ├── api_rest.c              # API REST (index, me, sys, aprs, config, morse, reboot, spiffs, repeater/squelch) + sondeos 404; `api_rest_register()`
│   ├── ax25ip.{h,c}            # Gateway IP RFC 1226 / modo TUN (tncattach), PID=0xCC
│   ├── aux_config.{h,c}        # Config JSON en /spiffs/config.json (config_get devuelve copia)
│   ├── aux_file_management.{h,c} # Montaje SPIFFS
│   ├── remote_cmd.{h,c}        # Comandos remotos vía APRS (tx sstv/morse/aprs position, rx sstv list)
│   ├── digipeater.{h,c}        # Digipeater WIDEn-N con dedup FNV-1a (TTL 30 s)
│   ├── morse.{h,c}             # Baliza CW periódica / one-shot
│   ├── sstv.{h,c}              # TX SSTV (Martin M1/M2, Scottie S1, Robot 36/72), JPEG vía TJpgDec en ROM
│   ├── gps.{h,c}               # NMEA por UART2 + tarea de baliza de posición (gps.use_for_beacon)
│   ├── display.{h,c}           # SSD1306 128×64 I2C sin librería externa
│   ├── heard.{h,c}             # Tabla de las últimas 20 estaciones oídas (/api/heard, consola RF)
│   ├── rf_console.{h,c}        # Consola de texto (TCP y/o UDP, puerto 23) sobre la interfaz RF de ax25ip
│   ├── ota.{h,c} / ota_net.{h,c} # OTA por HTTP (/api/ota/upload) y por espota.py (:3232)
│   ├── delay_tune.{h,c}        # Autoajuste de post_rx_tx_delay_ms por barrido ICMP
│   ├── rx_stats.{h,c}          # Contadores por demodulador (v1/v2): CRC ok/err, flags, overflows
│   ├── carrier_detect.{h,c}    # Detector de portadora ajena (caída de ruido HF + RMS): canal ocupado para LBT, calidad por trama, /api/carrier
│   ├── netdiag.{h,c}           # Diagnóstico de red desde la web: ping ICMP y miniconsola TCP/UDP/telnet (/api/netdiag*); anota tramas IP RX/TX
│   ├── igate.{h,c}             # iGate APRS-IS (RF→IS, IS→RF opcional); sólo si igate.enabled; /api/igate
│   ├── telemetry.{h,c}         # Telemetría APRS periódica (T#, PARM/UNIT/EQNS/BITS); sólo si telemetry.enabled
│   ├── aprs_objects.{h,c}      # Objetos y boletines APRS; /api/aprs/object|bulletin
│   ├── rx_capture.{h,c}        # Captura de audio RX a WAV (/api/rx/capture), buffer sólo durante la captura
│   ├── config_validate.{h,c}   # Validación de config.json en POST /api/config
│   ├── dedup.{h,c} / adpcm.{h,c} # Dedup compartido (FNV-1a con TTL) / códec IMA ADPCM
│   ├── il2p.{h,c} / rs_codec.{h,c} # IL2P (FEC Reed-Solomon + scrambler); RS(255,239) y cabecera sobre GF(2⁸)
│   ├── squelch_sf.{h,c}        # Squelch HFNE (Goertzel 3150–4350 Hz); sólo con monitor/repetidor activos
│   ├── rx_survey.{h,c}         # Modo SURVEY: mide audio RX (pico/RMS/Goertzel 1200-2200/HFNE) y emite filas por WS; sólo activo al iniciarlo (pestaña SURVEY)
│   ├── repeater.{h,c}          # Repetidor de voz analógico (DESHABILITADO: REPEATER_ENABLED 0)
│   ├── spiffs_data/
│   │   ├── config.json         # Config activa flasheada en SPIFFS (¡contiene credenciales!)
│   │   ├── config.json.normal  # Copia de referencia local (en .gitignore)
│   │   ├── config.json.defaults # Plantilla versionada sin secretos (todas las secciones, incl. `tx`); copiar a config.json y rellenar WiFi/indicativo
│   │   ├── index.html          # UI web completa (~89 KB, ~1600 líneas, un solo fichero); el build la flashea como index.html.gz (~23 KB) vía build/spiffs_staging
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

**Radio**: Quansheng UV-K5 ("UV-5K") vía placa ESPRI. El audio de RX viene de la **salida de altavoz** (el nivel depende del volumen de la radio); el **squelch de la radio está desactivado**, así que sin portadora el ADC ve ruido de FM continuo y al llegar una portadora ese ruido cae. Pruebas en 144.800 MHz con una segunda radio. Un indicador de nivel basado solo en pico mide ruido; usar `rx_survey` para calibrar.

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
  │     │     ├── xQueueCreate(s_tx_queue, 6)           ← cola de tramas TX (TX_QUEUE_DEPTH)
  │     │     ├── xTaskCreate(aprs_poll_task, prio 9)   ← APRS_poll() + il2p_poll() cada tick
  │     │     └── xTaskCreate(receive_audio_task, prio 10, stack 8192)
  │     │           └── [dentro de la tarea] adc_peripheral_start()
  │     ├── APRS_setCallsign() + afsk_set_post_rx_tx_delay_ms()   ← aprs.* de config.json
  │     ├── afsk_set_tx_fn(tx_noted_frame)       ← envoltorio: netdiag_note_tx + APRS_send_raw_frame (o smart_tx_frame si il2p.enabled)
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

HTTP server (audio_stream.c, port 80): hasta 56 URIs (`max_uri_handlers`), ver tabla en README.md
  (aprs/send, aprs/beacon, me, config, log, rx/stats, reboot, spiffs/upload, morse, sstv/*,
   ota/upload, repeater/*, squelch/*, tune/*, sys, carrier, survey/*, netdiag/*)

on_ax25_raw_frame (main.c, llamado por el raw_hook de AX25 y de IL2P):
  ├── rx_frame_is_duplicate()        → descarta la 2ª copia (doble módem), ventana 300 ms
  ├── afsk_notify_rx_frame()         → reinicia la ventana de inhibición post-RX
  ├── kiss_send_frame()              → host KISS TCP
  ├── try_auto_ack()                 → si va dirigida a nosotros con {NNN}: APRS_queue_ack()
  │     (el ACK se encola ANTES que la respuesta de remote_cmd_handle) + WS {type:ack_sent}
  ├── digi_process_frame()           → digipeater WIDEn-N (encola por afsk_queue_tx_frame)
  ├── ax25ip_rx_frame()              → inyecta en lwIP si PID=0xCC
  └── audio_stream_ws_send_text()    → JSON {type:aprs,...} (+ {type:digipeated,...})

server_task (transport_wifi.c) solo acepta; hasta 2 clientes KISS (KISS_MAX_CLIENTS), cada uno con tareas kiss_cliN/kiss_wrN, contexto KISS propio y cola de escritura (v1.10.0):
  recibe bytes KISS → kiss_ctx_rx_byte() → on_kiss_frame → afsk_queue_tx_frame()
    (bloquea hasta KISS_TX_ENQUEUE_TIMEOUT_MS = 15 s si la cola está llena: contrapresión)
    ↑ NO llama APRS_send_raw_frame directamente (viola mutex ADC)

Salida KISS hacia el host (v1.6.5): kiss_send_frame → transport_write → copia a MessageBuffer (3 KB, sin espera)
  → kiss_writer_task (prio 6) hace el send() con SO_SNDTIMEO 150 ms. El hook de RX nunca toca el socket.

TX (despachado por receive_audio_task desde s_tx_queue):
  s_tx_fn(data, len) = APRS_send_raw_frame (o smart_tx_frame: PID 0xCC → IL2P, resto AX.25)
    → ax25_sendRaw → AFSK_transmit
  switch_to_tx():
    tx_mode=true → adc_continuous_stop → adc_continuous_deinit
    → vTaskDelay(5ms) → dac_continuous_new_channels → dac_continuous_enable
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

- **`sdkconfig` vs `sdkconfig.defaults`**: `sdkconfig` está en `.gitignore` y ya no se versiona (se sacó del índice en v1.9.1; el fichero local se conserva); la configuración reproducible vive en `sdkconfig.defaults` (flash 4 MB, `partitions.csv`, WS + callback de handshake, `LWIP_IP_FORWARD=y` necesario para `ax25ip.c`, 16 sockets, WiFi 8/8 buffers, WPA3 e IPv6 desactivados). Si cambias un ajuste con `menuconfig`, trasládalo a `sdkconfig.defaults` (`idf.py save-defconfig` muestra las diferencias, pero sobrescribe el fichero: haz copia). Tras `idf.py fullclean` o borrar `sdkconfig`, ejecuta `idf.py reconfigure`. Optimización global `-Og` (valor por defecto de IDF); los ficheros de DSP caliente (`AFSK.cpp`, `rs_codec.c`, `il2p.c`, `squelch_sf.c`, `adpcm.c`, `CRC-CCIT.c`) se compilan con `-O2` vía `set_source_files_properties` en `main/CMakeLists.txt`. `CONFIG_FREERTOS_HZ=1000` (tick 1 ms): `sdkconfig.defaults` no pisa un `sdkconfig` existente, edítalo también si ya lo tienes.

- **`pbuf_length(p)` no existe en lwIP**: la longitud total de una cadena pbuf es `p->tot_len`. Usar `pbuf_clen(p)` (cuenta fragmentos) o `p->tot_len` (bytes totales).

- **`idf.py reconfigure` antes del primer ninja**: con IDF 6.1, `idf.py build` en un build directory vacío falla porque ninja dispara un re-run de CMake que intenta incluir `build/config/sdkconfig.cmake` antes de que CMake lo haya generado. Workaround: `idf.py reconfigure && ninja -C build`.

- **SPIFFS y `config.json`**: `aux_config.c` lee `/spiffs/config.json` en el arranque. `transport_wifi.c` usa la lista `wifi[]` (ssid/password/connect_timeout_s, se prueban en orden), reconecta automáticamente y cae a AP (`ap.*`) si falla. El fichero se flashea con `idf.py build` gracias a `spiffs_create_partition_image` (también `/flash-spiffs`). **`main/spiffs_data/config.json` está seguido por git** (a pesar de aparecer en `.gitignore`, que sólo afecta a ficheros no seguidos) y contiene credenciales WiFi reales también en commits anteriores: no hacer commit de credenciales; `git rm --cached` + plantilla sin secretos pendiente. `spiffs_data/sstv/*` sí está ignorado.

- **`doc/` está en `.gitignore`** (`doc/*`): las notas de diseño y backlog (`doc/pending`, `doc/obsolete`, `doc/to-migrate`, `doc/comandos_serial.md`…) no se versionan (sólo hay 2 imágenes seguidas). `report.md` ya no está en la raíz: está en `doc/obsolete/report.md`.

- **Librería `main/LibAPRS-esp32-i2s/` = repo git anidado** (remoto `luisesn/LibAPRS-esp32-i2s`, rama `master`). El repo principal no la rastrea (no hay `.gitmodules`; `git status` la muestra como `??`). Los cambios en `AFSK.*`, `AX25.cpp`, `FIFO.h`, `LibAPRS.cpp` se commitean **dentro** de ese repo (`git -C main/LibAPRS-esp32-i2s ...`). Un clon limpio del proyecto no la trae. Convertirla en submódulo está recomendado (ver `informe_mejoras.md` §2.bis): primero commit + push en la librería, luego `git submodule add`.

- **Listen-Before-Talk e inhibición post-RX** (`AFSK.cpp`, bucle de despacho TX): (a) el TX espera `aprs.post_rx_tx_delay_ms` desde la última trama recibida (`afsk_notify_rx_frame()`); (b) si `tx.lbt_enabled` (por defecto **false**), difiere mientras `channel_is_busy()`, hasta `tx.lbt_max_wait_ms` (10 s). `channel_is_busy()` = `carrier_detect_busy()` (siempre activo, `carrier_detect.c`) `||` HFNE (sólo si monitor/repetidor activos). `carrier_detect` declara ocupado cuando, en ventanas de ~107 ms, la energía HF (3,5–4 kHz) cae ≥8 dB bajo su suelo adaptativo **y** el RMS cae bajo 0,65× su suelo (calibrado con SURVEY: la radio sin squelch da ruido sin portadora y lo pierde con ella); falla en abierto (sin ruido → libre). El suelo no se adapta mientras está ocupado, así que tras `CD_MAX_BUSY_WINDOWS` (~12 s seguidos ocupado) se reaprende (`s_warm=0`, v1.4.7): evita quedarse ocupado para siempre si el suelo se aprendió demasiado alto (visto a volumen 25 %; causa sin confirmar, no reproducido offline con las capturas). Diagnóstico: `GET /api/carrier`. Umbrales como `#define` en `carrier_detect.c`, validados con una sola captura (10 tramas, un volumen): recalibrar con SURVEY si cambia la radio/volumen. **Calidad por trama** (`carrier_detect_frame_quality()` + `on_ax25_raw_frame`): campos `q`(1-4)/`qdb`/`lv`/`er` en el JSON `aprs`; umbrales de `q` (15/11/7 dB) provisionales. **Calibración (2 capturas SURVEY, 2026-11-10)**: volumen medio → reposo RMS≈21, pico≈52, caída de HF en portadora 10–12 dB; volumen máximo → reposo RMS≈66, pico 127 saturado, AGC=512 (tope), caída HF 8–11 dB. Ruido de reposo del HF: σ≈1–1,4 dB, por eso `CD_ENTER_HF_DB`=6/`CD_STAY_HF_DB`=4 (con 8 se perdían ventanas válidas). Los CRC erróneos son ≈90 % ruido en reposo (90–165 por captura frente a 4–12 durante portadora): `er` cuenta sólo los de la portadora. Portadoras sin tonos (RMS bajo, d12/d22 negativos) no se pueden decodificar. La web (v1.4.0) es de una columna con barra lateral, incluye la vista CANAL (canvas sin librerías) y muestra la versión del firmware (`/api/sys`).

- **Acceso al canal compartido (`tx_gate_open()` en `AFSK.cpp`, v1.5.0/v1.6.0)**: inhibición post-RX + LBT + CSMA p-persistente (`tx.persist` 63, `tx.slottime_ms` 100; sólo con LBT activo; también ajustables por KISS P/SlotTime). La cola `s_tx_queue` y el hook de despacho tienen cada uno su estado (`s_gate_queue`/`s_gate_hook`). Morse y SSTV llaman a `afsk_tx_clear_to_send()` **sólo cuando tienen algo pendiente** y arrancan únicamente si devuelve true. **El repetidor no lo usa a propósito** (retransmite una señal recibida: el canal está ocupado por su propia entrada). Un TX nuevo desde el hook debe llamar a `afsk_tx_clear_to_send()` antes de pulsar el PTT. Una transmisión SSTV dura hasta ~114 s en `receive_audio_task`: sin RX y con la cola AFSK parada; el canal sólo se comprueba antes de empezar.

- **Baliza GPS**: `gps.use_for_beacon` + `gps.beacon_period_s` (por defecto 600 s en código; el `config.json` actual usa 60 s). La tarea `gps_beacon` vive en `gps.c`. Para balizas manuales: `POST /api/aprs/beacon` o `tx,aprs,position` por APRS.

- **Consola RF** (`rf_console.c`): escucha en el puerto `console.port` (23) sólo en la interfaz RF de `ax25ip`; `console.tcp` y `console.udp` se activan por separado (por defecto TCP sí, UDP no). Comandos: `help`, `status`, `config`, `quit`. Sin autenticación.

- **v1.10.0 (F3–F8)**: `aprs.path`/`aprs.ack_path` (`APRS_set_path`/`APRS_set_ack_path` en la librería; vacío = sin path; hot). Baliza común `gps_build_beacon_info()` con SmartBeaconing (`gps.smart_beacon`), `gps.altitude` y posición fija (`gps.use_fixed`, `fixed_lat/lon`); `gps_send_beacon_now()` para la consola RF. 2 clientes KISS (≈+13 KB de heap, sin medir). `heard.c` (20 estaciones, `GET /api/heard`, comando `heard`). Chat persistente: `audio_stream_log_sent()` crea entradas `type:"sent"` en el log (estados pending/acked/rejected/failed, desde `msg_retry.c`; REJ vía `msg_retry_on_rej`). Todo sin probar en hardware.

- **v1.11.0 (mejoras opcionales §2.2)**: Mic-E y posiciones comprimidas en `heard.c`; `telemetry.c` (T#/PARM/UNIT/EQNS/BITS, `telemetry.enabled`), `igate.c` (APRS-IS, `igate.*`, `rf_gate` sólo mensajes a estaciones oídas <30 min, `/api/igate`), `aprs_objects.c` (`objects.*`, `POST /api/aprs/object|bulletin`), `rx_capture.c` (`GET /api/rx/capture?secs=&rate=9600`, WAV 8 bit), `config_validate.c` (400 en `POST /api/config` si no valida), comandos remotos `rx,stats`/`sys,status`/`sys,reboot` (`remote_cmd.allow_reboot`, `min_interval_s`; config sólo al arrancar) y `sstv.cw_id`. iGate/telemetría/objetos desactivados por defecto: sin tareas ni buffers. Caveat: el vector Mic-E de prueba daba 12.129W en vez de 112.129W (verificar con trama real). Todo sin probar en HW.

- **v1.13.0 (trazas en el mapa)**: `heard.c` guarda un anillo de 150 posiciones (`HEARD_TRACK_MAX`, ≈3,6 KB estáticos; sólo añade un punto si la estación se movió >~33 m); `GET /api/heard?track=<min>` (0 = todo; respuesta troceada, sin URI nueva) y selector «Traza» en la pestaña MAPA (15 min…6 h/todas, recordado en localStorage; un color por estación). El equipo pierde el historial al reiniciar (edades relativas al arranque), pero **la web lo acumula en `localStorage`** (`maptrk_pts`, hora absoluta = ahora − age, máx. 3000 puntos / 7 días, sondeo cada 30 s con la web abierta, dedup por estación/posición/tiempo) y filtra por tiempo en cliente; botón «Limpiar trazas locales» (sólo borra el navegador; el equipo vuelve a aportar sus ≤150 puntos). Test en `test_heard.c`. Sin probar en HW.
- **v1.12.0 (pestaña MAPA)**: mapa vectorial embebido en `index.html` (`MAP_DATA`: costa, fronteras y CCAA de España + sur de Francia/Portugal, Cantabria con más detalle; Natural Earth 10m, dominio público; ≈13 KB JSON, ≈5 KB gzip). Canvas sin librerías con arrastre/zoom/pellizco; pinta `/api/heard` (con posición) y la estación propia (`gps.use_fixed`). Sin fichero ni endpoint nuevos: se actualiza con el flasheo de SPIFFS o subiendo `index.html` (`/ui/index.html` del servidor `server/`). Sin Canarias. Sin probar en HW.
- **v1.11.1**: B23 (`kiss.c`: FEND tras FESC cierra la trama) y B24 (`digipeater.c`: alias `WIDEn-N` acepta `WIDEn-1..N`), hallados por el banco `test/`. Abierto B25: el demodulador v2 pierde tramas con deriva de reloj ≥0,5 % (medido en `test/demod`).

- **`aprs_poll_task` y el `raw_hook`**: `APRS_poll()` ya no corre en `receive_audio_task` sino en `aprs_poll_task` (prio 9), por lo que `on_ax25_raw_frame` se ejecuta en esa tarea. No bloquear en el hook (WS lento, mutex) o se llenan los FIFOs de RX. `FIFO.h` tiene variantes `_locked` (portMUX); el resto de `fifo_*` sin lock se usa en rutas de una sola tarea/ISR — auditar antes de tocar.

- **IL2P** (v1.8.1): `il2p.enabled` (por defecto false), `il2p.max_fec` (true), `il2p.tx_all` (false: sólo PID 0xCC por IL2P). Formato estándar (Dire Wolf): cabecera 13+2 tipo 1 (UI/I/S/U) o tipo 0 (path con repetidores, otras tramas, **TUN opaco** vía `il2p_tx_raw`), sync 0xF15E48, RS GF(2⁸) fcr=0 (`rs_codec.c`), scrambler por bloque, bits MSB primero (el TX hace `bitrev8` porque AFSK envía LSB primero; el ISR no escapa 0x1B con `il2p_tx`). La decodificación RS corre en `il2p_poll`. Validado en host con los vectores de la spec. **Prueba en el aire (2026-11-11): con `il2p.enabled` el Dire Wolf remoto NO decodifica los pings (modo TUN); con `il2p.enabled=false` sí. Abierto: falta saber si es nuestro TX IL2P (sync/scrambler/orden de bits/preámbulo) o la configuración IL2P-RX de ese Dire Wolf; comparar con `atest` sobre una captura del TX.** Ver `informe_bugs_funcionalidades.md` §2 bis.

- **Repetidor deshabilitado**: `REPEATER_ENABLED 0` en `config.h` (reservar ~15 KB contiguos de heap, pausa RX y detiene el servidor WAV). Con 0, ni `config.json` ni `/api/repeater/enable` pueden activarlo. Hay presión de heap: vigilar buffers `static` grandes y stacks de tareas.

- **Robustez (informe §3.5)**: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` (requiere reflashear bootloader por cable una vez); `ota_validate_task` (main.c) confirma la imagen a los 30 s. OTA HTTP acepta cabecera opcional `X-SHA256`. `save_config` escribe `config.tmp` y lo renombra (recuperación en `config_load_from_file`). Reconexión WiFi con backoff (5 rondas) antes de caer al AP. Reloj: SNTP al obtener IP + hora GPS (`gps_sync_system_time`). `GET /api/sys` da heap y stacks. Dedup compartido en `dedup.[ch]`; códec en `adpcm.c`.

- **AP y sondeos de conectividad**: el 404 de `audio_stream.c` responde a `/generate_204`, `/hotspot-detect.html`, `/connecttest.txt`, etc. como si hubiera Internet (`ap.captive_portal` = false por defecto; true = redirect a `/`). El DHCP del AP ofrece DNS 192.168.4.1 y **no** ofrece router (`ap.offer_gateway`, false por defecto) (se configura en `wifi_start_ap` con el servidor DHCP aún sin arrancar: si se hace después de `AP_START`, `esp_netif_dhcps_option` devuelve error). Sin probar en móvil real.

- **Dedup de comandos remotos** (`main.c`, `cmd_already_seen()`): el ACK de un mensaje con `{NNN}` se reenvía siempre, pero `remote_cmd_handle` solo se llama la primera vez por (origen, id) dentro de 5 min (8 entradas). Sin esto, un ACK perdido provocaba que `tx,sstv` se encolara varias veces. Los mensajes sin id no se deduplican.

- **OTA única** (`ota_acquire()`/`ota_release()` en `ota.c`): HTTP y espota comparten la exclusión; tras un OTA correcto no se libera (reinicio pendiente). Cualquier ruta nueva que escriba en la partición OTA debe usarla.

- **`POST /api/config`**: máx. 8 KB, buffer en heap. `config_reload()` devuelve copia: liberar con `config_free_json()`.

- **`FIFO.h`**: `fifo_len()` = bytes almacenados, `fifo_capacity()` = tamaño del buffer. `AFSK_transmit` vacía `txFifo` con `fifo_flush_locked`.

- **`config_load()` devuelve copia**: `aux_config.c` devuelve `cJSON_Duplicate(root, 1)` — el llamador es responsable de liberar el objeto con `config_free_json()`. No usar el puntero después de liberar.

- **Cola TX entre tareas (`s_tx_queue`)**: en modo KISS TNC, `server_task` **no puede** llamar `APRS_send_raw_frame` directamente porque internamente invoca `adc_continuous_stop`, que debe ejecutarse desde la misma tarea FreeRTOS que llamó a `adc_continuous_start` (mutex interno de ESP-IDF). Solución: `QueueHandle_t s_tx_queue` de capacidad 6 (`TX_QUEUE_DEPTH`) × `afsk_tx_frame_t`. `afsk_queue_tx_frame()` no bloquea (falla si la cola está llena); la ruta KISS (`on_kiss_frame`) usa `afsk_queue_tx_frame_wait()` con `KISS_TX_ENQUEUE_TIMEOUT_MS` (15 s) como contrapresión; `receive_audio_task` despacha al inicio de cada iteración. **Es obligatorio llamar `afsk_set_tx_fn(APRS_send_raw_frame)` en `app_main` antes de `APRS_set_raw_hook`** (hoy se registra el envoltorio `tx_noted_frame` de `main.c`, que anota la trama en `netdiag` y delega).

- **`TX_SAMPLE_BUFLEN=2048` — no reducir**: el descriptor DMA del DAC tiene `buf_size=2048` bytes (≈42 ms a 48 kHz). Con `DAC_DESC_NUM` = 4 descriptores, el pool total es ≈170 ms (la documentación anterior decía 8 / ≈336 ms; no se sabe si la reducción fue deliberada). Las tareas WiFi (prioridad 23) pueden preemptar `receive_audio_task` (prioridad 10) durante decenas de ms. Si el descriptor se consume antes de que la tarea recargue el siguiente, el semáforo `s_dac_wait_to_load_dma_data` expira → `dac_continuous_write` cuelga. Con `TX_SAMPLE_BUFLEN<buf_size` cada escritura llena menos de un descriptor y el margen desaparece.

- **Siempre escribir el buffer completo al DMA**: cuando `AFSK_dac_isr` establece `sending=false` a mitad del buffer, `transmit_audio_i2s` rellena el resto con `0x80` (nivel DC = silencio) y siempre escribe `TX_SAMPLE_BUFLEN` bytes. Escribir un buffer parcial agota el descriptor en <1 ms, el DMA se detiene, y las escrituras de silencio posteriores no consiguen cargar porque el ISR ya no dispara → timeout × N = bloqueo prolongado.

- **`dac_continuous_write` con timeout finito (2000 ms)**: **nunca usar `-1` (portMAX_DELAY)** para este write. Si el DAC se detiene inesperadamente, el timeout permite abortar con `afsk->sending = false` y volver al modo RX. Con timeout infinito, el firmware queda bloqueado con el PTT pulsado indefinidamente.

- **GPIO 26 = PTT = DAC2 (conflicto de recurso)**: GPIO 26 es simultáneamente el pin de PTT y DAC2 del ESP32. `DAC_CHANNEL_MODE_SIMUL` en `switch_to_tx()` puede reconfigurarlo como salida analógica DAC, dejando el GPIO en modo analógico y sacando el PTT del control digital. Mitigación aplicada: llamar `gpio_set_direction(GPIO_PTT_OUT, GPIO_MODE_OUTPUT)` después de `dac_continuous_new_channels()` en `switch_to_tx()` para restaurar el modo digital.

- **Demodulador v1 desactivado** (`AFSK_RX_V1_ENABLED 0` en `AFSK.h` de la librería; el arranque avisa si `rx.dual_modem_enabled`/`rx.active_modem` están en el config): sólo RX v2 (`AFSK_modem_v2` se crea siempre; `rx.dual_modem_enabled`/`rx.active_modem` se ignoran). `AFSK_modem` sigue siendo el modem de TX. `rx_stats_v1` queda a 0; `TNC_MODE_APRS` (`AX25.hook`) no recibe mensajes parseados. Poner a 1 para restaurar el doble módem (entonces aplica lo siguiente).

- **Doble módem RX entrega cada trama dos veces**: con `rx.active_modem = "best"` (valor por defecto), v1 (`ax25_poll`) y v2 (`aprs_poll_v2`) demodulan el mismo audio y llaman por separado a `s_app_raw_hook` con la misma trama, separados por unos ms. Sin filtro eso significa doble inyección en lwIP (`ax25ip`), doble auto-ACK y doble digipetición — en un ping por RF se ve como `DUP!` y dos ciclos TX. El dedup está **en `on_ax25_raw_frame()`** (`rx_frame_is_duplicate()`, ventana `RX_DUP_WINDOW_MS` = 300 ms, comparación byte a byte), que es el punto donde convergen todos los consumidores. No añadir un segundo filtro en LibAPRS (rompería `rx_stats_v1`/`rx_stats_v2`) ni en `ax25ip.c`/`digipeater.c`.

- **Polaridad PTT**: El hardware usa PTT **activo alto** (1 = transmitiendo, 0 = reposo). El control de PTT está centralizado en `AFSK.cpp`. **No invertir** las llamadas a `gpio_set_level`.

- **Tick de 1 ms (`FREERTOS_HZ=1000`)**: `vTaskDelay(1)` = 1 ms, no 10 ms. Los sondeos periódicos (`aprs_poll_task`, `gps_task`) usan `pdMS_TO_TICKS(10)` para no multiplicar la CPU; usa siempre `pdMS_TO_TICKS()` en código nuevo. `receive_audio_task` mantiene `vTaskDelay(1)` (necesario para que corra IDLE0).

- **Audio hacia la web = `StreamBuffer`** (`audio_stream_q`, 2048 B, 1 productor/1 consumidor): `audio_sample_hook` acumula 64 muestras y las envía de golpe; `audio_stream_task` solo codifica ADPCM si hay clientes WS `/ws` o WAV (`audio_stream_has_clients()`, flag refrescado cada 500 ms, así que el primer cliente puede tardar hasta 0,5 s en oír audio). El repetidor y el squelch siguen recibiendo cada muestra por separado en el hook. Solo `receive_audio_task` puede llamar al hook (un único productor).

- **Buffer de TX compartido** (`afsk_tx_scratch()`, 2048 B en `AFSK.cpp`): lo usan el TX AFSK, SSTV y morse. Es seguro porque los tres transmiten secuencialmente en `receive_audio_task` y hacen flush antes de ceder. No introducir un TX que mantenga datos parciales en él entre llamadas a otro tipo de TX.

- **FIFOs del módem de 2 KB** (`CONFIG_AFSK_RX_BUFLEN/TX_BUFLEN` en `AFSK.h`, antes 8 KB): las tramas miden ≤600 B y `AFSK_putchar` drena el FIFO TX si se llena. Si se aumentase `CUSTOM_FRAME_SIZE`, revisar. Buffers de OTA (HTTP y espota, 4 KB) en heap solo durante el OTA. Memoria estática DRAM ≈78 KB (43 % de uso); stack de `ota_net` 5 KB. Pilas sin medir en dispositivo: usar `GET /api/sys` antes de recortar más.

- **`index.html` se flashea comprimido**: `main/CMakeLists.txt` copia `spiffs_data/` a `build/spiffs_staging/`, gzipea `index.html` → `index.html.gz` y genera la imagen SPIFFS desde ahí (se regenera al tocar cualquier fichero de `spiffs_data`). El handler de `/` ([api_rest.c](main/api_rest.c)) prefiere un `/spiffs/index.html` suelto (subido con `/api/spiffs/upload`, útil para iterar la UI sin reflashear) y si no existe sirve el `.gz` con `Content-Encoding: gzip`. Si subes un `index.html` y luego quieres volver al flasheado, bórralo de SPIFFS.

## Al editar este proyecto

- **Versión del firmware (obligatorio)**: la fuente única es `version.txt` en la raíz (`MAJOR.MINOR.PATCH`; IDF la usa como `PROJECT_VER`). Se muestra en el log de arranque (`Firmware vX.Y.Z`), en `GET /api/sys` (`version`) y en la web de `server/`. **En cada cambio que hagas, sube la versión en `version.txt` antes de compilar**: cambio menor (corrección, ajuste, texto, refactor pequeño) → `+0.0.1`; cambio mayor (funcionalidad nueva, módulo, endpoint, cambio de comportamiento) → `+0.1.0` y se pone el último dígito a 0. Un cambio que rompa compatibilidad (config, partición, protocolo) → `+1.0.0`, pregunta antes. Los cambios solo de documentación o del servidor `server/` no suben la versión del firmware. Actualiza también la mención de versión en README/CLAUDE.md si cita una.
- **Commits (obligatorio)**: haz un commit por cada cambio relevante terminado (funcionalidad, corrección, refactor con sentido propio), sin esperar a que lo pida el usuario; un commit por tema, no uno gigante. Mensajes en español con prefijo convencional (`feat:`, `fix:`, `docs:`, `refactor:`, `chore:`), con la versión si cambió (`feat: ... (v1.1.0)`). Incluye `version.txt` en el mismo commit. **Nunca** incluyas `main/spiffs_data/config.json` (credenciales WiFi), `sdkconfig`, `sdkconfig.old`, `build/` ni `*.pem`; añade los ficheros por nombre (no `git add -A`). Los cambios en `main/LibAPRS-esp32-i2s/` se commitean dentro de ese repo anidado. No hagas `push` salvo que se pida.

- **Dudas (obligatorio)**: si algo no está claro (alcance, decisión de diseño, comportamiento deseado, riesgo para el hardware), **pregunta al usuario antes de implementarlo** (`AskUserQuestion`) en lugar de suponer; para lo trivial o con un valor por defecto evidente, elige y menciónalo en la respuesta. Los subagentes que lances no preguntan: devuelven sus dudas en el informe final.
- **Agentes en paralelo**: reparte el trabajo por ficheros (dos agentes no deben editar el mismo fichero) y usa el modelo más barato que baste (haiku/sonnet para cambios acotados). Los agentes sólo editan; compilar (`ninja -C build`, un único build dir), subir la versión y commitear lo hace el agente principal.
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

Ver [README.md](README.md) (uso, API, configuración), [PROGRESS.md](PROGRESS.md) (bitácora) , [informe_bugs_funcionalidades.md](informe_bugs_funcionalidades.md) e [informe_modem.md](informe_modem.md) (análisis de 2026-11: bugs, funcionalidades y mejoras del demodulador). `informe_mejoras.md` (2026-10-08, con anexos de seguridad y pruebas) es el análisis anterior y no está versionado. El informe técnico original está en `doc/obsolete/report.md`.

Resumen rápido (2026-11-11):
- Bloqueantes de la primera auditoría: todos resueltos en código ✅
- KISS TNC bidireccional (WiFi TCP :8001, keepalive, escritura con buffer, contrapresión) ✅
- TX verificado en hardware: datos decodificados por receptor externo ✅
- UI web (`index.html`): log APRS (incluye tráfico de terceros), CHAT con ACK, audio ADPCM, SSTV, OTA, TUNE, CONFIG, STATS ✅
- Comandos remotos vía APRS (`remote_cmd.c`), sin autenticación ✅
- Gateway IP RFC 1226 / TUN (`ax25ip.c`) verificado en hardware ✅
- Dedup de tramas RX por doble módem en `on_ax25_raw_frame()` ✅
- Listen-Before-Talk + inhibición post-RX + autoajuste `post_rx_tx_delay_ms` (TUNE) ✅ (LBT usa `carrier_detect`, siempre activo; desactivado por defecto; cubre cola AFSK, morse y SSTV, no el repetidor; con CSMA p-persistente)
- GPS + baliza de posición periódica; display SSD1306; digipeater WIDEn-N; baliza CW ✅
- IL2P ⚠️ reescrito al estándar (v1.8.0): vectores de la spec OK en host, IP (también en modo TUN) como UI AX.25 PID 0xCC estándar, interoperable; pendiente de pruebas en el aire/HW
- Subida de imágenes SSTV desde la web corregida (v1.6.6, B15; probada en host, falta HW) ✅
- v1.10.0: path configurable, SmartBeaconing/posición fija, 2 clientes KISS, chat persistente, `heard` ✅ (sin probar en HW)
- Reintento de mensajes con seguimiento de ACK (`msg_retry.c`, v1.8.0, F1) ✅
- Diagnóstico de red en la web (`netdiag.c`: ping, miniconsola TCP/UDP) ✅
- OTA por HTTP y por `espota.py`; WiFi multi-red con fallback AP ✅
- Consola RF (TCP/UDP) ✅
- `sdkconfig.defaults` completo (reproducible) ✅
- Build binario: ~928 KB (≈46 % libre de la partición de app) con `idf.py reconfigure && ninja -C build` ✅
- Verificación RX con señal RF real: pendiente ⚠️
- Repetidor de voz (deshabilitado por `REPEATER_ENABLED 0`), display, IL2P: verificación HW pendiente ⚠️
- Librería `LibAPRS-esp32-i2s` como repo anidado sin enlazar (sin cambios pendientes en la librería; falta convertirla en submódulo) ⬜
- Auditoría 2026-11-08: corregidos dedup de comandos remotos, límite de `POST /api/config` (+fuga), exclusión OTA, `fifo_len`/flush, aviso `rx.dual_modem_*`, baliza de posición. Seguridad (sin autenticación, `config` expone contraseñas) aparcada ⬜
- `config.json` con credenciales seguido por git ⬜
- Banco de pruebas en host `test/` (unit + demodulador con 2 WAV reales en `test/demod/corpus/`, `ctest`) ✅; sin CI ⬜; FIFOs no `_locked` en rutas multi-tarea sin auditar ⬜

## Referencias externas

- LibAPRS original (AVR): https://github.com/markqvist/LibAPRS
- LibAPRS-esp32-i2s upstream: https://github.com/handiko/LibAPRS-esp32-i2s
- Especificación AX.25 v2.2: https://www.tapr.org/pdf/AX25.2.2.pdf
- Fork propio de la librería: https://github.com/luisesn/LibAPRS-esp32-i2s

# esp32-aprs-modem

Módem APRS / KISS TNC (AX.25 sobre AFSK Bell-202, 1200 bps) para **ESP32**, basado en ESP-IDF v6.1.

Uso la placa ESPRI (de https://github.com/kamilsss655/ESPRI)

(Creado dando latigazos a Claude y otros...)

> ✅ **Estado (2026-05-26): compila limpio (binary ~874 KB, 48 % libre). KISS TNC bidireccional operativo. TX verificado en hardware. UI web funcional con log, mensajes, baliza de posición, editor de configuración, grabación de audio, actualización OTA de firmware y botón de reinicio remoto. Barra de nivel de audio con zonas de rango correcto (verde) e incorrecto (rojo), con marcadores de umbral en la barra AGC. Digipeater WIDEn-N operativo. Baliza morse CW periódica operativa. TX SSTV implementado (Martin M1/M2, Scottie S1, Robot 36/72) con botón de parada. GPS NMEA por UART2 implementado. Display SSD1306 I2C 128×64 implementado con estadísticas de decodificador. Gateway IP RFC 1226 (modo TUN) verificado en hardware con tncattach y dos Baofeng UV-5R (ping bidireccional operativo con `post_rx_tx_delay_ms: 950`). Reconexión WiFi automática con ciclo de redes y fallback a AP. Latencia TX→RX reducida (callback DMA en lugar de espera fija). **Repetidor de voz analógico implementado**: squelch HFNE (Goertzel sobre banda de ruido FM), grabación ADPCM IMA (~49 KB por 10 s), retransmisión con tono de cortesía e identificación CW, ventana mínima de grabación de 500 ms, modo monitor independiente. RX pendiente verificación con señal RF real.**

El firmware opera como un **KISS TNC bidireccional** accesible desde la red local vía TCP. Conecta `tncattach` o `direwolf` en el host y obtienes una interfaz de red AX.25 (`tnc0`) o un gateway APRS completo — sin cable USB, sin drivers adicionales.

Este repositorio adapta [LibAPRS-esp32-i2s](https://github.com/handiko/LibAPRS-esp32-i2s) — fork de [LibAPRS](https://github.com/markqvist/LibAPRS) de markqvist — para funcionar bajo ESP-IDF usando el DAC interno y ADC continuo por DMA.

## Hardware objetivo

- **MCU**: ESP32 (clásico, Xtensa LX6 dual-core).
- **Salida de audio (TX)**: DAC1 en GPIO 25 (`dac_continuous`, muestras 8-bit directas). Sin filtro externo necesario para la mayoría de transceptores.
- **Entrada de audio (RX)**: GPIO 35 / ADC1_CH7 (`adc_continuous` DMA a 48 kHz, atenuación 12 dB → rango 0–3,1 V). Entrada AC-acoplada desde la salida de altavoz del transceptor.
- **PTT**: GPIO 26, activo en nivel alto (1 = transmitir, 0 = reposo). Restricción de hardware (también es DAC2). Definido en `config.h`.
- **GPS**: UART2 a 9600 baud. RX = GPIO 16 (pin SD DAT2 liberado), TX = GPIO 4 (pin SD DAT1 liberado, opcional para configurar el módulo GPS).
- **Display LCD**: SSD1306 128×64 OLED por I2C. SDA = GPIO 13, SCL = GPIO 19.

## Modos de operación

Seleccionado en `config.h` con `TNC_MODE` (requiere recompilación):

| Modo | `TNC_MODE` | Descripción |
|---|---|---|
| **KISS TNC** (por defecto) | `TNC_MODE_KISS` | Protocolo KISS sobre WiFi TCP (port 8001). Compatible con `tncattach` y `direwolf`. Incluye servidor HTTP (UI web + WebSocket audio). |
| **APRS consola** | `TNC_MODE_APRS` | Imprime paquetes AX.25 decodificados por el monitor serie. Modo debug. |

## Estructura del repositorio

```
esp32-aprs-modem/
├── CMakeLists.txt                     proyecto ESP-IDF raíz
├── sdkconfig                          configuración IDF (target = esp32)
├── sdkconfig.defaults                 valores por defecto (incluye CONFIG_LWIP_IP_FORWARD=y)
├── partitions.csv                     tabla de particiones (NVS + OTA×2 + SPIFFS 704 KB)
├── main/
│   ├── CMakeLists.txt                 fuentes, dependencias y creación de imagen SPIFFS
│   │                                  (CUSTOM_FRAME_SIZE=600 para tncattach MTU 512)
│   ├── config.h                       TNC_MODE, KISS_TRANSPORT, TCP port y GPIOs
│   ├── main.c                         app_main — bifurca según TNC_MODE
│   ├── kiss.h / kiss.c                framing KISS (encode/decode), independiente del transporte
│   ├── transport.h / transport.c      interfaz abstracta { init, write } para transportes
│   ├── transport_wifi.h / .c          WiFi STA + servidor TCP KISS (transporte activo)
│   ├── audio_stream.h / audio_stream.c  servidor HTTP port 80 (UI web), WebSocket /ws
│   │                                  (audio IMA ADPCM + JSON APRS), WAV TCP port 8080,
│   │                                  REST: /api/aprs/send, /api/aprs/beacon, /api/me,
│   │                                        /api/config (GET/POST), /api/morse/trigger,
│   │                                        /api/log, /api/rx/stats
│   ├── digipeater.h / digipeater.c    digipeater WIDEn-N: aliases múltiples, supresión de
│   │                                  duplicados (FNV-1a, TTL 30 s), logging serie + WebSocket
│   ├── morse.h / morse.c              baliza morse CW periódica (tono, WPM, periodo, one-shot)
│   ├── sstv.h / sstv.c                TX SSTV (Martin M1/M2, Scottie S1, Robot 36/72);
│   │                                  JPEG on-the-fly vía ROM TJpgDec; REST: /api/sstv/*
│   ├── ota.h / ota.c                  actualización OTA de firmware vía HTTP
│   │                                  (POST /api/ota/upload, validación magic 0xE9, reboot 3 s)
│   ├── repeater.h / repeater.c        repetidor de voz analógico: squelch HFNE, grabación
│   │                                  ADPCM IMA, retransmisión con tono de cortesía y CW ID,
│   │                                  ventana mínima de grabación 500 ms; REST: /api/repeater/*
│   ├── squelch_sf.h / squelch_sf.c    squelch HFNE (High-Frequency Noise Energy) sobre 4 bins
│   │                                  Goertzel (3150–4350 Hz); fuentes: repetidor y monitor UI;
│   │                                  REST: /api/squelch/*; WS: {"type":"squelch_sf",...}
│   ├── rx_stats.h / rx_stats.c        estadísticas de recepción por demodulador; /api/rx/stats
│   ├── gps.h / gps.c                  receptor GPS por UART2 (NMEA $GPRMC/$GPGGA),
│   │                                  struct g_gps_pos con mutex FreeRTOS
│   ├── display.h / display.c          driver SSD1306 I2C 128×64 (sin librería externa),
│   │                                  task de refresco 2 Hz con callsign, IP, GPS, audio
│   ├── ax25ip.h / ax25ip.c            gateway IP sobre radio; modo TUN (tncattach) o AX.25 RFC 1226
│   ├── aux_config.h / aux_config.c    carga/guarda config JSON desde SPIFFS (/spiffs/config.json)
│   ├── aux_file_management.h / .c     utilidades de sistema de ficheros SPIFFS
│   ├── spiffs_data/config.json        configuración activa (credenciales reales — no subir a git)
│   ├── spiffs_data/config.json.example  esqueleto de configuración con todos los campos
│   ├── spiffs_data/index.html         UI web completa (log APRS, mensajes, baliza posición,
│   │                                  audio IMA ADPCM, grabación WAV, editor de configuración)
│   ├── idf_component.yml              declaración de dependencias (esp-dsp — pendiente eliminar)
│   └── LibAPRS-esp32-i2s/src/
│       ├── LibAPRS.{h,cpp}            API de alto nivel (APRS_init, queue_msg, queue_ack,
│       │                              queue_beacon, getCallsign…)
│       ├── AFSK.{h,cpp}               modulador/demodulador AFSK, DAC TX, ADC RX,
│       │                              cola TX s_tx_queue, hook de despacho periódico
│       ├── AX25.{h,cpp}               codificación/decodificación AX.25 + raw_hook para KISS
│       ├── CRC-CCIT.{h,c}             CRC-CCITT para tramas AX.25
│       ├── HDLC.h                     flags HDLC (0x7E, 0x7F, AX25_ESC)
│       ├── FIFO.h                     cola circular inline con variantes _locked (portMUX)
│       └── FakeArduino.{h,cpp}        stubs de Serial, F(), _BV(), cli/sei
├── managed_components/                dependencias gestionadas por IDF
├── CLAUDE.md                          guía de contexto para Claude Code
├── report.md                          informe técnico de problemas y soluciones
└── PROGRESS.md                        seguimiento de arreglos y bitácora
```

## Dependencias

- **ESP-IDF v6.1** (usa `dac_continuous`, `adc_continuous`, `esp_wifi`, `esp_netif`, `spiffs`, `esp_http_server`, `esp_timer`, `app_update`).
- Componentes IDF requeridos (declarados en `main/CMakeLists.txt`):
  `esp_wifi`, `nvs_flash`, `esp_netif`, `lwip`, `driver`, `esp_driver_dac`, `esp_driver_gpio`, `esp_driver_uart`, `esp_driver_i2c`, `esp_adc`, `spiffs`, `esp_http_server`, `vfs`, `espressif__cjson`, `esp_timer`, `app_update`.

## Compilación y flasheo

```bash
# Antes: editar main/spiffs_data/config.json con tus credenciales WiFi
idf.py set-target esp32
idf.py reconfigure   # necesario en IDF 6.1 antes del primer build
ninja -C build       # o: idf.py build (puede fallar en primer build sin reconfigure)
idf.py -p <PUERTO_SERIE> flash monitor
```

Para actualizar solo el firmware sin sobreescribir el config.json en el dispositivo:
```bash
idf.py -p <PUERTO_SERIE> app-flash monitor
```

En Windows con el entorno IDF, sustituye `<PUERTO_SERIE>` por `COM3`, `COM4`, etc.

## Qué hace el firmware (modo KISS TNC)

1. `config_load()` lee `config.json` desde la partición SPIFFS (callsign, WiFi, AP, IP, digi, morse).
2. `transport_init(&transport_wifi_ops)` conecta a la red WiFi configurada e imprime la IP asignada. Si la conexión se pierde en cualquier momento, la tarea `wifi_reconn` cicla automáticamente por todas las redes configuradas (en orden) y cae al modo AP si todas fallan.
3. `ax25ip_init()` activa el gateway IP RFC 1226 si `ip.enabled: true` en config.json.
4. `kiss_init(on_kiss_frame)` registra el callback que transmite por radio las tramas recibidas del host.
5. `APRS_init()` + `APRS_set_raw_hook(on_ax25_raw_frame)` arranca el demodulador AFSK y registra el callback que:
   - Envía la trama al host KISS TCP (`kiss_send_frame`).
   - Detecta mensajes dirigidos a nuestro indicativo y transmite un ACK automático (`try_auto_ack`).
   - Digipeata la trama si procede (`digi_process_frame`) y notifica vía WebSocket con badge DIGI.
   - Inyecta paquetes IP en la pila lwIP si PID=0xCC (`ax25ip_rx_frame`).
   - Notifica a los clientes WebSocket con JSON `{"src":..., "dst":..., "path":..., "info":...}`.
6. `digi_init()` configura el digipeater WIDEn-N desde `config.json`.
7. `morse_init()` configura la baliza morse CW. `sstv_init()` crea el directorio `/spiffs/sstv` y registra los endpoints REST de SSTV. `ota_init()` registra el endpoint `POST /api/ota/upload`. Los dos primeros se despachan desde `receive_audio_task` mediante el hook registrado con `afsk_set_dispatch_hook()`.
8. `gps_init()` arranca la tarea FreeRTOS GPS (UART2, GPIO16/GPIO4, 9600 baud). Parsea $GPRMC y $GPGGA y actualiza la struct global `g_gps_pos` bajo mutex.
9. `display_init()` inicializa el bus I2C y el SSD1306 (GPIO13/GPIO19) y arranca la tarea de refresco del display a 2 Hz.
10. `audio_stream_init()` arranca el servidor HTTP en port 80 (UI web + WebSocket `/ws`) y el WAV server en port 8080.
11. Cuando el host envía una trama KISS → `on_kiss_frame` → `afsk_queue_tx_frame()` (encola) → `receive_audio_task` despacha → `APRS_send_raw_frame()` → DAC → radio.
12. `audio_level_task` muestrea `audio_peak` cada 100 ms, genera la barra de nivel de la consola serie, y controla los LEDs: el **LED verde** (GPIO33, `GPIO_LED_RX`) lo gestiona AFSK.cpp y parpadea en cada paquete AX.25 decodificado; el **LED rojo** (GPIO23, `GPIO_LED_WARN`) parpadea cuando el nivel de audio supera `AUDIO_LEVEL_TOO_HIGH` (demasiado alto) o está por debajo de `AUDIO_LEVEL_TOO_LOW` (demasiado bajo). El display SSD1306 también muestra la barra y el texto "LOUD" en la misma condición.

## Interfaz web

Navega a `http://<IP-del-ESP32>/` para acceder a la UI web integrada:

- **Log APRS**: muestra paquetes recibidos por radio en tiempo real (newest-first). Badges: `PARA MÍ` si el mensaje va dirigido a tu indicativo, `ACK` para confirmaciones, `TX` para los enviados, `DIGI` (azul) para los retransmitidos por el digipeater. Click en callsign rellena el destino.
- **Pestaña MENSAJE**: formulario para transmitir mensajes APRS directamente desde el navegador.
- **Pestaña POSICIÓN**: formulario para transmitir baliza de posición APRS (lat/lon decimal, símbolo, comentario). Botón de baliza Morse on-demand.
- **Pestaña CONFIG**: editor JSON del `config.json` completo. Guardar recarga el digipeater y la baliza morse sin reiniciar el firmware. Botón **↺ Reiniciar** para reiniciar el ESP32 de forma remota (pide confirmación; el firmware responde, espera 800 ms y ejecuta `esp_restart()`).
- **Pestaña OTA**: sube un `.bin` generado por ESP-IDF para actualizar el firmware via OTA. Barra de progreso de subida, validación del magic byte del ESP32 (0xE9), y reinicio automático a los 3 s tras un flash exitoso.
- **Audio**: streaming de audio de recepción en tiempo real vía WebSocket (IMA ADPCM, 9600 Hz). La barra de nivel usa un gradiente rojo/verde/rojo con las zonas de rango correcto visibles incluso sin señal. La barra AGC de la pestaña STATS muestra el nivel de entrada al AGC en cuentas ADC (escala 4–512): verde si está en el rango óptimo (20–400), rojo si la señal es demasiado débil (< 20, solo ruido) o demasiado fuerte (> 400, cerca de saturación del ADC); el valor numérico cambia de color en consecuencia. Botón de grabación para capturar audio y descargar como WAV. También disponible como stream WAV en `http://<IP>:8080/`.

## REST API

| Método | Endpoint | Descripción |
|--------|----------|-------------|
| `GET`    | `/`                     | Sirve `index.html` desde SPIFFS |
| `GET`    | `/ws`                   | WebSocket upgrade (audio binario IMA ADPCM + JSON APRS/ACK/DIGI) |
| `POST`   | `/api/aprs/send`        | Envía mensaje APRS. Body JSON: `{"to":"CALL","ssid":N,"text":"..."}` |
| `POST`   | `/api/aprs/beacon`      | Transmite baliza de posición. Body JSON: `{"lat":40.4,"lon":-3.7,"symbol":">","comment":"..."}` |
| `GET`    | `/api/me`               | Devuelve `{"call":"...","ssid":N}` con el indicativo configurado |
| `GET`    | `/api/config`           | Devuelve `config.json` completo |
| `POST`   | `/api/config`           | Guarda nuevo `config.json` y recarga digipeater + morse en RAM |
| `POST`   | `/api/morse/trigger`    | Dispara baliza morse inmediata (one-shot) |
| `GET`    | `/api/log?since=N`      | Historial de log (entradas de secuencia > N) |
| `GET`    | `/api/rx/stats`         | Estadísticas de recepción (tramas OK/error, FIFO overflows, AGC, squelch) |
| `GET`    | `/api/sstv/list`        | Lista imágenes JPEG disponibles en `/spiffs/sstv/` |
| `POST`   | `/api/sstv/send`        | Transmite imagen SSTV. Body: `{"name":"foto.jpg","mode":"martin_m1"}` |
| `POST`   | `/api/sstv/test`        | Transmite patrón de prueba SMPTE. Body: `{"mode":"robot_36"}` |
| `POST`   | `/api/sstv/stop`        | Aborta la transmisión SSTV en curso. El firmware termina la línea actual y envía `{"type":"sstv_aborted"}` por WebSocket |
| `POST`   | `/api/sstv/upload`      | Sube JPEG vía `multipart/form-data` (partes: `name` + `image`; máx. 200 KB, 10 imágenes) |
| `DELETE` | `/api/sstv/image?name=` | Elimina imagen de la galería SSTV |
| `POST`   | `/api/ota/upload`       | Actualización OTA. Body: binario raw (`application/octet-stream`). Valida magic 0xE9, flashea partición inactiva y reinicia en 3 s |
| `POST`   | `/api/reboot`           | Reinicia el ESP32. Responde `{"ok":true}` y ejecuta `esp_restart()` 800 ms después |
| `GET`    | `/api/repeater/status`  | Estado del repetidor: `{"enabled":bool,"state":"idle|recording|tail|pending|tx"}` |
| `POST`   | `/api/repeater/enable`  | Activa o desactiva el repetidor. Body JSON: `{"enabled":true}` |
| `GET`    | `/api/squelch/status`   | Estado del squelch HFNE: `{"active":bool,"manual":bool,"hfne":float,"open":bool,"thr":float}` |
| `POST`   | `/api/squelch/monitor`  | Activa/desactiva modo monitor (squelch sin repetidor). Body JSON: `{"active":true}` |
| `POST`   | `/api/squelch/sf_config`| Ajusta umbral HFNE en runtime. Body JSON: `{"hfne_threshold":0.25}` |
| `POST`   | `/api/spiffs/upload?name=<fichero>` | Sube un fichero a SPIFFS. Body: binario raw. Escribe en `/spiffs/<fichero>`. Responde `{"ok":true,"name":"...","size":N}`. Útil para actualizar `index.html` sin reflashear todo el firmware |

## Digipeater WIDEn-N

El firmware incluye un digipeater AX.25 compatible WIDEn-N configurado desde `config.json`:

```json
"digi": {
  "enabled": true,
  "alias": ["WIDE1-1", "WIDE2-2", "RELAY"],
  "comment": "ESP32-DIGI"
}
```

- Soporta múltiples aliases (hasta 8).
- Inserción del indicativo del digipeater en el path (H-bit=1).
- Supresión de duplicados por hash FNV-1a con TTL de 30 segundos (32 slots en anillo).
- Prevención de bucles: si el propio indicativo (H=1) ya aparece en el path, descarta.
- Logging serie: `I (t) digi: SRC-N digipeated via alias WIDE1-1 → CALL-0 (len=...)`.
- Notificación WebSocket `{"type":"digipeated","src":...,"dst":...,"path":...,"info":...}` → badge DIGI en la UI.

## Baliza morse CW

Identificación CW periódica configurable desde `config.json`:

```json
"morse": {
  "enabled": true,
  "tone_hz": 1000,
  "wpm": 20,
  "period_s": 600
}
```

- El indicativo CW siempre se toma de `aprs.callsign`.
- Se despacha desde `receive_audio_task` (propietaria del I2S0) para respetar el mutex ADC/DAC. El mecanismo de despacho usa `afsk_set_dispatch_hook()` para mantener la librería LibAPRS libre de dependencias del proyecto.
- Activable on-demand con `POST /api/morse/trigger` o el botón "Morse" en la UI web.

## SSTV (Slow-Scan Television)

El firmware incluye un transmisor SSTV completo. Soporta los modos más habituales en HF/VHF:

| Modo | Dimensión | Duración aprox. |
|------|-----------|-----------------|
| Martin M1  | 320×256 | ~114 s |
| Martin M2  | 320×256 |  ~58 s |
| Scottie S1 | 320×256 | ~110 s |
| Robot 36   | 320×240 |  ~36 s |
| Robot 72   | 320×240 |  ~72 s |

### Flujo de transmisión

1. Sube una imagen JPEG desde la UI web (o vía `POST /api/sstv/upload`).
2. Selecciona modo y pulsa enviar (`POST /api/sstv/send`), o usa el patrón de prueba SMPTE (`POST /api/sstv/test`).
3. La petición se encola y se despacha desde `receive_audio_task` (propietaria del DAC I2S0).
4. El firmware decodifica el JPEG on-the-fly usando el decodificador TJpgDec de la ROM del ESP32, convierte cada línea a FSK (1500–2300 Hz) a 48 kHz y lo escribe al DAC por DMA.
5. El PTT se activa automáticamente durante la transmisión.
6. El botón **■ Stop TX** de la UI (o `POST /api/sstv/stop`) aborta la transmisión al final de la línea en curso (≤ 450 ms para Martin M1). El firmware baja el PTT, vuelve al modo RX y envía `{"type":"sstv_aborted"}` por WebSocket.

### Configuración SPIFFS

Las imágenes se almacenan en `/spiffs/sstv/` (máx. 10 ficheros × 200 KB). El directorio se crea automáticamente al arrancar.

### Limitaciones

- El JPEG debe caber en la partición SPIFFS (704 KB total, compartida con `index.html` y `config.json`).
- No se admite recepción SSTV (solo TX).
- La resolución de entrada es libre; TJpgDec escala al tamaño del modo seleccionado.

## Repetidor de voz analógico

El firmware incluye un repetidor de voz analógico completo para FM/VHF. Graba el audio de entrada, lo retransmite cuando el canal queda libre y envía una identificación CW tras la retransmisión.

### Principio de funcionamiento

1. El **squelch HFNE** (_High-Frequency Noise Energy_) monitoriza la energía en cuatro bins Goertzel (3 150–4 350 Hz, por encima del espectro de audio útil). La energía en esa banda es alta con solo ruido (sin portadora) y cae al recibir una señal FM: apertura del squelch → inicio de grabación.
2. La señal recibida se codifica en **IMA ADPCM** (512 B por 1 017 muestras, ratio 2:1) y se almacena en el buffer de grabación (≈ 49 KB para 10 s, según heap disponible).
3. Una vez cerrado el squelch, el firmware espera `tail_delay_ms` ms antes de retransmitir, para descartar dropouts breves.
4. La retransmisión decodifica el ADPCM, remuestrea 9 600 Hz → 48 000 Hz (×5) y lo escribe al DAC con PTT activo.
5. Opcionalmente envía un **tono de cortesía** (Hz y duración configurables) y dispara la **identificación CW** (si `cw_id: true`).
6. Un periodo de **lockout** post-TX (`lockout_ms`) evita que el eco de la propia transmisión reabra el squelch.

### Ventana mínima de grabación

Al abrirse el squelch se inicia un temporizador de 500 ms. Incluso si el squelch se cierra antes de que expire, la grabación continúa hasta que transcurran al menos 500 ms desde la última muestra con squelch abierto. Cada vez que el squelch se reabre durante la grabación, el temporizador se reinicia desde ese instante.

### Estados

| Estado | Descripción |
|--------|-------------|
| `idle` | Escuchando; squelch cerrado |
| `recording` | Squelch abierto; grabando audio |
| `tail` | Squelch cerrado; esperando `tail_delay_ms` antes de TX |
| `pending` | Buffer listo; esperando a que el canal quede libre |
| `tx` | Retransmitiendo + tono de cortesía |

### Configuración

```json
"repeater": {
  "enabled": false,
  "max_record_s": 10,
  "tail_delay_ms": 2000,
  "lockout_ms": 3000,
  "courtesy_tone_hz": 1000,
  "courtesy_tone_ms": 200,
  "cw_id": true
}
```

```json
"squelch_sf": {
  "hfne_threshold": 0.25
}
```

| Campo | Descripción |
|-------|-------------|
| `enabled` | Activa el repetidor al arrancar |
| `max_record_s` | Máximo de grabación (según heap disponible; típicamente 10 s → ≈ 49 KB ADPCM) |
| `tail_delay_ms` | Tiempo de espera tras cerrar el squelch antes de retransmitir |
| `lockout_ms` | Período de lockout post-TX para ignorar el eco propio |
| `courtesy_tone_hz` | Frecuencia del tono de cortesía (0 = sin tono) |
| `courtesy_tone_ms` | Duración del tono de cortesía |
| `cw_id` | Si `true`, dispara identificación morse CW tras cada retransmisión |
| `hfne_threshold` | Umbral HFNE [0.01–0.99]; valores más bajos → squelch más sensible |

### Modo monitor

La sección HFNE de la UI web incluye un interruptor **Monitor** que activa el squelch de forma independiente (sin habilitar el repetidor). Útil para visualizar la actividad de señal en la pestaña REPEATER sin que el firmware retransmita.

### Notas de memoria

El buffer ADPCM se asigna dinámicamente al habilitar el repetidor. Para maximizar el heap disponible, el servidor WAV (port 8080) se detiene automáticamente antes del malloc. Si el heap libre no alcanza los 28 KB de reserva operativa, el buffer se recorta al máximo allocable alineado a bloque ADPCM (512 B).

---

## Subida de ficheros a SPIFFS

La partición SPIFFS (704 KB) contiene `index.html`, `config.json` e imágenes SSTV. Se puede actualizar cualquier fichero por red sin cable serie.

### Desde la UI web

1. Abre `http://<IP-del-ESP32>/` y ve a la pestaña **SPIFFS**.
2. Elige el fichero local (p. ej. `index.html`).
3. Verifica o edita el nombre de destino en el campo de texto (se rellena automáticamente con el nombre del fichero seleccionado).
4. Pulsa **↑ Subir** — la barra de progreso refleja la subida y el estado final confirma los bytes escritos.

> Tras actualizar `index.html` recarga la página en el navegador (Ctrl+F5) para ver la nueva versión.

### Con curl

```bash
# Actualizar la UI web
curl -X POST "http://<IP-del-ESP32>/api/spiffs/upload?name=index.html" \
     -H "Content-Type: application/octet-stream" \
     --data-binary @main/spiffs_data/index.html

# Subir un fichero de configuración
curl -X POST "http://<IP-del-ESP32>/api/spiffs/upload?name=config.json" \
     -H "Content-Type: application/octet-stream" \
     --data-binary @main/spiffs_data/config.json
```

Respuesta en caso de éxito: `{"ok":true,"name":"index.html","size":98432}`.

### Notas

- Tamaño máximo por fichero: **512 KB** (limitado por RAM del handler; la partición total es 704 KB compartida).
- El nombre de destino solo puede contener caracteres sin `/` ni `..`. No hace falta incluir `/spiffs/` — se añade automáticamente.
- El fichero se sobrescribe si ya existe.
- La partición de firmware **no se toca** (para eso existe la pestaña OTA).

---

## Actualización OTA de firmware

El firmware soporta actualización Over-The-Air desde el navegador o con `curl`, sin necesidad de cable serie ni reflashear SPIFFS.

### Desde la UI web

1. Abre `http://<IP-del-ESP32>/` y ve a la pestaña **OTA**.
2. Selecciona el `.bin` generado por ESP-IDF (`build/esp32-aprs-modem.bin`).
3. Pulsa **Flashear** — la barra de progreso refleja la subida. Los eventos `ota_progress` llegan por WebSocket.
4. El firmware valida el magic byte (`0xE9`), escribe en la partición inactiva y reinicia automáticamente en 3 s.

### Con curl

```bash
curl -X POST http://<IP-del-ESP32>/api/ota/upload \
     -H "Content-Type: application/octet-stream" \
     --data-binary @build/esp32-aprs-modem.bin
```

Respuesta en caso de éxito: `{"ok":true}`. El ESP32 reinicia 3 s después.

### Detalles de implementación

- La partición SPIFFS (`config.json`, imágenes SSTV) **no se toca**; solo cambia la partición de aplicación inactiva.
- Si el `.bin` no empieza por el magic byte ESP32 (`0xE9`), la subida se rechaza antes de escribir nada.
- No se admiten subidas concurrentes: una segunda petición mientras hay OTA en curso recibe `503`.
- No hay rollback automático configurado (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` no activo); si el nuevo firmware no arranca, hay que reflashear por serie.

---

## GPS (NMEA por UART2)

El firmware incluye un receptor GPS por UART2 que parsea sentencias NMEA estándar.

### Conexión hardware

| Señal | GPIO |
|-------|------|
| UART2 RX (datos del GPS) | GPIO 16 (pin SD DAT2 liberado) |
| UART2 TX (configuración del GPS, opcional) | GPIO 4 (pin SD DAT1 liberado) |

### Protocolo y parsing

- Baud rate por defecto: 9600 (configurable en `config.json`).
- Sentencias parseadas: `$GPRMC` / `$GNRMC` (posición, velocidad, rumbo, hora, fecha) y `$GPGGA` / `$GNGGA` (calidad de fix, número de satélites).
- Checksum NMEA XOR verificado en cada sentencia antes de parsear.
- Los datos se almacenan en la struct global `g_gps_pos` (definida en `gps.h`), protegida por un mutex FreeRTOS. Para leer desde otra tarea:

```c
gps_lock_pos();
GpsPosition pos = g_gps_pos;   // copia bajo mutex
gps_unlock_pos();
if (pos.valid) { /* usar pos.lat, pos.lon, etc. */ }
```

Campos relevantes de `GpsPosition`:

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `valid` | `bool` | `true` si el RMC tiene status `A` (posición válida) |
| `data_received` | `bool` | `true` en cuanto se recibe la primera sentencia NMEA con checksum correcto |
| `lat` / `lon` | `double` | Coordenadas en grados decimales (+N/+E) |
| `satellites` | `uint8_t` | Número de satélites (del GGA) |
| `fix_quality` | `uint8_t` | Calidad GGA (0=sin fix, 1=GPS, 2=DGPS) |
| `time_utc` | `char[7]` | Hora UTC `"HHMMSS\0"` |
| `date_utc` | `char[7]` | Fecha UTC `"DDMMYY\0"` |

### Logging serie

La tarea GPS produce los siguientes mensajes en el monitor serie:

| Evento | Nivel | Mensaje ejemplo |
|--------|-------|-----------------|
| Primera sentencia NMEA válida | `printf` | `GPS: primera trama NMEA válida recibida` |
| Cambio de satélites / calidad | `printf` | `GPS sats:8  fix:3D  (quality=1)` |
| Primer fix | `printf` | `GPS primer fix: 123456 UTC  lat=40.41680  lon=-3.70380` |
| Actualización de posición | `printf` | `GPS: 123456 UTC  lat=40.41680  lon=-3.70380` |
| Pérdida de fix | `printf` | `GPS: fix perdido` |
| Cada sentencia NMEA (debug) | `ESP_LOGD` | `$GPRMC,...*hh` |

Para ver las sentencias NMEA crudas activa el nivel debug para el tag `gps`:
```c
esp_log_level_set("gps", ESP_LOG_DEBUG);
```
O establece `CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y` en `menuconfig`.

### Configuración

```json
"gps": {
  "enabled": true,
  "baud": 9600,
  "use_for_beacon": false
}
```

- `enabled: false` deshabilita UART2 y la tarea GPS; el firmware arranca igualmente.
- `use_for_beacon`: reservado para uso futuro (tomar lat/lon del GPS en balizas APRS automáticas).
- La tarea GPS (`gps_task`) tiene prioridad 4, stack 2048 B.

---

## Display LCD SSD1306 128×64 I2C

El firmware incluye un driver SSD1306 propio (sin librería externa) y una tarea de refresco a 2 Hz que muestra el estado del sistema en pantalla.

### Conexión hardware

| Señal I2C | GPIO |
|-----------|------|
| SDA | GPIO 13 |
| SCL | GPIO 19 |

Dirección I2C por defecto: `0x3C` (60 decimal). Algunos módulos usan `0x3D` (61).

### Layout de pantalla

```
+----------------------------+
| EA1JBS-11                  |  fila 0 — callsign
| 192.168.1.100              |  fila 1 — IP WiFi (o "no wifi")
| 40.41680N   3.70380W       |  fila 2 — coords GPS (o "GPS: sin fix" / "GPS: sin datos")
| Sats:08  12:34:56Z         |  fila 3 — satélites + hora UTC (vacía si no hay fix)
| APRS:12/5 48/31            |  fila 4 — stats: CRC-ok v1/v2  HDLC-flags v1/v2
|····························|  fila 5 — separador
| RX: [████░] 87             |  fila 6 — barra 80 px + nivel numérico
| (en silencio)              |  fila 7 — "LOUD" solo si pico > umbral alto
+----------------------------+
```

- Fila 2 diferencia tres estados: coordenadas (fix válido), `"GPS: sin fix"` (sentencias NMEA recibidas pero sin posición), `"GPS: sin datos"` (UART mudo o módulo no conectado).
- La pantalla se actualiza automáticamente cada 500 ms.
- Mientras el WiFi está conectando se muestra una pantalla de estado con SSID, número de red (N/M si hay varias), tiempo restante y barra de progreso. Si se pasa al modo hotspot se muestra el SSID del AP y la IP `192.168.4.1`.
- Si el SSD1306 no responde al arrancar (`i2c_master_probe` falla), el display se deshabilita y el firmware continúa sin bloquearse.

### Configuración

```json
"display": {
  "enabled": true,
  "i2c_addr": 60
}
```

- `i2c_addr`: dirección I2C en decimal (60 = 0x3C, 61 = 0x3D).
- `enabled: false` omite la inicialización I2C y la tarea de refresco.
- La tarea display (`display_task`) tiene prioridad 2, stack 2048 B.

---

## Gateway IP sobre radio (opcional)

El ESP32 puede actuar como gateway entre la red WiFi y la frecuencia de radio. Soporta dos modos:

| Modo | `ip.mode` | Descripción |
|------|-----------|-------------|
| **TUN** (por defecto) | `"tun"` | Compatible con `tncattach` de markqvist (TCP). El payload KISS es la cabecera TUN de Linux (`00 00 08 00`) + paquete IPv4 raw. **Verificado en hardware.** |
| **AX.25** | `"ax25"` | RFC 1226 clásico: encapsula IP en tramas AX.25 UI con PID=0xCC. Compatible con `kissattach` (ax25-tools). |

Para activarlo: editar `config.json` vía la pestaña CONFIG de la UI (o reflashear SPIFFS):
```json
"ip": {
  "enabled": true,
  "mode": "tun",
  "addr": "44.61.3.75",
  "netmask": "255.255.255.0",
  "gateway": "44.61.3.1"
}
```

**Nota**: requiere `CONFIG_LWIP_IP_FORWARD=y` (incluido en `sdkconfig.defaults`). MTU máximo: 300 bytes.

## Retardo TX tras recepción (`post_rx_tx_delay_ms`)

Cuando el ESP32 recibe una trama y necesita responder (ACK automático, paquete IP de vuelta, trama KISS reencolada), la radio remota acaba de terminar de transmitir y necesita tiempo para soltar el PTT y conmutar su electrónica de TX a RX. Si el ESP32 responde inmediatamente, la radio remota aún no está escuchando y la respuesta se pierde.

El parámetro `aprs.post_rx_tx_delay_ms` en `config.json` introduce una ventana de inhibición: el dispatcher de TX de `receive_audio_task` no despacha **ninguna** trama encolada hasta que hayan transcurrido N ms desde la última trama decodificada. Los beacons periódicos (Morse, SSTV) **no** se ven afectados — solo las respuestas a tráfico recibido.

### Desglose del tiempo con dos Baofeng UV-5R

Ensayado con dos ESP32 + Baofeng UV-5R, modo `ip.mode: "tun"` y `ping 44.61.3.71 -W 5 -i 10`:

| Contribución | Tiempo | Origen |
|---|---|---|
| Flags de cola pendientes (tail) | ~47 ms | La radio remota detecta fin de trama en el **primer** flag 0x7E de cierre, pero el firmware envía 8 flags de cola (`custom_tail = 50 → DIV_ROUND = 8 bytes`); quedan ~7 × 6,7 ms en el aire |
| Drenado DMA (`wait_dac_drain`) | ~84–168 ms | El pipeline DMA del DAC drena el bloque de silencio extra antes de que caiga el PTT |
| Conmutación firmware DAC→ADC | ~10 ms | `dac_continuous_del_channels` + `adc_peripheral_start` |
| **Conmutación hardware TX→RX del Baofeng UV-5R** | **~650–750 ms** | Término dominante: liberación del PA, relé de antena, encendido de la cadena RX y asentamiento del squelch |
| **Total** | **~800–975 ms** | Tiempo desde que el callback `on_ax25_raw_frame` se ejecuta hasta que la radio remota puede recibir |

El preámbulo que envía nuestro ESP32 antes de la respuesta (53 flags × 6,7 ms ≈ **353 ms**) actúa como margen de sincronización: la radio remota solo necesita recibir ~4–5 flags (27 ms) para sincronizar su PLL. Con `post_rx_tx_delay_ms = 950` el margen total disponible es ~436 ms.

Con `post_rx_tx_delay_ms = 500` el preámbulo termina ~57 ms antes de que el Baofeng complete su conmutación TX→RX → la sincronización falla de forma intermitente.

### Valores recomendados

| Radio | Valor recomendado |
|-------|------------------|
| Baofeng UV-5R / UV-82 y similares | **950 ms** |
| Radios con conmutación rápida (< 200 ms) | 200–300 ms |
| Sin radio externa (pruebas en banco) | 0 (deshabilitar) |

```json
"aprs": {
  "post_rx_tx_delay_ms": 950
}
```

Ponlo a `0` para deshabilitar el retardo completamente.

---

## Conectar al host

### Con tncattach (gateway IP bidireccional, verificado)

Usa la versión TCP de [markqvist/tncattach](https://github.com/markqvist/tncattach) con modo `"tun"` activo en `config.json`:

```bash
# Instalar tncattach
git clone https://github.com/markqvist/tncattach && cd tncattach && make && sudo make install

# Crear interfaz TUN (asigna IP al host en la misma /24 que el ESP32)
sudo ./tncattach -T -H <ip_wifi_del_esp32> -P 8001 --mtu 250 --noipv6 --ipv4 44.61.3.73/24

# El ESP32 tiene 44.61.3.75 → ping de prueba:
ping 44.61.3.75

# Capturar tráfico RF
sudo tcpdump -i tnc0 -n
```

El log del ESP32 mostrará por cada ping:
```
I ax25ip: RX 84 bytes TUN-IP←RF → lwIP
I ax25ip: TX 84 bytes TUN→RF
```

> **⚠️ Usa siempre `/24`, no `/32`.**  
> Con `--ipv4 44.61.3.73/32` el kernel solo añade una ruta de host para `44.61.3.73`; no hay ruta para ninguna otra dirección `44.61.3.x`, así que los paquetes al ESP32 remoto (`44.61.3.71`, por ejemplo) se envían por el gateway por defecto (WiFi/Ethernet) en lugar de por `tnc0` y el ping nunca llega.  
> Con `/24` el kernel añade automáticamente `44.61.3.0/24 dev tnc0` y todo el bloque queda accesible por radio.  
> Si necesitas `/32` por alguna razón, añade la ruta manualmente:
> ```bash
> sudo ip route add 44.61.3.71 dev tnc0
> ```

### Con direwolf (iGate / gateway APRS)

```
# direwolf.conf:
ADEVICE tcp:<ip_del_esp32> 8001
ACHANNEL 0 1200
MYCALL NO0CAL-1
```

```bash
direwolf -c direwolf.conf
```

## Configuración antes de flashear

Copia `main/spiffs_data/config.json.example` a `main/spiffs_data/config.json` y rellena tus datos. La estructura completa con todos los campos es:

```json
{
  "version": "1",
  "aprs": {
    "callsign": "TU_INDICATIVO",
    "ssid": 11,
    "symbol_table": "/",
    "symbol_code": ">",
    "post_rx_tx_delay_ms": 950
  },
  "wifi": [
    {
      "ssid": "MI_RED_WIFI",
      "password": "MI_PASSWORD",
      "connect_timeout_s": 30
    }
  ],
  "ap": {
    "enabled": true,
    "ssid": "APRS_AP",
    "password": "12345678"
  },
  "ip": {
    "enabled": false,
    "mode": "tun",
    "addr": "44.61.3.75",
    "netmask": "255.255.255.0",
    "gateway": "44.61.3.1"
  },
  "digi": {
    "enabled": false,
    "alias": ["WIDE1-1", "WIDE2-2", "RELAY"],
    "comment": "ESP32-DIGI"
  },
  "morse": {
    "enabled": false,
    "tone_hz": 1000,
    "wpm": 20,
    "period_s": 600
  },
  "gps": {
    "enabled": true,
    "baud": 9600,
    "use_for_beacon": false
  },
  "display": {
    "enabled": true,
    "i2c_addr": 60
  },
  "rx": {
    "dual_modem_enabled": true,
    "active_modem": "best",
    "squelch_threshold": 0,
    "deemphasis_enabled": false
  },
  "remote_cmd": {
    "enabled": true
  }
}
```

### Referencia de campos

| Sección | Campo | Tipo | Descripción |
|---------|-------|------|-------------|
| `aprs` | `callsign` | string | Indicativo (máx. 6 chars) |
| `aprs` | `ssid` | int 0–15 | SSID APRS |
| `aprs` | `symbol_table` | `"/"` o `"\\"` | Tabla de símbolos APRS (primaria o alternativa) |
| `aprs` | `symbol_code` | char | Código de símbolo APRS (p. ej. `">"` = coche) |
| `aprs` | `post_rx_tx_delay_ms` | int | Retardo TX tras recepción (ms). Tiempo mínimo desde que se decodifica una trama hasta que se despacha la siguiente TX. Compensa el tiempo de conmutación TX→RX de la radio remota. `0` = deshabilitado. Valor recomendado con Baofeng UV-5R: **950** |
| `wifi` | — | array | Lista de redes WiFi; se intenta cada una en orden circular |
| `wifi[n]` | `connect_timeout_s` | int | Tiempo máximo de espera por red (s) |
| `ap` | `enabled` | bool | Activa modo hotspot si no conecta a ninguna red WiFi |
| `ip` | `enabled` | bool | Activa gateway IP sobre radio |
| `ip` | `mode` | string | `"tun"` (tncattach, verificado) o `"ax25"` (RFC 1226 / kissattach) |
| `digi` | `alias` | array | Aliases a digipeatear (WIDEn-N y aliases legacy) |
| `digi` | `comment` | string | Comentario libre (informativo, no transmitido) |
| `morse` | `period_s` | int | Intervalo entre balizas CW (segundos) |
| `gps` | `baud` | int | Velocidad UART del módulo GPS |
| `gps` | `use_for_beacon` | bool | Reservado: usar posición GPS en balizas APRS (futuro) |
| `display` | `i2c_addr` | int | Dirección I2C del SSD1306 en decimal (60 = 0x3C) |
| `rx` | `dual_modem_enabled` | bool | Activa el segundo demodulador AFSK (V2) |
| `rx` | `active_modem` | string | `"v1"`, `"v2"` o `"best"` (ambos en paralelo, sin dedup) |
| `rx` | `squelch_threshold` | int 0–127 | Umbral de squelch del demodulador V2 (0 = abierto) |
| `rx` | `deemphasis_enabled` | bool | Activa filtro de de-énfasis en el demodulador V2 |
| `remote_cmd` | `enabled` | bool | Activa procesado de comandos remotos vía mensaje APRS dirigido al indicativo |

> **Nota sobre `rx.active_modem: "best"`**: en este modo ambos demoduladores decodifican de forma independiente. Si el mismo paquete lo decodifican los dos, el host KISS recibirá dos tramas idénticas. El host (tncattach, direwolf) debe manejar los duplicados si es necesario.

`main/config.h` mantiene parámetros de compilación (`TNC_MODE`, `KISS_TRANSPORT`, `KISS_TCP_PORT`, GPIOs).

## Limitaciones actuales

- **RX verificación RF** — el demodulador funciona en banco de pruebas; pendiente validación con señal RF real de un transceptor.
- **Repetidor sin verificación en hardware real** — implementado y compilado limpio; pendiente prueba con radio FM real.
- **SSTV solo TX** — no hay decodificador RX.
- **GPS y display sin verificación en hardware real** — implementados y compilados limpios; pendiente prueba con módulo GPS real y pantalla SSD1306 conectada.
- **SD e I2C comparten GPIO13** — GPIO13 es el CS/DAT3 de la SD y también el SDA del bus I2C (SSD1306). No se puede usar la tarjeta SD si el display está conectado.
- **Un solo cliente TCP KISS a la vez** — el servidor acepta reconexiones, pero no conexiones simultáneas.
- **FIFOs internos** (report.md §2.6) — protegidos con `portMUX_TYPE`; riesgo residual bajo en escenarios de alta carga.
- **`esp-dsp`** sigue declarado en `idf_component.yml` sin uso activo — alarga el build innecesariamente.

## Indicativo y licencia de radioaficionado

Editar `main/spiffs_data/config.json` (`aprs.callsign` y opcionalmente `aprs.ssid`) con tu propio indicativo antes de transmitir. También se puede cambiar en runtime desde la pestaña CONFIG de la UI sin reflashear. **Transmitir en la banda amateur requiere licencia válida**.

## Pines
### Mapeo de pines — Lolin32 Lite + placa ESPRI

Según el esquema de la placa ESPRI y el pinout de la Lolin32 Lite, estos son los GPIO usados y los que quedarían disponibles.

---

### Pines del firmware (device.h / config.h)

Estos son los GPIO **configurados en el código** y los que usa activamente el firmware:

| GPIO | Definición | Función | Notas |
|------|-----------|---------|-------|
| GPIO25 | DAC_CHAN_0 | Audio TX — salida DAC a radio | DAC1 del ESP32, 8-bit |
| GPIO35 | ADC1_CH7  | Audio RX — entrada ADC desde radio | Solo entrada; 12-bit; 0–3,1 V (atten 12 dB) |
| GPIO26 | `GPIO_PTT_OUT` | PTT activo alto (1=TX, 0=reposo) | Comparte recurso con DAC2; ver trampas en CLAUDE.md |
| GPIO33 | `GPIO_LED_RX`  | LED verde — parpadea al decodificar un paquete AX.25 | Controlado por AFSK.cpp (LED_RX_ON/OFF) |
| GPIO23 | `GPIO_LED_WARN` | LED rojo — señal de audio fuera de rango (demasiado alta) | Parpadeante mientras dure la condición |
| GPIO37 | `GPIO_AUDIO_TRIGGER` | Trigger de audio (sin uso activo) | Solo entrada |
| GPIO13 | `GPIO_I2C_SDA` | I2C SDA — bus SSD1306 y periféricos I2C | |
| GPIO19 | `GPIO_I2C_SCL` | I2C SCL — bus SSD1306 y periféricos I2C | |
| GPIO16 | `GPIO_GPS_RX`  | UART2 RX — datos NMEA del módulo GPS | Era SD DAT2 (no conectar SD si se usa GPS) |
| GPIO4  | `GPIO_GPS_TX`  | UART2 TX — configuración módulo GPS (opcional) | Era SD DAT1 |

GPIO_LED_TX está desactivado (`-1`) en `config.h`. GPIO22 (LED integrado Lolin32) está ahora libre.

---

### Pines usados por la ESPRI (esquema de placa)

Conexiones del hardware ESPRI. Los pines de audio y PTT coinciden con la configuración del firmware.

| GPIO ESP32 | Señal en esquema | Función |
|---|---|---|
| GPIO35 | `AUDIO IN PIN` | Entrada de audio desde radio (ADC1_CH7) |
| GPIO34 | `BATTERY MEASURE PIN` | Lectura ADC de batería (no usada por firmware) |
| GPIO32 | `LED PIN` | LED verde — no usado por firmware |
| GPIO33 | `LED2 PIN` | LED (verde en HW del usuario) — **usado como `GPIO_LED_RX`**: parpadea al decodificar cada paquete AX.25 |
| GPIO25 | `AUDIO OUT PIN` | Salida de audio a radio (DAC1) |
| GPIO26 | `PTT PIN` | Control PTT radio |
| GPIO27 | `RX PIN` | RX UART desde radio |
| GPIO14 | `TX PIN` | TX UART hacia radio |
| GPIO12 | — | Strapping boot (flash voltage) — no conectado a SD |
| GPIO13 | `SD CARD DAT3 / CS` | SD en modo SPI — **ahora usado como `GPIO_I2C_SDA`** |
| GPIO15 | `SD CARD CMD / MOSI` | SD en modo SPI — strapping boot |
| GPIO2  | `SD CARD DAT0 / MISO` | SD en modo SPI — strapping boot |
| GPIO18 | `SD CARD CLK / SCK` | SD en modo SPI |
| GPIO5  | `SD CARD ENABLE` | Alimentación/enable SD — no libre |
| GPIO0  | `TOUCH PAD 1` | Entrada táctil / strapping boot |
| GPIO4  | — | GPIO libre — **usado como GPS UART2 TX** |
| GPIO6  | `TOUCH SENSOR` | Sensor táctil — no libre |
| GPIO16 | — | GPIO libre — **usado como GPS UART2 RX** |
| GPIO17 | `TOUCH SENSOR` | Sensor táctil — no libre |

---

### Interfaces funcionales

#### Audio

| Función | GPIO |
|---|---|
| Audio OUT (DAC) | GPIO25 |
| Audio IN (ADC)  | GPIO35 |

- GPIO25 usa el DAC1 interno del ESP32 (`dac_continuous`).
- GPIO35 = ADC1_CH7; es solo entrada; rango 0–3,1 V con atenuación 12 dB.

---

#### UART radio

| Función | GPIO |
|---|---|
| TX hacia radio | GPIO14 |
| RX desde radio | GPIO27 |
| PTT | GPIO26 |

---

#### I2C (SSD1306 y periféricos)

| Función | GPIO |
|---------|------|
| SDA | GPIO13 |
| SCL | GPIO19 |

- Bus I2C0, 400 kHz. Dirección SSD1306: 0x3C (60) o 0x3D (61).
- GPIO23 tiene un LED en la placa; se usa como `GPIO_LED_WARN` (LED rojo de advertencia de volumen alto), por eso GPIO19 se usa como SCL en lugar de GPIO23.

---

#### GPS (UART2)

| Función | GPIO |
|---------|------|
| UART2 RX (datos GPS) | GPIO16 |
| UART2 TX (config GPS, opcional) | GPIO4 |

- Velocidad por defecto: 9600 baud. Configurable en `config.json` (`gps.baud`).
- GPIO16 y GPIO4 son GPIOs libres en la Lolin32 Lite (no forman parte del circuito SD de la ESPRI).

---

#### Tarjeta SD

La SD está conectada en modo SPI:

| Señal SD | GPIO |
|---|---|
| Enable (alimentación) | GPIO5 |
| CS / DAT3 | GPIO13 |
| MOSI / CMD | GPIO15 |
| MISO / DAT0 | GPIO2 |
| SCK / CLK | GPIO18 |

> GPIO15, GPIO2 y GPIO0 son pines de strapping/boot. GPIO13 está reasignado a I2C SDA en el firmware — no se puede usar la SD si el display SSD1306 está conectado.

---

#### LEDs

| LED | GPIO | Uso |
|-----|------|-----|
| Verde (`GPIO_LED_RX`) | GPIO33 | Parpadea al decodificar cada paquete AX.25 |
| Rojo (`GPIO_LED_WARN`) | GPIO23 | Enciende/parpadea cuando el audio de entrada está demasiado alto |
| Lolin32 built-in | GPIO22 | Libre — no usado por firmware |
| Verde ESPRI | GPIO32 | Libre — no usado por firmware |

---

#### Batería

| Función | GPIO |
|---|---|
| Medición batería ADC | GPIO34 |
| Enable divisor medida | GPIO18 |

---

### Pines libres disponibles

#### Pines probablemente libres y utilizables

| GPIO | Comentarios |
|---|---|
| GPIO22 | Libre (LED integrado Lolin32 — no usado por firmware) |
| GPIO32 | Libre (LED verde ESPRI — no usado por firmware) |

> GPIO33 → **LED verde RX** (`GPIO_LED_RX`). GPIO23 → **LED rojo WARN** (`GPIO_LED_WARN`). GPIO19 → **I2C SCL** (SSD1306). GPIO13 → **I2C SDA** (SSD1306). GPIO16 → **GPS UART2 RX**. GPIO4 → **GPS UART2 TX**. Todos asignados en `config.h`.

#### Usos recomendados para los pines libres restantes
- SPI auxiliar
- UART extra
- Relés/transistores

---

### Pines con restricciones o no recomendados

| GPIO | Motivo |
|---|---|
| GPIO0 | Strapping boot + SD touch pad |
| GPIO2 | Strapping boot + SD MISO |
| GPIO5 | SD Enable — no libre |
| GPIO6 | Sensor táctil ESPRI — no libre |
| GPIO12 | Strapping boot (flash voltage) |
| GPIO15 | Strapping boot + SD MOSI |
| GPIO17 | Sensor táctil ESPRI — no libre |
| GPIO21 | No expuesto en la Lolin32 Lite |
| GPIO34 | Solo entrada |
| GPIO35 | Solo entrada |
| GPIO36 (VP) | Solo entrada |
| GPIO39 (VN) | Solo entrada |

---

### Pines libres “solo entrada”

| GPIO | Uso típico |
|---|---|
| GPIO36 (VP) | ADC/sensores |
| GPIO39 (VN) | ADC/sensores |

#### Limitaciones
No sirven para:
- I2C
- SPI
- UART TX
- LEDs
- Salidas digitales

---

### Resumen rápido

#### Usados / ocupados (ESPRI / hardware)
- GPIO0 (strapping/touch), GPIO2 (strapping + SD MISO), GPIO5 (SD Enable)
- GPIO6 (touch sensor), GPIO12 (strapping), GPIO13\* (SD CS → I2C SDA), GPIO14 (TX radio)
- GPIO15 (strapping + SD MOSI), GPIO17 (touch sensor), GPIO18 (SD CLK)
- GPIO21 (no expuesto en Lolin32 Lite)
- GPIO25 (DAC audio TX), GPIO26 (PTT), GPIO27 (RX UART radio)
- GPIO32 (LED ESPRI — no usado por firmware), GPIO33 (**`GPIO_LED_RX`**, usado por firmware)
- GPIO34 (batería ADC), GPIO35 (audio ADC — activo en firmware)

\*GPIO13 es SD CS reasignado a I2C SDA. GPIO4 y GPIO16 son GPIOs libres usados por GPS UART2.

#### Usados por firmware
- GPIO25, GPIO26, GPIO35 (audio TX/PTT/RX)
- GPIO33 (LED_RX verde), GPIO23 (LED_WARN rojo)
- GPIO13, GPIO19 (I2C SDA/SCL — SSD1306)
- GPIO16, GPIO4 (GPS UART2 RX/TX)

#### Libres “buenos”
- GPIO22, GPIO32

#### Libres solo entrada
- GPIO36 (VP), GPIO39 (VN)

## Licencia

El código heredado de LibAPRS mantiene su licencia original (ver [main/LibAPRS-esp32-i2s/LICENSE](main/LibAPRS-esp32-i2s/LICENSE)). Las adaptaciones de este repositorio se distribuyen bajo los mismos términos.

## Créditos

- [Mark Qvist](https://github.com/markqvist) — LibAPRS original.
- [handiko](https://github.com/handiko) — port a ESP32 con I2S.
- Francesco Sacchi — código base de decodificación AX.25.
- [kamilsss655](https://github.com/kamilsss655) — Hardware https://github.com/kamilsss655/ESPRI

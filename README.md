# esp32-aprs-modem

Módem APRS / KISS TNC (AX.25 sobre AFSK Bell-202, 1200 bps) para **ESP32**, basado en ESP-IDF v6.1.

Uso la placa ESPRI (de https://github.com/kamilsss655/ESPRI)

(Creado dando latigazos a Claude y otros...)

> ✅ **Estado (2026-05-18): compila limpio (binary ~958 KB, 44 % libre). KISS TNC bidireccional operativo. TX verificado en hardware. UI web funcional con log, mensajes, baliza de posición, editor de configuración y grabación de audio. Digipeater WIDEn-N operativo. Baliza morse CW periódica operativa. TX SSTV implementado (Martin M1/M2, Scottie S1, Robot 36/72) con carga de JPEG vía web. GPS NMEA por UART2 implementado. Display SSD1306 I2C 128×64 implementado. Gateway IP RFC 1226 implementado. RX pendiente verificación con señal RF real.**

El firmware opera como un **KISS TNC bidireccional** accesible desde la red local vía TCP. Conecta `tncattach` o `direwolf` en el host y obtienes una interfaz de red AX.25 (`tnc0`) o un gateway APRS completo — sin cable USB, sin drivers adicionales.

Este repositorio adapta [LibAPRS-esp32-i2s](https://github.com/handiko/LibAPRS-esp32-i2s) — fork de [LibAPRS](https://github.com/markqvist/LibAPRS) de markqvist — para funcionar bajo ESP-IDF usando el DAC interno y ADC continuo por DMA.

## Hardware objetivo

- **MCU**: ESP32 (clásico, Xtensa LX6 dual-core).
- **Salida de audio (TX)**: DAC1 en GPIO 25 (`dac_continuous`, muestras 8-bit directas). Sin filtro externo necesario para la mayoría de transceptores.
- **Entrada de audio (RX)**: GPIO 35 / ADC1_CH7 (`adc_continuous` DMA a 48 kHz, atenuación 12 dB → rango 0–3,1 V). Entrada AC-acoplada desde la salida de altavoz del transceptor.
- **PTT**: GPIO 26, activo en nivel alto (1 = transmitir, 0 = reposo). Restricción de hardware (también es DAC2). Definido en `config.h`.
- **GPS**: UART2 a 9600 baud. RX = GPIO 16 (pin SD DAT2 liberado), TX = GPIO 4 (pin SD DAT1 liberado, opcional para configurar el módulo GPS).
- **Display LCD**: SSD1306 128×64 OLED por I2C. SDA = GPIO 21, SCL = GPIO 19.

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
│   ├── rx_stats.h / rx_stats.c        estadísticas de recepción por demodulador; /api/rx/stats
│   ├── gps.h / gps.c                  receptor GPS por UART2 (NMEA $GPRMC/$GPGGA),
│   │                                  struct g_gps_pos con mutex FreeRTOS
│   ├── display.h / display.c          driver SSD1306 I2C 128×64 (sin librería externa),
│   │                                  task de refresco 2 Hz con callsign, IP, GPS, audio
│   ├── ax25ip.h / ax25ip.c            gateway IP RFC 1226 (lwIP custom netif, PID=0xCC)
│   ├── aux_config.h / aux_config.c    carga/guarda config JSON desde SPIFFS (/spiffs/config.json)
│   ├── aux_file_management.h / .c     utilidades de sistema de ficheros SPIFFS
│   ├── spiffs_data/config.json        configuración inicial (callsign, WiFi, AP, IP, digi, morse)
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

- **ESP-IDF v6.1** (usa `dac_continuous`, `adc_continuous`, `esp_wifi`, `esp_netif`, `spiffs`, `esp_http_server`, `esp_timer`).
- Componentes IDF requeridos (declarados en `main/CMakeLists.txt`):
  `esp_wifi`, `nvs_flash`, `esp_netif`, `lwip`, `driver`, `esp_driver_dac`, `esp_driver_gpio`, `esp_driver_uart`, `esp_driver_i2c`, `esp_adc`, `spiffs`, `esp_http_server`, `vfs`, `espressif__cjson`, `esp_timer`.

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
2. `transport_init(&transport_wifi_ops)` conecta a la red WiFi configurada e imprime la IP asignada.
3. `ax25ip_init()` activa el gateway IP RFC 1226 si `ip.enabled: true` en config.json.
4. `kiss_init(on_kiss_frame)` registra el callback que transmite por radio las tramas recibidas del host.
5. `APRS_init()` + `APRS_set_raw_hook(on_ax25_raw_frame)` arranca el demodulador AFSK y registra el callback que:
   - Envía la trama al host KISS TCP (`kiss_send_frame`).
   - Detecta mensajes dirigidos a nuestro indicativo y transmite un ACK automático (`try_auto_ack`).
   - Digipeata la trama si procede (`digi_process_frame`) y notifica vía WebSocket con badge DIGI.
   - Inyecta paquetes IP en la pila lwIP si PID=0xCC (`ax25ip_rx_frame`).
   - Notifica a los clientes WebSocket con JSON `{"src":..., "dst":..., "path":..., "info":...}`.
6. `digi_init()` configura el digipeater WIDEn-N desde `config.json`.
7. `morse_init()` configura la baliza morse CW. `sstv_init()` crea el directorio `/spiffs/sstv` y registra los endpoints REST de SSTV. Ambos se despachan desde `receive_audio_task` mediante el hook registrado con `afsk_set_dispatch_hook()`.
8. `gps_init()` arranca la tarea FreeRTOS GPS (UART2, GPIO16/GPIO4, 9600 baud). Parsea $GPRMC y $GPGGA y actualiza la struct global `g_gps_pos` bajo mutex.
9. `display_init()` inicializa el bus I2C y el SSD1306 (GPIO21/GPIO19) y arranca la tarea de refresco del display a 2 Hz.
10. `audio_stream_init()` arranca el servidor HTTP en port 80 (UI web + WebSocket `/ws`) y el WAV server en port 8080.
11. Cuando el host envía una trama KISS → `on_kiss_frame` → `afsk_queue_tx_frame()` (encola) → `receive_audio_task` despacha → `APRS_send_raw_frame()` → DAC → radio.
12. `audio_level_task` genera barra de nivel y hace parpadeo rápido del LED RX cuando el pico queda fuera del rango permitido.

## Interfaz web

Navega a `http://<IP-del-ESP32>/` para acceder a la UI web integrada:

- **Log APRS**: muestra paquetes recibidos por radio en tiempo real (newest-first). Badges: `PARA MÍ` si el mensaje va dirigido a tu indicativo, `ACK` para confirmaciones, `TX` para los enviados, `DIGI` (azul) para los retransmitidos por el digipeater. Click en callsign rellena el destino.
- **Pestaña MENSAJE**: formulario para transmitir mensajes APRS directamente desde el navegador.
- **Pestaña POSICIÓN**: formulario para transmitir baliza de posición APRS (lat/lon decimal, símbolo, comentario). Botón de baliza Morse on-demand.
- **Pestaña CONFIG**: editor JSON del `config.json` completo. Guardar recarga el digipeater y la baliza morse sin reiniciar el firmware.
- **Audio**: streaming de audio de recepción en tiempo real vía WebSocket (IMA ADPCM, 9600 Hz). Botón de grabación para capturar audio y descargar como WAV. También disponible como stream WAV en `http://<IP>:8080/`.

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
| `POST`   | `/api/sstv/upload`      | Sube JPEG vía `multipart/form-data` (partes: `name` + `image`; máx. 200 KB, 10 imágenes) |
| `DELETE` | `/api/sstv/image?name=` | Elimina imagen de la galería SSTV |

## Digipeater WIDEn-N

El firmware incluye un digipeater AX.25 compatible WIDEn-N configurado desde `config.json`:

```json
"digi": {
  "enabled": true,
  "alias": ["WIDE1-1", "WIDE2-2", "RELAY"],
  "callsign": "TU_INDICATIVO",
  "ssid": 0
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
  "callsign": "",
  "tone_hz": 1000,
  "wpm": 20,
  "period_s": 600
}
```

- `callsign` vacío usa `aprs.callsign` como fallback.
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

### Configuración SPIFFS

Las imágenes se almacenan en `/spiffs/sstv/` (máx. 10 ficheros × 200 KB). El directorio se crea automáticamente al arrancar.

### Limitaciones

- El JPEG debe caber en la partición SPIFFS (704 KB total, compartida con `index.html` y `config.json`).
- No se admite recepción SSTV (solo TX).
- La resolución de entrada es libre; TJpgDec escala al tamaño del modo seleccionado.

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

### Configuración

```json
"gps": {
  "enabled": true,
  "baud": 9600
}
```

- `enabled: false` deshabilita UART2 y la tarea GPS; el firmware arranca igualmente.
- La tarea GPS (`gps_task`) tiene prioridad 4, stack 2048 B.

---

## Display LCD SSD1306 128×64 I2C

El firmware incluye un driver SSD1306 propio (sin librería externa) y una tarea de refresco a 2 Hz que muestra el estado del sistema en pantalla.

### Conexión hardware

| Señal I2C | GPIO |
|-----------|------|
| SDA | GPIO 21 |
| SCL | GPIO 19 |

Dirección I2C por defecto: `0x3C` (60 decimal). Algunos módulos usan `0x3D` (61).

### Layout de pantalla

```
+----------------------+
| EA1JBS-11  192.168.1.x  |   callsign + IP WiFi
|                      |
| GPS: 40.4168 -3.7038 |   coordenadas (o "GPS: sin fix")
| Sats:08  12:34:56UTC |   satélites + hora GPS
|                      |
|                      |
| [████░░░░░░░░░░░░░░] |   barra de nivel de audio RX
+----------------------+
```

La pantalla se actualiza automáticamente cada 500 ms. Si el SSD1306 no responde al arrancar (`i2c_master_probe` falla), el display se deshabilita y el firmware continúa sin bloquearse.

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

## Gateway IP RFC 1226 (opcional)

El ESP32 puede actuar como gateway entre la red WiFi y la frecuencia de radio, encapsulando tráfico IP en tramas AX.25 UI con PID=0xCC según RFC 1226.

Para activarlo: editar `config.json` vía la pestaña CONFIG de la UI (o reflashear SPIFFS):
```json
"ip": {"enabled": true, "addr": "44.61.3.71", "netmask": "255.255.255.0", "gateway": "44.61.3.1", "ssid": 1}
```

En el PC host:
```bash
ip route add 44.61.3.0/24 via <IP-WiFi-del-ESP32>
```

**Nota**: requiere `CONFIG_LWIP_IP_FORWARD=y` (incluido en `sdkconfig.defaults`). MTU máximo: 300 bytes (AX25_MAX_FRAME_LEN limitado a 600 por `CUSTOM_FRAME_SIZE`).

## Conectar al host

### Con tncattach (IP sobre AX.25)

```bash
# Instalar tncattach
git clone https://github.com/markqvist/tncattach && cd tncattach && make && sudo make install

# Crear interfaz de red AX.25
sudo tncattach --tcp <ip_del_esp32> 8001 --nosmall --ipv4 44.61.3.72/24

# Verificar
ip link show tnc0
sudo tcpdump -i tnc0 -n    # capturar tramas AX.25 recibidas por radio
```

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

## Configuración mínima antes de flashear

Editar `main/spiffs_data/config.json`. Estructura completa con todos los campos:

```json
{
  "aprs": {
    "callsign": "TU_INDICATIVO",
    "ssid": 11,
    "symbol_table": "/",
    "symbol_code": ">"
  },
  "wifi": [
    {
      "ssid": "TU_SSID_AQUI",
      "password": "TU_PASSWORD_AQUI",
      "connect_timeout_s": 20
    }
  ],
  "ap": {
    "enabled": true,
    "ssid": "APRS_AP",
    "password": "12345678"
  },
  "ip": {
    "enabled": false,
    "addr": "44.61.3.71",
    "netmask": "255.255.255.0",
    "gateway": "44.61.3.1",
    "ssid": 1
  },
  "digi": {
    "enabled": false,
    "alias": ["WIDE1-1", "WIDE2-2", "RELAY"],
    "callsign": "TU_INDICATIVO",
    "ssid": 0,
    "comment": "ESP32-DIGI"
  },
  "morse": {
    "enabled": false,
    "callsign": "",
    "tone_hz": 1000,
    "wpm": 20,
    "period_s": 600
  },
  "rx": {
    "dual_modem_enabled": true,
    "active_modem": "best",
    "squelch_threshold": 64,
    "deemphasis_enabled": false
  },
  "gps": {
    "enabled": true,
    "baud": 9600
  },
  "display": {
    "enabled": true,
    "i2c_addr": 60
  }
}
```

Notas:
- `wifi` es un **array**: se puede añadir más de una red; el firmware intenta cada una en orden.
- `aprs.symbol_table` / `aprs.symbol_code`: símbolo APRS (tabla primaria `/` o alternativa `\\`).
- `rx.dual_modem_enabled`: activa un segundo demodulador AFSK con filtros diferentes para mayor tasa de decodificación.
- `rx.active_modem`: `"v1"`, `"v2"` o `"best"` (usa el que tenga mejor calidad de señal en cada trama).

`main/config.h` mantiene parámetros de compilación (`TNC_MODE`, `KISS_TRANSPORT`, `KISS_TCP_PORT`, GPIOs).

## Limitaciones actuales

- **RX verificación RF** — el demodulador funciona en banco de pruebas; pendiente validación con señal RF real de un transceptor.
- **Gateway IP RFC 1226** — implementado y compila limpio; pendiente verificación en hardware real.
- **SSTV solo TX** — no hay decodificador RX. Tampoco se admite recepción SSTV.
- **GPS y display sin verificación en hardware real** — implementados y compilados limpios; pendiente prueba con módulo GPS real y pantalla SSD1306 conectada.
- **SD y GPS comparten pines** — GPIO4 (DAT1) y GPIO16 (DAT2) ahora usados por UART2 GPS. La tarjeta SD no puede usarse simultáneamente.
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
| GPIO21 | `GPIO_I2C_SDA` | I2C SDA — bus SSD1306 y periféricos I2C | |
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
| GPIO32 | `LED PIN` | LED verde (no usado por firmware) |
| GPIO33 | `LED2 PIN` | LED rojo (no usado por firmware) |
| GPIO25 | `AUDIO OUT PIN` | Salida de audio a radio (DAC1) |
| GPIO26 | `PTT PIN` | Control PTT radio |
| GPIO27 | `RX PIN` | RX UART desde radio |
| GPIO14 | `TX PIN` | TX UART hacia radio |
| GPIO12 | `SD CARD CLK PIN` | SPI clock SD |
| GPIO13 | `SD CARD DAT3 PIN` | SPI CS / DAT3 SD |
| GPIO15 | `SD CARD CMD PIN` | SPI MOSI/CMD SD |
| GPIO2  | `SD CARD DAT0 PIN` | SPI MISO/DAT0 SD |
| GPIO0  | `TOUCH PAD 1` | Entrada táctil / libre auxiliar |
| GPIO4  | `SD CARD DAT1 PIN` | DAT1 SD — **ahora usado como GPS UART2 TX** |
| GPIO16 | `SD CARD DAT2 PIN` | DAT2 SD — **ahora usado como GPS UART2 RX** |
| GPIO17 | `SD CARD ENABLE PIN` | Alimentación/enable SD |
| GPIO18 | `BATTERY MEASURE ENABLE PIN` | Habilita medición batería |

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
| SDA | GPIO21 |
| SCL | GPIO19 |

- Bus I2C0, 400 kHz. Dirección SSD1306: 0x3C (60) o 0x3D (61).
- GPIO23 tiene un LED en la placa y no se usa como SCL.

---

#### GPS (UART2)

| Función | GPIO |
|---------|------|
| UART2 RX (datos GPS) | GPIO16 |
| UART2 TX (config GPS, opcional) | GPIO4 |

- Velocidad por defecto: 9600 baud. Configurable en `config.json` (`gps.baud`).
- GPIO16 y GPIO4 son los pines de SD DAT2 y DAT1. **No usar la SD si el GPS está conectado.**

---

#### Tarjeta SD

La SD parece conectada en modo SDIO 4-bit:

| Señal SD | GPIO |
|---|---|
| CLK | GPIO12 |
| CMD | GPIO15 |
| DAT0 | GPIO2 |
| DAT1 | GPIO4 |
| DAT2 | GPIO16 |
| DAT3 | GPIO13 |
| Enable | GPIO17 |

> GPIO12, GPIO15, GPIO2 y GPIO0 son pines de strapping/boot. Hay que evitar niveles incorrectos al arrancar.

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
| GPIO5  | Libre |
| GPIO13 | Libre (era SD DAT3/CS) |
| GPIO22 | Libre (LED integrado Lolin32 — ya no usado por firmware) |
| GPIO32 | Libre (LED verde ESPRI — no usado por firmware) |

> GPIO33 → **LED verde RX** (`GPIO_LED_RX`). GPIO23 → **LED rojo WARN** (`GPIO_LED_WARN`). GPIO19 → **I2C SCL** (SSD1306). GPIO21 → **I2C SDA** (SSD1306). GPIO16 → **GPS UART2 RX**. GPIO4 → **GPS UART2 TX**. Todos asignados en `config.h`.

#### Usos recomendados para los pines libres restantes
- SPI auxiliar
- UART extra
- Relés/transistores

---

### Pines con restricciones o no recomendados

| GPIO | Motivo |
|---|---|
| GPIO0 | Strapping boot |
| GPIO2 | Strapping boot + SD |
| GPIO12 | Strapping boot |
| GPIO15 | Strapping boot |
| GPIO34 | Solo entrada |
| GPIO35 | Solo entrada |
| GPIO36 (VP) | Libre pero solo entrada |
| GPIO39 (VN) | Libre pero solo entrada |

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

#### Usados (ESPRI / hardware)
- GPIO0, GPIO2, GPIO4\*, GPIO12, GPIO13, GPIO14, GPIO15, GPIO16\*, GPIO17, GPIO18
- GPIO25 (DAC audio TX), GPIO26 (PTT), GPIO27 (RX UART)
- GPIO32, GPIO33 (LEDs ESPRI — no usados por firmware)
- GPIO34 (batería ADC), GPIO35 (audio ADC — activo en firmware)

\*GPIO4 y GPIO16 son pines SD reasignados a GPS UART2.

#### Usados por firmware
- GPIO25, GPIO26, GPIO35 (audio TX/PTT/RX — ver tabla firmware arriba)
- GPIO33 (LED_RX verde — parpadea en cada paquete AX.25 decodificado)
- GPIO23 (LED_WARN rojo — parpadea cuando el audio está demasiado alto)
- GPIO21, GPIO19 (I2C SDA/SCL — SSD1306)
- GPIO16, GPIO4 (GPS UART2 RX/TX)

#### Libres “buenos”
- GPIO5, GPIO13, GPIO22, GPIO32

#### Libres solo entrada
- GPIO36 (VP), GPIO39 (VN)

## Licencia

El código heredado de LibAPRS mantiene su licencia original (ver [main/LibAPRS-esp32-i2s/LICENSE](main/LibAPRS-esp32-i2s/LICENSE)). Las adaptaciones de este repositorio se distribuyen bajo los mismos términos.

## Créditos

- [Mark Qvist](https://github.com/markqvist) — LibAPRS original.
- [handiko](https://github.com/handiko) — port a ESP32 con I2S.
- Francesco Sacchi — código base de decodificación AX.25.
- [kamilsss655](https://github.com/kamilsss655) — Hardware https://github.com/kamilsss655/ESPRI

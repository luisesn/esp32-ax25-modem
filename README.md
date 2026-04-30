# esp32-aprs-modem

Módem APRS / KISS TNC (AX.25 sobre AFSK Bell-202, 1200 bps) para **ESP32**, basado en ESP-IDF v5.x.

> ✅ **Estado: compila limpio (binary 904 KB, 47 % libre). KISS TNC bidireccional operativo. TX verificado en hardware. UI web funcional. Gateway IP RFC 1226 implementado. RX pendiente verificación con señal RF real.**

El firmware opera como un **KISS TNC bidireccional** accesible desde la red local vía TCP. Conecta `tncattach` o `direwolf` en el host y obtienes una interfaz de red AX.25 (`tnc0`) o un gateway APRS completo — sin cable USB, sin drivers adicionales.

Este repositorio adapta [LibAPRS-esp32-i2s](https://github.com/handiko/LibAPRS-esp32-i2s) — fork de [LibAPRS](https://github.com/markqvist/LibAPRS) de markqvist — para funcionar bajo ESP-IDF usando el DAC interno y ADC continuo por DMA.

## Hardware objetivo

- **MCU**: ESP32 (clásico, Xtensa LX6 dual-core).
- **Salida de audio (TX)**: DAC1 en GPIO 25 (`dac_continuous`, muestras 8-bit directas). Sin filtro externo necesario para la mayoría de transceptores.
- **Entrada de audio (RX)**: GPIO 35 / ADC1_CH7 (`adc_continuous` DMA a 48 kHz, atenuación 12 dB → rango 0–3,1 V). Entrada AC-acoplada desde la salida de altavoz del transceptor.
- **PTT**: GPIO 26, activo en nivel alto (1 = transmitir, 0 = reposo). Restricción de hardware (también es DAC2). Definido en `config.h`.

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
│   ├── config.h                       TNC_MODE, KISS_TRANSPORT, TCP port y GPIOs
│   ├── main.c                         app_main — bifurca según TNC_MODE
│   ├── kiss.h / kiss.c                framing KISS (encode/decode), independiente del transporte
│   ├── transport.h / transport.c      interfaz abstracta { init, write } para transportes
│   ├── transport_wifi.h / .c          WiFi STA + servidor TCP KISS (transporte activo)
│   ├── audio_stream.h / audio_stream.c  HTTP server port 80 (UI web), WebSocket /ws (audio + APRS JSON),
│   │                                WAV TCP port 8080, REST /api/aprs/send y /api/me
│   ├── ax25ip.h / ax25ip.c            gateway IP RFC 1226 (lwIP custom netif, PID=0xCC)
│   ├── aux_config.h / aux_config.c    carga/guarda config JSON desde SPIFFS (/spiffs/config.json)
│   ├── aux_file_management.h / .c     utilidades de sistema de ficheros SPIFFS
│   ├── spiffs_data/config.json        configuración inicial (callsign, WiFi, AP, IP) flasheada en SPIFFS
│   ├── spiffs_data/index.html         UI web completa (log APRS, envío de mensajes, audio IMA ADPCM)
│   ├── idf_component.yml              declaración de dependencias (esp-dsp sin uso activo)
│   └── LibAPRS-esp32-i2s/src/
│       ├── LibAPRS.{h,cpp}            API de alto nivel (APRS_init, queue_msg, queue_ack, getCallsign…)
│       ├── AFSK.{h,cpp}               modulador/demodulador AFSK, DAC TX, ADC RX
│       ├── AX25.{h,cpp}               codificación/decodificación AX.25 + raw_hook para KISS
│       ├── CRC-CCIT.{h,c}             CRC-CCITT para tramas AX.25
│       ├── HDLC.h                     flags HDLC (0x7E, 0x7F, AX25_ESC)
│       ├── FIFO.h                     cola circular inline
│       └── FakeArduino.{h,cpp}        stubs de Serial, F(), _BV(), cli/sei
├── managed_components/                dependencias gestionadas por IDF
├── CLAUDE.md                          guía de contexto para Claude Code
├── report.md                          informe técnico de problemas y soluciones
└── PROGRESS.md                        seguimiento de arreglos
```

## Dependencias

- **ESP-IDF v6.1** (usa `dac_continuous`, `adc_continuous`, `esp_wifi`, `esp_netif`, `spiffs`, `esp_http_server`).
- Componentes IDF requeridos (declarados en `main/CMakeLists.txt`):
  `esp_wifi`, `nvs_flash`, `esp_netif`, `lwip`, `driver`, `esp_driver_dac`, `esp_driver_gpio`, `esp_adc`, `spiffs`, `esp_http_server`, `vfs`, `espressif__cjson`.

## Compilación y flasheo

```bash
# Antes: editar main/spiffs_data/config.json con tus credenciales WiFi
idf.py set-target esp32
idf.py reconfigure   # necesario en IDF 6.1 antes del primer build
ninja -C build       # o: idf.py build (puede fallar en primer build sin reconfigure)
idf.py -p <PUERTO_SERIE> flash monitor
```

En Windows con el entorno IDF, sustituye `<PUERTO_SERIE>` por `COM3`, `COM4`, etc.

## Qué hace el firmware (modo KISS TNC)

1. `config_load()` lee `config.json` desde la partición SPIFFS (callsign, WiFi, AP, IP).
2. `transport_init(&transport_wifi_ops)` conecta a la red WiFi configurada e imprime la IP asignada.
3. `ax25ip_init()` activa el gateway IP RFC 1226 si `ip.enabled: true` en config.json.
4. `audio_stream_init()` arranca el servidor HTTP en port 80 (UI web + WebSocket `/ws`) y el WAV server en port 8080.
5. `kiss_init(on_kiss_frame)` registra el callback que transmite por radio las tramas recibidas del host.
6. `APRS_init()` + `APRS_set_raw_hook(on_ax25_raw_frame)` arranca el demodulador AFSK y registra el callback que:
   - Envía la trama al host KISS TCP (`kiss_send_frame`).
   - Detecta mensajes dirigidos a nuestro indicativo y transmite un ACK automático (`try_auto_ack`).
   - Inyecta paquetes IP en la pila lwIP si PID=0xCC (`ax25ip_rx_frame`).
   - Notifica a los clientes WebSocket con JSON `{type:"aprs", src, dst, path, info}`.
7. Cuando el host envía una trama KISS → `on_kiss_frame` → `afsk_queue_tx_frame()` (encola) → `receive_audio_task` despacha → `APRS_send_raw_frame()` → DAC → radio.
8. `audio_level_task` genera barra de nivel y hace parpadeo rápido del LED RX cuando el pico queda fuera del rango permitido.

## Interfaz web

Navega a `http://<IP-del-ESP32>/` para acceder a la UI web integrada:

- **Log APRS**: muestra paquetes recibidos por radio en tiempo real (newest-first). Badges `PARA MÍ` si el mensaje va dirigido a tu indicativo, `ACK` para confirmaciones, `TX` para los enviados.
- **Enviar mensaje**: formulario para transmitir mensajes APRS directamente desde el navegador.
- **Audio**: streaming de audio de recepción en tiempo real vía WebSocket (IMA ADPCM, 9600 Hz). También disponible como stream WAV en `http://<IP>:8080/`.
- **Click en callsign**: rellena automáticamente el formulario de destino.

## Gateway IP RFC 1226 (opcional)

El ESP32 puede actuar como gateway entre la red WiFi y la frecuencia de radio, encapsulando tráfico IP en tramas AX.25 UI con PID=0xCC según RFC 1226.

Para activarlo:
1. Editar `main/spiffs_data/config.json`: poner `"ip": {"enabled": true, "addr": "44.61.3.71", "netmask": "255.255.255.0", ...}`
2. Flashear (incluyendo imagen SPIFFS).
3. En el PC host, añadir ruta estática:
   ```bash
   ip route add 44.61.3.0/24 via <IP-WiFi-del-ESP32>
   ```
4. Los paquetes IP hacia esa subred serán encapsulados en AX.25 y transmitidos por radio. Los paquetes IP recibidos por radio se reenvían al host WiFi.

**Nota**: el hardware de radio y el enlace RF deben soportar los MTU y las tasas de datos de AX.25 (1200 bps Bell 202). MTU máximo: 300 bytes.

## Conectar al host

### Con tncattach (IP sobre AX.25)

```bash
# Instalar tncattach
git clone https://github.com/markqvist/tncattach && cd tncattach && make && sudo make install

# Crear interfaz de red AX.25
sudo tncattach -T -H 192.168.1.242 -P 8001 --ethernet --mtu 250 --noipv6 --ipv4 44.61.3.72/24

sudo tncattach --tcp 192.168.1.242 8001 --nosmall --ipv4 44.61.3.72/24

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

Editar `main/spiffs_data/config.json`:

```json
{
  "aprs": { "callsign": "TU_INDICATIVO" },
  "wifi": {
    "ssid": "TU_SSID_AQUI",
    "password": "TU_PASSWORD_AQUI",
    "connect_timeout_s": 120
  },
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
  }
}
```

`main/config.h` mantiene parámetros de compilación (`TNC_MODE`, `KISS_TRANSPORT`, `KISS_TCP_PORT`, GPIOs).

Para modo APRS consola (debug sin WiFi):

```c
#define TNC_MODE TNC_MODE_APRS
// En app_main se puede llamar APRS_setCallsign("TU_INDICATIVO", 1)
```

## Limitaciones actuales

- **RX pendiente verificación con señal RF real** — TX funciona (datos verificados por receptor externo); RX necesita señal de audio desde un transceptor o SDR para confirmar demodulación AFSK.
- **Gateway IP RFC 1226** — implementado y compila limpio; requiere verificación en hardware real.
- **Configuración en SPIFFS** — cambiar `config.json` en runtime requiere endpoint/configurador o reflashear imagen SPIFFS.
- **FIFOs internos sin protección `portMUX_TYPE`** (report.md §2.6) — riesgo teórico de corrupción si TX y el callback de RX coinciden en el tiempo.
- **Un solo cliente TCP a la vez** — el servidor KISS acepta reconexiones, pero no conexiones simultáneas.

## Indicativo y licencia de radioaficionado

Editar `main/spiffs_data/config.json` (`aprs.callsign` y opcionalmente `ip.ssid`) con tu propio indicativo antes de transmitir. **Transmitir en la banda amateur requiere licencia válida**.

## Licencia

El código heredado de LibAPRS mantiene su licencia original (ver [main/LibAPRS-esp32-i2s/LICENSE](main/LibAPRS-esp32-i2s/LICENSE)). Las adaptaciones de este repositorio se distribuyen bajo los mismos términos.

## Créditos

- [Mark Qvist](https://github.com/markqvist) — LibAPRS original.
- [handiko](https://github.com/handiko) — port a ESP32 con I2S.
- Francesco Sacchi — código base de decodificación AX.25.

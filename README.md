# esp32-aprs-modem

Módem APRS / KISS TNC (AX.25 sobre AFSK Bell-202, 1200 bps) para **ESP32**, basado en ESP-IDF v5.x.

> ✅ **Estado: compila limpio. KISS TNC con WiFi TCP operativo. Pendiente verificación en hardware real.**

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
| **KISS TNC** (por defecto) | `TNC_MODE_KISS` | Protocolo KISS sobre WiFi TCP (port 8001). Compatible con `tncattach` y `direwolf`. |
| **APRS consola** | `TNC_MODE_APRS` | Imprime paquetes AX.25 decodificados por el monitor serie. Modo debug. |

## Estructura del repositorio

```
esp32-aprs-modem/
├── CMakeLists.txt                     proyecto ESP-IDF raíz
├── sdkconfig                          configuración IDF (target = esp32)
├── partitions.csv                     tabla de particiones (NVS + OTA×2 + SPIFFS 704 KB)
├── main/
│   ├── CMakeLists.txt                 fuentes, dependencias y creación de imagen SPIFFS
│   ├── config.h                       TNC_MODE, KISS_TRANSPORT, WiFi SSID/pass, TCP port
│   ├── main.c                         app_main — bifurca según TNC_MODE
│   ├── kiss.h / kiss.c                framing KISS (encode/decode), independiente del transporte
│   ├── transport.h / transport.c      interfaz abstracta { init, write } para transportes
│   ├── transport_wifi.h / .c          WiFi STA + servidor TCP KISS (transporte activo)
│   ├── ptt.h / ptt.c                  control GPIO del PTT
│   ├── aux_config.h / aux_config.c    carga/guarda config JSON desde SPIFFS (/spiffs/config.json)
│   ├── aux_file_management.h / .c     utilidades de sistema de ficheros SPIFFS
│   ├── spiffs_data/config.json        configuración inicial (callsign, WiFi, AP) flasheada en SPIFFS
│   ├── idf_component.yml              declaración de dependencias (esp-dsp sin uso activo)
│   └── LibAPRS-esp32-i2s/src/
│       ├── LibAPRS.{h,cpp}            API de alto nivel (APRS_init, set_raw_hook, send_raw_frame…)
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

- **ESP-IDF ≥ v5.1** (usa `dac_continuous`, `adc_continuous`, `esp_wifi`, `esp_netif`, `spiffs`).
- Componentes IDF requeridos (declarados en `main/CMakeLists.txt`):
  `esp_wifi`, `nvs_flash`, `esp_netif`, `lwip`, `driver`, `esp_driver_dac`, `esp_driver_gpio`, `esp_adc`, `spiffs`.

## Compilación y flasheo

```bash
# Antes: editar main/config.h con tus credenciales WiFi
idf.py set-target esp32
idf.py build
idf.py -p <PUERTO_SERIE> flash monitor
```

En Windows con el entorno IDF, sustituye `<PUERTO_SERIE>` por `COM3`, `COM4`, etc.

## Qué hace el firmware (modo KISS TNC)

1. `PTT_Init()` configura GPIO 26 como salida PTT (activo alto).
2. `config_load()` lee `config.json` desde la partición SPIFFS (callsign, WiFi, AP config).
3. `transport_init(&transport_wifi_ops)` conecta a la red WiFi configurada e imprime la IP asignada.
4. `kiss_init(on_kiss_frame)` registra el callback que transmite por radio las tramas recibidas del host.
5. `APRS_init()` + `APRS_set_raw_hook(on_ax25_raw_frame)` arranca el demodulador AFSK y registra el callback que envía al host las tramas recibidas por radio.
6. Cuando el host envía una trama KISS → `on_kiss_frame` → `APRS_send_raw_frame()` → DAC → radio.
7. Cuando llega una trama AX.25 por radio → `on_ax25_raw_frame` → `kiss_send_frame()` → socket TCP → host.

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
MYCALL NO0CALL-1
```

```bash
direwolf -c direwolf.conf
```

## Configuración mínima antes de flashear

Editar `main/config.h`:

```c
// Credenciales WiFi
#define WIFI_SSID     "TU_SSID_AQUI"
#define WIFI_PASSWORD "TU_PASSWORD_AQUI"

// Puerto TCP donde escucha el ESP32
#define KISS_TCP_PORT 8001
```

Para modo APRS consola (debug sin WiFi):

```c
#define TNC_MODE TNC_MODE_APRS
// En app_main se puede llamar APRS_setCallsign("TU_INDICATIVO", 1)
```

## Limitaciones actuales

- **Credenciales WiFi en código fuente** — para producción, usar menuconfig (Kconfig) en lugar de `config.h`.
- **Pendiente verificación en hardware real** — el código compila y la arquitectura es correcta, pero no se ha podido probar con un transceptor real en este entorno.
- **FIFOs internos sin protección `portMUX_TYPE`** (report.md §2.6) — riesgo teórico de corrupción si TX y el callback de RX coinciden en el tiempo.
- **Un solo cliente TCP a la vez** — el servidor acepta reconexiones, pero no conexiones simultáneas.

## Indicativo y licencia de radioaficionado

Editar `config.h` / `main.c` con tu propio indicativo antes de transmitir. **Transmitir en la banda amateur requiere licencia válida**. El indicativo de ejemplo `NO0CALL` no debe emitirse sin autorización expresa del titular.

## Licencia

El código heredado de LibAPRS mantiene su licencia original (ver [main/LibAPRS-esp32-i2s/LICENSE](main/LibAPRS-esp32-i2s/LICENSE)). Las adaptaciones de este repositorio se distribuyen bajo los mismos términos.

## Créditos

- [Mark Qvist](https://github.com/markqvist) — LibAPRS original.
- [handiko](https://github.com/handiko) — port a ESP32 con I2S.
- Francesco Sacchi — código base de decodificación AX.25.

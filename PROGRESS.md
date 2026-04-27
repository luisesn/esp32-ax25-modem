# Progreso de arreglos — esp32-aprs-modem

Seguimiento de la resolución de problemas listados en [report.md](report.md).
Convención: ⬜ pendiente · 🟨 en curso · ✅ resuelto en código · ⚠️ parcial / pendiente verificación HW.

Última actualización: 2026-04-27

---

## Bloqueantes (sección 1 del report)

| # | Problema | Archivo principal | Estado |
|---|----------|-------------------|--------|
| 1.7 | `main.c` no imprime payload recibido | [main/main.c](main/main.c) | ⚠️ |
| 1.5 | `AFSK_transmit` doble incremento | [AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) | ⚠️ |
| 1.4 | Offset DC hardcodeado 30523 | [AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) | ⚠️ |
| 1.3 | ADC en unidad incorrecta (ADC2_CH0, 0 dB atten) | [AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp), [device.h](main/LibAPRS-esp32-i2s/src/device.h) | ⚠️ |
| 1.1 | Sample rate del ADC no controlado | [AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) | ⚠️ |
| 1.2 | RX por ráfagas en lugar de streaming | [AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) | ⚠️ |
| 1.6 | PDM inadecuado como audio analógico | [AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) | ⚠️ |

> Todos los bloqueantes están **corregidos a nivel de código**. Marcados como ⚠️ porque la verificación final requiere flashear en hardware real (no disponible en este entorno). Para pasar a ✅ hay que:
> 1. Ejecutar `idf.py build` con ESP-IDF ≥ v5.1 y confirmar que no hay errores.
> 2. Flashear en un ESP32 conectado a un transceptor o SDR y verificar RX/TX.

---

## Pendientes (secciones 2 y 3 del report)

| # | Problema | Estado |
|---|----------|--------|
| 2.1 | Eliminar buffers FFT | ✅ eliminados de facto en ronda 2026-04-17 |
| 2.2 | `freeMemory()` constante ficticia | ⬜ pendiente |
| 2.3 | `Afsk` estático en lugar de `malloc` | ⬜ pendiente |
| 2.4 | Macros deprecated I2S/ADC en `device.h` | ⬜ pendiente |
| 2.5 | Pausar RX durante TX | ✅ resuelto 2026-04-27 (ver bitácora) |
| 2.6 | FIFOs sin protección de concurrencia | ⬜ pendiente |
| 2.7 | `APRS_poll` en tarea RX — desacoplar callback | ⬜ pendiente en modo APRS; en modo KISS el callback es `kiss_send_frame` (operación rápida) |
| 2.8 | `FakeArduino::Serial` stub sin implementación | ⬜ pendiente |
| 3.1-3.6 | Limpieza restos AVR, `src.ino`, logs, etc. | ⬜ pendiente |
| 3.3 | `aprs_msg_callback` como global implícita | ✅ `main.c` reescrito con hooks explícitos; ya no aplica |

Orden sugerido para continuar:
1. `esp-dsp` en `idf_component.yml` puede quitarse ya (los buffers FFT se eliminaron pero la dependencia sigue declarada — alarga el build).
2. **2.2** `freeMemory()` → `esp_get_free_heap_size()`.
3. **2.6** Proteger FIFOs con `portMUX_TYPE`.

---

## Bitácora

### 2026-04-17 — ronda inicial de arreglos

**Arreglos rápidos:**

1. **1.7 `main.c`** ([diff](main/main.c)): el payload AX.25 se imprime con escapes `\xNN` para bytes no imprimibles; también se listan repetidores (`rpt_list`). Antes el bucle tenía el `Serial.write` comentado, así que nunca se veía el contenido.

2. **1.5 `AFSK_transmit`** ([AFSK.cpp ~L178](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L178)): eliminado el doble incremento de `i`. Ahora recorre linealmente todo el buffer.

3. **1.4 + 1.3 DC offset + ADC unit/atten**:
   - `device.h`: las antiguas constantes `I2S_ADC_UNIT = ADC_UNIT_1`, `I2S_ADC_CHANNEL = ADC1_CHANNEL_7` se renombraron a `AUDIO_ADC_UNIT`, `AUDIO_ADC_CHANNEL`, `AUDIO_ADC_ATTEN = ADC_ATTEN_DB_12`, `AUDIO_ADC_BITWIDTH = ADC_BITWIDTH_12`. Elimina la confusión entre lo documentado y lo implementado.
   - `AFSK.cpp::AFSK_hw_init`: ahora usa ADC1 canal 7 (GPIO 35) con 12 dB de atenuación (rango 0..3,1 V).
   - `AFSK.cpp::adc_to_s8`: nuevo helper que resta un DC offset dinámico (EMA Q10.10 inicializado a 2048, constante de tiempo ~1024 muestras) y escala a int8_t. Sustituye al mágico `average -= 30523` que era válido para el antiguo driver I2S-ADC de 16 bits.

**Arreglos estructurales:**

4. **1.1 + 1.2 ADC continuo DMA + streaming**: reescritura completa de `receive_audio_task`.
   - Se sustituye `adc_oneshot_read` (sin control de tasa) por `adc_continuous_new_handle` + `adc_continuous_start` a `TNC_I2S_SAMPLE_RATE = 48000 Hz`.
   - Buffer DMA de 1024 muestras × 4 descriptores ~= 85 ms de pool.
   - Se eliminan `audio_buf_full`, `FULL_BUF_LEN`, `process_audio`, y **todos los buffers FFT** (fft_input/window/re/im sumaban ~96 KB de SRAM). Con ello la dependencia `dsps_*` de `esp-dsp` ya no se usa (pero el componente sigue listado en `idf_component.yml` por si se retoma telemetría).
   - El bucle principal parsea `adc_digi_output_data_t.type1`, filtra por canal, decima x5 y llama a `AFSK_adc_isr` **una muestra tras otra** — sin acumular ni esperar a silencio. `APRS_poll()` se invoca cada 4 muestras decimadas.

5. **1.6 PDM → DAC continuo**: reemplazado `i2s_channel_*` + PDM (GPIO 25/26) por `dac_continuous_*` sobre DAC1 (GPIO 25).
   - Muestras 8-bit unsigned directamente desde `AFSK_dac_isr` (el tipo nativo que genera). Se elimina el cálculo `(sample << 7) + (1<<15)` que producía un rango asimétrico.
   - `transmit_audio_i2s` ahora llama a `dac_continuous_write` con bloqueo infinito (`-1`).
   - `finish_transmission` escribe 20×256 muestras con valor 128 (nivel medio = 0 V tras el acople) y hace `dac_continuous_disable` para dejar la salida silenciosa y no consumir DMA.
   - Se añade configuración GPIO del PTT (antes se asumía configurado).

6. **Compile-fix**: se define `bool hw_afsk_dac_isr = false;` en `AFSK.cpp`. La variable es referenciada por las macros `AFSK_DAC_IRQ_START/STOP` en `AFSK.h` con `extern bool` pero nunca tenía definición — el linker original debía estar dando unresolved symbol. Hoy las macros son inocuas pero la definición es necesaria.

---

### 2026-04-27 — arreglos de compilación y runtime

**Errores de compilación nuevos (debidos al refactor anterior):**

1. **Orden de campos en `adc_continuous_config_t`** ([AFSK.cpp ~L91](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L91)): en C++ los inicializadores designados deben seguir el orden de declaración del struct. Los campos `pattern_num` y `adc_pattern` están declarados antes de `sample_freq_hz`, `conv_mode` y `format` en `adc_continuous_config_t`, pero el código los colocaba al final → `-Werror=missing-field-initializers` + error de orden. Corregido reordenando.

2. **`adc_continuous_handle_cfg_t::flags` sin inicializar** ([AFSK.cpp ~L54](main/LibAPRS-esp32-i2s/src/AFSK.cpp#L54)): `flags` es un struct anidado; asignarle `= 0` (entero) genera `-Werror=missing-field-initializers` en C++. Corregido con `= {}`.

**Crash en runtime — conflicto I2S0:**

3. **`dac_continuous` y `adc_continuous` comparten I2S0** en ESP32 clásico: ambos drivers reclaman internamente el periférico I2S0 para DMA. Inicializar los dos en `AFSK_hw_init` hacía que `adc_continuous_new_handle` fallara a mitad de su secuencia de init, y su rutina de limpieza llamaba a `adc_apb_periph_free` con contador en 0 → `abort()`.

   **Solución — conmutación half-duplex de I2S0** ([AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp)):
   - `AFSK_hw_init` arranca **solo** el ADC (modo RX por defecto).
   - `switch_to_tx()`: `adc_continuous_stop` + `adc_continuous_deinit` (libera I2S0) → `dac_continuous_new_channels`.
   - `switch_to_rx()`: `dac_continuous_del_channels` (libera I2S0) → `adc_peripheral_start()`.
   - Flag `volatile bool tx_mode` para que `receive_audio_task` se pause sin acceder a un handle inválido.
   - Timeout de `adc_continuous_read` bajado de `portMAX_DELAY` a `pdMS_TO_TICKS(20)` para que el read desbloquee cuando `adc_continuous_stop` se llama desde la tarea TX.
   - Esto resuelve también el punto **2.5** del report (pausar RX durante TX).

**Watchdog (Task WDT) en `receive_audio_task`:**

4. `receive_audio_task` corría a prioridad 10. El ring buffer del ADC (4 descriptores × 1024 muestras ≈ 85 ms de pool) siempre tenía datos disponibles, así que `adc_continuous_read` devolvía inmediatamente en cada iteración. La tarea nunca entraba en estado BLOCKED → `IDLE0` en CPU0 jamás ejecutaba su reset del WDT → disparo a los ~5 s.

   **Solución**: `vTaskDelay(1)` al final del bucle de procesamiento. Un tick (~1 ms) de bloqueo garantiza que IDLE0 pueda correr. Con 85 ms de buffer de audio, el impacto en latencia es nulo.

---

## Verificación pendiente en hardware

Pasos de validación a realizar:

```bash
# 1. Editar main/config.h con credenciales WiFi reales
# 2. Compilar y flashear
idf.py set-target esp32
idf.py build
idf.py -p <PUERTO> flash monitor
```

Qué esperar en el `monitor` (modo KISS TNC):

- `[wifi_transport] Conectando a 'MiRed'...`
- `[wifi_transport] IP obtenida: 192.168.x.x`
- `[wifi_transport] Servidor KISS TCP escuchando en puerto 8001`
- `Audio: |##--------| 15` cada 250 ms (monitor de nivel de audio)
- Cuando `tncattach` conecta: `[wifi_transport] Cliente KISS conectado desde 192.168.x.y`

Verificación KISS TNC en el host:

```bash
# Conectar tncattach al ESP32
sudo tncattach --tcp <ip_esp32> 8001 --nosmall

# Escuchar tramas AX.25 recibidas por radio
sudo tcpdump -i tnc0 -n -e

# Enviar una trama de prueba (requiere otro nodo en rango)
ping -I tnc0 44.168.x.x
```

Test de TX sin radio: cambiar a `TNC_MODE_APRS`, enviar una baliza y capturar con RTL-SDR + `multimon-ng` desde GPIO 25 a través de un divisor resistivo. Si `multimon-ng` decodifica, el TX funciona.

Test de RX sin radio: reproducir un archivo de audio APRS estándar en el GPIO 35 (con nivel apropiado) y verificar que `tcpdump -i tnc0` muestra la trama.

---

## Archivos modificados

### Ronda 2026-04-17
- [main/main.c](main/main.c) — impresión de payload.
- [main/LibAPRS-esp32-i2s/src/device.h](main/LibAPRS-esp32-i2s/src/device.h) — nombres `AUDIO_ADC_*`.
- [main/LibAPRS-esp32-i2s/src/AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) — refactor completo de init/TX/RX.

### Ronda 2026-04-27 — sesión 1
- [main/LibAPRS-esp32-i2s/src/AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) — correcciones compilación + conmutación half-duplex I2S0 + watchdog fix.

### Ronda 2026-04-27 — sesión 2 (KISS TNC + WiFi TCP)

**Nuevos ficheros:**
- [main/kiss.h](main/kiss.h) / [main/kiss.c](main/kiss.c) — framing KISS (máquina de estados S_IDLE→S_CMD→S_DATA→S_ESC; codificador con escape; buffer estático `AX25_MAX_FRAME_LEN × 2 + 3` bytes).
- [main/transport.h](main/transport.h) / [main/transport.c](main/transport.c) — ops struct `{ init, write }`; funciones globales `transport_init()` / `transport_write()`.
- [main/transport_wifi.h](main/transport_wifi.h) / [main/transport_wifi.c](main/transport_wifi.c) — WiFi STA con reconexión automática; servidor TCP que acepta un cliente a la vez; `server_task` alimenta `kiss_rx_byte()`; `wifi_transport_write()` llama `send()`.

**Ficheros modificados:**
- [main/config.h](main/config.h) — reescrito: `TNC_MODE`, `KISS_TRANSPORT`, WiFi SSID/password, `KISS_TCP_PORT`.
- [main/main.c](main/main.c) — reescrito: bifurca según `TNC_MODE`; modo KISS TNC activo por defecto; modo APRS preservado bajo `#elif`.
- [main/CMakeLists.txt](main/CMakeLists.txt) — añadidos `kiss.c transport.c transport_wifi.c` a `SRCS`; añadido `REQUIRES esp_wifi nvs_flash esp_netif lwip`.
- [main/LibAPRS-esp32-i2s/src/AX25.h](main/LibAPRS-esp32-i2s/src/AX25.h) — tipo `ax25_raw_callback_t`; campo `raw_hook` en `AX25Ctx`.
- [main/LibAPRS-esp32-i2s/src/AX25.cpp](main/LibAPRS-esp32-i2s/src/AX25.cpp) — en `ax25_poll()`: llama `raw_hook(buf, frame_len-2)` si está registrado, en lugar de `ax25_decode()`.
- [main/LibAPRS-esp32-i2s/src/LibAPRS.h](main/LibAPRS-esp32-i2s/src/LibAPRS.h) — añadidas `APRS_set_msg_hook()`, `APRS_set_raw_hook()`, `APRS_send_raw_frame()`.
- [main/LibAPRS-esp32-i2s/src/LibAPRS.cpp](main/LibAPRS-esp32-i2s/src/LibAPRS.cpp) — eliminado `extern "C" void aprs_msg_callback`; `APRS_init` ya no requiere símbolo externo; implementadas las tres funciones nuevas.

**Bug fix:**
- [main/transport_wifi.c](main/transport_wifi.c) — `IP2STR(&client_addr.sin_addr)` usaba macro de ESP-IDF (`esp_ip4_addr_t*` / campo `.addr`) con POSIX `struct in_addr` (campo `.s_addr`) → error de compilación. Corregido con `inet_ntop()` para la IP del socket y `esp_ip4addr_ntoa()` para la IP WiFi.

Archivos no tocados (intencionadamente): `AFSK.cpp` (capa física estable), `AX25.cpp` (lógica AX.25 madura), `CRC-CCIT.c`, `FIFO.h`, `HDLC.h`.

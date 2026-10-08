# /trap-check — Revisar cambios contra trampas conocidas del hardware

Analiza los cambios actuales (staged + unstaged) y los contrasta con las trampas específicas
documentadas para este proyecto. Detecta regresiones antes de compilar o flashear.

## Pasos

1. Obtén el diff completo: `git diff HEAD` (incluye staged y unstaged) **y** el de la librería, que es un repo anidado y no aparece en el anterior: `git -C main/LibAPRS-esp32-i2s diff HEAD`.
   Si no hay cambios, indica que no hay nada que revisar.

2. Para cada trampa de la lista siguiente, comprueba si el diff toca código relevante
   y si lo hace correctamente. Reporta solo las trampas que apliquen al diff.

## Lista de trampas a verificar

### ADC/DAC simultáneos (I2S0 compartido)
- ¿Algún cambio activa `dac_continuous` sin antes llamar `adc_continuous_stop` + `adc_continuous_deinit`?
- ¿Algún cambio activa `adc_continuous_start` sin antes llamar `dac_continuous_del_channels`?
- El patrón correcto es la conmutación half-duplex en `switch_to_tx()` / `switch_to_rx()`.

### PTT — GPIO 26 / polaridad
- ¿Algún cambio invierte la polaridad del PTT? Activo alto: `gpio_set_level(GPIO_PTT_OUT, 1)` = TX, `0` = reposo.
- ¿Algún cambio mueve GPIO_PTT_OUT a otro pin? Está fijado en hardware, no cambiar.
- Tras `dac_continuous_new_channels()`, ¿se restaura `gpio_set_direction(GPIO_PTT_OUT, GPIO_MODE_OUTPUT)`?
  (DAC_CHANNEL_MODE_SIMUL puede reconfigurarlo como analógico)

### TX_SAMPLE_BUFLEN y DMA
- ¿Algún cambio reduce `TX_SAMPLE_BUFLEN` por debajo de 2048? No hacerlo — es el tamaño del descriptor DMA.
- ¿Algún cambio escribe un buffer parcial (< TX_SAMPLE_BUFLEN bytes) al DAC? Siempre escribir el buffer completo, rellenando con `0x80`.
- ¿`dac_continuous_write` usa timeout finito (no `portMAX_DELAY` / `-1`)?

### Timeout ADC
- ¿`adc_continuous_read` usa `portMAX_DELAY`? Debe usar `pdMS_TO_TICKS(20)` para que `adc_continuous_stop` pueda desbloquear la tarea.

### `vTaskDelay(1)` en receive_audio_task
- ¿Algún cambio elimina el `vTaskDelay(1)` al final del bucle de `receive_audio_task`? Es necesario para que IDLE0 corra y el watchdog no expire.

### Cola TX entre tareas
- ¿Algún cambio llama `APRS_send_raw_frame` directamente desde `server_task` o cualquier tarea que no sea `receive_audio_task`? Solo `receive_audio_task` puede llamarla (restricción del mutex ADC de ESP-IDF). El path correcto es `afsk_queue_tx_frame()`.

### Inicializadores C++ / structs anidados
- ¿Algún cambio inicializa `adc_continuous_handle_cfg_t` con `= 0`? Debe ser `= {}`.
- ¿Los inicializadores designados de `adc_continuous_config_t` respetan el orden de campos? (`pattern_num`, `adc_pattern` antes de `sample_freq_hz`, `conv_mode`, `format`).

### `pbuf` en lwIP
- ¿Algún cambio usa `pbuf_length(p)`? No existe. Usar `p->tot_len`.

### AX25 / HDLC / CRC
- ¿Algún cambio toca `AX25.cpp`, `AX25.h`, `HDLC.h`, o `CRC-CCIT.c`? Estos ficheros son código maduro — cualquier cambio requiere justificación explícita.

### Callback RX y deduplicación
- ¿Se añade un filtro de duplicados fuera de `on_ax25_raw_frame()` (LibAPRS, `ax25ip.c`, `digipeater.c`)? El dedup del doble módem vive sólo ahí (`rx_frame_is_duplicate`, 300 ms); otro filtro rompe `rx_stats_v1/v2`.
- ¿El `raw_hook` o `aprs_poll_task` bloquea (mutex, WS lento, `vTaskDelay` largo)? Se ejecuta en `aprs_poll_task` (prio 9); bloquear llena los FIFOs de RX.
- ¿Se usan `fifo_*` sin lock en una ruta multi-tarea/ISR? Usar variantes `_locked`.

### LBT / inhibición post-RX
- LBT usa `channel_is_busy()` = squelch HFNE activo y abierto: sin monitor/repetidor activo no hace nada. ¿El cambio asume lo contrario?
- ¿Se rompe `afsk_notify_rx_frame()` o `post_rx_tx_delay_ms` (se salta la ventana de inhibición)?

### Configuración, credenciales y repos
- ¿Se cambia un ajuste de `sdkconfig` sin trasladarlo a `sdkconfig.defaults`?
- ¿El diff incluye credenciales WiFi reales en `main/spiffs_data/config.json`? No commitearlas.
- ¿Hay cambios en `main/LibAPRS-esp32-i2s/`? Deben commitearse dentro del repo anidado, no en el principal.

### PTT: no invertir
- Reconfirmar que ninguna llamada `gpio_set_level(GPIO_PTT_OUT, …)` cambia de polaridad (activo alto).

## Formato del informe

Para cada trampa que aplique al diff:
- ✅ **OK** — el cambio respeta la restricción
- ⚠️ **REVISAR** — el cambio toca código relevante y hay algo sospechoso
- ❌ **PROBLEMA** — el cambio viola claramente la restricción

Si ninguna trampa aplica al diff, indicarlo explícitamente: "Ninguna trampa conocida afecta a estos cambios."

Al final, un resumen de una línea: cuántas trampas se verificaron, cuántas pasaron, cuántas requieren atención.

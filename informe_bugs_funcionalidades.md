# Informe de mejoras: bugs y funcionalidades

Fecha: 2026-11-10 · Firmware analizado: v1.4.7 · Complementa a `informe_mejoras.md` (2026-10-08), no lo sustituye.

## Alcance y método

- Análisis **por lectura de código**. Nada de lo que sigue se ha probado en hardware ni se ha compilado ningún cambio.
- **Seguridad excluida** a petición (autenticación, exposición de credenciales, etc.).
- Leídos a fondo: `main.c`, `kiss.c`, `transport_wifi.c`, `audio_stream.c`, `api_rest.c`, `ax25ip.c`, `il2p.c`, `gps.c`, `digipeater.c`, `remote_cmd.c`, `carrier_detect.c`, `aux_config.c` y la librería (`AFSK.*`, `LibAPRS.cpp`).
- **No revisados**: `sstv.c`, `morse.c`, `display.c`, `ota.c`, `ota_net.c`, `rf_console.c`, `delay_tune.c`, `netdiag.c`, `squelch_sf.c`, `rx_survey.c`, `rs_codec.c`, `repeater.c` y el JavaScript de `index.html`. Puede haber fallos ahí que este informe no recoge.
- Las mejoras del demodulador están en [informe_modem.md](informe_modem.md).

Cada hallazgo lleva una etiqueta de confianza:

- **[Leído]**: el defecto se ve directamente en el código.
- **[Hipótesis]**: consecuencia razonada que conviene confirmar en el dispositivo antes de invertir tiempo.

## Resumen: lo más importante

| # | Problema | Efecto | Esfuerzo |
|---|----------|--------|----------|
| B1 | IL2P no se aplica al tráfico IP en modo TUN (y corrompe ~1 de cada 256 paquetes) | Tu configuración actual (`ip.mode=tun` + `il2p.enabled`) no obtiene FEC y pierde paquetes al azar | Bajo |
| B2 | `send()` bloqueante al cliente KISS dentro del hook de RX | Un cliente KISS colgado detiene toda la recepción | Bajo |
| B3 | La caída a AP es definitiva | Un corte de router de unos minutos deja el nodo en AP hasta reiniciar | Bajo |
| B4 | Minutos "60.00" en las balizas de posición | Posición inválida de forma esporádica | Bajo |
| B5 | Auto-ACK sólo con identificadores numéricos | Mensajes de clientes con ID alfanumérico nunca reciben ACK | Bajo |
| B6 | Digipeater: dedup sobre la trama completa y sin filtro de tramas propias | Repeticiones duplicadas en cuanto haya otro digi en cobertura | Medio |
| B7 | Servidor HTTP limitado a 3 sockets | Web lenta o con peticiones fallidas | Bajo |
| F1 | Reintento de mensajes con seguimiento de ACK | Hoy un mensaje perdido se pierde sin aviso | Medio |
| F2 | Acceso al canal CSMA p-persistente | Menos colisiones con LBT activo | Medio |

## 1. Bugs

### 1.1 Prioridad alta

**B1. IL2P y modo TUN no encajan** — `smart_tx_frame` en [main/main.c](main/main.c), `il2p.c` · **[Leído]**

`smart_tx_frame` recorre las direcciones AX.25 de la trama para localizar el PID y manda por IL2P sólo lo que lleva PID 0xCC. En modo TUN las tramas que genera `ax25ip.c` son `00 00 08 00` + IPv4 en crudo, **sin cabecera AX.25**. El byte que se interpreta como PID es entonces un byte cualquiera de la cabecera IP o del transporte. Consecuencias con la configuración actual (`ip.mode = "tun"`, `il2p.enabled = true`):

- Casi todo el tráfico IP sale por AX.25 normal: **IL2P no aporta FEC** justo al tráfico para el que se activó.
- Cuando ese byte vale 0xCC por casualidad (del orden de 1/256 de los paquetes), la trama se envía por IL2P, cuyo codificador trata los 14 primeros bytes como indicativos y los pasa por una codificación de 6 bits con pérdida. El paquete llega corrupto o no llega.

Arreglo propuesto: que `ax25ip` marque sus tramas (o que `smart_tx_frame` reconozca el prefijo TUN) y que IL2P tenga un modo "payload opaco" sin recodificar indicativos. Ver también B14.

**B2. Escritura bloqueante al cliente KISS desde la ruta de RX** — `wifi_transport_write` en [main/transport_wifi.c](main/transport_wifi.c) · **[Leído]**, impacto **[Hipótesis]**

`kiss_send_frame()` se llama desde `on_ax25_raw_frame`, que corre en `aprs_poll_task`. El `send()` es bloqueante y el socket no tiene `SO_SNDTIMEO`. Si el cliente KISS deja de leer (portátil suspendido, WiFi inestable), la tarea que procesa **toda** la recepción se queda parada hasta que el keepalive cierra la conexión (~25 s), o indefinidamente si el otro extremo sigue respondiendo a nivel TCP pero no lee. Mientras tanto se llena el FIFO de RX y se pierden tramas, ACK y digipeticiones.

Arreglo: `SO_SNDTIMEO` corto (100–200 ms) y descartar la trama si expira; mejor aún, una cola hacia una tarea de escritura para que el hook no toque nunca un socket.

El mismo patrón, más acotado, existe con el WebSocket: `audio_stream_ws_send_text` puede esperar hasta 200 ms de mutex más 500 ms por cliente dentro del hook.

**B3. La caída a AP es terminal** — bucle de reconexión en [main/transport_wifi.c](main/transport_wifi.c) · **[Leído]**

Tras 5 rondas fallidas el nodo pasa a AP y la tarea de reconexión deja de intentar (`if (state == WIFI_STATUS_HOTSPOT) continue;`). Un corte del router de unos minutos deja el equipo en AP hasta un reinicio manual, mal comportamiento para un nodo desatendido.

Arreglo: en modo AP, reintentar STA cada N minutos si no hay clientes asociados al AP (o usar APSTA).

**B4. Minutos "60.00" en la posición** — [main/gps.c](main/gps.c), `aprs_beacon_handler` en [main/api_rest.c](main/api_rest.c) · **[Leído]**

`gps.c` formatea los minutos con `%05.2f` sin acotar: 59,996' se imprime como `60.00`, que no es una posición APRS válida. En `api_rest.c` el comentario dice que se limita a 59.99 pero la comprobación es `lat_m >= 60.0`, que nunca se cumple. `remote_cmd.c` ya lo hace bien, con centésimas enteras.

Arreglo: una única función `aprs_format_position()` compartida por los tres sitios, usando la versión entera de `remote_cmd.c`. De paso se unifica el tipo de dato (`!` en `gps.c`, `=` en la API).

**B5. El auto-ACK sólo acepta identificadores numéricos** — `try_auto_ack` en [main/main.c](main/main.c) · **[Leído]**

El bucle que extrae el ID sólo admite `0`–`9`. La especificación APRS permite de 1 a 5 caracteres alfanuméricos, y muchos clientes (APRSdroid, Xastir, YAAC) usan IDs con letras o el formato reply-ack `{AB}CD`. A esos mensajes no se les envía ACK: el remitente reintenta hasta rendirse. Además, como no se entra en la rama de dedup, el comando remoto tampoco se ejecuta.

Arreglo: aceptar alfanuméricos y cortar en `}`.

**B6. Digipeater** — [main/digipeater.c](main/digipeater.c) · **[Leído]** (afecta sólo si `digi.enabled`, hoy desactivado)

- El hash de dedup cubre la trama entera, **incluido el path**. La misma trama recibida de nuevo a través de otro digipeater lleva otro path, da otro hash y se vuelve a repetir. El dedup debe calcularse sobre origen + destino + información.
- No se descartan las tramas cuyo origen es el propio indicativo: el nodo puede repetir sus balizas cuando vuelven por otro digi.
- No hay tope de N (`WIDE7-7` se acepta) ni comprobación del límite de 8 repetidores al insertar el indicativo propio.
- No se atiende el indicativo propio puesto explícitamente en el path, sólo los alias.
- Copia de 600 B en la pila de `aprs_poll_task` (3072 B); el comentario dice `receive_audio_task` (8 KB), que ya no es cierto. Conviene medir con `/api/sys` o pasar el buffer a `static`.

### 1.2 Prioridad media

**B7. El servidor HTTP sólo admite 3 sockets** — [main/audio_stream.c](main/audio_stream.c) · **[Leído]**, impacto **[Hipótesis]**

`max_open_sockets = 3` y `AUDIO_STREAM_MAX_CLIENT_FDS 3`, aunque el comentario habla de "2 clientes WS + ráfaga del portal cautivo + margen". Un navegador abre la página, el WebSocket y varias peticiones `fetch` en paralelo; con 3 sockets el servidor va purgando conexiones por LRU. `sdkconfig.defaults` ya reserva 16 sockets lwIP. Probar con 6–7.

**B8. Encolado de TX fallido que se da por bueno** — [LibAPRS.cpp](main/LibAPRS-esp32-i2s/src/LibAPRS.cpp), `aprs_send_handler` · **[Leído]**

`APRS_queue_msg` ignora el retorno de `afsk_queue_tx_frame` y devuelve el número de secuencia aunque la cola estuviera llena. La API responde con un `msg_id` de un mensaje que nunca saldrá y la web lo deja "esperando ACK". Propagar el error hasta `/api/aprs/send`.

**B9. `message_seq` sin sincronizar** — [LibAPRS.cpp](main/LibAPRS-esp32-i2s/src/LibAPRS.cpp) · **[Leído]**

Contador global que incrementan varias tareas (httpd, `aprs_poll_task` vía `remote_cmd`, consola). Dos mensajes simultáneos pueden salir con el mismo ID. Probabilidad baja, arreglo trivial (atómico o sección crítica).

**B10. `cJSON_Duplicate` dentro de una sección crítica** — [main/aux_config.c](main/aux_config.c) · **[Leído]**, impacto **[Hipótesis]**

`config_get()` duplica todo el árbol de configuración (decenas de `malloc`) con `taskENTER_CRITICAL`, es decir, con las interrupciones del núcleo deshabilitadas durante el orden de milisegundos. Si coincide con el núcleo del audio puede provocar pérdidas de muestras del ADC o tirones en el DAC durante TX. Debe ser un mutex. Varias rutas llaman a `config_get()` en ejecución (API, consola), no sólo en el arranque.

**B11. Condición de carrera en la entrega de tramas IL2P** — [main/il2p.c](main/il2p.c) · **[Leído]**

`frame_ready` se activa en `receive_audio_task` y se consume en `aprs_poll_task` sin protección; una segunda trama puede sobrescribir `ax25_frame` mientras se entrega la primera. Con tramas seguidas (ráfagas TCP por RF) es plausible. Copiar a un buffer o cola al completar la trama.

**B12. `POST /api/config` sólo reaplica digipeater y morse** — [main/api_rest.c](main/api_rest.c) · **[Leído]**

El resto (indicativo, GPS y periodo de baliza, IL2P, LBT, consola…) necesita reinicio y la web no lo indica. Mínimo: que la respuesta incluya `reboot_required` y la UI lo muestre. Mejor: reaplicar en caliente lo que sea barato (indicativo, `post_rx_tx_delay_ms`, LBT, periodo de baliza).

**B13. Baliza GPS** — [main/gps.c](main/gps.c) · **[Leído]**

- La primera baliza sale después de un periodo completo (hasta 10 min sin anunciarse tras arrancar).
- La validez del fix oscila: RMC con `V` la pone a falso y GGA con calidad > 0 la pone a verdadero, sentencia a sentencia.
- Siempre añade rumbo/velocidad aunque el nodo sea fijo; símbolo `/>` fijo; sin comentario ni altitud.

**B14. IL2P no es interoperable** — [main/il2p.c](main/il2p.c) · **[Hipótesis]** (comparación hecha de memoria con la especificación, sin contrastar con el documento)

La implementación difiere de IL2P estándar en varios puntos: tamaño de cabecera (14+2 frente a 13+2), codificación de indicativos, preámbulo 0x7E y exigencia de flags antes del sync, semilla del scrambler, reparto de bloques RS y uso de NRZI. Además descarta el path de digipeaters. Funciona entre dos equipos con este firmware, pero es muy improbable que Dire Wolf la decodifique. CLAUDE.md ya lo lista como "sin verificar"; esto lo convierte en "previsiblemente no". Decidir si se quiere IL2P estándar (reescritura parcial contrastando con la especificación y con Dire Wolf) o un FEC propio documentado como tal.

### 1.3 Prioridad baja y limpieza

- **Uso tras liberar en `TNC_MODE_APRS`** ([main/main.c](main/main.c)): `APRS_setCallsign` se llama después de `config_free_json(cfg)`. El modo está muerto desde que se desactivó el demodulador v1; lo razonable es borrar la rama. **[Leído]**
- **`rx_frame_is_duplicate` (300 ms)** sigue activo con un solo demodulador. Es inocuo, pero descartaría una retransmisión legítima idéntica dentro de la ventana. Hay que mantenerlo si se implementan los slicers múltiples del informe del módem.
- **Orden de arranque**: el hook de audio se instala antes de `carrier_detect_init`/`squelch_sf_init`/`rx_survey_init`. Hoy no falla; moverlo al final lo hace robusto.
- **`wav_server_task`** ([main/audio_stream.c](main/audio_stream.c)): `bind`/`listen` sin comprobar, `recv` sin timeout, y la comprobación de vida `send(fd, NULL, 0)` no detecta nada.
- **`/api/log`**: duplica hasta 250 entradas con el mutex tomado; pico de heap de decenas de KB en un sistema con presión de memoria. Serializar por trozos (`httpd_resp_send_chunk`).
- **`/api/sys`**: lista de tareas fija; faltan `httpd`, `iplog`, `cap_dns`, `ota_net`, `rx_survey`… Usar `uxTaskGetSystemState`.
- **`/api/spiffs/upload`**: no comprueba espacio libre antes de escribir.
- **`/api/aprs/send`**: no valida ni pasa a mayúsculas el destinatario.
- **KISS** ([main/kiss.c](main/kiss.c)): sólo trata DATA en el puerto 0; TXDELAY, P, SlotTime y TXTAIL se ignoran (ver F2).
- **`ax25ip.c`**: `ip_info()` usa un buffer `static` desde dos tareas (sólo afecta al texto del log).
- **WiFi**: no se llama a `esp_wifi_set_ps(WIFI_PS_NONE)`; el modem-sleep por defecto añade latencia a KISS y al audio. SNTP fijo a `pool.ntp.org` y sin zona horaria. Canal del AP fijo a 1.
- **Respuestas de `remote_cmd`**: salen como mensaje con `{NNN}` nuevo y sin reintento; `tx,morse` no responde; los comandos desconocidos se ignoran en silencio.
- **Código muerto**: `CONFIG_PTT_ON_DELAY_MS`/`OFF_DELAY_MS` definidos y sin usar; `s_rx[0]` de IL2P (~1,9 KB de RAM para el demodulador v1 desactivado); `APRS_sendLoc`/`APRS_sendMsg` heredados con `malloc` sin comprobar.

### 1.4 Documentación desalineada con el código

| CLAUDE.md dice | El código dice |
|----------------|----------------|
| Cola TX de capacidad 4 | `TX_QUEUE_DEPTH 6` |
| DAC `desc_num=8`, pool ≈336 ms | `DAC_DESC_NUM 4u` (≈170 ms) |
| `vTaskDelay(20 ms)` en `switch_to_tx` | 5 ms |
| Filtro paso banda "centro 1700 Hz, Q=2" | Pico real ≈2000 Hz (ver informe del módem) |
| Paso bajo "Butterworth 600 Hz" | −3 dB real ≈1350 Hz |
| `ax25ip_rx_frame` y el digipeater corren en `receive_audio_task` (comentarios) | Corren en `aprs_poll_task` |

El margen de DMA con 4 descriptores es la mitad del que describe la trampa de `TX_SAMPLE_BUFLEN`; conviene confirmar cuál de los dos valores es el deseado.

## 2. Funcionalidades

Ordenadas por relación utilidad/esfuerzo. Las marcadas con (backlog) ya figuran en `doc/pending/features_and_improvements.md`; aquí sólo se priorizan.

### 2.1 Recomendadas a corto plazo

**F1. Reintento de mensajes con seguimiento de ACK** (backlog). Tabla pequeña de mensajes pendientes (destino, ID, texto, intentos, próximo envío), reintento con espera creciente (por ejemplo 30/60/120 s, 3–5 intentos), cancelación al recibir `ackNNN` y evento WS `msg_failed`. Sirve al chat de la web y a las respuestas de `remote_cmd`. Requiere B5 y B8.

**F2. Acceso al canal CSMA p-persistente y parámetros KISS.** Hoy el TX sale en cuanto el canal queda libre: si dos estaciones esperan, colisionan siempre. Añadir slot aleatorio (`slottime` 100 ms, `persist` 63) al despacho de `s_tx_queue`, y aceptar los comandos KISS TXDELAY/P/SlotTime/TXTAIL para que APRSdroid, Xastir o Dire Wolf puedan ajustarlos. Tiene más valor cuando LBT esté activado.

**F3. Path configurable.** `WIDE1-1,WIDE2-x` está fijo en la librería y se aplica también a los ACK. Hacerlo configurable (`aprs.path`), con opción de path vacío y un path distinto, normalmente más corto, para ACK y mensajes.

**F4. Baliza: al arrancar, con comentario y símbolo configurables, y SmartBeaconing** (backlog en parte). Enviar la primera baliza al obtener fix; `gps.symbol`, `gps.comment`, altitud opcional; posición fija configurable para nodos sin GPS; SmartBeaconing si el equipo se usa en móvil.

**F5. Varios clientes KISS simultáneos** (backlog). Hoy `listen(fd, 1)` y un solo cliente. Con 2–3 se puede tener a la vez un cliente APRS y un monitor. Hacerlo junto con B2 (cola de escritura por cliente).

**F6. Reconexión desde AP** (es B3) y `reboot_required` en la config (es B12).

### 2.2 Medio plazo

- **Tabla de estaciones oídas** (backlog): indicativo, última vez, número de tramas, calidad `q`/`qdb` media, posición si la hay. Los datos ya pasan por `on_ax25_raw_frame`; con ella son casi gratis el **mapa** y las estadísticas por estación.
- **iGate APRS-IS** (backlog): RX→IS es sencillo (conexión TCP + filtro de tramas no reenviables); IS→RF pide más cuidado.
- **Captura de audio crudo** (`/api/rx/capture`: N segundos a 9600 Hz o 48 kHz en 12 bits): es la herramienta que desbloquea el banco de pruebas offline del [informe del módem](informe_modem.md).
- **Calibración guiada de nivel de RX**: con los datos de SURVEY, indicar "sube/baja el volumen" y avisar de saturación (AGC en el tope, pico 127).
- **Objetos/ítems, boletines y telemetría APRS** (backlog): la telemetría (heap, tramas RX/TX, tensión si hay divisor) es barata y útil para un nodo remoto.
- **Comandos remotos** (backlog): `rx,stats`, `sys,reboot`, respuesta a comandos desconocidos, límite de frecuencia.
- **Validación de esquema de `config.json`** (backlog) con mensajes de error legibles en la web.
- **Web** (pendiente según PROGRESS.md): pausar temporizadores con la pestaña oculta, envoltorio `api()` con `r.ok` y timeout, persistencia en `localStorage`.

### 2.3 Largo plazo o bajo retorno

KISS por UART/USB/Bluetooth, AGWPE, RX de SSTV, CTCSS, PTT activo bajo configurable, registro de arranque en SPIFFS, páginas del display (todos en el backlog). No se ha encontrado motivo para adelantarlos.

## 3. Calidad y mantenimiento

- **Sin tests ni CI.** Tres módulos se pueden probar en host sin tocar hardware y cubren los bugs de este informe: formateo de posición (B4), análisis de mensajes/ACK (B5) y digipeater (B6). `kiss.c` y `dedup.c` también son puros.
- **Formateadores duplicados** (posición ×3, separación indicativo/SSID ×3 en `try_auto_ack`): extraer a un módulo `aprs_util`.
- **Pilas sin medir**: `aprs_poll_task` (3072 B) ejecuta el hook completo, incluidos digipeater, cJSON y `snprintf`. Leer `/api/sys` tras una sesión con tráfico antes de dar el valor por bueno.
- **Librería como repo anidado sin enlazar** y `config.json` seguido por git: ya conocidos, siguen pendientes.

## 4. Orden de trabajo sugerido

1. B1, B2, B3, B4, B5: cambios pequeños y localizados con efecto directo en el uso diario.
2. B7, B8, B9, B10, B11 y corrección de la documentación (1.4).
3. F1 (reintentos) y F3 (path).
4. Captura de audio crudo y banco de pruebas del módem.
5. B6 antes de activar el digipeater; F2 antes de activar LBT de forma permanente.
6. Decisión sobre IL2P (B14).

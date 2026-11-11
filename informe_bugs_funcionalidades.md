# Informe de mejoras: bugs y funcionalidades

Fecha: 2026-11-10 (ampliado el 2026-11-11 con los módulos que faltaban) · Firmware analizado: v1.4.7 · Complementa a `informe_mejoras.md` (2026-10-08), no lo sustituye.

## Alcance y método

- Análisis **por lectura de código**, más dos comprobaciones en el PC (no en el dispositivo): el códec Reed-Solomon compilado con gcc y probado con errores aleatorios, y el bucle de subida de imágenes SSTV reproducido en Python. Nada se ha probado en hardware ni se ha cambiado código del firmware.
- **Seguridad excluida** a petición (autenticación, exposición de credenciales, etc.).
- Primera pasada: `main.c`, `kiss.c`, `transport_wifi.c`, `audio_stream.c`, `api_rest.c`, `ax25ip.c`, `il2p.c`, `gps.c`, `digipeater.c`, `remote_cmd.c`, `carrier_detect.c`, `aux_config.c` y la librería (`AFSK.*`, `LibAPRS.cpp`).
- Segunda pasada (2026-11-11): `sstv.c`, `morse.c`, `display.c`, `ota.c`, `ota_net.c`, `rf_console.c`, `delay_tune.c`, `netdiag.c`, `squelch_sf.c`, `rx_survey.c`, `rs_codec.c`, `repeater.c`, `dedup.c`, `adpcm.c` y el JavaScript de `index.html`. Sus hallazgos son B14 (reescrito) y B15–B22. Sin defectos reseñables: `ota.c`, `ota_net.c`, `rx_survey.c`, `dedup.c`, `adpcm.c`.
- Sin revisar: `server/` (herramienta de desarrollo en Python) y `espota.py`.
- Las mejoras del demodulador están en [informe_modem.md](informe_modem.md).

Cada hallazgo lleva una etiqueta de confianza:

- **[Leído]**: el defecto se ve directamente en el código.
- **[Calculado]**: reproducido en el PC con el código real (o un port fiel) fuera del dispositivo.
- **[Hipótesis]**: consecuencia razonada que conviene confirmar en el dispositivo antes de invertir tiempo.

## Resumen: lo más importante

| # | Problema | Efecto | Esfuerzo |
|---|----------|--------|----------|
| ~~B1~~ ✅ | IL2P no se aplica al tráfico IP en modo TUN (y corrompe ~1 de cada 256 paquetes) | Tu configuración actual (`ip.mode=tun` + `il2p.enabled`) no obtiene FEC y pierde paquetes al azar | Bajo |
| ~~B2~~ ✅ | `send()` bloqueante al cliente KISS dentro del hook de RX | Un cliente KISS colgado detiene toda la recepción | Bajo |
| ~~B3~~ ✅ | La caída a AP es definitiva | Un corte de router de unos minutos deja el nodo en AP hasta reiniciar | Bajo |
| ~~B4~~ ✅ | Minutos "60.00" en las balizas de posición | Posición inválida de forma esporádica | Bajo |
| ~~B5~~ ✅ | Auto-ACK sólo con identificadores numéricos | Mensajes de clientes con ID alfanumérico nunca reciben ACK | Bajo |
| ~~B6~~ ✅ | Digipeater: dedup sobre la trama completa y sin filtro de tramas propias | Repeticiones duplicadas en cuanto haya otro digi en cobertura | Medio |
| B7 | Servidor HTTP limitado a 3 sockets | Web lenta o con peticiones fallidas | Bajo |
| B14 | El códec Reed-Solomon no decodifica ni sus propios bloques | Con `il2p.enabled` no se recibe ninguna trama IL2P; lo transmitido no es RS estándar | Bajo (5 correcciones localizadas, ya validadas en PC) |
| B15 | La subida de imágenes SSTV pierde ~7 % de los bytes | Los JPEG subidos desde la web quedan corruptos y la subida dice "ok" | Bajo |
| ~~B16~~ | ✅ **HECHO (v1.5.0)** SSTV y morse transmitían sin LBT ni espera post-RX | Pisaban el canal aunque `tx.lbt_enabled` estuviera activo | Bajo |
| F1 | Reintento de mensajes con seguimiento de ACK | Hoy un mensaje perdido se pierde sin aviso | Medio |
| ~~F2~~ | ✅ **HECHO (v1.6.0)** Acceso al canal CSMA p-persistente | Menos colisiones con LBT activo | Medio |

## 1. Bugs

### 1.1 Prioridad alta

**B1. ✅ IL2P y modo TUN no encajan** — `smart_tx_frame` en [main/main.c](main/main.c), `il2p.c` · **[Leído]**

`smart_tx_frame` recorre las direcciones AX.25 de la trama para localizar el PID y manda por IL2P sólo lo que lleva PID 0xCC. En modo TUN las tramas que genera `ax25ip.c` son `00 00 08 00` + IPv4 en crudo, **sin cabecera AX.25**. El byte que se interpreta como PID es entonces un byte cualquiera de la cabecera IP o del transporte. Consecuencias con la configuración actual (`ip.mode = "tun"`, `il2p.enabled = true`):

- Casi todo el tráfico IP sale por AX.25 normal: **IL2P no aporta FEC** justo al tráfico para el que se activó.
- Cuando ese byte vale 0xCC por casualidad (del orden de 1/256 de los paquetes), la trama se envía por IL2P, cuyo codificador trata los 14 primeros bytes como indicativos y los pasa por una codificación de 6 bits con pérdida. El paquete llega corrupto o no llega.

Arreglo propuesto: que `ax25ip` marque sus tramas (o que `smart_tx_frame` reconozca el prefijo TUN) y que IL2P tenga un modo "payload opaco" sin recodificar indicativos. Ver también B14.

> **Hecho 2026-11-11** — parcial (v1.6.1): las tramas TUN (`00 00 08 00` + IPv4) van siempre por AX.25 (`smart_tx_or_tun` en `main.c`), lo que elimina la corrupción de ~1/256 paquetes. **Pendiente**: FEC IL2P para TUN (requiere modo payload opaco en `il2p.c` y arreglar B14).

**B2. ✅ Escritura bloqueante al cliente KISS desde la ruta de RX** — `wifi_transport_write` en [main/transport_wifi.c](main/transport_wifi.c) · **[Leído]**, impacto **[Hipótesis]**

`kiss_send_frame()` se llama desde `on_ax25_raw_frame`, que corre en `aprs_poll_task`. El `send()` es bloqueante y el socket no tiene `SO_SNDTIMEO`. Si el cliente KISS deja de leer (portátil suspendido, WiFi inestable), la tarea que procesa **toda** la recepción se queda parada hasta que el keepalive cierra la conexión (~25 s), o indefinidamente si el otro extremo sigue respondiendo a nivel TCP pero no lee. Mientras tanto se llena el FIFO de RX y se pierden tramas, ACK y digipeticiones.

Arreglo: `SO_SNDTIMEO` corto (100–200 ms) y descartar la trama si expira; mejor aún, una cola hacia una tarea de escritura para que el hook no toque nunca un socket.

El mismo patrón, más acotado, existe con el WebSocket: `audio_stream_ws_send_text` puede esperar hasta 200 ms de mutex más 500 ms por cliente dentro del hook.

> **Hecho 2026-11-11** — v1.6.2: `SO_SNDTIMEO` de 150 ms; si expira se descarta la trama, y tras un envío parcial o 100 descartes seguidos se cierra el cliente. El WebSocket (`audio_stream_ws_send_text`) sigue sin tratar.

**B3. ✅ La caída a AP es terminal** — bucle de reconexión en [main/transport_wifi.c](main/transport_wifi.c) · **[Leído]**

Tras 5 rondas fallidas el nodo pasa a AP y la tarea de reconexión deja de intentar (`if (state == WIFI_STATUS_HOTSPOT) continue;`). Un corte del router de unos minutos deja el equipo en AP hasta un reinicio manual, mal comportamiento para un nodo desatendido.

Arreglo: en modo AP, reintentar STA cada N minutos si no hay clientes asociados al AP (o usar APSTA).

> **Hecho 2026-11-11** — v1.6.2: desde el AP, sin clientes asociados, se reintenta STA cada 5 min (el AP cae mientras dura el intento).

**B4. ✅ Minutos "60.00" en la posición** — [main/gps.c](main/gps.c), `aprs_beacon_handler` en [main/api_rest.c](main/api_rest.c) · **[Leído]**

`gps.c` formatea los minutos con `%05.2f` sin acotar: 59,996' se imprime como `60.00`, que no es una posición APRS válida. En `api_rest.c` el comentario dice que se limita a 59.99 pero la comprobación es `lat_m >= 60.0`, que nunca se cumple. `remote_cmd.c` ya lo hace bien, con centésimas enteras.

Arreglo: una única función `aprs_format_position()` compartida por los tres sitios, usando la versión entera de `remote_cmd.c`. De paso se unifica el tipo de dato (`!` en `gps.c`, `=` en la API).

> **Hecho 2026-11-11** — v1.6.3: `aprs_format_position()` única (centésimas enteras con acarreo a grados) en `gps.c`, API y `remote_cmd.c`; tipo de dato unificado a `=`. NaN en la API no se rechaza.

**B5. ✅ El auto-ACK sólo acepta identificadores numéricos** — `try_auto_ack` en [main/main.c](main/main.c) · **[Leído]**

El bucle que extrae el ID sólo admite `0`–`9`. La especificación APRS permite de 1 a 5 caracteres alfanuméricos, y muchos clientes (APRSdroid, Xastir, YAAC) usan IDs con letras o el formato reply-ack `{AB}CD`. A esos mensajes no se les envía ACK: el remitente reintenta hasta rendirse. Además, como no se entra en la rama de dedup, el comando remoto tampoco se ejecuta.

Arreglo: aceptar alfanuméricos y cortar en `}`.

> **Hecho 2026-11-11** — v1.6.1: ID alfanumérico de 1–5 caracteres. El reply-ack `{AB}CD` confirma solo `AB`.

**B6. ✅ Digipeater** — [main/digipeater.c](main/digipeater.c) · **[Leído]** (afecta sólo si `digi.enabled`, hoy desactivado)

- El hash de dedup cubre la trama entera, **incluido el path**. La misma trama recibida de nuevo a través de otro digipeater lleva otro path, da otro hash y se vuelve a repetir. El dedup debe calcularse sobre origen + destino + información.
- No se descartan las tramas cuyo origen es el propio indicativo: el nodo puede repetir sus balizas cuando vuelven por otro digi.
- No hay tope de N (`WIDE7-7` se acepta) ni comprobación del límite de 8 repetidores al insertar el indicativo propio.
- No se atiende el indicativo propio puesto explícitamente en el path, sólo los alias.
- Copia de 600 B en la pila de `aprs_poll_task` (3072 B); el comentario dice `receive_audio_task` (8 KB), que ya no es cierto. Conviene medir con `/api/sys` o pasar el buffer a `static`.

> **Hecho 2026-11-11** — v1.6.4: dedup sobre destino+origen+info; descarte de origen propio; `DIGI_MAX_N`=2 y máximo 8 repetidores; indicativo propio explícito en el path; buffer `static`. Probado en host (8 casos); sin probar en hardware.

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

**B14. El códec Reed-Solomon está roto: IL2P no puede recibir nada** — [main/rs_codec.c](main/rs_codec.c), [main/il2p.c](main/il2p.c) · **[Calculado]** (prioridad alta si se usa IL2P)

`rs_codec.c` compilado en el PC tal como está en el repositorio:

| Prueba | Resultado |
|--------|-----------|
| `rs8_encode` + `rs8_decode` sin ningún error, 300 bloques por longitud (13, 50, 100, 205, 239 B) | 0 de 300 decodificados: devuelve −1 siempre |
| Cabecera (`rsh_encode` + `rsh_decode`) sin errores, 2000 bloques | 0 correctos, 1993 rechazados, 7 "corregidos" mal |

`il2p.c` descarta la trama cuando el decodificador devuelve un valor negativo, así que con `il2p.enabled` el receptor IL2P no entrega **ninguna** trama, ni siquiera las de otro equipo con este mismo firmware. La paridad que se transmite tampoco es la de un RS estándar, por lo que Dire Wolf no puede decodificarla. Corrige la versión anterior de este punto, que suponía que funcionaba entre dos equipos iguales.

Cinco defectos, aislados uno a uno:

1. **Polinomio generador mal construido** (`gf8_init` y `rsh_init`): el bucle debe ser `for (j = i + 1; j > 0; j--) tmp[j] = tmp[j-1] ^ gf8_mul(tmp[j], root);` seguido de `tmp[0] = gf8_mul(tmp[0], root);`.
2. **`rs8_encode` usa los coeficientes en orden inverso**: `rs8_gp[j]` debe ser `rs8_gp[RS8_NROOTS - j]`.
3. **`rsh_decode`**: la magnitud del error es `gf8_mul(s[0], gf8_inv(X))`, no `s[0]`.
4. **Cálculo de omega**: el bucle `m <= i && m <= L` debe ser `m <= i`.
5. **Forney**: sobra el factor `xk_fcr`; queda `err = gf8_mul(omega_val, gf8_inv(sigma_prime))`.

Con las cinco correcciones, en la misma prueba: 0 a 8 errores por bloque corregidos en 300 de 300 casos, 9 errores detectados y rechazados; cabecera con 0 o 1 error, 2000 de 2000. Una cabecera con 2 errores se "corrige" mal en ≈5,5 % de los casos, límite propio de tener sólo 2 bytes de paridad (conviene una comprobación de cordura de la cabecera decodificada).

Falta comprobar que el polinomio y la primera raíz coinciden con los de la especificación IL2P; la prueba sólo demuestra que el códec es coherente consigo mismo.

**B14 bis. IL2P no es interoperable aunque se arregle el códec** — [main/il2p.c](main/il2p.c) · **[Hipótesis]** (comparación hecha de memoria con la especificación)

La implementación difiere de IL2P estándar en varios puntos: tamaño de cabecera (14+2 frente a 13+2), codificación de indicativos, preámbulo 0x7E y exigencia de flags antes del sync, semilla del scrambler, reparto de bloques RS y uso de NRZI. Además descarta el path de digipeaters. Decidir si se quiere IL2P estándar (reescritura parcial contrastando con la especificación y con Dire Wolf) o un FEC propio documentado como tal.

**B15. La subida de imágenes SSTV corrompe el fichero** — `sstv_upload_handler` en [main/sstv.c](main/sstv.c) · **[Calculado]** (port del bucle a Python) (prioridad alta si se usa SSTV)

En el estado `MP_IMAGE_DATA`, tras escribir la parte segura del buffer, el bucle vuelve a entrar con 0 bytes seguros, avanza el índice hasta el final y el código de arrastre guarda 0 bytes. Los últimos ~42 bytes de cada ventana de 592 (la longitud del delimitador multipart) ni se escriben ni se arrastran. Una imagen de 20 000 bytes se guarda con 18 572 (≈7 % perdido), el estado termina en "hecho" y la respuesta es `ok`. La web sube el JPEG por esta ruta, así que todas las imágenes subidas desde ella están afectadas; las flasheadas con la imagen SPIFFS o subidas con `/api/spiffs/upload` no.

Comprobación en el dispositivo: comparar el `size` que devuelve la subida con el tamaño real del fichero enviado.

Arreglo: al quedarse sin bytes seguros, arrastrar el resto (`buf_len - i`) al principio del buffer en lugar de descartarlo. Otros fallos de la misma función: un `\r\n\r\n` partido entre dos lecturas no se detecta en los estados de cabecera; no se comprueba el espacio libre; si el último trozo llega corto y no aparece el delimitador final, se informa de éxito igualmente.

**B16. ✅ HECHO (v1.5.0) — SSTV, morse y repetidor transmiten sin LBT ni espera post-RX** — bucle de despacho en [AFSK.cpp](main/LibAPRS-esp32-i2s/src/AFSK.cpp) · **[Leído]**

La inhibición post-RX y el LBT sólo condicionan el despacho de `s_tx_queue`. El `s_dispatch_hook()` (morse, SSTV, repetidor) se ejecuta con la única condición de no estar ya transmitiendo. Una baliza CW o una imagen SSTV salen aunque el canal esté ocupado o se acabe de recibir una trama. Arreglo: aplicar al hook la misma comprobación (con el mismo tiempo máximo de espera).

**Resuelto el 2026-11-11 (v1.5.0, commits `748d048` y, en la librería, `8871690`)**: la comprobación se extrajo a `tx_gate_open()` (post-RX + LBT + espera máxima) y se expone como `afsk_tx_clear_to_send()`; `morse_check_and_dispatch()` y `sstv_dispatch_if_pending()` la llaman sólo cuando hay algo pendiente y arrancan únicamente si devuelve true (si no, reintentan en la siguiente iteración; la petición SSTV sigue en cola y el semáforo de morse queda tomado por la tarea de baliza, que espera hasta 60 s). La cola AFSK y el hook mantienen estados independientes. Además la puerta sólo se evalúa si hay una trama en cola (antes el temporizador LBT y su aviso corrían con la cola vacía). **El repetidor queda fuera a propósito**: retransmite una señal recién recibida, de modo que el canal está ocupado por su propia entrada y el LBT lo bloquearía siempre (documentado en README, CLAUDE.md y `repeater.c`). Compila; sin probar en hardware. Limitación: sólo se comprueba el canal antes de empezar; una imagen SSTV ya iniciada no se interrumpe.

**B17. Rutas de error de SSTV** — [main/sstv.c](main/sstv.c) · **[Leído]**

- El PTT y la cabecera VIS salen **antes** de abrir y validar el JPEG: con un fichero corrupto (ver B15) o de ancho no admitido se transmite ~1 s de VIS y nada más, y aun así se envía `sstv_done` por WebSocket.
- Si el fichero no se puede abrir no hay ningún evento hacia la web.
- La altura de la imagen no se comprueba; un modo desconocido se transmite como Martin M1 sin avisar.
- La orden de parada se borra al empezar la transmisión: un `stop` enviado mientras la petición está en cola se pierde.
- Una transmisión dura hasta ~114 s dentro de `receive_audio_task`: durante ese tiempo no hay RX, la cola AFSK no se despacha y un cliente KISS queda bloqueado por la contrapresión (15 s) y pierde tramas. Es inherente al diseño half-duplex, pero conviene documentarlo y rechazar SSTV si hay tramas en cola.

Arreglo del primer punto: decodificar la cabecera del JPEG y validar dimensiones antes de pulsar el PTT, y emitir un evento `sstv_error` con el motivo.

**B18. Squelch HFNE** — [main/squelch_sf.c](main/squelch_sf.c) · **[Leído]**, impacto **[Hipótesis]** (sólo con monitor o repetidor activos)

- `send_ws()` llama a `audio_stream_ws_send_text` desde `receive_audio_task` cada ~100 ms. Es el mismo patrón bloqueante que B2, pero en la tarea que lee el ADC: un cliente web lento puede provocar desbordamientos del ADC. Mover el envío a otra tarea (bandera + tarea de baja prioridad).
- El suelo de ruido nunca baja si el nivel de ruido disminuye (por ejemplo al bajar el volumen). `carrier_detect` ya lo resuelve reaprendiendo desde v1.4.7; aquí falta.
- Estado de Goertzel en `int32` con coeficiente en coma flotante: se trunca en cada muestra. Usar `float` para el estado.

**B19. Chat de la web** — JavaScript de [main/spiffs_data/index.html](main/spiffs_data/index.html) · **[Leído]**

- La conversación muestra como burbuja **cualquier** trama del contacto activo que no sea un ACK: balizas, mensajes a terceros, telemetría. Debe filtrarse por mensajes (`:DEST     :`) dirigidos al indicativo propio.
- Los mensajes enviados sólo existen en la memoria de la página: al recargar desaparecen, junto con el estado "pendiente de ACK". El firmware no los devuelve en `/api/log`.
- La expresión del ACK sólo admite dígitos (`/^ack\d+/`), coherente con B5; no se trata `rej`.
- Un mensaje sin ACK queda indefinidamente "pendiente": no hay tiempo límite ni indicación de fallo (ver F1).
- `parseAPRSPos` sólo entiende posiciones sin comprimir: Mic-E y comprimidas (la mayoría de los equipos portátiles y APRSdroid) no dan posición.

**B20. Indicadores que siguen mostrando el demodulador v1** — [main/display.c](main/display.c), pestaña STATS de la web · **[Leído]**

Con el v1 desactivado, la fila `APRS:` del display muestra `v1/v2` con el primer valor siempre a 0, y la pestaña STATS conserva la columna v1 vacía. Además, el contador `frames_err` del pie del LOG cuenta sobre todo CRC erróneos producidos por ruido en reposo (≈90 % según las capturas SURVEY), así que no indica calidad. Mostrar sólo v2 y, para los errores, el contador limitado a portadora (`er`).

**B21. Consola RF** — [main/rf_console.c](main/rf_console.c) · **[Leído]**

- `config` vuelca el JSON completo por un enlace de 1200 bps (más de un minuto de canal). Limitarlo a un resumen o a una sección (`config aprs`).
- Si `ax25ip` no está levantado al iniciar, la consola queda desactivada hasta el reinicio. `listen()` sin comprobar; lectura byte a byte.
- Sólo cuatro comandos; ver F7.

**B22. Diagnóstico de red** — [main/netdiag.c](main/netdiag.c) · **[Leído]**

- `POST /api/netdiag/stop` puede bloquear hasta 2 s la única tarea del servidor HTTP: en ese tiempo no se atiende ninguna otra petición.
- `GET /api/netdiag` usa ~3,7 KB de pila (varios buffers locales) de los 8 KB de la tarea httpd. Sin margen medido; pasar los buffers a heap o `static`.
- La web siempre pide `count: 1` en el ping.

### 1.3 Prioridad baja y limpieza

- **Uso tras liberar en `TNC_MODE_APRS`** ([main/main.c](main/main.c)): `APRS_setCallsign` se llama después de `config_free_json(cfg)`. El modo está muerto desde que se desactivó el demodulador v1; lo razonable es borrar la rama. **[Leído]**
- **`rx_frame_is_duplicate` (300 ms)** sigue activo con un solo demodulador. Es inocuo, pero descartaría una retransmisión legítima idéntica dentro de la ventana. Hay que mantenerlo si se implementan los slicers múltiples del informe del módem.
- **Orden de arranque**: el hook de audio se instala antes de `carrier_detect_init`/`squelch_sf_init`/`rx_survey_init`. Hoy no falla; moverlo al final lo hace robusto.
- **`wav_server_task`** ([main/audio_stream.c](main/audio_stream.c)): `bind`/`listen` sin comprobar, `recv` sin timeout, y la comprobación de vida `send(fd, NULL, 0)` no detecta nada.
- **`/api/log`**: duplica hasta 250 entradas con el mutex tomado; pico de heap de decenas de KB en un sistema con presión de memoria. Serializar por trozos (`httpd_resp_send_chunk`).
- **`/api/sys`**: lista de tareas fija; faltan `httpd`, `iplog`, `cap_dns`, `ota_net`, `rx_survey`… Usar `uxTaskGetSystemState`.
- **`/api/spiffs/upload`**: no comprueba espacio libre antes de escribir.
- **`/api/aprs/send`**: no valida ni pasa a mayúsculas el destinatario.
- **KISS** ([main/kiss.c](main/kiss.c)): sólo trata DATA en el puerto 0 y, desde v1.6.0, P y SlotTime (F2); TXDELAY, TXTAIL y FullDuplex se siguen ignorando (el preámbulo sale de `aprs.*` en `config.json`).
- **`ax25ip.c`**: `ip_info()` usa un buffer `static` desde dos tareas (sólo afecta al texto del log).
- **WiFi**: no se llama a `esp_wifi_set_ps(WIFI_PS_NONE)`; el modem-sleep por defecto añade latencia a KISS y al audio. SNTP fijo a `pool.ntp.org` y sin zona horaria. Canal del AP fijo a 1.
- **Respuestas de `remote_cmd`**: salen como mensaje con `{NNN}` nuevo y sin reintento; `tx,morse` no responde; los comandos desconocidos se ignoran en silencio.
- **Morse** ([main/morse.c](main/morse.c)): la tabla sólo tiene A–Z y 0–9 (un indicativo con `/`, como `EA1XXX/P`, pierde la barra); manipulación sin rampa (clics de manipulación, más ancho de banda); `wpm` y tono sin límites; la primera baliza sale tras un periodo completo.
- **TUNE** ([main/delay_tune.c](main/delay_tune.c)): `ip.remote.addr` se lee sólo al arrancar (cambiarlo exige reinicio); el comentario de cabecera dice 150–2000 ms y el barrido real es 50–2000 ms.
- **Display** ([main/display.c](main/display.c)): errores de I2C ignorados al refrescar (si la pantalla se desconecta no se detecta); ~2 KB estáticos aunque no haya pantalla.
- **Repetidor** ([main/repeater.c](main/repeater.c), deshabilitado): mismo envío WS desde `receive_audio_task` que B18 y buffer de 2 KB en la pila; revisar antes de reactivarlo.
- **OTA**: `espota.py` no genera eventos de progreso en la web; la web sólo confirma el éxito por el evento WS `ota_done` (sin WebSocket no hay confirmación).
- **Web**: `downloadLog` revoca la URL del blob inmediatamente (la descarga puede fallar en algunos navegadores); `repRefresh` se llama dos veces al cargar; la pestaña activa se restaura buscando el texto del `onclick`.
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

Las tres primeras filas, la ausencia de `netdiag` y `/api/sys` en la documentación, el rango de TUNE del README (350–1950 ms; real 50–2000 ms) y el estado de IL2P se corrigieron en CLAUDE.md, README.md y PROGRESS.md el 2026-11-11. Las tres últimas son comentarios del código y siguen pendientes.

El margen de DMA con 4 descriptores es la mitad del que describía la trampa de `TX_SAMPLE_BUFLEN`; conviene confirmar cuál de los dos valores es el deseado.

## 2. Funcionalidades

Ordenadas por relación utilidad/esfuerzo. Las marcadas con (backlog) ya figuran en `doc/pending/features_and_improvements.md`; aquí sólo se priorizan.

### 2.1 Recomendadas a corto plazo

**F1. Reintento de mensajes con seguimiento de ACK** (backlog). Tabla pequeña de mensajes pendientes (destino, ID, texto, intentos, próximo envío), reintento con espera creciente (por ejemplo 30/60/120 s, 3–5 intentos), cancelación al recibir `ackNNN` y evento WS `msg_failed`. Sirve al chat de la web y a las respuestas de `remote_cmd`. Requiere B5 y B8.

**F2. ✅ HECHO (v1.6.0) — Acceso al canal CSMA p-persistente y parámetros KISS.** Hoy el TX sale en cuanto el canal queda libre: si dos estaciones esperan, colisionan siempre. Añadir slot aleatorio (`slottime` 100 ms, `persist` 63) al despacho de `s_tx_queue`, y aceptar los comandos KISS TXDELAY/P/SlotTime/TXTAIL para que APRSdroid, Xastir o Dire Wolf puedan ajustarlos. Tiene más valor cuando LBT esté activado.

**Implementado el 2026-11-11 (v1.6.0, commits `c83268c` y, en la librería, `8871690`)**: con LBT activo y el canal libre se transmite si `random(0..255) <= persist`; si no, se espera un slot y se vuelve a escuchar (un canal ocupado reinicia el proceso). Configuración `tx.persist` (63) y `tx.slottime_ms` (100); `persist` 255 o `slottime_ms` 0 lo desactivan. Se aplica a la cola AFSK, morse y SSTV, y no actúa con LBT desactivado. El host puede ajustarlo con los comandos KISS **P** y **SlotTime** (×10 ms). **No implementado**: TXDELAY y TXTAIL por KISS (siguen fijos en `aprs.*`; las unidades de preámbulo de la librería no coinciden con las KISS y habría que mapearlas). Efecto en latencia: con P=63 cada trama espera de media ≈3 slots (≈300 ms) cuando el canal está libre; subir `persist` si se prioriza latencia. Compila; sin probar en hardware.

**F3. Path configurable.** `WIDE1-1,WIDE2-x` está fijo en la librería y se aplica también a los ACK. Hacerlo configurable (`aprs.path`), con opción de path vacío y un path distinto, normalmente más corto, para ACK y mensajes.

**F4. Baliza: al arrancar, con comentario y símbolo configurables, y SmartBeaconing** (backlog en parte). Enviar la primera baliza al obtener fix; `gps.symbol`, `gps.comment`, altitud opcional; posición fija configurable para nodos sin GPS; SmartBeaconing si el equipo se usa en móvil.

**F5. Varios clientes KISS simultáneos** (backlog). Hoy `listen(fd, 1)` y un solo cliente. Con 2–3 se puede tener a la vez un cliente APRS y un monitor. Hacerlo junto con B2 (cola de escritura por cliente).

**F6. Reconexión desde AP** (es B3) y `reboot_required` en la config (es B12).

**F7. Consola RF útil.** Añadir `stats` (tramas RX/TX, CRC), `heard`, `beacon`, `reboot` y `config <sección>`; son las órdenes que se necesitan cuando el nodo sólo es alcanzable por radio.

**F8. Chat persistente.** Guardar los mensajes enviados en el log del firmware (tipo `sent`, con `msg_id` y estado) para que sobrevivan a una recarga y se vean desde otro navegador. Encaja con F1.

### 2.2 Medio plazo

- **Posiciones Mic-E y comprimidas en la web** (ver B19): sin ellas el mapa y la tabla de estaciones quedarán medio vacíos.
- **SSTV**: validación previa al PTT con `sstv_error` (B17), más tamaños de reescalado en la web (hoy sólo 320×256 y 320×240) e identificación CW al final.
- **Morse**: carácter `/`, rampa de 5 ms en los flancos.

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

- **Sin tests ni CI.** `rs_codec.c` es el caso más claro: una prueba de ida y vuelta de diez líneas habría detectado B14. Otros tres módulos se pueden probar en host sin tocar hardware y cubren los bugs de este informe: formateo de posición (B4), análisis de mensajes/ACK (B5) y digipeater (B6). `kiss.c` y `dedup.c` también son puros.
- **Formateadores duplicados** (posición ×3, separación indicativo/SSID ×3 en `try_auto_ack`): extraer a un módulo `aprs_util`.
- **Pilas sin medir**: `aprs_poll_task` (3072 B) ejecuta el hook completo, incluidos digipeater, cJSON y `snprintf`. Leer `/api/sys` tras una sesión con tráfico antes de dar el valor por bueno.
- **Librería como repo anidado sin enlazar** y `config.json` seguido por git: ya conocidos, siguen pendientes.

## 4. Orden de trabajo sugerido

1. B1, B2, B3, B4, B5: cambios pequeños y localizados con efecto directo en el uso diario.
2. B15 y B17 si se usa SSTV. (B16 hecho)
3. B14 (códec RS, con su prueba en host) junto con B1 y B11 si se quiere IL2P entre equipos propios; si no, desactivar `il2p.enabled`, que hoy sólo resta.
4. B7, B8, B9, B10, B18 y los comentarios pendientes de 1.4.
5. F1 (reintentos) con F8 y B19; F3 (path).
6. Captura de audio crudo y banco de pruebas del módem.
7. B6 antes de activar el digipeater. (F2 hecho)
8. Decisión sobre IL2P estándar (B14 bis).

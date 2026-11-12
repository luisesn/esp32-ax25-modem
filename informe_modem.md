# Informe del módem: cómo decodificar más tramas

Fecha: 2026-11-10 (revisado el 2026-11-11: códec Reed-Solomon) · Firmware analizado: v1.4.7 · Código: `main/LibAPRS-esp32-i2s/src/AFSK.cpp` (demodulador v2, el único activo).

## Alcance y método

- La cadena de RX se ha leído completa y la respuesta de cada filtro se ha **calculado numéricamente** a partir de los coeficientes del código.
- **No se ha medido la tasa de decodificación**: ninguna propuesta se ha probado, ni offline ni en hardware. Las ganancias que se citan son las habituales en la literatura de módems AFSK 1200 (Dire Wolf, pruebas con el CD de WA8LMF); son orientativas, no promesas.
- Por eso la primera recomendación es montar un banco de pruebas: sin él no se puede saber qué cambio ayuda y cuál estorba.

Etiquetas: **[Calculado]** = verificado numéricamente con los coeficientes; **[Leído]** = se ve en el código; **[Hipótesis]** = efecto esperado sin medir.

## Resumen

1. Los dos filtros del demodulador **no son los que dicen sus comentarios**. El paso banda tiene el pico en ≈2000 Hz y no en 1700 Hz, y el paso bajo de después del detector corta en ≈1350 Hz en vez de 600 Hz. El demodulador funciona en parte por casualidad: la inclinación del paso banda compensa el de-énfasis de la salida de altavoz.
2. El filtro de diezmado 48 k → 9,6 k deja pasar el ruido de alta frecuencia casi sin atenuar, y con el squelch de la radio abierto ese ruido es abundante.
3. El AGC se queda sin recorrido con señal fuerte (tope en 512 cuentas de 2048) y todo el DSP posterior trabaja en 8 bits con recortes.
4. El decisor es un umbral fijo en 0 sobre un detector asimétrico: sensible al desequilibrio entre tonos ("twist"), que es justo lo que varía de una estación a otra.
5. No hay recuperación de tramas con un bit erróneo ni FEC efectivo: el códec Reed-Solomon de IL2P no decodifica ni sus propios bloques (comprobado en PC) y además IL2P no se aplica al tráfico en modo TUN (B14 y B1 del otro informe).

Orden recomendado: **banco de pruebas → filtros → decisor adaptativo o múltiple → demodulador por correlación → reparación de bits**.

## 1. Cadena de recepción actual

```
ADC 48 kHz, 12 bit
 → FIR de 5 coeficientes {6,58,128,58,6}/256 y diezmado ×5 → 9600 Hz
 → resta de continua + AGC (objetivo ±96 en int8; envolvente limitada a [4, 512])
 → "de-énfasis" opcional (desactivado en config)
 → paso banda biquad (Q15)
 → detector retardo-y-multiplica (retardo 4 muestras)
 → paso bajo de 2.º orden
 → decisor: signo (umbral 0)
 → DPLL de fase fija + voto por mayoría de 5 muestras
 → NRZI → HDLC → FIFO → CRC
```

## 2. Hallazgos

### 2.1 Paso banda mal centrado y sin margen — **[Calculado]**

Coeficientes `BP_B0=14871, BP_B2=−14871, BP_A1=−12058, BP_A2=17897`. El comentario dice "centro 1700 Hz, Q=2". Respuesta real:

| Frecuencia | Ganancia |
|-----------:|---------:|
| 300 Hz | −16 dB |
| 1200 Hz (mark) | −1,8 dB |
| 1700 Hz | +4,0 dB |
| 2000 Hz | +6,0 dB (pico) |
| 2200 Hz (space) | +5,5 dB |
| 3000 Hz | −1,9 dB |

Dos consecuencias:

- **Inclinación de ≈7,3 dB a favor del tono space.** La salida de altavoz de la radio está de-enfatizada (≈5,3 dB menos en 2200 Hz que en 1200 Hz para un emisor con pre-énfasis estándar), así que el filtro compensa casi exactamente el caso típico. Es una coincidencia afortunada, pero fija: una estación que transmita sin pre-énfasis (audio plano, frecuente en trackers y TNC por la entrada de datos) llega con el space ≈7 dB por encima del mark y el decisor se desplaza.
- **Recorte.** El AGC deja la señal en ±96 y el filtro añade hasta +6 dB (±190) antes de saturar a ±127. El tono space se recorta dentro del propio filtro, lo que genera armónicos y reduce la ventaja del detector.

### 2.2 Paso bajo posterior al detector demasiado ancho — **[Calculado]**

Coeficientes `LP_B0=1048, B1=2097, B2=1048, A1=−23148, A2=7339`. El comentario dice "Butterworth 600 Hz". Respuesta real: ganancia en continua −12,2 dB, corte a −3 dB en ≈1350 Hz, y sólo ≈−10 dB entre 2200 y 2400 Hz.

El detector retardo-y-multiplica produce, además de la señal útil, una componente al doble de la frecuencia del tono (2400 y 4400 Hz, ambas por debajo de Nyquist a 9600 Hz). Con −10 dB de atenuación ese rizado llega al decisor y mueve los cruces por cero, que es de donde el DPLL saca el reloj. Un Butterworth de 600 Hz real a 9600 Hz sería aproximadamente `b0=981, b1=1962, b2=981, a1=−47650, a2=18809` (Q15); valores a recalcular y validar en el banco, donde probablemente el óptimo esté entre 600 y 900 Hz.

### 2.3 Filtro de diezmado que apenas filtra — **[Calculado]**

El FIR `{6,58,128,58,6}/256` antes de diezmar ×5 comparado con una simple media de 5 muestras:

| Frecuencia que se pliega en banda | FIR actual | Media de 5 |
|-----------------------------------:|-----------:|-----------:|
| 7400 Hz | −2,6 dB | −11 dB |
| 8400 Hz | −3,4 dB | −16,7 dB |
| 10800 Hz | −5,6 dB | −18,6 dB |
| 12000 Hz | −6,9 dB | −14 dB |
| Ruido blanco (ganancia total) | −4,5 dB | −7 dB |

Todo lo que haya entre 7,4 y 12 kHz se pliega sobre la banda de 1200–2200 Hz casi sin atenuar. Con el squelch abierto la radio entrega ruido de FM de banda ancha; cuánto queda por encima de 7 kHz en la salida de altavoz del UV-K5 no está medido **[Hipótesis]**, pero el arreglo es barato: un FIR de 15–25 coeficientes con corte hacia 3 kHz (simétrico, en enteros, ~25 multiplicaciones por muestra de salida) o, como mínimo, la media de 5.

### 2.4 AGC con el tope demasiado bajo — **[Leído]**

`if (peak > 512) peak = 512`. El ADC da ±2048 cuentas. Cualquier señal con envolvente por encima de 512 se escala con la ganancia mínima y satura en ±127 a partir de ≈677 cuentas. Coincide con lo observado en SURVEY a volumen máximo (AGC=512, pico 127 saturado). Subir el tope a 2048 da 12 dB más de margen sin coste.

El AGC tiene ataque rápido (~8 muestras) y caída lenta (~512 muestras ≈ 53 ms). Con el squelch abierto, el ruido de reposo es **más fuerte** que la señal (al llegar la portadora el nivel cae), así que la ganancia tiene que subir al principio de cada trama y lo hace con la constante lenta. Los 350 ms de preámbulo por defecto lo cubren, pero las estaciones con TXDELAY corto (100–150 ms) dejan menos margen **[Hipótesis]**. Una caída algo más rápida o un AGC por tono (ver 3.4) lo resuelve.

### 2.5 Ruta de 8 bits — **[Leído]**

Tras el AGC todo es `int8` con saturación en cada etapa. El ESP32 multiplica 32×32 en un ciclo: no hay ahorro real frente a `int16`/`int32`, y sí pérdida de margen dinámico y recortes (2.1). Pasar la ruta a `int16` elimina los recortes y permite un AGC menos agresivo.

### 2.6 Decisor fijo en cero sobre un detector asimétrico — **[Calculado]** la asimetría, **[Hipótesis]** el impacto

Con retardo de 4 muestras a 9600 Hz, el detector da −1,0·A² para mark y +0,866·A² para space: ya nace desequilibrado un 13 %. Cualquier diferencia de amplitud entre tonos desplaza aún más el punto medio, y el umbral sigue en 0. Con twist fuerte, los bits del tono débil se acortan hasta desaparecer.

Dire Wolf atacó este mismo problema con varios decisores en paralelo a distintas ganancias relativas, y es la mejora a la que atribuye el mayor salto de tramas decodificadas en tráfico real, donde cada estación llega con un twist distinto.

### 2.7 Recuperación de reloj y decisión de bit — **[Leído]**

DPLL con corrección de fase fija en cada transición y voto por mayoría de 5 muestras ya decididas (bits duros). Es el esquema del LibAPRS original. Limitaciones: no distingue entre "enganchado" y "buscando" (la misma corrección que permite enganchar rápido hace que el ruido mueva el reloj a mitad de trama), y el voto por mayoría descarta la información de amplitud.

### 2.8 Squelch del v2 inoperante y HDLC siempre activo — **[Leído]**

`s_v2_squelch_power > s_squelch_threshold` se evalúa después del AGC, que normaliza el nivel: está siempre abierto. Con la radio sin squelch, el HDLC procesa ruido continuamente: según las capturas SURVEY, ≈90 % de los errores de CRC son ruido en reposo. No pierde tramas por sí mismo, pero ensucia las estadísticas, gasta CPU y, sobre todo, hace peligrosa cualquier reparación de bits (3.5) si no se condiciona a la detección de portadora.

### 2.9 Filtro de "de-énfasis" — **[Calculado]**

Desactivado en la configuración actual, y debe seguir así: atenúa 12,9 dB en 1200 Hz y 9,6 dB en 2200 Hz. Sólo corrige 3,3 dB de inclinación a cambio de ≈10 dB de nivel en una ruta de 8 bits. Eliminarlo o sustituirlo por un ecualizador de inclinación configurable (ver 3.2).

### 2.10 Estado del v2 en variables globales — **[Leído]**

El estado del demodulador v2 vive en variables `static` sueltas. Para tener varios decisores o demoduladores en paralelo hay que agruparlo antes en una estructura por instancia. Es un refactor mecánico, pero es requisito de 3.3.

## 3. Propuestas, por orden

### 3.1 Banco de pruebas offline (requisito de todo lo demás)

- Compilar el demodulador v2 para PC: `AFSK_adc_isr` y sus filtros son C puro, basta aislarlos de FreeRTOS con un pequeño adaptador.
- Entradas: (a) el **TNC Test CD de WA8LMF** (pista 1 plana y pista 2 de-enfatizada), referencia habitual: los buenos módems por software decodifican del orden de 950–1000+ tramas en la pista 1, y los basados en el LibAPRS original bastante menos; (b) **capturas propias** con esta radio a varios volúmenes.
- Para (b) hace falta un endpoint que vuelque el audio crudo (12 bits, idealmente a 48 kHz antes del diezmado). Las filas de SURVEY resumen el audio y no sirven para reinyectarlo.
- Métrica: tramas con CRC correcto por fichero. Comparar con Dire Wolf sobre el mismo audio para saber cuánto queda por ganar.
- El mismo banco sirve de test de regresión en CI.

Esfuerzo: 1–2 días. Sin esto, cada cambio de filtro es una apuesta que sólo se puede comprobar "a oído" con una segunda radio.

### 3.2 Arreglar los filtros (bajo riesgo, primer cambio)

1. Diezmado: FIR de 15–25 coeficientes con corte ≈3 kHz (o media de 5 como mínimo).
2. Ruta en `int16` y tope del AGC en 2048.
3. Paso banda diseñado de verdad: plano entre 1200 y 2200 Hz (dos biquads o un FIR corto), con margen para no recortar.
4. Ecualización de inclinación **separada y configurable** (`rx.tilt_db`, por defecto ≈+5 dB para salida de altavoz de-enfatizada), en lugar de dejarla implícita en un paso banda descentrado.
5. Paso bajo posterior al detector de 600–900 Hz real, con ganancia unidad.

Ganancia esperada **[Hipótesis]**: notable con señales débiles y con estaciones de audio plano; con señales fuertes de una radio cercana puede no notarse. Cambiar una cosa cada vez y medir, porque el filtro actual compensa el de-énfasis por accidente y "corregirlo" sin añadir la ecualización del punto 4 puede empeorar el resultado.

### 3.3 Decisor adaptativo y decisores múltiples (la mayor ganancia esperable en tráfico real)

- **Umbral adaptativo**: seguir los picos positivo y negativo de la salida del detector con caída lenta y decidir en el punto medio. Barato y corrige twist moderado.
- **Decisores múltiples**: 3–5 ramas con ganancia relativa mark/space distinta (por ejemplo −6, −3, 0, +3, +6 dB), cada una con su DPLL y su HDLC. El dedup de `on_ax25_raw_frame` (`rx_frame_is_duplicate`, 300 ms) ya elimina las copias. Coste: por rama sólo se repiten decisor + DPLL + HDLC (unas decenas de operaciones por muestra y un buffer de trama de 600 B); los filtros se comparten.
- Requiere 2.10 (estado por instancia) y revisar `rx_stats` para no contar errores de CRC por rama como errores reales.

### 3.4 Demodulador por correlación (mark/space)

> **Hecho en v1.17.0** (`afsk_v2_correlate`, ventana de 10 muestras): en el banco gana ~4 dB frente al ruido (SNR 7 dB: 5→95 de 100 tramas). El DPLL de §2.7 se cambió en v1.16.0. Los hallazgos de filtros (§2.1, §2.2, §2.6) ya no aplican con `AFSK_V2_CORRELATOR 1`.

Sustituir retardo-y-multiplica por dos correladores en cuadratura (1200 y 2200 Hz, ventana de un bit), con la envolvente de cada tono normalizada por separado (AGC por tono) y decisión por diferencia. Ventajas: mejor comportamiento con ruido (unos 2–3 dB frente al detector de retardo según la literatura), inmunidad al twist por construcción, y una medida natural de calidad y de portadora de datos (DCD). Coste: 4 acumulaciones por muestra con ventana deslizante; asumible a 9600 Hz en un ESP32, pero hay que medir CPU porque `receive_audio_task` ya hace Goertzel en coma flotante en `carrier_detect`.

Complementos: integración por bit con valores blandos en lugar del voto por mayoría, y DPLL con inercia variable (corrección grande buscando, pequeña enganchado).

Es el cambio más grande. Hacerlo sólo si, tras 3.2 y 3.3, el banco muestra una diferencia apreciable frente a Dire Wolf.

### 3.5 Recuperación de tramas con errores

- **Reparación de un bit**: si el CRC falla, probar a invertir cada bit de la trama (≤ 600×8 CRC; con CRC incremental son microsegundos) y aceptar si cuadra **y** la trama pasa comprobaciones de cordura (direcciones con caracteres válidos, control 0x03, PID 0xF0, texto imprimible). Imprescindible limitarlo a tramas recibidas con portadora detectada (`carrier_detect`), porque sobre ruido produciría tramas falsas. Aplicarlo sólo a APRS, no al tráfico IP (PID 0xCC), donde una trama falsa hace más daño que una perdida. Marcar las tramas reparadas en el JSON (`fix:1`) y no digipetearlas.
- **Códec Reed-Solomon (requisito de FX.25 e IL2P)** — **[Calculado]**: `rs_codec.c`, compilado en el PC, rechaza todos los bloques aunque no tengan errores (0 de 300 por longitud; cabecera 0 de 2000). Con cinco correcciones localizadas (detalle en B14 de [informe_bugs_funcionalidades.md](informe_bugs_funcionalidades.md)) corrige hasta 8 errores por bloque en 300 de 300 casos. Añadir esa prueba de ida y vuelta al banco offline.
- **IL2P**: con `il2p.enabled` el receptor IL2P no entrega hoy ninguna trama (B14), el tráfico en modo TUN no pasa por IL2P (B1) y el formato previsiblemente no interopera con Dire Wolf (B14 bis). Mientras no se arregle, activarlo no aporta nada a la recepción. Arreglados B14 y B1, es la mejora de fiabilidad más barata para el enlace IP entre dos equipos propios.
- **FX.25** (RX): compatible con AX.25 normal y cada vez más extendido; reutiliza el códec RS una vez corregido (ojo: FX.25 usa otros parámetros de código, comprobar polinomio y primera raíz). Prioridad baja mientras no haya estaciones FX.25 en la zona.

### 3.6 Portadora de datos, nivel y diagnóstico

- El squelch HFNE (`squelch_sf.c`) no participa en la decodificación (sólo monitor, repetidor y LBT); sus defectos están en B18 del otro informe.
- Condicionar HDLC y estadísticas a un DCD (el de `carrier_detect` o, mejor, el del propio demodulador en 3.4) y eliminar el squelch del v2.
- **Nivel de audio**: las capturas indican que a volumen máximo se satura (pico 127, AGC en el tope) y a volumen medio no. Mientras no se suba el tope del AGC, trabajar a volumen medio. Añadir a la web un aviso de saturación.
- Diagnóstico por trama: añadir al JSON `aprs` el twist medido (diferencia mark/space en dB) y, con decisores múltiples, qué rama decodificó. Con esos dos datos se ve enseguida si las tramas que se pierden son las de twist alto.

## 4. Lado de transmisión (para que los demás te decodifiquen)

No analizado con el mismo detalle; tres puntos a comprobar **[Hipótesis]**:

- **Pre-énfasis**: el audio del DAC entra por el micrófono de la radio, que aplica pre-énfasis. El receptor ajeno verá los tonos con la inclinación típica, lo cual es correcto para la mayoría, pero conviene medir la desviación real de cada tono con la segunda radio.
- **Nivel de DAC / desviación**: una desviación excesiva recorta en el limitador de la radio y es la causa más común de que una estación "se oiga fuerte pero no se decodifique". Buscar ≈3 kHz de desviación de pico.
- **TXDELAY**: 350 ms es conservador; cuando el UV-K5 conmute de forma fiable se puede bajar a 200–250 ms para ocupar menos canal.

## 5. Plan resumido

| Paso | Qué | Esfuerzo | Riesgo |
|-----:|-----|----------|--------|
| 1 | Captura de audio crudo + banco offline + referencia con Dire Wolf | 1–2 días | Ninguno (no toca el firmware de RX) |
| 2 | Diezmado correcto, `int16`, tope AGC 2048 | Horas | Bajo |
| 3 | Paso banda plano + inclinación configurable + paso bajo real | 1 día con el banco | Medio (hoy la compensación es implícita) |
| 4 | Umbral adaptativo | Horas | Bajo |
| 5 | Estado por instancia + decisores múltiples | 2–3 días | Medio |
| 6 | Reparación de un bit condicionada a portadora | 1 día | Medio (tramas falsas si se relaja) |
| 7 | Demodulador por correlación + DPLL mejorado | 1 semana | Alto; sólo si el banco lo justifica |

Los pasos 2–4 se pueden validar por completo en el banco antes de flashear.

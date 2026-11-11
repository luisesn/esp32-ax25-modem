#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Tabla en memoria de estaciones oídas (20 entradas, se reemplaza la más antigua).
void heard_init(void);

// Registra una trama AX.25 cruda (sin CRC): parsea el indicativo-SSID origen.
// q = calidad 1-4 (<=0: sin dato, no entra en la media). Rápido y sin bloqueo largo.
// Si el info es una posición APRS (! = / @ sin comprimir o comprimida, o Mic-E ' `), guarda lat/lon.
void heard_note(const uint8_t *ax25, size_t len, int q);

// ¿Se oyó esa estación (base sin SSID + ssid) hace <= secs segundos?
bool heard_seen_recent(const char *call, int ssid, uint32_t secs);

// Texto para consola (CRLF), hasta max_rows filas más recientes primero (0 = 10).
// Devuelve bytes escritos (sin contar el NUL).
int heard_list(char *buf, size_t n);

// JSON: {"heard":[{"call":"X-1","age":12,"n":3,"q":3.2,"lat":..,"lon":..},...]}
// (q sólo si hay dato; lat/lon sólo si hay posición). Devuelve longitud o -1 si
// no cabe en buf.
int heard_json(char *buf, size_t n);

#ifdef __cplusplus
}
#endif

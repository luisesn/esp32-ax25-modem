#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "cJSON.h"

typedef struct {
    bool    valid;          // RMC status A = true position fix
    bool    data_received;  // at least one valid NMEA sentence parsed
    double  lat;            // decimal degrees, + = N, - = S
    double  lon;            // decimal degrees, + = E, - = W
    uint8_t satellites;
    uint8_t fix_quality;    // 0 = no fix, 1 = GPS, 2 = DGPS
    float   speed_knots;
    float   course_deg;
    bool    alt_valid;      // GGA con fix: altitude_m válida
    float   altitude_m;     // altitud sobre el nivel del mar (GGA), metros
    char    time_utc[7];    // "HHMMSS\0"
    char    date_utc[7];    // "DDMMYY\0"
} GpsPosition;

// Updated by gps_task; guarded by an internal mutex.
// Callers may do a quick struct-copy under gps_lock_pos() / gps_unlock_pos().
extern GpsPosition g_gps_pos;

// Acquire / release the mutex that protects g_gps_pos.
void gps_lock_pos(void);
void gps_unlock_pos(void);

// Initialise UART2 and start gps_task.
// cfg may be NULL; reads gps.enabled and gps.baud from config.json if present.
void gps_init(cJSON *cfg);

// Reaplica en caliente gps.beacon_period_s, symbol_table, symbol_code, comment,
// course_speed, altitude, use_fixed/fixed_lat/fixed_lon y smart_beacon.{...}
// desde cfg (el resto de la sección gps requiere reinicio).
void gps_reload_config(cJSON *cfg);

// Posición APRS sin comprimir compartida (gps.c, api_rest.c, remote_cmd.c).
// Escribe "=DDMM.mmN/DDDMM.mmE" + symbol + '\0' en out (tabla de símbolos '/').
// out_len debe ser >= APRS_POS_BUF_LEN. Minutos en centésimas enteras, con
// acarreo a grados: 59,996' -> grado+1 y 00.00'. Lat/lon se acotan a 90/180.
#define APRS_POS_BUF_LEN 21
void aprs_format_position(char *out, size_t out_len, double lat, double lon, char symbol);

// Info field de baliza con la posición actual (fix GPS válido o, si no, la posición
// fija gps.fixed_lat/lon cuando gps.use_fixed): "=pos" + [cse/spd] + [/A=nnnnnn] +
// comentario, con símbolo/comentario/altitud/course_speed de la config. Es el mismo
// formato que usa la baliza periódica. Devuelve false si no hay posición.
// out_len recomendado >= 96.
bool gps_build_beacon_info(char *out, size_t out_len);

// Encola una baliza ahora (posición GPS o fija). false si no hay posición.
bool gps_send_beacon_now(void);

// Igual que la anterior pero con posición explícita (baliza manual de la API).
// sym_code = 0 → el de la config (la tabla de símbolos es siempre la de la config);
// comment = NULL → el de la config. has_cse añade cse/spd (grados / nudos) dados;
// la altitud /A= se añade si gps.altitude y hay altitud GPS válida.
void gps_build_beacon_info_at(char *out, size_t out_len, double lat, double lon,
                              bool has_cse, int cse, int spd,
                              char sym_code, const char *comment);

#pragma once
#include <stdbool.h>
#include <stdint.h>
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

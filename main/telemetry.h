#pragma once
// Telemetría APRS periódica (T#nnn,...) + definiciones PARM/UNIT/EQNS/BITS.
// Config "telemetry": { "enabled": false, "period_s": 900 }
#include "cJSON.h"

// Crea la tarea (stack 3 KB) sólo si telemetry.enabled. Sin enabled no reserva nada.
void telemetry_init(cJSON *cfg);
// Relee period_s/enabled; activar en caliente crea la tarea, desactivar la detiene.
void telemetry_reload_config(cJSON *cfg);
// Contadores: llamar por cada trama RX válida (on_ax25_raw_frame) y por cada TX.
void telemetry_note_rx(void);
void telemetry_note_tx(void);

#pragma once
// Modo SURVEY: medición temporal del audio de RX para calibrar los indicadores
// de nivel y de canal ocupado. Solo consume CPU/heap mientras está iniciado.
//
// Cada ventana de ~107 ms (8 bloques de 128 muestras @ 9600 Hz) produce una
// fila con: pico, RMS, continua, saturación, potencia en 1200/2200 Hz, banda
// media y banda alta (HFNE), AGC del demodulador v2 y contadores de tramas.
// Las filas se emiten por WebSocket ({"type":"survey",...}) y se acumulan en
// histogramas consultables con rx_survey_status_json().
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Arranque único (no reserva nada hasta rx_survey_start()).
void rx_survey_init(void);

// Llamar por cada muestra decimada desde receive_audio_task (único productor).
// Retorna al instante si el survey no está activo.
void rx_survey_push_sample(int8_t sample);

// Anotar una trama AX.25 recibida (longitud). Barato; no bloquea.
void rx_survey_note_frame(size_t len);

bool rx_survey_start(void);   // false si ya activo o sin memoria
void rx_survey_stop(void);
bool rx_survey_is_active(void);

// Marca manual: 0 = sin marca, 1 = canal libre, 2 = canal ocupado.
void rx_survey_set_mark(int mark);
int  rx_survey_get_mark(void);

// JSON de estado + histogramas. Devuelve memoria de malloc(); liberar con free().
char *rx_survey_status_json(void);

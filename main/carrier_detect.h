#pragma once
// Detector de portadora/trama ajeno para Listen-Before-Talk. Siempre activo,
// barato (2 bins Goertzel + RMS por muestra) y sin etiquetas manuales.
//
// Principio (medido con el modo SURVEY, radio con squelch desactivado): sin
// portadora el discriminador FM entrega ruido fuerte (RMS alto, energía en
// 3,5–4 kHz); con portadora ese ruido cae. Se declara OCUPADO cuando, durante
// ventanas de ~107 ms, la energía de alta frecuencia cae >= CD_ENTER_HF_DB por
// debajo de su suelo adaptativo Y el RMS cae por debajo de CD_ENTER_RMS_FRAC
// de su suelo. El suelo solo se actualiza en canal libre (EMA lenta), así que
// si la radio tuviese squelch (sin ruido) el detector no dispara (falla en
// abierto: el canal se considera libre).
#include <stdbool.h>
#include <stdint.h>

void carrier_detect_init(void);

// Por cada muestra decimada (9600 Hz) desde receive_audio_task (único productor).
void carrier_detect_push_sample(int8_t sample);

// true mientras se detecta una portadora ajena. Seguro desde cualquier tarea.
bool carrier_detect_busy(void);

// Pide descartar lo acumulado y las siguientes ventanas (llamar al entrar/salir de TX).
void carrier_detect_hold(void);

// CRC erróneos desde que el canal pasó a ocupado (0 si lleva libre >1,5 s).
int carrier_detect_errors_since_rise(void);

// Estado para diagnóstico (JSON en buffer del llamador).
int carrier_detect_status_json(char *buf, int len);

// Calidad de recepción de la trama que acaba de llegar: caída de ruido HF (dB)
// respecto al canal libre, sobre las ventanas que cubren `duration_ms`.
// false si el detector aún no tiene suelo válido (calentamiento o sin ruido).
bool carrier_detect_frame_quality(int duration_ms, float *quiet_db);

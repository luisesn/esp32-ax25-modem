#pragma once
#include <stdbool.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialise morse beacon from config.json root object.
// Parses morse.{enabled,callsign,tone_hz,wpm,period_s}.
// Falls back to aprs.callsign when morse.callsign is empty.
// Safe to call again on config reload — replaces internal state without
// duplicating the background task.
void morse_init(const cJSON *cfg);

// Called from receive_audio_task at the top of each loop iteration.
// Non-blocking: if a beacon is pending (semaphore given by the timer task or
// morse_trigger_now()), performs the full morse TX synchronously and returns
// true.  Returns false immediately when no beacon is pending.
// MUST run from receive_audio_task (I2S0 ADC/DAC mutex ownership).
bool morse_check_and_dispatch(void);

// Trigger an immediate one-shot beacon (e.g. from a REST endpoint).
// Non-blocking: gives the ready semaphore; TX happens on the next
// receive_audio_task iteration.
void morse_trigger_now(void);

// Envía `text` en CW (A-Z, 0-9, '/') con el DAC en TX y el PTT YA activos; no
// los toca. Bloqueante. Sólo desde receive_audio_task, con el buffer compartido
// de TX vacío (tras flush). Devuelve false si morse no está inicializado.
bool morse_send_text_in_tx(const char *text);

#ifdef __cplusplus
}
#endif

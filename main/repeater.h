#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    REPEATER_STATE_IDLE,
    REPEATER_STATE_RECORDING,
    REPEATER_STATE_TAIL,
    REPEATER_STATE_PENDING,
    REPEATER_STATE_TX,
} repeater_state_t;

// Initialise repeater from config.json root object.
// Parses repeater.{enabled,lockout_ms,max_record_s,tail_delay_ms,
//   courtesy_tone_hz,courtesy_tone_ms,cw_id}.
// Squelch is handled by the HFNE module (squelch_sf.h).
void repeater_init(cJSON *config);

// Audio hook: call with each decimated int8_t sample at 9600 Hz.
// Drives IDLE→RECORDING→TAIL→PENDING transitions using HFNE squelch.
// Must be called from receive_audio_task only (no mutex needed — single writer).
void repeater_audio_hook(int8_t sample);

// Dispatch hook: call from project_dispatch_hook (receive_audio_task context).
// When state == PENDING, performs full TX synchronously: playback + courtesy
// tone + optional CW ID, then returns to IDLE.
void repeater_dispatch_if_pending(void);

// Runtime control.
void repeater_set_enabled(bool en);

repeater_state_t repeater_get_state(void);
bool             repeater_is_enabled(void);

#ifdef __cplusplus
}
#endif

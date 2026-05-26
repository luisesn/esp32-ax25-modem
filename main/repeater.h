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
// Parses repeater.{enabled,squelch_threshold,max_record_s,tail_delay_ms,
//   courtesy_tone_hz,courtesy_tone_ms,cw_id}.
// Allocates recording buffer only when enabled.
void repeater_init(cJSON *config);

// Audio hook: call with each decimated int8_t sample at 9600 Hz.
// Runs IIR squelch and drives IDLE→RECORDING→TAIL→PENDING transitions.
// Must be called from receive_audio_task only (no mutex needed — single writer).
void repeater_audio_hook(int8_t sample);

// Dispatch hook: call from project_dispatch_hook (receive_audio_task context).
// When state == PENDING, performs full TX synchronously: playback + courtesy
// tone + optional CW ID, then returns to IDLE.
void repeater_dispatch_if_pending(void);

// Runtime control: enable/disable, adjust squelch threshold without reflash.
void repeater_set_enabled(bool en);
void repeater_set_threshold(uint32_t thr);

// Query current state, enabled flag, and envelope level (for REST API / display).
repeater_state_t repeater_get_state(void);
bool             repeater_is_enabled(void);
uint32_t         repeater_get_level(void);
uint32_t         repeater_get_threshold(void);

#ifdef __cplusplus
}
#endif

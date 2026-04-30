#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"

// Initialise the digipeater from the parsed config.json root object.
// Must be called after config_load(). Safe to call again after a config reload
// (digi_init is idempotent — it replaces its internal state).
void digi_init(const cJSON *cfg);

// Process a received raw AX.25 UI frame for digipeating.
// If the frame should be digipeated, modifies a working copy and enqueues it
// for TX via afsk_queue_tx_frame(). Returns true if the frame was digipeated.
// buf/len: the raw frame as received (without CRC).
bool digi_process_frame(const uint8_t *buf, size_t len);

// Runtime enable/disable without a full config reload.
void digi_set_enabled(bool en);
bool digi_is_enabled(void);

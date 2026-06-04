#pragma once
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Auto-tuner for aprs.post_rx_tx_delay_ms.
// Sweeps from TUNE_MIN_MS to TUNE_MAX_MS in steps of TUNE_STEP_MS,
// sending TUNE_PINGS_PER_STEP ICMP pings to ip.remote_addr per step.
// Results are streamed to WebSocket clients as tune_start / tune_step /
// tune_waiting / tune_done / tune_aborted JSON events.
// REST: POST /api/tune/start   POST /api/tune/stop
// Must be called after audio_stream_init().
void delay_tune_init(const cJSON *cfg);

#ifdef __cplusplus
}
#endif

#pragma once
#include <stdbool.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialise remote command handler from config.json root object.
// Reads remote_cmd.enabled.
void remote_cmd_init(const cJSON *cfg);

// Called when an APRS message addressed to our callsign arrives.
// src_call: source callsign (e.g. "EA1JBS"), src_ssid: 0-15.
// text: message body with {NNN} already stripped, null-terminated.
// Safe to call from receive_audio_task (all dispatches are non-blocking).
void remote_cmd_handle(const char *src_call, int src_ssid, const char *text);

#ifdef __cplusplus
}
#endif

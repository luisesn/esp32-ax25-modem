#pragma once
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start a line-based TCP console bound exclusively to the ax25ip RF interface.
// Must be called after ax25ip_init(). No-op if "console.enabled" is false or
// the ax25ip gateway was not initialised.
void rf_console_init(const cJSON *cfg);

#ifdef __cplusplus
}
#endif

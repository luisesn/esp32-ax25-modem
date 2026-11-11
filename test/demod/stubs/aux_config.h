#pragma once
// Stub: el harness no lee config.json; la config siempre es NULL.
#include "cJSON.h"
#ifdef __cplusplus
extern "C" {
#endif
static inline cJSON *config_get(void) { return NULL; }
static inline void config_free_json(cJSON *c) { (void)c; }
#ifdef __cplusplus
}
#endif

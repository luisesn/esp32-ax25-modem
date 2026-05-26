#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

void  squelch_sf_init(cJSON *config);
void  squelch_sf_push_sample(int8_t sample);

// Activation sources — processing only runs when at least one is true.
// Repeater source: controlled by repeater_set_enabled().
// Manual source: controlled by the UI monitor switch / REST endpoint.
void squelch_sf_set_repeater_active(bool en);
void squelch_sf_set_monitor_active(bool en);
bool squelch_sf_is_active(void);
bool squelch_sf_is_monitor_active(void);

void  squelch_hfne_set_threshold(float thr);
float squelch_hfne_get_value(void);
bool  squelch_hfne_is_open(void);
float squelch_hfne_get_threshold(void);

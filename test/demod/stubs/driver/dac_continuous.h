#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../freertos/FreeRTOS.h"
typedef void *dac_continuous_handle_t;
typedef struct { int dummy; } dac_event_data_t;
typedef bool (*dac_isr_cb_t)(dac_continuous_handle_t, const dac_event_data_t *, void *);
typedef struct { dac_isr_cb_t on_convert_done; dac_isr_cb_t on_stop; } dac_event_callbacks_t;
#define DAC_CHANNEL_MASK_CH0 1
#define DAC_DIGI_CLK_SRC_DEFAULT 0
#define DAC_CHANNEL_MODE_SIMUL 0
typedef struct { int chan_mask; uint32_t desc_num; size_t buf_size; uint32_t freq_hz; int offset; int clk_src; int chan_mode; } dac_continuous_config_t;
static inline int dac_continuous_new_channels(const dac_continuous_config_t *c, dac_continuous_handle_t *h) { return 0; }
static inline int dac_continuous_enable(dac_continuous_handle_t h) { return 0; }
static inline int dac_continuous_disable(dac_continuous_handle_t h) { return 0; }
static inline int dac_continuous_del_channels(dac_continuous_handle_t h) { return 0; }
static inline int dac_continuous_register_event_callback(dac_continuous_handle_t h, const dac_event_callbacks_t *c, void *u) { return 0; }
static inline int dac_continuous_write(dac_continuous_handle_t h, uint8_t *b, size_t l, size_t *w, int t) { return 0; }

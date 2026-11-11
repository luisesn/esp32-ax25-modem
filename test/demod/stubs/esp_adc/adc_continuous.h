#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../freertos/FreeRTOS.h"
typedef void *adc_continuous_handle_t;
#define SOC_ADC_DIGI_RESULT_BYTES 4
#define ADC_UNIT_1 0
#define ADC_CHANNEL_7 7
#define ADC_ATTEN_DB_12 3
#define ADC_BITWIDTH_12 12
#define ADC_CONV_SINGLE_UNIT_1 1
#define ADC_DIGI_OUTPUT_FORMAT_TYPE1 0
typedef struct { uint32_t max_store_buf_size; uint32_t conv_frame_size; struct { uint32_t flush_pool; } flags; } adc_continuous_handle_cfg_t;
typedef struct { uint8_t atten; uint8_t channel; uint8_t unit; uint8_t bit_width; } adc_digi_pattern_config_t;
typedef struct { uint32_t pattern_num; adc_digi_pattern_config_t *adc_pattern; uint32_t sample_freq_hz; int conv_mode; int format; } adc_continuous_config_t;
typedef struct { struct { uint32_t data:12; uint32_t reserved12:1; uint32_t channel:4; uint32_t unit:1; uint32_t reserved17_31:14; } type1; } adc_digi_output_data_t;
static inline int adc_continuous_new_handle(const adc_continuous_handle_cfg_t *c, adc_continuous_handle_t *h) { return 0; }
static inline int adc_continuous_config(adc_continuous_handle_t h, const adc_continuous_config_t *c) { return 0; }
static inline int adc_continuous_start(adc_continuous_handle_t h) { return 0; }
static inline int adc_continuous_stop(adc_continuous_handle_t h) { return 0; }
static inline int adc_continuous_deinit(adc_continuous_handle_t h) { return 0; }
static inline int adc_continuous_read(adc_continuous_handle_t h, uint8_t *b, uint32_t l, uint32_t *r, uint32_t t) { *r = 0; return 1; }

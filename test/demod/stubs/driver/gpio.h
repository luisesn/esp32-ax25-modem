#pragma once
#include <stdint.h>
#include "../freertos/FreeRTOS.h"
typedef int gpio_num_t;
#define GPIO_NUM_26 26
#define GPIO_NUM_37 37
#define GPIO_MODE_OUTPUT 2
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
typedef struct { uint64_t pin_bit_mask; int mode; int pull_up_en; int pull_down_en; int intr_type; } gpio_config_t;
static inline int gpio_config(const gpio_config_t *c) { return 0; }
static inline int gpio_set_level(int n, int l) { return 0; }
static inline int gpio_set_direction(int n, int m) { return 0; }

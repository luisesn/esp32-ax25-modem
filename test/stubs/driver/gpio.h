// Stub host de driver/gpio.h (ESP-IDF): sólo los tipos y funciones que usan los headers de la librería.
#pragma once
#include <stdint.h>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    GPIO_NUM_0 = 0, GPIO_NUM_4 = 4, GPIO_NUM_13 = 13, GPIO_NUM_16 = 16, GPIO_NUM_19 = 19,
    GPIO_NUM_23 = 23, GPIO_NUM_25 = 25, GPIO_NUM_26 = 26, GPIO_NUM_33 = 33, GPIO_NUM_35 = 35,
    GPIO_NUM_37 = 37, GPIO_NUM_NC = -1
} gpio_num_t;
esp_err_t gpio_set_level(gpio_num_t gpio, uint32_t level);
#ifdef __cplusplus
}
#endif

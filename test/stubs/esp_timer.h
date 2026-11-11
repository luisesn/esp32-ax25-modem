#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Reloj falso controlable desde los tests.
int64_t esp_timer_get_time(void);
void    test_time_set_us(int64_t us);
void    test_time_advance_us(int64_t us);
#ifdef __cplusplus
}
#endif

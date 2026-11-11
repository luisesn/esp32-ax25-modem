// Implementaciones de los stubs comunes a todos los tests.
#include "esp_timer.h"
static int64_t s_now_us = 0;
int64_t esp_timer_get_time(void) { return s_now_us; }
void test_time_set_us(int64_t us) { s_now_us = us; }
void test_time_advance_us(int64_t us) { s_now_us += us; }

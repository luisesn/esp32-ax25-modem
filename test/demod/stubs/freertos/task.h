#pragma once
#include "FreeRTOS.h"
#ifdef __cplusplus
extern "C" {
#endif
static inline TickType_t xTaskGetTickCount(void) { return 0; }
static inline void vTaskDelay(TickType_t t) { (void)t; }
static inline TaskHandle_t xTaskGetCurrentTaskHandle(void) { return 0; }
static inline int xTaskCreatePinnedToCore(void (*f)(void*), const char *n, uint32_t s, void *a, int p, TaskHandle_t *h, int c) { return 1; }
static inline void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *w) {}
static inline uint32_t ulTaskNotifyTake(BaseType_t c, TickType_t t) { return 0; }
#ifdef __cplusplus
}
#endif

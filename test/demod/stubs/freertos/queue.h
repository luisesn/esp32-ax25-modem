#pragma once
#include "FreeRTOS.h"
#ifdef __cplusplus
extern "C" {
#endif
static inline QueueHandle_t xQueueCreate(int n, int s) { return 0; }
static inline int uxQueueMessagesWaiting(QueueHandle_t q) { return 0; }
static inline int xQueueReceive(QueueHandle_t q, void *b, TickType_t t) { return 0; }
static inline int xQueueSendToBack(QueueHandle_t q, const void *b, TickType_t t) { return 0; }
#ifdef __cplusplus
}
#endif

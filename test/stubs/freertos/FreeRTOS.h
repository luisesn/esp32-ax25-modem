#pragma once
#include <stdint.h>
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m)  ((void)(m))
#define portENTER_CRITICAL_ISR(m) ((void)(m))
#define portEXIT_CRITICAL_ISR(m)  ((void)(m))
#define pdMS_TO_TICKS(x) (x)
#define pdPASS 1
#define pdTRUE 1
#define pdFALSE 0
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define taskENTER_CRITICAL(m) portENTER_CRITICAL(m)
#define taskEXIT_CRITICAL(m)  portEXIT_CRITICAL(m)
#define taskENTER_CRITICAL_ISR(m) portENTER_CRITICAL_ISR(m)
#define taskEXIT_CRITICAL_ISR(m)  portEXIT_CRITICAL_ISR(m)

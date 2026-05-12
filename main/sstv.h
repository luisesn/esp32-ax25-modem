#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SSTV_FMT_JPEG,
} SstvFormat;

typedef enum {
    SSTV_MODE_MARTIN_M1,
    SSTV_MODE_MARTIN_M2,
    SSTV_MODE_SCOTTIE_S1,
    SSTV_MODE_ROBOT_36,
    SSTV_MODE_ROBOT_72,
} SstvMode;

typedef struct {
    char       name[48];   // filename in /spiffs/sstv/, e.g. "foto.jpg"
    SstvFormat fmt;
    SstvMode   mode;
} SstvRequest;

extern QueueHandle_t s_sstv_queue;

void sstv_init(void);
void sstv_transmit(const SstvRequest *req);
void sstv_dispatch_if_pending(void);  // must be called from receive_audio_task

#ifdef __cplusplus
}
#endif

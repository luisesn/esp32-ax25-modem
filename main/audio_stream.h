#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Puerto del servidor HTTP (index.html + WebSocket /ws).
#define AUDIO_HTTP_PORT   80
// Puerto del servidor WAV TCP (ffplay/VLC → http://ip:8080/audio).
#define AUDIO_WAV_PORT    8080

// Parámetros IMA ADPCM
#define ADPCM_SAMPLE_RATE    9600
#define ADPCM_BLOCK_BYTES    512
// samplesPerBlock = (512 - 4) * 2 + 1 = 1017
#define ADPCM_SAMPLES_BLOCK  1017

// Cola de muestras de audio: receive_audio_task enqueue, audio_stream_task dequeue.
// Dimensionada para ~2 bloques de margen (2034 muestras ≈ 212 ms).
#define AUDIO_QUEUE_LEN      2048
extern QueueHandle_t audio_stream_q;

// Inicia el servidor HTTP (port AUDIO_HTTP_PORT) y la tarea de streaming.
// Debe llamarse después de que WiFi esté conectado.
void audio_stream_init(void);

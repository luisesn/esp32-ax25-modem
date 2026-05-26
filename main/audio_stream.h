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

// Envía un frame WebSocket TEXT a todos los clientes WS conectados.
// Seguro de llamar desde cualquier tarea FreeRTOS.
void audio_stream_ws_send_text(const char *text);

// Returns the httpd handle (valid after audio_stream_init()).
// Used by sstv_init() to register its URI handlers.
#include "esp_http_server.h"
httpd_handle_t audio_stream_get_httpd(void);

// IMA ADPCM codec state — shared with repeater.c for buffer compression.
typedef struct { int32_t predictor; int8_t step_index; } adpcm_state_t;

// Encodes 1017 int8_t samples into a 512-byte ADPCM block.
// step_index is inherited across calls (continuous stream); predictor is reset to samples[0].
void encode_block(adpcm_state_t *st, const int8_t *samples, uint8_t *block);

// Decodes a 512-byte ADPCM block into 1017 int8_t samples.
// State is fully restored from the block header — no inter-block context needed.
void decode_block(const uint8_t *block, int8_t *samples);

// Stops the WAV TCP server task and closes its listening socket.
// Called by repeater_set_enabled(true) to free heap before the recording buffer malloc.
void audio_stream_stop_wav_server(void);

#pragma once
#include <stdint.h>
#include <stddef.h>

#define KISS_FEND  0xC0
#define KISS_FESC  0xDB
#define KISS_TFEND 0xDC
#define KISS_TFESC 0xDD

// Llamada cuando el decodificador tiene una trama AX.25 completa del host.
typedef void (*kiss_frame_cb)(const uint8_t *buf, size_t len);

void kiss_init(kiss_frame_cb on_frame);

// Decodificador reentrante: un contexto por cliente KISS (F5). El callback
// on_frame es único y global (kiss_init); se invoca desde la tarea que llama
// a kiss_ctx_rx_byte, así que debe ser seguro entre tareas.
typedef struct kiss_ctx kiss_ctx_t;
kiss_ctx_t *kiss_ctx_new(void);                       // NULL si no hay memoria (~0,7 KB)
void        kiss_ctx_free(kiss_ctx_t *c);
void        kiss_ctx_reset(kiss_ctx_t *c);
void        kiss_ctx_rx_byte(kiss_ctx_t *c, uint8_t b);

// Versiones de contexto único (compatibilidad). kiss_rx_byte/kiss_reset operan
// sobre un contexto por defecto creado en kiss_init.
// Reset the decoder state machine without touching the registered callback.
// Call when a client disconnects so the next client starts from a clean state.
void kiss_reset(void);

// Alimentar bytes del host; llama on_frame() cuando la trama está completa.
void kiss_rx_byte(uint8_t b);

// Codificar trama AX.25 en KISS y enviarla vía transport_write().
void kiss_send_frame(const uint8_t *frame, size_t len);

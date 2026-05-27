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

// Reset the decoder state machine without touching the registered callback.
// Call when a client disconnects so the next client starts from a clean state.
void kiss_reset(void);

// Alimentar bytes del host; llama on_frame() cuando la trama está completa.
void kiss_rx_byte(uint8_t b);

// Codificar trama AX.25 en KISS y enviarla vía transport_write().
void kiss_send_frame(const uint8_t *frame, size_t len);

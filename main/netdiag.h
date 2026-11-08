#pragma once
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Diagnóstico de red básico: ping ICMP y miniconsola TCP/UDP.
// Una sola operación a la vez; el estado y la salida viven en el firmware
// (búfer circular) para que sobrevivan a recargas y a varios clientes web.
// REST: GET /api/netdiag?gen=G&since=N   POST /api/netdiag/{ping,connect,send,stop}
// Debe llamarse después de audio_stream_init().
void netdiag_init(void);

// Registra una trama AX.25 a punto de transmitirse por RF (panel de debug).
// Llamar desde la función TX registrada con afsk_set_tx_fn().
void netdiag_note_tx(const uint8_t *ax25_raw, size_t len);
// Ídem para una trama recibida (llamar desde on_ax25_raw_frame).
bool netdiag_describe_frame(const uint8_t *d, size_t len, char *out, size_t on);
void netdiag_note_rx(const uint8_t *ax25_raw, size_t len);

#ifdef __cplusplus
}
#endif

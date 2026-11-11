#pragma once
// Captura de audio crudo del RX (9600 Hz, ya decimado) como WAV por HTTP.
// GET /api/rx/capture?secs=N&rate=9600  (N 1-30). Sólo se reserva memoria
// durante la captura; sin captura rx_capture_push() es un no-op.
#include <stddef.h>
#include <stdint.h>
#include "esp_http_server.h"

// Llamar desde audio_sample_hook (único productor: receive_audio_task).
// Una muestra por llamada es válida (n=1). Nunca bloquea.
void rx_capture_push(const int8_t *s, size_t n);

// Registra el URI en el servidor HTTP.
void rx_capture_register(httpd_handle_t srv);

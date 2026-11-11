#pragma once
// F1: reintento de mensajes APRS con seguimiento de ACK.
// Tabla pequeña de mensajes pendientes; una tarea propia reencola el mensaje
// (mismo {NNN}) por la cola TX normal con espera creciente. Al recibir
// "ackNNN" del destino se cancela; al agotar intentos se emite por WebSocket
// {"type":"msg_failed",...}. Cada reintento emite {"type":"msg_retry",...}.
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MSG_RETRY_MAX_ENTRIES 8
#define MSG_RETRY_MAX_ATTEMPTS 4              // incluye el envío inicial
#define MSG_RETRY_WAITS_S { 30, 60, 120, 120 } // espera tras el intento 1..4

void msg_retry_init(void);
// Registra un mensaje ya encolado por primera vez (intento 1). Devuelve false
// si la tabla está llena (el mensaje se envió igualmente, sin seguimiento).
bool msg_retry_track(const char *to_call, int to_ssid, int id, const char *text);
// Llamar al recibir un ACK: id_str es el texto tras "ack" (sólo cuenta si es numérico).
void msg_retry_on_ack(const char *from_call, int from_ssid, const char *id_str);
void msg_retry_on_rej(const char *from_call, int from_ssid, const char *id_str);

#ifdef __cplusplus
}
#endif

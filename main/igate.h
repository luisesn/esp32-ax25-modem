#pragma once
// iGate APRS-IS (RX RF -> IS, y opcionalmente IS -> RF sólo mensajes a estaciones oídas).
// Con igate.enabled=false no se crea tarea ni se reserva memoria.
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// cfg: raíz de config.json (secciones "igate" y "aprs"). Crea la tarea sólo si igate.enabled.
void igate_init(cJSON *cfg);

// Llamar desde on_ax25_raw_frame con la trama AX.25 cruda (sin CRC).
// Rápida y sin bloqueo: copia a una cola pequeña. No hace nada si el iGate está desactivado
// o sin conexión con APRS-IS.
void igate_rx_frame(const uint8_t *ax25, size_t len);

// true si hay sesión APRS-IS iniciada (logresp recibido).
bool igate_connected(void);

// JSON de estado: {"enabled":..,"connected":..,"verified":..,"server":"..","rx_gated":n,
// "rx_dropped":n,"is_rx":n,"rf_gated":n,"reconnects":n}
void igate_status_json(char *buf, size_t n);

#ifdef __cplusplus
}
#endif

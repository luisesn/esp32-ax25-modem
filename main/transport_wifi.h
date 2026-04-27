#pragma once
#include "transport.h"

// Transporte WiFi TCP: conecta a la red como estación (STA),
// abre un servidor TCP en KISS_TCP_PORT y acepta un cliente a la vez.
// El host se conecta con:  tncattach --tcp <ip_esp32> <KISS_TCP_PORT>
extern const transport_ops_t transport_wifi_ops;

#pragma once
#include "transport.h"
#include <stdint.h>

// Transporte WiFi TCP: conecta a la red como estación (STA),
// abre un servidor TCP en KISS_TCP_PORT y acepta un cliente a la vez.
// El host se conecta con:  tncattach --tcp <ip_esp32> <KISS_TCP_PORT>
extern const transport_ops_t transport_wifi_ops;

// ---------------------------------------------------------------------------
// Shared WiFi connection status — updated by transport_wifi.c,
// read (without mutex) by display_task for status display.
// ---------------------------------------------------------------------------

typedef enum {
    WIFI_STATUS_IDLE       = 0, // before init
    WIFI_STATUS_CONNECTING = 1, // trying a STA network
    WIFI_STATUS_CONNECTED  = 2, // STA connected, IP obtained
    WIFI_STATUS_HOTSPOT    = 3, // fell back to AP mode
} wifi_status_t;

typedef struct {
    wifi_status_t state;
    char  ssid[33];           // current SSID (network being tried, or AP SSID)
    int   net_index;          // 0-based index of the network being tried
    int   net_count;          // total configured networks
    int   timeout_s;          // per-network timeout
    int64_t attempt_start_us; // esp_timer_get_time() when this attempt started
} WifiConnStatus;

extern WifiConnStatus g_wifi_status;

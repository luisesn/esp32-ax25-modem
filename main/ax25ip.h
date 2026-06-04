#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "cJSON.h"
#include "lwip/ip4_addr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// AX.25 IP Gateway (RFC 1226)
// ---------------------------------------------------------------------------
// Adds a lwIP netif with the IP address from config "ip" section.
// Outbound: IP packets routed to that netif are wrapped in AX.25 UI frames
//           (PID=0xCC) and queued via afsk_queue_tx_frame().
// Inbound:  Call ax25ip_rx_frame() from on_ax25_raw_frame(); frames with
//           PID=0xCC are decapsulated and injected into the lwIP IP stack.
//
// Requires CONFIG_LWIP_IP_FORWARD=y in sdkconfig.defaults so that packets
// arriving on the WiFi interface and addressed to the AX.25 subnet are
// forwarded rather than dropped.
//
// Manual route on the host PC:
//   ip route add <ip.addr>/<prefix> via <esp32-wifi-ip>
// ---------------------------------------------------------------------------

// Initialize the gateway from config JSON.
// Must be called after WiFi/lwIP are up (after transport_init).
// Returns true if "ip.enabled" is set and initialisation succeeded.
bool ax25ip_init(cJSON *cfg);

// Pass every received AX.25 frame here.
// Frames with PID=0xCC (IP) are injected into the lwIP stack.
// All other PIDs are silently ignored.
void ax25ip_rx_frame(const uint8_t *buf, size_t len);

// Returns the IPv4 address assigned to the RF netif.
// Returns false if ax25ip_init() was not called or failed.
bool ax25ip_get_addr(ip4_addr_t *addr);

#ifdef __cplusplus
}
#endif

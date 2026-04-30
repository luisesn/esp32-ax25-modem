#include "ax25ip.h"

#include <string.h>
#include <esp_log.h>

// lwIP
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"

// AFSK TX queue
#include "LibAPRS-esp32-i2s/src/AFSK.h"

#define TAG "ax25ip"

// RFC 1226 / AX.25 UI constants
#define AX25_CTRL_UI   0x03u
#define AX25_PID_IP    0xCCu

// Max IP payload per AX.25 frame.
// AX25_MAX_FRAME_LEN=330; header (2×7 addr + CTRL + PID) = 16 bytes.
// We use 300 bytes for some extra headroom.
#define AX25IP_MTU  300

// ── Module state ─────────────────────────────────────────────────────────────

static struct netif s_netif;
static bool         s_enabled = false;

// Our callsign (6-char, space-padded NUL-terminated) and AX.25 SSID for IP.
static char    s_call[7];
static uint8_t s_ssid;

// ── AX.25 address encoding ────────────────────────────────────────────────────

static void enc_addr(uint8_t *buf, const char *call, uint8_t ssid, bool last)
{
    for (int i = 0; i < 6; i++)
        buf[i] = (uint8_t)((uint8_t)(call[i] ? call[i] : ' ') << 1);
    buf[6] = (uint8_t)(0x60u | ((ssid & 0x0Fu) << 1) | (last ? 1u : 0u));
}

// ── lwIP netif output (WiFi/IP → AX.25 RF) ───────────────────────────────────
// Called by lwIP when an IP packet must be sent on this interface.
// p is a pbuf chain containing the full IP datagram.
// ipaddr is the destination IP (unused; we broadcast to QST-0).
static err_t ax25ip_output(struct netif *netif, struct pbuf *p,
                            const ip4_addr_t *ipaddr)
{
    (void)netif;
    (void)ipaddr;

    uint16_t ip_len = p->tot_len;
    if (ip_len > AX25IP_MTU) {
        ESP_LOGW(TAG, "drop: IP frame too large (%u > %d)", ip_len, AX25IP_MTU);
        return ERR_MEM;
    }

    // AX.25 UI frame layout:
    // [DST:7][SRC:7][CTRL:1=0x03][PID:1=0xCC][IP payload...]
    uint8_t frame[16 + AX25IP_MTU];

    enc_addr(frame + 0, "QST   ", 0, false);          // broadcast destination
    enc_addr(frame + 7, s_call, s_ssid, true);         // our address (last=1)
    frame[14] = AX25_CTRL_UI;
    frame[15] = AX25_PID_IP;

    // Flatten pbuf chain into frame
    uint16_t offset = 16;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        memcpy(frame + offset, q->payload, q->len);
        offset += q->len;
    }

    ESP_LOGD(TAG, "TX %u bytes IP→AX.25", ip_len);
    afsk_queue_tx_frame(frame, (size_t)offset);
    return ERR_OK;
}

// ── lwIP netif init callback ──────────────────────────────────────────────────
static err_t ax25ip_netif_init(struct netif *netif)
{
    netif->output     = ax25ip_output;
    netif->mtu        = AX25IP_MTU;
    // No link-layer ARP; broadcast-capable; already up.
    netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_UP | NETIF_FLAG_LINK_UP;
    netif->hwaddr_len = 0;
    return ERR_OK;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool ax25ip_init(cJSON *cfg)
{
    if (!cfg) return false;

    cJSON *ip_obj = cJSON_GetObjectItem(cfg, "ip");
    if (!ip_obj || !cJSON_IsTrue(cJSON_GetObjectItem(ip_obj, "enabled")))
        return false;

    cJSON *j_addr = cJSON_GetObjectItem(ip_obj, "addr");
    cJSON *j_mask = cJSON_GetObjectItem(ip_obj, "netmask");
    cJSON *j_gw   = cJSON_GetObjectItem(ip_obj, "gateway");
    cJSON *j_ssid = cJSON_GetObjectItem(ip_obj, "ssid");

    if (!cJSON_IsString(j_addr) || !cJSON_IsString(j_mask)) {
        ESP_LOGE(TAG, "ip.addr / ip.netmask missing or invalid");
        return false;
    }

    ip4_addr_t addr, mask, gw;
    if (!ip4addr_aton(j_addr->valuestring, &addr) ||
        !ip4addr_aton(j_mask->valuestring, &mask)) {
        ESP_LOGE(TAG, "Cannot parse ip.addr or ip.netmask");
        return false;
    }
    ip4_addr_set_zero(&gw);
    if (cJSON_IsString(j_gw)) ip4addr_aton(j_gw->valuestring, &gw);

    // Callsign from aprs section; SSID from ip section.
    cJSON *aprs_obj = cJSON_GetObjectItem(cfg, "aprs");
    const char *callsign = "NOCALL";
    if (aprs_obj) {
        cJSON *cs = cJSON_GetObjectItem(aprs_obj, "callsign");
        if (cJSON_IsString(cs) && cs->valuestring[0]) callsign = cs->valuestring;
    }

    // Build padded 6-char callsign
    memset(s_call, ' ', 6); s_call[6] = '\0';
    int clen = (int)strlen(callsign); if (clen > 6) clen = 6;
    memcpy(s_call, callsign, (size_t)clen);
    s_ssid = (uint8_t)(cJSON_IsNumber(j_ssid) ? (int)j_ssid->valueint : 0);

    // Add netif under lwIP core lock (lwIP already running via WiFi stack).
    LOCK_TCPIP_CORE();
    struct netif *r = netif_add(&s_netif, &addr, &mask, &gw,
                                NULL, ax25ip_netif_init, tcpip_input);
    UNLOCK_TCPIP_CORE();

    if (!r) {
        ESP_LOGE(TAG, "netif_add failed");
        return false;
    }

    s_enabled = true;
    ESP_LOGI(TAG, "AX.25 IP gateway up: %s/%s  (call=%s-%d  MTU=%d)",
             j_addr->valuestring, j_mask->valuestring,
             callsign, (int)s_ssid, AX25IP_MTU);
    ESP_LOGI(TAG, "Add host route:  ip route add %s/%s via <ESP32-WiFi-IP>",
             j_addr->valuestring, j_mask->valuestring);
    return true;
}

void ax25ip_rx_frame(const uint8_t *buf, size_t len)
{
    if (!s_enabled || len < 18) return;  // 16-byte header + at least 2 bytes IP

    const uint8_t *p   = buf;
    const uint8_t *end = buf + len;

    // Skip DST address (7 bytes)
    bool is_last = (p[6] & 0x01u) != 0;
    p += 7;

    // Skip SRC address (7 bytes)
    is_last = (p[6] & 0x01u) != 0;
    p += 7;

    // Skip optional repeater addresses
    while (!is_last && (p + 7) <= end) {
        is_last = (p[6] & 0x01u) != 0;
        p += 7;
    }

    if (p + 2 > end) return;
    if (*p++ != AX25_CTRL_UI) return;    // not a UI frame
    if (*p++ != AX25_PID_IP)  return;    // not IP

    size_t ip_len = (size_t)(end - p);
    if (ip_len < 20) return;             // shorter than minimum IP header

    // Allocate pbuf and copy IP payload
    struct pbuf *q = pbuf_alloc(PBUF_RAW, (uint16_t)ip_len, PBUF_RAM);
    if (!q) {
        ESP_LOGW(TAG, "pbuf_alloc failed (%u bytes)", (unsigned)ip_len);
        return;
    }
    memcpy(q->payload, p, ip_len);

    ESP_LOGD(TAG, "RX %u bytes IP←AX.25", (unsigned)ip_len);

    // Inject into lwIP (tcpip_input is task-safe; sends msg to tcpip task).
    if (s_netif.input(q, &s_netif) != ERR_OK)
        pbuf_free(q);
}

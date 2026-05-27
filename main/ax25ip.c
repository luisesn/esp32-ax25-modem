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
// CUSTOM_FRAME_SIZE=600 (set in CMakeLists.txt for tncattach MTU 512).
// Subtract 16 bytes of AX.25 header (2×7 addr + CTRL + PID) = 584 bytes usable.
#define AX25IP_MTU  584

// ── Module state ─────────────────────────────────────────────────────────────

static struct netif s_netif;
static bool         s_enabled  = false;
// TUN mode: frames are raw TUN packets (4-byte PI header + IP), no AX.25 headers.
// Set by ip.mode="tun" in config.json (matches markqvist's tncattach default).
static bool         s_tun_mode = false;

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

    if (s_tun_mode) {
        // TUN mode: prepend Linux TUN PI header (flags=0x0000, proto=IPv4 0x0800).
        // tncattach reads this 4-byte header from the TUN fd before the IP packet.
        uint8_t frame[4 + AX25IP_MTU];
        frame[0] = 0x00; frame[1] = 0x00; frame[2] = 0x08; frame[3] = 0x00;
        uint16_t offset = 4;
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            memcpy(frame + offset, q->payload, q->len);
            offset += q->len;
        }
        ESP_LOGI(TAG, "TX %u bytes TUN→RF", ip_len);
        afsk_queue_tx_frame(frame, (size_t)offset);
        return ERR_OK;
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

    // Callsign and SSID from aprs section (same identity as the rest of the station).
    cJSON *aprs_obj = cJSON_GetObjectItem(cfg, "aprs");
    const char *callsign = "NOCALL";
    int aprs_ssid = 0;
    if (aprs_obj) {
        cJSON *cs = cJSON_GetObjectItem(aprs_obj, "callsign");
        if (cJSON_IsString(cs) && cs->valuestring[0]) callsign = cs->valuestring;
        cJSON *ss = cJSON_GetObjectItem(aprs_obj, "ssid");
        if (cJSON_IsNumber(ss)) aprs_ssid = (int)ss->valueint;
    }

    // Build padded 6-char callsign
    memset(s_call, ' ', 6); s_call[6] = '\0';
    int clen = (int)strlen(callsign); if (clen > 6) clen = 6;
    memcpy(s_call, callsign, (size_t)clen);
    s_ssid = (uint8_t)aprs_ssid;

    cJSON *j_mode = cJSON_GetObjectItem(ip_obj, "mode");
    s_tun_mode = cJSON_IsString(j_mode) && strcmp(j_mode->valuestring, "tun") == 0;

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
    ESP_LOGI(TAG, "AX.25 IP gateway up: %s/%s  (call=%s-%d  MTU=%d  mode=%s)",
             j_addr->valuestring, j_mask->valuestring,
             callsign, (int)s_ssid, AX25IP_MTU,
             s_tun_mode ? "tun" : "ax25");
    ESP_LOGI(TAG, "Add host route:  ip route add %s/%s via <ESP32-WiFi-IP>",
             j_addr->valuestring, j_mask->valuestring);
    return true;
}

void ax25ip_rx_frame(const uint8_t *buf, size_t len)
{
    if (!s_enabled) return;

    // TUN mode: tncattach sends Linux TUN PI header (4 bytes) + raw IPv4.
    // No AX.25 DST/SRC/CTRL/PID headers are present.
    if (s_tun_mode) {
        // Expect: 00 00 08 00 (PI: flags=0, proto=IPv4) + IPv4 header (≥20 bytes)
        if (len < 8 || buf[0] != 0x00 || buf[1] != 0x00 ||
            buf[2] != 0x08 || buf[3] != 0x00 || (buf[4] & 0xF0u) != 0x40u) {
            ESP_LOGW(TAG, "TUN: drop: unexpected frame (len=%u hdr=%02X%02X%02X%02X)",
                     (unsigned)len, buf[0], buf[1], buf[2], buf[3]);
            return;
        }
        size_t ip_len = len - 4;
        if (ip_len < 20) {
            ESP_LOGW(TAG, "TUN: drop: IP payload too short (%u bytes)", (unsigned)ip_len);
            return;
        }
        struct pbuf *q = pbuf_alloc(PBUF_RAW, (uint16_t)ip_len, PBUF_RAM);
        if (!q) {
            ESP_LOGW(TAG, "TUN: pbuf_alloc failed (%u bytes)", (unsigned)ip_len);
            return;
        }
        memcpy(q->payload, buf + 4, ip_len);
        ESP_LOGI(TAG, "RX %u bytes TUN-IP←RF → lwIP", (unsigned)ip_len);
        if (s_netif.input(q, &s_netif) != ERR_OK) {
            ESP_LOGW(TAG, "TUN: lwIP input rejected pbuf");
            pbuf_free(q);
        }
        return;
    }

    if (len < 18) {
        ESP_LOGD(TAG, "drop: frame too short (%u bytes)", (unsigned)len);
        return;
    }

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

    if (p + 2 > end) {
        ESP_LOGW(TAG, "drop: no room for ctrl+pid after addresses");
        return;
    }

    uint8_t ctrl = *p++;
    uint8_t pid  = *p++;

    if (ctrl != AX25_CTRL_UI) {
        ESP_LOGW(TAG, "drop: not UI frame (ctrl=0x%02X)", ctrl);
        return;
    }
    // Accept both RFC 1226 IP PID (0xCC) and AX.25 no-layer-3 PID (0xF0).
    // tncattach (markqvist) uses 0xF0 by default; kissattach and some APRS TNCs use 0xCC.
    if (pid != AX25_PID_IP && pid != 0xF0u) {
        ESP_LOGW(TAG, "skip: pid=0x%02X (not IP 0xCC or 0xF0)", pid);
        return;
    }

    size_t ip_len = (size_t)(end - p);
    if (ip_len < 20) {
        ESP_LOGW(TAG, "drop: IP payload too short (%u bytes)", (unsigned)ip_len);
        return;
    }

    // Allocate pbuf and copy IP payload
    struct pbuf *q = pbuf_alloc(PBUF_RAW, (uint16_t)ip_len, PBUF_RAM);
    if (!q) {
        ESP_LOGW(TAG, "pbuf_alloc failed (%u bytes)", (unsigned)ip_len);
        return;
    }
    memcpy(q->payload, p, ip_len);

    ESP_LOGI(TAG, "RX %u bytes IP←AX.25 → lwIP", (unsigned)ip_len);

    // Inject into lwIP (tcpip_input is task-safe; sends msg to tcpip task).
    if (s_netif.input(q, &s_netif) != ERR_OK) {
        ESP_LOGW(TAG, "lwIP input rejected pbuf");
        pbuf_free(q);
    }
}

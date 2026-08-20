#include "ax25ip.h"

#include <stdio.h>
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
#include "audio_stream.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define TAG "ax25ip"

// RFC 1226 / AX.25 UI constants
#define AX25_CTRL_UI   0x03u
#define AX25_PID_IP    0xCCu

// Hard ceiling for the IP payload of one AX.25 frame, used to size the TX
// buffers. CUSTOM_FRAME_SIZE=600 (set in CMakeLists.txt) minus 16 bytes of
// AX.25 header (2×7 addr + CTRL + PID) = 584 bytes usable.
#define AX25IP_MTU_MAX  584

// Working MTU, from ip.mtu in config.json. This MUST match the MTU tncattach
// runs with on the host: our netif advertises it to lwIP, so it sets the MSS of
// locally-originated TCP. Advertising more than the peer accepts makes large
// segments disappear at the far end with no ICMP to fall back on. Default 250
// also keeps one frame near ~1.8 s of air time at 1200 bps, which matters more
// than efficiency here — a lost frame costs a full retransmit.
#define AX25IP_MTU_DEFAULT 250
static uint16_t s_mtu = AX25IP_MTU_DEFAULT;

// ── Module state ─────────────────────────────────────────────────────────────

static struct netif s_netif;
static bool         s_enabled  = false;
// TUN mode: frames are raw TUN packets (4-byte PI header + IP), no AX.25 headers.
// Set by ip.mode="tun" in config.json (matches markqvist's tncattach default).
static bool         s_tun_mode = false;

// Our callsign (6-char, space-padded NUL-terminated) and AX.25 SSID for IP.
static char    s_call[7];
static uint8_t s_ssid;

// ── IP debug helper ───────────────────────────────────────────────────────────
// Returns a static string: "PROTO src.ip:sport→dst.ip:dport" (or without ports
// for ICMP/unknown). Not re-entrant; consumed immediately by ESP_LOG*.
static const char *ip_info(const uint8_t *ip, size_t ip_len)
{
    static char s[80];
    if (ip_len < 20 || (ip[0] >> 4) != 4) {
        snprintf(s, sizeof(s), "(not IPv4)");
        return s;
    }
    uint8_t proto = ip[9];
    uint8_t ihl   = (uint8_t)((ip[0] & 0x0Fu) * 4u);
    if (ihl < 20) ihl = 20;
    const char *pname;
    char pbuf[6];
    switch (proto) {
        case 1:  pname = "ICMP"; break;
        case 6:  pname = "TCP";  break;
        case 17: pname = "UDP";  break;
        default:
            snprintf(pbuf, sizeof(pbuf), "P%u", proto);
            pname = pbuf;
            break;
    }
    if ((proto == 6 || proto == 17) && ip_len >= (size_t)ihl + 4u) {
        const uint8_t *t = ip + ihl;
        uint16_t sp = (uint16_t)((t[0] << 8) | t[1]);
        uint16_t dp = (uint16_t)((t[2] << 8) | t[3]);
        snprintf(s, sizeof(s), "%s %u.%u.%u.%u:%u→%u.%u.%u.%u:%u",
                 pname,
                 ip[12], ip[13], ip[14], ip[15], sp,
                 ip[16], ip[17], ip[18], ip[19], dp);
    } else {
        snprintf(s, sizeof(s), "%s %u.%u.%u.%u→%u.%u.%u.%u",
                 pname,
                 ip[12], ip[13], ip[14], ip[15],
                 ip[16], ip[17], ip[18], ip[19]);
    }
    return s;
}

// ── WebSocket log — async queue ───────────────────────────────────────────────
// ax25ip_output() is called from the lwIP tcpip task while holding the tcpip
// core lock.  Calling audio_stream_ws_send_text() from there would try to do
// socket I/O (httpd_ws_send_data → lwIP send → tcpip_api_call) which sends a
// message TO the tcpip task and waits — instant deadlock.
//
// Fix: enqueue the log string from the lwIP callback (non-blocking, no socket
// ops), and drain from a dedicated low-priority task that runs outside the lock.
//
// ax25ip_rx_frame() is called from receive_audio_task (not the lwIP task), so
// it can call audio_stream_ws_send_text() directly.

#define IPLOG_MSG_LEN  160
#define IPLOG_QUEUE_LEN  8

static QueueHandle_t s_iplog_q;

static void iplog_drain_task(void *arg)
{
    (void)arg;
    char buf[IPLOG_MSG_LEN];
    while (1) {
        if (xQueueReceive(s_iplog_q, buf, portMAX_DELAY) == pdTRUE)
            audio_stream_ws_send_text(buf);
    }
}

// Build the JSON and enqueue it non-blockingly.  Safe from any context.
static void ws_log_ip_async(const char *dir, const uint8_t *ip_pkt, size_t ip_len)
{
    if (!s_iplog_q) return;
    char buf[IPLOG_MSG_LEN];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"aprs\",\"src\":\"IP-GW\",\"dst\":\"%s\","
        "\"path\":\"\",\"info\":\"[IP %s] %s\"}",
        dir, dir, ip_info(ip_pkt, ip_len));
    xQueueSend(s_iplog_q, buf, 0);   // drop if full (never blocks)
}

// Direct send — only call from non-lwIP task contexts (e.g. receive_audio_task).
static void ws_log_ip(const char *dir, const uint8_t *ip_pkt, size_t ip_len)
{
    char buf[IPLOG_MSG_LEN];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"aprs\",\"src\":\"IP-GW\",\"dst\":\"%s\","
        "\"path\":\"\",\"info\":\"[IP %s] %s\"}",
        dir, dir, ip_info(ip_pkt, ip_len));
    audio_stream_ws_send_text(buf);
}

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
    if (ip_len > s_mtu) {
        ESP_LOGW(TAG, "drop: IP frame too large (%u > %u)", ip_len, s_mtu);
        return ERR_MEM;
    }

    if (s_tun_mode) {
        // TUN mode: prepend Linux TUN PI header (flags=0x0000, proto=IPv4 0x0800).
        // tncattach reads this 4-byte header from the TUN fd before the IP packet.
        uint8_t frame[4 + AX25IP_MTU_MAX];
        frame[0] = 0x00; frame[1] = 0x00; frame[2] = 0x08; frame[3] = 0x00;
        uint16_t offset = 4;
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            memcpy(frame + offset, q->payload, q->len);
            offset += q->len;
        }
        // Never block here: this runs on the lwIP tcpip thread. ERR_MEM tells the
        // stack the packet was not sent, so TCP keeps the segment and retries at
        // its own pace instead of assuming it went on air and waiting for an ACK
        // that can never arrive.
        if (!afsk_queue_tx_frame(frame, (size_t)offset))
            return ERR_MEM;
        ESP_LOGI(TAG, "TX %u bytes TUN→RF  %s", ip_len, ip_info(frame + 4, ip_len));
        ws_log_ip_async("TX", frame + 4, ip_len);
        return ERR_OK;
    }

    // AX.25 UI frame layout:
    // [DST:7][SRC:7][CTRL:1=0x03][PID:1=0xCC][IP payload...]
    uint8_t frame[16 + AX25IP_MTU_MAX];

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

    if (!afsk_queue_tx_frame(frame, (size_t)offset))
        return ERR_MEM;   // see TUN branch: signal the stack instead of dropping
    ESP_LOGI(TAG, "TX %u bytes IP→AX.25  %s", ip_len, ip_info(frame + 16, ip_len));
    ws_log_ip_async("TX", frame + 16, ip_len);
    return ERR_OK;
}

// ── lwIP netif init callback ──────────────────────────────────────────────────
static err_t ax25ip_netif_init(struct netif *netif)
{
    netif->output     = ax25ip_output;
    netif->mtu        = s_mtu;   // set from ip.mtu before netif_add()
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

    // MTU: must mirror the value tncattach uses on the host (--mtu).
    cJSON *j_mtu = cJSON_GetObjectItem(ip_obj, "mtu");
    if (cJSON_IsNumber(j_mtu)) {
        int m = (int)j_mtu->valueint;
        if (m < 128)             m = 128;              // below this IP is unusable
        if (m > AX25IP_MTU_MAX)  m = AX25IP_MTU_MAX;   // TX buffer ceiling
        if (m != (int)j_mtu->valueint)
            ESP_LOGW(TAG, "ip.mtu %d out of range, clamped to %d",
                     (int)j_mtu->valueint, m);
        s_mtu = (uint16_t)m;
    }

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

    // Async log queue + drain task (created once, even if ax25ip_init is called again)
    if (!s_iplog_q) {
        s_iplog_q = xQueueCreate(IPLOG_QUEUE_LEN, IPLOG_MSG_LEN);
        if (s_iplog_q)
            xTaskCreate(iplog_drain_task, "iplog", 2048, NULL, 2, NULL);
    }

    ESP_LOGI(TAG, "AX.25 IP gateway up: %s/%s  (call=%s-%d  MTU=%u  mode=%s)",
             j_addr->valuestring, j_mask->valuestring,
             callsign, (int)s_ssid, s_mtu,
             s_tun_mode ? "tun" : "ax25");
    ESP_LOGI(TAG, "Add host route:  ip route add %s/%s via <ESP32-WiFi-IP>",
             j_addr->valuestring, j_mask->valuestring);
    return true;
}

bool ax25ip_get_addr(ip4_addr_t *addr)
{
    if (!s_enabled) return false;
    addr->addr = s_netif.ip_addr.addr;  // IPv4-only: ip_addr_t == ip4_addr_t
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
            // Tráfico normal: on_ax25_raw_frame() nos pasa TODA trama recibida por
            // RF, y las APRS de terceros (dst "APxxxx" → hdr 82A0...) no llevan la
            // cabecera PI de TUN. No es un error, así que va en DEBUG.
            ESP_LOGD(TAG, "TUN: drop: not a TUN-IP frame (len=%u hdr=%02X%02X%02X%02X)",
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
        ESP_LOGI(TAG, "RX %u bytes TUN-IP←RF → lwIP  %s", (unsigned)ip_len, ip_info(buf + 4, ip_len));
        ws_log_ip("RX", buf + 4, ip_len);
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

    ESP_LOGI(TAG, "RX %u bytes IP←AX.25 → lwIP  %s", (unsigned)ip_len, ip_info(p, ip_len));
    ws_log_ip("RX", p, ip_len);

    // Inject into lwIP (tcpip_input is task-safe; sends msg to tcpip task).
    if (s_netif.input(q, &s_netif) != ERR_OK) {
        ESP_LOGW(TAG, "lwIP input rejected pbuf");
        pbuf_free(q);
    }
}

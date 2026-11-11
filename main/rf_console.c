#include "rf_console.h"

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"

#include "ax25ip.h"
#include "aux_config.h"
#include "rx_stats.h"
#include "heard.h"
#include <strings.h>
#include <stdlib.h>
#include <unistd.h>
#include "LibAPRS-esp32-i2s/src/LibAPRS.h"

#define TAG "rf_console"

static int s_port = 23;

// ---------------------------------------------------------------------------
// Per-connection write buffer
// Accumulates output so that banner + response + prompt travel as one TCP
// segment (or one UDP datagram) over the slow RF link, avoiding TCP
// reordering stalls at the client.
// ---------------------------------------------------------------------------

typedef struct {
    int  fd;
    bool is_udp;
    struct sockaddr_in peer;  // UDP destination (unused for TCP)
    char buf[1024];
    int  len;
} conn_t;

static void conn_send_raw(conn_t *c, const char *data, int n)
{
    if (c->is_udp)
        sendto(c->fd, data, n, 0, (struct sockaddr *)&c->peer, sizeof(c->peer));
    else
        send(c->fd, data, n, MSG_NOSIGNAL);
}

static void conn_write(conn_t *c, const char *data, int n)
{
    if (c->len + n > (int)sizeof(c->buf)) {
        if (c->len > 0) { conn_send_raw(c, c->buf, c->len); c->len = 0; }
        if (n >= (int)sizeof(c->buf)) { conn_send_raw(c, data, n); return; }
    }
    memcpy(c->buf + c->len, data, n);
    c->len += n;
}

static void conn_flush(conn_t *c)
{
    if (c->len > 0) { conn_send_raw(c, c->buf, c->len); c->len = 0; }
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

static void cmd_help(conn_t *c)
{
    static const char *msg =
        "Commands:\r\n"
        "  help    - this message\r\n"
        "  status  - system status\r\n"
        "  config  - summary; 'config <section>' shows one section\r\n"
        "  stats   - RX frame counters\r\n"
        "  heard   - last stations heard\r\n"
        "  beacon  - send position beacon now\r\n"
        "  reboot  - restart the device\r\n"
        "  quit    - close connection\r\n";
    conn_write(c, msg, strlen(msg));
}

static void cmd_status(conn_t *c)
{
    char rsp[512];
    int  n = 0;

    // Callsign
    char call[16] = "?";
    int  ssid_num = 0;
    APRS_getCallsign(call, &ssid_num);
    n += snprintf(rsp + n, sizeof(rsp) - (size_t)n,
                  "Callsign : %s-%d\r\n", call, ssid_num);

    // RF interface IP
    ip4_addr_t rf_ip;
    if (ax25ip_get_addr(&rf_ip)) {
        char ip_str[16];
        inet_ntop(AF_INET, &rf_ip.addr, ip_str, sizeof(ip_str));
        n += snprintf(rsp + n, sizeof(rsp) - (size_t)n,
                      "RF IP    : %s\r\n", ip_str);
    }

    // WiFi STA IP
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta) {
        esp_netif_ip_info_t info;
        if (esp_netif_get_ip_info(sta, &info) == ESP_OK && info.ip.addr) {
            char ip_str[16];
            esp_ip4addr_ntoa(&info.ip, ip_str, sizeof(ip_str));
            n += snprintf(rsp + n, sizeof(rsp) - (size_t)n,
                          "WiFi IP  : %s\r\n", ip_str);
        }
    }

    // Heap
    n += snprintf(rsp + n, sizeof(rsp) - (size_t)n,
                  "Heap     : %u B free, %u B min\r\n",
                  (unsigned)esp_get_free_heap_size(),
                  (unsigned)esp_get_minimum_free_heap_size());

    // Uptime
    uint32_t uptime_s = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
    n += snprintf(rsp + n, sizeof(rsp) - (size_t)n,
                  "Uptime   : %lu s\r\n", (unsigned long)uptime_s);

    conn_write(c, rsp, (size_t)n);
}

// Elimina recursivamente los campos "password" (objetos y arrays).
static void strip_secrets(cJSON *node)
{
    cJSON *it = node ? node->child : NULL;
    while (it) {
        cJSON *next = it->next;
        if (it->string && strcasecmp(it->string, "password") == 0)
            cJSON_Delete(cJSON_DetachItemViaPointer(node, it));
        else if (cJSON_IsObject(it) || cJSON_IsArray(it))
            strip_secrets(it);
        it = next;
    }
}

static const char *cfg_bool(const cJSON *cfg, const char *sec, const char *key)
{
    const cJSON *s = cJSON_GetObjectItem(cfg, sec);
    const cJSON *v = s ? cJSON_GetObjectItem(s, key) : NULL;
    return cJSON_IsBool(v) ? (cJSON_IsTrue(v) ? "on" : "off") : "?";
}

static void cmd_config(conn_t *c, const char *arg)
{
    static const char *err = "error: config not loaded\r\n";
    cJSON *cfg = config_get();
    if (!cfg) { conn_write(c, err, strlen(err)); return; }

    char rsp[256];
    if (!arg || !*arg) {
        // Resumen corto: el JSON completo tardaría minutos a 1200 bps.
        char call[16] = "?";
        int ssid_num = 0;
        APRS_getCallsign(call, &ssid_num);
        int n = snprintf(rsp, sizeof(rsp), "Callsign: %s-%d\r\nSections:", call, ssid_num);
        for (cJSON *it = cfg->child; it && n < (int)sizeof(rsp) - 20; it = it->next)
            if (it->string && (cJSON_IsObject(it) || cJSON_IsArray(it)))
                n += snprintf(rsp + n, sizeof(rsp) - (size_t)n, " %s", it->string);
        n += snprintf(rsp + n, sizeof(rsp) - (size_t)n,
                      "\r\nIL2P:%s LBT:%s IP:%s\r\nUse: config <section>\r\n",
                      cfg_bool(cfg, "il2p", "enabled"),
                      cfg_bool(cfg, "tx", "lbt_enabled"),
                      cfg_bool(cfg, "ip", "enabled"));
        config_free_json(cfg);
        conn_write(c, rsp, (size_t)n);
        return;
    }

    cJSON *sec = cJSON_GetObjectItem(cfg, arg);   // insensible a mayusculas
    if (!sec) {
        config_free_json(cfg);
        static const char *nf = "error: no such section\r\n";
        conn_write(c, nf, strlen(nf));
        return;
    }
    cJSON *copy = cJSON_Duplicate(sec, 1);
    config_free_json(cfg);
    if (!copy) { conn_write(c, err, strlen(err)); return; }
    strip_secrets(copy);
    char *js = cJSON_PrintUnformatted(copy);
    cJSON_Delete(copy);
    if (!js) {
        static const char *pe = "error: cJSON_Print failed\r\n";
        conn_write(c, pe, strlen(pe));
        return;
    }
    conn_write(c, js, strlen(js));
    conn_write(c, "\r\n", 2);
    free(js);
}

static void cmd_stats(conn_t *c)
{
    char rsp[256];
    int n = snprintf(rsp, sizeof(rsp),
        "v1 ok:%lu err:%lu flags:%lu ovf:%lu\r\n"
        "v2 ok:%lu err:%lu flags:%lu ovf:%lu\r\n",
        (unsigned long)rx_stats_v1.frames_crc_ok, (unsigned long)rx_stats_v1.frames_crc_err,
        (unsigned long)rx_stats_v1.hdlc_flags, (unsigned long)rx_stats_v1.fifo_overflows,
        (unsigned long)rx_stats_v2.frames_crc_ok, (unsigned long)rx_stats_v2.frames_crc_err,
        (unsigned long)rx_stats_v2.hdlc_flags, (unsigned long)rx_stats_v2.fifo_overflows);
    conn_write(c, rsp, (size_t)n);
}

// Opcional: la define gps.c si se añade (devuelve true si encola la baliza).
#include "gps.h"

static void cmd_heard(conn_t *c)
{
    char rsp[640];
    int n = heard_list(rsp, sizeof(rsp));
    conn_write(c, rsp, (size_t)n);
}

static void cmd_beacon(conn_t *c)
{
    const char *m;
    m = gps_send_beacon_now() ? "Beacon queued\r\n" : "beacon: failed (no fix?)\r\n";
    conn_write(c, m, strlen(m));
}

static void cmd_reboot(conn_t *c)
{
    static const char *m = "Rebooting\r\n";
    conn_write(c, m, strlen(m));
    conn_flush(c);
    vTaskDelay(pdMS_TO_TICKS(500));   // deja salir la respuesta
    esp_restart();
}

// Dispatch a NUL-terminated, trimmed line. Returns true to keep connection open.
static bool dispatch(conn_t *c, char *line)
{
    // Strip trailing whitespace
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' ||
                        line[len - 1] == ' '))
        line[--len] = '\0';

    if (len == 0) {
        conn_write(c, "> ", 2);
        return true;
    }

    // Lowercase for matching
    char lower[128];
    snprintf(lower, sizeof(lower), "%s", line);
    for (char *p = lower; *p; p++)
        if (*p >= 'A' && *p <= 'Z') *p += 32;

    if (strcmp(lower, "help") == 0 || strcmp(lower, "?") == 0) {
        cmd_help(c);
    } else if (strcmp(lower, "status") == 0) {
        cmd_status(c);
    } else if (strcmp(lower, "config") == 0 || strncmp(lower, "config ", 7) == 0) {
        const char *arg = lower + 6;
        while (*arg == ' ') arg++;
        cmd_config(c, arg);
    } else if (strcmp(lower, "stats") == 0) {
        cmd_stats(c);
    } else if (strcmp(lower, "heard") == 0) {
        cmd_heard(c);
    } else if (strcmp(lower, "beacon") == 0) {
        cmd_beacon(c);
    } else if (strcmp(lower, "reboot") == 0) {
        cmd_reboot(c);
    } else if (strcmp(lower, "quit") == 0 || strcmp(lower, "exit") == 0) {
        conn_write(c, "Bye\r\n", 5);
        return false;
    } else {
        char unk[80];
        snprintf(unk, sizeof(unk), "Unknown command: %s\r\n", line);
        conn_write(c, unk, strlen(unk));
    }

    conn_write(c, "> ", 2);
    return true;
}

// ---------------------------------------------------------------------------
// Per-client loop
// ---------------------------------------------------------------------------

static void handle_client(int fd, const char *client_ip)
{
    ESP_LOGI(TAG, "client connected: %s", client_ip);

    // Do NOT send the banner here.  The TCP connection was just accepted but the
    // RF round-trip (~6 s at 1200 bps) is longer than lwIP's default RTO (1.5 s).
    // Sending data before the client proves the link works in both directions would
    // trigger a retransmit storm that fills s_tx_queue and the connection would be
    // reset before the banner ever arrives.  Instead, send the banner with the first
    // response — by then recv() has returned, so we know RF client→ESP is working,
    // which means ESP→client will also work.

    conn_t c = { .fd = fd, .is_udp = false, .len = 0 };
    char line[128];
    int  pos = 0;
    bool skip_lf   = false;
    bool need_banner = true;

    bool done = false;
    while (!done) {
        char rb[32];
        int rn = recv(fd, rb, sizeof(rb), 0);
        if (rn <= 0) break;

        for (int i = 0; i < rn && !done; i++) {
            char ch = rb[i];
            if (ch == '\n' && skip_lf) {
                skip_lf = false;    // \n de un par \r\n ya despachado
            } else if (ch == '\n' || ch == '\r') {
                skip_lf = (ch == '\r');
                line[pos] = '\0';
                if (need_banner) {
                    static const char *banner = "RF Console (type 'help' for commands)\r\n";
                    conn_write(&c, banner, strlen(banner));
                    need_banner = false;
                }
                bool keep = dispatch(&c, line);
                conn_flush(&c);
                if (!keep) done = true;
                pos = 0;
            } else {
                skip_lf = false;
                if (pos < (int)sizeof(line) - 1)
                    line[pos++] = ch;
                else {
                    static const char *err = "error: line too long\r\n> ";
                    conn_write(&c, err, strlen(err));
                    conn_flush(&c);
                    pos = 0;
                }
            }
        }
    }

    ESP_LOGI(TAG, "client disconnected: %s", client_ip);
    close(fd);
}

// ---------------------------------------------------------------------------
// UDP server task
// Stateless: each received datagram is one command; each sendto() is one reply.
// No banner, no session state — avoids TCP ordering stalls entirely.
// ---------------------------------------------------------------------------

static void rf_console_udp_task(void *arg)
{
    (void)arg;
    ip4_addr_t rf_ip;
    int sock = -1;
    for (;;) {   // reintenta cada 5 s hasta que ax25ip este arriba y el bind funcione
        if (ax25ip_get_addr(&rf_ip)) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock >= 0) {
                int opt = 1;
                setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
                struct sockaddr_in addr = {
                    .sin_family      = AF_INET,
                    .sin_port        = htons((uint16_t)s_port),
                    .sin_addr.s_addr = rf_ip.addr,
                };
                if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) == 0) break;
                ESP_LOGW(TAG, "udp bind() failed: %d (retry)", errno);
                close(sock);
                sock = -1;
            } else {
                ESP_LOGW(TAG, "udp socket() failed: %d (retry)", errno);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    char ip_str[16];
    inet_ntop(AF_INET, &rf_ip.addr, ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "udp listening on %s:%d", ip_str, s_port);

    char rxbuf[128];
    for (;;) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int n = recvfrom(sock, rxbuf, sizeof(rxbuf) - 1, 0,
                         (struct sockaddr *)&peer, &peer_len);
        if (n <= 0) continue;
        rxbuf[n] = '\0';

        // Trim trailing CR/LF
        while (n > 0 && (rxbuf[n - 1] == '\r' || rxbuf[n - 1] == '\n'))
            rxbuf[--n] = '\0';

        char peer_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
        ESP_LOGD(TAG, "udp cmd from %s: %s", peer_ip, rxbuf);

        conn_t c = { .fd = sock, .is_udp = true, .peer = peer, .len = 0 };
        dispatch(&c, rxbuf);
        conn_flush(&c);
    }
}

// ---------------------------------------------------------------------------
// TCP server task
// ---------------------------------------------------------------------------

static void rf_console_task(void *arg)
{
    (void)arg;
    ip4_addr_t rf_ip;
    int srv = -1;
    for (;;) {   // reintenta cada 5 s hasta que ax25ip este arriba y listen() funcione
        if (ax25ip_get_addr(&rf_ip)) {
            srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (srv >= 0) {
                int opt = 1;
                setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
                struct sockaddr_in addr = {
                    .sin_family      = AF_INET,
                    .sin_port        = htons((uint16_t)s_port),
                    .sin_addr.s_addr = rf_ip.addr,
                };
                if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
                    if (listen(srv, 1) == 0) break;
                    ESP_LOGW(TAG, "listen() failed: %d (retry)", errno);
                } else {
                    ESP_LOGW(TAG, "bind() failed: %d (retry)", errno);
                }
                close(srv);
                srv = -1;
            } else {
                ESP_LOGW(TAG, "socket() failed: %d (retry)", errno);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    char ip_str[16];
    inet_ntop(AF_INET, &rf_ip.addr, ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "listening on %s:%d", ip_str, s_port);

    for (;;) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int fd = accept(srv, (struct sockaddr *)&client_addr, &client_len);
        if (fd < 0) {
            ESP_LOGW(TAG, "accept() error: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));

        int sndbuf = 512;
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        struct timeval tv = { .tv_sec = 120, .tv_usec = 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        handle_client(fd, client_ip);
    }
}

// ---------------------------------------------------------------------------
// Public init
// ---------------------------------------------------------------------------

void rf_console_init(const cJSON *cfg)
{
    bool enable_tcp = true;
    bool enable_udp = false;

    if (cfg) {
        cJSON *con = cJSON_GetObjectItem(cfg, "console");
        if (con) {
            // Legacy: "enabled" controls TCP (backward compat)
            cJSON *en = cJSON_GetObjectItem(con, "enabled");
            if (cJSON_IsBool(en)) enable_tcp = cJSON_IsTrue(en);

            // Explicit per-protocol flags override legacy "enabled"
            cJSON *tcp = cJSON_GetObjectItem(con, "tcp");
            if (cJSON_IsBool(tcp)) enable_tcp = cJSON_IsTrue(tcp);
            cJSON *udp = cJSON_GetObjectItem(con, "udp");
            if (cJSON_IsBool(udp)) enable_udp = cJSON_IsTrue(udp);

            cJSON *port = cJSON_GetObjectItem(con, "port");
            if (cJSON_IsNumber(port) && port->valueint > 0)
                s_port = port->valueint;
        }
    }

    if (!enable_tcp && !enable_udp) return;

    // Las tareas esperan a que ax25ip este disponible (reintento cada 5 s).
    if (enable_tcp)
        xTaskCreate(rf_console_task, "rf_console", 6144, NULL, 3, NULL);
    if (enable_udp)
        xTaskCreate(rf_console_udp_task, "rf_console_u", 4096, NULL, 3, NULL);
}

#include "rf_console.h"

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
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
#include "LibAPRS-esp32-i2s/src/LibAPRS.h"

#define TAG "rf_console"

static int s_port = 23;

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

static void cmd_help(int fd)
{
    static const char *msg =
        "Commands:\r\n"
        "  help    - this message\r\n"
        "  status  - system status\r\n"
        "  config  - dump config JSON\r\n"
        "  quit    - close connection\r\n";
    send(fd, msg, strlen(msg), MSG_NOSIGNAL);
}

static void cmd_status(int fd)
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

    send(fd, rsp, (size_t)n, MSG_NOSIGNAL);
}

static void cmd_config(int fd)
{
    cJSON *cfg = config_get();
    if (!cfg) {
        static const char *err = "error: config not loaded\r\n";
        send(fd, err, strlen(err), MSG_NOSIGNAL);
        return;
    }
    char *js = cJSON_PrintUnformatted(cfg);
    if (!js) {
        static const char *err = "error: cJSON_Print failed\r\n";
        send(fd, err, strlen(err), MSG_NOSIGNAL);
        return;
    }
    send(fd, js, strlen(js), MSG_NOSIGNAL);
    send(fd, "\r\n", 2, MSG_NOSIGNAL);
    free(js);
}

// Dispatch a NUL-terminated, trimmed line. Returns true to keep connection open.
static bool dispatch(int fd, char *line)
{
    // Strip trailing whitespace
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' ||
                        line[len - 1] == ' '))
        line[--len] = '\0';

    if (len == 0) {
        send(fd, "> ", 2, MSG_NOSIGNAL);
        return true;
    }

    // Lowercase for matching
    char lower[64];
    snprintf(lower, sizeof(lower), "%s", line);
    for (char *p = lower; *p; p++)
        if (*p >= 'A' && *p <= 'Z') *p += 32;

    if (strcmp(lower, "help") == 0 || strcmp(lower, "?") == 0) {
        cmd_help(fd);
    } else if (strcmp(lower, "status") == 0) {
        cmd_status(fd);
    } else if (strcmp(lower, "config") == 0) {
        cmd_config(fd);
    } else if (strcmp(lower, "quit") == 0 || strcmp(lower, "exit") == 0) {
        send(fd, "Bye\r\n", 5, MSG_NOSIGNAL);
        return false;
    } else {
        char unk[80];
        snprintf(unk, sizeof(unk), "Unknown command: %s\r\n", line);
        send(fd, unk, strlen(unk), MSG_NOSIGNAL);
    }

    send(fd, "> ", 2, MSG_NOSIGNAL);
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

    char line[128];
    int  pos = 0;
    bool skip_lf   = false;
    bool need_banner = true;

    while (1) {
        char ch;
        int n = recv(fd, &ch, 1, 0);
        if (n <= 0) break;

        if (ch == '\n' && skip_lf) {
            skip_lf = false;    // consumed the \n of a \r\n pair — already dispatched
        } else if (ch == '\n' || ch == '\r') {
            skip_lf = (ch == '\r');
            line[pos] = '\0';
            if (need_banner) {
                static const char *banner = "RF Console (type 'help' for commands)\r\n";
                send(fd, banner, strlen(banner), MSG_NOSIGNAL);
                need_banner = false;
            }
            if (!dispatch(fd, line)) break;
            pos = 0;
        } else {
            skip_lf = false;
            if (pos < (int)sizeof(line) - 1)
                line[pos++] = ch;
            else {
                // Line too long: flush and report
                line[pos] = '\0';
                static const char *err = "error: line too long\r\n> ";
                send(fd, err, strlen(err), MSG_NOSIGNAL);
                pos = 0;
            }
        }
    }

    ESP_LOGI(TAG, "client disconnected: %s", client_ip);
    close(fd);
}

// ---------------------------------------------------------------------------
// Server task
// ---------------------------------------------------------------------------

static void rf_console_task(void *arg)
{
    ip4_addr_t rf_ip = *(ip4_addr_t *)arg;
    free(arg);

    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons((uint16_t)s_port),
        .sin_addr.s_addr = rf_ip.addr,
    };
    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(srv);
        vTaskDelete(NULL);
        return;
    }
    listen(srv, 1);

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

        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
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
    if (cfg) {
        cJSON *con = cJSON_GetObjectItem(cfg, "console");
        if (con) {
            cJSON *en = cJSON_GetObjectItem(con, "enabled");
            if (cJSON_IsBool(en) && !cJSON_IsTrue(en)) return;
            cJSON *port = cJSON_GetObjectItem(con, "port");
            if (cJSON_IsNumber(port) && port->valueint > 0)
                s_port = port->valueint;
        }
    }

    ip4_addr_t *addr = malloc(sizeof(ip4_addr_t));
    if (!addr) {
        ESP_LOGE(TAG, "malloc failed");
        return;
    }
    if (!ax25ip_get_addr(addr)) {
        ESP_LOGW(TAG, "ax25ip not up — console disabled");
        free(addr);
        return;
    }

    xTaskCreate(rf_console_task, "rf_console", 4096, addr, 3, NULL);
}

#include "igate.h"
#include "dedup.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"   // afsk_queue_tx_frame
#include "LibAPRS-esp32-i2s/src/AX25.h"   // AX25_MAX_FRAME_LEN
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

static const char *TAG = "igate";

#define IGATE_QUEUE_DEPTH   4
#define IGATE_LINE_MAX      768
#define IGATE_KEEPALIVE_S   20
#define IGATE_IDLE_TIMEOUT_S 90     // el servidor envía '#' cada ~20 s
#define IGATE_HEARD_SECS    1800    // 30 min
#define IGATE_RF_MIN_GAP_MS 5000
#define IGATE_DEDUP_SLOTS   8
#define IGATE_DEDUP_TTL_US  (600LL * 1000000LL)

// Función opcional de heard.c (puede no existir: se comprueba que no sea NULL).
extern bool heard_seen_recent(const char *call, int ssid, uint32_t secs) __attribute__((weak));

typedef struct {
    uint16_t len;
    uint8_t  data[AX25_MAX_FRAME_LEN];
} igate_item_t;

static QueueHandle_t s_q;
static volatile bool s_connected;
static volatile bool s_verified;
static bool  s_enabled;
static bool  s_rf_gate;
static char  s_server[64];
static int   s_port;
static char  s_call[32];        // indicativo de login (con SSID)
static char  s_pass[12];
static char  s_filter[96];
static char  s_my_call[8];      // aprs.callsign (para el origen del tercero y filtros)
static int   s_my_ssid;
static volatile uint32_t s_rx_gated, s_rx_dropped, s_is_rx, s_rf_gated, s_reconnects;

static const char *cfg_str(const cJSON *o, const char *k, const char *def)
{
    const cJSON *j = cJSON_GetObjectItemCaseSensitive(o, k);
    return (cJSON_IsString(j) && j->valuestring) ? j->valuestring : def;
}

void igate_init(cJSON *cfg)
{
    s_enabled = false;
    if (!cfg) return;
    const cJSON *ig = cJSON_GetObjectItemCaseSensitive(cfg, "igate");
    if (!ig) return;
    const cJSON *en = cJSON_GetObjectItemCaseSensitive(ig, "enabled");
    if (!cJSON_IsTrue(en)) return;

    const cJSON *aprs = cJSON_GetObjectItemCaseSensitive(cfg, "aprs");
    const char *ac = aprs ? cfg_str(aprs, "callsign", "") : "";
    const cJSON *as = aprs ? cJSON_GetObjectItemCaseSensitive(aprs, "ssid") : NULL;
    strncpy(s_my_call, ac, sizeof(s_my_call) - 1);
    s_my_ssid = cJSON_IsNumber(as) ? (int)as->valueint : 0;

    const char *cs = cfg_str(ig, "callsign", "");
    if (cs[0]) snprintf(s_call, sizeof(s_call), "%s", cs);
    else if (s_my_ssid > 0) snprintf(s_call, sizeof(s_call), "%s-%d", s_my_call, s_my_ssid);
    else snprintf(s_call, sizeof(s_call), "%s", s_my_call);
    if (!s_call[0]) { ESP_LOGW(TAG, "iGate activado sin indicativo: desactivado"); return; }

    snprintf(s_server, sizeof(s_server), "%s", cfg_str(ig, "server", "rotate.aprs2.net"));
    const cJSON *pj = cJSON_GetObjectItemCaseSensitive(ig, "port");
    s_port = cJSON_IsNumber(pj) ? (int)pj->valueint : 14580;
    snprintf(s_pass, sizeof(s_pass), "%s", cfg_str(ig, "passcode", ""));
    if (!s_pass[0]) snprintf(s_pass, sizeof(s_pass), "-1");   // sólo lectura
    snprintf(s_filter, sizeof(s_filter), "%s", cfg_str(ig, "filter", ""));
    s_rf_gate = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(ig, "rf_gate"));

    s_q = xQueueCreate(IGATE_QUEUE_DEPTH, sizeof(igate_item_t));
    if (!s_q) { ESP_LOGE(TAG, "sin memoria para la cola"); return; }
    extern void igate_task(void *);
    if (xTaskCreate(igate_task, "igate", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "no se pudo crear la tarea");
        vQueueDelete(s_q); s_q = NULL;
        return;
    }
    s_enabled = true;
    ESP_LOGI(TAG, "iGate -> %s:%d como %s (rf_gate=%d)", s_server, s_port, s_call, s_rf_gate);
}

bool igate_connected(void) { return s_enabled && s_connected; }

void igate_status_json(char *buf, size_t n)
{
    snprintf(buf, n,
        "{\"enabled\":%s,\"connected\":%s,\"verified\":%s,\"server\":\"%s\","
        "\"rx_gated\":%lu,\"rx_dropped\":%lu,\"is_rx\":%lu,\"rf_gated\":%lu,\"reconnects\":%lu}",
        s_enabled ? "true" : "false", s_connected ? "true" : "false",
        s_verified ? "true" : "false", s_enabled ? s_server : "",
        (unsigned long)s_rx_gated, (unsigned long)s_rx_dropped, (unsigned long)s_is_rx,
        (unsigned long)s_rf_gated, (unsigned long)s_reconnects);
}

void igate_rx_frame(const uint8_t *ax25, size_t len)
{
    if (!s_enabled || !s_q || !s_connected || !ax25) return;
    if (len < 16 || len > AX25_MAX_FRAME_LEN) return;
    igate_item_t it;
    it.len = (uint16_t)len;
    memcpy(it.data, ax25, len);
    if (xQueueSend(s_q, &it, 0) != pdTRUE) s_rx_dropped++;
}

// ---------------------------------------------------------------------------
// AX.25 -> TNC2
// ---------------------------------------------------------------------------

// Escribe "CALL-SSID" (sin '*') de la dirección en p[0..6]. Devuelve longitud.
static int addr_str(const uint8_t *p, char *out, int *ssid_out)
{
    int k = 0;
    for (int i = 0; i < 6; i++) {
        char ch = (char)(p[i] >> 1);
        if (ch != ' ' && ch != 0) out[k++] = ch;
    }
    int ssid = (p[6] >> 1) & 0x0F;
    if (ssid_out) *ssid_out = ssid;
    if (ssid) k += sprintf(out + k, "-%d", ssid);
    out[k] = '\0';
    return k;
}

static bool path_blocked(const char *name)
{
    static const char *bad[] = { "TCPIP", "TCPXX", "NOGATE", "RFONLY" };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
        if (strncmp(name, bad[i], strlen(bad[i])) == 0) return true;
    return false;
}

// Convierte la trama a una línea TNC2 con q-construct. Devuelve longitud (0 = descartar).
static int ax25_to_tnc2(const uint8_t *f, size_t len, char *out, size_t max)
{
    if (len < 16) return 0;
    char src[12], dst[12], tmp[12];
    addr_str(f + 7, src, NULL);
    addr_str(f, dst, NULL);
    if (!src[0] || !dst[0]) return 0;

    // Origen propio: no reenviar
    char mine[16];
    snprintf(mine, sizeof(mine), "%s", s_my_call);
    if (s_my_ssid) snprintf(mine + strlen(mine), sizeof(mine) - strlen(mine), "-%d", s_my_ssid);
    if (strcmp(src, mine) == 0 || strcmp(src, s_call) == 0) return 0;

    int n = snprintf(out, max, "%s>%s", src, dst);
    size_t i = 13;
    int ndig = 0, last_h = -1;
    char digis[8][12];
    while (i < len && !(f[i] & 1)) {
        i += 7;
        if (i + 7 > len) return 0;
        if (ndig >= 8) return 0;
        addr_str(f + i, digis[ndig], NULL);
        if (path_blocked(digis[ndig])) return 0;
        if (f[i + 6] & 0x80) last_h = ndig;
        ndig++;
    }
    // Comprobación del path de nombre de origen/destino también
    (void)tmp;
    i += 7;   // tras la última dirección
    if (i + 2 > len) return 0;
    if (f[i] != 0x03 || f[i + 1] != 0xF0) return 0;   // sólo UI con PID F0
    i += 2;
    if (i >= len) return 0;
    const uint8_t *inf = f + i;
    size_t il = len - i;
    if (inf[0] == '}') return 0;                      // tercero: no reenviar
    while (il && (inf[il - 1] == '\r' || inf[il - 1] == '\n')) il--;
    if (!il) return 0;
    for (size_t k = 0; k < il; k++)
        if (inf[k] == 0 || inf[k] == '\r' || inf[k] == '\n') return 0;

    for (int d = 0; d < ndig && n < (int)max; d++)
        n += snprintf(out + n, max - n, ",%s%s", digis[d], d == last_h ? "*" : "");
    n += snprintf(out + n, max - n, ",qAR,%s:", s_call);
    if (n + (int)il + 3 >= (int)max) return 0;
    memcpy(out + n, inf, il);
    n += (int)il;
    out[n++] = '\r'; out[n++] = '\n';
    out[n] = '\0';
    return n;
}

// ---------------------------------------------------------------------------
// IS -> RF
// ---------------------------------------------------------------------------

static int put_addr(uint8_t *o, const char *call, int ssid, bool last)
{
    int k = 0;
    for (; k < 6 && call[k]; k++) o[k] = (uint8_t)(toupper((unsigned char)call[k]) << 1);
    for (; k < 6; k++) o[k] = ' ' << 1;
    o[6] = (uint8_t)(0x60 | ((ssid & 0x0F) << 1) | (last ? 1 : 0));
    return 7;
}

// Parsea "CALL" o "CALL-N" -> base (<=6) + ssid.
static bool split_call(const char *s, size_t sl, char *base, int *ssid)
{
    size_t k = 0;
    while (k < sl && s[k] != '-' && k < 6) { base[k] = s[k]; k++; }
    base[k] = '\0';
    *ssid = 0;
    if (k == 0) return false;
    if (k < sl) {
        if (s[k] != '-') return false;
        *ssid = atoi(s + k + 1);
        if (*ssid < 0 || *ssid > 15) return false;
    }
    return true;
}

static void handle_is_line(char *line, dedup_t *dd)
{
    static int64_t last_rf_us;
    if (line[0] == '#' || !line[0]) return;
    s_is_rx++;
    if (!s_rf_gate) return;

    char *gt = strchr(line, '>');
    char *colon = strchr(line, ':');
    if (!gt || !colon || gt > colon) return;
    const char *info = colon + 1;
    // Mensaje: ":DESTINAT:texto" (addressee de 9 chars)
    if (strlen(info) < 11 || info[0] != ':' || info[10] != ':') return;

    // Path: descartar NOGATE/RFONLY/TCPXX
    *colon = '\0';
    bool blocked = strstr(gt, "NOGATE") || strstr(gt, "RFONLY") || strstr(gt, "TCPXX");
    *colon = ':';
    if (blocked) return;

    char addr[10];
    memcpy(addr, info + 1, 9); addr[9] = '\0';
    int e = 8; while (e >= 0 && addr[e] == ' ') addr[e--] = '\0';
    char base[8]; int ssid;
    if (!split_call(addr, strlen(addr), base, &ssid)) return;
    if (!heard_seen_recent || !heard_seen_recent(base, ssid, IGATE_HEARD_SECS)) return;

    char sbase[8]; int sssid;
    if (!split_call(line, (size_t)(gt - line), sbase, &sssid)) return;
    // No reenviar mensajes de nuestra propia estación ni dirigidos a un origen propio
    if (strcasecmp(sbase, s_my_call) == 0) return;

    int64_t now = esp_timer_get_time();
    if (last_rf_us && now - last_rf_us < (int64_t)IGATE_RF_MIN_GAP_MS * 1000) return;

    // Info del tercero: }SRC>DST,TCPIP,callsign*:info (hasta el fin de la línea, sin q-construct)
    char head[48];
    *colon = '\0';
    // Path original sin q-construct: usar sólo SRC>DST
    char *comma = strchr(gt, ',');
    size_t hl = comma ? (size_t)(comma - line) : (size_t)(colon - line);
    *colon = ':';
    if (hl >= sizeof(head)) return;
    memcpy(head, line, hl); head[hl] = '\0';

    uint8_t frame[AX25_MAX_FRAME_LEN];
    int n = 0;
    n += put_addr(frame + n, "APESPM", 0, false);
    n += put_addr(frame + n, s_my_call, s_my_ssid, true);
    frame[n++] = 0x03; frame[n++] = 0xF0;
    int room = (int)sizeof(frame) - n;
    int tl = snprintf((char *)frame + n, room, "}%s,TCPIP,%s*:%s", head, s_call, info);
    if (tl <= 0 || tl >= room) return;
    n += tl;

    if (dedup_check_and_add(dd, dedup_hash(frame + 16, (size_t)tl))) return;
    if (afsk_queue_tx_frame(frame, (size_t)n)) {
        last_rf_us = now;
        s_rf_gated++;
        ESP_LOGI(TAG, "IS->RF %s -> %s", head, addr);
    }
}

// ---------------------------------------------------------------------------
// Tarea
// ---------------------------------------------------------------------------

static void drain_queue(void)
{
    igate_item_t *it = malloc(sizeof(*it));
    if (!it) return;
    while (xQueueReceive(s_q, it, 0) == pdTRUE) {}
    free(it);
}

static int send_all(int sock, const char *p, int n)
{
    while (n > 0) {
        int w = send(sock, p, n, 0);
        if (w <= 0) return -1;
        p += w; n -= w;
    }
    return 0;
}

static int connect_is(void)
{
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    char ps[8];
    snprintf(ps, sizeof(ps), "%d", s_port);
    if (getaddrinfo(s_server, ps, &hints, &res) != 0 || !res) {
        ESP_LOGW(TAG, "DNS falló para %s", s_server);
        return -1;
    }
    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) { freeaddrinfo(res); return -1; }
    struct timeval tv = { .tv_sec = 8, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    tv.tv_sec = 1;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int ka = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGW(TAG, "connect falló: errno %d", errno);
        close(sock); freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);
    return sock;
}

void igate_task(void *arg)
{
    (void)arg;
    char *line = malloc(IGATE_LINE_MAX);
    char *out = malloc(IGATE_LINE_MAX);
    igate_item_t *it = malloc(sizeof(*it));
    dedup_entry_t *dstore = calloc(IGATE_DEDUP_SLOTS, sizeof(dedup_entry_t));
    if (!line || !out || !it || !dstore) {
        ESP_LOGE(TAG, "sin memoria; tarea terminada");
        free(line); free(out); free(it); free(dstore);
        s_enabled = false;
        vTaskDelete(NULL);
        return;
    }
    dedup_t dd;
    dedup_init(&dd, dstore, IGATE_DEDUP_SLOTS, IGATE_DEDUP_TTL_US);

    uint32_t backoff_s = 5;
    for (;;) {
        int sock = connect_is();
        if (sock >= 0) {
            char login[256];
            int ll = snprintf(login, sizeof(login), "user %s pass %s vers esp32-aprs-modem %s%s%s\r\n",
                              s_call, s_pass, esp_app_get_description()->version,
                              s_filter[0] ? " filter " : "", s_filter);
            if (ll > 0 && ll < (int)sizeof(login) && send_all(sock, login, ll) == 0) {
                int pos = 0;
                int64_t last_rx = esp_timer_get_time();
                int64_t last_tx = last_rx;
                bool ok = true;
                while (ok) {
                    int r = recv(sock, line + pos, IGATE_LINE_MAX - 1 - pos, 0);
                    int64_t now = esp_timer_get_time();
                    if (r > 0) {
                        last_rx = now;
                        pos += r;
                        line[pos] = '\0';
                        char *start = line, *nl;
                        while ((nl = memchr(start, '\n', (size_t)(line + pos - start)))) {
                            *nl = '\0';
                            if (nl > start && nl[-1] == '\r') nl[-1] = '\0';
                            if (strncmp(start, "# logresp", 9) == 0) {
                                s_verified = strstr(start, " verified") && !strstr(start, "unverified");
                                s_connected = true;
                                backoff_s = 5;
                                ESP_LOGI(TAG, "%s", start);
                            } else if (start[0] != '#') {
                                handle_is_line(start, &dd);
                            }
                            start = nl + 1;
                        }
                        pos = (int)(line + pos - start);
                        memmove(line, start, (size_t)pos);
                        if (pos >= IGATE_LINE_MAX - 1) pos = 0;   // línea excesiva: descartar
                    } else if (r == 0) {
                        ESP_LOGW(TAG, "servidor cerró la conexión");
                        ok = false;
                    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT) {
                        ESP_LOGW(TAG, "recv errno %d", errno);
                        ok = false;
                    }
                    if (!ok) break;
                    if (now - last_rx > (int64_t)IGATE_IDLE_TIMEOUT_S * 1000000) {
                        ESP_LOGW(TAG, "sin datos del servidor, reconectando");
                        break;
                    }
                    // RF -> IS (sólo si la sesión está activa y con passcode válido)
                    while (s_connected && xQueueReceive(s_q, it, 0) == pdTRUE) {
                        if (strcmp(s_pass, "-1") == 0) continue;     // sólo lectura
                        int n = ax25_to_tnc2(it->data, it->len, out, IGATE_LINE_MAX);
                        if (n <= 0) continue;
                        if (send_all(sock, out, n) != 0) { ok = false; break; }
                        s_rx_gated++;
                        last_tx = now;
                    }
                    if (ok && now - last_tx > (int64_t)IGATE_KEEPALIVE_S * 1000000) {
                        if (send_all(sock, "# keepalive\r\n", 13) != 0) ok = false;
                        last_tx = now;
                    }
                }
            }
            s_connected = false;
            s_verified = false;
            close(sock);
            s_reconnects++;
        }
        drain_queue();
        for (uint32_t i = 0; i < backoff_s; i++) vTaskDelay(pdMS_TO_TICKS(1000));
        backoff_s = backoff_s * 2 > 300 ? 300 : backoff_s * 2;
    }
}

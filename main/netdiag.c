// Diagnóstico de red: ping ICMP + miniconsola TCP/UDP.
//
// Estado en el firmware: un búfer circular de texto con contador monótono
// (s_total) y número de generación (s_gen, se incrementa al empezar una
// operación). Los clientes sondean GET /api/netdiag?gen=G&since=N y reciben
// sólo lo nuevo; tras recargar la página piden since=0 y recuperan el búfer.

#include "netdiag.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "ping/ping_sock.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/ip_addr.h"

#include "audio_stream.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"

#define TAG "netdiag"

#define BUF_SZ            4096
#define HOST_MAX          64
#define CONNECT_TIMEOUT_S 30
#define TX_HIST           6

typedef enum { ND_IDLE = 0, ND_PING, ND_TCP, ND_UDP } nd_mode_t;
static const char *const MODE_NAME[] = { "idle", "ping", "tcp", "udp" };

static SemaphoreHandle_t s_mx;
static char       s_buf[BUF_SZ];
static uint32_t   s_total;            // bytes escritos desde el inicio de la operación
static uint32_t   s_gen;
static nd_mode_t  s_mode;             // protegido por s_mx
static volatile bool s_abort;
static volatile bool s_connected;
static int        s_fd = -1;          // protegido por s_mx
static char       s_host[HOST_MAX];
static uint16_t   s_port;
static int        s_ping_count, s_ping_size;
static TaskHandle_t s_ping_task;

// ── Historial TX (debug) ────────────────────────────────────────────────────
typedef struct { int64_t t_us; uint16_t len; int pid; char src[10], dst[10]; } tx_ev_t;
static tx_ev_t  s_tx_hist[TX_HIST];
static uint32_t s_tx_count;
static portMUX_TYPE s_tx_mux = portMUX_INITIALIZER_UNLOCKED;

static void call_str(const uint8_t *a, char *out)
{
    int n = 0;
    for (int i = 0; i < 6; i++) { char c = a[i] >> 1; if (c != ' ') out[n++] = c; }
    int ssid = (a[6] >> 1) & 0x0F;
    if (ssid) n += snprintf(out + n, 10 - n, "-%d", ssid);
    out[n] = '\0';
}

void netdiag_note_tx(const uint8_t *d, size_t len)
{
    if (!d || len < 16) return;
    tx_ev_t e = { .t_us = esp_timer_get_time(), .len = (uint16_t)len, .pid = -1 };
    call_str(d, e.dst);
    call_str(d + 7, e.src);
    size_t i = 7;                                  // fin del campo de direcciones
    while (i + 7 <= len && !(d[i + 6] & 1)) i += 7;
    i += 7;
    if (i + 1 < len) e.pid = d[i + 1];             // d[i] = control
    portENTER_CRITICAL(&s_tx_mux);
    s_tx_hist[s_tx_count % TX_HIST] = e;
    s_tx_count++;
    portEXIT_CRITICAL(&s_tx_mux);
}

// ── Búfer de salida ─────────────────────────────────────────────────────────
// Sólo ASCII imprimible y '\n': simplifica el escapado JSON y la vista.
static void buf_put_locked(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '\r') continue;
        if (c == '\t') c = ' ';
        else if (c != '\n' && (c < 0x20 || c > 0x7E)) c = '.';
        s_buf[s_total % BUF_SZ] = (char)c;
        s_total++;
    }
}

static void buf_put(const char *s, size_t n)
{
    xSemaphoreTake(s_mx, portMAX_DELAY);
    buf_put_locked(s, n);
    xSemaphoreGive(s_mx);
}

static void buf_printf(const char *fmt, ...)
{
    char tmp[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof(tmp)) n = sizeof(tmp) - 1;
    buf_put(tmp, (size_t)n);
}

static void finish(void)
{
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (s_fd >= 0) { close(s_fd); s_fd = -1; }
    s_connected = false;
    s_ping_task = NULL;
    s_mode = ND_IDLE;
    xSemaphoreGive(s_mx);
}

// ── Resolución ──────────────────────────────────────────────────────────────
static bool resolve(const char *host, uint16_t port, int socktype, struct addrinfo **res)
{
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = socktype };
    char ps[8];
    snprintf(ps, sizeof(ps), "%u", port);
    return getaddrinfo(host, ps, &hints, res) == 0 && *res;
}

// ── Ping ────────────────────────────────────────────────────────────────────
static void on_ping_success(esp_ping_handle_t h, void *a)
{
    (void)a;
    uint16_t seq; uint8_t ttl; uint32_t size, ms; ip_addr_t addr;
    esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seq, sizeof(seq));
    esp_ping_get_profile(h, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(h, ESP_PING_PROF_SIZE, &size, sizeof(size));
    esp_ping_get_profile(h, ESP_PING_PROF_TIMEGAP, &ms, sizeof(ms));
    esp_ping_get_profile(h, ESP_PING_PROF_IPADDR, &addr, sizeof(addr));
    char ip[16];
    ip4addr_ntoa_r(ip_2_ip4(&addr), ip, sizeof(ip));
    buf_printf("%lu bytes de %s: seq=%u ttl=%u tiempo=%lu ms\n",
               (unsigned long)size, ip, seq, ttl, (unsigned long)ms);
}

static void on_ping_timeout(esp_ping_handle_t h, void *a)
{
    (void)a;
    uint16_t seq;
    esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seq, sizeof(seq));
    buf_printf("seq=%u: tiempo agotado\n", seq);
}

static void on_ping_end(esp_ping_handle_t h, void *a)
{
    (void)h;
    xTaskNotifyGive((TaskHandle_t)a);
}

static void ping_task(void *arg)
{
    (void)arg;
    struct addrinfo *res = NULL;
    if (!resolve(s_host, 0, SOCK_DGRAM, &res)) {
        buf_printf("No se pudo resolver %s\n", s_host);
        finish();
        vTaskDelete(NULL);
    }
    ip_addr_t target = {};
    ip_addr_set_ip4_u32(&target, ((struct sockaddr_in *)res->ai_addr)->sin_addr.s_addr);
    freeaddrinfo(res);

    char ip[16];
    ip4addr_ntoa_r(ip_2_ip4(&target), ip, sizeof(ip));
    buf_printf("PING %s (%s) %d bytes de datos\n", s_host, ip, s_ping_size);

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr = target;
    cfg.count       = (uint32_t)s_ping_count;
    cfg.data_size   = (uint32_t)s_ping_size;
    cfg.interval_ms = 1000;
    cfg.timeout_ms  = 8000;           // el enlace RF a 1200 bps es lento
    cfg.task_stack_size = 3072;
    cfg.task_prio   = 2;
    esp_ping_callbacks_t cbs = {
        .cb_args = xTaskGetCurrentTaskHandle(),
        .on_ping_success = on_ping_success,
        .on_ping_timeout = on_ping_timeout,
        .on_ping_end = on_ping_end,
    };
    esp_ping_handle_t h;
    if (esp_ping_new_session(&cfg, &cbs, &h) != ESP_OK) {
        buf_printf("Error creando la sesión de ping\n");
        finish();
        vTaskDelete(NULL);
    }
    esp_ping_start(h);
    // Techo duro por si on_ping_end no llega nunca.
    int slices = (s_ping_count * (1000 + 8000) + 5000) / 250;
    bool ended = false;
    for (int i = 0; i < slices && !s_abort; i++)
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250))) { ended = true; break; }
    if (!ended) esp_ping_stop(h);

    uint32_t tx = 0, rx = 0, dur = 0;
    esp_ping_get_profile(h, ESP_PING_PROF_REQUEST, &tx, sizeof(tx));
    esp_ping_get_profile(h, ESP_PING_PROF_REPLY, &rx, sizeof(rx));
    esp_ping_get_profile(h, ESP_PING_PROF_DURATION, &dur, sizeof(dur));
    buf_printf("--- %s: %lu enviados, %lu recibidos, %lu%% perdidos, %lu ms%s\n",
               s_host, (unsigned long)tx, (unsigned long)rx,
               (unsigned long)(tx ? (tx - rx) * 100 / tx : 100), (unsigned long)dur,
               ended ? "" : " (interrumpido)");
    esp_ping_delete_session(h);
    finish();
    vTaskDelete(NULL);
}

// ── Consola TCP/UDP ─────────────────────────────────────────────────────────
static void conn_task(void *arg)
{
    bool tcp = (s_mode == ND_TCP);
    (void)arg;
    struct addrinfo *res = NULL;
    buf_printf("Conectando (%s) a %s:%u…\n", tcp ? "TCP" : "UDP", s_host, s_port);
    if (!resolve(s_host, s_port, tcp ? SOCK_STREAM : SOCK_DGRAM, &res)) {
        buf_printf("No se pudo resolver %s\n", s_host);
        finish();
        vTaskDelete(NULL);
    }
    int fd = socket(AF_INET, tcp ? SOCK_STREAM : SOCK_DGRAM, 0);
    if (fd < 0) {
        buf_printf("Error creando socket (errno %d)\n", errno);
        freeaddrinfo(res);
        finish();
        vTaskDelete(NULL);
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    bool ok = true;
    int rc = connect(fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (rc < 0 && errno == EINPROGRESS) {
        ok = false;
        for (int i = 0; i < CONNECT_TIMEOUT_S * 4 && !s_abort; i++) {
            fd_set w; FD_ZERO(&w); FD_SET(fd, &w);
            struct timeval tv = { 0, 250000 };
            if (select(fd + 1, NULL, &w, NULL, &tv) > 0) {
                int err = 0; socklen_t el = sizeof(err);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el);
                if (err) buf_printf("Conexión fallida (errno %d)\n", err);
                else ok = true;
                goto connected;
            }
        }
        if (!s_abort) buf_printf("Tiempo de conexión agotado\n");
    } else if (rc < 0) {
        buf_printf("Conexión fallida (errno %d)\n", errno);
        ok = false;
    }
connected:
    if (!ok) { close(fd); finish(); vTaskDelete(NULL); }

    xSemaphoreTake(s_mx, portMAX_DELAY);
    s_fd = fd;
    s_connected = true;
    xSemaphoreGive(s_mx);
    buf_printf(tcp ? "Conectado.\n" : "Socket UDP listo (sin conexión real).\n");

    char rx[256];
    while (!s_abort) {
        fd_set r; FD_ZERO(&r); FD_SET(fd, &r);
        struct timeval tv = { 0, 250000 };
        if (select(fd + 1, &r, NULL, NULL, &tv) <= 0) continue;
        int n = recv(fd, rx, sizeof(rx), 0);
        if (n > 0) buf_put(rx, (size_t)n);
        else if (n == 0 && tcp) { buf_printf("[conexión cerrada por el remoto]\n"); break; }
        else if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            buf_printf("[error de recepción, errno %d]\n", errno);
            break;
        }
    }
    if (s_abort) buf_printf("[desconectado]\n");
    finish();
    vTaskDelete(NULL);
}

// ── REST ────────────────────────────────────────────────────────────────────
static cJSON *read_json(httpd_req_t *req)
{
    char body[320];
    int len = (int)req->content_len;
    if (len <= 0 || len >= (int)sizeof(body)) return NULL;
    int got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, body + got, (size_t)(len - got));
        if (r <= 0) { if (r == HTTPD_SOCK_ERR_TIMEOUT) continue; return NULL; }
        got += r;
    }
    body[got] = '\0';
    return cJSON_Parse(body);
}

static esp_err_t reply(httpd_req_t *req, const char *status, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    if (status) httpd_resp_set_status(req, status);
    return httpd_resp_sendstr(req, json);
}

static bool host_valid(const char *h)
{
    size_t n = h ? strlen(h) : 0;
    if (n == 0 || n >= HOST_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        char c = h[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              c == '.' || c == '-' || c == '_'))
            return false;
    }
    return true;
}

// Reserva la operación (si no hay otra), limpia el búfer y fija host/modo.
static bool begin_op(nd_mode_t mode, const char *host, uint16_t port)
{
    bool ok = false;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (s_mode == ND_IDLE) {
        s_mode = mode;
        s_abort = false;
        s_connected = false;
        s_total = 0;
        s_gen++;
        snprintf(s_host, sizeof(s_host), "%s", host);
        s_port = port;
        ok = true;
    }
    xSemaphoreGive(s_mx);
    return ok;
}

static esp_err_t ping_handler(httpd_req_t *req)
{
    cJSON *j = read_json(req);
    cJSON *jh = j ? cJSON_GetObjectItem(j, "host") : NULL;
    if (!cJSON_IsString(jh) || !host_valid(jh->valuestring)) {
        cJSON_Delete(j);
        return reply(req, "400 Bad Request", "{\"error\":\"host inválido\"}");
    }
    cJSON *jc = cJSON_GetObjectItem(j, "count"), *js = cJSON_GetObjectItem(j, "size");
    int count = cJSON_IsNumber(jc) ? (int)jc->valuedouble : 4;
    int size  = cJSON_IsNumber(js) ? (int)js->valuedouble : 32;
    s_ping_count = count < 1 ? 1 : (count > 10 ? 10 : count);
    s_ping_size  = size < 1 ? 1 : (size > 200 ? 200 : size);
    if (!begin_op(ND_PING, jh->valuestring, 0)) {
        cJSON_Delete(j);
        return reply(req, "409 Conflict", "{\"error\":\"hay otra operación en curso\"}");
    }
    cJSON_Delete(j);
    if (xTaskCreate(ping_task, "nd_ping", 4096, NULL, 3, &s_ping_task) != pdPASS) {
        buf_printf("Error creando la tarea\n");
        finish();
        return reply(req, "500 Internal Server Error", "{\"error\":\"task create failed\"}");
    }
    return reply(req, NULL, "{\"ok\":true}");
}

static esp_err_t connect_handler(httpd_req_t *req)
{
    cJSON *j = read_json(req);
    cJSON *jh = j ? cJSON_GetObjectItem(j, "host") : NULL;
    cJSON *jp = j ? cJSON_GetObjectItem(j, "port") : NULL;
    cJSON *jt = j ? cJSON_GetObjectItem(j, "proto") : NULL;
    int port = cJSON_IsNumber(jp) ? (int)jp->valuedouble : 0;
    if (!cJSON_IsString(jh) || !host_valid(jh->valuestring) || port < 1 || port > 65535) {
        cJSON_Delete(j);
        return reply(req, "400 Bad Request", "{\"error\":\"host o puerto inválido\"}");
    }
    bool udp = cJSON_IsString(jt) && strcmp(jt->valuestring, "udp") == 0;
    if (!begin_op(udp ? ND_UDP : ND_TCP, jh->valuestring, (uint16_t)port)) {
        cJSON_Delete(j);
        return reply(req, "409 Conflict", "{\"error\":\"hay otra operación en curso\"}");
    }
    cJSON_Delete(j);
    if (xTaskCreate(conn_task, "nd_conn", 4096, NULL, 3, NULL) != pdPASS) {
        buf_printf("Error creando la tarea\n");
        finish();
        return reply(req, "500 Internal Server Error", "{\"error\":\"task create failed\"}");
    }
    return reply(req, NULL, "{\"ok\":true}");
}

static esp_err_t send_handler(httpd_req_t *req)
{
    cJSON *j = read_json(req);
    cJSON *jd = j ? cJSON_GetObjectItem(j, "data") : NULL;
    cJSON *je = j ? cJSON_GetObjectItem(j, "eol") : NULL;
    if (!cJSON_IsString(jd)) {
        cJSON_Delete(j);
        return reply(req, "400 Bad Request", "{\"error\":\"falta data\"}");
    }
    const char *eol = "\r\n";
    if (cJSON_IsString(je)) {
        if (!strcmp(je->valuestring, "lf")) eol = "\n";
        else if (!strcmp(je->valuestring, "cr")) eol = "\r";
        else if (!strcmp(je->valuestring, "none")) eol = "";
    }
    char out[300];
    int n = snprintf(out, sizeof(out), "%s%s", jd->valuestring, eol);
    cJSON_Delete(j);
    if (n >= (int)sizeof(out)) n = sizeof(out) - 1;

    esp_err_t r = ESP_OK;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (s_fd >= 0 && s_connected) {
        int w = send(s_fd, out, (size_t)n, 0);
        if (w == n) {
            buf_put_locked("> ", 2);
            buf_put_locked(out, (size_t)n);
            if (n == 0 || out[n - 1] != '\n') buf_put_locked("\n", 1);
        } else {
            char m[48];
            int l = snprintf(m, sizeof(m), "[envío fallido, errno %d]\n", w < 0 ? errno : 0);
            buf_put_locked(m, (size_t)l);
            r = ESP_FAIL;
        }
    } else {
        r = ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(s_mx);
    if (r == ESP_ERR_INVALID_STATE) return reply(req, "409 Conflict", "{\"error\":\"sin conexión\"}");
    if (r != ESP_OK) return reply(req, "502 Bad Gateway", "{\"error\":\"envío fallido\"}");
    return reply(req, NULL, "{\"ok\":true}");
}

static esp_err_t stop_handler(httpd_req_t *req)
{
    s_abort = true;
    return reply(req, NULL, "{\"ok\":true}");
}

// GET /api/netdiag?gen=G&since=N
static esp_err_t status_handler(httpd_req_t *req)
{
    uint32_t gen_c = 0, since = 0;
    char q[48], v[12];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "gen", v, sizeof(v)) == ESP_OK) gen_c = strtoul(v, NULL, 10);
        if (httpd_query_key_value(q, "since", v, sizeof(v)) == ESP_OK) since = strtoul(v, NULL, 10);
    }

    char head[320];
    uint32_t total, from, gen;
    nd_mode_t mode;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    total = s_total; gen = s_gen; mode = s_mode;
    from = (gen_c == gen) ? since : 0;
    if (from > total) from = 0;
    if (total - from > BUF_SZ) from = total - BUF_SZ;
    int hn = snprintf(head, sizeof(head),
        "{\"mode\":\"%s\",\"connected\":%s,\"host\":\"%s\",\"port\":%u,"
        "\"gen\":%lu,\"from\":%lu,\"total\":%lu,\"text\":\"",
        MODE_NAME[mode], s_connected ? "true" : "false", s_host, s_port,
        (unsigned long)gen, (unsigned long)from, (unsigned long)total);
    xSemaphoreGive(s_mx);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, head, hn);

    char chunk[256], esc[520];
    uint32_t pos = from;
    while (pos < total) {
        size_t n = total - pos > sizeof(chunk) ? sizeof(chunk) : total - pos;
        xSemaphoreTake(s_mx, portMAX_DELAY);
        for (size_t i = 0; i < n; i++) chunk[i] = s_buf[(pos + i) % BUF_SZ];
        xSemaphoreGive(s_mx);
        size_t e = 0;
        for (size_t i = 0; i < n; i++) {
            char c = chunk[i];
            if (c == '\n') { esc[e++] = '\\'; esc[e++] = 'n'; }
            else if (c == '"' || c == '\\') { esc[e++] = '\\'; esc[e++] = c; }
            else esc[e++] = c;
        }
        httpd_resp_send_chunk(req, esc, e);
        pos += n;
    }

    // Panel de debug TX
    tx_ev_t h[TX_HIST];
    uint32_t cnt;
    portENTER_CRITICAL(&s_tx_mux);
    memcpy(h, s_tx_hist, sizeof(h));
    cnt = s_tx_count;
    portEXIT_CRITICAL(&s_tx_mux);
    int64_t now = esp_timer_get_time();
    char tail[700];
    int t = snprintf(tail, sizeof(tail),
        "\",\"tx\":{\"pending\":%d,\"dropped\":%lu,\"sent\":%lu,\"post_rx_ms\":%lu,\"recent\":[",
        afsk_tx_queue_pending(), (unsigned long)afsk_tx_dropped_count(),
        (unsigned long)cnt, (unsigned long)afsk_get_post_rx_tx_delay_ms());
    uint32_t shown = cnt < TX_HIST ? cnt : TX_HIST;
    for (uint32_t i = 0; i < shown && t < (int)sizeof(tail) - 120; i++) {
        const tx_ev_t *ev = &h[(cnt - 1 - i) % TX_HIST];
        t += snprintf(tail + t, sizeof(tail) - t,
            "%s{\"age_s\":%lu,\"len\":%u,\"src\":\"%s\",\"dst\":\"%s\",\"pid\":%d}",
            i ? "," : "", (unsigned long)((now - ev->t_us) / 1000000), ev->len,
            ev->src, ev->dst, ev->pid);
    }
    t += snprintf(tail + t, sizeof(tail) - t, "]}}");
    httpd_resp_send_chunk(req, tail, t);
    return httpd_resp_send_chunk(req, NULL, 0);
}

void netdiag_init(void)
{
    s_mx = xSemaphoreCreateMutex();
    httpd_handle_t server = audio_stream_get_httpd();
    if (!server) {
        ESP_LOGE(TAG, "httpd not ready — call after audio_stream_init()");
        return;
    }
    static const httpd_uri_t uris[] = {
        { .uri = "/api/netdiag",         .method = HTTP_GET,  .handler = status_handler },
        { .uri = "/api/netdiag/ping",    .method = HTTP_POST, .handler = ping_handler },
        { .uri = "/api/netdiag/connect", .method = HTTP_POST, .handler = connect_handler },
        { .uri = "/api/netdiag/send",    .method = HTTP_POST, .handler = send_handler },
        { .uri = "/api/netdiag/stop",    .method = HTTP_POST, .handler = stop_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        if (httpd_register_uri_handler(server, &uris[i]) != ESP_OK)
            ESP_LOGE(TAG, "register_uri %s failed", uris[i].uri);
}

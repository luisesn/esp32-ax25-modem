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
#include <stdlib.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "ping/ping_sock.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "lwip/netdb.h"
#include "lwip/ip_addr.h"

#include "audio_stream.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"

#define TAG "netdiag"

#define BUF_SZ            4096
#define HOST_MAX          64
#define CONNECT_TIMEOUT_S 30
#define TX_HIST           10
#define PING_TIMEOUT_MS   30000

typedef enum { ND_IDLE = 0, ND_PING, ND_TCP, ND_UDP } nd_mode_t;
static const char *const MODE_NAME[] = { "idle", "ping", "tcp", "udp" };

static SemaphoreHandle_t s_mx;
static char       s_buf[BUF_SZ];
static uint32_t   s_total;            // bytes escritos desde el inicio de la operación
static uint32_t   s_gen;
static nd_mode_t  s_mode;             // protegido por s_mx
static volatile bool s_abort;
static volatile bool s_connected;
static bool       s_telnet;           // TCP con negociación IAC rechazada
static volatile uint32_t s_rx_pkts, s_rx_bytes;   // recibidos en la operación actual
static int        s_fd = -1;          // protegido por s_mx
static char       s_host[HOST_MAX];
static uint16_t   s_port;
static int        s_ping_count, s_ping_size;
static TaskHandle_t s_ping_task;

// ── Historial TX (debug) ────────────────────────────────────────────────────
typedef struct { int64_t t_us; uint16_t len; char dir; int pid; char src[10], dst[10], info[64]; } tx_ev_t;
static tx_ev_t  s_tx_hist[TX_HIST];
static uint32_t s_tx_count;                    // eventos (TX+RX)
static uint32_t s_sent_count;                  // sólo TX
static portMUX_TYPE s_tx_mux = portMUX_INITIALIZER_UNLOCKED;

static void call_str(const uint8_t *a, char *out)
{
    int n = 0;
    // Las tramas IP (modo TUN) llevan bytes arbitrarios en las direcciones:
    // sólo ASCII imprimible y sin comillas/barras, para no romper el JSON.
    for (int i = 0; i < 6; i++) {
        char c = a[i] >> 1;
        if (c == ' ') continue;
        out[n++] = (c < 0x21 || c > 0x7E || c == '"' || c == '\\') ? '?' : c;
    }
    int ssid = (a[6] >> 1) & 0x0F;
    if (ssid) n += snprintf(out + n, 10 - n, "-%d", ssid);
    out[n] = '\0';
}

static const char *port_name(unsigned p)
{
    switch (p) {
    case 22: return "ssh"; case 23: return "telnet"; case 53: return "dns";
    case 80: return "http"; case 123: return "ntp"; case 443: return "https";
    case 8001: return "kiss"; default: return NULL;
    }
}

// Describe una trama recibida/transmitida: en modo TUN es "00 00 08 00 + IPv4"
// (sin cabeceras AX.25). Devuelve true si era IP; out = "tcp 54321>23 telnet SYN ACK".
bool netdiag_describe_frame(const uint8_t *d, size_t len, char *out, size_t on)
{
    out[0] = '\0';
    if (len < 24 || d[0] || d[1] || d[2] != 8 || d[3] || (d[4] >> 4) != 4) return false;
    const uint8_t *ip = d + 4;
    size_t pn = len - 4, ihl = (size_t)(ip[0] & 0x0F) * 4;
    if (ihl < 20 || pn < ihl) return false;
    uint8_t pr = ip[9];
    const uint8_t *t = ip + ihl;
    size_t tn = pn - ihl;
    if (pr == 1 && tn >= 1) {
        const char *op = t[0] == 8 ? "echo request" : t[0] == 0 ? "echo reply" :
                         t[0] == 3 ? "destino inalcanzable" : t[0] == 11 ? "TTL excedido" : NULL;
        if (op) snprintf(out, on, "icmp %s", op);
        else snprintf(out, on, "icmp tipo %u", t[0]);
    } else if ((pr == 6 || pr == 17) && tn >= 4) {
        unsigned sp = (unsigned)((t[0] << 8) | t[1]), dp = (unsigned)((t[2] << 8) | t[3]);
        const char *nm = port_name(dp); if (!nm) nm = port_name(sp);
        int k = snprintf(out, on, "%s %u>%u%s%s", pr == 6 ? "tcp" : "udp", sp, dp, nm ? " " : "", nm ? nm : "");
        if (pr == 6 && tn >= 20 && k > 0 && (size_t)k < on) {
            uint8_t fl = t[13];
            size_t dl = tn > (size_t)((t[12] >> 4) * 4) ? tn - (size_t)((t[12] >> 4) * 4) : 0;
            snprintf(out + k, on - (size_t)k, "%s%s%s%s%s %uB datos",
                     (fl & 2) ? " SYN" : "", (fl & 1) ? " FIN" : "", (fl & 4) ? " RST" : "",
                     (fl & 0x10) ? " ACK" : "", (fl & 8) ? " PSH" : "", (unsigned)dl);
        }
    } else {
        snprintf(out, on, "ip proto %u", pr);
    }
    return true;
}

static bool callsign_ok(const uint8_t *a)
{
    for (int i = 0; i < 6; i++) {
        char c = (char)(a[i] >> 1);
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || c == ' ') || (a[i] & 1)) return false;
    }
    return true;
}

static void note_ev(char dir, const uint8_t *d, size_t len)
{
    if (!d || len < 4) return;
    tx_ev_t e = { .t_us = esp_timer_get_time(), .len = (uint16_t)len, .dir = dir, .pid = -1 };
    if (!netdiag_describe_frame(d, len, e.info, sizeof(e.info))) {
        if (len >= 16 && callsign_ok(d) && callsign_ok(d + 7)) {
            call_str(d, e.dst);
            call_str(d + 7, e.src);
            size_t i = 7;
            while (i + 7 <= len && !(d[i + 6] & 1)) i += 7;
            i += 7;
            if (i + 1 < len) e.pid = d[i + 1];
        } else {
            snprintf(e.info, sizeof(e.info), "trama desconocida");
        }
    }
    portENTER_CRITICAL(&s_tx_mux);
    s_tx_hist[s_tx_count % TX_HIST] = e;
    s_tx_count++;
    if (dir == 'T') s_sent_count++;
    portEXIT_CRITICAL(&s_tx_mux);
}

void netdiag_note_tx(const uint8_t *d, size_t len) { note_ev('T', d, len); }
void netdiag_note_rx(const uint8_t *d, size_t len) { note_ev('R', d, len); }

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
    ESP_LOGI(TAG, "%.*s", n > 0 && tmp[n - 1] == '\n' ? n - 1 : n, tmp);
}

// Sólo actúa si la operación sigue siendo la del worker que llama (un stop
// forzado incrementa s_gen y deja a un worker atascado sin efecto).
static void finish(uint32_t gen)
{
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (gen != s_gen) { xSemaphoreGive(s_mx); return; }
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
// Cada sesión lleva un contexto propio. Al abortar, el worker no espera a que
// termine el ping (el timeout puede ser largo): marca el contexto como
// "detached" y es on_ping_end quien borra la sesión y libera el contexto.
typedef struct {
    uint32_t    gen;
    TaskHandle_t task;
    bool        ended, detached;
} ping_ctx_t;
static portMUX_TYPE s_ping_mux = portMUX_INITIALIZER_UNLOCKED;

static void on_ping_success(esp_ping_handle_t h, void *a)
{
    if (((ping_ctx_t *)a)->gen != s_gen) return;
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
    if (((ping_ctx_t *)a)->gen != s_gen) return;
    uint16_t seq;
    esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seq, sizeof(seq));
    buf_printf("seq=%u: tiempo agotado\n", seq);
}

static void on_ping_end(esp_ping_handle_t h, void *a)
{
    ping_ctx_t *c = a;
    portENTER_CRITICAL(&s_ping_mux);
    c->ended = true;
    bool det = c->detached;
    portEXIT_CRITICAL(&s_ping_mux);
    if (det) {
        esp_ping_delete_session(h);
        free(c);
    } else {
        xTaskNotifyGive(c->task);
    }
}

static void ping_task(void *arg)
{
    (void)arg;
    const uint32_t gen = s_gen;
    struct addrinfo *res = NULL;
    if (!resolve(s_host, 0, SOCK_DGRAM, &res)) {
        buf_printf("No se pudo resolver %s\n", s_host);
        finish(gen);
        vTaskDelete(NULL);
    }
    ip_addr_t target = {};
    ip_addr_set_ip4_u32(&target, ((struct sockaddr_in *)res->ai_addr)->sin_addr.s_addr);
    freeaddrinfo(res);

    char ip[16];
    ip4addr_ntoa_r(ip_2_ip4(&target), ip, sizeof(ip));
    buf_printf("PING %s (%s) %d bytes de datos\n", s_host, ip, s_ping_size);

    ping_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) { buf_printf("Sin memoria\n"); finish(gen); vTaskDelete(NULL); }
    ctx->gen = s_gen;
    ctx->task = xTaskGetCurrentTaskHandle();

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr = target;
    cfg.count       = (uint32_t)s_ping_count;
    cfg.data_size   = (uint32_t)s_ping_size;
    cfg.interval_ms = 1000;
    cfg.timeout_ms  = PING_TIMEOUT_MS;  // el enlace RF a 1200 bps es lento
    cfg.task_stack_size = 3072;
    cfg.task_prio   = 2;
    esp_ping_callbacks_t cbs = {
        .cb_args = ctx,
        .on_ping_success = on_ping_success,
        .on_ping_timeout = on_ping_timeout,
        .on_ping_end = on_ping_end,
    };
    esp_ping_handle_t h;
    if (esp_ping_new_session(&cfg, &cbs, &h) != ESP_OK) {
        buf_printf("Error creando la sesion de ping\n");
        free(ctx);
        finish(gen);
        vTaskDelete(NULL);
    }
    esp_ping_start(h);
    // Techo duro por si on_ping_end no llega nunca.
    int slices = (s_ping_count * (1000 + PING_TIMEOUT_MS) + 5000) / 250;
    bool ended = false;
    for (int i = 0; i < slices && !s_abort; i++)
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250))) { ended = true; break; }

    uint32_t tx = 0, rx = 0, dur = 0;
    esp_ping_get_profile(h, ESP_PING_PROF_REQUEST, &tx, sizeof(tx));
    esp_ping_get_profile(h, ESP_PING_PROF_REPLY, &rx, sizeof(rx));
    esp_ping_get_profile(h, ESP_PING_PROF_DURATION, &dur, sizeof(dur));
    if (ended) {
        buf_printf("--- %s: %lu enviados, %lu recibidos, %lu%% perdidos, %lu ms\n",
                   s_host, (unsigned long)tx, (unsigned long)rx,
                   (unsigned long)(tx ? (tx - rx) * 100 / tx : 100), (unsigned long)dur);
    } else {
        buf_printf("--- %s: interrumpido (%lu enviados, %lu recibidos)\n",
                   s_host, (unsigned long)tx, (unsigned long)rx);
        esp_ping_stop(h);
    }

    // Si la sesión ya terminó la borramos aquí; si no, lo hará on_ping_end.
    portENTER_CRITICAL(&s_ping_mux);
    bool done = ctx->ended;
    if (!done) ctx->detached = true;
    portEXIT_CRITICAL(&s_ping_mux);
    if (done) {
        esp_ping_delete_session(h);
        free(ctx);
    }
    finish(gen);
    vTaskDelete(NULL);
}

// ── Filtro Telnet ───────────────────────────────────────────────────────────
// Elimina las secuencias IAC del flujo y rechaza toda opción (DO→WONT,
// WILL→DONT): es lo más barato por un enlace de 1200 bps.
typedef struct { uint8_t st, verb; } tn_t;

static size_t telnet_filter(int fd, tn_t *t, uint8_t *b, size_t n)
{
    size_t o = 0;
    uint8_t rb[96];                               // respuestas agrupadas: un solo send()
    size_t rn = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = b[i];
        switch (t->st) {
        case 0:
            if (c == 255) t->st = 1; else b[o++] = c;
            break;
        case 1:                                   // tras IAC
            if (c == 255) { b[o++] = 255; t->st = 0; }
            else if (c >= 251 && c <= 254) { t->verb = c; t->st = 2; }
            else if (c == 250) t->st = 3;         // SB
            else t->st = 0;                       // NOP, GA…
            break;
        case 2: {                                 // opción de WILL/WONT/DO/DONT
            uint8_t rsp[3] = { 255, 0, c };
            if (t->verb == 253) rsp[1] = 252;       // DO   → WONT
            else if (t->verb == 251) rsp[1] = 254;  // WILL → DONT
            if (rsp[1] && rn + 3 <= sizeof(rb)) { memcpy(rb + rn, rsp, 3); rn += 3; }
            t->st = 0;
            break; }
        case 3: if (c == 255) t->st = 4; break;   // dentro de SB
        case 4: t->st = (c == 240) ? 0 : 3; break; // IAC SE termina SB
        }
    }
    if (rn) send(fd, rb, rn, 0);
    return o;
}

// ── Consola TCP/UDP ─────────────────────────────────────────────────────────
static void conn_task(void *arg)
{
    bool tcp = (s_mode == ND_TCP);
    const uint32_t gen = s_gen;
    (void)arg;
    struct addrinfo *res = NULL;
    buf_printf("Conectando (%s) a %s:%u ...\n", !tcp ? "UDP" : (s_telnet ? "TELNET" : "TCP"), s_host, s_port);
    if (!resolve(s_host, s_port, tcp ? SOCK_STREAM : SOCK_DGRAM, &res)) {
        buf_printf("No se pudo resolver %s\n", s_host);
        finish(gen);
        vTaskDelete(NULL);
    }
    int fd = socket(AF_INET, tcp ? SOCK_STREAM : SOCK_DGRAM, 0);
    if (fd < 0) {
        buf_printf("Error creando socket (errno %d)\n", errno);
        freeaddrinfo(res);
        finish(gen);
        vTaskDelete(NULL);
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    bool ok = true;
    int rc = connect(fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (rc < 0 && errno == EINPROGRESS) {
        ok = false;
        for (int i = 0; i < CONNECT_TIMEOUT_S * 4 && !s_abort; i++) {
            // lwIP sólo señala los errores (RST, ICMP) por exceptfds.
            fd_set w, x; FD_ZERO(&w); FD_ZERO(&x); FD_SET(fd, &w); FD_SET(fd, &x);
            struct timeval tv = { 0, 250000 };
            if (select(fd + 1, NULL, &w, &x, &tv) > 0) {
                int err = 0; socklen_t el = sizeof(err);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el);
                if (err) buf_printf("Conexion fallida: %s (errno %d)\n", strerror(err), err);
                else ok = true;
                goto connected;
            }
        }
        if (!s_abort) buf_printf("Tiempo de conexion agotado\n");
    } else if (rc < 0) {
        buf_printf("Conexion fallida: %s (errno %d)\n", strerror(errno), errno);
        ok = false;
    }
connected:
    if (!ok) { close(fd); finish(gen); vTaskDelete(NULL); }

    xSemaphoreTake(s_mx, portMAX_DELAY);
    s_fd = fd;
    s_connected = true;
    xSemaphoreGive(s_mx);
    buf_printf(tcp ? "Conectado.\n" : "Socket UDP listo (sin conexion real).\n");

    if (tcp) {
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    }
    char rx[256];
    tn_t tn = {0, 0};
    const bool telnet = tcp && s_telnet;
    while (!s_abort) {
        fd_set r, x; FD_ZERO(&r); FD_ZERO(&x); FD_SET(fd, &r); FD_SET(fd, &x);
        struct timeval tv = { 0, 250000 };
        if (select(fd + 1, &r, NULL, &x, &tv) <= 0) continue;
        int n = recv(fd, rx, sizeof(rx), 0);
        if (n > 0 && telnet) {
            n = (int)telnet_filter(fd, &tn, (uint8_t *)rx, (size_t)n);
            if (n == 0) continue;               // sólo negociación: no es un cierre
        }
        if (n > 0) {
            s_rx_pkts++;
            s_rx_bytes += (uint32_t)n;
            ESP_LOGI(TAG, "rx %d bytes (%02x %02x %02x ...)", n, rx[0],
                     n > 1 ? rx[1] : 0, n > 2 ? rx[2] : 0);
            if (!tcp) buf_put("< ", 2);             // cada datagrama en su línea
            buf_put(rx, (size_t)n);
            if (!tcp && rx[n - 1] != '\n') buf_put("\n", 1);
        }
        else if (n == 0 && tcp) { buf_printf("[conexion cerrada por el remoto]\n"); break; }
        else if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            buf_printf("[error de recepcion: %s (errno %d)]\n", strerror(errno), errno);
            break;
        }
    }
    if (s_abort) buf_printf("[desconectado]\n");
    finish(gen);
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
        s_rx_pkts = s_rx_bytes = 0;
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
    int count = cJSON_IsNumber(jc) ? (int)jc->valuedouble : 1;
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
        finish(s_gen);
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
    bool telnet = cJSON_IsString(jt) && strcmp(jt->valuestring, "telnet") == 0;
    if (!begin_op(udp ? ND_UDP : ND_TCP, jh->valuestring, (uint16_t)port)) {
        cJSON_Delete(j);
        return reply(req, "409 Conflict", "{\"error\":\"hay otra operación en curso\"}");
    }
    s_telnet = telnet;   // el worker aún no ha arrancado
    cJSON_Delete(j);
    if (xTaskCreate(conn_task, "nd_conn", 4096, NULL, 3, NULL) != pdPASS) {
        buf_printf("Error creando la tarea\n");
        finish(s_gen);
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
            // Eco local sin prefijo.
            bool nl = n > 0 && out[n - 1] == '\n';
            size_t vis = (size_t)n;
            if (nl) vis--;
            if (vis > 0 && out[vis - 1] == '\r') vis--;
            buf_put_locked(out, vis);
            if (nl || n == 0) buf_put_locked("\n", 1);
        } else {
            char m[80];
            int e = w < 0 ? errno : 0;
            int l = snprintf(m, sizeof(m), "[envio fallido: %s (errno %d)]\n", e ? strerror(e) : "parcial", e);
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
    // Los workers comprueban s_abort cada 250 ms; si tras 2 s sigue ocupada
    // (p. ej. atascado en DNS) se fuerza el reinicio del estado.
    for (int i = 0; i < 20 && s_mode != ND_IDLE; i++) vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (s_mode != ND_IDLE) {
        ESP_LOGW(TAG, "stop: operacion atascada, forzando reinicio");
        if (s_fd >= 0) { close(s_fd); s_fd = -1; }
        s_connected = false;
        s_mode = ND_IDLE;
        s_gen++;
        s_total = 0;
    }
    xSemaphoreGive(s_mx);
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
        "{\"mode\":\"%s\",\"telnet\":%s,\"connected\":%s,\"host\":\"%s\",\"port\":%u,"
        "\"rx_pkts\":%lu,\"rx_bytes\":%lu,"
        "\"gen\":%lu,\"from\":%lu,\"total\":%lu,\"text\":\"",
        MODE_NAME[mode], s_telnet ? "true" : "false",
        s_connected ? "true" : "false", s_host, s_port,
        (unsigned long)s_rx_pkts, (unsigned long)s_rx_bytes,
        (unsigned long)gen, (unsigned long)from, (unsigned long)total);
    xSemaphoreGive(s_mx);

    httpd_resp_set_type(req, "application/json");
    esp_err_t se = httpd_resp_send_chunk(req, head, hn);

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
        if (se == ESP_OK) se = httpd_resp_send_chunk(req, esc, e);
        pos += n;
    }

    // Panel de debug TX
    tx_ev_t h[TX_HIST];
    uint32_t cnt, sent;
    portENTER_CRITICAL(&s_tx_mux);
    memcpy(h, s_tx_hist, sizeof(h));
    cnt = s_tx_count;
    sent = s_sent_count;
    portEXIT_CRITICAL(&s_tx_mux);
    int64_t now = esp_timer_get_time();
    char tail[1500];
    int t = snprintf(tail, sizeof(tail),
        "\",\"tx\":{\"pending\":%d,\"dropped\":%lu,\"sent\":%lu,\"post_rx_ms\":%lu,\"recent\":[",
        afsk_tx_queue_pending(), (unsigned long)afsk_tx_dropped_count(),
        (unsigned long)sent, (unsigned long)afsk_get_post_rx_tx_delay_ms());
    uint32_t shown = cnt < TX_HIST ? cnt : TX_HIST;
    for (uint32_t i = 0; i < shown && t < (int)sizeof(tail) - 120; i++) {
        const tx_ev_t *ev = &h[(cnt - 1 - i) % TX_HIST];
        t += snprintf(tail + t, sizeof(tail) - t,
            "%s{\"age_s\":%lu,\"dir\":\"%c\",\"len\":%u,\"src\":\"%s\",\"dst\":\"%s\",\"info\":\"%s\",\"pid\":%d}",
            i ? "," : "", (unsigned long)((now - ev->t_us) / 1000000), ev->dir, ev->len,
            ev->src, ev->dst, ev->info, ev->pid);
    }
    t += snprintf(tail + t, sizeof(tail) - t, "]}}");
    if (se == ESP_OK) se = httpd_resp_send_chunk(req, tail, t);
    if (se == ESP_OK) se = httpd_resp_send_chunk(req, NULL, 0);
    if (se != ESP_OK)
        ESP_LOGW(TAG, "status: envio fallido (%s), heap libre %u, minimo %u",
                 esp_err_to_name(se), (unsigned)esp_get_free_heap_size(),
                 (unsigned)esp_get_minimum_free_heap_size());
    return se;
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

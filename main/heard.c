#include "heard.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"

#define HEARD_MAX       20
#define HEARD_CONSOLE   10

typedef struct {
    char     call[10];     // "CALL-SS"
    uint32_t last_s;       // segundos desde arranque
    uint32_t count;
    uint32_t q_sum;
    uint32_t q_cnt;
    bool     has_pos;
    float    lat, lon;
    bool     used;
} heard_t;

static heard_t s_tab[HEARD_MAX];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void heard_init(void)
{
    portENTER_CRITICAL(&s_mux);
    memset(s_tab, 0, sizeof(s_tab));
    portEXIT_CRITICAL(&s_mux);
}

static uint32_t now_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

static bool digits(const uint8_t *p, int n)
{
    for (int i = 0; i < n; i++) if (p[i] < '0' || p[i] > '9') return false;
    return true;
}

// "DDMM.mmN" / "DDDMM.mmE" sin comprimir. p apunta a la latitud.
static bool parse_pos(const uint8_t *p, size_t n, float *lat, float *lon)
{
    if (n < 18) return false;
    if (!digits(p, 4) || p[4] != '.' || !digits(p + 5, 2) || (p[7] != 'N' && p[7] != 'S')) return false;
    if (!digits(p + 9, 5) || p[14] != '.' || !digits(p + 15, 2) || (p[17] != 'E' && p[17] != 'W')) return false;
    float la = (p[0] - '0') * 10 + (p[1] - '0') + ((p[2] - '0') * 10 + (p[3] - '0') + ((p[5] - '0') * 10 + (p[6] - '0')) / 100.0f) / 60.0f;
    float lo = (p[9] - '0') * 100 + (p[10] - '0') * 10 + (p[11] - '0') +
               ((p[12] - '0') * 10 + (p[13] - '0') + ((p[15] - '0') * 10 + (p[16] - '0')) / 100.0f) / 60.0f;
    if (la > 90.0f || lo > 180.0f) return false;
    *lat = (p[7] == 'S') ? -la : la;
    *lon = (p[17] == 'W') ? -lo : lo;
    return true;
}

void heard_note(const uint8_t *ax25, size_t len, int q)
{
    if (!ax25 || len < 16) return;

    char call[10];
    int k = 0;
    for (int i = 7; i < 13; i++) {
        char ch = (char)(ax25[i] >> 1);
        if (ch != ' ' && k < 6) call[k++] = ch;
    }
    if (k == 0) return;
    int ssid = (ax25[13] >> 1) & 0x0F;
    if (ssid) k += snprintf(call + k, sizeof(call) - (size_t)k, "-%d", ssid);
    else call[k] = '\0';

    // Fin de direcciones: primer byte con bit0 = 1 desde el origen.
    size_t i = 13;
    while (i < len && !(ax25[i] & 1)) i += 7;
    i++;                       // al control
    i += 2;                    // control + PID
    bool pos_ok = false;
    float lat = 0, lon = 0;
    if (i < len) {
        const uint8_t *inf = ax25 + i;
        size_t il = len - i;
        if (inf[0] == '!' || inf[0] == '=')
            pos_ok = parse_pos(inf + 1, il - 1, &lat, &lon);
        else if ((inf[0] == '/' || inf[0] == '@') && il > 8)
            pos_ok = parse_pos(inf + 8, il - 8, &lat, &lon);
    }

    uint32_t t = now_s();
    portENTER_CRITICAL(&s_mux);
    heard_t *e = NULL;
    for (int j = 0; j < HEARD_MAX; j++)
        if (s_tab[j].used && strcmp(s_tab[j].call, call) == 0) { e = &s_tab[j]; break; }
    if (!e) {
        // elegir hueco libre o la más antigua
        heard_t *cand = NULL;
        for (int j = 0; j < HEARD_MAX; j++) if (!s_tab[j].used) { cand = &s_tab[j]; break; }
        if (!cand) {
            cand = &s_tab[0];
            for (int j = 1; j < HEARD_MAX; j++) if (s_tab[j].last_s < cand->last_s) cand = &s_tab[j];
        }
        e = cand;
        memset(e, 0, sizeof(*e));
        memcpy(e->call, call, sizeof(e->call));
        e->used = true;
    }
    e->last_s = t;
    e->count++;
    if (q > 0) { e->q_sum += (uint32_t)q; e->q_cnt++; }
    if (pos_ok) { e->has_pos = true; e->lat = lat; e->lon = lon; }
    portEXIT_CRITICAL(&s_mux);
}

// Copia ordenada (más reciente primero) fuera de la sección crítica.
static int snapshot(heard_t *out, int max)
{
    heard_t tmp[HEARD_MAX];
    portENTER_CRITICAL(&s_mux);
    memcpy(tmp, s_tab, sizeof(tmp));
    portEXIT_CRITICAL(&s_mux);
    int n = 0;
    bool taken[HEARD_MAX] = {0};
    while (n < max) {
        int best = -1;
        for (int j = 0; j < HEARD_MAX; j++)
            if (tmp[j].used && !taken[j] && (best < 0 || tmp[j].last_s > tmp[best].last_s)) best = j;
        if (best < 0) break;
        taken[best] = true;
        out[n++] = tmp[best];
    }
    return n;
}

int heard_list(char *buf, size_t n)
{
    if (!buf || n == 0) return 0;
    heard_t h[HEARD_CONSOLE];
    int cnt = snapshot(h, HEARD_CONSOLE);
    uint32_t t = now_s();
    size_t o = 0;
    buf[0] = '\0';
    if (cnt == 0) return snprintf(buf, n, "No stations heard\r\n");
    for (int i = 0; i < cnt && o < n; i++) {
        uint32_t age = t - h[i].last_s;
        char qs[6] = "-";
        if (h[i].q_cnt) snprintf(qs, sizeof(qs), "%u.%u", (unsigned)(h[i].q_sum / h[i].q_cnt),
                                 (unsigned)((h[i].q_sum * 10 / h[i].q_cnt) % 10));
        int w = snprintf(buf + o, n - o, "%-9s %4lus n%-3lu q%s%s\r\n", h[i].call,
                         (unsigned long)age, (unsigned long)h[i].count, qs, h[i].has_pos ? " P" : "");
        if (w < 0 || (size_t)w >= n - o) { buf[o] = '\0'; break; }
        o += (size_t)w;
    }
    return (int)o;
}

int heard_json(char *buf, size_t n)
{
    if (!buf || n < 16) return -1;
    heard_t h[HEARD_MAX];
    int cnt = snapshot(h, HEARD_MAX);
    uint32_t t = now_s();
    size_t o = 0;
    int w = snprintf(buf, n, "{\"heard\":[");
    if (w < 0 || (size_t)w >= n) return -1;
    o = (size_t)w;
    for (int i = 0; i < cnt; i++) {
        w = snprintf(buf + o, n - o, "%s{\"call\":\"%s\",\"age\":%lu,\"n\":%lu", i ? "," : "",
                     h[i].call, (unsigned long)(t - h[i].last_s), (unsigned long)h[i].count);
        if (w < 0 || (size_t)w >= n - o) return -1;
        o += (size_t)w;
        if (h[i].q_cnt) {
            w = snprintf(buf + o, n - o, ",\"q\":%.1f", (double)h[i].q_sum / h[i].q_cnt);
            if (w < 0 || (size_t)w >= n - o) return -1;
            o += (size_t)w;
        }
        if (h[i].has_pos) {
            w = snprintf(buf + o, n - o, ",\"lat\":%.5f,\"lon\":%.5f", (double)h[i].lat, (double)h[i].lon);
            if (w < 0 || (size_t)w >= n - o) return -1;
            o += (size_t)w;
        }
        if (o + 2 >= n) return -1;
        buf[o++] = '}';
        buf[o] = '\0';
    }
    w = snprintf(buf + o, n - o, "]}");
    if (w < 0 || (size_t)w >= n - o) return -1;
    return (int)(o + (size_t)w);
}

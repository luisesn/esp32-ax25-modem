#include "rx_survey.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_stream.h"
#include "rx_stats.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"   // afsk_v2_agc_peak

static const char *TAG = "rx_survey";

// ── Parámetros de medida ─────────────────────────────────────────────────────
#define SV_N          128      // muestras por bloque (13,3 ms @ 9600 Hz)
#define SV_BLOCKS     8        // bloques por ventana (≈107 ms)
#define SV_FS         9600.0f
#define SV_NB         9        // bins Goertzel

// 0:1200  1:2200  2..4: banda media (700/1700/2700)  5..8: HFNE (3150–4350)
static const float s_freq[SV_NB] = {
    1200.f, 2200.f, 700.f, 1700.f, 2700.f, 3150.f, 3525.f, 3975.f, 4350.f
};

#define RING_LEN      64
#define HIST_BINS     32
#define HIST_CLASSES  4        // 0 sin marca, 1 libre, 2 ocupado, 3 ventana con trama
#define HIST_METRICS  4        // pico, rms, hf dB, tonos dB

typedef struct {
    uint32_t t_ms;
    float    p12, p22, pmid, phf;   // potencia media (amplitud² en cuentas²)
    uint32_t sumsq;
    int32_t  sum;
    uint16_t n;
    uint16_t agc;
    uint16_t frame_len;             // longitud de la última trama CRC-ok de la ventana
    uint8_t  peak, clip, fr, fl, fe, mark;
} sv_row_t;

// ── Estado ───────────────────────────────────────────────────────────────────
static volatile bool    s_active   = false;
static volatile bool    s_reset    = false;
static volatile int     s_mark     = 0;
static volatile uint32_t s_note_len = 0;
static volatile uint32_t s_dropped  = 0;
static volatile uint32_t s_rows     = 0;

static sv_row_t *s_ring = NULL;
static volatile uint32_t s_head = 0;   // productor (hook)
static volatile uint32_t s_tail = 0;   // consumidor (tarea)

static uint16_t (*s_hist)[HIST_METRICS][HIST_BINS] = NULL;   // [clase][métrica][bin]
static TaskHandle_t s_task = NULL;
static volatile uint32_t s_t0_ms = 0;

// Estado del hook (solo receive_audio_task)
static float    s_coeff[SV_NB];
static float    s_s1[SV_NB], s_s2[SV_NB];
static float    s_pw[SV_NB];
static int      s_pos, s_blk;
static int32_t  s_sum;
static uint32_t s_sumsq;
static uint16_t s_cnt;
static uint8_t  s_peak, s_clip;
static uint32_t s_prev_ok, s_prev_err, s_prev_flags;

static void reset_hook_state(void)
{
    memset(s_s1, 0, sizeof(s_s1));
    memset(s_s2, 0, sizeof(s_s2));
    memset(s_pw, 0, sizeof(s_pw));
    s_pos = 0; s_blk = 0; s_sum = 0; s_sumsq = 0; s_cnt = 0;
    s_peak = 0; s_clip = 0;
    s_prev_ok    = rx_stats_v2.frames_crc_ok;
    s_prev_err   = rx_stats_v2.frames_crc_err;
    s_prev_flags = rx_stats_v2.hdlc_flags;
    s_note_len = 0;
}

void rx_survey_init(void)
{
    for (int i = 0; i < SV_NB; i++)
        s_coeff[i] = 2.0f * cosf(2.0f * (float)M_PI * s_freq[i] / SV_FS);
}

static inline uint8_t sat8(uint32_t v) { return v > 255 ? 255 : (uint8_t)v; }

static void finish_window(void)
{
    uint32_t head = s_head;
    if ((uint32_t)(head - s_tail) >= RING_LEN) { s_dropped++; goto reset; }

    sv_row_t *r = &s_ring[head % RING_LEN];
    const float norm = (float)SV_BLOCKS * ((float)SV_N * (float)SV_N / 4.0f);
    r->t_ms  = (uint32_t)(esp_timer_get_time() / 1000);
    r->p12   = s_pw[0] / norm;
    r->p22   = s_pw[1] / norm;
    r->pmid  = (s_pw[2] + s_pw[3] + s_pw[4]) / (3.0f * norm);
    r->phf   = (s_pw[5] + s_pw[6] + s_pw[7] + s_pw[8]) / (4.0f * norm);
    r->sumsq = s_sumsq;
    r->sum   = s_sum;
    r->n     = s_cnt;
    r->agc   = (uint16_t)(afsk_v2_agc_peak > 65535 ? 65535 : afsk_v2_agc_peak);
    r->peak  = s_peak;
    r->clip  = s_clip;

    uint32_t ok = rx_stats_v2.frames_crc_ok;
    uint32_t er = rx_stats_v2.frames_crc_err;
    uint32_t fl = rx_stats_v2.hdlc_flags;
    r->fr = sat8(ok - s_prev_ok);
    r->fe = sat8(er - s_prev_err);
    r->fl = sat8(fl - s_prev_flags);
    s_prev_ok = ok; s_prev_err = er; s_prev_flags = fl;

    uint32_t fb = s_note_len; s_note_len = 0;
    r->frame_len = (uint16_t)(fb > 65535 ? 65535 : fb);
    r->mark = (uint8_t)s_mark;

    s_head = head + 1;   // publicar
    s_rows++;

reset:
    memset(s_pw, 0, sizeof(s_pw));
    s_blk = 0; s_sum = 0; s_sumsq = 0; s_cnt = 0; s_peak = 0; s_clip = 0;
}

void rx_survey_push_sample(int8_t sample)
{
    if (!s_active) return;
    if (s_reset) { s_reset = false; reset_hook_state(); }

    int32_t x = sample;
    int32_t a = x < 0 ? -x : x;
    if (a > s_peak) s_peak = (uint8_t)(a > 255 ? 255 : a);
    if (a >= 126 && s_clip < 255) s_clip++;
    s_sum   += x;
    s_sumsq += (uint32_t)(x * x);
    s_cnt++;

    float fx = (float)x;
    for (int i = 0; i < SV_NB; i++) {
        float s0 = s_coeff[i] * s_s1[i] - s_s2[i] + fx;
        s_s2[i] = s_s1[i];
        s_s1[i] = s0;
    }

    if (++s_pos < SV_N) return;
    s_pos = 0;

    for (int i = 0; i < SV_NB; i++) {
        float p = s_s1[i] * s_s1[i] + s_s2[i] * s_s2[i] - s_coeff[i] * s_s1[i] * s_s2[i];
        s_pw[i] += p < 0.0f ? 0.0f : p;
        s_s1[i] = 0.0f; s_s2[i] = 0.0f;
    }
    if (++s_blk >= SV_BLOCKS) finish_window();
}

void rx_survey_note_frame(size_t len)
{
    if (s_active) s_note_len = (uint32_t)len;
}

// ── Tarea: serializa filas a WS y rellena histogramas ───────────────────────

static inline float to_db(float p) { return 10.0f * log10f(p + 1e-3f); }

static inline int bin_db(float db)
{
    int b = (int)((db + 10.0f) * (float)HIST_BINS / 70.0f);
    return b < 0 ? 0 : (b >= HIST_BINS ? HIST_BINS - 1 : b);
}

static inline void hist_add(int cls, int metric, int bin)
{
    uint16_t *c = &s_hist[cls][metric][bin];
    if (*c < 65535) (*c)++;
}

static void emit_row(const sv_row_t *r)
{
    float rms  = r->n ? sqrtf((float)r->sumsq / (float)r->n) : 0.0f;
    float dc   = r->n ? (float)r->sum / (float)r->n : 0.0f;
    float d12  = to_db(r->p12), d22 = to_db(r->p22);
    float dmid = to_db(r->pmid), dhf = to_db(r->phf);
    float dtone = to_db(r->p12 + r->p22);

    int cls = (r->fr || r->fl) ? 3 : (r->mark == 1 ? 1 : (r->mark == 2 ? 2 : 0));
    int bpk  = r->peak >> 2;            if (bpk > HIST_BINS - 1) bpk = HIST_BINS - 1;
    int brms = (int)(rms / 2.0f);       if (brms > HIST_BINS - 1) brms = HIST_BINS - 1;
    hist_add(cls, 0, bpk);
    hist_add(cls, 1, brms);
    hist_add(cls, 2, bin_db(dhf));
    hist_add(cls, 3, bin_db(dtone));

    char buf[300];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"survey\",\"t\":%lu,\"pk\":%u,\"rms\":%.1f,\"dc\":%.1f,\"clip\":%u,"
        "\"d12\":%.1f,\"d22\":%.1f,\"dm\":%.1f,\"hf\":%.1f,\"agc\":%u,"
        "\"fr\":%u,\"fl\":%u,\"fe\":%u,\"fb\":%u,\"mk\":%u,\"dr\":%lu}",
        (unsigned long)r->t_ms, r->peak, (double)rms, (double)dc, r->clip,
        (double)d12, (double)d22, (double)dmid, (double)dhf, r->agc,
        r->fr, r->fl, r->fe, r->frame_len, r->mark, (unsigned long)s_dropped);
    audio_stream_ws_send_text(buf);
}

static void survey_task(void *arg)
{
    (void)arg;
    for (;;) {
        while (s_tail != s_head) {
            sv_row_t row = s_ring[s_tail % RING_LEN];   // copia antes de liberar el hueco
            s_tail++;
            emit_row(&row);
        }
        if (!s_active) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Drenar lo último que pudiera haber quedado y liberar el anillo.
    while (s_tail != s_head) {
        sv_row_t row = s_ring[s_tail % RING_LEN];
        s_tail++;
        emit_row(&row);
    }
    sv_row_t *ring = s_ring;
    s_ring = NULL;
    free(ring);
    s_task = NULL;
    ESP_LOGI(TAG, "survey detenido (%lu filas, %lu descartadas)",
             (unsigned long)s_rows, (unsigned long)s_dropped);
    vTaskDelete(NULL);
}

// ── Control ──────────────────────────────────────────────────────────────────

bool rx_survey_start(void)
{
    if (s_active || s_task) return false;       // ya activo o aún cerrando
    if (!s_hist) {
        s_hist = calloc(HIST_CLASSES, sizeof(*s_hist));
        if (!s_hist) return false;
    } else {
        memset(s_hist, 0, HIST_CLASSES * sizeof(*s_hist));
    }
    s_ring = calloc(RING_LEN, sizeof(sv_row_t));
    if (!s_ring) return false;

    s_head = s_tail = 0;
    s_rows = 0;
    s_dropped = 0;
    s_mark = 0;
    s_t0_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_reset = true;                             // el hook inicializa su estado
    s_active = true;

    if (xTaskCreate(survey_task, "rx_survey", 4096, NULL, 3, &s_task) != pdPASS) {
        s_active = false;
        free(s_ring);
        s_ring = NULL;
        s_task = NULL;
        return false;
    }
    ESP_LOGI(TAG, "survey iniciado");
    return true;
}

void rx_survey_stop(void)        { s_active = false; }
bool rx_survey_is_active(void)   { return s_active; }

void rx_survey_set_mark(int mark)
{
    s_mark = (mark < 0 || mark > 2) ? 0 : mark;
}
int rx_survey_get_mark(void)     { return s_mark; }

// ── Estado JSON ──────────────────────────────────────────────────────────────

char *rx_survey_status_json(void)
{
    const size_t cap = 4608;
    char *out = malloc(cap);
    if (!out) return NULL;
    size_t n = 0;
    #define APP(...) do { if (n < cap) n += (size_t)snprintf(out + n, cap - n, __VA_ARGS__); } while (0)

    APP("{\"active\":%s,\"mark\":%d,\"rows\":%lu,\"dropped\":%lu,\"t0\":%lu,\"bins\":%d,"
        "\"classes\":[\"sin_marca\",\"libre\",\"ocupado\",\"trama\"],",
        s_active ? "true" : "false", s_mark, (unsigned long)s_rows,
        (unsigned long)s_dropped, (unsigned long)s_t0_ms, HIST_BINS);

    static const char *names[HIST_METRICS] = { "pk", "rms", "hf", "tone" };
    APP("\"hist\":");
    if (!s_hist) {
        APP("null");
    } else {
        APP("{");
        for (int m = 0; m < HIST_METRICS; m++) {
            APP("%s\"%s\":[", m ? "," : "", names[m]);
            for (int c = 0; c < HIST_CLASSES; c++) {
                APP("%s[", c ? "," : "");
                for (int b = 0; b < HIST_BINS; b++)
                    APP("%s%u", b ? "," : "", (unsigned)s_hist[c][m][b]);
                APP("]");
            }
            APP("]");
        }
        APP("}");
    }
    APP("}");
    #undef APP
    if (n >= cap) { free(out); return NULL; }   // no debería ocurrir: cap con margen
    return out;
}

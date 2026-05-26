#include "squelch_sf.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_stream.h"

static const char *TAG = "squelch_sf";

// ── Goertzel — HFNE bins (FM noise band, above audio) ────────────────────────
// N=128 samples at 9600 Hz → 13.3 ms per block, ~75 blocks/s
// Bins: 3150, 3525, 3975, 4350 Hz → k = 42, 47, 53, 58
// FM discriminator noise is HIGH here with no carrier; DROPS when any carrier arrives.

#define SF_N       128
#define HFNE_NBINS 4

static const int s_k[HFNE_NBINS] = {42, 47, 53, 58};

static float   s_coeff[HFNE_NBINS];
static int32_t s_s1[HFNE_NBINS];
static int32_t s_s2[HFNE_NBINS];
static int32_t s_buf_pos = 0;

// ── HFNE state ────────────────────────────────────────────────────────────────
// Normalised to [0,1] via a slow noise-floor EMA:
//   hfne ≈ 1.0 → power equals noise floor → no carrier → closed
//   hfne ≈ 0.0 → power well below noise floor → carrier present → open
static volatile float s_hfne      = 1.0f;
static volatile bool  s_open      = false;
static float          s_thr       = 0.1f;

// Noise-floor tracker — only updates when signal looks like noise (raw > 70% of floor)
// so a carrier cannot pull the floor down.
static float s_floor = -1.0f;          // negative = not yet initialised
#define HFNE_FLOOR_ALPHA 0.02f         // ~50 blocks ≈ 0.67 s time constant

// Close-hold: once closed, stay closed for at least SQUELCH_HOLD_MS.
// Prevents rapid re-open after a brief signal dropout.
#define SQUELCH_HOLD_MS 500
static TickType_t s_hold_until = 0;

// Activation sources
static volatile bool s_rep_active = false;   // set by repeater_set_enabled()
static volatile bool s_mon_active = false;   // set by UI monitor switch / REST

// Flags consumed by push_sample (always in receive_audio_task context).
// Avoids calling reset_state()/send_ws() from httpd handler, which would
// race on Goertzel state and block the httpd task on a WebSocket send.
static volatile bool s_reset_needed  = false;
static volatile bool s_send_ws_once  = false;

// WS throttle
static uint32_t s_block_count = 0;
#define WS_EVERY_BLOCKS 8              // ~100 ms

// ── helpers ───────────────────────────────────────────────────────────────────

static void send_ws(void)
{
    char buf[100];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"squelch_sf\",\"hfne\":%.3f,\"open\":%s,\"active\":%s,\"manual\":%s}",
             (double)s_hfne,
             s_open        ? "true" : "false",
             (s_rep_active || s_mon_active) ? "true" : "false",
             s_mon_active  ? "true" : "false");
    audio_stream_ws_send_text(buf);
}

static void reset_state(void)
{
    s_open  = false;
    s_hfne  = 1.0f;
    s_floor = -1.0f;
    memset(s_s1, 0, sizeof(s_s1));
    memset(s_s2, 0, sizeof(s_s2));
    s_buf_pos    = 0;
    s_block_count = 0;
    s_hold_until = 0;
}

// ── public API ────────────────────────────────────────────────────────────────

void squelch_sf_init(cJSON *config)
{
    for (int i = 0; i < HFNE_NBINS; i++)
        s_coeff[i] = 2.0f * cosf(2.0f * (float)M_PI * s_k[i] / SF_N);

    if (config) {
        cJSON *cfg = cJSON_GetObjectItem(config, "squelch_sf");
        if (cfg) {
            cJSON *t = cJSON_GetObjectItem(cfg, "hfne_threshold");
            if (cJSON_IsNumber(t)) {
                s_thr = (float)t->valuedouble;
                if (s_thr < 0.01f) s_thr = 0.01f;
                if (s_thr > 0.99f) s_thr = 0.99f;
            }
        }
    }

    memset(s_s1, 0, sizeof(s_s1));
    memset(s_s2, 0, sizeof(s_s2));
    s_buf_pos = 0;
    s_block_count = 0;
    s_hold_until = 0;
    s_floor = -1.0f;

    ESP_LOGI(TAG, "HFNE init: thr=%.2f, bins k=[%d,%d,%d,%d] (~3150-4350 Hz)",
             (double)s_thr, s_k[0], s_k[1], s_k[2], s_k[3]);
}

void squelch_sf_push_sample(int8_t sample)
{
    if (s_reset_needed) { s_reset_needed = false; reset_state(); }

    if (!s_rep_active && !s_mon_active) {
        if (s_send_ws_once) { s_send_ws_once = false; send_ws(); }
        return;
    }

    int32_t x = (int32_t)sample;
    for (int i = 0; i < HFNE_NBINS; i++) {
        int32_t s0 = (int32_t)(s_coeff[i] * (float)s_s1[i]) - s_s2[i] + x;
        s_s2[i] = s_s1[i];
        s_s1[i] = s0;
    }

    if (++s_buf_pos < SF_N) return;

    // Compute average Goertzel power across HFNE bins
    float raw = 0.0f;
    for (int i = 0; i < HFNE_NBINS; i++) {
        float p = (float)s_s1[i] * s_s1[i]
                + (float)s_s2[i] * s_s2[i]
                - s_coeff[i] * (float)s_s1[i] * (float)s_s2[i];
        raw += p < 0.0f ? 0.0f : p;
    }
    raw /= HFNE_NBINS;

    // Initialise floor on first block; afterwards update only while in noise
    if (s_floor < 0.0f) {
        s_floor = raw + 1.0f;
    } else if (raw > 0.7f * s_floor) {
        s_floor += (raw - s_floor) * HFNE_FLOOR_ALPHA;
    }

    float norm = raw / (s_floor + 1.0f);
    if (norm > 1.0f) norm = 1.0f;
    s_hfne = norm;

    // Close-hold squelch logic
    TickType_t now = xTaskGetTickCount();
    if (norm >= s_thr) {
        s_open = false;
        s_hold_until = now + pdMS_TO_TICKS(SQUELCH_HOLD_MS);
    } else if ((int32_t)(now - s_hold_until) >= 0) {
        s_open = true;
    }

    memset(s_s1, 0, sizeof(s_s1));
    memset(s_s2, 0, sizeof(s_s2));
    s_buf_pos = 0;

    if (++s_block_count >= WS_EVERY_BLOCKS) {
        s_block_count = 0;
        send_ws();
    }
}

void squelch_sf_set_repeater_active(bool en)
{
    bool was_active = s_rep_active || s_mon_active;
    s_rep_active = en;
    bool now_active = s_rep_active || s_mon_active;
    if (!en && !now_active && was_active) s_reset_needed = true;
    s_send_ws_once = true;
}

void squelch_sf_set_monitor_active(bool en)
{
    bool was_active = s_rep_active || s_mon_active;
    s_mon_active = en;
    bool now_active = s_rep_active || s_mon_active;
    if (!en && !now_active && was_active) s_reset_needed = true;
    s_send_ws_once = true;
}

bool squelch_sf_is_active(void)         { return s_rep_active || s_mon_active; }
bool squelch_sf_is_monitor_active(void) { return s_mon_active; }

void  squelch_hfne_set_threshold(float thr)
{
    if (thr < 0.01f) thr = 0.01f;
    if (thr > 0.99f) thr = 0.99f;
    s_thr = thr;
}

float squelch_hfne_get_value(void)     { return s_hfne; }
bool  squelch_hfne_is_open(void)       { return s_open; }
float squelch_hfne_get_threshold(void) { return s_thr; }

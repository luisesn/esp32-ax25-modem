#include "repeater.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

#include "LibAPRS-esp32-i2s/src/AFSK.h"
#include "morse.h"
#include "audio_stream.h"

static const char *TAG = "repeater";

#define REPEATER_SAMPLE_RATE_HZ  9600u
#define REPEATER_DAC_RATE_HZ     48000u
#define REPEATER_UPSAMPLE        5u     // DAC_RATE / SAMPLE_RATE
#define REPEATER_DAC_FRAME       AFSK_DAC_FRAME_SIZE  // 2048 bytes

// IIR envelope: s_env decays with shift 7 → time constant ~128 samples (~13 ms)
#define ENV_DECAY_SHIFT  7u

// ─── state ────────────────────────────────────────────────────────────────────

static bool              s_enabled       = false;
static bool              s_configured    = false; // config parsed, ready to enable
static int8_t           *s_buf           = NULL;
static uint32_t          s_buf_len       = 0;   // capacity (samples)
static uint32_t          s_rec_samples   = 0;   // samples written this session

static volatile repeater_state_t s_state = REPEATER_STATE_IDLE;
// Exposes current amplitude level to the UI.
static volatile uint32_t s_level = 0;

static uint32_t   s_env              = 0;
static uint32_t   s_squelch_thr      = 15;  // hardware squelch gates audio; threshold detects presence

// Post-TX lockout: squelch cannot reopen until this tick is reached.
// Prevents courtesy tone / TX echo from immediately re-triggering recording.
static TickType_t s_lockout_until  = 0;
static uint32_t   s_lockout_ms     = 3000;

static uint32_t   s_tail_delay_ms  = 2000;
static TickType_t s_tail_start     = 0;
static uint32_t   s_courtesy_hz    = 1000;
static uint32_t   s_courtesy_ms    = 200;
static bool       s_cw_id          = true;

// ─── helpers ──────────────────────────────────────────────────────────────────

static void send_ws_state(const char *state_str)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"type\":\"repeater\",\"state\":\"%s\"}", state_str);
    audio_stream_ws_send_text(buf);
}

// Flush a partial DAC block (remaining bytes set to 0x80 silence).
static void flush_dac_block(uint8_t *blk, uint32_t used)
{
    if (used == 0) return;
    memset(blk + used, 0x80, REPEATER_DAC_FRAME - used);
    afsk_write_dac_block(blk, REPEATER_DAC_FRAME, 2000);
}

// ─── API ──────────────────────────────────────────────────────────────────────

void repeater_init(cJSON *config)
{
    if (!config) return;
    cJSON *rep = cJSON_GetObjectItem(config, "repeater");
    if (!rep) return;

    cJSON *it;
    it = cJSON_GetObjectItem(rep, "squelch_threshold");
    if (cJSON_IsNumber(it)) s_squelch_thr = (uint32_t)it->valueint;
    it = cJSON_GetObjectItem(rep, "lockout_ms");
    if (cJSON_IsNumber(it)) s_lockout_ms    = (uint32_t)it->valueint;

    uint32_t max_s = 10;
    it = cJSON_GetObjectItem(rep, "max_record_s");
    if (cJSON_IsNumber(it) && it->valueint > 0) max_s = (uint32_t)it->valueint;

    it = cJSON_GetObjectItem(rep, "tail_delay_ms");
    if (cJSON_IsNumber(it)) s_tail_delay_ms = (uint32_t)it->valueint;

    it = cJSON_GetObjectItem(rep, "courtesy_tone_hz");
    if (cJSON_IsNumber(it)) s_courtesy_hz = (uint32_t)it->valueint;

    it = cJSON_GetObjectItem(rep, "courtesy_tone_ms");
    if (cJSON_IsNumber(it)) s_courtesy_ms = (uint32_t)it->valueint;

    it = cJSON_GetObjectItem(rep, "cw_id");
    if (cJSON_IsBool(it)) s_cw_id = cJSON_IsTrue(it);

    s_buf_len = max_s * REPEATER_SAMPLE_RATE_HZ;
    s_configured = true;

    cJSON *en = cJSON_GetObjectItem(rep, "enabled");
    if (cJSON_IsTrue(en)) repeater_set_enabled(true);

    ESP_LOGI(TAG, "configured: sq_thr=%u lockout=%ums, max=%us (%u B), tail=%ums, tone=%uHz/%ums, cw_id=%d%s",
             s_squelch_thr, s_lockout_ms, max_s, s_buf_len,
             s_tail_delay_ms, s_courtesy_hz, s_courtesy_ms, (int)s_cw_id,
             s_enabled ? " [enabled]" : "");
}

void repeater_set_enabled(bool en)
{
    if (en) {
        if (!s_configured) {
            ESP_LOGW(TAG, "can't enable: no repeater section in config.json");
            return;
        }
        if (!s_buf) {
            // Keep at least this much heap free so WiFi stays operational.
            // WiFi management frames, lwIP buffers and HTTP server need room.
            const size_t HEAP_RESERVE = 28 * 1024;

            size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
            ESP_LOGI(TAG, "heap free=%u B, largest=%u B, want=%u B (reserve=%u B)",
                     (unsigned)esp_get_free_heap_size(), (unsigned)largest,
                     s_buf_len, (unsigned)HEAP_RESERVE);

            // Clamp to what fits while honouring the reserve; halve until it fits.
            uint32_t try_len = s_buf_len;
            if (try_len + HEAP_RESERVE > largest)
                try_len = (largest > HEAP_RESERVE) ? (uint32_t)(largest - HEAP_RESERVE) : 0;
            // Align down to whole seconds for clarity
            try_len = (try_len / REPEATER_SAMPLE_RATE_HZ) * REPEATER_SAMPLE_RATE_HZ;

            while (try_len >= REPEATER_SAMPLE_RATE_HZ) {
                s_buf = (int8_t *)malloc(try_len);
                if (s_buf) break;
                try_len -= REPEATER_SAMPLE_RATE_HZ; // reduce by 1 s steps
            }
            if (!s_buf) {
                ESP_LOGE(TAG, "malloc failed — not enough contiguous heap (need >%u B free)",
                         (unsigned)HEAP_RESERVE);
                return;
            }
            s_buf_len = try_len;
            if (try_len < s_buf_len)
                ESP_LOGW(TAG, "buffer capped at %u B (%.1f s) to preserve heap headroom",
                         try_len, (float)try_len / REPEATER_SAMPLE_RATE_HZ);
            ESP_LOGI(TAG, "buffer allocated: %u B (%.1f s), heap after=%u B",
                     s_buf_len, (float)s_buf_len / REPEATER_SAMPLE_RATE_HZ,
                     (unsigned)esp_get_free_heap_size());
        }
    } else {
        s_state = REPEATER_STATE_IDLE;
        s_rec_samples = 0;
        s_env = 0; s_lockout_until = 0;
        free(s_buf);
        s_buf = NULL;
    }
    s_enabled = en;
    afsk_pause_rx(en);
    ESP_LOGI(TAG, "%s — RX decoding %s", en ? "enabled" : "disabled, buffer freed",
             en ? "paused" : "resumed");
}

void             repeater_set_threshold(uint32_t thr) { s_squelch_thr = thr; }
repeater_state_t repeater_get_state(void)    { return s_state; }
bool             repeater_is_enabled(void)   { return s_enabled; }
uint32_t         repeater_get_level(void)    { return s_level; }
uint32_t         repeater_get_threshold(void){ return s_squelch_thr; }

// ─── audio hook (9600 Hz, from receive_audio_task) ────────────────────────────

void repeater_audio_hook(int8_t sample)
{
    if (!s_enabled || !s_buf) return;

    uint32_t abs_s = (sample < 0) ? (uint32_t)(-sample) : (uint32_t)sample;
    s_env = s_env - (s_env >> ENV_DECAY_SHIFT) + abs_s;
    uint32_t level = s_env >> ENV_DECAY_SHIFT;
    s_level = level;

    // Hardware squelch gates audio at the radio; software only needs to detect
    // the presence/absence of signal above a single threshold.
    bool in_lockout = (s_lockout_until != 0) &&
                      ((int32_t)(s_lockout_until - xTaskGetTickCount()) > 0);
    bool squelch_open = (level > s_squelch_thr) && !in_lockout;

    // Push level to web UI ~every 0.5 s (4800 samples at 9600 Hz)
    static uint32_t s_level_div = 0;
    if (++s_level_div >= 4800) {
        s_level_div = 0;
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "{\"type\":\"repeater\",\"state\":\"%s\",\"level\":%lu,\"threshold\":%lu}",
                 s_state == REPEATER_STATE_IDLE      ? "idle"      :
                 s_state == REPEATER_STATE_RECORDING ? "recording" :
                 s_state == REPEATER_STATE_TAIL      ? "tail"      :
                 s_state == REPEATER_STATE_PENDING   ? "pending"   : "tx",
                 (unsigned long)level, (unsigned long)s_squelch_thr);
        audio_stream_ws_send_text(buf);
    }

    switch (s_state) {
        case REPEATER_STATE_IDLE:
            if (squelch_open) {
                s_rec_samples = 0;
                s_state = REPEATER_STATE_RECORDING;
                ESP_LOGI(TAG, "RECORDING (level=%u, thr=%u)", level, s_squelch_thr);
                send_ws_state("recording");
                s_buf[s_rec_samples++] = sample;
            }
            break;

        case REPEATER_STATE_RECORDING:
            // Write only while buffer has space; once full, keep waiting for signal to stop.
            if (s_rec_samples < s_buf_len) {
                s_buf[s_rec_samples++] = sample;
                if (s_rec_samples == s_buf_len)
                    ESP_LOGI(TAG, "buffer full (%.1f s) — waiting for signal to stop",
                             (float)s_buf_len / REPEATER_SAMPLE_RATE_HZ);
            }
            if (!squelch_open) {
                s_tail_start = xTaskGetTickCount();
                s_state = REPEATER_STATE_TAIL;
            }
            break;

        case REPEATER_STATE_TAIL:
            // Signal stopped; just wait out the tail delay before transmitting.
            if (squelch_open) {
                s_state = REPEATER_STATE_RECORDING;
            } else if ((xTaskGetTickCount() - s_tail_start) >= pdMS_TO_TICKS(s_tail_delay_ms)) {
                s_state = REPEATER_STATE_PENDING;
                ESP_LOGI(TAG, "PENDING (%u samples, %.1f s)",
                         s_rec_samples, (float)s_rec_samples / (float)REPEATER_SAMPLE_RATE_HZ);
                send_ws_state("pending");
            }
            break;

        case REPEATER_STATE_PENDING:
        case REPEATER_STATE_TX:
            // Don't overwrite the buffer while waiting to TX or during TX
            break;
    }
}

// ─── dispatch hook (receive_audio_task context, not in tx_mode) ───────────────

void repeater_dispatch_if_pending(void)
{
    if (s_state != REPEATER_STATE_PENDING) return;

    // Safety guard: don't TX if the emitter is still transmitting.
    if (s_level > s_squelch_thr) {
        ESP_LOGW(TAG, "emitter still active (level=%lu), delaying TX — back to TAIL",
                 (unsigned long)s_level);
        s_tail_start = xTaskGetTickCount();
        s_state = REPEATER_STATE_TAIL;
        return;
    }

    s_state = REPEATER_STATE_TX;
    ESP_LOGI(TAG, "TX: %u samples (%.1f s)",
             s_rec_samples, (float)s_rec_samples / (float)REPEATER_SAMPLE_RATE_HZ);
    send_ws_state("tx");

    afsk_switch_to_tx();
    afsk_ptt_set(true);

    uint8_t dac_buf[REPEATER_DAC_FRAME];
    uint32_t dac_pos = 0;

    // ── 1 s pre-TX silence: radio hardware needs time to switch RX→TX ───────────
    memset(dac_buf, 0x80, REPEATER_DAC_FRAME);
    // ceil(48000 / 2048) = 24 frames ≈ 1.02 s
    for (uint32_t f = 0; f < (REPEATER_DAC_RATE_HZ + REPEATER_DAC_FRAME - 1) / REPEATER_DAC_FRAME; f++)
        afsk_write_dac_block(dac_buf, REPEATER_DAC_FRAME, 2000);

    // ── playback: upsample 5× (9600 → 48000 Hz) ──────────────────────────────
    for (uint32_t i = 0; i < s_rec_samples; i++) {
        uint8_t pcm = (uint8_t)((int32_t)s_buf[i] + 128);
        for (uint32_t u = 0; u < REPEATER_UPSAMPLE; u++) {
            dac_buf[dac_pos++] = pcm;
            if (dac_pos >= REPEATER_DAC_FRAME) {
                afsk_write_dac_block(dac_buf, REPEATER_DAC_FRAME, 2000);
                dac_pos = 0;
            }
        }
    }
    flush_dac_block(dac_buf, dac_pos);
    dac_pos = 0;

    // ── courtesy tone ─────────────────────────────────────────────────────────
    if (s_courtesy_ms > 0 && s_courtesy_hz > 0) {
        uint32_t tone_samples = (s_courtesy_ms * REPEATER_DAC_RATE_HZ) / 1000u;
        float phase     = 0.0f;
        float phase_inc = (2.0f * (float)M_PI * (float)s_courtesy_hz) / (float)REPEATER_DAC_RATE_HZ;
        for (uint32_t t = 0; t < tone_samples; t++) {
            uint8_t v = (uint8_t)(128 + (int)(sinf(phase) * 100.0f));
            dac_buf[dac_pos++] = v;
            phase += phase_inc;
            if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
            if (dac_pos >= REPEATER_DAC_FRAME) {
                afsk_write_dac_block(dac_buf, REPEATER_DAC_FRAME, 2000);
                dac_pos = 0;
            }
        }
        flush_dac_block(dac_buf, dac_pos);
    }

    afsk_ptt_set(false);
    afsk_switch_to_rx();
    // Start lockout: squelch stays deaf for lockout_ms to avoid re-triggering
    // on our own courtesy tone or TX echo through the radio.
    s_lockout_until = xTaskGetTickCount() + pdMS_TO_TICKS(s_lockout_ms);

    // CW ID fires on next dispatch cycle via the existing Morse task
    if (s_cw_id) morse_trigger_now();

    // Reset
    s_rec_samples = 0;
    s_env         = 0;
    s_state       = REPEATER_STATE_IDLE;

    ESP_LOGI(TAG, "TX done → IDLE");
    send_ws_state("idle");
}

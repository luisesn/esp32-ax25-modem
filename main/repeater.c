#include "repeater.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

#include "LibAPRS-esp32-i2s/src/AFSK.h"
#include "morse.h"
#include "audio_stream.h"
#include "squelch_sf.h"

static const char *TAG = "repeater";

#define REPEATER_SAMPLE_RATE_HZ  9600u
#define REPEATER_DAC_RATE_HZ     48000u
#define REPEATER_UPSAMPLE        5u
#define REPEATER_DAC_FRAME       AFSK_DAC_FRAME_SIZE  // 2048 bytes

// ─── state ────────────────────────────────────────────────────────────────────

static bool              s_enabled    = false;
static bool              s_configured = false;
static uint8_t          *s_buf        = NULL;   // ADPCM blocks
static uint32_t          s_buf_len    = 0;       // buffer capacity in bytes
static uint32_t          s_rec_blocks = 0;       // complete ADPCM blocks recorded

// Staging: accumulate samples until a full ADPCM block can be encoded.
static int8_t        s_stage[ADPCM_SAMPLES_BLOCK];
static int           s_stage_pos = 0;
static adpcm_state_t s_enc_state;

// Decode scratch used during TX playback — static to avoid stack pressure.
static int8_t s_decode_scratch[ADPCM_SAMPLES_BLOCK];

static volatile repeater_state_t s_state = REPEATER_STATE_IDLE;

// Post-TX lockout: blocks HFNE squelch from re-triggering on our own TX echo.
static TickType_t s_lockout_until = 0;
static uint32_t   s_lockout_ms    = 3000;

// Minimum recording window: even if the squelch closes early, keep recording
// until at least MIN_REC_MS have elapsed since the squelch was last open.
// Resets each time the squelch reopens during recording.
#define MIN_REC_MS 500
static TickType_t s_min_rec_until = 0;

static uint32_t   s_tail_delay_ms = 2000;
static TickType_t s_tail_start    = 0;
static uint32_t   s_courtesy_hz   = 1000;
static uint32_t   s_courtesy_ms   = 200;
static bool       s_cw_id         = true;

// ─── helpers ──────────────────────────────────────────────────────────────────

static void send_ws_state(const char *state_str)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"type\":\"repeater\",\"state\":\"%s\"}", state_str);
    audio_stream_ws_send_text(buf);
}

static void flush_dac_block(uint8_t *blk, uint32_t used)
{
    if (used == 0) return;
    memset(blk + used, 0x80, REPEATER_DAC_FRAME - used);
    afsk_write_dac_block(blk, REPEATER_DAC_FRAME, 2000);
}

// Encode the partial staging buffer as a final ADPCM block (zero-padded).
static void flush_staging(void)
{
    if (s_stage_pos == 0) return;
    uint32_t max_blocks = s_buf_len / ADPCM_BLOCK_BYTES;
    if (s_rec_blocks < max_blocks) {
        memset(s_stage + s_stage_pos, 0,
               (size_t)(ADPCM_SAMPLES_BLOCK - s_stage_pos));
        encode_block(&s_enc_state, s_stage,
                     s_buf + s_rec_blocks * ADPCM_BLOCK_BYTES);
        s_rec_blocks++;
    }
    s_stage_pos = 0;
}

// ─── API ──────────────────────────────────────────────────────────────────────

void repeater_init(cJSON *config)
{
    if (!config) return;
    cJSON *rep = cJSON_GetObjectItem(config, "repeater");
    if (!rep) return;

    cJSON *it;
    it = cJSON_GetObjectItem(rep, "lockout_ms");
    if (cJSON_IsNumber(it)) s_lockout_ms = (uint32_t)it->valueint;

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

    uint32_t max_blocks = (max_s * REPEATER_SAMPLE_RATE_HZ + ADPCM_SAMPLES_BLOCK - 1)
                          / ADPCM_SAMPLES_BLOCK;
    s_buf_len = max_blocks * ADPCM_BLOCK_BYTES;
    s_configured = true;

    cJSON *en = cJSON_GetObjectItem(rep, "enabled");
    if (cJSON_IsTrue(en)) repeater_set_enabled(true);

    ESP_LOGI(TAG, "configured: lockout=%ums max=%us tail=%ums tone=%uHz/%ums cw_id=%d%s",
             s_lockout_ms, max_s, s_tail_delay_ms,
             s_courtesy_hz, s_courtesy_ms, (int)s_cw_id,
             s_enabled ? " [enabled]" : "");
}

void repeater_set_enabled(bool en)
{
#if !REPEATER_ENABLED
    if (en) {
        ESP_LOGW(TAG, "repetidor deshabilitado en compilación (REPEATER_ENABLED=0)");
        return;
    }
#endif
    if (en) {
        if (!s_configured) {
            ESP_LOGW(TAG, "can't enable: no repeater section in config.json");
            return;
        }
        if (!s_buf) {
            // Free the WAV server task before attempting the large malloc —
            // releases its 4 KB stack and open socket structures.
            audio_stream_stop_wav_server();

            const size_t HEAP_RESERVE = 28 * 1024;
            size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
            ESP_LOGI(TAG, "heap free=%u B, largest=%u B, want=%u B (reserve=%u B)",
                     (unsigned)esp_get_free_heap_size(), (unsigned)largest,
                     s_buf_len, (unsigned)HEAP_RESERVE);

            uint32_t try_len = s_buf_len;
            if (try_len + HEAP_RESERVE > largest)
                try_len = (largest > HEAP_RESERVE) ? (uint32_t)(largest - HEAP_RESERVE) : 0;
            try_len = (try_len / ADPCM_BLOCK_BYTES) * ADPCM_BLOCK_BYTES;

            while (try_len >= ADPCM_BLOCK_BYTES) {
                s_buf = (uint8_t *)malloc(try_len);
                if (s_buf) break;
                try_len -= ADPCM_BLOCK_BYTES;
            }
            if (!s_buf) {
                ESP_LOGE(TAG, "malloc failed — not enough contiguous heap (need >%u B free)",
                         (unsigned)HEAP_RESERVE);
                return;
            }
            s_buf_len = try_len;
            float secs = (float)(s_buf_len / ADPCM_BLOCK_BYTES * ADPCM_SAMPLES_BLOCK)
                         / REPEATER_SAMPLE_RATE_HZ;
            ESP_LOGI(TAG, "buffer allocated: %u B (%.1f s), heap after=%u B",
                     s_buf_len, secs, (unsigned)esp_get_free_heap_size());
        }
    } else {
        s_state = REPEATER_STATE_IDLE;
        s_rec_blocks = 0;
        s_stage_pos  = 0;
        s_lockout_until = 0;
        s_min_rec_until = 0;
        free(s_buf);
        s_buf = NULL;
    }
    s_enabled = en;
    squelch_sf_set_repeater_active(en);
    afsk_pause_rx(en);
    ESP_LOGI(TAG, "%s — RX decoding %s", en ? "enabled" : "disabled, buffer freed",
             en ? "paused" : "resumed");
}

repeater_state_t repeater_get_state(void)  { return s_state; }
bool             repeater_is_enabled(void) { return s_enabled; }

// ─── audio hook (9600 Hz, from receive_audio_task) ────────────────────────────

void repeater_audio_hook(int8_t sample)
{
    if (!s_enabled || !s_buf) return;

    TickType_t now = xTaskGetTickCount();
    bool in_lockout = (s_lockout_until != 0) &&
                      ((int32_t)(s_lockout_until - now) > 0);
    bool squelch_open = squelch_hfne_is_open() && !in_lockout;

    // Push state to web UI ~every 0.5 s
    static uint32_t s_state_div = 0;
    if (++s_state_div >= 4800) {
        s_state_div = 0;
        send_ws_state(s_state == REPEATER_STATE_IDLE      ? "idle"      :
                      s_state == REPEATER_STATE_RECORDING ? "recording" :
                      s_state == REPEATER_STATE_TAIL      ? "tail"      :
                      s_state == REPEATER_STATE_PENDING   ? "pending"   : "tx");
    }

    switch (s_state) {
        case REPEATER_STATE_IDLE:
            if (squelch_open) {
                s_rec_blocks = 0;
                s_stage_pos  = 0;
                s_enc_state.predictor  = 0;
                s_enc_state.step_index = 0;
                s_min_rec_until = now + pdMS_TO_TICKS(MIN_REC_MS);
                s_state = REPEATER_STATE_RECORDING;
                ESP_LOGI(TAG, "RECORDING");
                send_ws_state("recording");
                s_stage[s_stage_pos++] = sample;
            }
            break;

        case REPEATER_STATE_RECORDING: {
            uint32_t max_blocks = s_buf_len / ADPCM_BLOCK_BYTES;
            if (s_rec_blocks < max_blocks) {
                s_stage[s_stage_pos++] = sample;
                if (s_stage_pos == ADPCM_SAMPLES_BLOCK) {
                    encode_block(&s_enc_state, s_stage,
                                 s_buf + s_rec_blocks * ADPCM_BLOCK_BYTES);
                    s_rec_blocks++;
                    s_stage_pos = 0;
                    if (s_rec_blocks == max_blocks)
                        ESP_LOGI(TAG, "buffer full (%.1f s) — waiting for signal to stop",
                                 (float)(max_blocks * ADPCM_SAMPLES_BLOCK)
                                 / REPEATER_SAMPLE_RATE_HZ);
                }
            }
            if (squelch_open) {
                // Signal still present: keep refreshing the minimum recording window.
                s_min_rec_until = now + pdMS_TO_TICKS(MIN_REC_MS);
            } else if ((int32_t)(now - s_min_rec_until) >= 0) {
                // Signal gone AND minimum window expired → move to tail.
                s_tail_start = now;
                s_state = REPEATER_STATE_TAIL;
            }
            // else: signal gone but minimum window still active — keep recording.
            break;
        }

        case REPEATER_STATE_TAIL:
            if (squelch_open) {
                s_min_rec_until = now + pdMS_TO_TICKS(MIN_REC_MS);
                s_state = REPEATER_STATE_RECORDING;
            } else if ((now - s_tail_start) >= pdMS_TO_TICKS(s_tail_delay_ms)) {
                flush_staging();
                s_state = REPEATER_STATE_PENDING;
                ESP_LOGI(TAG, "PENDING (%u blocks, %.1f s)", s_rec_blocks,
                         (float)(s_rec_blocks * ADPCM_SAMPLES_BLOCK) / REPEATER_SAMPLE_RATE_HZ);
                send_ws_state("pending");
            }
            break;

        case REPEATER_STATE_PENDING:
        case REPEATER_STATE_TX:
            break;
    }
}

// ─── dispatch hook (receive_audio_task context, not in tx_mode) ───────────────

void repeater_dispatch_if_pending(void)
{
    if (s_state != REPEATER_STATE_PENDING) return;

    // Safety guard: don't TX while emitter is still active.
    if (squelch_hfne_is_open()) {
        ESP_LOGW(TAG, "emitter still active, delaying TX — back to TAIL");
        s_tail_start = xTaskGetTickCount();
        s_state = REPEATER_STATE_TAIL;
        return;
    }

    s_state = REPEATER_STATE_TX;
    ESP_LOGI(TAG, "TX: %u blocks (%.1f s)", s_rec_blocks,
             (float)(s_rec_blocks * ADPCM_SAMPLES_BLOCK) / REPEATER_SAMPLE_RATE_HZ);
    send_ws_state("tx");

    afsk_switch_to_tx();
    afsk_ptt_set(true);

    uint8_t dac_buf[REPEATER_DAC_FRAME];
    uint32_t dac_pos = 0;

    // 1 s pre-TX silence: radio hardware needs time to switch RX→TX
    memset(dac_buf, 0x80, REPEATER_DAC_FRAME);
    for (uint32_t f = 0; f < (REPEATER_DAC_RATE_HZ + REPEATER_DAC_FRAME - 1) / REPEATER_DAC_FRAME; f++)
        afsk_write_dac_block(dac_buf, REPEATER_DAC_FRAME, 2000);

    // Playback: decode ADPCM blocks, upsample 5× (9600 → 48000 Hz)
    for (uint32_t b = 0; b < s_rec_blocks; b++) {
        decode_block(s_buf + b * ADPCM_BLOCK_BYTES, s_decode_scratch);
        for (int i = 0; i < ADPCM_SAMPLES_BLOCK; i++) {
            uint8_t pcm = (uint8_t)((int32_t)s_decode_scratch[i] + 128);
            for (uint32_t u = 0; u < REPEATER_UPSAMPLE; u++) {
                dac_buf[dac_pos++] = pcm;
                if (dac_pos >= REPEATER_DAC_FRAME) {
                    afsk_write_dac_block(dac_buf, REPEATER_DAC_FRAME, 2000);
                    dac_pos = 0;
                }
            }
        }
    }
    flush_dac_block(dac_buf, dac_pos);
    dac_pos = 0;

    // Courtesy tone
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
    // Lockout: prevent TX echo from re-triggering HFNE squelch
    s_lockout_until = xTaskGetTickCount() + pdMS_TO_TICKS(s_lockout_ms);

    if (s_cw_id) morse_trigger_now();

    s_rec_blocks = 0;
    s_stage_pos  = 0;
    s_state = REPEATER_STATE_IDLE;

    ESP_LOGI(TAG, "TX done → IDLE");
    send_ws_state("idle");
}

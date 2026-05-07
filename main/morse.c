#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "cJSON.h"
#include "morse.h"
#include "audio_stream.h"

static const char *TAG = "morse";

// ---------------------------------------------------------------------------
// Morse encoding table: A-Z (indices 0-25), 0-9 (indices 26-35)
// ---------------------------------------------------------------------------
static const char *s_morse_table[36] = {
    /* A */ ".-",   /* B */ "-...", /* C */ "-.-.", /* D */ "-..",
    /* E */ ".",    /* F */ "..-.", /* G */ "--.",  /* H */ "....",
    /* I */ "..",   /* J */ ".---", /* K */ "-.-",  /* L */ ".-..",
    /* M */ "--",   /* N */ "-.",   /* O */ "---",  /* P */ ".--.",
    /* Q */ "--.-", /* R */ ".-.",  /* S */ "...",  /* T */ "-",
    /* U */ "..-",  /* V */ "...-", /* W */ ".--",  /* X */ "-..-",
    /* Y */ "-.--", /* Z */ "--..",
    /* 0 */ "-----", /* 1 */ ".----", /* 2 */ "..---", /* 3 */ "...--",
    /* 4 */ "....-", /* 5 */ ".....", /* 6 */ "-....", /* 7 */ "--...",
    /* 8 */ "---..", /* 9 */ "----.",
};

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
#define MORSE_DAC_FRAME_SIZE 2048   /* must match TX_SAMPLE_BUFLEN in AFSK.cpp */

static uint8_t  s_sine_lut[256];
static uint32_t s_phase;           /* 32-bit phase acc; upper 8 bits = LUT index */
static uint32_t s_phase_step;      /* added each sample for the chosen tone_hz   */
static uint32_t s_unit_samples;    /* samples per 1 timing unit (PARIS standard) */

static uint8_t  s_dac_buf[MORSE_DAC_FRAME_SIZE];
static int      s_buf_fill = 0;

static SemaphoreHandle_t s_ready_sem   = NULL;
static SemaphoreHandle_t s_done_sem    = NULL;
static TaskHandle_t      s_beacon_task = NULL;

static volatile bool s_enabled   = false;
static volatile int  s_period_s  = 600;
static char          s_callsign[16];

// ---------------------------------------------------------------------------
// Forward declarations of DAC primitives (defined in AFSK.cpp, extern "C")
// ---------------------------------------------------------------------------
extern void afsk_morse_tx_begin(void);
extern void afsk_morse_tx_frame(const uint8_t *buf);
extern void afsk_morse_tx_end(void);

// ---------------------------------------------------------------------------
// Sample generation helpers
// ---------------------------------------------------------------------------

// Write `count` samples (tone or silence) into s_dac_buf, flushing a full
// 2048-byte frame to the DAC every time the buffer fills up.
static void write_samples(uint32_t count, bool tone) {
    for (uint32_t i = 0; i < count; i++) {
        if (tone) {
            s_phase += s_phase_step;
            s_dac_buf[s_buf_fill] = s_sine_lut[(s_phase >> 24) & 0xFF];
        } else {
            s_dac_buf[s_buf_fill] = 128; /* DC mid-rail = silence */
        }
        if (++s_buf_fill == MORSE_DAC_FRAME_SIZE) {
            afsk_morse_tx_frame(s_dac_buf);
            s_buf_fill = 0;
        }
    }
}

// Flush whatever is left in s_dac_buf (padded with silence) to the DAC.
static void flush_dac_buf(void) {
    if (s_buf_fill > 0) {
        memset(s_dac_buf + s_buf_fill, 128, (size_t)(MORSE_DAC_FRAME_SIZE - s_buf_fill));
        afsk_morse_tx_frame(s_dac_buf);
        s_buf_fill = 0;
    }
}

// ---------------------------------------------------------------------------
// Morse character output (PARIS standard timing)
//   Dot  = 1 unit tone
//   Dash = 3 units tone
//   Inter-element gap = 1 unit silence   (written after every element)
//   Inter-letter gap  = 3 units silence  (= 1 inter-element + 2 extra)
//   Inter-word gap    = 7 units silence  (= 1 inter-element + 6 extra)
// ---------------------------------------------------------------------------
static void send_morse_char(char c) {
    if (c == ' ') {
        // 7-unit inter-word gap; 1 unit already written as last inter-element,
        // so write 6 more.
        write_samples(6 * s_unit_samples, false);
        return;
    }

    int idx = -1;
    if      (c >= 'A' && c <= 'Z') idx = c - 'A';
    else if (c >= 'a' && c <= 'z') idx = c - 'a';
    else if (c >= '0' && c <= '9') idx = c - '0' + 26;
    if (idx < 0 || idx >= 36) return;

    const char *code = s_morse_table[idx];
    while (*code) {
        uint32_t dur = (*code == '-') ? 3u : 1u;
        write_samples(dur * s_unit_samples, true);   /* element tone            */
        write_samples(s_unit_samples, false);         /* 1-unit inter-element gap */
        code++;
    }
    /* Inter-letter gap = 3 units; 1 already written above, add 2 more. */
    write_samples(2 * s_unit_samples, false);
}

static void do_morse_tx(void) {
    s_phase    = 0;
    s_buf_fill = 0;

    /* Short lead-in silence (3 units) */
    write_samples(3 * s_unit_samples, false);

    for (int i = 0; s_callsign[i] != '\0'; i++) {
        send_morse_char(s_callsign[i]);
    }

    flush_dac_buf();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool morse_check_and_dispatch(void) {
    if (!s_enabled || !s_ready_sem) return false;
    if (xSemaphoreTake(s_ready_sem, 0) != pdTRUE) return false;

    ESP_LOGI(TAG, "Morse beacon TX: %s  unit=%lu samp", s_callsign, (unsigned long)s_unit_samples);

    {
        static char s_morse_json[64];
        snprintf(s_morse_json, sizeof(s_morse_json),
                 "{\"type\":\"morse_tx\",\"call\":\"%s\"}", s_callsign);
        audio_stream_ws_send_text(s_morse_json);
    }

    afsk_morse_tx_begin();
    do_morse_tx();
    afsk_morse_tx_end();

    xSemaphoreGive(s_done_sem);
    return true;
}

void morse_trigger_now(void) {
    if (s_ready_sem) xSemaphoreGive(s_ready_sem);
}

// ---------------------------------------------------------------------------
// Background timer task
// ---------------------------------------------------------------------------
static void morse_beacon_task(void *arg) {
    (void)arg;
    for (;;) {
        /* Wait for the configured period, then trigger a beacon. */
        vTaskDelay(pdMS_TO_TICKS((uint32_t)s_period_s * 1000u));
        if (s_enabled) {
            xSemaphoreGive(s_ready_sem);
            /* Wait until receive_audio_task finishes the TX. */
            xSemaphoreTake(s_done_sem, portMAX_DELAY);
        }
    }
}

// ---------------------------------------------------------------------------
// Initialisation (safe to call repeatedly on config reload)
// ---------------------------------------------------------------------------
void morse_init(const cJSON *cfg) {
    /* --- Parse config ---------------------------------------------------- */
    bool new_enabled = false;
    int  tone_hz     = 1000;
    int  wpm         = 20;
    int  period_s    = 600;
    char callsign[16];
    strncpy(callsign, "NOCALL", sizeof(callsign) - 1);
    callsign[sizeof(callsign) - 1] = '\0';

    if (cfg) {
        cJSON *m = cJSON_GetObjectItem(cfg, "morse");
        if (m) {
            cJSON *j;
            j = cJSON_GetObjectItem(m, "enabled");
            if (cJSON_IsBool(j))   new_enabled = cJSON_IsTrue(j);

            j = cJSON_GetObjectItem(m, "tone_hz");
            if (cJSON_IsNumber(j) && j->valueint > 0) tone_hz = j->valueint;

            j = cJSON_GetObjectItem(m, "wpm");
            if (cJSON_IsNumber(j) && j->valueint > 0) wpm = j->valueint;

            j = cJSON_GetObjectItem(m, "period_s");
            if (cJSON_IsNumber(j) && j->valueint > 0) period_s = j->valueint;

            j = cJSON_GetObjectItem(m, "callsign");
            if (cJSON_IsString(j) && j->valuestring[0] != '\0') {
                strncpy(callsign, j->valuestring, sizeof(callsign) - 1);
                callsign[sizeof(callsign) - 1] = '\0';
            } else {
                /* Fall back to aprs.callsign */
                cJSON *aprs = cJSON_GetObjectItem(cfg, "aprs");
                if (aprs) {
                    cJSON *cs = cJSON_GetObjectItem(aprs, "callsign");
                    if (cJSON_IsString(cs) && cs->valuestring[0] != '\0') {
                        strncpy(callsign, cs->valuestring, sizeof(callsign) - 1);
                        callsign[sizeof(callsign) - 1] = '\0';
                    }
                }
            }
        }
    }

    /* --- Apply ----------------------------------------------------------- */
    s_enabled  = new_enabled;
    s_period_s = period_s;
    strncpy(s_callsign, callsign, sizeof(s_callsign) - 1);
    s_callsign[sizeof(s_callsign) - 1] = '\0';

    /* Build 256-entry sine LUT: unsigned 8-bit, midpoint = 128, peak = 255 */
    for (int i = 0; i < 256; i++) {
        s_sine_lut[i] = (uint8_t)(128.0f + 127.0f * sinf(2.0f * (float)M_PI * i / 256.0f));
    }

    /* Phase step: s_phase is uint32; upper 8 bits index into 256-entry LUT.
     * step = tone_hz / samplerate * 2^32
     *      = tone_hz * 4294967296 / 48000                                   */
    s_phase_step = (uint32_t)((double)tone_hz * 4294967296.0 / 48000.0);

    /* Samples per PARIS timing unit: 48000 * 60 / (50 * wpm) */
    s_unit_samples = (uint32_t)(48000UL * 60UL / (50UL * (uint32_t)wpm));

    /* Create semaphores once */
    if (!s_ready_sem) s_ready_sem = xSemaphoreCreateBinary();
    if (!s_done_sem)  s_done_sem  = xSemaphoreCreateBinary();

    /* Create the background task once; it reads s_enabled/s_period_s dynamically */
    if (s_beacon_task == NULL) {
        xTaskCreate(morse_beacon_task, "morse_beacon", 2048, NULL, 2, &s_beacon_task);
    }

    ESP_LOGI(TAG, "morse_init: %s  call=%s tone=%dHz wpm=%d period=%ds",
             s_enabled ? "ENABLED" : "disabled", s_callsign, tone_hz, wpm, period_s);
}

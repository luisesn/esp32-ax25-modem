#include "carrier_detect.h"

#include <math.h>
#include <stdio.h>
#include "esp_timer.h"

#define CD_N              128      // muestras por bloque (13,3 ms @ 9600 Hz)
#define CD_BLOCKS         8        // bloques por ventana (~107 ms)
#define CD_FS             9600.0f
#define CD_NB             2        // 3525 y 3975 Hz (banda de ruido del discriminador)

#define CD_ENTER_HF_DB    8.0f     // caída de HF respecto al suelo para entrar
#define CD_ENTER_RMS_FRAC 0.65f    // RMS máximo (fracción del suelo) para entrar
#define CD_STAY_HF_DB     5.0f     // umbrales más laxos para mantener (histéresis)
#define CD_STAY_RMS_FRAC  0.80f
#define CD_ENTER_WINDOWS  2        // ventanas consecutivas para declarar ocupado
#define CD_EXIT_WINDOWS   3        // ventanas consecutivas sin cumplir para liberar
#define CD_WARMUP_WINDOWS 20       // ~2 s de suelo antes de poder disparar
#define CD_FLOOR_SHIFT    6        // EMA del suelo: 1/64 por ventana (~7 s)
#define CD_MIN_FLOOR_RMS  6.0f     // suelo de RMS mínimo útil: sin ruido => no detecta
#define CD_GAP_US         400000   // hueco entre ventanas (TX/ADC parado) => descartar

static const float s_freq[CD_NB] = { 3525.0f, 3975.0f };
static float s_coeff[CD_NB];

// Estado del hook (solo receive_audio_task)
static float    s_s1[CD_NB], s_s2[CD_NB];
static float    s_phf;
static uint32_t s_sumsq;
static int      s_pos, s_blk;
static int64_t  s_last_win_us;

static float    s_floor_hf, s_floor_rms;   // potencia HF y RMS en canal libre
static int      s_warm, s_hits, s_miss;
static float    s_last_hf_db, s_last_rms;
#define CD_HIST 16                 // ventanas recientes (~1,7 s) para calidad por trama
static float    s_hist_db[CD_HIST];
static volatile int s_hist_pos;
static volatile bool s_busy;
static volatile uint32_t s_busy_events;

void carrier_detect_init(void)
{
    for (int i = 0; i < CD_NB; i++)
        s_coeff[i] = 2.0f * cosf(2.0f * (float)M_PI * s_freq[i] / CD_FS);
}

bool carrier_detect_busy(void) { return s_busy; }

static void window_done(float hf, float rms)
{
    int64_t now = esp_timer_get_time();
    bool gap = s_last_win_us && (now - s_last_win_us) > CD_GAP_US;
    s_last_win_us = now;
    if (gap) {                 // tras un TX o una pausa: no mezclar ventanas
        s_hits = 0; s_miss = 0; s_busy = false;
        return;
    }

    s_last_hf_db = 10.0f * log10f(hf + 1e-3f);
    s_last_rms = rms;
    s_hist_db[s_hist_pos % CD_HIST] = s_last_hf_db;
    s_hist_pos = (s_hist_pos + 1) % CD_HIST;

    if (s_warm < CD_WARMUP_WINDOWS) {          // aprender el suelo inicial
        if (s_warm == 0) { s_floor_hf = hf; s_floor_rms = rms; }
        else {
            s_floor_hf  += (hf  - s_floor_hf)  / (float)(s_warm + 1);
            s_floor_rms += (rms - s_floor_rms) / (float)(s_warm + 1);
        }
        s_warm++;
        return;
    }

    float hf_ratio_enter = powf(10.0f, -CD_ENTER_HF_DB / 10.0f);
    float hf_ratio_stay  = powf(10.0f, -CD_STAY_HF_DB / 10.0f);
    bool usable = s_floor_rms >= CD_MIN_FLOOR_RMS;
    bool enter_ok = usable && hf < s_floor_hf * hf_ratio_enter && rms < s_floor_rms * CD_ENTER_RMS_FRAC;
    bool stay_ok  = usable && hf < s_floor_hf * hf_ratio_stay  && rms < s_floor_rms * CD_STAY_RMS_FRAC;

    if (!s_busy) {
        s_hits = enter_ok ? s_hits + 1 : 0;
        if (s_hits >= CD_ENTER_WINDOWS) { s_busy = true; s_miss = 0; s_busy_events++; }
        else if (!enter_ok && !stay_ok) {
            // canal libre: el suelo sigue al ruido (EMA lenta)
            s_floor_hf  += (hf  - s_floor_hf)  / (float)(1 << CD_FLOOR_SHIFT);
            s_floor_rms += (rms - s_floor_rms) / (float)(1 << CD_FLOOR_SHIFT);
        }
    } else {
        s_miss = stay_ok ? 0 : s_miss + 1;
        if (s_miss >= CD_EXIT_WINDOWS) { s_busy = false; s_hits = 0; }
    }
}

void carrier_detect_push_sample(int8_t sample)
{
    float fx = (float)sample;
    s_sumsq += (uint32_t)((int32_t)sample * sample);
    for (int i = 0; i < CD_NB; i++) {
        float s0 = s_coeff[i] * s_s1[i] - s_s2[i] + fx;
        s_s2[i] = s_s1[i];
        s_s1[i] = s0;
    }
    if (++s_pos < CD_N) return;
    s_pos = 0;

    for (int i = 0; i < CD_NB; i++) {
        float p = s_s1[i] * s_s1[i] + s_s2[i] * s_s2[i] - s_coeff[i] * s_s1[i] * s_s2[i];
        s_phf += p < 0.0f ? 0.0f : p;
        s_s1[i] = 0.0f; s_s2[i] = 0.0f;
    }
    if (++s_blk < CD_BLOCKS) return;

    const float norm = (float)CD_BLOCKS * ((float)CD_N * (float)CD_N / 4.0f) * (float)CD_NB;
    float hf  = s_phf / norm;
    float rms = sqrtf((float)s_sumsq / (float)(CD_BLOCKS * CD_N));
    s_blk = 0; s_phf = 0.0f; s_sumsq = 0;
    window_done(hf, rms);
}

int carrier_detect_status_json(char *buf, int len)
{
    return snprintf(buf, len,
        "{\"busy\":%s,\"events\":%u,\"warm\":%s,\"floor_hf_db\":%.1f,\"floor_rms\":%.1f,"
        "\"hf_db\":%.1f,\"rms\":%.1f}",
        s_busy ? "true" : "false", (unsigned)s_busy_events,
        s_warm >= CD_WARMUP_WINDOWS ? "true" : "false",
        10.0f * log10f(s_floor_hf + 1e-3f), s_floor_rms, s_last_hf_db, s_last_rms);
}

// Calidad de una trama recién recibida: caída del ruido HF (quieting) respecto
// al suelo del canal libre, promediada en dB sobre las ventanas que cubren la
// trama. Más caída = portadora más fuerte. Es relativa a esta radio y volumen.
bool carrier_detect_frame_quality(int duration_ms, float *quiet_db)
{
    if (s_warm < CD_WARMUP_WINDOWS || s_floor_rms < CD_MIN_FLOOR_RMS) return false;
    int n = duration_ms / 107 + 1;
    if (n < 2) n = 2;
    if (n > CD_HIST) n = CD_HIST;
    int pos = s_hist_pos;
    float acc = 0.0f;
    for (int i = 0; i < n; i++)
        acc += s_hist_db[(pos - 1 - i + 2 * CD_HIST) % CD_HIST];
    float q = 10.0f * log10f(s_floor_hf + 1e-3f) - acc / (float)n;
    *quiet_db = q < 0.0f ? 0.0f : q;
    return true;
}

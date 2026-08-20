#include "sstv.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"
#include "config.h"
#include "audio_stream.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "rom/tjpgd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>   // unlink()

#define TAG "sstv"

#define SSTV_MAX_FILES  10
#define SSTV_MAX_SIZE   (200 * 1024)
#define SSTV_NAME_LEN   48
#define SSTV_DAC_FS     48000
#define SSTV_BUF_SIZE   AFSK_DAC_FRAME_SIZE   /* 2048 bytes */

QueueHandle_t s_sstv_queue = NULL;
static volatile bool s_sstv_abort = false;

// ---------------------------------------------------------------------------
// 256-entry uint8_t sine table (DC=128, amplitude ~127).
// Generated from round(128 + 127.5*sin(2*pi*i/256)).
// ---------------------------------------------------------------------------
static const uint8_t sin_table_256[256] = {
    128,131,134,137,140,144,147,150,153,156,159,162,165,168,171,174,
    177,180,183,185,188,191,194,196,199,201,204,206,209,211,214,216,
    218,220,222,225,227,229,230,232,234,236,237,239,240,242,243,245,
    246,247,248,249,250,251,252,252,253,254,254,255,255,255,255,255,
    255,255,255,255,255,255,254,254,253,252,252,251,250,249,248,247,
    246,245,243,242,240,239,237,236,234,232,230,229,227,225,222,220,
    218,216,214,211,209,206,204,201,199,196,194,191,188,185,183,180,
    177,174,171,168,165,162,159,156,153,150,147,144,140,137,134,131,
    128,125,122,119,116,112,109,106,103,100, 97, 94, 91, 88, 85, 82,
     79, 76, 73, 71, 68, 65, 62, 60, 57, 55, 52, 50, 47, 45, 42, 40,
     38, 36, 34, 31, 29, 27, 26, 24, 22, 20, 19, 17, 16, 14, 13, 11,
     10,  9,  8,  7,  6,  5,  4,  4,  3,  2,  2,  1,  1,  1,  1,  1,
      0,  1,  1,  1,  1,  1,  2,  2,  3,  4,  4,  5,  6,  7,  8,  9,
     10, 11, 13, 14, 16, 17, 19, 20, 22, 24, 26, 27, 29, 31, 34, 36,
     38, 40, 42, 45, 47, 50, 52, 55, 57, 60, 62, 65, 68, 71, 73, 76,
     79, 82, 85, 88, 91, 94, 97,100,103,106,109,112,116,119,122,125,
};

// ---------------------------------------------------------------------------
// Mode parameters table
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  vis_code;
    int      cols;
    int      rows;
    uint32_t sync_us;
    uint32_t sync_porch_us; /* post-sync porch: 3000 µs for Robot, 0 for others */
    uint32_t scan_us;       /* total scan per channel (R/G/B or Y) */
    uint32_t c_scan_us;     /* chroma scan per channel (Robot only) */
    uint32_t porch_us;      /* separator pulse duration / inter-channel porch */
    uint32_t sep_porch_us;  /* porch after separator before chroma (Robot: 1500 µs) */
    bool     is_ycbcr;
    bool     chroma_422;    /* Robot 72: Cb+Cr each line; Robot 36: alternating */
    bool     is_scottie;    /* Scottie line order: porch+G+porch+B+sync+porch+R */
} SstvModeParams;

/* Timing from the Robot/Martin/Scottie SSTV specifications.
 * Robot 36 per line: 9+3+88+4.5+1.5+44 = 150 ms × 240 = 36 s  (scanTime=150 in decoder)
 * Robot 72 per line: 9+3+138+4.5+1.5+69+4.5+1.5+69 = 300 ms × 240 = 72 s
 * Martin M1 per line: 4.862+0.572+146.432×3+0.572×2 = 445.9 ms × 256 ≈ 114 s
 * Scottie S1 per line: 1.5+138.24+1.5+138.24+9+1.5+138.24 = 428.2 ms × 256 ≈ 110 s */
static const SstvModeParams sstv_modes[5] = {
    /* MARTIN_M1 */ { 0xAC, 320, 256,  4862,    0, 146432,      0,  572,    0, false, false, false },
    /* MARTIN_M2 */ { 0x28, 320, 256,  4862,    0,  73216,      0,  572,    0, false, false, false },
    /* SCOTTIE_S1*/{ 0x3C, 320, 256,  9000,    0, 138240,      0, 1500,    0, false, false, true  },
    /* ROBOT_36  */ { 0x08, 320, 240,  9000, 3000,  88000,  44000, 4500, 1500, true,  false, false },
    /* ROBOT_72  */ { 0x0C, 320, 240,  9000, 3000, 138000,  69000, 4500, 1500, true,  true,  false },
};

// ---------------------------------------------------------------------------
// Tone generator
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t phase_acc;
    uint32_t phase_inc;
} ToneGen;

static inline uint32_t freq_to_inc(float f) {
    return (uint32_t)((double)f / SSTV_DAC_FS * 4294967296.0);
}

/* Global sample clock — carries fractional µs→sample error forward across all
 * tx_tone/tx_scan segments in the session.  Without this, each us_to_samples()
 * call truncates independently: Martin M1 porches lose 0.456 samples × 3 × 256
 * lines = 350 samples, plus scan error, totalling ~21 ms = ~15 px slant.
 * With s_sample_acc the total error across the entire TX is ≤ 1 sample. */
static int64_t  s_sample_acc = 0;

static uint32_t sclk_us(uint32_t us) {
    s_sample_acc += (int64_t)us * SSTV_DAC_FS;
    uint32_t n = (uint32_t)(s_sample_acc / 1000000LL);
    s_sample_acc %= 1000000LL;
    return n;
}

/* Shared DAC output buffer — only used from receive_audio_task.
 * s_dac_pos persists across tx_tone/tx_scan calls so samples from consecutive
 * short elements (sync, porch, scan) are packed into the same 2048-byte block.
 * Without this, each call would zero-pad to 2048 bytes and every 7-ms sync
 * pulse would actually take 42 ms, making Robot 36 run at ~72 s. */
static uint8_t  s_dac_buf[SSTV_BUF_SIZE];
static uint32_t s_dac_pos = 0;

static inline void dac_out(uint8_t s) {
    s_dac_buf[s_dac_pos++] = s;
    if (s_dac_pos == SSTV_BUF_SIZE) {
        afsk_write_dac_block(s_dac_buf, SSTV_BUF_SIZE, 2000);
        s_dac_pos = 0;
    }
}

static void dac_flush(void) {
    if (s_dac_pos > 0) {
        memset(s_dac_buf + s_dac_pos, 128, SSTV_BUF_SIZE - s_dac_pos);
        afsk_write_dac_block(s_dac_buf, SSTV_BUF_SIZE, 2000);
        s_dac_pos = 0;
    }
}

/* tx_tone takes duration in microseconds; sclk_us converts to sample count. */
static void tx_tone(ToneGen *g, float freq_hz, uint32_t us) {
    uint32_t n = sclk_us(us);
    g->phase_inc = freq_to_inc(freq_hz);
    for (uint32_t i = 0; i < n; i++) {
        g->phase_acc += g->phase_inc;
        dac_out(sin_table_256[g->phase_acc >> 24]);
    }
}

/* Transmit n_px pixels over exactly total_us microseconds.
 * sclk_us carries the fractional-sample error forward across segments. */
static void tx_scan(ToneGen *g, const uint8_t *ch, int n_px, uint32_t total_us) {
    uint32_t total_smp = sclk_us(total_us);
    uint32_t acc = 0;
    for (int x = 0; x < n_px; x++) {
        g->phase_inc = freq_to_inc(1500.0f + (ch[x] / 255.0f) * 800.0f);
        acc += total_smp;
        uint32_t px_smp = acc / (uint32_t)n_px;
        acc %= (uint32_t)n_px;
        for (uint32_t s = 0; s < px_smp; s++) {
            g->phase_acc += g->phase_inc;
            dac_out(sin_table_256[g->phase_acc >> 24]);
        }
    }
}

// ---------------------------------------------------------------------------
// VIS preamble
// ---------------------------------------------------------------------------
static void tx_vis(SstvMode mode) {
    ToneGen g = {0};
    uint8_t vis = sstv_modes[mode].vis_code;

    tx_tone(&g, 1900.0f, 300000);  /* leader */
    tx_tone(&g, 1200.0f,  10000);  /* break */
    tx_tone(&g, 1900.0f, 300000);  /* leader */
    tx_tone(&g, 1200.0f,  30000);  /* VIS start bit */

    int parity = 0;
    for (int b = 0; b < 7; b++) {
        int bit = (vis >> b) & 1;
        parity ^= bit;
        tx_tone(&g, bit ? 1300.0f : 1100.0f, 30000);
    }
    tx_tone(&g, parity ? 1300.0f : 1100.0f, 30000); /* parity */
    tx_tone(&g, 1200.0f, 30000);  /* VIS stop bit */
}

// ---------------------------------------------------------------------------
// Color conversion (BT.601 Q8 fixed-point)
// ---------------------------------------------------------------------------
static void rgb_to_ycbcr(uint8_t r, uint8_t g, uint8_t b,
                          uint8_t *Y, uint8_t *Cb, uint8_t *Cr) {
    /* BT.601 full-range (Y 0-255, Cb/Cr 0-255 centered at 128).
     * SSTV maps ch=0 → 1500 Hz (black) and ch=255 → 2300 Hz (white), so
     * full-range is required — studio swing (Y 16-235) would compress the
     * frequency span and produce wrong brightness and hue. */
    int32_t y  = (( 77 * r + 150 * g +  29 * b + 128) >> 8);
    int32_t cb = 128 + ((-43 * r -  85 * g + 128 * b + 128) >> 8);
    int32_t cr = 128 + ((128 * r - 107 * g -  21 * b + 128) >> 8);
    *Y  = (uint8_t)(y  < 0 ? 0 : y  > 255 ? 255 : y);
    *Cb = (uint8_t)(cb < 0 ? 0 : cb > 255 ? 255 : cb);
    *Cr = (uint8_t)(cr < 0 ? 0 : cr > 255 ? 255 : cr);
}

// ---------------------------------------------------------------------------
// Transmit one RGB line (Martin or Scottie)
// ---------------------------------------------------------------------------
static void tx_rgb_line(ToneGen *g, const SstvModeParams *mp,
                         const uint8_t *r_ch, const uint8_t *g_ch,
                         const uint8_t *b_ch) {
    if (mp->is_scottie) {
        tx_tone(g, 1500.0f, mp->porch_us);
        tx_scan(g, g_ch, mp->cols, mp->scan_us);
        tx_tone(g, 1500.0f, mp->porch_us);
        tx_scan(g, b_ch, mp->cols, mp->scan_us);
        tx_tone(g, 1200.0f, mp->sync_us);
        tx_tone(g, 1500.0f, mp->porch_us);
        tx_scan(g, r_ch, mp->cols, mp->scan_us);
    } else {
        /* Martin: GBR (green, blue, red) — el estándar y los decodificadores
         * esperan G → B → R en ese orden. Invertir produce un desplazamiento
         * cíclico de canales (rojo aparece como verde, etc.).
         * Línea: sync + porch + G + porch + B + porch + R + porch.
         * El cuarto porch (tras R) es imprescindible: completa la duración de
         * línea estándar 446.446 ms (= 4.862 + 4×0.572 + 3×146.432). Sin él
         * cada línea pierde 0.572 ms = 1.25 px (a 457.6 µs/px), lo que produce
         * exactamente 45° de slant sobre 256 líneas en Martin M1. */
        tx_tone(g, 1200.0f, mp->sync_us);
        tx_tone(g, 1500.0f, mp->porch_us);
        tx_scan(g, g_ch, mp->cols, mp->scan_us);
        tx_tone(g, 1500.0f, mp->porch_us);
        tx_scan(g, b_ch, mp->cols, mp->scan_us);
        tx_tone(g, 1500.0f, mp->porch_us);
        tx_scan(g, r_ch, mp->cols, mp->scan_us);
        tx_tone(g, 1500.0f, mp->porch_us);
    }
}

// ---------------------------------------------------------------------------
// Transmit one YCbCr line (Robot)
// ---------------------------------------------------------------------------
/* is_even: row % 2 == 0.
 * Robot 36 separator frequency encodes the chroma type so decoders can detect
 * even/odd without counting lines.  Convención mayoritaria (MMSSTV, QSSTV,
 * pySSTV y la espec original de Robot Research):
 *   1500 Hz separator → R-Y (Cr)
 *   2300 Hz separator → B-Y (Cb)
 * Aquí elegimos: línea par envía Cr tras 1500 Hz; línea impar envía Cb tras
 * 2300 Hz. La asignación opuesta intercambia rojo↔azul en el receptor.
 * After the separator there is always a 1500 Hz porch (sep_porch_us = 1500 µs)
 * before chroma data starts; without it the chroma arrives 1.5 ms early per
 * line and the image slants. */
static void tx_ycbcr_line(ToneGen *g, const SstvModeParams *mp,
                            const uint8_t *y_ch, const uint8_t *cb_ch,
                            const uint8_t *cr_ch, bool is_even) {
    int c_px = mp->cols / 2;

    tx_tone(g, 1200.0f, mp->sync_us);
    if (mp->sync_porch_us)
        tx_tone(g, 1500.0f, mp->sync_porch_us);
    tx_scan(g, y_ch, mp->cols, mp->scan_us);

    if (mp->chroma_422) {
        /* Robot 72: Cb then Cr every line */
        tx_tone(g, 1500.0f, mp->porch_us);
        if (mp->sep_porch_us)
            tx_tone(g, 1500.0f, mp->sep_porch_us);
        tx_scan(g, cb_ch, c_px, mp->c_scan_us);
        tx_tone(g, 1500.0f, mp->porch_us);
        if (mp->sep_porch_us)
            tx_tone(g, 1500.0f, mp->sep_porch_us);
        tx_scan(g, cr_ch, c_px, mp->c_scan_us);
    } else {
        /* Robot 36: la frecuencia del separador identifica qué croma sigue.
         *   1500 Hz → Cr (R-Y);  2300 Hz → Cb (B-Y). */
        if (is_even) {
            tx_tone(g, 1500.0f, mp->porch_us); /* sep 1500 Hz → Cr */
            if (mp->sep_porch_us)
                tx_tone(g, 1500.0f, mp->sep_porch_us);
            tx_scan(g, cr_ch, c_px, mp->c_scan_us);
        } else {
            tx_tone(g, 2300.0f, mp->porch_us); /* sep 2300 Hz → Cb */
            if (mp->sep_porch_us)
                tx_tone(g, 1500.0f, mp->sep_porch_us);
            tx_scan(g, cb_ch, c_px, mp->c_scan_us);
        }
    }
}

// ---------------------------------------------------------------------------
// TJpgDec integration
// ---------------------------------------------------------------------------
#define JPEG_ROW_BUF_HEIGHT  16   /* max MCU height for 4:2:0 */
#define JPEG_WORK_SIZE       8192 /* ROM TJpgDec with JD_FASTDECODE=2 needs ~6 KB */

typedef struct {
    FILE *f;
    uint8_t  row_buf[JPEG_ROW_BUF_HEIGHT][320 * 3];
    int      base_row;    /* image row index of row_buf[0] */
    bool     error;
    /* SSTV TX state */
    ToneGen       *tg;
    SstvMode       mode;
    const SstvModeParams *mp;
    uint8_t        cb_prev[160]; /* Robot 36: Cb from previous even line */
    int            lines_done;
} JpegCtx;

/* Contexto (~15,5 KB) y work area (8 KB) se reservan en heap durante la TX y se
   liberan al terminar: como .bss se comían la mitad del heap libre (47 KB tras
   enlazar) y dejaban al driver WiFi sin memoria para sus buffers TX — síntoma
   "wifi:m f null" repetido. Solo hay una TX SSTV a la vez. */
static JDEC     s_jdec;

static UINT jpeg_input_cb(JDEC *jdec, BYTE *buf, UINT nbytes) {
    JpegCtx *ctx = (JpegCtx *)jdec->device;
    if (!buf) {
        /* Seek forward nbytes */
        if (fseek(ctx->f, (long)nbytes, SEEK_CUR) != 0) ctx->error = true;
        return nbytes;
    }
    return (UINT)fread(buf, 1, nbytes, ctx->f);
}

static UINT jpeg_output_cb(JDEC *jdec, void *bitmap, JRECT *rect) {
    JpegCtx *ctx = (JpegCtx *)jdec->device;
    if (ctx->error || s_sstv_abort) return 0;

    const uint8_t *pix = (const uint8_t *)bitmap;
    int bw  = (int)(rect->right  - rect->left + 1);
    int bh  = (int)(rect->bottom - rect->top  + 1);
    /* Use mp->cols instead of jdec->width: the JDEC struct layout in ROM
     * may differ from the header, making jdec->width unreliable. */
    int img_w = ctx->mp->cols;

    if (ctx->lines_done == 0 && (int)rect->top == 0 && (int)rect->left == 0)
        ESP_LOGI(TAG, "output_cb first block: [%d,%d,%d,%d] img_w=%d",
                 rect->left, rect->top, rect->right, rect->bottom, img_w);

    /* row_buf is fixed at 320 px wide (JPEG_ROW_BUF_HEIGHT x 320*3). A JPEG
     * wider than the SSTV mode (e.g. an unresized camera photo uploaded
     * directly via the API) would make rect->left/bw index past that row,
     * corrupting the heap. Bail out instead of overflowing. */
    if (rect->left < 0 || bw <= 0 ||
        ((size_t)rect->left + (size_t)bw) * 3 > sizeof(ctx->row_buf[0])) {
        ESP_LOGE(TAG, "MCU block [%d,%d] out of bounds for %d-px row buffer — image too wide?",
                 rect->left, bw, img_w);
        ctx->error = true;
        return 0;
    }

    /* Copy MCU block pixels into row_buf. */
    for (int row = 0; row < bh; row++) {
        int img_row = (int)rect->top + row;
        int rel_row = img_row % JPEG_ROW_BUF_HEIGHT;
        uint8_t *dst = ctx->row_buf[rel_row] + (int)rect->left * 3;
        const uint8_t *src = pix + row * bw * 3;
        memcpy(dst, src, bw * 3);
    }

    /* When the rightmost MCU of this row band arrives, all rows are complete. */
    if ((int)rect->right + 1 == img_w) {
        int band_top    = (int)rect->top;
        int band_bottom = (int)rect->bottom;
        const SstvModeParams *mp = ctx->mp;

        for (int img_row = band_top; img_row <= band_bottom; img_row++) {
            if (img_row >= mp->rows) break;  /* don't transmit past image height */

            int rel_row = img_row % JPEG_ROW_BUF_HEIGHT;
            const uint8_t *rgb = ctx->row_buf[rel_row];

            if (mp->is_ycbcr) {
                /* Robot modes */
                uint8_t y_ch[320], cb_ch[160], cr_ch[160];
                for (int x = 0; x < mp->cols; x++) {
                    uint8_t Y, Cb, Cr;
                    rgb_to_ycbcr(rgb[x*3], rgb[x*3+1], rgb[x*3+2], &Y, &Cb, &Cr);
                    y_ch[x] = Y;
                    if (x % 2 == 0) {
                        uint8_t Y2, Cb2, Cr2;
                        int nx = (x + 1 < mp->cols) ? x + 1 : x;
                        rgb_to_ycbcr(rgb[nx*3], rgb[nx*3+1], rgb[nx*3+2], &Y2, &Cb2, &Cr2);
                        cb_ch[x/2] = (uint8_t)((Cb + Cb2) / 2);
                        cr_ch[x/2] = (uint8_t)((Cr + Cr2) / 2);
                    }
                }
                bool is_even = (img_row % 2 == 0);
                tx_ycbcr_line(ctx->tg, mp, y_ch, cb_ch, cr_ch, is_even);
            } else {
                /* RGB modes (Martin, Scottie) */
                uint8_t r_ch[320], g_ch[320], b_ch[320];
                for (int x = 0; x < mp->cols; x++) {
                    r_ch[x] = rgb[x*3+0];
                    g_ch[x] = rgb[x*3+1];
                    b_ch[x] = rgb[x*3+2];
                }
                tx_rgb_line(ctx->tg, mp, r_ch, g_ch, b_ch);
            }
            ctx->lines_done++;
            if (ctx->lines_done % 32 == 0)
                ESP_LOGI(TAG, "SSTV line %d/%d", ctx->lines_done, mp->rows);
        }
    }
    return 1;
}

// ---------------------------------------------------------------------------
// Test pattern transmit (no file — pixels generated in RAM)
// 8 SMPTE color bars (top 75%) + gray ramp (bottom 25%)
// ---------------------------------------------------------------------------
static const uint8_t s_bar_rgb[8][3] = {
    {255, 255, 255},  /* white   */
    {255, 255,   0},  /* yellow  */
    {  0, 255, 255},  /* cyan    */
    {  0, 255,   0},  /* green   */
    {255,   0, 255},  /* magenta */
    {255,   0,   0},  /* red     */
    {  0,   0, 255},  /* blue    */
    {  0,   0,   0},  /* black   */
};

static void sstv_transmit_test_pattern(SstvMode mode) {
    const SstvModeParams *mp = &sstv_modes[(int)mode];
    int bars_rows = (mp->rows * 3) / 4;   /* top 75% = color bars */

    s_sstv_abort = false;
    afsk_switch_to_tx();
    gpio_set_level(GPIO_PTT_OUT, 1);
    s_dac_pos = 0;
    s_sample_acc = 0;

    tx_vis(mode);

    ToneGen g = {0};
    /* Guard tone: prevents VIS stop bit (1200 Hz, 30 ms) from merging with the
     * first line sync (also 1200 Hz).  Without it the decoder sees one 39 ms
     * pulse instead of a 30 ms VIS stop + a 9 ms sync, missing the image start. */
    tx_tone(&g, 1500.0f, 10000);
    if (mp->is_scottie)
        tx_tone(&g, 1200.0f, mp->sync_us);

    for (int row = 0; row < mp->rows; row++) {
        if (s_sstv_abort) break;
        uint8_t r_ch[320], g_ch[320], b_ch[320];

        if (row < bars_rows) {
            /* Color bars */
            for (int x = 0; x < mp->cols; x++) {
                int bar = x * 8 / mp->cols;
                r_ch[x] = s_bar_rgb[bar][0];
                g_ch[x] = s_bar_rgb[bar][1];
                b_ch[x] = s_bar_rgb[bar][2];
            }
        } else {
            /* Gray ramp */
            for (int x = 0; x < mp->cols; x++) {
                uint8_t v = (uint8_t)((uint32_t)x * 255 / (mp->cols - 1));
                r_ch[x] = g_ch[x] = b_ch[x] = v;
            }
        }

        if (mp->is_ycbcr) {
            uint8_t y_ch[320], cb_ch[160], cr_ch[160];
            for (int x = 0; x < mp->cols; x++) {
                uint8_t Y, Cb, Cr;
                rgb_to_ycbcr(r_ch[x], g_ch[x], b_ch[x], &Y, &Cb, &Cr);
                y_ch[x] = Y;
                if (x % 2 == 0) {
                    int nx = (x + 1 < mp->cols) ? x + 1 : x;
                    uint8_t Y2, Cb2, Cr2;
                    rgb_to_ycbcr(r_ch[nx], g_ch[nx], b_ch[nx], &Y2, &Cb2, &Cr2);
                    cb_ch[x/2] = (uint8_t)((Cb + Cb2) / 2);
                    cr_ch[x/2] = (uint8_t)((Cr + Cr2) / 2);
                }
            }
            tx_ycbcr_line(&g, mp, y_ch, cb_ch, cr_ch, (row % 2 == 0));
        } else {
            tx_rgb_line(&g, mp, r_ch, g_ch, b_ch);
        }

        if ((row + 1) % 32 == 0)
            ESP_LOGI(TAG, "Test pattern line %d/%d", row + 1, mp->rows);
    }

    dac_flush();
    gpio_set_level(GPIO_PTT_OUT, 0);
    afsk_switch_to_rx();
    if (s_sstv_abort) {
        audio_stream_ws_send_text("{\"type\":\"sstv_aborted\"}");
        ESP_LOGI(TAG, "SSTV test pattern TX aborted");
    } else {
        audio_stream_ws_send_text("{\"type\":\"sstv_done\",\"name\":\"test_pattern\"}");
        ESP_LOGI(TAG, "SSTV test pattern TX done");
    }
}

// ---------------------------------------------------------------------------
// Main transmit function
// ---------------------------------------------------------------------------
void sstv_transmit(const SstvRequest *req) {
    if (req->is_test) {
        sstv_transmit_test_pattern(req->mode);
        return;
    }
    char path[80];
    snprintf(path, sizeof(path), "%s/%s", SSTV_DIR, req->name);
    ESP_LOGI(TAG, "SSTV TX start: %s mode=%d", path, (int)req->mode);

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s", path);
        return;
    }

    const SstvModeParams *mp = &sstv_modes[(int)req->mode];

    /* Reservar antes de levantar el PTT: si no hay heap, se aborta sin emitir. */
    JpegCtx *jctx  = (JpegCtx *)calloc(1, sizeof(JpegCtx));
    uint8_t *jwork = (uint8_t *)malloc(JPEG_WORK_SIZE);
    if (!jctx || !jwork) {
        ESP_LOGE(TAG, "sin heap para el decodificador JPEG (%u B necesarios)",
                 (unsigned)(sizeof(JpegCtx) + JPEG_WORK_SIZE));
        free(jctx);
        free(jwork);
        fclose(f);
        audio_stream_ws_send_text("{\"type\":\"sstv_aborted\"}");
        return;
    }

    s_sstv_abort = false;
    afsk_switch_to_tx();
    gpio_set_level(GPIO_PTT_OUT, 1);
    s_dac_pos = 0;
    s_sample_acc = 0;

    tx_vis(req->mode);

    ToneGen g = {0};
    tx_tone(&g, 1500.0f, 10000); /* guard: separate VIS stop from first sync */
    if (mp->is_scottie)
        tx_tone(&g, 1200.0f, mp->sync_us);

    /* Set up JPEG decode context (calloc ya lo dejó a cero). */
    jctx->f    = f;
    jctx->tg   = &g;
    jctx->mode = req->mode;
    jctx->mp   = mp;

    JRESULT rc = jd_prepare(&s_jdec, jpeg_input_cb, jwork, JPEG_WORK_SIZE, jctx);
    if (rc != JDR_OK) {
        ESP_LOGE(TAG, "jd_prepare failed: rc=%d (3=pool_small, 6=fmt, 8=unsupported)",
                 (int)rc);
        goto done;
    }
    ESP_LOGI(TAG, "jd_prepare OK, jdec.width=%d jdec.height=%d (expect %dx%d)",
             (int)s_jdec.width, (int)s_jdec.height, mp->cols, mp->rows);

    if ((int)s_jdec.width != mp->cols) {
        /* row_buf and every per-line channel buffer are sized for mp->cols
         * (320 px) exactly. A narrower image would silently transmit only
         * the VIS header (the row-complete check never fires); a wider one
         * is now rejected defensively inside jpeg_output_cb too. Refuse
         * up front with a clear error instead of either failure mode. */
        ESP_LOGE(TAG, "image width %d != mode width %d — resize before uploading",
                 (int)s_jdec.width, mp->cols);
        goto done;
    }

    /* Scale 0 = no scaling (output at full resolution). */
    rc = jd_decomp(&s_jdec, jpeg_output_cb, 0);
    ESP_LOGI(TAG, "jd_decomp rc=%d, lines_done=%d", (int)rc, jctx->lines_done);
    if (rc != JDR_OK && rc != JDR_INTR) {
        ESP_LOGE(TAG, "jd_decomp error: rc=%d", (int)rc);
    }

done:
    dac_flush();  /* send any remaining samples before dropping PTT */
    fclose(f);
    gpio_set_level(GPIO_PTT_OUT, 0);
    afsk_switch_to_rx();

    int lines_done = jctx->lines_done;
    free(jctx);
    free(jwork);

    if (s_sstv_abort) {
        audio_stream_ws_send_text("{\"type\":\"sstv_aborted\"}");
        ESP_LOGI(TAG, "SSTV TX aborted after %d lines", lines_done);
    } else {
        char ws_msg[96];
        snprintf(ws_msg, sizeof(ws_msg),
                 "{\"type\":\"sstv_done\",\"name\":\"%s\"}", req->name);
        audio_stream_ws_send_text(ws_msg);
        ESP_LOGI(TAG, "SSTV TX done: %d lines", lines_done);
    }
}

// ---------------------------------------------------------------------------
// sstv_dispatch_if_pending — called from receive_audio_task
// ---------------------------------------------------------------------------
void sstv_dispatch_if_pending(void) {
    if (!s_sstv_queue) return;
    SstvRequest req;
    if (xQueueReceive(s_sstv_queue, &req, 0) == pdTRUE)
        sstv_transmit(&req);
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------

/* ---- GET /api/sstv/list ---- */
static esp_err_t sstv_list_handler(httpd_req_t *req) {
    DIR *d = opendir(SSTV_DIR);
    if (!d) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }
    static char buf[1024];
    int pos = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "[");
    struct dirent *ent;
    bool first = true;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_type != DT_REG) continue;
        char fpath[270];
        snprintf(fpath, sizeof(fpath), "%s/%s", SSTV_DIR, ent->d_name);
        struct stat st;
        if (stat(fpath, &st) != 0) continue;
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "%s{\"name\":\"%s\",\"size\":%ld}",
                        first ? "" : ",", ent->d_name, (long)st.st_size);
        first = false;
    }
    closedir(d);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* ---- POST /api/sstv/send ---- */
static esp_err_t sstv_send_handler(httpd_req_t *req) {
    char body[200] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    const char *name = cJSON_GetStringValue(cJSON_GetObjectItem(root, "name"));
    const char *mode = cJSON_GetStringValue(cJSON_GetObjectItem(root, "mode"));
    if (!name || !mode) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing name or mode");
        return ESP_FAIL;
    }
    if (strchr(name, '/') || strchr(name, '\\') || strstr(name, "..")) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }
    char path[80];
    snprintf(path, sizeof(path), "%s/%s", SSTV_DIR, name);
    struct stat st;
    if (stat(path, &st) != 0) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Image not found");
        return ESP_FAIL;
    }
    SstvRequest sr = { .fmt = SSTV_FMT_JPEG, .mode = SSTV_MODE_MARTIN_M1 };
    strncpy(sr.name, name, sizeof(sr.name) - 1);
    if      (strcmp(mode, "martin_m1") == 0) sr.mode = SSTV_MODE_MARTIN_M1;
    else if (strcmp(mode, "martin_m2") == 0) sr.mode = SSTV_MODE_MARTIN_M2;
    else if (strcmp(mode, "scottie_s1") == 0) sr.mode = SSTV_MODE_SCOTTIE_S1;
    else if (strcmp(mode, "robot_36")  == 0) sr.mode = SSTV_MODE_ROBOT_36;
    else if (strcmp(mode, "robot_72")  == 0) sr.mode = SSTV_MODE_ROBOT_72;
    cJSON_Delete(root);
    if (xQueueSend(s_sstv_queue, &sr, 0) != pdTRUE) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TX busy");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"queued\"}");
    return ESP_OK;
}

/* ---- DELETE /api/sstv/image?name=<n> ---- */
static esp_err_t sstv_delete_handler(httpd_req_t *req) {
    char query[SSTV_NAME_LEN + 16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing name param");
        return ESP_FAIL;
    }
    char name[SSTV_NAME_LEN] = {0};
    if (httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing name param");
        return ESP_FAIL;
    }
    if (strchr(name, '/') || strstr(name, "..")) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid name");
        return ESP_FAIL;
    }
    char path[80];
    snprintf(path, sizeof(path), "%s/%s", SSTV_DIR, name);
    if (unlink(path) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"deleted\"}");
    return ESP_OK;
}

/* ---- POST /api/sstv/upload (multipart/form-data) ---- */

/* Streaming multipart state machine.
 * Form structure expected:
 *   Part 1: name="name"        → text value = desired filename
 *   Part 2: name="image"       → JPEG binary data
 */
#define MP_CARRY  80   /* overlap between chunks for boundary detection */

typedef enum {
    MP_FIRST_HDR,   /* scanning for \r\n\r\n of first part header */
    MP_NAME_VAL,    /* reading name value until \r\n--boundary */
    MP_SECOND_HDR,  /* scanning for \r\n\r\n of second part header */
    MP_IMAGE_DATA,  /* writing image bytes to SPIFFS */
    MP_DONE,
    MP_ERROR
} MpState;

static esp_err_t sstv_upload_handler(httpd_req_t *req) {
    if ((int)req->content_len > SSTV_MAX_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Image too large (max 200 KB)");
        return ESP_FAIL;
    }

    /* Extract boundary from Content-Type: multipart/form-data; boundary=XXX */
    char ct[128] = {0};
    if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing Content-Type");
        return ESP_FAIL;
    }
    char *bp = strstr(ct, "boundary=");
    if (!bp) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No boundary");
        return ESP_FAIL;
    }
    bp += 9; /* skip "boundary=" */
    char boundary[74] = {0};
    strncpy(boundary, bp, sizeof(boundary) - 1);
    /* Remove trailing whitespace/newline */
    for (int i = (int)strlen(boundary) - 1; i >= 0; i--) {
        if (boundary[i] == '\r' || boundary[i] == '\n' || boundary[i] == ' ')
            boundary[i] = '\0';
        else break;
    }
    int blen = (int)strlen(boundary);
    if (blen < 1 || blen > 70) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad boundary");
        return ESP_FAIL;
    }

    /* Build the end-of-part marker we look for: "\r\n--<boundary>" */
    char end_mark[80];
    int end_mark_len = snprintf(end_mark, sizeof(end_mark), "\r\n--%s", boundary);

    /* Check gallery is not full. */
    {
        DIR *d = opendir(SSTV_DIR);
        int cnt = 0;
        if (d) {
            struct dirent *e;
            while ((e = readdir(d)) != NULL) if (e->d_type == DT_REG) cnt++;
            closedir(d);
        }
        if (cnt >= SSTV_MAX_FILES) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "Gallery full (max 10 images)");
            return ESP_FAIL;
        }
    }

    /* --- Streaming parse --- */
    static uint8_t rbuf[512 + MP_CARRY];  /* chunk + carry overlap */
    char img_name[SSTV_NAME_LEN] = {0};
    FILE *out = NULL;
    int  out_bytes = 0;
    MpState state = MP_FIRST_HDR;
    int carry = 0;   /* bytes carried over from previous chunk in rbuf[0..carry-1] */
    int total_left = (int)req->content_len;
    bool done = false;
    bool ok   = true;

    while (!done && total_left > 0 && state != MP_ERROR && state != MP_DONE) {
        int to_recv = (int)sizeof(rbuf) - carry;
        if (to_recv > total_left) to_recv = total_left;
        int got = httpd_req_recv(req, (char *)(rbuf + carry), to_recv);
        if (got <= 0) { state = MP_ERROR; break; }
        total_left -= got;
        int buf_len = carry + got;
        int i = 0;

        /* Process the window rbuf[0..buf_len-1] */
        while (i < buf_len && state != MP_DONE && state != MP_ERROR) {
            switch (state) {
            case MP_FIRST_HDR: {
                /* Scan for \r\n\r\n */
                if (i + 3 < buf_len &&
                    rbuf[i]=='\r' && rbuf[i+1]=='\n' &&
                    rbuf[i+2]=='\r' && rbuf[i+3]=='\n') {
                    i += 4;
                    state = MP_NAME_VAL;
                } else {
                    i++;
                }
                break;
            }
            case MP_NAME_VAL: {
                /* Read until \r\n--boundary (this ends the first part) */
                if (i + end_mark_len <= buf_len &&
                    memcmp(rbuf + i, end_mark, end_mark_len) == 0) {
                    /* Name value is rbuf[name_start..i-1] but we already
                     * lost track of name_start. Simpler: name was already
                     * collected byte-by-byte below. */
                    i += end_mark_len;
                    state = MP_SECOND_HDR;
                } else {
                    /* Accumulate name chars, stop at CR/LF */
                    if (rbuf[i] != '\r' && rbuf[i] != '\n') {
                        int nlen = (int)strlen(img_name);
                        if (nlen < (int)sizeof(img_name) - 1)
                            img_name[nlen] = (char)rbuf[i];
                    }
                    i++;
                }
                break;
            }
            case MP_SECOND_HDR: {
                /* Scan for \r\n\r\n (end of second part headers) */
                if (i + 3 < buf_len &&
                    rbuf[i]=='\r' && rbuf[i+1]=='\n' &&
                    rbuf[i+2]=='\r' && rbuf[i+3]=='\n') {
                    i += 4;
                    /* Sanitize image name */
                    for (char *p = img_name; *p; p++) {
                        if (!isalnum((unsigned char)*p) &&
                            *p != '-' && *p != '_' && *p != '.')
                            *p = '_';
                    }
                    /* Force .jpg extension */
                    char *dot = strrchr(img_name, '.');
                    if (!dot)
                        strncat(img_name, ".jpg",
                                sizeof(img_name) - strlen(img_name) - 1);
                    if (img_name[0] == '\0')
                        strncpy(img_name, "image.jpg", sizeof(img_name) - 1);

                    char fpath[80];
                    snprintf(fpath, sizeof(fpath), "%s/%s", SSTV_DIR, img_name);
                    out = fopen(fpath, "wb");
                    if (!out) { state = MP_ERROR; break; }
                    state = MP_IMAGE_DATA;
                } else {
                    i++;
                }
                break;
            }
            case MP_IMAGE_DATA: {
                /* Write bytes, stopping when end_mark is found.
                 * Keep a look-ahead window of end_mark_len bytes. */
                int safe = buf_len - i - end_mark_len;
                if (safe > 0) {
                    /* Check for end_mark in [i..i+safe+end_mark_len-1] */
                    int found = -1;
                    for (int k = i; k < i + safe; k++) {
                        if (memcmp(rbuf + k, end_mark, end_mark_len) == 0) {
                            found = k;
                            break;
                        }
                    }
                    if (found >= 0) {
                        if (found > i && out) {
                            size_t want = (size_t)(found - i);
                            if (fwrite(rbuf + i, 1, want, out) != want)
                                { ok = false; done = true; }
                            else
                                out_bytes += (int)want;
                        }
                        state = MP_DONE;
                        done = true;
                        i = buf_len; /* consume rest */
                    } else {
                        if (out) {
                            if (fwrite(rbuf + i, 1, (size_t)safe, out) != (size_t)safe)
                                { ok = false; done = true; }
                            else
                                out_bytes += safe;
                        }
                        i += safe;
                    }
                } else {
                    /* Not enough data to safely check; carry remainder. */
                    i = buf_len; /* force chunk reload with carry */
                }
                break;
            }
            default:
                i = buf_len;
                break;
            }
        }

        /* Carry unprocessed tail into next iteration. */
        if (i < buf_len && state != MP_DONE && state != MP_ERROR) {
            carry = buf_len - i;
            if (carry > MP_CARRY) carry = MP_CARRY;
            memmove(rbuf, rbuf + (buf_len - carry), carry);
        } else {
            carry = 0;
        }
    }

    if (out) fclose(out);

    if (!ok || state == MP_ERROR || out_bytes == 0) {
        /* Remove partial file */
        if (img_name[0]) {
            char fpath[80];
            snprintf(fpath, sizeof(fpath), "%s/%s", SSTV_DIR, img_name);
            unlink(fpath);
        }
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Upload parse error");
        return ESP_FAIL;
    }

    char fpath[80];
    snprintf(fpath, sizeof(fpath), "%s/%s", SSTV_DIR, img_name);
    struct stat st;
    stat(fpath, &st);
    char resp[128];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"name\":\"%s\",\"size\":%ld}",
             img_name, (long)st.st_size);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    ESP_LOGI(TAG, "Upload OK: %s (%d bytes)", img_name, out_bytes);
    return ESP_OK;
}

/* ---- POST /api/sstv/stop ---- */
static esp_err_t sstv_stop_handler(httpd_req_t *req) {
    s_sstv_abort = true;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

/* ---- POST /api/sstv/test  {"mode":"robot_36"} ---- */
static esp_err_t sstv_test_handler(httpd_req_t *req) {
    char buf[128] = {0};
    int len = httpd_req_recv(req, buf, (int)sizeof(buf) - 1);
    SstvMode mode = SSTV_MODE_ROBOT_36;
    if (len > 0) {
        cJSON *root = cJSON_ParseWithLength(buf, len);
        if (root) {
            cJSON *m = cJSON_GetObjectItem(root, "mode");
            if (m && m->valuestring) {
                const char *ms = m->valuestring;
                if      (strcmp(ms, "martin_m1") == 0) mode = SSTV_MODE_MARTIN_M1;
                else if (strcmp(ms, "martin_m2") == 0) mode = SSTV_MODE_MARTIN_M2;
                else if (strcmp(ms, "scottie_s1") == 0) mode = SSTV_MODE_SCOTTIE_S1;
                else if (strcmp(ms, "robot_36")  == 0) mode = SSTV_MODE_ROBOT_36;
                else if (strcmp(ms, "robot_72")  == 0) mode = SSTV_MODE_ROBOT_72;
            }
            cJSON_Delete(root);
        }
    }
    SstvRequest sr = {.is_test = true, .mode = mode};
    strncpy(sr.name, "test_pattern", sizeof(sr.name) - 1);
    if (xQueueSend(s_sstv_queue, &sr, 0) != pdTRUE) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TX busy");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"queued\"}");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
void sstv_init(void) {
    /* Create /spiffs/sstv directory (no-op on SPIFFS but VFS needs it). */
    struct stat st;
    if (stat(SSTV_DIR, &st) != 0)
        mkdir(SSTV_DIR, 0755);

    s_sstv_queue = xQueueCreate(1, sizeof(SstvRequest));

    httpd_handle_t server = audio_stream_get_httpd();
    if (!server) {
        ESP_LOGE(TAG, "HTTP server not available — SSTV handlers not registered");
        return;
    }

    static const httpd_uri_t uri_list = {
        .uri = "/api/sstv/list", .method = HTTP_GET,
        .handler = sstv_list_handler
    };
    static const httpd_uri_t uri_send = {
        .uri = "/api/sstv/send", .method = HTTP_POST,
        .handler = sstv_send_handler
    };
    static const httpd_uri_t uri_delete = {
        .uri = "/api/sstv/image", .method = HTTP_DELETE,
        .handler = sstv_delete_handler
    };
    static const httpd_uri_t uri_upload = {
        .uri = "/api/sstv/upload", .method = HTTP_POST,
        .handler = sstv_upload_handler
    };
    static const httpd_uri_t uri_test = {
        .uri = "/api/sstv/test", .method = HTTP_POST,
        .handler = sstv_test_handler
    };
    static const httpd_uri_t uri_stop = {
        .uri = "/api/sstv/stop", .method = HTTP_POST,
        .handler = sstv_stop_handler
    };

    httpd_register_uri_handler(server, &uri_list);
    httpd_register_uri_handler(server, &uri_send);
    httpd_register_uri_handler(server, &uri_delete);
    httpd_register_uri_handler(server, &uri_upload);
    httpd_register_uri_handler(server, &uri_test);
    httpd_register_uri_handler(server, &uri_stop);

    ESP_LOGI(TAG, "SSTV initialized (queue=%p, httpd=%p)", s_sstv_queue, server);
}

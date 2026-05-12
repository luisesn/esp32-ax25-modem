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

#define TAG "sstv"

#define SSTV_DIR        "/spiffs/sstv"
#define SSTV_MAX_FILES  10
#define SSTV_MAX_SIZE   (200 * 1024)
#define SSTV_NAME_LEN   48
#define SSTV_DAC_FS     48000
#define SSTV_BUF_SIZE   AFSK_DAC_FRAME_SIZE   /* 2048 bytes */

QueueHandle_t s_sstv_queue = NULL;

// ---------------------------------------------------------------------------
// 256-entry uint8_t sine table (DC=128, amplitude ~127).
// Generated from round(128 + 127.5*sin(2*pi*i/256)).
// ---------------------------------------------------------------------------
static const uint8_t sin_table_256[256] = {
    128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
    176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,
    218,220,222,224,226,228,230,232,234,235,237,238,240,241,242,244,
    245,246,247,248,249,249,250,251,251,252,252,252,253,253,253,253,
    253,253,253,253,252,252,252,251,251,250,249,249,248,247,246,245,
    244,242,241,240,238,237,235,234,232,230,228,226,224,222,220,218,
    215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,
    173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,
    124,121,118,115,112,109,106,103,100, 97, 94, 90, 87, 85, 82, 79,
     76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 41, 39, 37,
     34, 32, 30, 28, 26, 24, 22, 20, 18, 17, 15, 14, 12, 11, 10,  8,
      7,  6,  5,  4,  3,  3,  2,  1,  1,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  1,  1,  2,  3,  3,  4,  5,  6,  7,  8, 10, 11,
     12, 14, 15, 17, 18, 20, 22, 24, 26, 28, 30, 32, 34, 37, 39, 41,
     44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76, 79, 82, 85,
     87, 90, 94, 97,100,103,106,109,112,115,118,121,124,128,122,125,
};

// ---------------------------------------------------------------------------
// Mode parameters table
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  vis_code;
    int      cols;
    int      rows;
    uint32_t sync_us;
    uint32_t scan_us;       /* total scan per channel (R/G/B or Y) */
    uint32_t c_scan_us;     /* chroma scan per channel (Robot only) */
    uint32_t porch_us;
    bool     is_ycbcr;
    bool     chroma_422;    /* Robot 72: Cb+Cr each line; Robot 36: alternating */
    bool     is_scottie;    /* Scottie line order: porch+G+porch+B+sync+porch+R */
} SstvModeParams;

static const SstvModeParams sstv_modes[5] = {
    /* MARTIN_M1 */ { 0xAC, 320, 256,  4862, 146432,      0,  572, false, false, false },
    /* MARTIN_M2 */ { 0x28, 320, 256,  4862,  73216,      0,  572, false, false, false },
    /* SCOTTIE_S1*/{ 0x3C, 320, 256,  9000, 138240,      0, 1500, false, false, true  },
    /* ROBOT_36  */ { 0x08, 320, 240,  7000,  88000,  44000, 4500, true,  false, false },
    /* ROBOT_72  */ { 0x0C, 320, 240,  7000,  88000,  44000, 4500, true,  true,  false },
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

static inline uint32_t us_to_samples(uint32_t us) {
    return (uint32_t)((uint64_t)us * SSTV_DAC_FS / 1000000UL);
}

/* Shared DAC output buffer — only used from receive_audio_task. */
static uint8_t s_dac_buf[SSTV_BUF_SIZE];

/* Transmit n_samples of a single frequency; writes full 2048-byte DAC blocks. */
static void tx_tone(ToneGen *g, float freq_hz, uint32_t n_samples) {
    g->phase_inc = freq_to_inc(freq_hz);
    uint32_t done = 0;
    while (done < n_samples) {
        uint32_t chunk = n_samples - done;
        if (chunk > SSTV_BUF_SIZE) chunk = SSTV_BUF_SIZE;
        for (uint32_t i = 0; i < chunk; i++) {
            g->phase_acc += g->phase_inc;
            s_dac_buf[i] = sin_table_256[g->phase_acc >> 24];
        }
        if (chunk < SSTV_BUF_SIZE)
            memset(s_dac_buf + chunk, 128, SSTV_BUF_SIZE - chunk);
        afsk_write_dac_block(s_dac_buf, SSTV_BUF_SIZE, 2000);
        done += chunk;
    }
}

/* Transmit n_px pixels; px_us = microseconds per pixel (Q0, integer).
 * Uses a sub-sample accumulator to prevent timing drift over a scan line. */
static void tx_scan(ToneGen *g, const uint8_t *ch, int n_px, uint32_t px_us) {
    uint32_t num = (uint64_t)px_us * SSTV_DAC_FS;
    uint32_t den = 1000000UL;
    uint32_t acc = 0;
    uint32_t pos = 0;

    for (int x = 0; x < n_px; x++) {
        g->phase_inc = freq_to_inc(1500.0f + (ch[x] / 255.0f) * 800.0f);
        acc += num;
        uint32_t px_smp = acc / den;
        acc %= den;

        for (uint32_t s = 0; s < px_smp; s++) {
            g->phase_acc += g->phase_inc;
            s_dac_buf[pos++] = sin_table_256[g->phase_acc >> 24];
            if (pos == SSTV_BUF_SIZE) {
                afsk_write_dac_block(s_dac_buf, SSTV_BUF_SIZE, 2000);
                pos = 0;
            }
        }
    }
    if (pos > 0) {
        memset(s_dac_buf + pos, 128, SSTV_BUF_SIZE - pos);
        afsk_write_dac_block(s_dac_buf, SSTV_BUF_SIZE, 2000);
    }
}

// ---------------------------------------------------------------------------
// VIS preamble
// ---------------------------------------------------------------------------
static void tx_vis(SstvMode mode) {
    ToneGen g = {0};
    uint8_t vis = sstv_modes[mode].vis_code;

    tx_tone(&g, 1900.0f, us_to_samples(300000));  /* leader */
    tx_tone(&g, 1200.0f, us_to_samples( 10000));  /* break */
    tx_tone(&g, 1900.0f, us_to_samples(300000));  /* leader */
    tx_tone(&g, 1200.0f, us_to_samples( 30000));  /* VIS start bit */

    int parity = 0;
    for (int b = 0; b < 7; b++) {
        int bit = (vis >> b) & 1;
        parity ^= bit;
        tx_tone(&g, bit ? 1300.0f : 1100.0f, us_to_samples(30000));
    }
    tx_tone(&g, parity ? 1300.0f : 1100.0f, us_to_samples(30000)); /* parity */
    tx_tone(&g, 1200.0f, us_to_samples( 30000));  /* VIS stop bit */
}

// ---------------------------------------------------------------------------
// Color conversion (BT.601 Q8 fixed-point)
// ---------------------------------------------------------------------------
static void rgb_to_ycbcr(uint8_t r, uint8_t g, uint8_t b,
                          uint8_t *Y, uint8_t *Cb, uint8_t *Cr) {
    int32_t y  =  16 + (( 66 * r + 129 * g +  25 * b + 128) >> 8);
    int32_t cb = 128 + ((-38 * r -  74 * g + 112 * b + 128) >> 8);
    int32_t cr = 128 + ((112 * r -  94 * g -  18 * b + 128) >> 8);
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
    uint32_t px_us = mp->scan_us / (uint32_t)mp->cols;
    if (mp->is_scottie) {
        tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));
        tx_scan(g, g_ch, mp->cols, px_us);
        tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));
        tx_scan(g, b_ch, mp->cols, px_us);
        tx_tone(g, 1200.0f, us_to_samples(mp->sync_us));
        tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));
        tx_scan(g, r_ch, mp->cols, px_us);
    } else {
        /* Martin */
        tx_tone(g, 1200.0f, us_to_samples(mp->sync_us));
        tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));
        tx_scan(g, r_ch, mp->cols, px_us);
        tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));
        tx_scan(g, g_ch, mp->cols, px_us);
        tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));
        tx_scan(g, b_ch, mp->cols, px_us);
    }
}

// ---------------------------------------------------------------------------
// Transmit one YCbCr line (Robot)
// ---------------------------------------------------------------------------
static void tx_ycbcr_line(ToneGen *g, const SstvModeParams *mp,
                            const uint8_t *y_ch, const uint8_t *cb_ch,
                            const uint8_t *cr_ch, bool send_cb) {
    /* Robot: sync + Y + porch + Cb_or_Cr */
    uint32_t y_px_us = mp->scan_us / (uint32_t)mp->cols;
    uint32_t c_px_us = mp->c_scan_us / ((uint32_t)mp->cols / 2);

    tx_tone(g, 1200.0f, us_to_samples(mp->sync_us));
    tx_scan(g, y_ch, mp->cols, y_px_us);
    tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));

    if (mp->chroma_422) {
        /* Robot 72: Cb + Cr each line */
        tx_scan(g, cb_ch, mp->cols / 2, c_px_us);
        tx_tone(g, 1500.0f, us_to_samples(mp->porch_us));
        tx_scan(g, cr_ch, mp->cols / 2, c_px_us);
    } else {
        /* Robot 36: Cb on even lines, Cr on odd lines */
        if (send_cb)
            tx_scan(g, cb_ch, mp->cols / 2, c_px_us);
        else
            tx_scan(g, cr_ch, mp->cols / 2, c_px_us);
    }
}

// ---------------------------------------------------------------------------
// TJpgDec integration
// ---------------------------------------------------------------------------
#define JPEG_ROW_BUF_HEIGHT  16   /* max MCU height for 4:2:0 */
#define JPEG_WORK_SIZE       4096

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

/* Work area and context are static — only one SSTV TX at a time. */
static JpegCtx  s_jctx;
static uint8_t  s_jwork[JPEG_WORK_SIZE];
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
    if (ctx->error) return 0;

    const uint8_t *pix = (const uint8_t *)bitmap;
    int bw  = (int)(rect->right  - rect->left + 1);
    int bh  = (int)(rect->bottom - rect->top  + 1);
    int img_w = (int)jdec->width;

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
                if (mp->chroma_422) {
                    tx_ycbcr_line(ctx->tg, mp, y_ch, cb_ch, cr_ch, true);
                } else {
                    bool even = (img_row % 2 == 0);
                    if (even) {
                        memcpy(ctx->cb_prev, cb_ch, 160);
                        tx_ycbcr_line(ctx->tg, mp, y_ch, cb_ch, cr_ch, true);
                    } else {
                        tx_ycbcr_line(ctx->tg, mp, y_ch, ctx->cb_prev, cr_ch, false);
                    }
                }
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
// Main transmit function
// ---------------------------------------------------------------------------
void sstv_transmit(const SstvRequest *req) {
    char path[80];
    snprintf(path, sizeof(path), "%s/%s", SSTV_DIR, req->name);
    ESP_LOGI(TAG, "SSTV TX start: %s mode=%d", path, (int)req->mode);

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s", path);
        return;
    }

    const SstvModeParams *mp = &sstv_modes[(int)req->mode];

    afsk_switch_to_tx();
    gpio_set_level(GPIO_PTT_OUT, 1);

    tx_vis(req->mode);

    /* Scottie needs an initial sync pulse before line 0. */
    ToneGen g = {0};
    if (mp->is_scottie) {
        tx_tone(&g, 1200.0f, us_to_samples(mp->sync_us));
    }

    /* Set up JPEG decode context. */
    memset(&s_jctx, 0, sizeof(s_jctx));
    s_jctx.f    = f;
    s_jctx.tg   = &g;
    s_jctx.mode = req->mode;
    s_jctx.mp   = mp;

    JRESULT rc = jd_prepare(&s_jdec, jpeg_input_cb, s_jwork, sizeof(s_jwork), &s_jctx);
    if (rc != JDR_OK) {
        ESP_LOGE(TAG, "jd_prepare failed: %d", (int)rc);
        goto done;
    }

    /* Scale 0 = no scaling (output at full resolution). */
    rc = jd_decomp(&s_jdec, jpeg_output_cb, 0);
    if (rc != JDR_OK && rc != JDR_INTR) {
        ESP_LOGE(TAG, "jd_decomp failed: %d", (int)rc);
    }

done:
    fclose(f);
    gpio_set_level(GPIO_PTT_OUT, 0);
    afsk_switch_to_rx();

    char ws_msg[96];
    snprintf(ws_msg, sizeof(ws_msg),
             "{\"type\":\"sstv_done\",\"name\":\"%s\"}", req->name);
    audio_stream_ws_send_text(ws_msg);
    ESP_LOGI(TAG, "SSTV TX done: %d lines", s_jctx.lines_done);
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
                        if (found > i && out)
                            out_bytes += (int)fwrite(rbuf + i, 1, found - i, out);
                        state = MP_DONE;
                        done = true;
                        i = buf_len; /* consume rest */
                    } else {
                        if (out)
                            out_bytes += (int)fwrite(rbuf + i, 1, safe, out);
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

    if (state == MP_ERROR || out_bytes == 0) {
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

    httpd_register_uri_handler(server, &uri_list);
    httpd_register_uri_handler(server, &uri_send);
    httpd_register_uri_handler(server, &uri_delete);
    httpd_register_uri_handler(server, &uri_upload);

    ESP_LOGI(TAG, "SSTV initialized (queue=%p, httpd=%p)", s_sstv_queue, server);
}

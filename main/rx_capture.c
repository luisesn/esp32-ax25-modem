// Captura de audio RX a WAV (8 bit mono, 9600 Hz). Ver rx_capture.h.
#include "rx_capture.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"

static const char *TAG = "rx_capture";

#define CAP_RATE      9600
#define CAP_RING_SIZE 8192          // ~0,85 s de audio 8 bit; sólo existe durante la captura
#define CAP_BLK       64

static volatile StreamBufferHandle_t s_sb = NULL;
static volatile bool s_active = false;   // el hook sólo escribe si true
static volatile bool s_in_push = false;  // handshake con el handler antes de liberar
static volatile bool s_busy = false;     // una captura a la vez
static volatile uint32_t s_dropped = 0;

void rx_capture_push(const int8_t *s, size_t n)
{
    if (!s_active) return;
    s_in_push = true;
    StreamBufferHandle_t sb = s_sb;
    if (s_active && sb) {
        // Convierte a 8 bit sin signo (WAV) por bloques en pila.
        uint8_t blk[CAP_BLK];
        while (n) {
            size_t c = n > CAP_BLK ? CAP_BLK : n;
            for (size_t i = 0; i < c; i++) blk[i] = (uint8_t)(s[i] + 128);
            if (xStreamBufferSend(sb, blk, c, 0) != c) s_dropped += c;
            s += c; n -= c;
        }
    }
    s_in_push = false;
}

static void put32(uint8_t *p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }

static void wav_header(uint8_t h[44], uint32_t data_len)
{
    memcpy(h, "RIFF", 4); put32(h + 4, 36 + data_len);
    memcpy(h + 8, "WAVEfmt ", 8); put32(h + 16, 16);
    h[20] = 1; h[21] = 0;              // PCM
    h[22] = 1; h[23] = 0;              // mono
    put32(h + 24, CAP_RATE);
    put32(h + 28, CAP_RATE);           // byte rate (8 bit mono)
    h[32] = 1; h[33] = 0;              // block align
    h[34] = 8; h[35] = 0;              // bits
    memcpy(h + 36, "data", 4); put32(h + 40, data_len);
}

static int query_int(const char *q, const char *key, int def)
{
    char v[12];
    if (httpd_query_key_value(q, key, v, sizeof(v)) != ESP_OK) return def;
    return atoi(v);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    char q[48] = "";
    httpd_req_get_url_query_str(req, q, sizeof(q));
    int secs = query_int(q, "secs", 5);
    int rate = query_int(q, "rate", CAP_RATE);
    if (secs < 1 || secs > 30) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "secs debe estar entre 1 y 30");
        return ESP_OK;
    }
    if (rate != CAP_RATE) {
        // 48000 exigiría exponer las muestras crudas del ADC desde AFSK.cpp
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "solo rate=9600 soportado");
        return ESP_OK;
    }
    if (s_busy) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_send(req, "captura en curso", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    s_busy = true;

    StreamBufferHandle_t sb = xStreamBufferCreate(CAP_RING_SIZE, 1);
    uint8_t *buf = malloc(512);
    if (!sb || !buf) {
        if (sb) vStreamBufferDelete(sb);
        free(buf);
        s_busy = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "sin memoria");
        return ESP_OK;
    }

    uint32_t total = (uint32_t)secs * CAP_RATE;
    uint32_t sent = 0;
    esp_err_t err = ESP_OK;
    s_dropped = 0;
    s_sb = sb;
    s_active = true;

    httpd_resp_set_type(req, "audio/wav");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    uint8_t hdr[44];
    wav_header(hdr, total);
    err = httpd_resp_send_chunk(req, (const char *)hdr, sizeof(hdr));

    TickType_t last_data = xTaskGetTickCount();
    while (err == ESP_OK && sent < total) {
        size_t want = total - sent;
        if (want > 512) want = 512;
        size_t n = xStreamBufferReceive(sb, buf, want, pdMS_TO_TICKS(100));
        if (n) {
            last_data = xTaskGetTickCount();
            err = httpd_resp_send_chunk(req, (const char *)buf, n);
            sent += n;
        } else if (xTaskGetTickCount() - last_data > pdMS_TO_TICKS(3000)) {
            break;   // sin audio (RX parado): rellena con silencio
        }
    }

    // Detiene el productor y espera a que salga del hook antes de liberar.
    s_active = false;
    for (int i = 0; i < 100 && s_in_push; i++) vTaskDelay(pdMS_TO_TICKS(2));
    s_sb = NULL;
    vTaskDelay(pdMS_TO_TICKS(5));
    vStreamBufferDelete(sb);

    // Si faltaron muestras, completa con silencio para respetar la cabecera.
    if (err == ESP_OK && sent < total) {
        memset(buf, 0x80, 512);
        while (err == ESP_OK && sent < total) {
            uint32_t c = total - sent > 512 ? 512 : total - sent;
            err = httpd_resp_send_chunk(req, (const char *)buf, c);
            sent += c;
        }
    }
    if (err == ESP_OK) httpd_resp_send_chunk(req, NULL, 0);
    if (s_dropped) ESP_LOGW(TAG, "captura: %u muestras perdidas", (unsigned)s_dropped);
    free(buf);
    s_busy = false;
    return err;
}

void rx_capture_register(httpd_handle_t srv)
{
    static const httpd_uri_t u = { .uri = "/api/rx/capture", .method = HTTP_GET, .handler = capture_handler };
    if (srv) httpd_register_uri_handler(srv, &u);
}

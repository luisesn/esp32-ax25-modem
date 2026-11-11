#include "msg_retry.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "LibAPRS-esp32-i2s/src/LibAPRS.h"
#include "audio_stream.h"

static const char *TAG = "msg_retry";

typedef struct {
    bool     used;
    char     call[7];
    int8_t   ssid;
    int16_t  id;
    char     text[68];
    uint8_t  attempts;      // envíos realizados (el inicial cuenta)
    TickType_t next;        // instante del próximo reintento / vencimiento
} pending_t;

static pending_t         s_tab[MSG_RETRY_MAX_ENTRIES];
static SemaphoreHandle_t s_mtx;
static TaskHandle_t      s_task;
static const uint16_t    s_waits_s[MSG_RETRY_MAX_ATTEMPTS] = MSG_RETRY_WAITS_S;

static void ws_event(const char *type, const char *call, int ssid, int id, int attempt)
{
    char j[112];
    snprintf(j, sizeof(j),
             "{\"type\":\"%s\",\"to\":\"%s\",\"ssid\":%d,\"id\":%d,\"attempt\":%d}",
             type, call, ssid, id, attempt);
    audio_stream_ws_send_text(j);
}

static void retry_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        for (int i = 0; i < MSG_RETRY_MAX_ENTRIES; i++) {
            pending_t e = {0};
            bool due = false, give_up = false;
            xSemaphoreTake(s_mtx, portMAX_DELAY);
            pending_t *p = &s_tab[i];
            if (p->used && (int32_t)(xTaskGetTickCount() - p->next) >= 0) {
                if (p->attempts >= MSG_RETRY_MAX_ATTEMPTS) {
                    give_up = true;
                    e = *p;
                    p->used = false;
                } else {
                    e = *p;
                    due = true;
                }
            }
            xSemaphoreGive(s_mtx);

            if (give_up) {
                ESP_LOGW(TAG, "sin ACK de %s-%d {%03d}: se abandona", e.call, e.ssid, e.id);
                ws_event("msg_failed", e.call, e.ssid, e.id, e.attempts);
                audio_stream_log_sent(e.call, e.ssid, e.text, e.id, "failed");
            } else if (due) {
                // Cola TX no bloqueante; si está llena no se cuenta el intento.
                bool ok = APRS_queue_msg_id(e.call, e.ssid, e.text, e.id) == 0;
                xSemaphoreTake(s_mtx, portMAX_DELAY);
                // Puede haberse cancelado (ACK) o reutilizado la ranura mientras tanto.
                if (s_tab[i].used && s_tab[i].id == e.id &&
                    strcmp(s_tab[i].call, e.call) == 0 && s_tab[i].ssid == e.ssid) {
                    if (ok) {
                        s_tab[i].attempts++;
                        s_tab[i].next = xTaskGetTickCount() +
                            pdMS_TO_TICKS((uint32_t)s_waits_s[s_tab[i].attempts - 1] * 1000);
                    } else {
                        s_tab[i].next = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
                    }
                }
                xSemaphoreGive(s_mtx);
                if (ok) {
                    ESP_LOGI(TAG, "reintento %s-%d {%03d} (intento %d)", e.call, e.ssid, e.id, e.attempts + 1);
                    ws_event("msg_retry", e.call, e.ssid, e.id, e.attempts + 1);
                }
            }
        }
    }
}

void msg_retry_init(void)
{
    if (s_mtx) return;
    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) { ESP_LOGE(TAG, "sin mutex"); return; }
    if (xTaskCreate(retry_task, "msg_retry", 4096, NULL, 3, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "no se pudo crear la tarea");
        s_task = NULL;
    }
}

bool msg_retry_track(const char *to_call, int to_ssid, int id, const char *text)
{
    if (!s_mtx || !s_task || !to_call || !text || id < 0) return false;
    bool stored = false;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < MSG_RETRY_MAX_ENTRIES; i++) {
        pending_t *p = &s_tab[i];
        // Mismo destino+id (el contador de ids dio la vuelta): se sustituye.
        if (p->used && p->id == id && p->ssid == to_ssid &&
            strncasecmp(p->call, to_call, 6) == 0) { slot = i; break; }
        if (!p->used && slot < 0) slot = i;
    }
    if (slot >= 0) {
        pending_t *p = &s_tab[slot];
        memset(p, 0, sizeof(*p));
        strncpy(p->call, to_call, sizeof(p->call) - 1);
        p->ssid = (int8_t)to_ssid;
        p->id = (int16_t)id;
        strncpy(p->text, text, sizeof(p->text) - 1);
        p->attempts = 1;
        p->next = xTaskGetTickCount() + pdMS_TO_TICKS((uint32_t)s_waits_s[0] * 1000);
        p->used = true;
        stored = true;
    }
    xSemaphoreGive(s_mtx);
    if (stored) audio_stream_log_sent(to_call, to_ssid, text, id, "pending");
    return stored;
}

static void on_response(const char *from_call, int from_ssid, const char *id_str, const char *state)
{
    if (!s_mtx || !from_call || !id_str || !*id_str) return;
    for (const char *c = id_str; *c; c++) if (*c < '0' || *c > '9') return;
    int id = atoi(id_str);
    pending_t done = {0};
    bool acked = false;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (int i = 0; i < MSG_RETRY_MAX_ENTRIES; i++) {
        pending_t *p = &s_tab[i];
        if (p->used && p->id == id && p->ssid == from_ssid &&
            strncasecmp(p->call, from_call, 6) == 0) {
            done = *p; acked = true;
            p->used = false;
            ESP_LOGI(TAG, "ACK de %s-%d {%03d}: seguimiento cancelado", p->call, p->ssid, id);
        }
    }
    xSemaphoreGive(s_mtx);
    if (acked) audio_stream_log_sent(done.call, done.ssid, done.text, done.id, state);
}

void msg_retry_on_ack(const char *from_call, int from_ssid, const char *id_str)
{ on_response(from_call, from_ssid, id_str, "acked"); }

/* REJ del destinatario: se deja de reintentar y se registra como rechazado. */
void msg_retry_on_rej(const char *from_call, int from_ssid, const char *id_str)
{ on_response(from_call, from_ssid, id_str, "rejected"); }

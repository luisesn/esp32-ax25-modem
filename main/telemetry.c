// Telemetría APRS periódica. Ver telemetry.h.
#include "telemetry.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "LibAPRS.h"
#include "rx_stats.h"

static const char *TAG = "telemetry";

#define DEFS_PERIOD_S  (6 * 3600)

static volatile bool     s_enabled = false;
static volatile uint32_t s_period_s = 900;
static volatile uint32_t s_rx_cnt = 0, s_tx_cnt = 0;
static TaskHandle_t      s_task = NULL;

void telemetry_note_rx(void) { s_rx_cnt++; }
void telemetry_note_tx(void) { s_tx_cnt++; }

static int clamp255(uint32_t v) { return v > 255 ? 255 : (int)v; }

// Encola un paquete APRS cuyo info empieza por ':' (mensaje) o 'T' (telemetría)
static void send_def(const char *addr9, const char *body)
{
    char info[100];
    snprintf(info, sizeof(info), ":%s:%s", addr9, body);
    APRS_queue_beacon(info);
    vTaskDelay(pdMS_TO_TICKS(5000));   // espaciado: la cola TX tiene pocos huecos
}

static void send_defs(void)
{
    char call[8] = {0};
    int ssid = 0;
    APRS_getCallsign(call, &ssid);
    char name[24];
    if (ssid > 0) snprintf(name, sizeof(name), "%s-%d", call, ssid);
    else snprintf(name, sizeof(name), "%s", call);
    char addr[16];
    snprintf(addr, sizeof(addr), "%-9.9s", name);

    send_def(addr, "PARM.Heap,RxMin,TxMin,CrcErr,UpH");
    send_def(addr, "UNIT.KB,fr/m,fr/m,err,h");
    send_def(addr, "EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0");
    send_def(addr, "BITS.11111111,Telemetria ESP32 APRS");
}

static void telemetry_task(void *arg)
{
    (void)arg;
    uint32_t seq = 0;
    uint32_t last_defs = 0;       // s desde arranque del último envío de defs
    int64_t  last_t = esp_timer_get_time();
    uint32_t last_crc = rx_stats_v2.frames_crc_err;
    uint32_t last_rx = s_rx_cnt, last_tx = s_tx_cnt;
    bool first = true;

    // Espera inicial para que WiFi/GPS/baliza arranquen y el canal se estabilice.
    vTaskDelay(pdMS_TO_TICKS(30000));

    while (s_enabled) {
        uint32_t now_s = (uint32_t)(esp_timer_get_time() / 1000000);
        if (first || now_s - last_defs >= DEFS_PERIOD_S) {
            send_defs();
            last_defs = now_s;
            first = false;
        }

        int64_t t = esp_timer_get_time();
        uint32_t dt_s = (uint32_t)((t - last_t) / 1000000);
        if (dt_s < 1) dt_s = 1;
        last_t = t;
        uint32_t rx = s_rx_cnt, tx = s_tx_cnt, crc = rx_stats_v2.frames_crc_err;
        uint32_t rx_min = (uint32_t)(((uint64_t)(rx - last_rx) * 60 + dt_s / 2) / dt_s);
        uint32_t tx_min = (uint32_t)(((uint64_t)(tx - last_tx) * 60 + dt_s / 2) / dt_s);
        uint32_t crc_d  = crc - last_crc;
        last_rx = rx; last_tx = tx; last_crc = crc;

        char pkt[64];
        snprintf(pkt, sizeof(pkt), "T#%03u,%03d,%03d,%03d,%03d,%03d,11111111",
                 (unsigned)seq,
                 clamp255(esp_get_free_heap_size() / 1024),
                 clamp255(rx_min), clamp255(tx_min), clamp255(crc_d),
                 clamp255(now_s / 3600));
        APRS_queue_beacon(pkt);
        ESP_LOGI(TAG, "%s", pkt);
        seq = (seq + 1) % 1000;

        // Espera troceada para reaccionar a cambios de period_s / enabled.
        uint32_t waited = 0;
        while (s_enabled && waited < s_period_s) { vTaskDelay(pdMS_TO_TICKS(1000)); waited++; }
    }
    s_task = NULL;
    vTaskDelete(NULL);
}

static void apply_cfg(cJSON *cfg)
{
    bool en = false;
    uint32_t per = 900;
    cJSON *t = cfg ? cJSON_GetObjectItem(cfg, "telemetry") : NULL;
    if (t) {
        cJSON *j = cJSON_GetObjectItem(t, "enabled");
        if (cJSON_IsBool(j)) en = cJSON_IsTrue(j);
        j = cJSON_GetObjectItem(t, "period_s");
        if (cJSON_IsNumber(j) && j->valueint >= 60) per = (uint32_t)j->valueint;
    }
    s_period_s = per;
    s_enabled = en;
    if (en && !s_task) {
        if (xTaskCreate(telemetry_task, "telemetry", 3072, NULL, 3, &s_task) != pdPASS) {
            ESP_LOGE(TAG, "no se pudo crear la tarea");
            s_task = NULL;
            s_enabled = false;
        }
    }
}

void telemetry_init(cJSON *cfg) { apply_cfg(cfg); }
void telemetry_reload_config(cJSON *cfg) { apply_cfg(cfg); }

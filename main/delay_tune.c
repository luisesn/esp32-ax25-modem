// Auto-tuner for aprs.post_rx_tx_delay_ms.
//
// For each test delay D (150..2000 ms in 150 ms steps):
//   1. Set post_rx_tx_delay_ms = D (in-memory only).
//   2. Send TUNE_PINGS_PER_STEP ICMP pings to ip.remote_addr.
//      IMPORTANT: interval_ms must be LESS than D so that each ping is already
//      queued when the inhibit window expires.  With a small interval the actual
//      inter-ping TX spacing is controlled by the inhibit window (≈ RTT + D),
//      not by esp_ping's timer.  With a large interval (≥ D) the inhibit window
//      has already elapsed before the next ping is queued, so every D value
//      looks identical — all 100 %.
//   3. Broadcast tune_step WebSocket event with recv count, success %, avg RTT.
//   4. Wait TUNE_PAUSE_S seconds (with 10-s countdown broadcasts).
// After all steps, restore the original delay and broadcast tune_done with the
// minimum D that reached TUNE_SUCCESS_THRESHOLD success rate.

#include "delay_tune.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "ping/ping_sock.h"
#include "lwip/ip_addr.h"

#include "audio_stream.h"
#include "ax25ip.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"
#include "aux_config.h"

#define TAG "delay_tune"

#define TUNE_MIN_MS          50
#define TUNE_MAX_MS          2000
#define TUNE_STEP_MS         150
#define TUNE_PINGS_PER_STEP  5
// interval_ms must be well below the smallest D (150 ms) so the inhibit window
// is always the bottleneck, not esp_ping's own timer.
#define TUNE_PING_INTERVAL_MS 100
#define TUNE_PING_TIMEOUT_MS  5000  // per-ping timeout
// Ceiling for the whole session: pings are spaced by (RTT + D) ≈ 0.5–3 s each.
// 5 pings × 3 s max + timeout overhead = ~25 s.
#define TUNE_PING_WAIT_MS    30000
#define TUNE_PAUSE_S         30
#define TUNE_SUCCESS_THRESHOLD 0.80f  // >= 80% = "works"

#define NUM_STEPS  ((TUNE_MAX_MS - TUNE_MIN_MS) / TUNE_STEP_MS + 1)

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static volatile bool    s_running  = false;
static volatile bool    s_abort    = false;
static TaskHandle_t     s_task     = NULL;
static char             s_remote_ip[24] = {};

// Ping counters — written from the esp_ping callback task,
// read from the tune task after the task-notify synchronisation point.
static volatile uint32_t s_ping_recv;
static volatile uint32_t s_ping_total_rtt_ms;

// ---------------------------------------------------------------------------
// esp_ping callbacks
// ---------------------------------------------------------------------------

static void on_ping_success(esp_ping_handle_t hdl, void *args)
{
    uint32_t rtt = 0;
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &rtt, sizeof(rtt));
    s_ping_recv++;
    s_ping_total_rtt_ms += rtt;
}

static void on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    (void)hdl; (void)args;
}

static void on_ping_end(esp_ping_handle_t hdl, void *args)
{
    (void)hdl;
    xTaskNotifyGive((TaskHandle_t)args);
}

// ---------------------------------------------------------------------------
// Run one ping round, return false on session-create failure
// ---------------------------------------------------------------------------

static bool ping_step(uint32_t *out_recv, uint32_t *out_avg_rtt_ms)
{
    s_ping_recv         = 0;
    s_ping_total_rtt_ms = 0;

    ip_addr_t target;
    if (!ipaddr_aton(s_remote_ip, &target)) {
        ESP_LOGE(TAG, "invalid remote IP: %s", s_remote_ip);
        *out_recv     = 0;
        *out_avg_rtt_ms = 0;
        return false;
    }

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr   = target;
    cfg.count         = TUNE_PINGS_PER_STEP;
    cfg.interval_ms   = TUNE_PING_INTERVAL_MS;
    cfg.timeout_ms    = TUNE_PING_TIMEOUT_MS;
    cfg.task_stack_size = 2048;
    cfg.task_prio       = 2;

    esp_ping_callbacks_t cbs = {
        .cb_args        = xTaskGetCurrentTaskHandle(),
        .on_ping_success = on_ping_success,
        .on_ping_timeout = on_ping_timeout,
        .on_ping_end     = on_ping_end,
    };

    esp_ping_handle_t hdl;
    if (esp_ping_new_session(&cfg, &cbs, &hdl) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ping_new_session failed");
        *out_recv     = 0;
        *out_avg_rtt_ms = 0;
        return false;
    }

    esp_ping_start(hdl);
    // Wait for on_ping_end to notify us (generous timeout)
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TUNE_PING_WAIT_MS));
    esp_ping_stop(hdl);
    esp_ping_delete_session(hdl);

    *out_recv       = s_ping_recv;
    *out_avg_rtt_ms = (s_ping_recv > 0) ? (s_ping_total_rtt_ms / s_ping_recv) : 0;
    return true;
}

// ---------------------------------------------------------------------------
// Sweep task
// ---------------------------------------------------------------------------

static void tune_task(void *arg)
{
    (void)arg;

    uint32_t original_delay = afsk_get_post_rx_tx_delay_ms();
    char buf[128];

    // Announce sweep start
    snprintf(buf, sizeof(buf),
        "{\"type\":\"tune_start\","
        "\"from_ms\":%d,\"to_ms\":%d,\"step_ms\":%d,"
        "\"count\":%d,\"thr_pct\":%.0f}",
        TUNE_MIN_MS, TUNE_MAX_MS, TUNE_STEP_MS,
        TUNE_PINGS_PER_STEP, TUNE_SUCCESS_THRESHOLD * 100.0f);
    audio_stream_ws_send_text(buf);
    ESP_LOGI(TAG, "sweep start: %d..%d ms step %d  remote=%s",
             TUNE_MIN_MS, TUNE_MAX_MS, TUNE_STEP_MS, s_remote_ip);

    // Collect per-step results for the final summary
    typedef struct { uint32_t d; uint32_t recv; uint32_t rtt; } step_t;
    step_t results[NUM_STEPS];
    int nresults = 0;
    int32_t best_ms = -1;

    for (uint32_t d = TUNE_MIN_MS; d <= TUNE_MAX_MS && !s_abort; d += TUNE_STEP_MS) {
        afsk_set_post_rx_tx_delay_ms(d);
        ESP_LOGI(TAG, "testing %lu ms...", (unsigned long)d);

        uint32_t recv = 0, avg_rtt = 0;
        ping_step(&recv, &avg_rtt);

        float pct = (float)recv * 100.0f / TUNE_PINGS_PER_STEP;
        bool ok    = pct >= TUNE_SUCCESS_THRESHOLD * 100.0f;

        results[nresults].d    = d;
        results[nresults].recv = recv;
        results[nresults].rtt  = avg_rtt;
        nresults++;

        if (best_ms < 0 && ok)
            best_ms = (int32_t)d;

        snprintf(buf, sizeof(buf),
            "{\"type\":\"tune_step\","
            "\"d\":%lu,\"sent\":%d,\"recv\":%lu,"
            "\"pct\":%.0f,\"rtt\":%lu,\"ok\":%s}",
            (unsigned long)d, TUNE_PINGS_PER_STEP, (unsigned long)recv,
            pct, (unsigned long)avg_rtt,
            ok ? "true" : "false");
        audio_stream_ws_send_text(buf);

        ESP_LOGI(TAG, "  %lu ms: %lu/%d (%.0f%%) avg_rtt=%lu ms",
                 (unsigned long)d, (unsigned long)recv,
                 TUNE_PINGS_PER_STEP, pct, (unsigned long)avg_rtt);

        // 30-second pause between steps (skip after last step)
        if (d + TUNE_STEP_MS <= TUNE_MAX_MS && !s_abort) {
            for (int rem = TUNE_PAUSE_S; rem > 0 && !s_abort; rem--) {
                if (rem == TUNE_PAUSE_S || rem % 10 == 0) {
                    snprintf(buf, sizeof(buf),
                        "{\"type\":\"tune_waiting\","
                        "\"next_ms\":%lu,\"remaining_s\":%d}",
                        (unsigned long)(d + TUNE_STEP_MS), rem);
                    audio_stream_ws_send_text(buf);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }

    // Restore original delay before announcing result
    afsk_set_post_rx_tx_delay_ms(original_delay);

    if (s_abort) {
        audio_stream_ws_send_text("{\"type\":\"tune_aborted\"}");
        ESP_LOGI(TAG, "sweep aborted");
    } else {
        // Build compact all-steps array then emit tune_done
        char done_buf[768];
        int sp = 0;
        sp += snprintf(done_buf + sp, (int)sizeof(done_buf) - sp,
            "{\"type\":\"tune_done\","
            "\"best_ms\":%ld,\"orig_ms\":%lu,\"steps\":[",
            (long)best_ms, (unsigned long)original_delay);
        for (int i = 0; i < nresults && sp < (int)sizeof(done_buf) - 40; i++) {
            float pct = (float)results[i].recv * 100.0f / TUNE_PINGS_PER_STEP;
            sp += snprintf(done_buf + sp, (int)sizeof(done_buf) - sp,
                "%s{\"d\":%lu,\"r\":%lu,\"rtt\":%lu,\"p\":%.0f}",
                i ? "," : "",
                (unsigned long)results[i].d,
                (unsigned long)results[i].recv,
                (unsigned long)results[i].rtt, pct);
        }
        snprintf(done_buf + sp, (int)sizeof(done_buf) - sp, "]}");
        audio_stream_ws_send_text(done_buf);

        if (best_ms >= 0)
            ESP_LOGI(TAG, "sweep done — recommended: %ld ms  (original: %lu ms)",
                     (long)best_ms, (unsigned long)original_delay);
        else
            ESP_LOGW(TAG, "sweep done — no delay reached %.0f%% success",
                     TUNE_SUCCESS_THRESHOLD * 100.0f);
    }

    s_running = false;
    s_task    = NULL;
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// REST handlers
// ---------------------------------------------------------------------------

static esp_err_t handle_start(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    if (s_remote_ip[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req,
            "{\"error\":\"ip.remote_addr not set in config.json\"}");
        return ESP_OK;
    }

    ip4_addr_t dummy;
    if (!ax25ip_get_addr(&dummy)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req,
            "{\"error\":\"ip gateway not active — enable ip.enabled in config\"}");
        return ESP_OK;
    }

    if (s_running) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "{\"error\":\"tune already running\"}");
        return ESP_OK;
    }

    s_abort   = false;
    s_running = true;
    if (xTaskCreate(tune_task, "delay_tune", 4096, NULL, 3, &s_task) != pdPASS) {
        s_running = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"error\":\"task create failed\"}");
        return ESP_OK;
    }

    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t handle_stop(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (!s_running) {
        httpd_resp_sendstr(req, "{\"ok\":false,\"msg\":\"not running\"}");
        return ESP_OK;
    }
    s_abort = true;
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void delay_tune_init(const cJSON *cfg)
{
    // Read ip.remote_addr from config
    s_remote_ip[0] = '\0';
    if (cfg) {
        cJSON *ip_obj = cJSON_GetObjectItem(cfg, "ip");
        if (ip_obj) {
            // Support both flat "remote_addr" and nested "remote": {"addr": ...}
            cJSON *flat = cJSON_GetObjectItem(ip_obj, "remote_addr");
            cJSON *nested_obj = cJSON_GetObjectItem(ip_obj, "remote");
            cJSON *nested = nested_obj ? cJSON_GetObjectItem(nested_obj, "addr") : NULL;
            cJSON *src = cJSON_IsString(flat) ? flat : (cJSON_IsString(nested) ? nested : NULL);
            if (src && src->valuestring[0] != '\0')
                snprintf(s_remote_ip, sizeof(s_remote_ip), "%s", src->valuestring);
        }
    }

    httpd_handle_t server = audio_stream_get_httpd();
    if (!server) {
        ESP_LOGE(TAG, "httpd not ready — call after audio_stream_init()");
        return;
    }

    static const httpd_uri_t uri_start = {
        .uri     = "/api/tune/start",
        .method  = HTTP_POST,
        .handler = handle_start,
    };
    static const httpd_uri_t uri_stop = {
        .uri     = "/api/tune/stop",
        .method  = HTTP_POST,
        .handler = handle_stop,
    };
    httpd_register_uri_handler(server, &uri_start);
    httpd_register_uri_handler(server, &uri_stop);

    if (s_remote_ip[0])
        ESP_LOGI(TAG, "delay tuner ready  remote=%s  (POST /api/tune/start)", s_remote_ip);
    else
        ESP_LOGW(TAG, "delay tuner ready but ip.remote_addr not set in config");
}

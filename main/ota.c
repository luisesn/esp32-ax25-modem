#include "ota.h"

#include <string.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_http_server.h>
#include <sys/socket.h>

#include "audio_stream.h"

#define TAG "ota"
#define OTA_BUF_SIZE    4096
#define OTA_SOCK_TIMEOUT_S 60

static bool s_ota_in_progress = false;

static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
}

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    if (s_ota_in_progress) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"OTA already in progress\"}");
        return ESP_OK;
    }

    int total = (int)req->content_len;
    if (total <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"Content-Length required\"}");
        return ESP_OK;
    }

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        ESP_LOGE(TAG, "no OTA partition found");
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"no OTA partition\"}");
        return ESP_OK;
    }

    if ((size_t)total > update_part->size) {
        ESP_LOGE(TAG, "firmware too large: %d > %u", total, (unsigned)update_part->size);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"firmware too large for partition\"}");
        return ESP_OK;
    }

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"ota_begin failed\"}");
        return ESP_OK;
    }

    s_ota_in_progress = true;

    // Raise socket recv timeout: sector erases stall both cores for ~30-50ms each;
    // with ~240 sectors in a 1 MB image the cumulative delay can exceed 5 s default.
    int ota_fd = httpd_req_to_sockfd(req);
    if (ota_fd >= 0) {
        struct timeval tv = { .tv_sec = OTA_SOCK_TIMEOUT_S, .tv_usec = 0 };
        setsockopt(ota_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    ESP_LOGI(TAG, "OTA start: %d bytes → %s", total, update_part->label);

    {
        char msg[80];
        snprintf(msg, sizeof(msg), "{\"type\":\"ota_start\",\"size\":%d}", total);
        audio_stream_ws_send_text(msg);
    }

    static uint8_t buf[OTA_BUF_SIZE];
    int received = 0;
    bool magic_ok = false;
    int next_report = 4 * 1024;

    while (received < total) {
        int want = total - received;
        if (want > OTA_BUF_SIZE) want = OTA_BUF_SIZE;

        int got = httpd_req_recv(req, (char *)buf, want);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "recv timeout at %d/%d — retrying", received, total);
            continue;
        }
        if (got <= 0) {
            ESP_LOGE(TAG, "recv failed at %d/%d (ret=%d, errno=%d)", received, total, got, errno);
            goto abort_ota;
        }

        if (!magic_ok) {
            magic_ok = true;
            if (buf[0] != 0xE9u) {
                ESP_LOGE(TAG, "bad magic 0x%02X (expected 0xE9)", buf[0]);
                audio_stream_ws_send_text(
                    "{\"type\":\"ota_error\",\"msg\":\"not an ESP32 firmware image\"}");
                goto abort_ota;
            }
        }

        err = esp_ota_write(ota_handle, buf, (size_t)got);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            goto abort_ota;
        }

        received += got;

        if (received >= next_report) {
            ESP_LOGI(TAG, "OTA progress: %d/%d bytes (%.0f%%)",
                     received, total, (float)received * 100.0f / (float)total);
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "{\"type\":\"ota_progress\",\"received\":%d,\"total\":%d}",
                     received, total);
            audio_stream_ws_send_text(msg);
            next_report += 4 * 1024;
        }
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        audio_stream_ws_send_text(
            "{\"type\":\"ota_error\",\"msg\":\"image validation failed\"}");
        s_ota_in_progress = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"ota_end failed\"}");
        return ESP_OK;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        audio_stream_ws_send_text(
            "{\"type\":\"ota_error\",\"msg\":\"set boot partition failed\"}");
        s_ota_in_progress = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"set_boot failed\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA OK (%d bytes) — rebooting in 3 s", received);
    audio_stream_ws_send_text("{\"type\":\"ota_done\"}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");

    xTaskCreate(reboot_task, "ota_reboot", 1536, NULL, 5, NULL);
    return ESP_OK;

abort_ota:
    esp_ota_abort(ota_handle);
    s_ota_in_progress = false;
    audio_stream_ws_send_text(
        "{\"type\":\"ota_error\",\"msg\":\"upload aborted\"}");
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"error\":\"upload aborted\"}");
    return ESP_OK;
}

void ota_init(void)
{
    httpd_handle_t server = audio_stream_get_httpd();
    if (!server) {
        ESP_LOGE(TAG, "httpd not ready — call after audio_stream_init()");
        return;
    }

    static const httpd_uri_t uri_upload = {
        .uri     = "/api/ota/upload",
        .method  = HTTP_POST,
        .handler = ota_upload_handler,
    };
    httpd_register_uri_handler(server, &uri_upload);
    ESP_LOGI(TAG, "OTA endpoint ready (POST /api/ota/upload)");
}

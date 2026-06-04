// OTA over Wi-Fi — espota.py protocol (Arduino-compatible, UDP+TCP port 3232).
// Compatible with: idf.py -p <ESP32_IP> app-flash
//
// Protocol (host = espota.py):
//   1. Host → Device UDP port 3232: "<cmd> <filesize> <md5_hex32>"
//      cmd 0 = FLASH (the only one we handle)
//   2. Device → Host UDP: "OK"
//   3. Host opens TCP to Device port 3232 and streams <filesize> bytes.
//   4. Device sends 'O' (0x4F) for each 1024-byte chunk received (progress ack).
//   5. After all bytes: Device verifies MD5, sends "OK\n", reboots.
//      On error: Device closes connection without sending OK.

#include "ota_net.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_rom_md5.h"

#define TAG              "ota_net"
#define ESPOTA_PORT      3232
#define OTA_CMD_FLASH    0
#define CHUNK_ACK_BYTES  1024
#define OTA_BUF_SIZE     4096
#define TCP_ACCEPT_TIMEOUT_S  10
#define TCP_RECV_TIMEOUT_S    60   // sector erases can stall ~30 ms each

// ---------------------------------------------------------------------------

static bool do_ota(int tcp_conn, unsigned int filesize, const char *expected_md5)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        ESP_LOGE(TAG, "no OTA update partition found");
        return false;
    }
    if (filesize > part->size) {
        ESP_LOGE(TAG, "firmware %u B exceeds partition %u B", filesize, (unsigned)part->size);
        return false;
    }

    esp_ota_handle_t hdl;
    esp_err_t err = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        return false;
    }

    md5_context_t md5_ctx;
    esp_rom_md5_init(&md5_ctx);

    static uint8_t s_buf[OTA_BUF_SIZE];
    unsigned int received = 0;
    unsigned int last_ack = 0;
    bool ok = true;

    while (received < filesize) {
        int want = (int)(filesize - received);
        if (want > OTA_BUF_SIZE) want = OTA_BUF_SIZE;

        int got = recv(tcp_conn, s_buf, want, 0);
        if (got <= 0) {
            ESP_LOGE(TAG, "TCP recv failed at %u/%u (ret=%d errno=%d)",
                     received, filesize, got, errno);
            ok = false;
            break;
        }

        esp_rom_md5_update(&md5_ctx, s_buf, (unsigned int)got);

        err = esp_ota_write(hdl, s_buf, (size_t)got);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            ok = false;
            break;
        }

        received += (unsigned int)got;

        // Send 'O' progress ack for every CHUNK_ACK_BYTES received
        while (last_ack + CHUNK_ACK_BYTES <= received) {
            char ack = 'O';
            send(tcp_conn, &ack, 1, MSG_NOSIGNAL);
            last_ack += CHUNK_ACK_BYTES;
        }

        if (received % (64 * 1024) == 0 || received == filesize)
            ESP_LOGI(TAG, "OTA progress: %u/%u bytes (%.0f%%)",
                     received, filesize, (float)received * 100.0f / (float)filesize);
    }

    if (!ok || received != filesize) {
        esp_ota_abort(hdl);
        return false;
    }

    // Verify MD5
    uint8_t digest[16];
    esp_rom_md5_final(digest, &md5_ctx);

    char computed[33];
    for (int i = 0; i < 16; i++)
        snprintf(computed + i * 2, 3, "%02x", digest[i]);

    if (strcasecmp(computed, expected_md5) != 0) {
        ESP_LOGE(TAG, "MD5 mismatch: got=%s expected=%s", computed, expected_md5);
        esp_ota_abort(hdl);
        return false;
    }

    err = esp_ota_end(hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------

static void ota_net_task(void *arg)
{
    (void)arg;

    int opt = 1;

    // Persistent UDP socket for receiving OTA requests
    int udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "UDP socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in srv = {
        .sin_family      = AF_INET,
        .sin_port        = htons(ESPOTA_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(udp_sock, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
        ESP_LOGE(TAG, "UDP bind: %d", errno);
        close(udp_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "espota server listening on UDP port %d", ESPOTA_PORT);

    while (1) {
        // ----------------------------------------------------------------
        // Phase 1: wait for UDP OTA request
        // ----------------------------------------------------------------
        char req[192];
        struct sockaddr_in cli_addr = {};
        socklen_t cli_len = sizeof(cli_addr);

        int n = recvfrom(udp_sock, req, sizeof(req) - 1, 0,
                         (struct sockaddr *)&cli_addr, &cli_len);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        req[n] = '\0';

        // Parse: "<cmd> <filesize> <md5>" (ignore extra auth fields)
        int cmd = -1;
        unsigned int filesize = 0;
        char md5[33] = {};
        if (sscanf(req, "%d %u %32s", &cmd, &filesize, md5) != 3
            || cmd != OTA_CMD_FLASH || filesize == 0) {
            ESP_LOGD(TAG, "ignored UDP packet: '%s'", req);
            continue;
        }

        char host_str[16];
        inet_ntop(AF_INET, &cli_addr.sin_addr, host_str, sizeof(host_str));
        ESP_LOGI(TAG, "OTA request from %s: %u bytes MD5=%s", host_str, filesize, md5);

        // ----------------------------------------------------------------
        // Phase 2: open TCP server BEFORE acknowledging host (no race)
        // ----------------------------------------------------------------
        int tcp_srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcp_srv < 0) {
            ESP_LOGE(TAG, "TCP socket: %d", errno);
            continue;
        }
        setsockopt(tcp_srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(tcp_srv, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
            ESP_LOGE(TAG, "TCP bind: %d", errno);
            close(tcp_srv);
            continue;
        }
        if (listen(tcp_srv, 1) < 0) {
            ESP_LOGE(TAG, "TCP listen: %d", errno);
            close(tcp_srv);
            continue;
        }

        // ----------------------------------------------------------------
        // Phase 3: acknowledge host via UDP → host will open TCP connection
        // ----------------------------------------------------------------
        sendto(udp_sock, "OK", 2, 0, (struct sockaddr *)&cli_addr, cli_len);

        // ----------------------------------------------------------------
        // Phase 4: accept TCP connection (with timeout)
        // ----------------------------------------------------------------
        struct timeval tv = { .tv_sec = TCP_ACCEPT_TIMEOUT_S, .tv_usec = 0 };
        setsockopt(tcp_srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        int tcp_conn = accept(tcp_srv, NULL, NULL);
        close(tcp_srv);  // we only need one connection

        if (tcp_conn < 0) {
            ESP_LOGE(TAG, "TCP accept timeout/error: %d", errno);
            continue;
        }

        tv.tv_sec = TCP_RECV_TIMEOUT_S;
        setsockopt(tcp_conn, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // ----------------------------------------------------------------
        // Phase 5: receive firmware, write OTA, verify MD5
        // ----------------------------------------------------------------
        bool success = do_ota(tcp_conn, filesize, md5);

        if (success) {
            ESP_LOGI(TAG, "OTA OK — sending OK and rebooting");
            send(tcp_conn, "OK\n", 3, MSG_NOSIGNAL);
            close(tcp_conn);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            ESP_LOGE(TAG, "OTA failed — connection closed");
            close(tcp_conn);
        }
    }

    close(udp_sock);
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------

void ota_net_init(void)
{
    xTaskCreate(ota_net_task, "ota_net", 8192, NULL, 5, NULL);
}

#include "transport_wifi.h"
#include "kiss.h"
#include "config.h"

#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

#define TAG "wifi_transport"
#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_wifi_eg;

// Acceso al fd del cliente sin mutex: en ESP32 (32-bit) leer/escribir un int
// alineado es atómico. El peor caso es un send() sobre un fd recién cerrado,
// que falla con EBADF de forma inocua.
static volatile int s_client_fd = -1;

// ---------------------------------------------------------------------------
// WiFi event handler
// ---------------------------------------------------------------------------

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Desconectado de WiFi, reintentando...");
        s_client_fd = -1;   // invalidar socket de cliente si lo había
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        char ip_str[16];
        esp_ip4addr_ntoa(&evt->ip_info.ip, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "IP obtenida: %s", ip_str);
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

// ---------------------------------------------------------------------------
// Conexión WiFi STA
// ---------------------------------------------------------------------------

static void wifi_connect(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS llena, borrando...");
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    s_wifi_eg = xEventGroupCreate();

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_cfg = {};
    memcpy(wifi_cfg.sta.ssid,     WIFI_SSID,     strlen(WIFI_SSID));
    memcpy(wifi_cfg.sta.password, WIFI_PASSWORD, strlen(WIFI_PASSWORD));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();

    ESP_LOGI(TAG, "Conectando a '%s'...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

// ---------------------------------------------------------------------------
// Servidor TCP KISS
// ---------------------------------------------------------------------------

static void server_task(void *arg) {
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port        = htons(KISS_TCP_PORT),
    };

    int server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd < 0) {
        ESP_LOGE(TAG, "Error creando socket servidor: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() falló: %d", errno);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    listen(server_fd, 1);
    ESP_LOGI(TAG, "Servidor KISS TCP escuchando en puerto %d", KISS_TCP_PORT);

    for (;;) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int fd = accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
        ESP_LOGI(TAG, "Cliente KISS conectado desde %s", client_ip);

        s_client_fd = fd;

        // Bucle de recepción: bytes del host → decodificador KISS
        uint8_t buf[256];
        int n;
        while ((n = recv(fd, buf, sizeof(buf), 0)) > 0) {
            for (int i = 0; i < n; i++)
                kiss_rx_byte(buf[i]);
        }

        ESP_LOGI(TAG, "Cliente KISS desconectado");
        s_client_fd = -1;
        close(fd);
    }
}

// ---------------------------------------------------------------------------
// Operaciones del transporte
// ---------------------------------------------------------------------------

static void wifi_transport_init(void) {
    wifi_connect();
    xTaskCreate(server_task, "kiss_tcp_srv", 4096, NULL, 7, NULL);
}

static void wifi_transport_write(const uint8_t *buf, size_t len) {
    int fd = s_client_fd;
    if (fd >= 0) {
        // send() sin MSG_DONTWAIT: admisible porque las tramas AX.25 a 1200 bps
        // son pequeñas y el buffer del kernel raramente se llena.
        if (send(fd, buf, len, 0) < 0) {
            ESP_LOGD(TAG, "send() falló (%d), cliente probablemente desconectado", errno);
        }
    }
}

const transport_ops_t transport_wifi_ops = {
    .init  = wifi_transport_init,
    .write = wifi_transport_write,
};

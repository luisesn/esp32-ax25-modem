#include "transport_wifi.h"
#include "kiss.h"
#include "config.h"
#include "aux_config.h"

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

#define DEFAULT_CONNECT_TIMEOUT_S 120

static EventGroupHandle_t s_wifi_eg;

// true mientras estamos intentando la conexión STA; false en cuanto
// expira el timeout o nos pasamos a modo AP.
static volatile bool s_sta_active = false;

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
        s_client_fd = -1;
        if (s_sta_active) {
            ESP_LOGW(TAG, "Desconectado de WiFi, reintentando...");
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        char ip_str[16];
        esp_ip4addr_ntoa(&evt->ip_info.ip, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "IP obtenida: %s", ip_str);
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

// ---------------------------------------------------------------------------
// DNS cautivo: responde a todas las consultas con 192.168.4.1
// ---------------------------------------------------------------------------

static void captive_dns_task(void *arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port        = htons(53),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "DNS bind falló: %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "DNS captivo escuchando en puerto 53");

    uint8_t buf[256];
    uint8_t resp[512];

    for (;;) {
        struct sockaddr_in client;
        socklen_t clen = sizeof(client);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&client, &clen);
        if (n < 12) continue;

        // Construir respuesta: copiar la consulta y añadir una respuesta A.
        memcpy(resp, buf, n);
        resp[2] |= 0x80;    // QR = 1 (respuesta)
        resp[3]  = 0x80;    // RA = 1, sin errores
        resp[6]  = 0x00; resp[7]  = 0x01; // ANCOUNT = 1
        resp[8]  = 0x00; resp[9]  = 0x00; // NSCOUNT = 0
        resp[10] = 0x00; resp[11] = 0x00; // ARCOUNT = 0

        int pos = n;
        // Registro A apuntando al nombre de la pregunta (puntero 0xC00C).
        resp[pos++] = 0xC0; resp[pos++] = 0x0C; // puntero a offset 12
        resp[pos++] = 0x00; resp[pos++] = 0x01; // TYPE  A
        resp[pos++] = 0x00; resp[pos++] = 0x01; // CLASS IN
        resp[pos++] = 0x00; resp[pos++] = 0x00; // TTL (4 bytes)
        resp[pos++] = 0x00; resp[pos++] = 60;
        resp[pos++] = 0x00; resp[pos++] = 0x04; // RDLENGTH = 4
        resp[pos++] = 192;  resp[pos++] = 168;  // 192.168.4.1
        resp[pos++] = 4;    resp[pos++] = 1;

        sendto(sock, resp, pos, 0, (struct sockaddr *)&client, clen);
    }
}

// ---------------------------------------------------------------------------
// Inicio de modo AP
// ---------------------------------------------------------------------------

static void wifi_start_ap(const char *ssid, const char *password) {
    ESP_LOGI(TAG, "Iniciando AP '%s'...", ssid);

    esp_netif_create_default_wifi_ap();

    wifi_config_t ap_cfg = {};
    snprintf((char *)ap_cfg.ap.ssid,     sizeof(ap_cfg.ap.ssid),     "%s", ssid);
    ap_cfg.ap.ssid_len = (uint8_t)strlen(ssid);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.beacon_interval = 100;

    if (password && strlen(password) >= 8) {
        snprintf((char *)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s", password);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();

    xTaskCreate(captive_dns_task, "cap_dns", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "AP iniciado. IP: 192.168.4.1  SSID: %s", ssid);
}

// ---------------------------------------------------------------------------
// Conexión WiFi: intenta STA con timeout, cae a AP si no conecta
// ---------------------------------------------------------------------------

static void wifi_connect(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS llena, borrando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
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

    cJSON *config = config_load();
    if (!config) {
        ESP_LOGE(TAG, "No hay configuración disponible");
        return;
    }

    // --- Leer sección wifi (array o objeto único, retrocompatible) ---
    cJSON *wifi_j = cJSON_GetObjectItem(config, "wifi");
    cJSON *ap_j   = cJSON_GetObjectItem(config, "ap");

    bool connected   = false;
    bool sta_started = false;

    if (wifi_j) {
        int net_count = cJSON_IsArray(wifi_j) ? cJSON_GetArraySize(wifi_j) : 1;

        for (int ni = 0; ni < net_count && !connected; ni++) {
            cJSON *net = cJSON_IsArray(wifi_j)
                         ? cJSON_GetArrayItem(wifi_j, ni)
                         : wifi_j;
            if (!cJSON_IsObject(net)) continue;

            const char *ssid     = NULL;
            const char *password = NULL;
            int  timeout_s       = DEFAULT_CONNECT_TIMEOUT_S;
            cJSON *it;
            it = cJSON_GetObjectItem(net, "ssid");
            if (cJSON_IsString(it)) ssid = it->valuestring;
            it = cJSON_GetObjectItem(net, "password");
            if (cJSON_IsString(it)) password = it->valuestring;
            it = cJSON_GetObjectItem(net, "connect_timeout_s");
            if (cJSON_IsNumber(it) && it->valuedouble > 0)
                timeout_s = (int)it->valuedouble;

            if (!ssid || !ssid[0]) continue;

            wifi_config_t sta_cfg = {};
            snprintf((char *)sta_cfg.sta.ssid,     sizeof(sta_cfg.sta.ssid),
                     "%s", ssid);
            snprintf((char *)sta_cfg.sta.password, sizeof(sta_cfg.sta.password),
                     "%s", password ? password : "");

            if (sta_started) {
                // Parar antes de reconfigurar para la siguiente red
                s_sta_active = false;
                esp_wifi_stop();
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);

            s_sta_active = true;
            esp_wifi_set_mode(WIFI_MODE_STA);
            esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
            esp_wifi_start();
            sta_started = true;

            ESP_LOGI(TAG, "Conectando a '%s' (timeout %d s)...", ssid, timeout_s);
            EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                                   pdFALSE, pdTRUE,
                                                   pdMS_TO_TICKS((uint32_t)timeout_s * 1000UL));
            s_sta_active = false;
            connected = (bits & WIFI_CONNECTED_BIT) != 0;

            if (!connected)
                ESP_LOGW(TAG, "No se pudo conectar a '%s'%s", ssid,
                         ni + 1 < net_count ? ". Probando siguiente..." : ".");
        }
    }

    if (!connected) {
        if (sta_started) {
            ESP_LOGW(TAG, "No se pudo conectar a WiFi. Iniciando AP de respaldo...");
            esp_wifi_stop();
        }

        // --- Leer sección ap ---
        const char *ap_ssid = "APRS-TNC";
        const char *ap_pass = "";
        bool ap_enabled     = true;

        if (ap_j) {
            cJSON *it;
            it = cJSON_GetObjectItem(ap_j, "enabled");
            if (cJSON_IsBool(it)) ap_enabled = cJSON_IsTrue(it);
            it = cJSON_GetObjectItem(ap_j, "ssid");
            if (cJSON_IsString(it)) ap_ssid = it->valuestring;
            it = cJSON_GetObjectItem(ap_j, "password");
            if (cJSON_IsString(it)) ap_pass = it->valuestring;
        }

        if (ap_enabled) {
            wifi_start_ap(ap_ssid, ap_pass);
        } else {
            ESP_LOGW(TAG, "AP de respaldo deshabilitado. Sin red.");
        }
    }

    config_free_json(config);
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
        if (send(fd, buf, len, 0) < 0) {
            ESP_LOGD(TAG, "send() falló (%d), cliente probablemente desconectado", errno);
        }
    }
}

const transport_ops_t transport_wifi_ops = {
    .init  = wifi_transport_init,
    .write = wifi_transport_write,
};

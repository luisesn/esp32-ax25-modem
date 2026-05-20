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
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

#define TAG "wifi_transport"
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_STOPPED_BIT    BIT1  // set by WIFI_EVENT_STA_STOP

#define DEFAULT_CONNECT_TIMEOUT_S 30
#define RECONNECT_DELAY_US  (3LL * 1000000LL)  // 3 s between retries

static EventGroupHandle_t s_wifi_eg;

WifiConnStatus g_wifi_status = {0};

// true while we want automatic reconnects on disconnect.
static volatile bool s_sta_active = false;

// One-shot timer: fires RECONNECT_DELAY_US after a disconnect to retry.
static esp_timer_handle_t s_reconnect_timer;

// Acceso al fd del cliente sin mutex: en ESP32 (32-bit) leer/escribir un int
// alineado es atómico. El peor caso es un send() sobre un fd recién cerrado,
// que falla con EBADF de forma inocua.
static volatile int s_client_fd = -1;

// Cached network credentials (populated at init, used by reconnect task).
#define MAX_WIFI_NETS 5
typedef struct { char ssid[32]; char pass[64]; int timeout_s; } wifi_net_t;
static wifi_net_t   s_nets[MAX_WIFI_NETS];
static int          s_net_count  = 0;
static bool         s_ap_enabled = false;
static char         s_ap_ssid[32] = "APRS-TNC";
static char         s_ap_pass[64] = "";
static TaskHandle_t s_reconnect_task_handle = NULL;

// ---------------------------------------------------------------------------
// WiFi event handler
// ---------------------------------------------------------------------------

static void reconnect_timer_cb(void *arg) {
    if (s_reconnect_task_handle)
        xTaskNotifyGive(s_reconnect_task_handle);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_STOP) {
        xEventGroupSetBits(s_wifi_eg, WIFI_STOPPED_BIT);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_client_fd = -1;
        if (s_sta_active) {
            wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)data;
            ESP_LOGW(TAG, "Desconectado (reason=%d), reintentando en 3 s...", ev->reason);
            esp_timer_stop(s_reconnect_timer);
            esp_timer_start_once(s_reconnect_timer, RECONNECT_DELAY_US);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        char ip_str[16];
        esp_ip4addr_ntoa(&evt->ip_info.ip, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "IP obtenida: %s", ip_str);
        g_wifi_status.state = WIFI_STATUS_CONNECTED;
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

    // Reconnect one-shot timer: fires 3 s after a disconnect.
    esp_timer_create_args_t targs = {
        .callback = reconnect_timer_cb,
        .name     = "wifi_reconnect",
    };
    esp_timer_create(&targs, &s_reconnect_timer);

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

    // Cache AP config for reconnect task (read before STA loop so it's always populated).
    if (ap_j) {
        cJSON *it;
        it = cJSON_GetObjectItem(ap_j, "enabled");
        if (cJSON_IsBool(it)) s_ap_enabled = cJSON_IsTrue(it);
        it = cJSON_GetObjectItem(ap_j, "ssid");
        if (cJSON_IsString(it)) snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s", it->valuestring);
        it = cJSON_GetObjectItem(ap_j, "password");
        if (cJSON_IsString(it)) snprintf(s_ap_pass, sizeof(s_ap_pass), "%s", it->valuestring);
    }

    bool connected   = false;
    bool sta_started = false;
    int  net_count   = 0;
    if (wifi_j) net_count = cJSON_IsArray(wifi_j) ? cJSON_GetArraySize(wifi_j) : 1;

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

        // Cache credentials for the reconnect task.
        if (s_net_count < MAX_WIFI_NETS) {
            snprintf(s_nets[s_net_count].ssid, sizeof(s_nets[s_net_count].ssid), "%s", ssid);
            snprintf(s_nets[s_net_count].pass, sizeof(s_nets[s_net_count].pass), "%s", password ? password : "");
            s_nets[s_net_count].timeout_s = timeout_s;
            s_net_count++;
        }

        // Stop previous attempt cleanly before reconfiguring.
        if (sta_started) {
            esp_timer_stop(s_reconnect_timer);
            s_sta_active = false;
            xEventGroupClearBits(s_wifi_eg, WIFI_STOPPED_BIT);
            esp_wifi_stop();
            // Wait for STA_STOP event instead of a fixed delay.
            xEventGroupWaitBits(s_wifi_eg, WIFI_STOPPED_BIT,
                                pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
        }
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);

        // Update shared status for display.
        g_wifi_status.state           = WIFI_STATUS_CONNECTING;
        g_wifi_status.net_index       = ni;
        g_wifi_status.net_count       = net_count;
        g_wifi_status.timeout_s       = timeout_s;
        g_wifi_status.attempt_start_us = esp_timer_get_time();
        snprintf(g_wifi_status.ssid, sizeof(g_wifi_status.ssid), "%s", ssid);

        wifi_config_t sta_cfg = {};
        snprintf((char *)sta_cfg.sta.ssid,     sizeof(sta_cfg.sta.ssid),     "%s", ssid);
        snprintf((char *)sta_cfg.sta.password, sizeof(sta_cfg.sta.password), "%s",
                 password ? password : "");

        s_sta_active = true;
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        esp_wifi_start();
        sta_started = true;

        ESP_LOGI(TAG, "[%d/%d] Conectando a '%s' (timeout %d s)...",
                 ni + 1, net_count, ssid, timeout_s);
        EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                               pdFALSE, pdTRUE,
                                               pdMS_TO_TICKS((uint32_t)timeout_s * 1000UL));
        // Keep s_sta_active=true on success so disconnect events trigger reconnect.
        // Only clear it on timeout (failed attempt).
        connected = (bits & WIFI_CONNECTED_BIT) != 0;
        if (!connected) s_sta_active = false;
        esp_timer_stop(s_reconnect_timer);

        if (!connected)
            ESP_LOGW(TAG, "No se pudo conectar a '%s'%s", ssid,
                     ni + 1 < net_count ? ". Probando siguiente..." : ".");
    }

    if (!connected) {
        if (sta_started) {
            ESP_LOGW(TAG, "No se pudo conectar a WiFi. Iniciando AP de respaldo...");
            xEventGroupClearBits(s_wifi_eg, WIFI_STOPPED_BIT);
            esp_wifi_stop();
            xEventGroupWaitBits(s_wifi_eg, WIFI_STOPPED_BIT,
                                pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
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
            snprintf(g_wifi_status.ssid, sizeof(g_wifi_status.ssid), "%s", ap_ssid);
            g_wifi_status.state = WIFI_STATUS_HOTSPOT;
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
// Reconexión automática: cicla todas las redes y cae a AP si todas fallan
// ---------------------------------------------------------------------------

static void wifi_reconnect_task(void *arg) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (g_wifi_status.state == WIFI_STATUS_HOTSPOT) continue;

        ESP_LOGI(TAG, "Reconexión: probando %d red(es)...", s_net_count);
        s_sta_active = false;
        esp_timer_stop(s_reconnect_timer);

        // Stop current WiFi cleanly before reconfiguring.
        xEventGroupClearBits(s_wifi_eg, WIFI_STOPPED_BIT);
        esp_wifi_stop();
        xEventGroupWaitBits(s_wifi_eg, WIFI_STOPPED_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));

        bool connected = false;
        for (int ni = 0; ni < s_net_count && !connected; ni++) {
            xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
            g_wifi_status.state            = WIFI_STATUS_CONNECTING;
            g_wifi_status.net_index        = ni;
            g_wifi_status.net_count        = s_net_count;
            g_wifi_status.timeout_s        = s_nets[ni].timeout_s;
            g_wifi_status.attempt_start_us = esp_timer_get_time();
            snprintf(g_wifi_status.ssid, sizeof(g_wifi_status.ssid), "%s", s_nets[ni].ssid);

            wifi_config_t sta_cfg = {};
            snprintf((char *)sta_cfg.sta.ssid,     sizeof(sta_cfg.sta.ssid),     "%s", s_nets[ni].ssid);
            snprintf((char *)sta_cfg.sta.password, sizeof(sta_cfg.sta.password), "%s", s_nets[ni].pass);

            s_sta_active = true;
            esp_wifi_set_mode(WIFI_MODE_STA);
            esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
            esp_wifi_start();  // STA_START event calls esp_wifi_connect() automatically

            ESP_LOGI(TAG, "[%d/%d] Reconect: '%s' (timeout %ds)...",
                     ni + 1, s_net_count, s_nets[ni].ssid, s_nets[ni].timeout_s);
            EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                                    pdFALSE, pdTRUE,
                                                    pdMS_TO_TICKS((uint32_t)s_nets[ni].timeout_s * 1000UL));
            connected = (bits & WIFI_CONNECTED_BIT) != 0;
            if (!connected) {
                s_sta_active = false;
                esp_timer_stop(s_reconnect_timer);
                ESP_LOGW(TAG, "Reconect: '%s' falló%s", s_nets[ni].ssid,
                         ni + 1 < s_net_count ? ", probando siguiente..." : ".");
                if (ni + 1 < s_net_count) {
                    xEventGroupClearBits(s_wifi_eg, WIFI_STOPPED_BIT);
                    esp_wifi_stop();
                    xEventGroupWaitBits(s_wifi_eg, WIFI_STOPPED_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
                }
            } else {
                ESP_LOGI(TAG, "Reconectado a '%s'", s_nets[ni].ssid);
                // s_sta_active stays true to allow future reconnects
            }
        }

        if (!connected) {
            xEventGroupClearBits(s_wifi_eg, WIFI_STOPPED_BIT);
            esp_wifi_stop();
            xEventGroupWaitBits(s_wifi_eg, WIFI_STOPPED_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
            if (s_ap_enabled) {
                ESP_LOGW(TAG, "Todas las redes fallaron. Iniciando AP de respaldo...");
                g_wifi_status.state = WIFI_STATUS_HOTSPOT;
                snprintf(g_wifi_status.ssid, sizeof(g_wifi_status.ssid), "%s", s_ap_ssid);
                wifi_start_ap(s_ap_ssid, s_ap_pass);
            } else {
                ESP_LOGW(TAG, "Todas las redes fallaron. AP deshabilitado, sin red.");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Operaciones del transporte
// ---------------------------------------------------------------------------

static void wifi_transport_init(void) {
    // Create reconnect task before wifi_connect() so the handle is valid
    // before any disconnect event can fire.
    xTaskCreate(wifi_reconnect_task, "wifi_reconn", 3072, NULL, 5, &s_reconnect_task_handle);
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

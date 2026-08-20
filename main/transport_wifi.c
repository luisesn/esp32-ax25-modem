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
#include "esp_mac.h"      // MACSTR / MAC2STR
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

#define TAG "wifi_transport"
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_STOPPED_BIT    BIT1  // set by WIFI_EVENT_STA_STOP / AP_STOP
#define WIFI_FAIL_BIT       BIT2  // set by STA_DISCONNECTED during a connect attempt

#define DEFAULT_CONNECT_TIMEOUT_S 30
#define RECONNECT_DELAY_US  (3LL * 1000000LL)   // 3 s tras perder la conexión
#define RETRY_NO_NET_US     (60LL * 1000000LL)  // 60 s cuando no hay ninguna red
#define ASSOC_RETRY_MS      1000                // reintento de asociación dentro del timeout

static EventGroupHandle_t s_wifi_eg;

WifiConnStatus g_wifi_status = {0};

// true en cuanto estamos asociados con IP: un disconnect debe programar reintento.
static volatile bool s_sta_steady = false;

// true mientras un ciclo de conexión es dueño de la radio: los disconnect son
// fallos de intento y los gestiona el propio ciclo, no el timer de reconexión.
static volatile bool s_sta_attempting = false;

// true mientras el driver WiFi está arrancado (esp_wifi_start() sin stop posterior).
static bool s_radio_started = false;

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
static bool         s_ap_enabled     = true;   // sin sección "ap" en el JSON: hotspot habilitado
static bool         s_ap_netif_created = false;   // guard against double esp_netif_create_default_wifi_ap()
static bool         s_captive_dns_started = false; // guard against a second bind() on port 53
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

// Programa un despertar de wifi_reconnect_task dentro de `delay_us`.
static void schedule_retry(int64_t delay_us) {
    esp_timer_stop(s_reconnect_timer);
    esp_timer_start_once(s_reconnect_timer, delay_us);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_STOP) {
        xEventGroupSetBits(s_wifi_eg, WIFI_STOPPED_BIT);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)data;
        s_client_fd = -1;
        // Orden importante: s_sta_steady se comprueba primero para que un
        // disconnect llegado entre GOT_IP y el fin del ciclo no se pierda.
        if (s_sta_steady) {
            s_sta_steady = false;
            g_wifi_status.state = WIFI_STATUS_CONNECTING;
            ESP_LOGW(TAG, "Desconectado (reason=%d), reintentando en 3 s...", ev->reason);
            schedule_retry(RECONNECT_DELAY_US);
        } else if (s_sta_attempting) {
            // Fallo de asociación: el ciclo de conexión decide si reintenta esta
            // red o pasa a la siguiente. No tocar el timer aquí.
            ESP_LOGD(TAG, "Intento fallido (reason=%d)", ev->reason);
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        // Prueba definitiva de que el beacon está en el aire: si esta línea no
        // aparece en el log, el AP nunca arrancó (no es un problema de alcance).
        ESP_LOGI(TAG, "AP_START: beacon activo, SSID '%s'", g_wifi_status.ssid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STOP) {
        // También libera a wifi_radio_stop(): al parar en modo AP no hay STA_STOP.
        xEventGroupSetBits(s_wifi_eg, WIFI_STOPPED_BIT);
        ESP_LOGW(TAG, "AP_STOP: el AP se ha detenido");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "AP: cliente conectado " MACSTR " (aid=%d)", MAC2STR(ev->mac), ev->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "AP: cliente desconectado " MACSTR, MAC2STR(ev->mac));
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

// Canal explícito: dejar ap.channel a 0 hace que el driver elija, y en algunas
// versiones de IDF eso deja el AP sin canal válido y sin emitir beacon.
#define AP_CHANNEL 1

// Devuelve true solo si el driver aceptó toda la secuencia y quedó en modo AP.
// El llamante NO debe marcar WIFI_STATUS_HOTSPOT si esto devuelve false: en
// modo hotspot la tarea de reconexión deja de reintentar, así que un AP que
// falló en silencio dejaría el nodo sin ninguna red y sin volver a intentarlo.
static bool wifi_start_ap(const char *ssid, const char *password) {
    ESP_LOGI(TAG, "Iniciando AP '%s'...", ssid);

    if (!s_ap_netif_created) {
        if (esp_netif_create_default_wifi_ap() == NULL) {
            ESP_LOGE(TAG, "esp_netif_create_default_wifi_ap() falló (sin memoria?)");
            return false;
        }
        s_ap_netif_created = true;
    }

    wifi_config_t ap_cfg = {};
    snprintf((char *)ap_cfg.ap.ssid,     sizeof(ap_cfg.ap.ssid),     "%s", ssid);
    ap_cfg.ap.ssid_len = (uint8_t)strlen(ssid);
    ap_cfg.ap.channel = AP_CHANNEL;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.beacon_interval = 100;

    if (password && strlen(password) >= 8) {
        snprintf((char *)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s", password);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        if (password && password[0])
            ESP_LOGW(TAG, "Contraseña del AP con menos de 8 caracteres: AP abierto");
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_err_t err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_AP)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) falló: %s", esp_err_to_name(err));
        return false;
    }
    if ((err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) falló: %s", esp_err_to_name(err));
        return false;
    }
    if ((err = esp_wifi_start()) != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start() en modo AP falló: %s", esp_err_to_name(err));
        return false;
    }
    s_radio_started = true;

    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    if (mode != WIFI_MODE_AP && mode != WIFI_MODE_APSTA) {
        ESP_LOGE(TAG, "Tras esp_wifi_start() el modo es %d, no AP", (int)mode);
        return false;
    }

    if (!s_captive_dns_started) {
        xTaskCreate(captive_dns_task, "cap_dns", 4096, NULL, 5, NULL);
        s_captive_dns_started = true;
    }

    ESP_LOGI(TAG, "AP iniciado: SSID '%s', canal %d, %s, IP 192.168.4.1",
             ssid, AP_CHANNEL,
             ap_cfg.ap.authmode == WIFI_AUTH_OPEN ? "abierto" : "WPA2-PSK");
    return true;
}

// ---------------------------------------------------------------------------
// Conexión WiFi: recorre las redes guardadas y cae a hotspot si ninguna conecta
// ---------------------------------------------------------------------------

// Para la radio y espera al evento STA_STOP/AP_STOP. Sin este estado conocido,
// esp_wifi_set_config() sobre un driver arrancado puede quedarse a medias.
static void wifi_radio_stop(void) {
    if (!s_radio_started) return;
    s_sta_steady = false;
    xEventGroupClearBits(s_wifi_eg, WIFI_STOPPED_BIT);
    if (esp_wifi_stop() == ESP_OK)
        xEventGroupWaitBits(s_wifi_eg, WIFI_STOPPED_BIT, pdTRUE, pdTRUE,
                            pdMS_TO_TICKS(5000));
    s_radio_started = false;
}

// Vuelca config.json a s_nets[] / s_ap_*. Se cachean TODAS las redes (no solo
// las probadas) para que la reconexión disponga de la lista completa.
static void wifi_load_config(void) {
    cJSON *config = config_load();
    if (!config) {
        ESP_LOGE(TAG, "No hay configuración disponible");
        return;
    }

    cJSON *ap_j = cJSON_GetObjectItem(config, "ap");
    if (ap_j) {
        cJSON *it;
        it = cJSON_GetObjectItem(ap_j, "enabled");
        if (cJSON_IsBool(it)) s_ap_enabled = cJSON_IsTrue(it);
        it = cJSON_GetObjectItem(ap_j, "ssid");
        if (cJSON_IsString(it)) snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s", it->valuestring);
        it = cJSON_GetObjectItem(ap_j, "password");
        if (cJSON_IsString(it)) snprintf(s_ap_pass, sizeof(s_ap_pass), "%s", it->valuestring);
    }

    // Sección "wifi": array de redes u objeto único (retrocompatible).
    cJSON *wifi_j = cJSON_GetObjectItem(config, "wifi");
    int listed = 0;
    if (wifi_j) listed = cJSON_IsArray(wifi_j) ? cJSON_GetArraySize(wifi_j) : 1;

    s_net_count = 0;
    for (int i = 0; i < listed && s_net_count < MAX_WIFI_NETS; i++) {
        cJSON *net = cJSON_IsArray(wifi_j) ? cJSON_GetArrayItem(wifi_j, i) : wifi_j;
        if (!cJSON_IsObject(net)) continue;

        cJSON *it = cJSON_GetObjectItem(net, "ssid");
        if (!cJSON_IsString(it) || !it->valuestring[0]) continue;
        snprintf(s_nets[s_net_count].ssid, sizeof(s_nets[s_net_count].ssid), "%s", it->valuestring);

        it = cJSON_GetObjectItem(net, "password");
        snprintf(s_nets[s_net_count].pass, sizeof(s_nets[s_net_count].pass), "%s",
                 cJSON_IsString(it) ? it->valuestring : "");

        it = cJSON_GetObjectItem(net, "connect_timeout_s");
        s_nets[s_net_count].timeout_s = (cJSON_IsNumber(it) && it->valuedouble > 0)
                                        ? (int)it->valuedouble : DEFAULT_CONNECT_TIMEOUT_S;
        s_net_count++;
    }

    if (listed > MAX_WIFI_NETS)
        ESP_LOGW(TAG, "config.json declara %d redes; solo se usan las %d primeras",
                 listed, MAX_WIFI_NETS);

    config_free_json(config);
}

// Recorre en orden todas las redes guardadas. Devuelve true en cuanto una
// obtiene IP (y deja s_sta_steady=true); si ninguna conecta la radio queda parada.
static bool wifi_try_saved_nets(void) {
    if (s_net_count == 0) {
        ESP_LOGW(TAG, "No hay redes WiFi configuradas");
        return false;
    }

    for (int ni = 0; ni < s_net_count; ni++) {
        const int timeout_s = s_nets[ni].timeout_s;

        wifi_radio_stop();  // deja el driver en estado conocido antes de reconfigurar
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

        g_wifi_status.state            = WIFI_STATUS_CONNECTING;
        g_wifi_status.net_index        = ni;
        g_wifi_status.net_count        = s_net_count;
        g_wifi_status.timeout_s        = timeout_s;
        g_wifi_status.attempt_start_us = esp_timer_get_time();
        snprintf(g_wifi_status.ssid, sizeof(g_wifi_status.ssid), "%s", s_nets[ni].ssid);

        wifi_config_t sta_cfg = {};
        snprintf((char *)sta_cfg.sta.ssid,     sizeof(sta_cfg.sta.ssid),     "%s", s_nets[ni].ssid);
        snprintf((char *)sta_cfg.sta.password, sizeof(sta_cfg.sta.password), "%s", s_nets[ni].pass);

        s_sta_attempting = true;
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        esp_err_t err = esp_wifi_start();  // el evento STA_START llama a esp_wifi_connect()
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_start() falló: %s", esp_err_to_name(err));
            s_sta_attempting = false;
            continue;
        }
        s_radio_started = true;

        ESP_LOGI(TAG, "[%d/%d] Conectando a '%s' (timeout %d s)...",
                 ni + 1, s_net_count, s_nets[ni].ssid, timeout_s);

        const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_s * 1000000LL;
        bool ok = false;
        for (;;) {
            int64_t left_ms = (deadline - esp_timer_get_time()) / 1000;
            if (left_ms <= 0) break;

            EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                   pdFALSE, pdFALSE,
                                                   pdMS_TO_TICKS((uint32_t)left_ms));
            if (bits & WIFI_CONNECTED_BIT) { ok = true; break; }
            if (!(bits & WIFI_FAIL_BIT))   break;   // timeout de esta red

            // Asociación fallida (AP fuera de alcance, password erróneo...): el
            // driver no reintenta solo, así que se reintenta hasta agotar el
            // timeout en vez de esperar de brazos cruzados.
            xEventGroupClearBits(s_wifi_eg, WIFI_FAIL_BIT);
            vTaskDelay(pdMS_TO_TICKS(ASSOC_RETRY_MS));
            if (esp_timer_get_time() >= deadline) break;
            esp_wifi_connect();
        }

        // Un disconnect justo tras GOT_IP deja WIFI_FAIL_BIT: no dar la red por buena.
        if (ok && (xEventGroupGetBits(s_wifi_eg) & WIFI_FAIL_BIT)) ok = false;

        if (ok) {
            s_sta_steady     = true;   // a partir de aquí un disconnect programa reintento
            s_sta_attempting = false;
            ESP_LOGI(TAG, "Conectado a '%s'", s_nets[ni].ssid);
            return true;
        }

        s_sta_attempting = false;
        ESP_LOGW(TAG, "No se pudo conectar a '%s'%s", s_nets[ni].ssid,
                 ni + 1 < s_net_count ? ". Probando siguiente..." : ".");
    }

    wifi_radio_stop();
    return false;
}

// Ninguna red guardada disponible: hotspot si está habilitado. Si no hay AP
// (deshabilitado o fallo al arrancar) se reprograma un reintento: sin él el
// nodo se quedaría sin red y sin volver a intentarlo hasta el próximo reinicio.
static void wifi_enter_fallback(void) {
    wifi_radio_stop();

    if (s_ap_enabled) {
        ESP_LOGW(TAG, "Ninguna red guardada disponible. Iniciando AP de respaldo...");
        snprintf(g_wifi_status.ssid, sizeof(g_wifi_status.ssid), "%s", s_ap_ssid);
        if (wifi_start_ap(s_ap_ssid, s_ap_pass)) {
            // Estado terminal: se sale del hotspot reiniciando o reconfigurando.
            g_wifi_status.state = WIFI_STATUS_HOTSPOT;
            return;
        }
        ESP_LOGE(TAG, "El AP de respaldo no arrancó.");
    } else {
        ESP_LOGW(TAG, "AP de respaldo deshabilitado.");
    }

    g_wifi_status.state = WIFI_STATUS_IDLE;
    ESP_LOGW(TAG, "Sin red: reintentando las redes guardadas en %d s",
             (int)(RETRY_NO_NET_US / 1000000LL));
    schedule_retry(RETRY_NO_NET_US);
}

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

    wifi_load_config();

    if (!wifi_try_saved_nets())
        wifi_enter_fallback();
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

        /* KISS hosts can legitimately stay silent for a long time (they only
         * send when they have a frame to TX), so recv() below has no
         * timeout — a plain read timeout would kick out idle-but-alive
         * clients. TCP keepalive instead only fires when the peer stops
         * ACKing: a client that vanishes without a clean FIN/RST (crash,
         * cable pull, silent NAT/firewall drop) would otherwise leave this
         * task blocked in recv() forever, with the single-slot listen()
         * backlog meaning no other client could ever connect again until
         * reboot. */
        int ka = 1;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        int ka_idle = 10, ka_intvl = 5, ka_cnt = 3;
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &ka_idle,  sizeof(ka_idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &ka_cnt,   sizeof(ka_cnt));

        s_client_fd = fd;

        uint8_t buf[256];
        int n;
        while ((n = recv(fd, buf, sizeof(buf), 0)) > 0) {
            for (int i = 0; i < n; i++)
                kiss_rx_byte(buf[i]);
        }

        ESP_LOGI(TAG, "Cliente KISS desconectado");
        kiss_reset();   // clear stale partial-frame state before next client connects
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

        // En hotspot no se reintenta: bajar el AP echaría a los clientes
        // conectados (que es justo el modo en que se reconfigura el nodo).
        if (g_wifi_status.state == WIFI_STATUS_HOTSPOT) continue;

        esp_timer_stop(s_reconnect_timer);
        ESP_LOGI(TAG, "Reconexión: probando %d red(es) guardada(s)...", s_net_count);

        if (!wifi_try_saved_nets())
            wifi_enter_fallback();
    }
}

// ---------------------------------------------------------------------------
// Operaciones del transporte
// ---------------------------------------------------------------------------

static void wifi_transport_init(void) {
    // La tarea de reconexión se crea antes de wifi_connect() para que el handle
    // exista cuando llegue el primer evento. No hay riesgo de que arranque un
    // ciclo en paralelo al inicial: durante los intentos s_sta_attempting=true
    // y los disconnect no programan el timer.
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

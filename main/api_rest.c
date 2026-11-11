// API REST del servidor HTTP (extraída de audio_stream.c): index, estado, APRS,
// configuración, morse, reinicio, subida a SPIFFS, repetidor/squelch y
// respuestas a los sondeos de conectividad (404).
#include "audio_stream.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <lwip/sockets.h>
#include <errno.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdio.h>
#include <math.h>
#include "cJSON.h"
#include "LibAPRS.h"
#include "aux_config.h"
#include "digipeater.h"
#include "morse.h"
#include "rx_stats.h"
#include "AFSK.h"
#include "squelch_sf.h"
#include "rx_survey.h"
#include "carrier_detect.h"
#include "repeater.h"
#include "gps.h"

#define TAG "api_rest"

// ─── HTTP handlers ────────────────────────────────────────────────────────────

// GET / → sirve /spiffs/index.html (si se subió suelto) o index.html.gz (imagen
// flasheada, comprimida en el build) con Content-Encoding: gzip.
static esp_err_t index_handler(httpd_req_t *req) {
    bool gz = false;
    FILE *f = fopen("/spiffs/index.html", "r");
    if (!f) {
        f = fopen("/spiffs/index.html.gz", "r");
        gz = (f != NULL);
    }
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "index.html not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/html");
    if (gz) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        // Abortar en cuanto el cliente deja de drenar: con send_wait_timeout=1
        // cada chunk fallido bloquea la tarea httpd 1 s ("httpd_sock_err: error
        // in send : 11" una vez por segundo), dejando el servidor entero — WS
        // incluido — sin atender.
        if (httpd_resp_send_chunk(req, buf, (ssize_t)n) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;   // httpd cierra el socket; no enviar el chunk final
        }
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// GET /api/me → returns our configured callsign+ssid as JSON
static esp_err_t me_handler(httpd_req_t *req) {
    char my_call[7]; int my_ssid;
    APRS_getCallsign(my_call, &my_ssid);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"call\":\"%s\",\"ssid\":%d}", my_call, my_ssid);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

// GET /api/sys → heap libre/mínimo/bloque mayor y high-water mark (bytes libres
// de stack) de las tareas conocidas, para vigilar fragmentación y stacks justos.
static esp_err_t sys_handler(httpd_req_t *req) {
    static const char *const tasks[] = {
        "receive_audio_task", "aprs_poll_task", "processPacket", "audio_lvl",
        "gps_task", "gps_beacon", "morse_beacon", "audio_stream", "wav_server",
        "kiss_tcp_srv", "wifi_reconn", "display_task", "rf_console", "delay_tune",
    };
    char buf[768];
    int n = snprintf(buf, sizeof(buf),
        "{\"version\":\"%s\",\"uptime_s\":%lld,\"heap_free\":%u,\"heap_min\":%u,\"heap_largest\":%u,\"stacks\":{",
        esp_app_get_description()->version,
        (long long)(esp_timer_get_time() / 1000000LL),
        (unsigned)esp_get_free_heap_size(),
        (unsigned)esp_get_minimum_free_heap_size(),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    bool first = true;
    for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]) && n < (int)sizeof(buf) - 64; i++) {
        // FreeRTOS trunca los nombres a configMAX_TASK_NAME_LEN-1 (15) al crear la
        // tarea, y xTaskGetHandle aborta (assert) si el nombre buscado es más largo.
        char qname[configMAX_TASK_NAME_LEN];
        strlcpy(qname, tasks[i], sizeof(qname));
        TaskHandle_t h = xTaskGetHandle(qname);
        if (!h) continue;
        n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\":%u", first ? "" : ",",
                      tasks[i], (unsigned)uxTaskGetStackHighWaterMark(h));
        first = false;
    }
    snprintf(buf + n, sizeof(buf) - n, "}}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

// Reads the whole request body into buf (capacity buf_cap, NUL-terminated on
// success). A single httpd_req_recv() call only reads up to its own buffer
// size — if content_len is bigger, the rest is left unread on the socket and
// httpd misreads it as the start of the next request on a keep-alive
// connection. Sends an HTTP error and returns -1 on failure/oversized body.
static int recv_full_body(httpd_req_t *req, char *buf, size_t buf_cap) {
    int remaining = req->content_len > 0 ? (int)req->content_len : (int)buf_cap - 1;
    if (remaining > (int)buf_cap - 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return -1;
    }
    int total = 0, timeouts = 0;
    while (total < remaining) {
        int ret = httpd_req_recv(req, buf + total, (size_t)(remaining - total));
        if (ret <= 0) {
            // A client that keeps stalling could otherwise retry here forever,
            // tying up an httpd worker indefinitely.
            if (ret == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 5) continue;
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv error");
            return -1;
        }
        timeouts = 0;
        total += ret;
    }
    buf[total] = '\0';
    return total;
}

// POST /api/aprs/send
// Body JSON: {"to":"NO0CAL","ssid":0,"text":"Hello"}
// Encodes and queues an APRS message frame via afsk_queue_tx_frame (safe from
// any task; dispatched by receive_audio_task). Works in both KISS TNC and APRS mode.
static esp_err_t aprs_send_handler(httpd_req_t *req) {
    char body[256];
    int len = recv_full_body(req, body, sizeof(body));
    if (len <= 0) return ESP_FAIL;

    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *j_to   = cJSON_GetObjectItem(root, "to");
    cJSON *j_ssid = cJSON_GetObjectItem(root, "ssid");
    cJSON *j_text = cJSON_GetObjectItem(root, "text");

    if (!cJSON_IsString(j_to) || !cJSON_IsString(j_text)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing \"to\" or \"text\"");
        return ESP_FAIL;
    }

    int ssid_val = (j_ssid && cJSON_IsNumber(j_ssid)) ? (int)j_ssid->valuedouble : 0;
    if (ssid_val < 0)  ssid_val = 0;
    if (ssid_val > 15) ssid_val = 15;

    int seq = APRS_queue_msg(j_to->valuestring, ssid_val, j_text->valuestring);
    ESP_LOGI(TAG, "APRS MSG queued → %s-%d: %s (seq=%d)", j_to->valuestring, ssid_val, j_text->valuestring, seq);

    char resp[48];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"msg_id\":%d}", seq);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// POST /api/aprs/beacon
// Body JSON: {"lat": 40.4168, "lon": -3.7038, "symbol": ">", "comment": "ESP32"}
// Converts decimal coordinates to APRS uncompressed position and queues for TX.
static esp_err_t aprs_beacon_handler(httpd_req_t *req) {
    char body[256];
    int len = recv_full_body(req, body, sizeof(body));
    if (len <= 0) return ESP_FAIL;

    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *j_lat     = cJSON_GetObjectItem(root, "lat");
    cJSON *j_lon     = cJSON_GetObjectItem(root, "lon");
    cJSON *j_symbol  = cJSON_GetObjectItem(root, "symbol");
    cJSON *j_comment = cJSON_GetObjectItem(root, "comment");
    cJSON *j_course  = cJSON_GetObjectItem(root, "course");  // degrees 0-360
    cJSON *j_speed   = cJSON_GetObjectItem(root, "speed");   // knots

    if (!cJSON_IsNumber(j_lat) || !cJSON_IsNumber(j_lon)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing lat/lon");
        return ESP_FAIL;
    }

    double lat = j_lat->valuedouble;
    double lon = j_lon->valuedouble;
    char  sym = (j_symbol && cJSON_IsString(j_symbol) && j_symbol->valuestring[0])
                    ? j_symbol->valuestring[0] : '>';
    const char *comment = (j_comment && cJSON_IsString(j_comment))
                              ? j_comment->valuestring : "";

    bool has_cse = cJSON_IsNumber(j_course) && cJSON_IsNumber(j_speed);
    int  cse = has_cse ? (int)j_course->valuedouble : 0;
    int  spd = has_cse ? (int)j_speed->valuedouble  : 0;
    if (cse < 0 || cse > 360) cse = 0;
    if (spd < 0 || spd > 999) spd = 0;

    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Coordinates out of range");
        return ESP_FAIL;
    }

    // APRS info field: =DDMM.mmN/DDDMM.mmE sym [cse/spd] comment
    // (tabla de símbolos '/', formato compartido en aprs_format_position)
    char pos[APRS_POS_BUF_LEN];
    aprs_format_position(pos, sizeof(pos), lat, lon, sym);

    char info[120];
    if (has_cse)
        snprintf(info, sizeof(info), "%s%03d/%03d%s", pos, cse, spd, comment);
    else
        snprintf(info, sizeof(info), "%s%s", pos, comment);

    ESP_LOGI(TAG, "APRS beacon TX: %s", info);
    APRS_queue_beacon(info);

    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// GET /api/config → devuelve el config.json completo almacenado en SPIFFS.
static esp_err_t config_get_handler(httpd_req_t *req) {
    FILE *f = fopen(CONFIG_FILE_PATH, "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot open config");
        return ESP_FAIL;
    }
    fseek(f, 0, SEEK_END); long fsz = ftell(f); fseek(f, 0, SEEK_SET);
    if (fsz < 0) {
        // ftell() failure used to fall through as (size_t)-1 + 1 == 0 into
        // malloc(0), followed by fread(buf, 1, (size_t)-1, f) — a huge read
        // into a near-zero-size buffer.
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ftell failed");
        return ESP_FAIL;
    }
    char *buf = malloc((size_t)fsz + 1);
    if (!buf) { fclose(f); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM"); return ESP_FAIL; }
    size_t rd = fread(buf, 1, (size_t)fsz, f); fclose(f); buf[rd] = '\0';
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, buf);
    free(buf);
    return ESP_OK;
}

// POST /api/config → guarda el body como nuevo config.json, recarga en RAM.
// El body debe ser JSON válido (<= CONFIG_BODY_MAX). Recarga inmediata sin reiniciar.
// El buffer se reserva en el heap solo mientras dura la petición (antes era un
// static de 2 KB permanente y el límite dejaba poco margen sobre el config real).
#define CONFIG_BODY_MAX 8192

static esp_err_t config_post_handler(httpd_req_t *req) {
    int remaining = (int)req->content_len;
    if (remaining <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length required");
        return ESP_FAIL;
    }
    if (remaining > CONFIG_BODY_MAX) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large (max 8 KB)");
        return ESP_FAIL;
    }
    char *body = malloc((size_t)remaining + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int total = 0, ret, timeouts = 0;
    while (total < remaining) {
        ret = httpd_req_recv(req, body + total, (size_t)(remaining - total));
        if (ret <= 0) {
            // A client that keeps stalling could otherwise retry here forever,
            // tying up an httpd worker indefinitely.
            if (ret == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 5) continue;
            free(body);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv error");
            return ESP_FAIL;
        }
        timeouts = 0;
        total += ret;
    }
    body[total] = '\0';

    // Validate JSON before saving
    cJSON *parsed = cJSON_Parse(body);
    if (!parsed) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON_Delete(parsed);

    bool saved = save_config(body);
    free(body);
    if (!saved) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
        return ESP_FAIL;
    }

    // Reload config into RAM and re-apply runtime-updatable settings
    cJSON *new_cfg = config_reload();
    if (new_cfg) {
        digi_init(new_cfg);
        morse_init(new_cfg);
        config_free_json(new_cfg);   // config_reload() devuelve una copia: liberarla
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// POST /api/morse/trigger → trigger an immediate one-shot morse beacon.
static esp_err_t morse_trigger_handler(httpd_req_t *req) {
    morse_trigger_now();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// Modo del AP frente a los sondeos de conectividad del sistema operativo.
//   false (por defecto): se responde como si hubiera Internet. Android/iOS/Windows
//     dan la red por "validada", no muestran "Iniciar sesión en la red" y no la
//     abandonan al rato por "sin Internet". Contrapartida: el móvil intentará usar
//     esta WiFi (sin salida) como red por defecto mientras esté conectado.
//   true (ap.captive_portal): cualquier URL desconocida redirige a "/" (portal
//     cautivo clásico); Android muestra el aviso de inicio de sesión.
static bool s_captive_portal = false;

// Rutas de sondeo conocidas → respuesta que cada SO espera para dar la red por buena.
static esp_err_t probe_reply(httpd_req_t *req) {
    size_t plen = strcspn(req->uri, "?#");
    #define URI_IS(lit) (plen == sizeof(lit) - 1 && strncmp(req->uri, lit, plen) == 0)
    if (URI_IS("/generate_204") || URI_IS("/gen_204")) {          // Android / Chrome
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, NULL, 0);
    }
    const char *type = "text/plain", *body = NULL;
    if (URI_IS("/hotspot-detect.html") || URI_IS("/library/test/success.html")) {  // Apple
        type = "text/html";
        body = "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
    } else if (URI_IS("/connecttest.txt")) {                       // Windows 10/11
        body = "Microsoft Connect Test";
    } else if (URI_IS("/ncsi.txt")) {                              // Windows (NCSI)
        body = "Microsoft NCSI";
    } else if (URI_IS("/success.txt")) {                           // Firefox
        body = "success\n";
    }
    #undef URI_IS
    if (!body) return ESP_ERR_NOT_FOUND;
    httpd_resp_set_type(req, type);
    return httpd_resp_sendstr(req, body);
}

// 404: sondeos de conectividad (Android/iOS/Windows/Firefox) o redirección a "/".
static esp_err_t captive_redirect_handler(httpd_req_t *req, httpd_err_code_t err) {
    (void)err;
    if (!s_captive_portal && probe_reply(req) == ESP_OK) return ESP_OK;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}


// Con el OPTIONS "/*" (preflight CORS) toda URI desconocida casa por ruta y un GET
// da 405 en vez de 404: un GET a una ruta inexistente debe seguir tratándose como 404
// (sondeos de conectividad, favicon...). Un 405 genuino (p. ej. POST a una ruta GET) se mantiene.
static esp_err_t not_allowed_handler(httpd_req_t *req, httpd_err_code_t err) {
    if (req->method == HTTP_GET || req->method == HTTP_HEAD) return captive_redirect_handler(req, err);
    httpd_resp_set_status(req, "405 Method Not Allowed");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// ─── Reboot endpoint ──────────────────────────────────────────────────────────

static void reboot_delay_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

static esp_err_t reboot_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    xTaskCreate(reboot_delay_task, "reboot", 1024, NULL, 3, NULL);
    return ESP_OK;
}

static esp_err_t spiffs_upload_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");   // subida desde el servidor de desarrollo
    // ?name=<filename> — must be a bare name, no slashes, no '..'
    char qs[128] = {0};
    char name[64] = {0};
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK ||
        httpd_query_key_value(qs, "name", name, sizeof(name)) != ESP_OK ||
        name[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"missing ?name= param\"}");
        return ESP_OK;
    }
    if (strchr(name, '/') || strstr(name, "..")) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"invalid filename\"}");
        return ESP_OK;
    }

    int total = (int)req->content_len;
    if (total <= 0 || total > 512 * 1024) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"invalid content-length (max 512 KB)\"}");
        return ESP_OK;
    }

    char path[80];
    snprintf(path, sizeof(path), "/spiffs/%s", name);

    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGE(TAG, "spiffs_upload: fopen(%s) failed", path);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"cannot open file for writing\"}");
        return ESP_OK;
    }

    static char buf[1024];  // seguro: httpd es mono-hilo (ver config_post_handler)
    int received = 0;
    bool ok = true;
    int timeouts = 0;
    while (received < total) {
        int want = total - received;
        if (want > (int)sizeof(buf)) want = (int)sizeof(buf);
        int got = httpd_req_recv(req, buf, (size_t)want);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            // A client that keeps stalling could otherwise retry here forever,
            // tying up an httpd worker indefinitely.
            if (++timeouts <= 5) continue;
            ESP_LOGE(TAG, "spiffs_upload: too many recv timeouts at %d/%d", received, total);
            ok = false;
            break;
        }
        timeouts = 0;
        if (got <= 0) {
            ESP_LOGE(TAG, "spiffs_upload: recv error at %d/%d (ret=%d)", received, total, got);
            ok = false;
            break;
        }
        if (fwrite(buf, 1, (size_t)got, f) != (size_t)got) {
            ESP_LOGE(TAG, "spiffs_upload: fwrite failed (disk full?)");
            ok = false;
            break;
        }
        received += got;
    }
    fclose(f);

    if (!ok) {
        remove(path);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"upload aborted\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "spiffs_upload: %s (%d bytes)", path, received);
    char resp[96];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"name\":\"%s\",\"size\":%d}", name, received);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// ─── Explorador SPIFFS: listar / descargar / borrar ───────────────────────────

// Nombre relativo válido: no vacío, sin ".." ni barra inicial ni caracteres de control.
// Se admite una barra interior (SPIFFS es plano: "sstv/foto.jpg" es un nombre más).
static bool spiffs_name_ok(const char *name)
{
    if (!name[0] || name[0] == '/' || strstr(name, "..")) return false;
    for (const char *c = name; *c; c++) if ((unsigned char)*c < 0x20) return false;
    return true;
}

static bool spiffs_query_name(httpd_req_t *req, char *name, size_t len)
{
    char qs[160] = {0};
    return httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK &&
           httpd_query_key_value(qs, "name", name, len) == ESP_OK &&
           spiffs_name_ok(name);
}

static void spiffs_reply_err(httpd_req_t *req, const char *status, const char *msg)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, msg);
}

// GET /api/spiffs/list → {"total":N,"used":N,"files":[{"name":..,"size":..},...]}
static esp_err_t spiffs_list_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(root, "files");
    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    cJSON_AddNumberToObject(root, "total", (double)total);
    cJSON_AddNumberToObject(root, "used", (double)used);
    DIR *dir = opendir("/spiffs");
    if (dir) {
        struct dirent *e;
        while ((e = readdir(dir)) != NULL) {
            char path[sizeof("/spiffs/") + sizeof(e->d_name)];
            snprintf(path, sizeof(path), "/spiffs/%s", e->d_name);
            struct stat st;
            if (stat(path, &st) != 0) continue;
            cJSON *o = cJSON_CreateObject();
            cJSON_AddStringToObject(o, "name", e->d_name);
            cJSON_AddNumberToObject(o, "size", (double)st.st_size);
            cJSON_AddItemToArray(arr, o);
        }
        closedir(dir);
    }
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!out) { spiffs_reply_err(req, "500 Internal Server Error", "{\"error\":\"out of memory\"}"); return ESP_OK; }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out);
    return ESP_OK;
}

// POST /api/spiffs/delete?name=<fichero>
static esp_err_t spiffs_delete_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char name[64] = {0};
    if (!spiffs_query_name(req, name, sizeof(name))) {
        spiffs_reply_err(req, "400 Bad Request", "{\"error\":\"missing or invalid ?name=\"}");
        return ESP_OK;
    }
    char path[80];
    snprintf(path, sizeof(path), "/spiffs/%s", name);
    if (remove(path) != 0) {
        spiffs_reply_err(req, "404 Not Found", "{\"error\":\"not found\"}");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "spiffs_delete: %s", path);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// GET /api/spiffs/get?name=<fichero> → descarga el fichero (adjunto)
static esp_err_t spiffs_get_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char name[64] = {0};
    if (!spiffs_query_name(req, name, sizeof(name))) {
        spiffs_reply_err(req, "400 Bad Request", "{\"error\":\"missing or invalid ?name=\"}");
        return ESP_OK;
    }
    char path[80];
    snprintf(path, sizeof(path), "/spiffs/%s", name);
    FILE *f = fopen(path, "r");
    if (!f) { spiffs_reply_err(req, "404 Not Found", "{\"error\":\"not found\"}"); return ESP_OK; }
    char cd[112];
    snprintf(cd, sizeof(cd), "attachment; filename=\"%s\"", strrchr(name, '/') ? strrchr(name, '/') + 1 : name);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", cd);
    static char buf[1024];  // httpd es mono-hilo (ver spiffs_upload_handler)
    size_t n;
    esp_err_t r = ESP_OK;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, (ssize_t)n) != ESP_OK) { r = ESP_FAIL; break; }
    }
    fclose(f);
    if (r == ESP_OK) httpd_resp_send_chunk(req, NULL, 0);
    return r;
}

// ─── GET /api/repeater/status ─────────────────────────────────────────────────

static const char *repeater_state_str(repeater_state_t st)
{
    switch (st) {
        case REPEATER_STATE_IDLE:      return "idle";
        case REPEATER_STATE_RECORDING: return "recording";
        case REPEATER_STATE_TAIL:      return "tail";
        case REPEATER_STATE_PENDING:   return "pending";
        case REPEATER_STATE_TX:        return "tx";
        default:                       return "unknown";
    }
}

static esp_err_t repeater_status_handler(httpd_req_t *req)
{
    char buf[80];
    snprintf(buf, sizeof(buf), "{\"enabled\":%s,\"state\":\"%s\"}",
             repeater_is_enabled() ? "true" : "false",
             repeater_state_str(repeater_get_state()));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

// ─── POST /api/repeater/enable ────────────────────────────────────────────────

static esp_err_t repeater_enable_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int got = recv_full_body(req, body, sizeof(body));
    if (got <= 0) return ESP_FAIL;
    cJSON *j = cJSON_ParseWithLength(body, (size_t)got);
    if (!j) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_FAIL;
    }
    cJSON *en = cJSON_GetObjectItem(j, "enabled");
    if (cJSON_IsBool(en)) repeater_set_enabled(cJSON_IsTrue(en));
    cJSON_Delete(j);

    char resp[96];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"enabled\":%s}",
             repeater_is_enabled() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// ─── POST /api/squelch/sf_config ─────────────────────────────────────────────

static esp_err_t squelch_sf_config_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int got = recv_full_body(req, body, sizeof(body));
    if (got <= 0) return ESP_FAIL;
    cJSON *j = cJSON_ParseWithLength(body, (size_t)got);
    if (!j) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json"); return ESP_FAIL; }

    cJSON *thr = cJSON_GetObjectItem(j, "hfne_threshold");
    if (cJSON_IsNumber(thr)) squelch_hfne_set_threshold((float)thr->valuedouble);
    cJSON_Delete(j);

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"hfne_thr\":%.2f}", (double)squelch_hfne_get_threshold());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// ─── GET /api/squelch/status ──────────────────────────────────────────────────

static esp_err_t squelch_status_handler(httpd_req_t *req)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"active\":%s,\"manual\":%s,\"hfne\":%.3f,\"open\":%s,\"thr\":%.2f}",
             squelch_sf_is_active()         ? "true" : "false",
             squelch_sf_is_monitor_active() ? "true" : "false",
             (double)squelch_hfne_get_value(),
             squelch_hfne_is_open()         ? "true" : "false",
             (double)squelch_hfne_get_threshold());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

// ─── POST /api/squelch/monitor ────────────────────────────────────────────────

static esp_err_t squelch_monitor_handler(httpd_req_t *req)
{
    char body[32] = {0};
    int got = recv_full_body(req, body, sizeof(body));
    if (got <= 0) return ESP_FAIL;
    cJSON *j = cJSON_ParseWithLength(body, (size_t)got);
    if (!j) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json"); return ESP_FAIL; }
    cJSON *en = cJSON_GetObjectItem(j, "active");
    if (cJSON_IsBool(en)) squelch_sf_set_monitor_active(cJSON_IsTrue(en));
    cJSON_Delete(j);
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"active\":%s,\"manual\":%s}",
             squelch_sf_is_active()         ? "true" : "false",
             squelch_sf_is_monitor_active() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// ─── Modo SURVEY (medición de audio/canal para calibración) ──────────────────

static esp_err_t carrier_status_handler(httpd_req_t *req)
{
    char buf[200];
    carrier_detect_status_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t survey_send_status(httpd_req_t *req)
{
    char *js = rx_survey_status_json();
    if (!js) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "sin memoria"); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t e = httpd_resp_sendstr(req, js);
    free(js);
    return e;
}

static esp_err_t survey_status_handler(httpd_req_t *req) { return survey_send_status(req); }

static esp_err_t survey_start_handler(httpd_req_t *req)
{
    if (!rx_survey_start() && !rx_survey_is_active()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no se pudo iniciar (memoria o cierre en curso)");
        return ESP_FAIL;
    }
    return survey_send_status(req);
}

static esp_err_t survey_stop_handler(httpd_req_t *req)
{
    rx_survey_stop();
    return survey_send_status(req);
}

static esp_err_t survey_mark_handler(httpd_req_t *req)
{
    char body[48] = {0};
    int got = recv_full_body(req, body, sizeof(body));
    if (got <= 0) return ESP_FAIL;
    cJSON *j = cJSON_ParseWithLength(body, (size_t)got);
    if (!j) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json"); return ESP_FAIL; }
    cJSON *m = cJSON_GetObjectItem(j, "mark");
    if (cJSON_IsNumber(m)) rx_survey_set_mark(m->valueint);
    cJSON_Delete(j);
    char resp[32];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"mark\":%d}", rx_survey_get_mark());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

void api_rest_register(httpd_handle_t server)
{
    static const httpd_uri_t routes[] = {
        { .uri = "/", .method = HTTP_GET, .handler = index_handler },
        { .uri = "/api/aprs/send", .method = HTTP_POST, .handler = aprs_send_handler },
        { .uri = "/api/me", .method = HTTP_GET, .handler = me_handler },
        { .uri = "/api/sys", .method = HTTP_GET, .handler = sys_handler },
        { .uri = "/api/aprs/beacon", .method = HTTP_POST, .handler = aprs_beacon_handler },
        { .uri = "/api/config", .method = HTTP_GET, .handler = config_get_handler },
        { .uri = "/api/config", .method = HTTP_POST, .handler = config_post_handler },
        { .uri = "/api/morse/trigger", .method = HTTP_POST, .handler = morse_trigger_handler },
        { .uri = "/api/reboot", .method = HTTP_POST, .handler = reboot_handler },
        { .uri = "/api/spiffs/upload", .method = HTTP_POST, .handler = spiffs_upload_handler },
        { .uri = "/api/spiffs/list",   .method = HTTP_GET,  .handler = spiffs_list_handler },
        { .uri = "/api/spiffs/delete", .method = HTTP_POST, .handler = spiffs_delete_handler },
        { .uri = "/api/spiffs/get",    .method = HTTP_GET,  .handler = spiffs_get_handler },
        { .uri = "/api/repeater/status", .method = HTTP_GET, .handler = repeater_status_handler },
        { .uri = "/api/repeater/enable", .method = HTTP_POST, .handler = repeater_enable_handler },
        { .uri = "/api/squelch/sf_config", .method = HTTP_POST, .handler = squelch_sf_config_handler },
        { .uri = "/api/squelch/status", .method = HTTP_GET, .handler = squelch_status_handler },
        { .uri = "/api/squelch/monitor", .method = HTTP_POST, .handler = squelch_monitor_handler },
        { .uri = "/api/carrier", .method = HTTP_GET, .handler = carrier_status_handler },
        { .uri = "/api/survey", .method = HTTP_GET, .handler = survey_status_handler },
        { .uri = "/api/survey/start", .method = HTTP_POST, .handler = survey_start_handler },
        { .uri = "/api/survey/stop", .method = HTTP_POST, .handler = survey_stop_handler },
        { .uri = "/api/survey/mark", .method = HTTP_POST, .handler = survey_mark_handler },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        if (httpd_register_uri_handler(server, &routes[i]) != ESP_OK)
            ESP_LOGE(TAG, "register_uri %s failed", routes[i].uri);
    }

    {
        cJSON *c = config_get();
        if (c) {
            cJSON *ap = cJSON_GetObjectItem(c, "ap");
            cJSON *it = ap ? cJSON_GetObjectItem(ap, "captive_portal") : NULL;
            s_captive_portal = cJSON_IsTrue(it);
            config_free_json(c);
        }
        ESP_LOGI(TAG, "AP: sondeos de conectividad %s",
                 s_captive_portal ? "-> portal cautivo (redirect)" : "-> responden OK (sin aviso de Internet)");
    }
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, captive_redirect_handler);
    httpd_register_err_handler(server, HTTPD_405_METHOD_NOT_ALLOWED, not_allowed_handler);
}

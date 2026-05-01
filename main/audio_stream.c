#include "audio_stream.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <lwip/sockets.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "cJSON.h"
#include "LibAPRS.h"
#include "aux_config.h"
#include "digipeater.h"
#include "morse.h"

#define TAG "audio_stream"

QueueHandle_t audio_stream_q = NULL;

// ─── IMA ADPCM ───────────────────────────────────────────────────────────────

static const int step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41,
    45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190,
    209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499,
    2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845,
    8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385,
    24623, 27086, 29794, 32767
};
static const int index_table[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

typedef struct { int32_t predictor; int8_t step_index; } adpcm_state_t;

static uint8_t adpcm_encode(adpcm_state_t *st, int16_t sample) {
    int diff = sample - (int16_t)st->predictor;
    uint8_t nibble = 0;
    if (diff < 0) { nibble = 8; diff = -diff; }
    int step = step_table[(int)st->step_index];
    if (diff >= step)       { nibble |= 4; diff -= step; }
    if (diff >= (step >> 1)){ nibble |= 2; diff -= step >> 1; }
    if (diff >= (step >> 2)){ nibble |= 1; }

    int delta = step >> 3;
    if (nibble & 4) delta += step;
    if (nibble & 2) delta += step >> 1;
    if (nibble & 1) delta += step >> 2;
    if (nibble & 8) st->predictor -= delta;
    else            st->predictor += delta;
    if (st->predictor >  32767) st->predictor =  32767;
    if (st->predictor < -32768) st->predictor = -32768;

    st->step_index += index_table[nibble & 7];
    if (st->step_index < 0)  st->step_index = 0;
    if (st->step_index > 88) st->step_index = 88;
    return nibble & 0x0F;
}

// Codifica 1017 muestras int8 en un bloque ADPCM WAV de 512 bytes.
// samples[0] se guarda como predictor de cabecera (no encoded); samples[1..1016]
// se empacan como nibbles (low nibble primero), 2 por byte → 508 bytes de datos.
static void encode_block(adpcm_state_t *st, const int8_t *samples, uint8_t *block) {
    // La primera muestra es el predictor inicial (no se codifica como nibble)
    int16_t first = (int16_t)samples[0] << 8;
    st->predictor  = first;
    // step_index se hereda del bloque anterior (estado continuo)

    block[0] = (uint8_t)(st->predictor & 0xFF);
    block[1] = (uint8_t)((st->predictor >> 8) & 0xFF);
    block[2] = (uint8_t)st->step_index;
    block[3] = 0;

    for (int i = 0; i < 508; i++) {
        int si = 1 + i * 2;
        uint8_t lo = adpcm_encode(st, (int16_t)samples[si]     << 8);
        uint8_t hi = adpcm_encode(st, (int16_t)samples[si + 1] << 8);
        block[4 + i] = (uint8_t)(lo | (hi << 4));
    }
}

// ─── WAV header ──────────────────────────────────────────────────────────────

// nAvgBytesPerSec = 512 * 9600 / 1017 = 4834
#define WAV_AVG_BYTES_SEC 4834U

static void build_wav_header(uint8_t *buf) {
    // RIFF
    memcpy(buf,      "RIFF", 4); buf += 4;
    uint32_t v = 0xFFFFFFFFU;
    memcpy(buf, &v, 4);           buf += 4;
    memcpy(buf,      "WAVE", 4); buf += 4;
    // fmt  (chunk size 20 = extended ADPCM)
    memcpy(buf,      "fmt ", 4); buf += 4;
    v = 20; memcpy(buf, &v, 4);  buf += 4;
    uint16_t u;
    u = 0x0011; memcpy(buf, &u, 2); buf += 2; // IMA ADPCM
    u = 1;      memcpy(buf, &u, 2); buf += 2; // channels
    v = ADPCM_SAMPLE_RATE; memcpy(buf, &v, 4); buf += 4;
    v = WAV_AVG_BYTES_SEC; memcpy(buf, &v, 4); buf += 4;
    u = ADPCM_BLOCK_BYTES; memcpy(buf, &u, 2); buf += 2; // block align
    u = 4;      memcpy(buf, &u, 2); buf += 2; // bits per sample
    u = 2;      memcpy(buf, &u, 2); buf += 2; // cbSize
    u = ADPCM_SAMPLES_BLOCK; memcpy(buf, &u, 2); buf += 2;
    // fact
    memcpy(buf,      "fact", 4); buf += 4;
    v = 4; memcpy(buf, &v, 4);   buf += 4;
    v = 0; memcpy(buf, &v, 4);   buf += 4;
    // data
    memcpy(buf,      "data", 4); buf += 4;
    v = 0xFFFFFFFFU; memcpy(buf, &v, 4);
}
#define WAV_HEADER_LEN 60

// ─── Shared stream state ──────────────────────────────────────────────────────

static httpd_handle_t s_httpd      = NULL;
static QueueHandle_t  s_wav_q      = NULL; // blocks for WAV TCP client

// ─── audio_stream_task ───────────────────────────────────────────────────────

static int8_t  s_sample_buf[ADPCM_SAMPLES_BLOCK];
static uint8_t s_adpcm_block[ADPCM_BLOCK_BYTES];

// En ESP-IDF 6.1 el handler WebSocket NO se llama al conectar el cliente,
// solo cuando el cliente envía frames. Para streaming unidireccional (server→client)
// usamos httpd_get_client_list + httpd_ws_get_fd_info para encontrar clientes WS.
static void audio_stream_task(void *arg) {
    adpcm_state_t state = { .predictor = 0, .step_index = 0 };
    int sample_count = 0;
    int client_fds[5];

    for (;;) {
        int8_t sample;
        if (xQueueReceive(audio_stream_q, &sample, pdMS_TO_TICKS(150)) != pdTRUE) {
            sample_count = 0;
            continue;
        }
        s_sample_buf[sample_count++] = sample;

        if (sample_count < ADPCM_SAMPLES_BLOCK) continue;
        sample_count = 0;

        encode_block(&state, s_sample_buf, s_adpcm_block);

        // Enviar a todos los clientes WebSocket activos.
        if (s_httpd != NULL) {
            size_t n = sizeof(client_fds) / sizeof(client_fds[0]);
            if (httpd_get_client_list(s_httpd, &n, client_fds) == ESP_OK) {
                for (int i = 0; i < (int)n; i++) {
                    if (httpd_ws_get_fd_info(s_httpd, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                        httpd_ws_frame_t pkt = {
                            .final      = true,
                            .fragmented = false,
                            .type       = HTTPD_WS_TYPE_BINARY,
                            .payload    = s_adpcm_block,
                            .len        = ADPCM_BLOCK_BYTES,
                        };
                        if (httpd_ws_send_data(s_httpd, client_fds[i], &pkt) != ESP_OK) {
                            // Socket muerto: cerrar sesión para que httpd lo limpie
                            httpd_sess_trigger_close(s_httpd, client_fds[i]);
                        }
                    }
                }
            }
        }

        // Encolar para el cliente WAV TCP si está activo (lossy).
        if (s_wav_q != NULL) {
            xQueueSendToBack(s_wav_q, s_adpcm_block, 0);
        }
    }
}

// ─── HTTP handlers ────────────────────────────────────────────────────────────

// GET / → sirve /index.html desde SPIFFS
static esp_err_t index_handler(httpd_req_t *req) {
    FILE *f = fopen("/spiffs/index.html", "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "index.html not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/html");
    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, (ssize_t)n);
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

// POST /api/aprs/send
// Body JSON: {"to":"NO0CAL","ssid":0,"text":"Hello"}
// Encodes and queues an APRS message frame via afsk_queue_tx_frame (safe from
// any task; dispatched by receive_audio_task). Works in both KISS TNC and APRS mode.
static esp_err_t aprs_send_handler(httpd_req_t *req) {
    char body[256];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    body[len] = '\0';

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

    APRS_queue_msg(j_to->valuestring, ssid_val, j_text->valuestring);
    ESP_LOGI(TAG, "APRS MSG queued → %s-%d: %s", j_to->valuestring, ssid_val, j_text->valuestring);

    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// POST /api/aprs/beacon
// Body JSON: {"lat": 40.4168, "lon": -3.7038, "symbol": ">", "comment": "ESP32"}
// Converts decimal coordinates to APRS uncompressed position and queues for TX.
static esp_err_t aprs_beacon_handler(httpd_req_t *req) {
    char body[256];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *j_lat     = cJSON_GetObjectItem(root, "lat");
    cJSON *j_lon     = cJSON_GetObjectItem(root, "lon");
    cJSON *j_symbol  = cJSON_GetObjectItem(root, "symbol");
    cJSON *j_comment = cJSON_GetObjectItem(root, "comment");

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

    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Coordinates out of range");
        return ESP_FAIL;
    }

    // APRS 1.01 §8 uncompressed position: DDMM.HHN (lat) DDDMM.HHW (lon)
    // Use double throughout to avoid float precision loss (~7 sig. digits).
    // Clamp minutes to 59.99 to prevent %05.2f producing "60.00" on rounding edge.
    int    lat_d = (int)fabs(lat);
    double lat_m = (fabs(lat) - lat_d) * 60.0;
    if (lat_m >= 60.0) { lat_d++; lat_m = 0.0; }
    char lat_str[9];  // DDMM.HHN\0  (2+5+1+1 = 9)
    snprintf(lat_str, sizeof(lat_str), "%02d%05.2f%c",
             lat_d, lat_m, lat >= 0.0 ? 'N' : 'S');

    int    lon_d = (int)fabs(lon);
    double lon_m = (fabs(lon) - lon_d) * 60.0;
    if (lon_m >= 60.0) { lon_d++; lon_m = 0.0; }
    char lon_str[10]; // DDDMM.HHE\0 (3+5+1+1 = 10)
    snprintf(lon_str, sizeof(lon_str), "%03d%05.2f%c",
             lon_d, lon_m, lon >= 0.0 ? 'E' : 'W');

    // APRS info field: =lat/lon_sym_comment  (symbol table = '/', primary)
    // '=' means "position without timestamp, APRS messaging enabled".
    // Functionally identical to '!' but some radios (e.g. Yaesu) treat it
    // differently in their station list display.
    char info[120];
    snprintf(info, sizeof(info), "=%s/%s%c%s", lat_str, lon_str, sym, comment);

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
// El body debe ser JSON válido (<= 2 KB). Recarga immediate sin reiniciar.
static esp_err_t config_post_handler(httpd_req_t *req) {
    // Límite de 2 KB para el body
    static char body[2048];
    int total = 0, ret;
    int remaining = req->content_len > 0 ? (int)req->content_len : (int)sizeof(body) - 1;
    if (remaining > (int)sizeof(body) - 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return ESP_FAIL;
    }
    while (total < remaining) {
        ret = httpd_req_recv(req, body + total, (size_t)(remaining - total));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv error");
            return ESP_FAIL;
        }
        total += ret;
    }
    body[total] = '\0';

    // Validate JSON before saving
    cJSON *parsed = cJSON_Parse(body);
    if (!parsed) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON_Delete(parsed);

    if (!save_config(body)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
        return ESP_FAIL;
    }

    // Reload config into RAM and re-apply runtime-updatable settings
    cJSON *new_cfg = config_reload();
    if (new_cfg) {
        digi_init(new_cfg);
        morse_init(new_cfg);
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

// 404 → redirect to / (captive-portal probes from Android/iOS/Windows)
static esp_err_t captive_redirect_handler(httpd_req_t *req, httpd_err_code_t err) {
    (void)err;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// GET /ws → WebSocket upgrade.
// En ESP-IDF 6.1 este handler se llama:
//   a) en el handshake inicial (req->method == HTTP_GET)
//   b) cada vez que el cliente envía un frame
// Es CRÍTICO consumir el payload completo de cada frame, de lo contrario los
// bytes sobrantes quedan en el buffer TCP y la próxima lectura intenta parsearlos
// como cabecera de frame → "WS frame is not properly masked" en bucle.
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // Handshake inicial — solo confirmar upgrade
        return ESP_OK;
    }

    // 1. Leer cabecera para obtener tipo y longitud del payload
    httpd_ws_frame_t pkt = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
    if (ret != ESP_OK) {
        return ret;  // httpd cerrará el socket al recibir error
    }

    // 2. Consumir el payload (si lo hay) para vaciar el buffer TCP
    uint8_t *buf = NULL;
    if (pkt.len > 0) {
        buf = malloc(pkt.len);
        if (!buf) return ESP_ERR_NO_MEM;
        pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
        if (ret != ESP_OK) {
            free(buf);
            return ret;
        }
    }

    // 3. Responder a frames de control
    if (pkt.type == HTTPD_WS_TYPE_PING) {
        httpd_ws_frame_t pong = {
            .final = true, .type = HTTPD_WS_TYPE_PONG,
            .payload = buf, .len = pkt.len,
        };
        httpd_ws_send_frame(req, &pong);
    } else if (pkt.type == HTTPD_WS_TYPE_CLOSE) {
        // Responder CLOSE y dejar que httpd cierre el socket
        httpd_ws_frame_t close_f = {
            .final = true, .type = HTTPD_WS_TYPE_CLOSE,
            .payload = NULL, .len = 0,
        };
        httpd_ws_send_frame(req, &close_f);
        free(buf);
        return ESP_FAIL;  // provoca cierre de sesión por parte de httpd
    }

    free(buf);
    return ESP_OK;
}

// ─── Servidor WAV TCP (puerto AUDIO_WAV_PORT) ─────────────────────────────────

static void wav_server_task(void *arg) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(AUDIO_WAV_PORT),
        .sin_addr   = { .s_addr = INADDR_ANY },
    };
    bind(srv, (struct sockaddr *)&addr, sizeof(addr));
    listen(srv, 1);
    ESP_LOGI(TAG, "Servidor WAV escuchando en puerto %d", AUDIO_WAV_PORT);

    for (;;) {
        struct sockaddr_in client;
        socklen_t clen = sizeof(client);
        int fd = accept(srv, (struct sockaddr *)&client, &clen);
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        ESP_LOGI(TAG, "Cliente WAV conectado");

        // Leer la petición HTTP (ignoramos el contenido, asumimos GET /audio)
        {
            char rbuf[256];
            recv(fd, rbuf, sizeof(rbuf) - 1, 0);
        }

        // Respuesta HTTP con WAV streaming
        const char *hdr =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: audio/wav\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n"
            "\r\n";
        send(fd, hdr, strlen(hdr), 0);

        uint8_t wav_hdr[WAV_HEADER_LEN];
        build_wav_header(wav_hdr);
        send(fd, wav_hdr, WAV_HEADER_LEN, 0);

        // Crear cola de bloques para esta sesión
        s_wav_q = xQueueCreate(8, ADPCM_BLOCK_BYTES);

        uint8_t block[ADPCM_BLOCK_BYTES];
        while (1) {
            if (xQueueReceive(s_wav_q, block, pdMS_TO_TICKS(500)) != pdTRUE) {
                // Timeout: comprobar si el socket sigue vivo
                if (send(fd, NULL, 0, MSG_NOSIGNAL) < 0) break;
                continue;
            }
            if (send(fd, block, ADPCM_BLOCK_BYTES, MSG_NOSIGNAL) < 0) break;
        }

        ESP_LOGI(TAG, "Cliente WAV desconectado");
        vQueueDelete(s_wav_q);
        s_wav_q = NULL;
        close(fd);
    }
}

// ─── Envío de texto a clientes WebSocket ─────────────────────────────────────

void audio_stream_ws_send_text(const char *text) {
    if (s_httpd == NULL || text == NULL) return;
    httpd_ws_frame_t pkt = {
        .final      = true,
        .fragmented = false,
        .type       = HTTPD_WS_TYPE_TEXT,
        .payload    = (uint8_t *)text,
        .len        = strlen(text),
    };
    int client_fds[5];
    size_t n = sizeof(client_fds) / sizeof(client_fds[0]);
    if (httpd_get_client_list(s_httpd, &n, client_fds) != ESP_OK) return;
    for (int i = 0; i < (int)n; i++) {
        if (httpd_ws_get_fd_info(s_httpd, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            if (httpd_ws_send_data(s_httpd, client_fds[i], &pkt) != ESP_OK) {
                httpd_sess_trigger_close(s_httpd, client_fds[i]);
            }
        }
    }
}

// ─── Punto de entrada ─────────────────────────────────────────────────────────

void audio_stream_init(void) {
    audio_stream_q = xQueueCreate(AUDIO_QUEUE_LEN, sizeof(int8_t));

    // HTTP server (index.html + WebSocket)
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = AUDIO_HTTP_PORT;
    cfg.stack_size       = 8192;
    cfg.max_open_sockets = 5;
    cfg.max_uri_handlers = 12;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando HTTP server");
        return;
    }

    static const httpd_uri_t uri_index = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = index_handler,
    };
    static const httpd_uri_t uri_ws = {
        .uri         = "/ws",
        .method      = HTTP_GET,
        .handler     = ws_handler,
        .is_websocket = true,
    };
    static const httpd_uri_t uri_aprs_send = {
        .uri     = "/api/aprs/send",
        .method  = HTTP_POST,
        .handler = aprs_send_handler,
    };
    static const httpd_uri_t uri_me = {
        .uri     = "/api/me",
        .method  = HTTP_GET,
        .handler = me_handler,
    };
    static const httpd_uri_t uri_beacon = {
        .uri     = "/api/aprs/beacon",
        .method  = HTTP_POST,
        .handler = aprs_beacon_handler,
    };
    static const httpd_uri_t uri_cfg_get = {
        .uri     = "/api/config",
        .method  = HTTP_GET,
        .handler = config_get_handler,
    };
    static const httpd_uri_t uri_cfg_post = {
        .uri     = "/api/config",
        .method  = HTTP_POST,
        .handler = config_post_handler,
    };
    static const httpd_uri_t uri_morse_trigger = {
        .uri     = "/api/morse/trigger",
        .method  = HTTP_POST,
        .handler = morse_trigger_handler,
    };
    httpd_register_uri_handler(s_httpd, &uri_index);
    httpd_register_uri_handler(s_httpd, &uri_ws);
    httpd_register_uri_handler(s_httpd, &uri_aprs_send);
    httpd_register_uri_handler(s_httpd, &uri_me);
    httpd_register_uri_handler(s_httpd, &uri_beacon);
    httpd_register_uri_handler(s_httpd, &uri_cfg_get);
    httpd_register_uri_handler(s_httpd, &uri_cfg_post);
    httpd_register_uri_handler(s_httpd, &uri_morse_trigger);
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, captive_redirect_handler);

    // Tarea de encoding + dispatch
    xTaskCreate(audio_stream_task, "audio_stream", 4096, NULL, 3, NULL);
    // Servidor WAV TCP
    xTaskCreate(wav_server_task,   "wav_server",   4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Streaming iniciado — http://ip/ (player) | http://ip:%d/audio (ffplay/VLC)",
             AUDIO_WAV_PORT);
}

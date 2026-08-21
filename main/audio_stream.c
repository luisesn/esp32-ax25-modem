#include "audio_stream.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <lwip/sockets.h>
#include <errno.h>
#include "esp_timer.h"
#include <string.h>
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
#include "repeater.h"

#define TAG "audio_stream"

// Must stay >= cfg.max_open_sockets below: httpd_get_client_list() only fills
// as many fds as this array can hold, so a smaller array silently hides
// clients past the cutoff from the WS broadcast loops (audio + JSON events).
#define AUDIO_STREAM_MAX_CLIENT_FDS 3

QueueHandle_t audio_stream_q = NULL;

// ─── RAM log buffer (last s_log_max entries of persistent WS messages) ────────
//
// High-frequency periodic events (audio_level, squelch_sf, repeater, rx_stats)
// are NOT stored — they fire every 100–500 ms and would evict APRS packets within
// seconds. Only discrete events (aprs, ack_sent, digipeated, sstv_*, ota_*,
// tune_*, morse_tx, digi_config) are persisted for replay on reconnect.
//
// Capacity is configurable: config.json "aprs_msg.max_stored" (default 25, max 250).

#define LOG_CAPACITY 250   // compile-time array size; runtime limit ≤ this

typedef struct { uint32_t seq; char *json; } log_entry_t;

static log_entry_t       s_log_ring[LOG_CAPACITY];
static int               s_log_head     = 0;
static int               s_log_count    = 0;
static uint32_t          s_log_seq_next = 1;
static SemaphoreHandle_t s_log_mutex    = NULL;
static int               s_log_max      = 25; // runtime effective capacity

// Returns true for high-frequency periodic events that should NOT be persisted.
static bool log_is_ephemeral(const char *text) {
    return strstr(text, "\"type\":\"audio_level\"")  != NULL ||
           strstr(text, "\"type\":\"squelch_sf\"")    != NULL ||
           strstr(text, "\"type\":\"repeater\"")       != NULL ||
           strstr(text, "\"type\":\"rx_stats\"")       != NULL;
}

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
void encode_block(adpcm_state_t *st, const int8_t *samples, uint8_t *block) {
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

static int8_t adpcm_decode_nibble(adpcm_state_t *st, uint8_t nibble) {
    int step  = step_table[(int)st->step_index];
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
    return (int8_t)(st->predictor >> 8);
}

void decode_block(const uint8_t *block, int8_t *samples) {
    adpcm_state_t st;
    st.predictor  = (int16_t)((uint16_t)block[0] | ((uint16_t)block[1] << 8));
    st.step_index = (int8_t)block[2];
    if (st.step_index < 0)  st.step_index = 0;
    if (st.step_index > 88) st.step_index = 88;
    samples[0] = (int8_t)(st.predictor >> 8);
    for (int i = 0; i < 508; i++) {
        uint8_t byte = block[4 + i];
        samples[1 + i * 2]     = adpcm_decode_nibble(&st, byte & 0x0F);
        samples[1 + i * 2 + 1] = adpcm_decode_nibble(&st, byte >> 4);
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

static httpd_handle_t s_httpd          = NULL;
static QueueHandle_t  s_wav_q          = NULL; // blocks for WAV TCP client
// Guards all reads/writes of s_wav_q across tasks. Copying the handle to a
// local before use (as audio_stream_task did) is not enough: the copy can
// still be read right before wav_server_task/audio_stream_stop_wav_server()
// calls vQueueDelete() on the same handle, leaving audio_stream_task about to
// send into freed memory. Both sides must serialize on this mutex instead.
static SemaphoreHandle_t s_wav_q_mutex = NULL;
static TaskHandle_t   s_wav_task_hdl   = NULL;
static volatile int   s_wav_srv_fd     = -1;
static volatile int   s_wav_client_fd  = -1; // fd of the connected WAV client, if any

httpd_handle_t audio_stream_get_httpd(void) { return s_httpd; }

// ─── audio_stream_task ───────────────────────────────────────────────────────

static int8_t  s_sample_buf[ADPCM_SAMPLES_BLOCK];
static uint8_t s_adpcm_block[ADPCM_BLOCK_BYTES];

// Contadores de diagnóstico del stream, expuestos en GET /api/rx/stats. Sin
// consola serie es la única forma de ver dónde se corta el audio:
//   samples≈0            → el hook del ADC no alimenta audio_stream_q
//   starve alto          → llegan muestras a ráfagas y el bloque se reinicia
//   blocks sube, sent=0  → los bloques se generan pero ws_try_send los descarta
//   drop sube            → sock_writable() da el socket por lleno
static volatile uint32_t s_a_samples = 0, s_a_blocks = 0;
static volatile uint32_t s_a_sent    = 0, s_a_drop   = 0, s_a_starve = 0;
static volatile uint32_t s_a_wsc     = 0;   // clientes WS vistos en el último bloque
static volatile uint32_t s_a_hs      = 0;   // handshakes /ws completados desde el arranque

// ─── Detección de clientes WS atascados ──────────────────────────────────────
//
// Un cliente que deja de drenar su socket (pestaña en segundo plano, móvil que
// se aleja del AP sin cerrar la conexión) llena el buffer TCP. Sin este control,
// httpd_ws_send_data() se bloqueaba hasta agotar send_wait_timeout dentro de
// audio_stream_task: eso congelaba el audio de TODOS los clientes durante ese
// tiempo y desbordaba audio_stream_q, además de dejar buffers TX del driver WiFi
// retenidos (síntomas: "httpd_sock_err: error in send : 11" y "wifi:m f null").
//
// El audio es en tiempo real: descartar el bloque es mejor que bloquear.

#define WS_STALL_SLOTS 8
#define WS_STALL_LIMIT 24   // ~2,5 s de bloques (106 ms) descartados → cerrar sesión

// Se guarda fd+1 para que 0 signifique "libre" con inicialización estática.
static struct { int fd1; uint16_t misses; } s_ws_stall[WS_STALL_SLOTS];

// Suma un fallo consecutivo al cliente `fd` y devuelve el total acumulado.
static uint16_t ws_stall_bump(int fd) {
    int free_i = -1;
    for (int i = 0; i < WS_STALL_SLOTS; i++) {
        if (s_ws_stall[i].fd1 == fd + 1) return ++s_ws_stall[i].misses;
        if (s_ws_stall[i].fd1 == 0 && free_i < 0) free_i = i;
    }
    if (free_i < 0) return WS_STALL_LIMIT;  // sin hueco libre: tratarlo como atascado
    s_ws_stall[free_i].fd1    = fd + 1;
    s_ws_stall[free_i].misses = 1;
    return 1;
}

static void ws_stall_clear(int fd) {
    for (int i = 0; i < WS_STALL_SLOTS; i++) {
        if (s_ws_stall[i].fd1 == fd + 1) {
            s_ws_stall[i].fd1    = 0;
            s_ws_stall[i].misses = 0;
            return;
        }
    }
}

// true si el socket admite escritura ahora mismo (select con timeout 0).
static bool sock_writable(int fd) {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);
    struct timeval tv = { .tv_sec = 0, .tv_usec = 0 };
    return select(fd + 1, NULL, &wfds, NULL, &tv) > 0;
}

// Escritura completa sobre el socket, instalada como send_fn de las sesiones WS.
//
// httpd_ws_send_frame_async() emite la cabecera y el payload con UN send() cada
// uno y solo comprueba `< 0` (httpd_ws.c:446-458): un envío PARCIAL —el socket
// admite 300 de los 512 B y send() devuelve el parcial al agotar SO_SNDTIMEO— lo
// da por bueno. Eso desincroniza el stream: el navegador lee la cabecera del
// bloque siguiente como payload del anterior, ve violación de protocolo y cierra
// la conexión, sin que aquí se registre ningún error. Síntoma medido: la sesión
// se abre, recibe unos segundos de audio y desaparece (sent 385 de 2421 bloques
// con ws_clients=0). sock_writable() no lo evita: select() solo garantiza que
// quepa 1 byte.
//
// Devolver <0 tras un parcial es correcto: el stream ya es irrecuperable y
// ws_try_send() cierra la sesión para que el navegador reconecte limpio.
#define WS_SEND_DEADLINE_US 500000   // 500 ms para colocar el buffer entero

static int ws_send_all(httpd_handle_t hd, int fd, const char *buf, size_t len, int flags) {
    (void)hd;
    size_t  off      = 0;
    int64_t deadline = esp_timer_get_time() + WS_SEND_DEADLINE_US;

    while (off < len) {
        int n = send(fd, buf + off, len - off, flags);
        if (n > 0) {
            off += (size_t)n;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            if (esp_timer_get_time() > deadline)
                return HTTPD_SOCK_ERR_TIMEOUT;   // cliente inservible
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        return HTTPD_SOCK_ERR_FAIL;   // n == 0 también es fallo: no hay progreso
    }
    return (int)len;
}

// Serializa los envíos WS: audio_stream_task y los emisores de texto (nivel de
// audio, squelch, APRS…) escriben sobre los mismos sockets desde tareas
// distintas, y httpd_ws_send_data() no es atómica — dos tramas entrelazadas
// rompen el stream WebSocket del navegador.
static SemaphoreHandle_t s_ws_tx_mutex = NULL;

// Envía `pkt` al cliente `fd` sin bloquear nunca la tarea llamante:
//   - socket sin espacio → se descarta y se cuenta como fallo;
//   - WS_STALL_LIMIT fallos seguidos → sesión dada por muerta y cerrada.
// Devuelve true si la trama se entregó al stack.
static bool ws_try_send(int fd, httpd_ws_frame_t *pkt) {
    if (!sock_writable(fd)) {
        if (ws_stall_bump(fd) >= WS_STALL_LIMIT) {
            ESP_LOGW(TAG, "WS fd=%d atascado, cerrando sesión", fd);
            ws_stall_clear(fd);
            httpd_sess_trigger_close(s_httpd, fd);
        }
        return false;
    }

    // El socket admite escritura, así que el send no debería esperar; el
    // timeout del mutex solo cubre el solape con otro emisor.
    if (s_ws_tx_mutex && xSemaphoreTake(s_ws_tx_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
        return false;
    // httpd_ws_send_frame_async() escribe el socket EN ESTA TAREA. No usar
    // httpd_ws_send_data(): esa encola el envío como mensaje de trabajo en el
    // socket de control UDP de httpd y espera (portMAX_DELAY) a que la tarea
    // httpd lo ejecute. La tarea httpd drena UN mensaje de control por vuelta de
    // select(), y por esa misma cola viajan los cierres LRU que habilitan
    // accept(); con ~9,4 bloques/s por cliente la cola se satura, los mensajes
    // (UDP) se pierden y las conexiones nuevas — el propio /ws — se quedan
    // minutos sin ser aceptadas. El mutex sustituye a esa serialización.
    esp_err_t err = httpd_ws_send_frame_async(s_httpd, fd, pkt);
    if (s_ws_tx_mutex) xSemaphoreGive(s_ws_tx_mutex);

    if (err != ESP_OK) {
        // Socket muerto: cerrar sesión para que httpd lo limpie.
        ws_stall_clear(fd);
        httpd_sess_trigger_close(s_httpd, fd);
        return false;
    }
    ws_stall_clear(fd);
    // Sin esto la sesión WS es siempre la víctima del purgado LRU: httpd solo
    // refresca lru_counter al procesar una petición ENTRANTE (httpd_sess.c), y
    // este WebSocket solo recibe datos del servidor. Cada conexión nueva con la
    // tabla llena cerraría justo la sesión de audio que sí está viva.
    httpd_sess_update_lru_counter(s_httpd, fd);
    return true;
}

// En ESP-IDF 6.1 el handler WebSocket NO se llama al conectar el cliente,
// solo cuando el cliente envía frames. Para streaming unidireccional (server→client)
// usamos httpd_get_client_list + httpd_ws_get_fd_info para encontrar clientes WS.
static void audio_stream_task(void *arg) {
    adpcm_state_t state = { .predictor = 0, .step_index = 0 };
    int sample_count = 0;
    int client_fds[AUDIO_STREAM_MAX_CLIENT_FDS];

    for (;;) {
        int8_t sample;
        if (xQueueReceive(audio_stream_q, &sample, pdMS_TO_TICKS(150)) != pdTRUE) {
            if (sample_count) s_a_starve++;   // bloque a medias descartado
            sample_count = 0;
            continue;
        }
        s_sample_buf[sample_count++] = sample;
        s_a_samples++;

        if (sample_count < ADPCM_SAMPLES_BLOCK) continue;
        sample_count = 0;

        encode_block(&state, s_sample_buf, s_adpcm_block);
        s_a_blocks++;

        // Enviar a todos los clientes WebSocket activos.
        if (s_httpd != NULL) {
            size_t n = sizeof(client_fds) / sizeof(client_fds[0]);
            uint32_t wsc = 0;
            if (httpd_get_client_list(s_httpd, &n, client_fds) == ESP_OK) {
                for (int i = 0; i < (int)n; i++) {
                    int fd = client_fds[i];
                    if (httpd_ws_get_fd_info(s_httpd, fd) != HTTPD_WS_CLIENT_WEBSOCKET)
                        continue;
                    wsc++;

                    httpd_ws_frame_t pkt = {
                        .final      = true,
                        .fragmented = false,
                        .type       = HTTPD_WS_TYPE_BINARY,
                        .payload    = s_adpcm_block,
                        .len        = ADPCM_BLOCK_BYTES,
                    };
                    // descarta el bloque si el cliente no drena
                    if (ws_try_send(fd, &pkt)) s_a_sent++;
                    else                       s_a_drop++;
                }
            }
            s_a_wsc = wsc;
        }

        // Encolar para el cliente WAV TCP si está activo (lossy).
        // s_wav_q_mutex serializes against wav_server_task/audio_stream_stop_wav_server
        // deleting the queue; timeout 0 keeps this from ever blocking the audio
        // pipeline — worst case this block is dropped, which the WAV stream
        // already tolerates.
        if (xSemaphoreTake(s_wav_q_mutex, 0) == pdTRUE) {
            if (s_wav_q) xQueueSendToBack(s_wav_q, s_adpcm_block, 0);
            xSemaphoreGive(s_wav_q_mutex);
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
        // Abortar en cuanto el cliente deja de drenar: index.html son ~157
        // chunks y con send_wait_timeout=1 cada chunk fallido bloquea la tarea
        // httpd 1 s ("httpd_sock_err: error in send : 11" una vez por segundo),
        // dejando el servidor entero — WS incluido — sin atender.
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
    if (has_cse)
        snprintf(info, sizeof(info), "=%s/%s%c%03d/%03d%s",
                 lat_str, lon_str, sym, cse, spd, comment);
    else
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
    int timeouts = 0;
    while (total < remaining) {
        ret = httpd_req_recv(req, body + total, (size_t)(remaining - total));
        if (ret <= 0) {
            // A client that keeps stalling could otherwise retry here forever,
            // tying up an httpd worker indefinitely.
            if (ret == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 5) continue;
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
        // Responder CLOSE y dejar que httpd cierre el socket. El eco solo se
        // intenta si el socket admite escritura: el cliente que cierra suele
        // tener ya el buffer TX lleno, y entonces el send se queda hasta agotar
        // send_wait_timeout para acabar en EAGAIN ("error in send : 11").
        int cfd = httpd_req_to_sockfd(req);
        if (cfd >= 0 && sock_writable(cfd)) {
            httpd_ws_frame_t close_f = {
                .final = true, .type = HTTPD_WS_TYPE_CLOSE,
                .payload = NULL, .len = 0,
            };
            httpd_ws_send_frame(req, &close_f);
        }
        if (cfd >= 0) ws_stall_clear(cfd);
        free(buf);
        return ESP_FAIL;  // provoca cierre de sesión por parte de httpd
    }

    free(buf);
    return ESP_OK;
}

// Llamado por httpd justo tras responder el 101 del upgrade. Sirve para separar
// en /api/rx/stats "el handshake no llega a ejecutarse" (hs no sube) de "la
// sesión se establece pero muere después" (hs sube y ws_clients sigue a 0).
static esp_err_t ws_post_handshake(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "WS handshake OK fd=%d", fd);
    if (fd >= 0) {
        ws_stall_clear(fd);
        // Sustituye el send() suelto de httpd por uno que coloca el buffer
        // entero; sin esto un envío parcial rompe el stream WebSocket. El
        // override es por sesión y httpd_sess_new() lo repone al reutilizar
        // el descriptor, así que no contamina sockets HTTP normales.
        httpd_sess_set_send_override(req->handle, fd, ws_send_all);
    }
    s_a_hs++;
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
    s_wav_srv_fd = srv;
    ESP_LOGI(TAG, "Servidor WAV escuchando en puerto %d", AUDIO_WAV_PORT);

    for (;;) {
        struct sockaddr_in client;
        socklen_t clen = sizeof(client);
        int fd = accept(srv, (struct sockaddr *)&client, &clen);
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        s_wav_client_fd = fd;
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
        QueueHandle_t q = xQueueCreate(8, ADPCM_BLOCK_BYTES);
        xSemaphoreTake(s_wav_q_mutex, portMAX_DELAY);
        s_wav_q = q;
        xSemaphoreGive(s_wav_q_mutex);

        uint8_t block[ADPCM_BLOCK_BYTES];
        while (1) {
            if (xQueueReceive(q, block, pdMS_TO_TICKS(500)) != pdTRUE) {
                // Timeout: comprobar si el socket sigue vivo
                if (send(fd, NULL, 0, MSG_NOSIGNAL) < 0) break;
                continue;
            }
            if (send(fd, block, ADPCM_BLOCK_BYTES, MSG_NOSIGNAL) < 0) break;
        }

        ESP_LOGI(TAG, "Cliente WAV desconectado");
        xSemaphoreTake(s_wav_q_mutex, portMAX_DELAY);
        s_wav_q = NULL;
        xSemaphoreGive(s_wav_q_mutex);
        vQueueDelete(q);
        s_wav_client_fd = -1;
        close(fd);
    }
}

// ─── GET /api/log?since=N ─────────────────────────────────────────────────────

static esp_err_t log_handler(httpd_req_t *req)
{
    uint32_t since = 0;
    char qs[32];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK) {
        char val[12];
        if (httpd_query_key_value(qs, "since", val, sizeof(val)) == ESP_OK)
            since = (uint32_t)strtoul(val, NULL, 10);
    }

    char *ptrs[LOG_CAPACITY];
    int   pcount   = 0;
    uint32_t last_seq = 0;

    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    last_seq = s_log_seq_next > 0 ? s_log_seq_next - 1 : 0;
    int cap   = s_log_max;
    int start = (s_log_head - s_log_count + cap) % cap;
    for (int i = 0; i < s_log_count; i++) {
        int idx = (start + i) % cap;
        if (s_log_ring[idx].seq > since && s_log_ring[idx].json) {
            char *p = strdup(s_log_ring[idx].json);
            if (p) ptrs[pcount++] = p;
        }
    }
    xSemaphoreGive(s_log_mutex);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char hdr[48];
    snprintf(hdr, sizeof(hdr), "{\"last_seq\":%lu,\"entries\":[",
             (unsigned long)last_seq);
    // Igual que index_handler: en cuanto un chunk falla se aborta. Son hasta 250
    // entradas y con send_wait_timeout=1 cada chunk fallido bloquearía la tarea
    // httpd un segundo más.
    esp_err_t err = httpd_resp_sendstr_chunk(req, hdr);
    for (int i = 0; i < pcount; i++) {
        if (err == ESP_OK && i > 0) err = httpd_resp_sendstr_chunk(req, ",");
        if (err == ESP_OK)          err = httpd_resp_sendstr_chunk(req, ptrs[i]);
        free(ptrs[i]);
    }
    if (err != ESP_OK) return ESP_FAIL;  // sin chunk final: httpd cierra el socket
    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ─── GET /api/rx/stats ────────────────────────────────────────────────────────

static esp_err_t rx_stats_handler(httpd_req_t *req)
{
    char buf[448];
    snprintf(buf, sizeof(buf),
        "{\"v1\":{\"frames_ok\":%lu,\"frames_err\":%lu,\"hdlc_flags\":%lu,\"fifo_ov\":%lu},"
        "\"v2\":{\"frames_ok\":%lu,\"frames_err\":%lu,\"hdlc_flags\":%lu,\"fifo_ov\":%lu},"
        "\"v2_enabled\":%s,\"v2_agc_peak\":%ld,\"v2_squelch_open\":%s,"
        "\"audio\":{\"samples\":%lu,\"blocks\":%lu,\"sent\":%lu,\"drop\":%lu,"
        "\"starve\":%lu,\"ws_clients\":%lu,\"hs\":%lu,\"q_waiting\":%u,\"heap\":%u}}",
        (unsigned long)rx_stats_v1.frames_crc_ok,
        (unsigned long)rx_stats_v1.frames_crc_err,
        (unsigned long)rx_stats_v1.hdlc_flags,
        (unsigned long)rx_stats_v1.fifo_overflows,
        (unsigned long)rx_stats_v2.frames_crc_ok,
        (unsigned long)rx_stats_v2.frames_crc_err,
        (unsigned long)rx_stats_v2.hdlc_flags,
        (unsigned long)rx_stats_v2.fifo_overflows,
        AFSK_modem_v2 ? "true" : "false",
        (long)afsk_v2_agc_peak,
        afsk_v2_squelch_open ? "true" : "false",
        (unsigned long)s_a_samples, (unsigned long)s_a_blocks,
        (unsigned long)s_a_sent,    (unsigned long)s_a_drop,
        (unsigned long)s_a_starve,  (unsigned long)s_a_wsc,
        (unsigned long)s_a_hs,
        (unsigned)(audio_stream_q ? uxQueueMessagesWaiting(audio_stream_q) : 0),
        (unsigned)esp_get_free_heap_size());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

// ─── Envío de texto a clientes WebSocket ─────────────────────────────────────

void audio_stream_ws_send_text(const char *text) {
    if (s_httpd == NULL || text == NULL) return;

    // Obtener número de secuencia
    uint32_t seq = 0;
    if (s_log_mutex) {
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
        seq = s_log_seq_next++;
        xSemaphoreGive(s_log_mutex);
    }

    // Construir copia con seq inyectado: {"seq":N,...campos originales...}
    size_t tlen   = strlen(text);
    char  *wrapped = (char *)malloc(tlen + 24);
    if (wrapped)
        snprintf(wrapped, tlen + 24, "{\"seq\":%lu,%s", (unsigned long)seq, text + 1);

    // Persist in ring buffer only discrete events; skip high-frequency telemetry.
    // El ring adopta `wrapped`; si NO entra (evento efímero, o sin mutex/malloc)
    // el dueño sigue siendo esta función y hay que liberarlo tras enviarlo.
    bool ring_owns = false;
    char *to_free = NULL;
    if (s_log_mutex && wrapped && !log_is_ephemeral(text)) {
        ring_owns = true;
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
        int cap = s_log_max; // snapshot under mutex
        to_free = s_log_ring[s_log_head].json;
        s_log_ring[s_log_head].seq  = seq;
        s_log_ring[s_log_head].json = wrapped;
        s_log_head = (s_log_head + 1) % cap;
        if (s_log_count < cap) s_log_count++;
        xSemaphoreGive(s_log_mutex);
    }
    free(to_free);

    // Enviar por WebSocket
    const char *send_str = wrapped ? wrapped : text;
    httpd_ws_frame_t pkt = {
        .final      = true,
        .fragmented = false,
        .type       = HTTPD_WS_TYPE_TEXT,
        .payload    = (uint8_t *)send_str,
        .len        = strlen(send_str),
    };
    int    client_fds[AUDIO_STREAM_MAX_CLIENT_FDS];
    size_t n = sizeof(client_fds) / sizeof(client_fds[0]);
    if (httpd_get_client_list(s_httpd, &n, client_fds) == ESP_OK) {
        // Igual que el audio: nunca esperar por un cliente que no drena. Los
        // eventos no efímeros ya están en el ring con su `seq`, y el cliente los
        // recupera al reconectar con GET /api/log?since=<last_seq>, así que
        // descartarlos aquí no pierde información.
        for (int i = 0; i < (int)n; i++) {
            if (httpd_ws_get_fd_info(s_httpd, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET)
                ws_try_send(client_fds[i], &pkt);
        }
    }

    if (!ring_owns) free(wrapped);
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

    static char buf[1024];
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

// ─── WAV server teardown ──────────────────────────────────────────────────────

void audio_stream_stop_wav_server(void)
{
    if (s_wav_task_hdl == NULL) return;

    // Close the listening socket — unblocks accept() with an error so the task
    // stops trying to serve new clients.
    int fd = s_wav_srv_fd;
    s_wav_srv_fd = -1;
    if (fd >= 0) close(fd);

    // Close the connected client's socket too, if any — without this, forcibly
    // deleting the task below (which may be blocked in send()/recv() on this fd,
    // or simply never reaches its own close(fd) at the bottom of the loop) leaks
    // the socket. lwIP has a small fixed socket pool, so repeated mode toggles
    // would eventually exhaust it.
    int cfd = s_wav_client_fd;
    s_wav_client_fd = -1;
    if (cfd >= 0) close(cfd);

    // Delete the task BEFORE touching s_wav_q: it may be blocked inside
    // xQueueReceive(s_wav_q, ...), and deleting a queue a task is still
    // waiting on is unsafe. vTaskDelete() removes the task from the queue's
    // wait list as part of teardown, so it's safe to free the queue after.
    vTaskDelete(s_wav_task_hdl);
    s_wav_task_hdl = NULL;

    // audio_stream_task may still be sending into s_wav_q concurrently — take
    // the mutex before freeing it out from under that read.
    xSemaphoreTake(s_wav_q_mutex, portMAX_DELAY);
    QueueHandle_t q = s_wav_q;
    s_wav_q = NULL;
    xSemaphoreGive(s_wav_q_mutex);
    if (q) vQueueDelete(q);

    ESP_LOGI(TAG, "WAV server stopped (repeater mode)");
}

// ─── Punto de entrada ─────────────────────────────────────────────────────────

void audio_stream_init(void) {
    // Read configurable log capacity (aprs_msg.max_stored, default 25)
    {
        cJSON *cfg = config_get();
        if (cfg) {
            const cJSON *am = cJSON_GetObjectItem(cfg, "aprs_msg");
            if (am) {
                const cJSON *ms = cJSON_GetObjectItem(am, "max_stored");
                if (cJSON_IsNumber(ms) && ms->valueint >= 1) {
                    int v = ms->valueint;
                    if (v > LOG_CAPACITY) v = LOG_CAPACITY;
                    s_log_max = v;
                }
            }
            config_free_json(cfg);
        }
        ESP_LOGI(TAG, "log: max_stored=%d (ephemeral events not persisted)", s_log_max);
    }

    audio_stream_q = xQueueCreate(AUDIO_QUEUE_LEN, sizeof(int8_t));

    // Antes de httpd_start(): en cuanto el servidor acepta clientes puede haber
    // envíos WS concurrentes.
    s_ws_tx_mutex = xSemaphoreCreateMutex();
    s_wav_q_mutex = xSemaphoreCreateMutex();

    // HTTP server (index.html + WebSocket)
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = AUDIO_HTTP_PORT;
    cfg.stack_size       = 8192;
    cfg.max_open_sockets   = AUDIO_STREAM_MAX_CLIENT_FDS; // 2 WS clients + captive-portal burst (3-5) + margin
    cfg.lru_purge_enable   = true; // evict oldest idle socket when limit reached
    cfg.max_uri_handlers   = 32;
    // Por defecto son 5 s: un cliente que no drena su socket bloqueaba la tarea
    // emisora 5 s por intento. 1 s acota el peor caso de las rutas que sí
    // esperan (texto WS, respuestas HTTP); el audio ni siquiera llega a esperar
    // gracias al chequeo sock_writable().
    cfg.send_wait_timeout  = 1;
    // Recolecta sockets medio abiertos (pestaña cerrada de golpe, móvil que se
    // aleja del AP): sin keep-alive se quedan ocupando slot indefinidamente y,
    // con la tabla llena, cada conexión nueva depende del purgado LRU.
    cfg.keep_alive_enable   = true;
    cfg.keep_alive_idle     = 5;   // s de inactividad antes del primer sondeo
    cfg.keep_alive_interval = 5;   // s entre sondeos
    cfg.keep_alive_count    = 3;   // sondeos sin respuesta → socket cerrado

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
        .ws_post_handshake_cb = ws_post_handshake,
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
    static const httpd_uri_t uri_log = {
        .uri     = "/api/log",
        .method  = HTTP_GET,
        .handler = log_handler,
    };
    static const httpd_uri_t uri_rx_stats = {
        .uri     = "/api/rx/stats",
        .method  = HTTP_GET,
        .handler = rx_stats_handler,
    };
    static const httpd_uri_t uri_reboot = {
        .uri     = "/api/reboot",
        .method  = HTTP_POST,
        .handler = reboot_handler,
    };
    static const httpd_uri_t uri_spiffs_upload = {
        .uri     = "/api/spiffs/upload",
        .method  = HTTP_POST,
        .handler = spiffs_upload_handler,
    };

    s_log_mutex = xSemaphoreCreateMutex();

    httpd_register_uri_handler(s_httpd, &uri_index);
    httpd_register_uri_handler(s_httpd, &uri_ws);
    httpd_register_uri_handler(s_httpd, &uri_aprs_send);
    httpd_register_uri_handler(s_httpd, &uri_me);
    httpd_register_uri_handler(s_httpd, &uri_beacon);
    httpd_register_uri_handler(s_httpd, &uri_cfg_get);
    httpd_register_uri_handler(s_httpd, &uri_cfg_post);
    httpd_register_uri_handler(s_httpd, &uri_morse_trigger);
    httpd_register_uri_handler(s_httpd, &uri_log);
    httpd_register_uri_handler(s_httpd, &uri_rx_stats);
    httpd_register_uri_handler(s_httpd, &uri_reboot);
    httpd_register_uri_handler(s_httpd, &uri_spiffs_upload);

    static const httpd_uri_t uri_rep_status = {
        .uri     = "/api/repeater/status",
        .method  = HTTP_GET,
        .handler = repeater_status_handler,
    };
    static const httpd_uri_t uri_rep_enable = {
        .uri     = "/api/repeater/enable",
        .method  = HTTP_POST,
        .handler = repeater_enable_handler,
    };
    httpd_register_uri_handler(s_httpd, &uri_rep_status);
    httpd_register_uri_handler(s_httpd, &uri_rep_enable);

    static const httpd_uri_t uri_sf_config = {
        .uri     = "/api/squelch/sf_config",
        .method  = HTTP_POST,
        .handler = squelch_sf_config_handler,
    };
    static const httpd_uri_t uri_sq_status = {
        .uri     = "/api/squelch/status",
        .method  = HTTP_GET,
        .handler = squelch_status_handler,
    };
    static const httpd_uri_t uri_sq_monitor = {
        .uri     = "/api/squelch/monitor",
        .method  = HTTP_POST,
        .handler = squelch_monitor_handler,
    };
    httpd_register_uri_handler(s_httpd, &uri_sf_config);
    httpd_register_uri_handler(s_httpd, &uri_sq_status);
    httpd_register_uri_handler(s_httpd, &uri_sq_monitor);

    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, captive_redirect_handler);

    // Tarea de encoding + dispatch
    xTaskCreate(audio_stream_task, "audio_stream", 4096, NULL, 3, NULL);
    // Servidor WAV TCP
    xTaskCreate(wav_server_task, "wav_server", 4096, NULL, 3, &s_wav_task_hdl);

    ESP_LOGI(TAG, "Streaming iniciado — http://ip/ (player) | http://ip:%d/audio (ffplay/VLC)",
             AUDIO_WAV_PORT);
}

// Objetos y boletines APRS. Sólo encola tramas con APRS_queue_beacon
// (no bloquea; el TX real lo hace receive_audio_task).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "aprs_objects.h"
#include "gps.h"
#include "LibAPRS.h"

static const char *TAG = "aprs_obj";

#define MAX_ENTRIES      32     // objetos+boletines de config
#define MAX_RT_OBJECTS   8      // objetos creados por la API (sólo RAM)
#define MIN_GAP_S        30     // mínimo entre paquetes periódicos
#define MIN_PERIOD_S     60
#define TASK_STACK       3072

typedef struct {
    bool     bulletin;
    bool     runtime;          // creado por la API (sobrevive a reload)
    bool     live;             // '*' (vivo); false -> '_' (killed)
    char     name[10];         // objeto: nombre; boletín: no usado
    char     tbl, code;
    double   lat, lon;
    char     text[68];         // objeto: comentario (<=43); boletín: texto (<=67)
    uint8_t  bln_id;
    uint32_t period_s;
    int64_t  due_us;           // próxima emisión
} entry_t;

static entry_t *s_ent;         // array dinámico
static int      s_n;
static bool     s_enabled;
static uint32_t s_default_period = 900;
static TaskHandle_t s_task;
static SemaphoreHandle_t s_mtx;
static StaticSemaphore_t s_mtx_buf;
static int64_t  s_next_tx_us;

static void lock(void)   { xSemaphoreTake(s_mtx, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mtx); }
static void ensure_mutex(void) {
    if (!s_mtx) s_mtx = xSemaphoreCreateMutexStatic(&s_mtx_buf);
}

// ---------- formato ----------

// Nombre válido: 1-9 caracteres ASCII imprimibles sin '*' '_' '|' '~' ni ';'
static bool name_ok(const char *n) {
    size_t l = n ? strlen(n) : 0;
    if (l < 1 || l > 9) return false;
    for (size_t i = 0; i < l; i++) {
        unsigned char c = (unsigned char)n[i];
        if (c < 0x20 || c > 0x7e || c == '*' || c == '_' || c == '|' || c == '~' || c == ';')
            return false;
    }
    return true;
}

// Texto sin caracteres de control ni '|' '~' (reservados en APRS)
static void clean_copy(char *dst, size_t cap, const char *src) {
    size_t j = 0;
    for (; src && *src && j + 1 < cap; src++) {
        unsigned char c = (unsigned char)*src;
        if (c < 0x20 || c > 0x7e || c == '|' || c == '~') continue;
        dst[j++] = (char)c;
    }
    dst[j] = '\0';
}

// ";NAME_____*DDHHMMz" + pos(8 lat + tabla + 9 lon + código) + comentario.
// aprs_format_position da "=" + lat8 + '/' + lon9 + símbolo: mismo orden que
// el objeto, así que se reutiliza cambiando la tabla (idx 9) y saltando el '='.
static void build_object(char *out, size_t cap, const entry_t *e, bool kill) {
    char pos[APRS_POS_BUF_LEN];
    aprs_format_position(pos, sizeof(pos), e->lat, e->lon, e->code ? e->code : 'r');
    pos[9] = e->tbl ? e->tbl : '/';
    char ts[8] = "111111z";
    time_t now = time(NULL);
    if (now > 1700000000) {     // reloj sincronizado (SNTP/GPS)
        struct tm tm;
        gmtime_r(&now, &tm);
        snprintf(ts, sizeof(ts), "%02d%02d%02dz", tm.tm_mday, tm.tm_hour, tm.tm_min);
    }
    snprintf(out, cap, ";%-9s%c%s%s%s", e->name, (e->live && !kill) ? '*' : '_', ts,
             pos + 1, e->text);
}

static void build_bulletin(char *out, size_t cap, int id, const char *text) {
    snprintf(out, cap, ":BLN%d     :%.67s", id, text);
}

// ---------- tarea ----------

static void obj_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        char info[128];
        bool have = false;
        int64_t now = esp_timer_get_time();
        lock();
        if (s_n == 0 || (!s_enabled && !s_ent)) {   // nada que emitir: terminar
            s_task = NULL;
            unlock();
            vTaskDelete(NULL);
        }
        if (now >= s_next_tx_us) {
            int best = -1;
            for (int i = 0; i < s_n; i++) {
                if (!s_ent[i].runtime && !s_enabled) continue;
                if (s_ent[i].due_us <= now && (best < 0 || s_ent[i].due_us < s_ent[best].due_us))
                    best = i;
            }
            if (best >= 0) {
                entry_t *e = &s_ent[best];
                if (e->bulletin) build_bulletin(info, sizeof(info), e->bln_id, e->text);
                else             build_object(info, sizeof(info), e, false);
                e->due_us = now + (int64_t)e->period_s * 1000000LL;
                s_next_tx_us = now + (int64_t)MIN_GAP_S * 1000000LL;
                have = true;
            }
        }
        unlock();
        if (have) {
            APRS_queue_beacon(info);
            ESP_LOGI(TAG, "tx: %s", info);
        }
    }
}

// Arranca la tarea si procede (llamar con el mutex tomado)
static void maybe_start_task(void) {
    if (s_task || s_n == 0) return;
    if (xTaskCreate(obj_task, "aprs_obj", TASK_STACK, NULL, 3, &s_task) != pdPASS) {
        s_task = NULL;
        ESP_LOGW(TAG, "no se pudo crear la tarea");
    }
}

// ---------- config ----------

static const char *jstr(cJSON *o, const char *k, const char *def) {
    cJSON *v = cJSON_GetObjectItem(o, k);
    return cJSON_IsString(v) && v->valuestring ? v->valuestring : def;
}

// Reconstruye las entradas de config; conserva las de runtime. Mutex tomado.
static void rebuild_locked(cJSON *cfg) {
    cJSON *o = cfg ? cJSON_GetObjectItem(cfg, "objects") : NULL;
    s_enabled = cJSON_IsTrue(cJSON_GetObjectItem(o, "enabled"));
    cJSON *pj = cJSON_GetObjectItem(o, "period_s");
    s_default_period = cJSON_IsNumber(pj) && pj->valueint >= MIN_PERIOD_S ? (uint32_t)pj->valueint : 900;

    // Contar para reservar justo lo necesario
    cJSON *items = s_enabled ? cJSON_GetObjectItem(o, "items") : NULL;
    cJSON *blns  = s_enabled ? cJSON_GetObjectItem(o, "bulletins") : NULL;
    int nrt = 0;
    for (int i = 0; i < s_n; i++) if (s_ent[i].runtime) nrt++;
    int ni = cJSON_IsArray(items) ? cJSON_GetArraySize(items) : 0;
    int nb = cJSON_IsArray(blns) ? cJSON_GetArraySize(blns) : 0;
    int cap = nrt + ni + nb;
    if (cap > MAX_ENTRIES + MAX_RT_OBJECTS) cap = MAX_ENTRIES + MAX_RT_OBJECTS;

    entry_t *na = NULL;
    if (cap > 0) {
        na = calloc((size_t)cap, sizeof(entry_t));
        if (!na) { ESP_LOGW(TAG, "sin memoria"); return; }
    }
    int n = 0;
    for (int i = 0; i < s_n && n < cap; i++)
        if (s_ent[i].runtime) na[n++] = s_ent[i];
    int64_t now = esp_timer_get_time();
    int slot = 0;   // reparto inicial: separa las primeras emisiones MIN_GAP_S entre sí
    cJSON *it;
    cJSON_ArrayForEach(it, items) {
        if (n >= cap) break;
        const char *nm = jstr(it, "name", "");
        cJSON *la = cJSON_GetObjectItem(it, "lat"), *lo = cJSON_GetObjectItem(it, "lon");
        if (!name_ok(nm) || !cJSON_IsNumber(la) || !cJSON_IsNumber(lo)) continue;
        entry_t *e = &na[n++];
        strlcpy(e->name, nm, sizeof(e->name));
        e->lat = la->valuedouble;  e->lon = lo->valuedouble;
        e->tbl  = jstr(it, "symbol_table", "/")[0];
        e->code = jstr(it, "symbol_code", "r")[0];
        char c[68];
        clean_copy(c, sizeof(c), jstr(it, "comment", ""));
        c[43] = '\0';
        strlcpy(e->text, c, sizeof(e->text));
        cJSON *lv = cJSON_GetObjectItem(it, "live");
        e->live = lv ? !cJSON_IsFalse(lv) : true;
        e->period_s = s_default_period;
        e->due_us = now + (int64_t)(10 + slot++ * MIN_GAP_S) * 1000000LL;
    }
    cJSON_ArrayForEach(it, blns) {
        if (n >= cap) break;
        cJSON *id = cJSON_GetObjectItem(it, "id"), *pp = cJSON_GetObjectItem(it, "period_s");
        const char *tx = jstr(it, "text", "");
        if (!cJSON_IsNumber(id) || id->valueint < 0 || id->valueint > 9 || !*tx) continue;
        entry_t *e = &na[n++];
        e->bulletin = true;
        e->bln_id = (uint8_t)id->valueint;
        char c[68];
        clean_copy(c, sizeof(c), tx);
        strlcpy(e->text, c, sizeof(e->text));
        e->period_s = cJSON_IsNumber(pp) && pp->valueint >= MIN_PERIOD_S ? (uint32_t)pp->valueint : 1800;
        e->due_us = now + (int64_t)(10 + slot++ * MIN_GAP_S) * 1000000LL;
    }
    free(s_ent);
    s_ent = na;
    s_n = n;
}

void aprs_objects_init(cJSON *cfg) {
    ensure_mutex();
    lock();
    rebuild_locked(cfg);
    maybe_start_task();
    unlock();
}

void aprs_objects_reload(cJSON *cfg) {
    aprs_objects_init(cfg);   // la tarea se retira sola si ya no hay nada
}

// ---------- HTTP ----------

static cJSON *read_json(httpd_req_t *req) {
    size_t len = req->content_len;
    if (len == 0 || len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad body size");
        return NULL;
    }
    char *buf = malloc(len + 1);
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory"); return NULL; }
    size_t tot = 0;
    int to = 0;
    while (tot < len) {
        int r = httpd_req_recv(req, buf + tot, len - tot);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT && ++to <= 5) continue;
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv error");
            return NULL;
        }
        tot += (size_t)r;
    }
    buf[tot] = '\0';
    cJSON *j = cJSON_Parse(buf);
    free(buf);
    if (!j) httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return j;
}

static esp_err_t object_handler(httpd_req_t *req) {
    cJSON *j = read_json(req);
    if (!j) return ESP_OK;
    const char *nm = jstr(j, "name", "");
    cJSON *la = cJSON_GetObjectItem(j, "lat"), *lo = cJSON_GetObjectItem(j, "lon");
    cJSON *kj = cJSON_GetObjectItem(j, "kill");
    cJSON *pj = cJSON_GetObjectItem(j, "persist");
    bool kill = cJSON_IsTrue(kj), persist = cJSON_IsTrue(pj);
    if (!name_ok(nm) || !cJSON_IsNumber(la) || !cJSON_IsNumber(lo) ||
        fabs(la->valuedouble) > 90.0 || fabs(lo->valuedouble) > 180.0) {
        cJSON_Delete(j);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid name/lat/lon");
        return ESP_OK;
    }
    entry_t e = {0};
    strlcpy(e.name, nm, sizeof(e.name));
    e.lat = la->valuedouble;  e.lon = lo->valuedouble;
    e.tbl  = jstr(j, "symbol_table", "/")[0];
    e.code = jstr(j, "symbol_code", "r")[0];
    if (e.tbl < 0x21 || e.tbl > 0x7e) e.tbl = '/';
    if (e.code < 0x21 || e.code > 0x7e) e.code = 'r';
    char c[68];
    clean_copy(c, sizeof(c), jstr(j, "comment", ""));
    c[43] = '\0';
    strlcpy(e.text, c, sizeof(e.text));
    e.live = true;
    e.runtime = true;
    e.period_s = s_default_period ? s_default_period : 900;
    cJSON_Delete(j);

    char info[128];
    build_object(info, sizeof(info), &e, kill);
    bool ok = true;
    ensure_mutex();
    lock();
    int idx = -1, nrt = 0;
    for (int i = 0; i < s_n; i++) {
        if (s_ent[i].runtime) nrt++;
        if (s_ent[i].runtime && !s_ent[i].bulletin && !strcmp(s_ent[i].name, e.name)) idx = i;
    }
    if (kill) {
        // Retira el objeto de la lista de RAM (ya no se repite)
        if (idx >= 0) { s_ent[idx] = s_ent[--s_n]; }
    } else if (persist) {
        if (idx >= 0) {
            e.due_us = esp_timer_get_time() + (int64_t)e.period_s * 1000000LL;
            s_ent[idx] = e;
        } else if (nrt >= MAX_RT_OBJECTS) {
            ok = false;
        } else {
            entry_t *na = realloc(s_ent, (size_t)(s_n + 1) * sizeof(entry_t));
            if (na) {
                s_ent = na;
                e.due_us = esp_timer_get_time() + (int64_t)e.period_s * 1000000LL;
                s_ent[s_n++] = e;
                maybe_start_task();
            } else ok = false;
        }
    }
    unlock();
    if (!ok) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Object list full or no memory");
        return ESP_OK;
    }
    APRS_queue_beacon(info);
    ESP_LOGI(TAG, "object tx: %s", info);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t bulletin_handler(httpd_req_t *req) {
    cJSON *j = read_json(req);
    if (!j) return ESP_OK;
    cJSON *id = cJSON_GetObjectItem(j, "id");
    const char *tx = jstr(j, "text", "");
    if (!cJSON_IsNumber(id) || id->valueint < 0 || id->valueint > 9 || !*tx) {
        cJSON_Delete(j);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid id/text");
        return ESP_OK;
    }
    char c[68], info[96];
    clean_copy(c, sizeof(c), tx);
    build_bulletin(info, sizeof(info), id->valueint, c);
    cJSON_Delete(j);
    APRS_queue_beacon(info);
    ESP_LOGI(TAG, "bulletin tx: %s", info);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

void aprs_objects_register(httpd_handle_t srv) {
    static const httpd_uri_t u1 = { .uri = "/api/aprs/object",   .method = HTTP_POST, .handler = object_handler };
    static const httpd_uri_t u2 = { .uri = "/api/aprs/bulletin", .method = HTTP_POST, .handler = bulletin_handler };
    httpd_register_uri_handler(srv, &u1);
    httpd_register_uri_handler(srv, &u2);
}

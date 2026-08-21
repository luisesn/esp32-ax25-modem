#include "remote_cmd.h"
#include "sstv.h"
#include "morse.h"
#include "gps.h"
#include "LibAPRS-esp32-i2s/src/LibAPRS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <dirent.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "remote_cmd";
static bool s_enabled = false;

void remote_cmd_init(const cJSON *cfg) {
    if (!cfg) return;
    cJSON *rc = cJSON_GetObjectItem(cfg, "remote_cmd");
    if (!rc) return;
    cJSON *en = cJSON_GetObjectItem(rc, "enabled");
    if (cJSON_IsBool(en)) s_enabled = cJSON_IsTrue(en);
    if (s_enabled)
        ESP_LOGI(TAG, "enabled");
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static SstvMode parse_sstv_mode(const char *s) {
    if (strcasecmp(s, "m1")  == 0) return SSTV_MODE_MARTIN_M1;
    if (strcasecmp(s, "m2")  == 0) return SSTV_MODE_MARTIN_M2;
    if (strcasecmp(s, "s1")  == 0) return SSTV_MODE_SCOTTIE_S1;
    if (strcasecmp(s, "r72") == 0) return SSTV_MODE_ROBOT_72;
    return SSTV_MODE_ROBOT_36;
}

// ---------------------------------------------------------------------------
// tx,sstv,<filename>,<mode>
// ---------------------------------------------------------------------------

static void cmd_tx_sstv(const char *src_call, int src_ssid,
                        const char *fname, const char *mode_str) {
    if (!fname || fname[0] == '\0') {
        APRS_queue_msg(src_call, src_ssid, "ERR:tx,sstv needs filename");
        return;
    }
    // Same check as the HTTP upload/list handlers (sstv.c) — SPIFFS is a flat
    // namespace today so this is defense in depth, but a name coming in over
    // RF should never be trusted more than one coming over HTTP.
    if (strchr(fname, '/') || strchr(fname, '\\') || strstr(fname, "..")) {
        APRS_queue_msg(src_call, src_ssid, "ERR:invalid filename");
        return;
    }
    if (!s_sstv_queue) {
        APRS_queue_msg(src_call, src_ssid, "ERR:SSTV not init");
        return;
    }
    SstvRequest req = {0};
    strncpy(req.name, fname, sizeof(req.name) - 1);
    req.fmt    = SSTV_FMT_JPEG;
    req.mode   = parse_sstv_mode(mode_str ? mode_str : "r36");
    req.is_test = false;

    if (xQueueSend(s_sstv_queue, &req, 0) != pdTRUE) {
        APRS_queue_msg(src_call, src_ssid, "ERR:SSTV busy");
    } else {
        char reply[48];
        snprintf(reply, sizeof(reply), "SSTV TX:%s", fname);
        APRS_queue_msg(src_call, src_ssid, reply);
    }
}

// ---------------------------------------------------------------------------
// tx,morse,beacon
// ---------------------------------------------------------------------------

static void cmd_tx_morse(const char *src_call, int src_ssid) {
    (void)src_call; (void)src_ssid;
    morse_trigger_now();
    ESP_LOGI(TAG, "morse beacon triggered by remote cmd");
}

// ---------------------------------------------------------------------------
// tx,aprs,position
// ---------------------------------------------------------------------------

static void cmd_tx_aprs_position(const char *src_call, int src_ssid) {
    gps_lock_pos();
    bool   valid = g_gps_pos.valid;
    double lat   = g_gps_pos.lat;
    double lon   = g_gps_pos.lon;
    gps_unlock_pos();

    if (!valid) {
        APRS_queue_msg(src_call, src_ssid, "No GPS fix");
        return;
    }

    double abs_lat = fabs(lat);
    int    dlat    = (int)abs_lat;
    double mlat    = (abs_lat - dlat) * 60.0;
    double abs_lon = fabs(lon);
    int    dlon    = (int)abs_lon;
    double mlon    = (abs_lon - dlon) * 60.0;

    // APRS compressed position: !DDMM.mmN/DDDMM.mmW>
    char info[32];
    snprintf(info, sizeof(info), "!%02d%05.2f%c/%03d%05.2f%c>",
             dlat, mlat, lat >= 0.0 ? 'N' : 'S',
             dlon, mlon, lon >= 0.0 ? 'E' : 'W');
    APRS_queue_beacon(info);
    APRS_queue_msg(src_call, src_ssid, "Position sent");
}

// ---------------------------------------------------------------------------
// rx,sstv,list — one APRS message per file, 5 s apart
// ---------------------------------------------------------------------------

#define LIST_MAX_FILES 20

typedef struct {
    char src_call[10];
    int  src_ssid;
    char files[LIST_MAX_FILES][48];
    int  count;
} ListArgs;

// Runs the SPIFFS directory scan itself, not just the reply pacing —
// opendir()/readdir() are blocking flash I/O and used to run synchronously
// inside cmd_rx_sstv_list(), called directly from remote_cmd_handle() on
// aprs_poll_task. If the scan was slow, it delayed draining rxFifo for the
// next already-decoded frame, risking overflow/loss of HDLC bytes arriving
// from receive_audio_task in the meantime. Moving the scan into this
// low-priority one-shot task keeps aprs_poll_task's RX path non-blocking.
static void list_scan_and_reply_task(void *arg) {
    ListArgs *a = (ListArgs *)arg;

    DIR *dir = opendir(SSTV_DIR);
    if (!dir) {
        APRS_queue_msg(a->src_call, a->src_ssid, "SSTV:no files");
        free(a);
        vTaskDelete(NULL);
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && a->count < LIST_MAX_FILES) {
        const char *n = ent->d_name;
        size_t nl = strlen(n);
        // Accept .jpg / .jpeg (case-insensitive)
        bool is_jpg = (nl > 4 && strcasecmp(n + nl - 4, ".jpg") == 0) ||
                      (nl > 5 && strcasecmp(n + nl - 5, ".jpeg") == 0);
        if (!is_jpg) continue;
        strncpy(a->files[a->count], n, sizeof(a->files[0]) - 1);
        a->count++;
    }
    closedir(dir);

    if (a->count == 0) {
        APRS_queue_msg(a->src_call, a->src_ssid, "SSTV:no files");
        free(a);
        vTaskDelete(NULL);
    }

    for (int i = 0; i < a->count; i++) {
        APRS_queue_msg(a->src_call, a->src_ssid, a->files[i]);
        if (i < a->count - 1)
            vTaskDelay(pdMS_TO_TICKS(5000));
    }
    free(a);
    vTaskDelete(NULL);
}

static void cmd_rx_sstv_list(const char *src_call, int src_ssid) {
    ListArgs *a = calloc(1, sizeof(ListArgs));
    if (!a) return;
    strncpy(a->src_call, src_call, sizeof(a->src_call) - 1);
    a->src_ssid = src_ssid;

    if (xTaskCreate(list_scan_and_reply_task, "sstv_list_rply", 3072, a, 2, NULL) != pdPASS) {
        APRS_queue_msg(src_call, src_ssid, "ERR:task alloc");
        free(a);
    }
}

// ---------------------------------------------------------------------------
// Main dispatcher
// ---------------------------------------------------------------------------

// Split text on commas, up to max_parts. Returns number of parts found.
// Modifies buf in-place.
static int split_csv(char *buf, char **parts, int max_parts) {
    int n = 0;
    char *p = buf;
    while (n < max_parts) {
        parts[n++] = p;
        char *c = strchr(p, ',');
        if (!c) break;
        *c = '\0';
        p = c + 1;
    }
    return n;
}

void remote_cmd_handle(const char *src_call, int src_ssid, const char *text) {
    if (!s_enabled || !text || text[0] == '\0') return;

    char buf[68];
    strncpy(buf, text, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *parts[5] = {0};
    int n = split_csv(buf, parts, 5);
    if (n < 2) return;

    // Lowercase verb and noun for matching; leave rest (filenames) unchanged
    for (char *c = parts[0]; *c; c++) if (*c >= 'A' && *c <= 'Z') *c += 32;
    for (char *c = parts[1]; *c; c++) if (*c >= 'A' && *c <= 'Z') *c += 32;

    ESP_LOGI(TAG, "from %s-%d: %s,%s%s%s",
             src_call, src_ssid, parts[0], parts[1],
             n > 2 ? "," : "", n > 2 ? parts[2] : "");

    if (strcmp(parts[0], "tx") == 0) {
        if (strcmp(parts[1], "sstv") == 0) {
            // tx,sstv,<filename>,<mode>
            cmd_tx_sstv(src_call, src_ssid,
                        n > 2 ? parts[2] : NULL,
                        n > 3 ? parts[3] : "r36");
        } else if (strcmp(parts[1], "morse") == 0) {
            // tx,morse,beacon
            if (n < 3 || strcasecmp(parts[2], "beacon") == 0)
                cmd_tx_morse(src_call, src_ssid);
        } else if (strcmp(parts[1], "aprs") == 0) {
            // tx,aprs,position
            if (n > 2 && strcasecmp(parts[2], "position") == 0)
                cmd_tx_aprs_position(src_call, src_ssid);
        }
    } else if (strcmp(parts[0], "rx") == 0) {
        if (strcmp(parts[1], "sstv") == 0) {
            // rx,sstv,list
            if (n > 2 && strcasecmp(parts[2], "list") == 0)
                cmd_rx_sstv_list(src_call, src_ssid);
        }
    }
}

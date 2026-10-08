#include "gps.h"
#include "config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "LibAPRS-esp32-i2s/src/LibAPRS.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define GPS_UART       UART_NUM_2
#define GPS_RX_BUF_SZ   512
#define GPS_LINE_MAX    96

static const char *TAG = "gps";

GpsPosition g_gps_pos = {0};
static SemaphoreHandle_t s_pos_mutex;

static uint8_t s_prev_quality = 0xFF; // sentinel "unknown"
static uint8_t s_prev_sats    = 0xFF;
static bool    s_first_fix    = true;

static volatile bool s_beacon_enabled  = false;
static volatile int  s_beacon_period_s = 600;

void gps_lock_pos(void)   { xSemaphoreTake(s_pos_mutex, portMAX_DELAY); }
void gps_unlock_pos(void) { xSemaphoreGive(s_pos_mutex); }

// ---------------------------------------------------------------------------
// NMEA helpers
// ---------------------------------------------------------------------------

// Convert NMEA DDDMM.MMMM + hemisphere to decimal degrees.
static double nmea_to_dd(const char *val, char hemi) {
    if (!val || val[0] == '\0') return 0.0;
    double raw = atof(val);
    int    deg = (int)(raw / 100);
    double dd  = deg + (raw - deg * 100.0) / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -dd : dd;
}

// XOR checksum over bytes between '$' and '*'.
static bool checksum_ok(const char *s) {
    if (*s == '$') s++;
    uint8_t csum = 0;
    while (*s && *s != '*') csum ^= (uint8_t)*s++;
    if (*s != '*') return false;
    return csum == (uint8_t)strtol(s + 1, NULL, 16);
}

// Split buf on commas in-place; fill fields[]; return field count.
static int csv_split(char *buf, char **fields, int max) {
    int n = 0;
    char *p = buf;
    while (n < max) {
        fields[n++] = p;
        char *c = strchr(p, ',');
        if (!c) break;
        *c = '\0';
        p  = c + 1;
    }
    return n;
}

// ---------------------------------------------------------------------------
// Sentence parsers (fields[] starts at sentence-type, e.g. fields[0]="GPRMC")
// ---------------------------------------------------------------------------

// Pone el reloj del sistema con la hora UTC del GPS (HHMMSS.ss, DDMMYY), como
// mucho una vez cada 10 min y sólo si SNTP no lo ha sincronizado ya.
static void gps_sync_system_time(const char *hhmmss, const char *ddmmyy) {
    static int64_t s_last_sync_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (s_last_sync_us && now_us - s_last_sync_us < 600LL * 1000000LL) return;
    if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) return;
    if (strlen(hhmmss) < 6 || strlen(ddmmyy) < 6) return;
    int v[6];
    const char *src[6] = { hhmmss, hhmmss + 2, hhmmss + 4, ddmmyy, ddmmyy + 2, ddmmyy + 4 };
    for (int i = 0; i < 6; i++) {
        if (src[i][0] < '0' || src[i][0] > '9' || src[i][1] < '0' || src[i][1] > '9') return;
        v[i] = (src[i][0] - '0') * 10 + (src[i][1] - '0');
    }
    struct tm t = { .tm_hour = v[0], .tm_min = v[1], .tm_sec = v[2],
                    .tm_mday = v[3], .tm_mon = v[4] - 1, .tm_year = 100 + v[5] };
    if (t.tm_year < 124 || t.tm_mon < 0 || t.tm_mon > 11 || t.tm_mday < 1 || t.tm_mday > 31) return;
    setenv("TZ", "UTC0", 1); tzset();
    time_t secs = mktime(&t);
    struct timeval tv = { .tv_sec = secs, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_last_sync_us = now_us;
    ESP_LOGI("gps", "Reloj del sistema ajustado desde GPS: %04d-%02d-%02d %02d:%02d:%02d UTC",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
}

static void parse_rmc(char **f, int n) {
    // $GxRMC,HHMMSS.ss,status,lat,NS,lon,EW,speed,course,DDMMYY,...*hh
    // f[0]=type, f[1]=time, f[2]=status, f[3]=lat, f[4]=NS, f[5]=lon,
    // f[6]=EW, f[7]=speed, f[8]=course, f[9]=date
    if (n < 10) return;
    bool active = (f[2][0] == 'A');

    gps_lock_pos();
    g_gps_pos.valid = active;
    if (active) {
        g_gps_pos.lat         = nmea_to_dd(f[3], f[4][0]);
        g_gps_pos.lon         = nmea_to_dd(f[5], f[6][0]);
        g_gps_pos.speed_knots = (float)atof(f[7]);
        g_gps_pos.course_deg  = (float)atof(f[8]);
        snprintf(g_gps_pos.time_utc, sizeof(g_gps_pos.time_utc), "%.6s", f[1]);
        snprintf(g_gps_pos.date_utc, sizeof(g_gps_pos.date_utc), "%.6s", f[9]);
    }
    gps_unlock_pos();

    if (active) gps_sync_system_time(f[1], f[9]);

    if (active && s_first_fix) {
        s_first_fix = false;
        // printf("GPS primer fix: %s UTC  lat=%.5f  lon=%.5f\n", g_gps_pos.time_utc, g_gps_pos.lat, g_gps_pos.lon);
    }
    // else if (active) { printf("GPS: %s UTC  lat=%.5f  lon=%.5f\n", g_gps_pos.time_utc, g_gps_pos.lat, g_gps_pos.lon); }
    if (!active && !s_first_fix) {
        s_first_fix = true;
        // printf("GPS: fix perdido\n");
    }
}

static void parse_gga(char **f, int n) {
    // $GxGGA,time,lat,NS,lon,EW,quality,sats,hdop,alt,...*hh
    // f[0]=type, f[1]=time, f[2]=lat, f[3]=NS, f[4]=lon, f[5]=EW,
    // f[6]=quality, f[7]=sats
    if (n < 8) return;
    uint8_t quality = (uint8_t)atoi(f[6]);
    uint8_t sats    = (uint8_t)atoi(f[7]);
    bool log_change = (quality != s_prev_quality || sats != s_prev_sats);

    gps_lock_pos();
    g_gps_pos.fix_quality = quality;
    g_gps_pos.satellites  = sats;
    // Use GGA for position if RMC hasn't given a fix yet
    if (quality > 0 && !g_gps_pos.valid && n >= 6) {
        g_gps_pos.lat   = nmea_to_dd(f[2], f[3][0]);
        g_gps_pos.lon   = nmea_to_dd(f[4], f[5][0]);
        g_gps_pos.valid = true;
    }
    gps_unlock_pos();

    if (log_change) {
        // const char *fix_str = quality == 0 ? "sin fix" : sats < 4 ? "2D" : "3D";
        // printf("GPS sats:%d  fix:%s  (quality=%d)\n", sats, fix_str, quality);
        s_prev_quality = quality;
        s_prev_sats    = sats;
    }
}

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------

static void gps_task(void *arg) {
    char    line[GPS_LINE_MAX];
    int     pos = 0;
    uint8_t rx[64];
    bool    s_uart_active = false;

    for (;;) {
        int len = uart_read_bytes(GPS_UART, rx, sizeof(rx), pdMS_TO_TICKS(200));
        for (int i = 0; i < len; i++) {
            char c = (char)rx[i];
            if (c == '\n' || pos >= GPS_LINE_MAX - 1) {
                line[pos] = '\0';
                // Strip trailing CR
                if (pos > 0 && line[pos - 1] == '\r') line[--pos] = '\0';

                if (pos > 6 && line[0] == '$' && checksum_ok(line)) {
                    if (!s_uart_active) {
                        s_uart_active = true;
                        // printf("GPS: primera trama NMEA válida recibida\n");
                    }
                    ESP_LOGD(TAG, "%s", line);
                    gps_lock_pos();
                    g_gps_pos.data_received = true;
                    gps_unlock_pos();
                    char  buf[GPS_LINE_MAX];
                    char *f[16];
                    // Work on a copy; skip '$', strip checksum suffix
                    strncpy(buf, line + 1, sizeof(buf) - 1);
                    char *star = strchr(buf, '*');
                    if (star) *star = '\0';
                    int n = csv_split(buf, f, 16);
                    if (n > 0) {
                        const char *id = f[0];
                        if ((strncmp(id, "GPRMC", 5) == 0 || strncmp(id, "GNRMC", 5) == 0) && n >= 10)
                            parse_rmc(f, n); // f[0]=type, f[1]=time, f[2]=status...
                        else if ((strncmp(id, "GPGGA", 5) == 0 || strncmp(id, "GNGGA", 5) == 0) && n >= 8)
                            parse_gga(f, n); // f[0]=type, f[1]=time, f[2]=lat...
                    }
                }
                pos = 0;
            } else if (c != '\r') {
                line[pos++] = c;
            }
        }
        vTaskDelay(1);
    }
}

// Periodic APRS position beacon. Only queues a frame (APRS_queue_beacon is
// non-blocking); the actual TX happens in receive_audio_task.
static void gps_beacon_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)s_beacon_period_s * 1000u));

        gps_lock_pos();
        bool   valid  = g_gps_pos.valid;
        double lat    = g_gps_pos.lat;
        double lon    = g_gps_pos.lon;
        float  course = g_gps_pos.course_deg;
        float  speed  = g_gps_pos.speed_knots;
        gps_unlock_pos();

        if (!valid) {
            ESP_LOGW(TAG, "beacon: no GPS fix, skipping");
            continue;
        }

        double abs_lat = fabs(lat);
        int    dlat    = (int)abs_lat;
        double mlat    = (abs_lat - dlat) * 60.0;
        double abs_lon = fabs(lon);
        int    dlon    = (int)abs_lon;
        double mlon    = (abs_lon - dlon) * 60.0;

        char info[48];
        snprintf(info, sizeof(info), "!%02d%05.2f%c/%03d%05.2f%c>%03.0f/%03.0f",
                 dlat, mlat, lat >= 0.0 ? 'N' : 'S',
                 dlon, mlon, lon >= 0.0 ? 'E' : 'W',
                 (double)course, (double)speed);
        APRS_queue_beacon(info);
        ESP_LOGI(TAG, "beacon sent: %s", info);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void gps_init(cJSON *cfg) {
    bool enabled = true;
    int  baud    = 9600;

    if (cfg) {
        cJSON *g = cJSON_GetObjectItem(cfg, "gps");
        if (g) {
            cJSON *en = cJSON_GetObjectItem(g, "enabled");
            if (cJSON_IsBool(en)) enabled = cJSON_IsTrue(en);
            cJSON *bd = cJSON_GetObjectItem(g, "baud");
            if (cJSON_IsNumber(bd)) baud = bd->valueint;
            cJSON *beacon = cJSON_GetObjectItem(g, "use_for_beacon");
            if (cJSON_IsBool(beacon)) s_beacon_enabled = cJSON_IsTrue(beacon);
            cJSON *period = cJSON_GetObjectItem(g, "beacon_period_s");
            if (cJSON_IsNumber(period) && period->valueint > 0)
                s_beacon_period_s = period->valueint;
        }
    }

    if (!enabled) {
        ESP_LOGI(TAG, "disabled in config");
        return;
    }

    s_pos_mutex = xSemaphoreCreateMutex();

    uart_config_t ucfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, GPS_RX_BUF_SZ, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &ucfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART,
                                 GPIO_GPS_TX, GPIO_GPS_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(gps_task, "gps_task", 2048, NULL, 4, NULL);
    if (s_beacon_enabled) {
        xTaskCreate(gps_beacon_task, "gps_beacon", 3072, NULL, 3, NULL);
        ESP_LOGI(TAG, "beacon enabled, period=%ds", s_beacon_period_s);
    }
    ESP_LOGI(TAG, "UART2 RX=GPIO%d TX=GPIO%d @ %d baud",
             GPIO_GPS_RX, GPIO_GPS_TX, baud);
}

#include "display.h"
#include "config.h"
#include "gps.h"
#include "transport_wifi.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"   // audio_peak
#include "LibAPRS-esp32-i2s/src/LibAPRS.h" // APRS_getCallsign
#include "rx_stats.h"
#include "repeater.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "display";

// ---------------------------------------------------------------------------
// SSD1306 constants
// ---------------------------------------------------------------------------
#define DISPLAY_W       128
#define DISPLAY_H       64
#define DISPLAY_PAGES   (DISPLAY_H / 8)         // 8
#define FB_SIZE         (DISPLAY_W * DISPLAY_PAGES) // 1024

// I2C SSD1306 control bytes
#define SSD1306_CMD_BYTE  0x00  // following bytes = commands
#define SSD1306_DATA_BYTE 0x40  // following bytes = GDDRAM data

// Initialization command sequence
static const uint8_t ssd1306_init_seq[] = {
    0xAE,       // display off
    0xD5, 0x80, // clock divide ratio / oscillator frequency
    0xA8, 0x3F, // multiplex ratio (64 rows)
    0xD3, 0x00, // display offset
    0x40,       // start line = 0
    0x8D, 0x14, // charge pump on
    0x20, 0x00, // horizontal addressing mode
    0xA1,       // column 127 → SEG0 (mirror)
    0xC8,       // scan from COM[N-1] to COM0 (flip)
    0xDA, 0x12, // COM pins hardware config
    0x81, 0xCF, // contrast
    0xD9, 0xF1, // pre-charge period
    0xDB, 0x40, // VCOMH level
    0xA4,       // output follows RAM
    0xA6,       // normal (non-inverted) display
    0xAF,       // display on
};

// ---------------------------------------------------------------------------
// 5×8 font (ASCII 0x20–0x7F, public domain)
// Each entry: 5 column bytes, bit0 = top row, bit7 = bottom row.
// ---------------------------------------------------------------------------
static const uint8_t font5x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x08,0x2A,0x1C,0x2A,0x08}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x10,0x08,0x08,0x10,0x08}, // '~'
    {0x00,0x00,0x00,0x00,0x00}, // DEL / placeholder
};

// ---------------------------------------------------------------------------
// I2C + frame buffer state
// ---------------------------------------------------------------------------
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_fb[FB_SIZE];

// Reusable TX buffer for frame-buffer flush: 1 control byte + 1024 data bytes.
static uint8_t s_tx_buf[1 + FB_SIZE];

// ---------------------------------------------------------------------------
// Low-level SSD1306 I2C helpers
// ---------------------------------------------------------------------------

static esp_err_t ssd1306_send_cmds(const uint8_t *cmds, size_t len) {
    uint8_t buf[32];
    buf[0] = SSD1306_CMD_BYTE;
    for (size_t i = 0; i < len; i += sizeof(buf) - 1) {
        size_t chunk = len - i;
        if (chunk > sizeof(buf) - 1) chunk = sizeof(buf) - 1;
        memcpy(buf + 1, cmds + i, chunk);
        esp_err_t err = i2c_master_transmit(s_dev, buf, chunk + 1, 50);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void ssd1306_cmd1(uint8_t cmd) {
    uint8_t buf[2] = {SSD1306_CMD_BYTE, cmd};
    i2c_master_transmit(s_dev, buf, 2, 50);
}

static void ssd1306_cmd2(uint8_t cmd, uint8_t arg) {
    uint8_t buf[3] = {SSD1306_CMD_BYTE, cmd, arg};
    i2c_master_transmit(s_dev, buf, 3, 50);
}

// Flush the entire frame buffer to GDDRAM in one I2C transaction.
static void ssd1306_flush(void) {
    // Set column address 0–127 and page address 0–7
    ssd1306_cmd2(0x21, 0); ssd1306_cmd1(127);   // col start, end
    ssd1306_cmd2(0x22, 0); ssd1306_cmd1(DISPLAY_PAGES - 1); // page start, end

    s_tx_buf[0] = SSD1306_DATA_BYTE;
    memcpy(s_tx_buf + 1, s_fb, FB_SIZE);
    i2c_master_transmit(s_dev, s_tx_buf, sizeof(s_tx_buf), 200);
}

// ---------------------------------------------------------------------------
// Frame buffer helpers
// ---------------------------------------------------------------------------

static void fb_clear(void) { memset(s_fb, 0, FB_SIZE); }

static void fb_hline(int page, uint8_t pattern) {
    if (page < 0 || page >= DISPLAY_PAGES) return;
    memset(s_fb + page * DISPLAY_W, pattern, DISPLAY_W);
}

static void fb_char(int page, int col, char c) {
    if (page < 0 || page >= DISPLAY_PAGES) return;
    if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7F) c = ' ';
    const uint8_t *g = font5x8[(unsigned char)c - 0x20];
    uint8_t *row = s_fb + page * DISPLAY_W;
    for (int i = 0; i < 5; i++) {
        if (col + i < DISPLAY_W) row[col + i] = g[i];
    }
    if (col + 5 < DISPLAY_W) row[col + 5] = 0x00; // char gap
}

// Write string at (page, starting col); returns final column.
static int fb_text(int page, int col, const char *s) {
    while (*s && col < DISPLAY_W) {
        fb_char(page, col, *s++);
        col += 6;
    }
    return col;
}

// ---------------------------------------------------------------------------
// Display task
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Screen renderers
// ---------------------------------------------------------------------------

static void screen_wifi_connecting(char *line) {
    WifiConnStatus ws = g_wifi_status; // snapshot (no mutex needed for display)

    fb_text(0, 0, "WiFi conectando...");
    fb_hline(1, 0x08);

    // SSID (truncate to 21 chars)
    snprintf(line, 32, "%.21s", ws.ssid);
    fb_text(2, 0, line);

    // Network index when multiple are configured
    if (ws.net_count > 1) {
        snprintf(line, 32, "Red %d/%d", ws.net_index + 1, ws.net_count);
        fb_text(4, 0, line);
    }

    fb_hline(5, 0x08);

    // Countdown: time remaining for this attempt
    int64_t elapsed_s = (esp_timer_get_time() - ws.attempt_start_us) / 1000000LL;
    int remaining = ws.timeout_s - (int)elapsed_s;
    if (remaining < 0) remaining = 0;
    snprintf(line, 32, "Espera: %ds", remaining);
    fb_text(6, 0, line);

    // Progress bar for remaining time
    int bar_px = (remaining * 100) / (ws.timeout_s > 0 ? ws.timeout_s : 1);
    if (bar_px > 100) bar_px = 100;
    for (int x = 18; x < 18 + bar_px && x < 118; x++)
        s_fb[7 * DISPLAY_W + x] = 0x18; // thin bar
}

static void screen_hotspot(char *line) {
    fb_text(0, 0, "Modo hotspot");
    fb_hline(1, 0x08);
    snprintf(line, 32, "%.21s", g_wifi_status.ssid);
    fb_text(2, 0, line);
    fb_text(4, 0, "192.168.4.1");
    fb_text(6, 0, "Sin WiFi externa");
}

static void screen_normal(char *line) {
    // --- Page 0: callsign ---
    char call[8] = "?"; int ssid_n = 0;
    APRS_getCallsign(call, &ssid_n);
    char callssid[12];
    snprintf(callssid, sizeof(callssid), "%s-%d", call, ssid_n);
    fb_text(0, 0, callssid);

    // --- Page 1: IP address ---
    char ip_str[16] = "no wifi";
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta) {
        esp_netif_ip_info_t ip_info = {0};
        if (esp_netif_get_ip_info(sta, &ip_info) == ESP_OK && ip_info.ip.addr)
            esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
    }
    fb_text(1, 0, ip_str);

    // --- Pages 2-3: GPS ---
    GpsPosition pos;
    gps_lock_pos();
    pos = g_gps_pos;
    gps_unlock_pos();

    if (pos.valid) {
        snprintf(line, 32, "%9.5f%c %9.5f%c",
                 pos.lat >= 0 ? pos.lat : -pos.lat, pos.lat >= 0 ? 'N' : 'S',
                 pos.lon >= 0 ? pos.lon : -pos.lon, pos.lon >= 0 ? 'E' : 'W');
        fb_text(2, 0, line);
        const char *t = pos.time_utc;
        snprintf(line, 32, "Sats:%02d %c%c:%c%c:%c%cZ",
                 pos.satellites, t[0],t[1], t[2],t[3], t[4],t[5]);
        fb_text(3, 0, line);
    } else if (pos.data_received) {
        fb_text(2, 0, "GPS: sin fix");
    } else {
        fb_text(2, 0, "GPS: sin datos");
    }

    // --- Page 4: APRS decoder stats ---
    snprintf(line, 32, "APRS:%u/%u %u/%u",
             (unsigned)(rx_stats_v1.frames_crc_ok % 1000),
             (unsigned)(rx_stats_v2.frames_crc_ok % 1000),
             (unsigned)(rx_stats_v1.hdlc_flags    % 1000),
             (unsigned)(rx_stats_v2.hdlc_flags    % 1000));
    fb_text(4, 0, line);

    fb_hline(5, 0x08);

    // --- Page 6: RX audio bar (80 px) + numeric peak ---
    int peak = (int)(int8_t)audio_peak;
    if (peak < 0) peak = 0;
    bool loud = (peak > AUDIO_LEVEL_TOO_HIGH);

    fb_text(6, 0, loud ? "RX!" : "RX:");
    int bar_px = (peak * 80) / 127;
    if (bar_px > 80) bar_px = 80;
    uint8_t bar_fill = loud ? 0x7E : 0x3C;
    for (int x = 18; x < 18 + bar_px && x < 98; x++)
        s_fb[6 * DISPLAY_W + x] = bar_fill;

    // Threshold marker scaled to 80-px bar
    int thr_x = 18 + (AUDIO_LEVEL_TOO_HIGH * 80) / 127;
    if (thr_x < 98)
        s_fb[6 * DISPLAY_W + thr_x] = 0x7F;

    // Numeric peak value at column 100
    snprintf(line, 32, "%d", peak);
    fb_text(6, 100, line);

    if (loud)
        fb_text(7, 80, "LOUD");

    // --- Page 7 left: repeater state + level (when enabled) ---
    if (repeater_is_enabled()) {
        const char *rep_label = NULL;
        switch (repeater_get_state()) {
            case REPEATER_STATE_RECORDING: rep_label = "REC "; break;
            case REPEATER_STATE_TAIL:      rep_label = "TAIL"; break;
            case REPEATER_STATE_PENDING:   rep_label = "WAIT"; break;
            case REPEATER_STATE_TX:        rep_label = "RPT!"; break;
            default:                       rep_label = "RPT:"; break;
        }
        // Show state label + numeric level + threshold marker
        uint32_t lv  = repeater_get_level();
        uint32_t thr = repeater_get_threshold();
        snprintf(line, 32, "%s%2u/%u", rep_label, (unsigned)(lv > 99 ? 99 : lv), (unsigned)thr);
        fb_text(7, 0, line);
    }
}

// ---------------------------------------------------------------------------
// Display task
// ---------------------------------------------------------------------------

static void display_task(void *arg) {
    char line[32];

    for (;;) {
        fb_clear();

        switch (g_wifi_status.state) {
            case WIFI_STATUS_CONNECTING:
                screen_wifi_connecting(line);
                break;
            case WIFI_STATUS_HOTSPOT:
                screen_hotspot(line);
                break;
            default:
                screen_normal(line);
                break;
        }

        ssd1306_flush();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void display_init(cJSON *cfg) {
    bool    enabled  = true;
    uint8_t i2c_addr = 0x3C;

    if (cfg) {
        cJSON *d = cJSON_GetObjectItem(cfg, "display");
        if (d) {
            cJSON *en = cJSON_GetObjectItem(d, "enabled");
            if (cJSON_IsBool(en)) enabled = cJSON_IsTrue(en);
            cJSON *addr = cJSON_GetObjectItem(d, "i2c_addr");
            if (cJSON_IsNumber(addr)) i2c_addr = (uint8_t)addr->valueint;
        }
    }

    if (!enabled) {
        ESP_LOGI(TAG, "disabled in config");
        return;
    }

    // --- I2C master bus ---
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_NUM_0,
        .sda_io_num          = GPIO_I2C_SDA,
        .scl_io_num          = GPIO_I2C_SCL,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return;
    }

    // --- SSD1306 device ---
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_addr,
        .scl_speed_hz    = 400000,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add_device failed: %s", esp_err_to_name(err));
        return;
    }

    // Probe before sending init sequence (graceful if display not wired)
    err = i2c_master_probe(s_bus, i2c_addr, 50);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SSD1306 not found at 0x%02X — display task not started", i2c_addr);
        return;
    }

    // --- SSD1306 init ---
    err = ssd1306_send_cmds(ssd1306_init_seq, sizeof(ssd1306_init_seq));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 init failed: %s", esp_err_to_name(err));
        return;
    }

    fb_clear();
    ssd1306_flush();

    xTaskCreate(display_task, "display_task", 2048, NULL, 2, NULL);
    ESP_LOGI(TAG, "SSD1306 0x%02X on SDA=GPIO%d SCL=GPIO%d",
             i2c_addr, GPIO_I2C_SDA, GPIO_I2C_SCL);
}

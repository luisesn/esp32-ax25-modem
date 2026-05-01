#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_system.h"

#include "config.h"
#include "aux_config.h"
#include"aux_file_management.h"

#include "driver/gpio.h"

#include "LibAPRS-esp32-i2s/src/LibAPRS.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"
#include "audio_stream.h"
#include "ax25ip.h"
#include "digipeater.h"
#include "morse.h"

#include "device.h"

#if TNC_MODE == TNC_MODE_KISS
#include "kiss.h"
#include "transport.h"
#  if KISS_TRANSPORT == KISS_TRANSPORT_WIFI
#    include "transport_wifi.h"
#  endif
#endif

// ---------------------------------------------------------------------------
// AX.25 mini-parser → JSON (KISS TNC mode only)
// ---------------------------------------------------------------------------
// Formato trama raw (sin CRC): [DST:7][SRC:7][RPT:7×N][CTRL:1][PID:1][INFO:]
// Cada dirección: bytes[0-5] = char ASCII << 1, byte[6]: SSID=(b>>1)&0xF, H-bit=b&1.
// AX25_CTRL_UI y AX25_PID_NOLAYER3 vienen de AX25.h vía LibAPRS.h.

#if TNC_MODE == TNC_MODE_KISS

#define APRS_JSON_BUF 512

static const uint8_t *ax25_decode_addr(const uint8_t *p,
                                        char *call_out, bool *is_last) {
    char tmp[7];
    for (int i = 0; i < 6; i++) {
        char c = (char)(p[i] >> 1);
        tmp[i] = (c >= 0x20 && c <= 0x7E) ? c : '?';
    }
    int end = 5;
    while (end >= 0 && tmp[end] == ' ') end--;
    tmp[end + 1] = '\0';
    int ssid = (p[6] >> 1) & 0x0F;
    *is_last  = (p[6] & 0x01) != 0;
    if (ssid > 0) snprintf(call_out, 10, "%s-%d", tmp, ssid);
    else          snprintf(call_out, 10, "%s",     tmp);
    return p + 7;
}

static bool ax25_frame_to_json(const uint8_t *buf, size_t len,
                                char *out, size_t out_size) {
    if (len < 16) return false;
    const uint8_t *p = buf, *end = buf + len;
    bool is_last;
    char dst[10], src[10];

    p = ax25_decode_addr(p, dst, &is_last);
    p = ax25_decode_addr(p, src, &is_last);

    // Repeater path
    static char path[96]; int plen = 0; path[0] = '\0';
    while (!is_last && (p + 7) <= end) {
        char rpt[10];
        p = ax25_decode_addr(p, rpt, &is_last);
        if (plen > 0 && plen < (int)sizeof(path) - 1) path[plen++] = ',';
        int rem = (int)sizeof(path) - plen - 1;
        if (rem > 0) plen += snprintf(path + plen, rem, "%s", rpt);
    }
    path[sizeof(path) - 1] = '\0';

    if ((p + 2) > end) return false;
    if (*p++ != AX25_CTRL_UI || *p++ != AX25_PID_NOLAYER3) return false;

    // Sanitize info field: printable ASCII only, escape " and \.
    static char info[300], escaped[350]; int ilen = 0;
    while (p < end && ilen < (int)sizeof(info) - 1) {
        uint8_t c = *p++;
        if (c == '\r' || c == '\n') continue;
        info[ilen++] = (c >= 0x20 && c <= 0x7E) ? (char)c : '.';
    }
    info[ilen] = '\0';

    int ei = 0;
    for (int i = 0; i < ilen && ei < (int)sizeof(escaped) - 3; i++) {
        if (info[i] == '"' || info[i] == '\\') escaped[ei++] = '\\';
        escaped[ei++] = info[i];
    }
    escaped[ei] = '\0';

    int n = snprintf(out, out_size,
        "{\"src\":\"%s\",\"dst\":\"%s\",\"path\":\"%s\",\"info\":\"%s\"}",
        src, dst, path, escaped);
    return (n > 0 && (size_t)n < out_size);
}

#endif // TNC_MODE == TNC_MODE_KISS

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH  false

#define AUDIO_LEVEL_TOO_LOW    8
#define AUDIO_LEVEL_TOO_HIGH   115
#define AUDIO_ALERT_BLINK_MS   100

// ---------------------------------------------------------------------------
// Monitor de nivel de audio (250 ms)
// ---------------------------------------------------------------------------

static void audio_level_task(void *arg)
{
    (void)arg;
    char bar[13];
    int log_div = 0;
    bool alarm_led_state = false;
    bool alarm_active = false;
    for (;;) {
        vTaskDelay(AUDIO_ALERT_BLINK_MS / portTICK_PERIOD_MS);

        int8_t peak = audio_peak;
        audio_peak  = 0;

        bool out_of_range = (peak < AUDIO_LEVEL_TOO_LOW) || (peak > AUDIO_LEVEL_TOO_HIGH);
        if (out_of_range) {
            alarm_active = true;
#if GPIO_LED_RX >= 0
            alarm_led_state = !alarm_led_state;
            gpio_set_level(GPIO_LED_RX, alarm_led_state ? 1 : 0);
#endif
        } else if (alarm_active) {
            alarm_active = false;
            alarm_led_state = false;
#if GPIO_LED_RX >= 0
            gpio_set_level(GPIO_LED_RX, 0);
#endif
        }

        int level = (int)peak * 10 / 127;
        if (level > 10) level = 10;

        bar[0] = '|';
        for (int i = 1; i <= 10; i++)
            bar[i] = (i <= level) ? '#' : '-';
        bar[11] = '|';
        bar[12] = '\0';

        if (++log_div >= 3) {
            log_div = 0;
            printf("Audio: %s %3d%s\n", bar, (int)peak,
                   out_of_range ? " [ALERT]" : "");
        }
    }
}

// ---------------------------------------------------------------------------
// Modo KISS TNC
// ---------------------------------------------------------------------------

#if TNC_MODE == TNC_MODE_KISS

// Auto-ACK: si el info field es ":NUESTRO_CALL :texto{NNN}", devuelve
// ":SRC   :ackNNN" por radio. No ACKea si la trama ya es un ACK.
static void try_auto_ack(const uint8_t *buf, size_t len)
{
    if (len < 16) return;
    const uint8_t *p = buf, *end = buf + len;
    bool is_last;
    char dst_call[10], src_call[10];

    p = ax25_decode_addr(p, dst_call, &is_last);
    p = ax25_decode_addr(p, src_call, &is_last);
    while (!is_last && (p + 7) <= end) {
        char tmp[10];
        p = ax25_decode_addr(p, tmp, &is_last);
    }
    if ((p + 2) > end) return;
    if (*p++ != AX25_CTRL_UI || *p++ != AX25_PID_NOLAYER3) return;

    // p → info field
    size_t info_len = (size_t)(end - p);
    if (info_len < 12) return;                // ":XXXXXXXXX:m" mínimo
    if (p[0] != ':' || p[10] != ':') return;

    // Comparar los 9 chars del addressee con nuestro call+ssid (sin distinguir mayúsculas)
    char my_call[7]; int my_ssid;
    APRS_getCallsign(my_call, &my_ssid);
    // Build 9-char padded addressee string without snprintf (avoids -Werror=format-truncation)
    char our_addr[12];
    memset(our_addr, ' ', 9);
    our_addr[9] = '\0';
    int clen = (int)strlen(my_call); if (clen > 6) clen = 6;
    int pos = 0;
    for (int i = 0; i < clen; i++) our_addr[pos++] = my_call[i];
    if (my_ssid > 0) {
        our_addr[pos++] = '-';
        if (my_ssid >= 10) our_addr[pos++] = (char)('0' + my_ssid / 10);
        our_addr[pos++] = (char)('0' + my_ssid % 10);
    }

    for (int i = 0; i < 9; i++) {
        char a = (char)p[1 + i]; if (a >= 'a' && a <= 'z') a -= 32;
        char b = our_addr[i];    if (b >= 'a' && b <= 'z') b -= 32;
        if (a != b) return;
    }

    // Es para nosotros. Si el texto ya empieza por "ack" → no ACKear.
    const uint8_t *txt = p + 11;
    size_t tlen = info_len - 11;
    if (tlen >= 3 &&
        (txt[0]=='a'||txt[0]=='A') && (txt[1]=='c'||txt[1]=='C') &&
        (txt[2]=='k'||txt[2]=='K')) return;

    // Buscar "{NNN}" al final
    const uint8_t *brace = NULL;
    for (const uint8_t *q = txt + tlen - 1; q >= txt; q--)
        if (*q == '{') { brace = q; break; }
    if (!brace) return;

    char msg_id[6]; size_t id_len = 0;
    const uint8_t *q = brace + 1;
    while (q < end && id_len < 5 && *q >= '0' && *q <= '9')
        msg_id[id_len++] = (char)*q++;
    msg_id[id_len] = '\0';
    if (id_len == 0) return;

    // Separar src_call en indicativo y SSID
    char src_base[7]; int src_ssid_num = 0;
    char *dash = strchr(src_call, '-');
    if (dash) {
        int blen = (int)(dash - src_call); if (blen > 6) blen = 6;
        strncpy(src_base, src_call, (size_t)blen); src_base[blen] = '\0';
        src_ssid_num = atoi(dash + 1);
    } else {
        strncpy(src_base, src_call, 6); src_base[6] = '\0';
    }

    printf("AUTO-ACK → %s-%d ack%s\n", src_base, src_ssid_num, msg_id);
    APRS_queue_ack(src_base, src_ssid_num, msg_id);

    // Notificar al frontend via WebSocket
    static char ack_json[96];
    snprintf(ack_json, sizeof(ack_json),
             "{\"type\":\"ack_sent\",\"to\":\"%s\",\"ssid\":%d,\"id\":\"%s\"}",
             src_base, src_ssid_num, msg_id);
    audio_stream_ws_send_text(ack_json);
}

// Trama AX.25 recibida por radio → codificar en KISS → enviar al host.
static void on_ax25_raw_frame(const uint8_t *buf, size_t len) {
    kiss_send_frame(buf, len);
    try_auto_ack(buf, len);
    digi_process_frame(buf, len);
    ax25ip_rx_frame(buf, len);

    static char s_aprs_json[APRS_JSON_BUF];
    if (ax25_frame_to_json(buf, len, s_aprs_json, sizeof(s_aprs_json)))
        audio_stream_ws_send_text(s_aprs_json);
}

// Trama KISS completa recibida del host → encolar para TX en receive_audio_task.
// No llamar APRS_send_raw_frame directamente desde server_task: adc_continuous_stop
// requiere ser invocado por la misma tarea que llamó adc_continuous_start.
static void on_kiss_frame(const uint8_t *buf, size_t len) {
    afsk_queue_tx_frame(buf, len);
}

// ---------------------------------------------------------------------------
// Modo APRS clásico (imprime paquetes por consola)
// ---------------------------------------------------------------------------

#elif TNC_MODE == TNC_MODE_APRS

static bool      gotPacket = false;
static AX25Msg   incomingPacket;
static uint8_t  *packetData;

void aprs_msg_callback(struct AX25Msg *msg) {
    if (gotPacket) return;
    gotPacket = true;
    memcpy(&incomingPacket, msg, sizeof(AX25Msg));
    if ((int)esp_get_free_heap_size() > (int)msg->len) {
        packetData = (uint8_t *)malloc(msg->len);
        memcpy(packetData, msg->info, msg->len);
        incomingPacket.info = packetData;
    } else {
        gotPacket = false;
    }
}

static void processPacket(void *arg) {
    (void)arg;
    while (1) {
        if (gotPacket) {
            gotPacket = false;
            printf("SRC: %s-%d DST: %s-%d",
                   incomingPacket.src.call, incomingPacket.src.ssid,
                   incomingPacket.dst.call, incomingPacket.dst.ssid);
            for (int i = 0; i < incomingPacket.rpt_count; i++)
                printf(" via %s-%d", incomingPacket.rpt_list[i].call,
                       incomingPacket.rpt_list[i].ssid);
            printf(" data(%u): ", (unsigned)incomingPacket.len);
            for (size_t i = 0; i < incomingPacket.len; i++) {
                uint8_t c = incomingPacket.info[i];
                if (c >= 0x20 && c < 0x7F) putchar(c);
                else printf("\\x%02X", c);
            }
            printf("\r\n");
            free(packetData);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

#endif  // TNC_MODE

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

// Hook instalado en receive_audio_task: cada muestra decimada (9600 Hz, int8)
// se encola de forma no bloqueante hacia audio_stream_task.
static void audio_sample_hook(int8_t sample) {
    if (audio_stream_q)
        xQueueSendToBack(audio_stream_q, &sample, 0);
}

void app_main(void)
{
    file_management_init();
    // Cargamos aquí la configuración por primera vez
    config_load();
    /*
    // test ptt
    for (int i = 0; i < 5; i++) {
        PTT_Press();
        vTaskDelay(2500 / portTICK_PERIOD_MS);
        PTT_Release();
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }*/

#if TNC_MODE == TNC_MODE_KISS

#  if KISS_TRANSPORT == KISS_TRANSPORT_WIFI
    transport_init(&transport_wifi_ops);
    ax25ip_init(config_get());
#  elif KISS_TRANSPORT == KISS_TRANSPORT_UART
    // transport_init(&transport_uart_ops);  // pendiente implementar
#  elif KISS_TRANSPORT == KISS_TRANSPORT_BT
    // transport_init(&transport_bt_ops);    // pendiente implementar
#  endif

    kiss_init(on_kiss_frame);

    APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
    AFSK_set_leds(GPIO_LED_TX, GPIO_LED_RX);

    {
        const char *cs = "NOCALL";
        int cs_ssid = 0;
        cJSON *cfg = config_get();
        if (cfg) {
            cJSON *aprs_obj = cJSON_GetObjectItem(cfg, "aprs");
            if (aprs_obj) {
                cJSON *cs_item = cJSON_GetObjectItem(aprs_obj, "callsign");
                if (cJSON_IsString(cs_item) && cs_item->valuestring[0] != '\0')
                    cs = cs_item->valuestring;
                cJSON *ssid_item = cJSON_GetObjectItem(aprs_obj, "ssid");
                if (cJSON_IsNumber(ssid_item))
                    cs_ssid = (int)ssid_item->valueint;
            }
        }
        APRS_setCallsign((char *)cs, cs_ssid);
    }

    afsk_set_tx_fn(APRS_send_raw_frame);
    APRS_set_raw_hook(on_ax25_raw_frame);

    digi_init(config_get());
    morse_init(config_get());

    // Streaming de audio: instala el hook y arranca servidor HTTP + WebSocket.
    // Debe llamarse después de transport_init (WiFi ya conectado).
    audio_stream_init();
    afsk_set_audio_hook(audio_sample_hook);

#elif TNC_MODE == TNC_MODE_APRS

    APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
    AFSK_set_leds(GPIO_LED_TX, GPIO_LED_RX);

    const char *callsign = "NO0CALL";
    int ssid = 1;
    cJSON *cfg = config_get();
    if (cfg) {
        cJSON *aprs_obj = cJSON_GetObjectItem(cfg, "aprs");
        if (aprs_obj) {
            cJSON *cs = cJSON_GetObjectItem(aprs_obj, "callsign");
            if (cJSON_IsString(cs) && cs->valuestring[0] != '\0')
                callsign = cs->valuestring;
        }
        cJSON *ip_obj = cJSON_GetObjectItem(cfg, "ip");
        if (ip_obj) {
            cJSON *s = cJSON_GetObjectItem(ip_obj, "ssid");
            if (cJSON_IsNumber(s)) ssid = (int)s->valueint;
        }
    }
    APRS_setCallsign(callsign, ssid);
    APRS_set_msg_hook(aprs_msg_callback);
    xTaskCreate(processPacket, "processPacket", 2048, NULL, 5, NULL);

#endif  // TNC_MODE

    xTaskCreate(audio_level_task, "audio_lvl", 2048, NULL, 3, NULL);
}

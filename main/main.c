#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "aux_config.h"
#include "ptt.h"
#include "LibAPRS-esp32-i2s/src/LibAPRS.h"
#include "LibAPRS-esp32-i2s/src/AFSK.h"

#if TNC_MODE == TNC_MODE_KISS
#include "kiss.h"
#include "transport.h"
#  if KISS_TRANSPORT == KISS_TRANSPORT_WIFI
#    include "transport_wifi.h"
#  endif
#endif

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH  false

// ---------------------------------------------------------------------------
// Monitor de nivel de audio (250 ms)
// ---------------------------------------------------------------------------

static void audio_level_task(void *arg)
{
    (void)arg;
    char bar[13];
    for (;;) {
        vTaskDelay(250 / portTICK_PERIOD_MS);

        int8_t peak = audio_peak;
        audio_peak  = 0;

        int level = (int)peak * 10 / 127;
        if (level > 10) level = 10;

        bar[0] = '|';
        for (int i = 1; i <= 10; i++)
            bar[i] = (i <= level) ? '#' : '-';
        bar[11] = '|';
        bar[12] = '\0';

        printf("Audio: %s %3d\n", bar, (int)peak);
    }
}

// ---------------------------------------------------------------------------
// Modo KISS TNC
// ---------------------------------------------------------------------------

#if TNC_MODE == TNC_MODE_KISS

// Trama AX.25 recibida por radio → codificar en KISS → enviar al host.
static void on_ax25_raw_frame(const uint8_t *buf, size_t len) {
    kiss_send_frame(buf, len);
}

// Trama KISS completa recibida del host → transmitir por radio.
static void on_kiss_frame(const uint8_t *buf, size_t len) {
    APRS_send_raw_frame(buf, len);
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
    if (freeMemory() > (int)msg->len) {
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

void app_main(void)
{
    PTT_Init();
    config_load();

#if TNC_MODE == TNC_MODE_KISS

#  if KISS_TRANSPORT == KISS_TRANSPORT_WIFI
    transport_init(&transport_wifi_ops);
#  elif KISS_TRANSPORT == KISS_TRANSPORT_UART
    // transport_init(&transport_uart_ops);  // pendiente implementar
#  elif KISS_TRANSPORT == KISS_TRANSPORT_BT
    // transport_init(&transport_bt_ops);    // pendiente implementar
#  endif

    kiss_init(on_kiss_frame);

    APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
    APRS_set_raw_hook(on_ax25_raw_frame);

#elif TNC_MODE == TNC_MODE_APRS

    APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
    APRS_setCallsign("NO0CALL", 1);
    APRS_set_msg_hook(aprs_msg_callback);
    xTaskCreate(processPacket, "processPacket", 2048, NULL, 5, NULL);

#endif  // TNC_MODE

    xTaskCreate(audio_level_task, "audio_lvl", 2048, NULL, 3, NULL);
}

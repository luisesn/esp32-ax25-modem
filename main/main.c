#include <stdio.h>
#include "./LibAPRS-esp32-i2s/src/LibAPRS.h"
#include "./LibAPRS-esp32-i2s/src/AFSK.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ptt.h"


#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH false

bool gotPacket = false;
AX25Msg incomingPacket;
uint8_t *packetData;
void aprs_msg_callback(struct AX25Msg *msg) {
  // If we already have a packet waiting to be
  // processed, we must drop the new one.
  if (!gotPacket) {
    // Set flag to indicate we got a packet
    gotPacket = true;

    // The memory referenced as *msg is volatile
    // and we need to copy all the data to a
    // local variable for later processing.
    memcpy(&incomingPacket, msg, sizeof(AX25Msg));

    // We need to allocate a new buffer for the
    // data payload of the packet. First we check
    // if there is enough free RAM.
    if (freeMemory() > msg->len) {
      packetData = (uint8_t*)malloc(msg->len);
      memcpy(packetData, msg->info, msg->len);
      incomingPacket.info = packetData;
    } else {
      // We did not have enough free RAM to receive
      // this packet, so we drop it.
      gotPacket = false;
    }
  }
}

// Here's a function to process incoming packets
// Remember to call this function often, so you
// won't miss any packets due to one already
// waiting to be processed
void processPacket(void *pvParameters) {
    while (1) {
        if (gotPacket) {
            gotPacket = false;
            
            printf("Received APRS packet. SRC: %s-%d. DST: %s-%d",
                   incomingPacket.src.call, incomingPacket.src.ssid,
                   incomingPacket.dst.call, incomingPacket.dst.ssid);

            for (int i = 0; i < incomingPacket.rpt_count; i++) {
                printf(" via %s-%d", incomingPacket.rpt_list[i].call,
                       incomingPacket.rpt_list[i].ssid);
            }

            printf(". Data (%u bytes): ", (unsigned)incomingPacket.len);
            for (int i = 0; i < incomingPacket.len; i++) {
                uint8_t c = incomingPacket.info[i];
                if (c >= 0x20 && c < 0x7F) {
                    putchar(c);
                } else {
                    printf("\\x%02X", c);
                }
            }
            printf("\r\n");

            free(packetData);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


static void audio_level_task(void *arg)
{
    (void)arg;
    char bar[13];
    for (;;) {
        vTaskDelay(250 / portTICK_PERIOD_MS);

        int8_t peak = audio_peak;
        audio_peak = 0;

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

void app_main(void)
{
    PTT_Init();
    // Initialise APRS library - This starts the modem
    APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
    APRS_setCallsign("NO0CALL", 1);
    
    // APRS_setDestination("APZMDM", 0);
    // APRS_setPath1("WIDE1", 1);
    // APRS_setPreamble(350);
    // APRS_setTail(50);
    // APRS_useAlternateSymbolTable(false);
    // APRS_setSymbol('n');
    
    // We can print out all the settings
    //APRS_printSettings();

    // Create a task to process incoming packets
    xTaskCreate(processPacket, "processPacket", 2048, NULL, 5, NULL);
    xTaskCreate(audio_level_task, "audio_lvl", 2048, NULL, 3, NULL);
}
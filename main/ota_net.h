#pragma once

// OTA firmware update via Wi-Fi using the espota.py protocol (port 3232 UDP+TCP).
// Compatible with: idf.py -p <ESP32_IP> app-flash
// Must be called after transport_init() (WiFi must be up).
void ota_net_init(void);

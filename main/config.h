// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
#define GPIO_PTT_OUT          GPIO_NUM_26
#define GPIO_LED_RX           GPIO_NUM_33  // RX activity — green LED (blinks on each decoded packet)
#define GPIO_LED_TX          -1            // TX activity indicator (disabled)
#define GPIO_LED_WARN         GPIO_NUM_23  // Volume warning — red LED (on when audio out of range)
#define CONFIG_PTT_ON_DELAY_MS  10
#define CONFIG_PTT_OFF_DELAY_MS 10

// Audio level thresholds (int8_t, 0–127).  Used by audio_level_task and display.
#define AUDIO_LEVEL_TOO_LOW    8
#define AUDIO_LEVEL_TOO_HIGH   115

// I2C bus (SSD1306 LCD and any future I2C peripherals)
#define GPIO_I2C_SDA          GPIO_NUM_13
#define GPIO_I2C_SCL          GPIO_NUM_19  // GPIO23 has a board LED

// GPS UART2 — GPIO16 and GPIO4 are free GPIOs on the Lolin32 Lite (not SD pins)
#define GPIO_GPS_RX           GPIO_NUM_16
#define GPIO_GPS_TX           GPIO_NUM_4   // optional, for sending config to GPS module

// ---------------------------------------------------------------------------
// Modo de operación
// ---------------------------------------------------------------------------
#define TNC_MODE_APRS  1   // Imprime paquetes APRS por consola
#define TNC_MODE_KISS  2   // KISS TNC puro (compatible con tncattach / direwolf)
#define TNC_MODE       TNC_MODE_KISS

// ---------------------------------------------------------------------------
// Transporte (solo relevante en TNC_MODE_KISS)
// ---------------------------------------------------------------------------
#define KISS_TRANSPORT_WIFI  1   // WiFi TCP (implementado)
#define KISS_TRANSPORT_UART  2   // UART serie (futuro)
#define KISS_TRANSPORT_BT    3   // Bluetooth SPP (futuro)
#define KISS_TRANSPORT       KISS_TRANSPORT_WIFI

// ---------------------------------------------------------------------------
// WiFi (cuando KISS_TRANSPORT == KISS_TRANSPORT_WIFI)
// Para producción usar menuconfig (CONFIG_EXAMPLE_WIFI_SSID / _PASS) en su
// lugar, para no almacenar credenciales en el código fuente.
// ---------------------------------------------------------------------------
#define KISS_TCP_PORT 8001

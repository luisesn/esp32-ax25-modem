// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
#define GPIO_PTT_OUT          GPIO_NUM_26
#define CONFIG_PTT_ON_DELAY_MS  100
#define CONFIG_PTT_OFF_DELAY_MS 100

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
#define WIFI_SSID     "TU_SSID_AQUI"
#define WIFI_PASSWORD "TU_PASSWORD_AQUI"
#define KISS_TCP_PORT 8001

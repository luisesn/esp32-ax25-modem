idf_component_register(
    SRCS
        "main.c"
        "kiss.c"
        "transport.c"
        "transport_wifi.c"
        "LibAPRS-esp32-i2s/src/LibAPRS.cpp"
        "LibAPRS-esp32-i2s/src/AFSK.cpp"
        "LibAPRS-esp32-i2s/src/AX25.cpp"
        "LibAPRS-esp32-i2s/src/CRC-CCIT.c"
        "aux_file_management.c"
        "aux_config.c"
        "audio_stream.c"
    INCLUDE_DIRS
        "."
        "LibAPRS-esp32-i2s"
        "LibAPRS-esp32-i2s/src/"
    REQUIRES
        esp_wifi
        nvs_flash
        esp_netif
        lwip
        driver
        esp_driver_dac
        esp_driver_gpio
        esp_adc
        spiffs
        esp_http_server
        vfs
)
# AX25_MAX_FRAME_LEN por defecto es 330 (suficiente para APRS pero no para IP).
# 600 cubre el MTU típico de tncattach (512 B payload + 16 B cabecera AX.25 + 2 B CRC).
target_compile_definitions(__idf_main PRIVATE CUSTOM_FRAME_SIZE=600)

spiffs_create_partition_image(
    spiffs
    spiffs_data
    FLASH_IN_PROJECT
)

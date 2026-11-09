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
        "api_rest.c"
        "adpcm.c"
        "ax25ip.c"
        "digipeater.c"
        "dedup.c"
        "morse.c"
        "rx_stats.c"
        "sstv.c"
        "gps.c"
        "display.c"
        "remote_cmd.c"
        "rf_console.c"
        "ota.c"
        "ota_net.c"
        "delay_tune.c"
        "netdiag.c"
        "repeater.c"
        "squelch_sf.c"
        "rs_codec.c"
        "il2p.c"
    INCLUDE_DIRS
        "."
        "LibAPRS-esp32-i2s"
        "LibAPRS-esp32-i2s/src/"
    REQUIRES
        esp_wifi
        mbedtls
        nvs_flash
        esp_netif
        lwip
        driver
        esp_driver_dac
        esp_driver_gpio
        esp_driver_uart
        esp_driver_i2c
        esp_adc
        spiffs
        esp_http_server
        vfs
        espressif__cjson
        esp_timer
        app_update
        esp_rom
)
# AX25_MAX_FRAME_LEN por defecto es 330 (suficiente para APRS pero no para IP).
# 600 cubre el MTU típico de tncattach (512 B payload + 16 B cabecera AX.25 + 2 B CRC).
target_compile_definitions(__idf_main PRIVATE CUSTOM_FRAME_SIZE=600)

# Ficheros de DSP calientes: -O2 en lugar de -Og (el valor por defecto de IDF).
set_source_files_properties(
    "LibAPRS-esp32-i2s/src/AFSK.cpp"
    "rs_codec.c"
    "il2p.c"
    "squelch_sf.c"
    "adpcm.c"
    "LibAPRS-esp32-i2s/src/CRC-CCIT.c"
    PROPERTIES COMPILE_OPTIONS "-O2")

# Imagen SPIFFS: se prepara un directorio intermedio con index.html comprimido
# (index.html.gz, ~75 % menos: más rápido por WiFi y ahorra partición). El handler
# de "/" prefiere un index.html suelto (subido por /api/spiffs/upload) y si no
# existe sirve el .gz con Content-Encoding: gzip. Se regenera al cambiar spiffs_data.
set(SPIFFS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/spiffs_data")
set(SPIFFS_STAGE_DIR "${CMAKE_BINARY_DIR}/spiffs_staging")
file(GLOB_RECURSE _spiffs_files CONFIGURE_DEPENDS "${SPIFFS_SRC_DIR}/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_spiffs_files})
file(REMOVE_RECURSE "${SPIFFS_STAGE_DIR}")
file(COPY "${SPIFFS_SRC_DIR}/" DESTINATION "${SPIFFS_STAGE_DIR}")
if(EXISTS "${SPIFFS_STAGE_DIR}/index.html")
    if(NOT DEFINED python)
        set(python python3)
    endif()
    execute_process(
        COMMAND ${python} -c "import gzip,sys; d=open(sys.argv[1],'rb').read(); open(sys.argv[2],'wb').write(gzip.compress(d,9,mtime=0))"
                "${SPIFFS_STAGE_DIR}/index.html" "${SPIFFS_STAGE_DIR}/index.html.gz"
        RESULT_VARIABLE _gz_rc)
    if(_gz_rc EQUAL 0)
        file(REMOVE "${SPIFFS_STAGE_DIR}/index.html")
    else()
        message(WARNING "gzip de index.html falló: se flashea sin comprimir")
        file(REMOVE "${SPIFFS_STAGE_DIR}/index.html.gz")
    endif()
endif()

spiffs_create_partition_image(
    spiffs
    "${SPIFFS_STAGE_DIR}"
    FLASH_IN_PROJECT
)

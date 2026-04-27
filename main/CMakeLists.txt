idf_component_register(
                    SRCS "main.c" "LibAPRS-esp32-i2s/src/LibAPRS.cpp" "LibAPRS-esp32-i2s/src/AFSK.cpp" "LibAPRS-esp32-i2s/src/AX25.cpp" "LibAPRS-esp32-i2s/src/CRC-CCIT.c" "ptt.c"
                    INCLUDE_DIRS "." "LibAPRS-esp32-i2s" "LibAPRS-esp32-i2s/src/"
                    )
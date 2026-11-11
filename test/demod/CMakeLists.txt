# Regresión del demodulador RX v2 (harness.cpp incluye AFSK.cpp de producción).
add_executable(demod_regress harness.cpp ${LIB}/CRC-CCIT.c)
# Los stubs de test/demod/stubs tienen prioridad sobre test/stubs (esp_log.h, etc.)
target_include_directories(demod_regress BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs ${LIB} ${MAIN} ${CJSON_DIR})
target_include_directories(demod_regress PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../stubs)
target_compile_definitions(demod_regress PRIVATE CORPUS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/corpus")
target_compile_options(demod_regress PRIVATE -Wno-unused-function -Wno-unused-variable -Wno-missing-field-initializers -O2)
target_link_libraries(demod_regress PRIVATE cjson m)
add_test(NAME demod_synth COMMAND demod_regress synth)
file(GLOB DEMOD_WAVS ${CMAKE_CURRENT_SOURCE_DIR}/corpus/*.wav)
if(DEMOD_WAVS)
  add_test(NAME demod_corpus COMMAND demod_regress corpus)
endif()

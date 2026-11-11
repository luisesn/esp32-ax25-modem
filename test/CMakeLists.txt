# Banco de pruebas en host (sin ESP-IDF). Uso:
#   cmake -S test -B build-test [-DSANITIZE=ON] && cmake --build build-test && ctest --test-dir build-test --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(aprs_host_tests C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
option(SANITIZE "ASan + UBSan" ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -g -O1)
if(SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()
enable_testing()

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(MAIN ${ROOT}/main)
set(LIB  ${MAIN}/LibAPRS-esp32-i2s/src)
set(CJSON_DIR ${ROOT}/managed_components/espressif__cjson/cJSON)

# cJSON real (compila en host)
add_library(cjson STATIC ${CJSON_DIR}/cJSON.c)
target_include_directories(cjson PUBLIC ${CJSON_DIR})
target_compile_options(cjson PRIVATE -w)

# Stubs de ESP-IDF / FreeRTOS comunes
add_library(stubs STATIC stubs/stubs.c)
target_include_directories(stubs PUBLIC stubs)

# --- Tests unitarios: cada test/unit/test_<x>.c es un ejecutable; las fuentes de
# producción que enlaza se declaran en test/unit/<x>.srcs (una ruta por línea,
# relativa a la raíz del repo) o, si no existe, ninguna.
file(GLOB UNIT_TESTS ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_*.c)
foreach(src ${UNIT_TESTS})
  get_filename_component(name ${src} NAME_WE)
  set(srcs ${src})
  set(listfile ${CMAKE_CURRENT_SOURCE_DIR}/unit/${name}.srcs)
  if(EXISTS ${listfile})
    file(STRINGS ${listfile} lines)
    foreach(l ${lines})
      if(NOT l STREQUAL "" AND NOT l MATCHES "^#")
        list(APPEND srcs ${ROOT}/${l})
      endif()
    endforeach()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${listfile})
  endif()
  add_executable(${name} ${srcs})
  target_include_directories(${name} PRIVATE ${MAIN} ${LIB} ${CMAKE_CURRENT_SOURCE_DIR}/unit)
  target_link_libraries(${name} PRIVATE stubs cjson m)
  add_test(NAME ${name} COMMAND ${name})
endforeach()

# --- Regresión del demodulador (test/demod/CMakeLists.txt, si existe)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demod/CMakeLists.txt)
  add_subdirectory(demod)
endif()

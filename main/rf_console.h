#pragma once
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start a line-based TCP console bound exclusively to the ax25ip RF interface.
// Puede llamarse antes de que ax25ip este listo: las tareas reintentan cada 5 s. No-op si la consola esta desactivada.
void rf_console_init(const cJSON *cfg);

#ifdef __cplusplus
}
#endif

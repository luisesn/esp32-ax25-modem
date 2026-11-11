/* Compat host para incluir headers de la librería AFSK (AFSK.h -> FIFO.h) en un test.
 * FIFO.h usa taskENTER_CRITICAL/taskEXIT_CRITICAL, que el stub de FreeRTOS no define
 * (test/stubs/freertos es compartido: no se toca desde aquí). Incluir este fichero
 * ANTES de cualquier header de la librería o de un .c de producción que la incluya. */
#pragma once
#include "freertos/FreeRTOS.h"
#ifndef taskENTER_CRITICAL
#define taskENTER_CRITICAL(m) ((void)(m))
#define taskEXIT_CRITICAL(m)  ((void)(m))
#endif

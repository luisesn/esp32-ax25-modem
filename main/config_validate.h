#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Valida una configuración (config.json) recibida por POST /api/config antes de guardarla.
 *
 * - La raíz debe ser un objeto JSON.
 * - Cada clave conocida, si existe, debe tener el tipo correcto (bool, número, string,
 *   objeto o array) y, cuando aplica, estar dentro de su rango.
 * - Las claves desconocidas se aceptan sin error.
 *
 * Devuelve true si la configuración es válida. Si no lo es, devuelve false y escribe en
 * `err` (de tamaño `err_len`) un mensaje legible en español (≤120 caracteres).
 * No reserva memoria en el heap.
 */
bool config_validate(const cJSON *cfg, char *err, size_t err_len);

#ifdef __cplusplus
}
#endif

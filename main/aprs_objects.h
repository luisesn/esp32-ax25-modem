#pragma once
// Objetos y boletines APRS (emisión periódica + endpoints REST).
// Con objects.enabled=false y sin objetos en RAM no hay tarea ni buffers.
#include "cJSON.h"
#include "esp_http_server.h"

// Lee la sección "objects" de cfg (puede ser NULL) y arranca la tarea sólo si
// enabled y hay items/bulletins. No toma posesión de cfg.
void aprs_objects_init(cJSON *cfg);

// Reaplica la config en caliente (conserva los objetos creados por la API con
// persist:true). Arranca o detiene la tarea según proceda.
void aprs_objects_reload(cJSON *cfg);

// Registra POST /api/aprs/object y POST /api/aprs/bulletin (2 URIs).
void aprs_objects_register(httpd_handle_t srv);

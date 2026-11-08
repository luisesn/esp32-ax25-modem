#pragma once

#include <stdbool.h>

// OTA firmware update over HTTP.
// Registers POST /api/ota/upload on the audio_stream httpd server.
// Must be called after audio_stream_init().
void ota_init(void);

// Exclusión mutua entre las dos rutas de OTA (HTTP y espota): ambas escriben en
// la misma partición ota_next, así que solo una puede estar activa. ota_acquire()
// devuelve false si ya hay una en curso; quien la obtiene debe llamar a
// ota_release() en cualquier salida salvo tras un OTA correcto (el reinicio
// posterior libera todo, y mantenerla evita arrancar otra durante la espera).
bool ota_acquire(void);
void ota_release(void);

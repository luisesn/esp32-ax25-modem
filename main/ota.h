#pragma once

// OTA firmware update over HTTP.
// Registers POST /api/ota/upload on the audio_stream httpd server.
// Must be called after audio_stream_init().
void ota_init(void);

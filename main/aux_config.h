#ifndef CONFIG_H
#define CONFIG_H

#include "cJSON.h"

#define CONFIG_FILE_PATH "/spiffs/config.json"

#ifdef __cplusplus
extern "C" {
#endif

cJSON *config_load();
cJSON *config_reload();
void config_free_json(cJSON *config);
cJSON *config_get();
bool save_config(const char *json_str);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H
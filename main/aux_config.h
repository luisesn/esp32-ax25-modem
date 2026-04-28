#ifndef CONFIG_H
#define CONFIG_H

#include "cJSON.h"

#define CONFIG_FILE_PATH "/spiffs/config.json"

cJSON *config_load();
cJSON *config_reload();
void config_free_json(cJSON *config);
cJSON *config_get();
bool save_config(const char *json_str);
bool config_updated();

#endif // CONFIG_H
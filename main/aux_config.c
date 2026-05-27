#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "cJSON.h"
#include "aux_config.h"
#include "aux_file_management.h"

static const char *TAG = "Config";
static cJSON *root = NULL;

// Internal function to load config from file
static cJSON *config_load_from_file() {
    file_management_list_files();

    if (!file_management_file_exists(CONFIG_FILE_PATH)) {
        ESP_LOGE(TAG, "Config file does not exist: %s", CONFIG_FILE_PATH);
        return NULL;
    }

    FILE *f = fopen(CONFIG_FILE_PATH, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open config file: %s", CONFIG_FILE_PATH);
        return NULL;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *buffer = malloc(fsize + 1);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for config");
        fclose(f);
        return NULL;
    }

    size_t read_size = fread(buffer, 1, fsize, f);
    buffer[read_size] = '\0';
    fclose(f);

    cJSON *parsed_root = cJSON_Parse(buffer);
    free(buffer);

    if (parsed_root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return NULL;
    }

    ESP_LOGI(TAG, "Config loaded successfully");
    return parsed_root;
}

cJSON *config_get() {
    return root;
}

cJSON *config_load() {
    if (root != NULL) {
        ESP_LOGD(TAG, "Config already loaded, returning cached version");
        return cJSON_Duplicate(root, 1); // Return a deep copy
    }

    root = config_load_from_file();
    if (root != NULL) {
        return cJSON_Duplicate(root, 1); // Return a deep copy
    }
    return NULL;
}

cJSON *config_reload() {
    ESP_LOGI(TAG, "Reloading config...");

    // Free existing config if it exists
    if (root != NULL) {
        cJSON_Delete(root);
        root = NULL;
    }

    // Load fresh config
    root = config_load_from_file();
    // Return a duplicate, consistent with config_load(), so callers can safely
    // pass the result to init functions without risk of freeing the internal root.
    return root ? cJSON_Duplicate(root, 1) : NULL;
}

void config_free_json(cJSON *config) {
    if (config) {
        cJSON_Delete(config);
    }
}

bool save_config(const char *json_str) {
    FILE *f = fopen(CONFIG_FILE_PATH, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open config file for writing: %s", CONFIG_FILE_PATH);
        return false;
    }

    size_t written = fwrite(json_str, 1, strlen(json_str), f);
    fclose(f);

    if (written != strlen(json_str)) {
        ESP_LOGE(TAG, "Failed to write complete config to file");
        return false;
    }

    ESP_LOGI(TAG, "Config saved successfully");
    return true;
}


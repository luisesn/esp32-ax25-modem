#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "cJSON.h"
#include "aux_config.h"
#include "aux_file_management.h"

static const char *TAG = "Config";
static cJSON *root = NULL;
// Protects `root` against config_reload() (POST /api/config, on an httpd
// worker task) freeing/replacing it while another task is mid-cJSON_Duplicate
// in config_get()/config_load(). cJSON_Duplicate is pure memory copying (no
// I/O, bounded by config.json's size), so a short spinlock is appropriate —
// unlike e.g. the repeater's TX playback, this never blocks while held.
static portMUX_TYPE s_config_lock = portMUX_INITIALIZER_UNLOCKED;

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

// Returns a deep copy, consistent with config_load()/config_reload() — the
// caller must free it with config_free_json()/cJSON_Delete(). config_get()
// used to hand back the shared `root` pointer directly, which any concurrent
// config_reload() (triggered by POST /api/config from another task) could
// free out from under a caller still reading it. A lazy-init caller such as
// aprs_poll_v2() in LibAPRS.cpp can run this arbitrarily long after boot.
cJSON *config_get() {
    taskENTER_CRITICAL(&s_config_lock);
    cJSON *dup = root ? cJSON_Duplicate(root, 1) : NULL;
    taskEXIT_CRITICAL(&s_config_lock);
    return dup;
}

cJSON *config_load() {
    taskENTER_CRITICAL(&s_config_lock);
    if (root != NULL) {
        ESP_LOGD(TAG, "Config already loaded, returning cached version");
        cJSON *dup = cJSON_Duplicate(root, 1); // Return a deep copy
        taskEXIT_CRITICAL(&s_config_lock);
        return dup;
    }
    taskEXIT_CRITICAL(&s_config_lock);

    // File I/O must happen outside the critical section.
    cJSON *loaded = config_load_from_file();

    taskENTER_CRITICAL(&s_config_lock);
    if (root == NULL) {
        root = loaded;
        loaded = NULL;
    }
    cJSON *dup = root ? cJSON_Duplicate(root, 1) : NULL;
    taskEXIT_CRITICAL(&s_config_lock);

    if (loaded != NULL) {
        // Lost the race to another concurrent config_load() — discard.
        cJSON_Delete(loaded);
    }
    return dup;
}

cJSON *config_reload() {
    ESP_LOGI(TAG, "Reloading config...");

    // File I/O must happen outside the critical section.
    cJSON *fresh = config_load_from_file();
    if (fresh == NULL) {
        ESP_LOGE(TAG, "Reload failed, keeping previous config");
        return config_get();
    }

    taskENTER_CRITICAL(&s_config_lock);
    cJSON *old = root;
    root = fresh;
    cJSON *dup = cJSON_Duplicate(root, 1);
    taskEXIT_CRITICAL(&s_config_lock);

    if (old != NULL) {
        cJSON_Delete(old);
    }
    // Return a duplicate, consistent with config_load(), so callers can safely
    // pass the result to init functions without risk of freeing the internal root.
    return dup;
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


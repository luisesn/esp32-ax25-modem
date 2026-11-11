#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "aux_config.h"
#include "aux_file_management.h"

static const char *TAG = "Config";
static cJSON *root = NULL;
// Protege `root` frente a config_reload() (POST /api/config, tarea httpd) que
// lo libera/reemplaza mientras otra tarea hace cJSON_Duplicate. Mutex FreeRTOS
// (no sección crítica): la duplicación hace muchos malloc y no debe deshabilitar
// interrupciones (B10). Creación perezosa y segura entre tareas.
static portMUX_TYPE s_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_config_mutex = NULL;

static SemaphoreHandle_t config_mutex(void)
{
    if (s_config_mutex == NULL) {
        SemaphoreHandle_t m = xSemaphoreCreateMutex();
        taskENTER_CRITICAL(&s_mutex_init_lock);
        if (s_config_mutex == NULL) { s_config_mutex = m; m = NULL; }
        taskEXIT_CRITICAL(&s_mutex_init_lock);
        if (m) vSemaphoreDelete(m);
    }
    return s_config_mutex;
}
#define CONFIG_LOCK()   xSemaphoreTake(config_mutex(), portMAX_DELAY)
#define CONFIG_UNLOCK() xSemaphoreGive(s_config_mutex)

// Internal function to load config from file
static cJSON *config_load_from_file() {
    file_management_list_files();

    // Recuperación tras corte durante save_config(): config.tmp completo, sin config.json.
    if (!file_management_file_exists(CONFIG_FILE_PATH) &&
        file_management_file_exists("/spiffs/config.tmp")) {
        ESP_LOGW(TAG, "config.json missing, recovering from config.tmp");
        rename("/spiffs/config.tmp", CONFIG_FILE_PATH);
    }

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
    CONFIG_LOCK();
    cJSON *dup = root ? cJSON_Duplicate(root, 1) : NULL;
    CONFIG_UNLOCK();
    return dup;
}

cJSON *config_load() {
    CONFIG_LOCK();
    if (root != NULL) {
        ESP_LOGD(TAG, "Config already loaded, returning cached version");
        cJSON *dup = cJSON_Duplicate(root, 1); // Return a deep copy
        CONFIG_UNLOCK();
        return dup;
    }
    CONFIG_UNLOCK();

    // File I/O must happen outside the critical section.
    cJSON *loaded = config_load_from_file();

    CONFIG_LOCK();
    if (root == NULL) {
        root = loaded;
        loaded = NULL;
    }
    cJSON *dup = root ? cJSON_Duplicate(root, 1) : NULL;
    CONFIG_UNLOCK();

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

    CONFIG_LOCK();
    cJSON *old = root;
    root = fresh;
    cJSON *dup = cJSON_Duplicate(root, 1);
    CONFIG_UNLOCK();

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

// Escritura segura: se escribe a config.tmp y sólo entonces se sustituye
// config.json. Un corte de corriente durante la escritura deja el config.json
// anterior intacto; si el corte ocurre entre remove y rename, config_load_from_file()
// recupera config.tmp (completo y verificado por tamaño).
#define CONFIG_TMP_PATH "/spiffs/config.tmp"

bool save_config(const char *json_str) {
    size_t len = strlen(json_str);
    FILE *f = fopen(CONFIG_TMP_PATH, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open temp config for writing: %s", CONFIG_TMP_PATH);
        return false;
    }

    size_t written = fwrite(json_str, 1, len, f);
    int close_rc = fclose(f);
    struct stat st;
    if (written != len || close_rc != 0 || stat(CONFIG_TMP_PATH, &st) != 0 || (size_t)st.st_size != len) {
        ESP_LOGE(TAG, "Failed to write complete config to temp file");
        unlink(CONFIG_TMP_PATH);
        return false;
    }

    unlink(CONFIG_FILE_PATH);   // SPIFFS no sobrescribe en rename()
    if (rename(CONFIG_TMP_PATH, CONFIG_FILE_PATH) != 0) {
        ESP_LOGE(TAG, "rename(%s -> %s) failed; temp kept for recovery", CONFIG_TMP_PATH, CONFIG_FILE_PATH);
        return false;
    }

    ESP_LOGI(TAG, "Config saved successfully");
    return true;
}


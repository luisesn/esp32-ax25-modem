#include <stdio.h> //for prints
#include <string.h> //for strings
#include <sys/unistd.h> //for file open and close
#include <sys/stat.h> //file system interaction
#include <stdbool.h> //for bool
#include <dirent.h> //for directory operations
#include "esp_log.h" //for logs
#include "esp_spiffs.h" //spiffs initialization
#include "aux_file_management.h"

static const char *TAG = "Spiffs";

esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs", // here "/" is root and "/spiffs" is our spiffs partition
      .partition_label = NULL, // start from partition label
      .max_files = 10, //this tells how many files spiffs can open
      .format_if_mount_failed = true //intimidation of mount status
    };

void file_management_init() {
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    ESP_LOGI(TAG, "SPIFFS mounted successfully");
}

void file_management_close() {
    esp_vfs_spiffs_unregister(conf.partition_label);// this is for unregister
    ESP_LOGI(TAG, "SPIFFS unmounted");
}

bool file_management_file_exists(const char *filename) {
    struct stat st;
    if (stat(filename, &st) == 0) {
        return true;
    } else {
        return false;
    }
}

void file_management_list_files() {
    DIR *dir = opendir("/spiffs");
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory /spiffs");
        return;
    }

    struct dirent *entry;
    ESP_LOGI(TAG, "Files in SPIFFS:");
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "  %s", entry->d_name);
    }

    closedir(dir);
}


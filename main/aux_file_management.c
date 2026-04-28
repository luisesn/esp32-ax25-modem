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
      .max_files = 5, //this tells how many files spiffs can open
      .format_if_mount_failed = true //intimidation of mount status
    };

void file_management_init() {
    esp_vfs_spiffs_register(&conf);//this is for register
    ESP_LOGI(TAG, "SPIFFS mounted successfully");
}

void file_management_close() {
    esp_vfs_spiffs_unregister(conf.partition_label);// this is for unregister
    ESP_LOGI(TAG, "SPIFFS unmounted");
}

void test() {
    ESP_LOGI(TAG, "Opening file");
    FILE *f;
    f = fopen("/spiffs/my_file.txt","w");
    fprintf(f,"Hello Medium People Do you like my Blogs?");
    fclose(f);
    ESP_LOGI(TAG,"File written");
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

void file_write_string(const char *filename, const char *content) {
    FILE *f = fopen(filename, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s for writing", filename);
        return;
    }
    fprintf(f, "%s", content);
    fclose(f);
    ESP_LOGI(TAG, "File %s written successfully", filename);
}

char* file_read_string(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s for reading", filename);
        return NULL;
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    if (file_size <= 0) {
        fclose(f);
        return NULL;
    }
    
    // Allocate memory for content + null terminator
    char *content = (char*)malloc(file_size + 1);
    if (content == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file content");
        fclose(f);
        return NULL;
    }
    
    // Read file content
    size_t bytes_read = fread(content, 1, file_size, f);
    content[bytes_read] = '\0'; // Null terminate
    
    fclose(f);
    ESP_LOGI(TAG, "File %s read successfully", filename);
    return content;
}
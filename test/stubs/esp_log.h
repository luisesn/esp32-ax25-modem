#pragma once
#include <stdio.h>
#ifdef TEST_VERBOSE
#define ESP_LOGI(t, f, ...) fprintf(stderr, "I %s: " f "\n", t, ##__VA_ARGS__)
#define ESP_LOGW(t, f, ...) fprintf(stderr, "W %s: " f "\n", t, ##__VA_ARGS__)
#define ESP_LOGE(t, f, ...) fprintf(stderr, "E %s: " f "\n", t, ##__VA_ARGS__)
#else
#define ESP_LOGI(t, f, ...) ((void)(t))
#define ESP_LOGW(t, f, ...) ((void)(t))
#define ESP_LOGE(t, f, ...) ((void)(t))
#endif
#define ESP_LOGD(t, f, ...) ((void)(t))
#define ESP_LOGV(t, f, ...) ((void)(t))

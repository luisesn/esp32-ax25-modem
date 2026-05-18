#pragma once
#include "cJSON.h"

// Initialise I2C bus, SSD1306, and start display_task.
// cfg may be NULL; reads display.enabled and display.i2c_addr from config if present.
void display_init(cJSON *cfg);

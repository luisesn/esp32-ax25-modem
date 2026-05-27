#ifndef FILE_MANAGEMENT_H
#define FILE_MANAGEMENT_H

#include <stdbool.h>

void file_management_init();
void file_management_close();
bool file_management_file_exists(const char *filename);
void file_management_list_files();

#endif // FILE_MANAGEMENT_H

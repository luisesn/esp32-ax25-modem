#ifndef FILE_MANAGEMENT_H
#define FILE_MANAGEMENT_H

#include <stdbool.h>

void file_management_init();
void file_management_close();
void test();
bool file_management_file_exists(const char *filename);
void file_management_list_files();
void file_write_string(const char *filename, const char *content);
char* file_read_string(const char *filename);

#endif // FILE_MANAGEMENT_H
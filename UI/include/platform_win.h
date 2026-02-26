#ifndef ASTRA_GS_PLATFORM_WIN_H
#define ASTRA_GS_PLATFORM_WIN_H

#include "flight_data.h" // for ComPortInfo
#include <string>
#include <vector>

void OpenFileDialog(char *path);
const char *get_filename_from_path(const char *full_path);
std::vector<ComPortInfo> enumerate_ports();

#endif
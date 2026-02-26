#ifndef ASTRA_GS_PLATFORM_WIN_H
#define ASTRA_GS_PLATFORM_WIN_H

#include <string>
#include <vector>

void OpenFileDialog(char *path);
const char *get_filename_from_path(const char *full_path);

struct ComPortInfo {
  std::string portName;     // COM3
  std::string friendlyName; // USB Serial Device (COM3)
};

extern std::vector<ComPortInfo> ports;
void enumerate_ports();

#endif
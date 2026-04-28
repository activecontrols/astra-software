#ifndef ASTRA_GS_PLATFORM_WIN_H
#define ASTRA_GS_PLATFORM_WIN_H

#include "flight_data_state.h" // for ComPortInfo

void OpenFileDialog(char *path);
const char *get_filename_from_path(const char *full_path);
std::vector<ComPortInfo> enumerate_ports();

Serial *open_serial_port(const char *name);
unsigned long long get_time_us();
void platform_begin();

void SaveFileDialog(char *path);
void SaveFileDialog(char *path, const char *file_specs[], const char *spec_names[], unsigned int n_specs, unsigned int default_spec_idx);

#endif
#ifndef ASTRA_GS_PLATFORM_WIN_H
#define ASTRA_GS_PLATFORM_WIN_H

#include "flight_data_state.h" // for ComPortInfo

void OpenFileDialog(char *path);
const char *get_filename_from_path(const char *full_path);
std::vector<ComPortInfo> enumerate_ports();
void open_serial_port(HANDLE *hSerial, const char *com_port);
void close_serial_port(HANDLE *hSerial);

#endif
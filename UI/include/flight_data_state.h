#ifndef ASTRA_GS_FLIGHT_DATA_STATE
#define ASTRA_GS_FLIGHT_DATA_STATE

#include <stdio.h>
#include <string>
#include <vector>
#include <windows.h>

struct ComPortInfo {
  std::string portName;     // COM3
  std::string friendlyName; // USB Serial Device (COM3)
};

#define MODE_SERIAL_INPUT 0
#define MODE_FILE_INPUT 1

struct flight_data_state_t {
  int data_input_mode;

  // serial input mode
  std::vector<ComPortInfo> ports;
  int fv_serial_idx;         // serial index for flight vehicle radio
  int rtk_serial_idx;        // serial index for GPS RTK source
  bool fv_serial_port_open;  // used for opening/closing ports and communicating status back to user
  bool rtk_serial_port_open; // used for opening/closing ports and communicating status back to user
  HANDLE fv_serial = INVALID_HANDLE_VALUE;
  HANDLE rtk_serial = INVALID_HANDLE_VALUE;

  // file input mode
  char selected_file_path[260] = "";
  FILE *input_file;
  int file_length;
  int file_read_progress;
  bool file_reading_paused;
};

extern flight_data_state_t FlightDataState; // this ends up in flight_data

#endif
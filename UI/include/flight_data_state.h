#ifndef ASTRA_GS_FLIGHT_DATA_STATE
#define ASTRA_GS_FLIGHT_DATA_STATE

#include "serial.h"
#include <stdio.h>
#include <string>
#include <vector>
#include <windows.h>
#include <memory>

#include <cstdint>

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

  std::unique_ptr<Serial> fv_serial = 0;
  std::unique_ptr<Serial> rtk_serial = 0;

  // file input mode
  char selected_file_path[260] = "";
  FILE *input_file;
  int file_length;
  int file_read_progress;
  bool file_reading_paused;

  uint64_t replay_play_start_us;   // the time when the play button was last pressed
  uint64_t replay_pause_offset_us; // the time "accumulated" before the play button was last pressed
};

extern flight_data_state_t FlightDataState; // this ends up in flight_data

#endif
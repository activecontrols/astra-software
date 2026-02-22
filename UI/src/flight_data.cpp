#include "flight_data.h"
#include <stdio.h>

flight_history_t FlightHistory;
flight_packet_t active_packet; // may be partially filled
FILE *input_file;

void init_flight_data() {
  FlightHistory = {};

  FlightHistory.write_pos = 0;
  FlightHistory.read_start_pos = FlightHistory.write_pos;
  FlightHistory.read_end_pos = FlightHistory.read_start_pos + FLIGHT_HISTORY_LENGTH - 1;

  input_file = fopen("sample_flight.txt", "r");
}

void deinit_flight_data() {
  fclose(input_file);
}

void parse_packet(char *msg) {
  // TODO - full decode
  // TODO - modernize this with the new flight data decoding system
  if (msg[0] == '>' && msg[1] == 'a') {
    sscanf(msg, ">a %f %f %f", &active_packet.accel_x, &active_packet.accel_y, &active_packet.accel_z);
  }

  // commit packet // TODO - only do this when ready
  // TODO - logic to generate this automatically
  FlightHistory.accel_x[FlightHistory.write_pos] = active_packet.accel_x;
  FlightHistory.accel_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_x;
  FlightHistory.accel_y[FlightHistory.write_pos] = active_packet.accel_y;
  FlightHistory.accel_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_y;
  FlightHistory.accel_z[FlightHistory.write_pos] = active_packet.accel_z;
  FlightHistory.accel_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_z;

  FlightHistory.write_pos += 1;
  FlightHistory.write_pos %= FLIGHT_HISTORY_LENGTH;
  FlightHistory.read_start_pos = FlightHistory.write_pos;
  FlightHistory.read_end_pos = FlightHistory.read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}

// TODO - goofy old code should be re-written

#define BUF_SIZE 4096
char buffer[BUF_SIZE];

void load_data_from_file_periodic() {
  int write_idx = 0;
  while (true) {
    int c = fgetc(input_file);
    if (c == EOF) {
      return;
    }
    buffer[write_idx] = c;
    write_idx++;
    if (write_idx >= BUF_SIZE) {
      return;
    }
    if (c == '\r' || c == '\n') {
      buffer[write_idx - 1] = '\0';
      parse_packet(buffer);
      return;
    }
  }
}

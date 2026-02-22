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
    sscanf(msg, ">a %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &active_packet.accel_x, &active_packet.accel_y, &active_packet.accel_z, &active_packet.gyro_yaw, &active_packet.gyro_pitch,
           &active_packet.gyro_roll, &active_packet.mag_x, &active_packet.mag_y, &active_packet.mag_z, &active_packet.gps_pos_north, &active_packet.gps_pos_west, &active_packet.gps_pos_up,
           &active_packet.gps_vel_north, &active_packet.gps_vel_west, &active_packet.gps_vel_up);
  } else if (msg[0] == '>' && msg[1] == 'b') {
    sscanf(msg, ">b %f %f %f %f %f %f %f %f %f %f", &active_packet.state_q_vec_new, &active_packet.state_q_vec_0, &active_packet.state_q_vec_1, &active_packet.state_q_vec_2,
           &active_packet.state_pos_north, &active_packet.state_pos_west, &active_packet.state_pos_up, &active_packet.state_vel_north, &active_packet.state_vel_west, &active_packet.state_vel_up);
  } else if (msg[0] == '>' && msg[1] == 'c') {
    sscanf(msg, ">c %f %f %f %f %f %f %f", &active_packet.gimbal_yaw_raw, &active_packet.gimbal_pitch_raw, &active_packet.thrust_N, &active_packet.roll_N, &active_packet.target_pos_north,
           &active_packet.target_pos_west, &active_packet.target_pos_up);
  }

  // commit packet // TODO - only do this when ready
  // use `gen_flight_data_code.py` to update this
  FlightHistory.accel_x[FlightHistory.write_pos] = active_packet.accel_x;
  FlightHistory.accel_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_x;
  FlightHistory.accel_y[FlightHistory.write_pos] = active_packet.accel_y;
  FlightHistory.accel_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_y;
  FlightHistory.accel_z[FlightHistory.write_pos] = active_packet.accel_z;
  FlightHistory.accel_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_z;
  FlightHistory.gyro_yaw[FlightHistory.write_pos] = active_packet.gyro_yaw;
  FlightHistory.gyro_yaw[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.gyro_yaw;
  FlightHistory.gyro_pitch[FlightHistory.write_pos] = active_packet.gyro_pitch;
  FlightHistory.gyro_pitch[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.gyro_pitch;
  FlightHistory.gyro_roll[FlightHistory.write_pos] = active_packet.gyro_roll;
  FlightHistory.gyro_roll[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.gyro_roll;
  FlightHistory.mag_x[FlightHistory.write_pos] = active_packet.mag_x;
  FlightHistory.mag_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.mag_x;
  FlightHistory.mag_y[FlightHistory.write_pos] = active_packet.mag_y;
  FlightHistory.mag_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.mag_y;
  FlightHistory.mag_z[FlightHistory.write_pos] = active_packet.mag_z;
  FlightHistory.mag_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.mag_z;
  FlightHistory.gps_pos_north = active_packet.gps_pos_north;
  FlightHistory.gps_pos_west = active_packet.gps_pos_north;
  FlightHistory.gps_pos_up = active_packet.gps_pos_north;
  FlightHistory.gps_vel_north = active_packet.gps_pos_north;
  FlightHistory.gps_vel_west = active_packet.gps_pos_north;
  FlightHistory.gps_vel_up = active_packet.gps_pos_north;

  FlightHistory.state_q_vec_new = active_packet.state_q_vec_new;
  FlightHistory.state_q_vec_0 = active_packet.state_q_vec_0;
  FlightHistory.state_q_vec_1 = active_packet.state_q_vec_1;
  FlightHistory.state_q_vec_2 = active_packet.state_q_vec_2;

  FlightHistory.state_pos_north = active_packet.state_pos_north;
  FlightHistory.state_pos_west = active_packet.state_pos_west;
  FlightHistory.state_pos_up = active_packet.state_pos_up;
  FlightHistory.state_vel_north = active_packet.state_vel_north;
  FlightHistory.state_vel_west = active_packet.state_vel_west;
  FlightHistory.state_vel_up = active_packet.state_vel_up;

  FlightHistory.gimbal_yaw_raw = active_packet.gimbal_yaw_raw;
  FlightHistory.gimbal_pitch_raw = active_packet.gimbal_pitch_raw;
  FlightHistory.thrust_N = active_packet.thrust_N;
  FlightHistory.roll_N = active_packet.roll_N;

  FlightHistory.target_pos_north = active_packet.target_pos_north;
  FlightHistory.target_pos_west = active_packet.target_pos_west;
  FlightHistory.target_pos_up = active_packet.target_pos_up;

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

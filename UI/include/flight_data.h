#ifndef ASTRA_GS_FLIGHT_DATA_H
#define ASTRA_GS_FLIGHT_DATA_H

#include <stdio.h>
#include <string>
#include <vector>

// this is a ring buffer to create history graphs
// if packets arrive from earlier to later as ABCDE we store
// [{0 0 0 0} 0 0 0 0] - rs = 0, wp = 0/4
// [A {0 0 0 A} 0 0 0] - rs = 1, wp = 1/5
// [A B {0 0 A B} 0 0] - rs = 2, wp = 2/6
// [A B C {0 A B C} 0] - rs = 3, wp = 3/7
// [{A B C D} A B C D] - rs = 0, wp = 0/4
// [E {B C D E} B C D] - rs = 1, wp = 1/5

#define FLIGHT_HISTORY_LENGTH 1000

struct flight_history_t {
  float accel_x[FLIGHT_HISTORY_LENGTH * 2];
  float accel_y[FLIGHT_HISTORY_LENGTH * 2];
  float accel_z[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_yaw[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_pitch[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_roll[FLIGHT_HISTORY_LENGTH * 2];
  float mag_x[FLIGHT_HISTORY_LENGTH * 2];
  float mag_y[FLIGHT_HISTORY_LENGTH * 2];
  float mag_z[FLIGHT_HISTORY_LENGTH * 2];
  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;
  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float state_q_vec_new;
  float state_q_vec_0;
  float state_q_vec_1;
  float state_q_vec_2;
  float state_pos_north;
  float state_pos_west;
  float state_pos_up;
  float state_vel_north;
  float state_vel_west;
  float state_vel_up;

  float gimbal_yaw_raw;
  float gimbal_pitch_raw;
  float thrust_N;
  float roll_N;

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;

  // points to the oldest stored data
  int read_start_pos;
  // points to the most recently written data
  int read_end_pos; // always equal to read_start + FLIGHT_HISTORY_LENGTH - 1
  // points to the next location to write (same as read_start)
  int write_pos;
};

// a single frame of flight history
struct flight_packet_t {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;
  float mag_x;
  float mag_y;
  float mag_z;
  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;
  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float state_q_vec_new;
  float state_q_vec_0;
  float state_q_vec_1;
  float state_q_vec_2;
  float state_pos_north;
  float state_pos_west;
  float state_pos_up;
  float state_vel_north;
  float state_vel_west;
  float state_vel_up;

  float gimbal_yaw_raw;
  float gimbal_pitch_raw;
  float thrust_N;
  float roll_N;

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;
};

extern flight_history_t FlightHistory; // public interface

#define MODE_SERIAL_INPUT 0
#define MODE_FILE_INPUT 1

struct ComPortInfo {
  std::string portName;     // COM3
  std::string friendlyName; // USB Serial Device (COM3)
};

struct flight_data_state_t {
  int data_input_mode;

  // serial input mode
  std::vector<ComPortInfo> ports;

  // file input mode
  char selected_file_path[260] = "";
  FILE *input_file;
  int file_length;
  int file_read_progress;
  bool file_reading_paused;
};

extern flight_data_state_t FlightDataState;

void init_flight_data();
void deinit_flight_data();
void load_flight_replay();
void load_data_from_file_periodic();

#endif

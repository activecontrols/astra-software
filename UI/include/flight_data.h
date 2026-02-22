#ifndef ASTRA_GS_FLIGHT_DATA_H
#define ASTRA_GS_FLIGHT_DATA_H

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
  float gps_pos_north[FLIGHT_HISTORY_LENGTH * 2];
  float gps_pos_west[FLIGHT_HISTORY_LENGTH * 2];
  float gps_pos_up[FLIGHT_HISTORY_LENGTH * 2];
  float gps_vel_north[FLIGHT_HISTORY_LENGTH * 2];
  float gps_vel_west[FLIGHT_HISTORY_LENGTH * 2];
  float gps_vel_up[FLIGHT_HISTORY_LENGTH * 2];

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
};

extern flight_history_t FlightHistory; // public interface

void init_flight_data();
void deinit_flight_data();
void load_data_from_file_periodic();

#endif

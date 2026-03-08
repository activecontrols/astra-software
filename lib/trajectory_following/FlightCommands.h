#pragma once
#include "stdint.h"

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

namespace FlightCommands {
extern bool kill_flag;
extern bool arm_flag;

void reset();
void process_cmd(uint8_t *cmd_buf, int cmd_len);
void encode(uint8_t c);
void send_telemetry(flight_packet_t fp);
} // namespace FlightCommands

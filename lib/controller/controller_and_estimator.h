#pragma once

#include "astra_structs.h"
#include "matlab_funcs.h"

struct Controller_State {
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
  float gyro_bias_yaw;
  float gyro_bias_pitch;
  float gyro_bias_roll;
  float accel_bias_x;
  float accel_bias_y;
  float accel_bias_z;
  float mag_bias_x;
  float mag_bias_y;
  float mag_bias_z;

  float filter_out[9];
};

struct Controller_Input {
  // System Status
  bool GND_val;
  bool new_imu_packet;
  bool new_gps_packet;

  // Sensor Inputs
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

  float gps_pos_covar[3][3];
  float gps_vel_covar[3][3];

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;
};

namespace ControllerAndEstimator {
extern Matrix9_9 Flight_P;
extern Vector19 x_est;

void init_controller_and_estimator_constants();
Controller_Output get_controller_output(Controller_Input ci, float ideal_dT, float loop_dT, Controller_State *cs);
}; // namespace ControllerAndEstimator

#pragma once
#define __packed __attribute__((__packed__)) // patch this in on all platforms

// this file contains shared structs for use across the embedded code and UI

struct GPS_Point {
  float north; // meters
  float west;  // meters
  float up;    // meters
};

struct GPS_Velocity {
  float north; // m/s velocity north
  float west;  // m/s velocity west
  float up;    // m/s velocity up
};

struct __packed IMU_MAG_State {
  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;

  float mag_x;
  float mag_y;
  float mag_z;
};

struct __packed GPS_State {
  GPS_Point gps_pos;
  GPS_Velocity gps_vel;

  float posCovNN; // m^2
  float posCovNE; // m^2
  float posCovND; // m^2
  float posCovEE; // m^2
  float posCovED; // m^2
  float posCovDD; // m^2
  float velCovNN; // m^2/s^2
  float velCovNE; // m^2/s^2
  float velCovND; // m^2/s^2
  float velCovEE; // m^2/s^2
  float velCovED; // m^2/s^2
  float velCovDD; // m^2/s^2
};

struct Controller_Input {
  // System Status
  bool GND_val;
  bool new_imu_packet;
  bool new_gps_packet;

  // Sensor Inputs
  IMU_MAG_State imu_mag_state;
  GPS_State gps_state;

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;
};

struct Controller_X_Est {
  float q_vec_w;
  float q_vec_x;
  float q_vec_y;
  float q_vec_z;
  float est_pos_north;
  float est_pos_west;
  float est_pos_up;
  float est_vel_north;
  float est_vel_west;
  float est_vel_up;
  float gyro_bias_yaw;
  float gyro_bias_pitch;
  float gyro_bias_roll;
  float accel_bias_x;
  float accel_bias_y;
  float accel_bias_z;
  float mag_bias_x;
  float mag_bias_y;
  float mag_bias_z;
};

struct Controller_Internals {
  Controller_X_Est x_est;
  float filter_out[9];
};

struct Controller_Output {
  float thrust_N;
  float roll_rad_sec_squared;
  float gimbal_pitch_deg;
  float gimbal_yaw_deg;
};

// telemetry packet sent by vehicle to the UI
// also used by the UI save format
struct telemetry_packet_t {
  IMU_MAG_State imu_mag_state;
  GPS_State gps_state;
  Controller_X_Est x_est;
  Controller_Output co;

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;

  float elapsed_time;
  bool GND_flag;
  bool flight_armed;
  float thrust_perc;
  float diffy_perc;
  int rtk_status;
  float gps_hor_prec;
  float gps_ver_prec;
  int gps_sat_count;
};
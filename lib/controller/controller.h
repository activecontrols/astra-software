#ifndef CONTROLLER_H
#define CONTROLLER_H

struct Controller_Output {
  // TODO - what are these?

  float thrust;
  float roll;
  float gimbal_pitch;
  float gimbal_yaw;
};

struct Controller_Input {
  // System Status
  float GND_val;
  bool new_imu_packet;
  bool new_gps_packet;

  // Sensor Inputs
  float accel_north;
  float accel_west;
  float accel_up;
  float gyro_pitch;
  float gyro_yaw;
  float gyro_roll;
  float mag_north;
  float mag_west;
  float mag_up;
  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;
  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  // TODO - what are these?
};

namespace Controller {
void begin();
void reset_controller_state();
Controller_Output get_controller_output(Controller_Input ci);
}; // namespace Controller

#endif
#include "controller.h"
#include "matlab_funcs.h"

namespace Controller {
t_constantsASTRA constantsASTRA;
Matrix12_12 P;
Vector13 x_est;
Vector3 lastEMA;
Matrix4_12 K;

unsigned long last_call_time; // ms

void begin() {
  reset_controller_state();
}

void reset_controller_state() {
  constantsASTRA.g = 9.8100;
  constantsASTRA.mag << 0.8660, 0, -0.5000;
  constantsASTRA.Q << 5.000020833333334e-06, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000000000001e-10, 0, 0, 0, 5.000020833333334e-06, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000000000001e-10, 0, 0, 0,
      5.000020833333334e-06, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000000000001e-10, 0, 0, 0, 6.250000000000001e-07, 0, 0, 2.083333333333334e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000000000001e-07, 0, 0,
      2.083333333333334e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000000000001e-07, 0, 0, 2.083333333333334e-09, 0, 0, 0, 0, 0, 0, 2.500000000000000e-04, 0, 0, 6.250000000000001e-07, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 2.500000000000000e-04, 0, 0, 6.250000000000001e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000000000000e-04, 0, 0, 6.250000000000001e-07, 0, 0, 0, -6.250000000000001e-10, 0, 0, 0, 0, 0, 0, 0, 0,
      6.250000000000001e-08, 0, 0, 0, -6.250000000000001e-10, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000000000001e-08, 0, 0, 0, -6.250000000000001e-10, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000000000001e-08;
  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.1500;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.1000;

  P = 1 * Matrix12_12::Identity();
  x_est = Vector13::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();

  K << 1.264151651090222, -0.000000000000001, 0.000000000000000, -0.000000000000000, -0.000053046537955, -0.000000000000000, -0.000000000000000, -0.096483681802069, -0.000000000000000,
      0.211048333968153, -0.000000000000000, 0.000000000000000, 0.000000000000001, 1.281438633135994, 0.000000000000000, 0.000053046537955, 0.000000000000000, 0.000000000000000, 0.096484166222293,
      -0.000000000000000, -0.000000000000000, 0.000000000000000, 0.216859134794888, 0.000000000000000, 0.000000000000024, -0.000000000000097, 0.000000000000049, -0.000000000000000, 0.000000000000000,
      0.004559014113909, -0.000000000000000, -0.000000000000001, 2.281506180341063, 0.000000000000005, -0.000000000000010, 0.000000000000002, 0.000000000000006, -0.000000000000007, 4.748973035322446,
      0.000000000000000, 0.000000000000000, 0.000000000000000, -0.000000000000000, -0.000000000000001, -0.000000000000000, -0.000000000000000, -0.000000000000000, 0.536451267774102;

  last_call_time = millis();
}

Controller_Output get_controller_output(Controller_Input ci) {
  unsigned long call_time = millis();
  float dT = (call_time - last_call_time) / 1000.0;
  last_call_time = call_time;

  Vector15 z;
  // clang-format off
  z << ci.accel_north, ci.accel_west, ci.accel_up, 
       ci.gyro_pitch, ci.gyro_yaw, ci.gyro_roll,
       ci.mag_north, ci.mag_west, ci.mag_up,
       ci.gps_pos_north, ci.gps_pos_west, ci.gps_pos_up, 
       ci.gps_vel_north, ci.gps_vel_west, ci.gps_vel_up;
  // clang-format on

  x_est = EstimateStateFCN(x_est, constantsASTRA, z, dT, ci.GND_val, P, ci.new_imu_packet, ci.new_gps_packet);
  Vector3 EMA_G = EMA_Gyros(z, lastEMA);
  Vector15 X = StateAUG(x_est, EMA_G);
  Vector12 error = ref_generator3();
  Vector4 raw_co = -K * error;
  // TODO - clamp

  Controller_Output co;
  // TODO - fill this
  return co;
}
} // namespace Controller
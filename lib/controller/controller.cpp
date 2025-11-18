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
  constantsASTRA.g = 9.80145;
  constantsASTRA.m = 1;
  constantsASTRA.mag << -0.4512, 0, 0.8924;
  constantsASTRA.Q << 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, 0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, 0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10,
      0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0, 0, 0, 0,
      2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0, -6.250000e-10, 0, 0, 0, 0, 0,
      0, 0, 0, 1.250000e-07, 0, 0, 0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07, 0, 0, 0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07;
  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.1500;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.1000;

  P = 1 * Matrix12_12::Identity();
  x_est = Vector13::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();

  K << 1.221739e+00, 1.138010e-15, -3.842269e-15, -1.776182e-18, -5.304654e-05, -4.322925e-18, 2.964490e-17, -9.648249e-02, -1.793211e-17, 1.971226e-01, 1.182562e-16, -3.093029e-16, 2.656070e-16,
      1.240890e+00, -5.030489e-16, 5.304654e-05, -1.777516e-19, 7.801908e-19, 9.648303e-02, 6.718166e-17, -9.218939e-18, 1.725272e-16, 2.033502e-01, -4.973344e-17, 1.605792e-14, -6.956013e-14,
      -1.047017e-14, -5.286303e-17, -1.077854e-17, 4.559014e-03, -1.028685e-15, -4.583739e-15, 2.281506e+00, 3.380334e-16, -1.370787e-14, -4.785263e-16, 5.754042e-14, 1.577365e-14, 4.748973e+00,
      1.334778e-18, 9.473637e-18, 6.725418e-19, 2.688724e-15, -7.161693e-15, 1.992377e-17, 5.853777e-15, -5.543101e-16, 5.364513e-01;

  last_call_time = millis();
}

Controller_Output get_controller_output(Controller_Input ci) {
  unsigned long call_time = millis();
  float dT = (call_time - last_call_time) / 1000.0;
  last_call_time = call_time;

  Vector15 z;
  // clang-format off
  z << ci.accel_x, ci.accel_y, ci.accel_z, 
       ci.gyro_yaw, ci.gyro_pitch, ci.gyro_roll,
       ci.mag_x, ci.mag_y, ci.mag_z,
       ci.gps_pos_north, ci.gps_pos_west, ci.gps_pos_up, 
       ci.gps_vel_north, ci.gps_vel_west, ci.gps_vel_up;
  // clang-format on

  x_est = EstimateStateFCN(x_est, constantsASTRA, z, dT, ci.GND_val, P, ci.new_imu_packet, ci.new_gps_packet);
  Vector3 EMA_G = EMA_Gyros(z, lastEMA);
  Vector15 X = StateAUG(x_est, EMA_G);
  Vector3 TargetPos;
  TargetPos << ci.target_pos_north, ci.target_pos_west, ci.target_pos_up;
  Vector12 error = ref_generator3(X, TargetPos);
  Vector4 raw_co = -K * error;
  raw_co(2) = raw_co(2) + constantsASTRA.g * constantsASTRA.m;
  raw_co = output_clamp(raw_co);
  if (ci.GND_val) {
    raw_co = Vector4::Zero();
  }

  Controller_Output co;
  co.gimbal_yaw_deg = raw_co(0) * 180 / M_PI;
  co.gimbal_pitch_deg = raw_co(1) * 180 / M_PI;
  co.thrust_N = raw_co(2);
  co.roll_N = raw_co(3);

  Serial.print(">");
  for (int i = 0; i < 15; i++) {
    Serial.print(z(i), 4);
    Serial.print(" ");
  }
  for (int i = 0; i < 15; i++) {
    Serial.print(X(i), 4);
    Serial.print(" ");
  }
  for (int i = 0; i < 4; i++) {
    Serial.print(raw_co(i), 4);
    Serial.print(" ");
  }
  for (int i = 0; i < 3; i++) {
    Serial.print(TargetPos(i), 4);
    Serial.print(" ");
  }
  Serial.print(millis() / 1000.0);
  Serial.print(" ");
  Serial.print(ci.GND_val);
  Serial.println();

  return co;
}
} // namespace Controller
#include "controller.h"
#include "matlab_funcs.h"

namespace Controller {
t_constantsASTRA constantsASTRA;
Matrix12_12 P;
Vector13 x_est;
Vector3 lastEMA;
Matrix4_12 K;
Matrix9_4 dnf_X;
Matrix9_4 dnf_Y;
float last_thrust;

unsigned long last_call_time; // ms

void begin() {
  reset_controller_state();
}

void reset_controller_state() {
  constantsASTRA.g = 9.80145;
  constantsASTRA.m = 1.249;
  constantsASTRA.mag << -0.4512, 0, 0.8924;
  constantsASTRA.Q << 4.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, //
      0, 4.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0,                 //
      0, 0, 4.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10,                 //
      0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0,                  //
      0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0,                  //
      -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07, 0, 0,                 //
      0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07, 0,                 //
      0, 0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07;                 //
  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.1500;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.1000;

  P = 1 * Matrix12_12::Identity();
  x_est = Vector13::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();

  K << 1.454588e+00, -4.930912e-16, 5.403553e-16, 1.988144e-17, -9.946226e-05, 1.728938e-18, 1.369956e-17, -1.808813e-01, -1.146883e-16, 1.491384e-01, 2.016787e-17, 3.358270e-16,   //
      -1.575943e-15, 1.478460e+00, 6.032135e-16, 9.946226e-05, -1.235488e-17, 9.895805e-19, 1.808819e-01, 1.962303e-16, -1.162821e-16, -8.479662e-17, 1.540745e-01, -1.022005e-17,   //
      1.873785e-14, -2.273709e-13, 6.415369e-15, -1.415555e-17, -4.535712e-17, 4.559014e-03, -1.235028e-14, -1.217727e-14, 3.041216e+00, -1.043676e-15, -1.380587e-14, 2.081769e-15, //
      -4.157755e-16, -3.099593e-15, 8.683836e-01, -3.012412e-18, 2.930859e-19, 9.176150e-19, -3.222563e-16, 2.391674e-16, -6.529443e-17, 3.211746e-17, -2.430887e-16, 9.318711e-01;  //

  dnf_X = Matrix9_4::Ones();
  dnf_Y = Matrix9_4::Ones();
  last_thrust = 0;

  last_call_time = millis();
}

Controller_Output get_controller_output(Controller_Input ci, bool should_log, int traj_idx) {
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

  // Vector9 imu = z.segment<9>(0);
  // Vector9 filt_imu = DigitalNF(imu, ci.GND_val, last_thrust, dT, dnf_X, dnf_Y);
  // z.segment<9>(0) = filt_imu;

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
  last_thrust = raw_co(2);

  if (should_log) {
    Serial.print(">a");
    for (int i = 0; i < 15; i++) {
      Serial.print(z(i), 4);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print(">b");
    Serial.print(x_est(0));
    Serial.print(" ");
    for (int i = 0; i < 15; i++) {
      Serial.print(X(i), 4);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print(">c");
    for (int i = 0; i < 4; i++) {
      Serial.print(raw_co(i), 4);
      Serial.print(" ");
    }
    if (traj_idx == 0) {
      Serial.print("0 0 0.0 ");
    } else if (traj_idx == 1) {
      Serial.print("0 0 0.3 ");
    } else if (traj_idx == 2) {
      Serial.print("0 0 0.3 ");
    } else {
      Serial.print("0 0 0.0 ");
    }
    Serial.println();
  }

  Controller_Output co;
  co.gimbal_yaw_deg = raw_co(0) * 180 / M_PI;
  co.gimbal_pitch_deg = raw_co(1) * 180 / M_PI;
  co.thrust_N = raw_co(2);
  co.roll_rad_sec_squared = raw_co(3);
  return co;
}
} // namespace Controller
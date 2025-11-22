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
  constantsASTRA.m = 1.4; // 1.249;
  constantsASTRA.mag << -0.4512, 0, 0.8924;
  constantsASTRA.Q << 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, //
      0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0,                 //
      0, 0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10,                 //
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
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.100;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.100;

  P = 1 * Matrix12_12::Identity();
  x_est = Vector13::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();

  K << 9.179531e-01, -1.506151e-15, 7.408800e-17, 2.701827e-18, -5.967736e-05, 5.988034e-18, -1.311617e-16, -9.949035e-08, 3.415428e-16, 1.075627e-01, -1.171032e-16, 6.760950e-17, //
      1.925415e-15, 9.342147e-01, 8.412668e-17, 5.967736e-05, -4.789246e-19, 1.560786e-19, 9.949085e-08, -2.805066e-16, -1.359168e-16, 8.318348e-17, 1.114232e-01, 8.326603e-17,    //
      3.700471e-14, 8.492853e-15, 4.653811e-16, 7.931200e-18, 5.236077e-17, 4.559014e-03, 1.601474e-15, -1.083596e-14, 2.279532e+01, 1.665528e-15, 3.067750e-15, 8.421264e-17,      //
      -3.810392e-15, 1.408476e-14, 8.683836e-01, 1.045122e-18, -2.230125e-19, 1.286397e-18, 1.362368e-15, 5.648169e-16, 1.821742e-17, -5.086891e-16, 1.403369e-15, 9.318711e-01;    //

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
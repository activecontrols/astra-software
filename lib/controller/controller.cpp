#include "controller.h"
#include "Router.h"
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
  constantsASTRA.m = 1;
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
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.1500;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.1000;

  P = 1 * Matrix12_12::Identity();
  x_est = Vector13::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();

  K << 1.727316e+00, 1.397551e-15, -4.496957e-16, -2.191563e-17, -9.946226e-05, 2.443746e-18, 1.009896e-16, -1.808889e-01, -9.204288e-17, 2.101464e-01, 9.046160e-17, -2.504290e-16,   //
      -1.001805e-15, 1.752664e+00, -1.979585e-14, 9.946226e-05, -3.640769e-18, 1.341432e-18, 1.808896e-01, 1.090076e-16, 1.485138e-16, -8.495741e-17, 2.163589e-01, -4.800818e-15,     //
      2.068573e-14, -1.808309e-13, -7.941772e-14, -1.782415e-17, -7.805697e-17, 4.559014e-03, -2.450348e-14, -1.100699e-14, 3.040842e+00, -1.390689e-15, -2.931921e-15, -2.405146e-14, //
      -3.030093e-15, 1.483349e-14, 0.7598, 4.278588e-18, -5.374655e-18, 2.685444e-18, 2.056540e-15, 1.574826e-16, 2.541668e-16, -1.544970e-16, 9.234013e-16, 0.871;                    //

  dnf_X = Matrix9_4::Ones();
  dnf_Y = Matrix9_4::Ones();
  last_thrust = 0;

  last_call_time = millis();
}

Controller_Output get_controller_output(Controller_Input ci, bool should_log, bool first_cycle) {
  unsigned long call_time = millis();
  float dT = (call_time - last_call_time) / 1000.0;
  last_call_time = call_time;
  if (dT < 0.001) {
    dT = 0.001;
  }

  Vector15 z;
  // clang-format off
  z << ci.accel_x, ci.accel_y, ci.accel_z, 
       ci.gyro_yaw, ci.gyro_pitch, ci.gyro_roll,
       ci.mag_x, ci.mag_y, ci.mag_z,
       ci.gps_pos_north, ci.gps_pos_west, ci.gps_pos_up, 
       ci.gps_vel_north, ci.gps_vel_west, ci.gps_vel_up;
  // clang-format on

  // Vector9 imu = z.segment<9>(0);
  // Vector9 filt_imu;
  // if (first_cycle) {
  //   filt_imu = DigitalNF(imu, 1.0, last_thrust, dT, dnf_X, dnf_Y);
  // } else {
  //   filt_imu = DigitalNF(imu, 0.0, last_thrust, dT, dnf_X, dnf_Y);
  // }

  // z.segment<9>(0) = filt_imu;
  // Router::printf("%f\n", dT);

  x_est = EstimateStateFCN(x_est, constantsASTRA, z, dT, 1.0, P, ci.new_imu_packet, ci.new_gps_packet);
  Vector3 EMA_G = EMA_Gyros(z, lastEMA);
  Vector15 X = StateAUG(x_est, EMA_G);
  Vector3 TargetPos;
  TargetPos << ci.target_pos_north, ci.target_pos_west, ci.target_pos_up;
  Vector12 error = ref_generator3(X, TargetPos);
  Vector4 raw_co = -K * error;
  raw_co(2) = raw_co(2) + constantsASTRA.g * constantsASTRA.m;
  raw_co = output_clamp(raw_co);
  // if (0.0) {
  //   raw_co = Vector4::Zero();
  // }
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
    for (int i = 0; i < 3; i++) {
      Serial.print(TargetPos(i), 4);
      Serial.print(" ");
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
#include "controller_and_estimator.h"
#include "matlab_funcs.h"

namespace ControllerAndEstimator {
constantsASTRA_t constantsASTRA;
Matrix18_18 P;
Matrix9_9 Flight_P;
Vector19 x_est;
Vector3 lastEMA;
Matrix9_4 dnf_X;
Matrix9_4 dnf_Y;
float last_thrust;
bool last_GND;

unsigned long last_call_time; // ms

void init_controller_and_estimator_constants() {
  constantsASTRA.g = 9.8015;
  constantsASTRA.m = 1.2490;
  constantsASTRA.mag << -0.4512, 0, 0.8924;
  constantsASTRA.Q << 1.000017e-06, 0, 0, 0, 0, 0, 0, 0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, //
      0, 1.000017e-06, 0, 0, 0, 0, 0, 0, 0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0,                 //
      0, 0, 1.000017e-06, 0, 0, 0, 0, 0, 0, 0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0,                 //
      0, 0, 0, 2.083349e-09, 0, 0, 6.250078e-07, 0, 0, 0, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0,      //
      0, 0, 0, 0, 2.083349e-09, 0, 0, 6.250078e-07, 0, 0, 0, 0, 0, -2.083333e-09, 0, 0, 0, 0,      //
      0, 0, 0, 0, 0, 2.083349e-09, 0, 0, 6.250078e-07, 0, 0, 0, 0, 0, -2.083333e-09, 0, 0, 0,      //
      0, 0, 0, 6.250078e-07, 0, 0, 2.500042e-04, 0, 0, 0, 0, 0, -1.250000e-06, 0, 0, 0, 0, 0,      //
      0, 0, 0, 0, 6.250078e-07, 0, 0, 2.500042e-04, 0, 0, 0, 0, 0, -1.250000e-06, 0, 0, 0, 0,      //
      0, 0, 0, 0, 0, 6.250078e-07, 0, 0, 2.500042e-04, 0, 0, 0, 0, 0, -1.250000e-06, 0, 0, 0,      //
      -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 5.000000e-06, 0, 0, 0, 0, 0, 0, 0, 0,                 //
      0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 5.000000e-06, 0, 0, 0, 0, 0, 0, 0,                 //
      0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 5.000000e-06, 0, 0, 0, 0, 0, 0,                 //
      0, 0, 0, -1.250000e-06, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0, 2.500000e-05, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, -1.250000e-06, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0, 2.500000e-05, 0, 0, 0, 0,     //
      0, 0, 0, 0, 0, -1.250000e-06, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0, 2.500000e-05, 0, 0, 0,     //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-06, 0, 0,                             //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-06, 0,                             //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-06;                             //

  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.100;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.100;

  constantsASTRA.K_Att << 8.722500e-01, -1.643694e-16, -8.894669e-16, 1.345039e-01, 7.189700e-19, -7.736332e-17, -4.787136e-01, 3.300943e-16, 4.539530e-16, //
      6.198463e-16, 8.722500e-01, -1.023859e-15, 4.915936e-17, 1.345039e-01, -2.388831e-16, -1.440612e-15, -4.787136e-01, 3.240570e-16,                     //
      -9.959333e-17, 7.261800e-16, 4.035990e+00, -9.370403e-17, 2.685714e-17, 2.059154e+00, 7.242069e-16, 2.974059e-16, -1.581139e+00;                      //

  P = 1 * Matrix18_18::Identity();
  x_est = Vector19::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();
  dnf_X = Matrix9_4::Ones();
  dnf_Y = Matrix9_4::Ones();
  last_thrust = 0;
  last_GND = true;

  ASTRAv2_Controller_reset();

  last_call_time = micros();
}

Controller_Output get_controller_output(Vector15 z, Vector3 TargetPos, bool GND_val, bool new_imu_packet, bool new_gps_packet, float dT) {
  // unsigned long call_time = micros();
  // float dT = (call_time - last_call_time) / 1000000.0;
  // last_call_time = call_time;

  // Vector15 z;
  // // clang-format off
  // z << ci.accel_x, ci.accel_y, ci.accel_z,
  //      ci.gyro_yaw, ci.gyro_pitch, ci.gyro_roll,
  //      ci.mag_x, ci.mag_y, ci.mag_z,
  //      ci.gps_pos_north, ci.gps_pos_west, ci.gps_pos_up,
  //      ci.gps_vel_north, ci.gps_vel_west, ci.gps_vel_up;
  // // clang-format on

  Vector9 imu = z.segment<9>(0);
  Vector9 filt_imu = DigitalNF(imu, GND_val, last_thrust, dT, dnf_X, dnf_Y);
  z.segment<9>(0) = filt_imu;

  if (!GND_val && last_GND) { // we left GND this frame
    Flight_P = P.block<9, 9>(0, 0);
    ASTRAv2_Controller_reset(); // reset integral gains in the controller itself
  }

  if (GND_val) {
    x_est = GroundEstimator(x_est, constantsASTRA, z, dT, P, new_imu_packet, new_gps_packet);
  } else {
    x_est = FlightEstimator(x_est, constantsASTRA, z, dT, Flight_P, new_gps_packet);
  }

  Vector16 X = StateAUG(x_est.segment<13>(0), z.segment<3>(3));
  // Vector3 TargetPos;
  // TargetPos << ci.target_pos_north, ci.target_pos_west, ci.target_pos_up;
  Vector4 raw_co = ASTRAv2_Controller(TargetPos, X, constantsASTRA, dT);
  if (GND_val) {
    raw_co = Vector4::Zero();
  }
  last_thrust = raw_co(2);
  last_GND = GND_val;

  Controller_Output co;
  co.gimbal_yaw_deg = raw_co(0) * 180 / M_PI;
  co.gimbal_pitch_deg = raw_co(1) * 180 / M_PI;
  co.thrust_N = raw_co(2);
  co.roll_rad_sec_squared = raw_co(3);
  return co;
}
} // namespace ControllerAndEstimator
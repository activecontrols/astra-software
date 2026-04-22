#include "controller_and_estimator.h"
#include "matlab_funcs.h"

namespace ControllerAndEstimator {
constantsASTRA_t constantsASTRA;
Matrix18_18 P;
Matrix9_9 Flight_P;
Vector19 x_est;
Vector2 lastEMA;
Matrix9_4 dnf_X;
Matrix9_4 dnf_Y;
float last_thrust;
bool last_GND;
const float tau = 0.03; // seconds - time constant for ema low pass filter applied to gimbal angles

void init_controller_and_estimator_constants() {
  constantsASTRA.g = 9.8015;
  constantsASTRA.m = 1.2490;
  constantsASTRA.mag << 0.38535, 0.03198, -0.9222;
  constantsASTRA.Q = Q_gen();

  constantsASTRA.K_Att << 1.394935e+00, 4.672543e-16, -7.564760e-16, 4.106678e-01, 8.281746e-17, -4.138796e-17, -2.449490e-01, -1.843561e-16, 3.177379e-16, //
      -3.283640e-16, 1.394935e+00, -5.002970e-17, 9.692012e-18, 4.106678e-01, 3.141303e-17, -2.199100e-16, -2.449490e-01, 5.048427e-16,                     //
      7.781684e-16, 4.351920e-16, 6.015416e+00, 8.275175e-17, 1.006071e-16, 2.493901e+00, -9.720773e-17, 1.731782e-16, -1.581139e+00;                       //                 //

  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.050;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.100;

  P = Matrix18_18::Identity();
  P.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.09;
  P.block<3, 3>(9, 9) = Matrix3_3::Identity() * 0.001;
  P.block<3, 3>(12, 12) = Matrix3_3::Identity() * 0.0001;
  P.block<3, 3>(15, 15) = Matrix3_3::Identity() * 0.0001;

  x_est = Vector19::Zero();
  x_est[0] = 1;
  lastEMA = Vector2::Zero();
  dnf_X = Matrix9_4::Ones();
  dnf_Y = Matrix9_4::Ones();
  last_thrust = 0;
  last_GND = true;

  ASTRAv2_Controller_reset();
}

Controller_Output get_controller_output(Controller_Input ci, float ideal_dT, float loop_dT, Controller_Internals *cs) {

  Vector15 z;
  // clang-format off
  z << ci.imu_mag_state.accel_x, ci.imu_mag_state.accel_y, ci.imu_mag_state.accel_z, 
       ci.imu_mag_state.gyro_yaw, ci.imu_mag_state.gyro_pitch, ci.imu_mag_state.gyro_roll,
       ci.imu_mag_state.mag_x, ci.imu_mag_state.mag_y, ci.imu_mag_state.mag_z,
       ci.gps_pos.north, ci.gps_pos.west, ci.gps_pos.up, 
       ci.gps_vel.north, ci.gps_vel.west, ci.gps_vel.up;
  // clang-format on

  Vector9 imu = z.segment<9>(0);
  if (last_thrust < 1) { // prevent div by 0 in filter
    last_thrust = 9.8;
  }
  Vector9 filt_imu = DigitalNF(imu, ci.GND_val, last_thrust, ideal_dT, dnf_X, dnf_Y);
  z.segment<9>(0) = filt_imu;

  for (int i = 0; i < 9; i++) {
    cs->filter_out[i] = z(i);
  }

  if (!ci.GND_val && last_GND) { // we left GND this frame
    Flight_P = P.block<9, 9>(0, 0);
    ASTRAv2_Controller_reset(); // reset integral gains in the controller itself
  }

  // TODO - check row major / column major
  Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> gps_vel_covar((float *)ci.gps_vel_covar);
  Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> gps_pos_covar((float *)ci.gps_pos_covar);

  if (ci.GND_val) {
    x_est = GroundEstimator(x_est, constantsASTRA, z, loop_dT, P, ci.new_imu_packet, ci.new_gps_packet, gps_vel_covar, gps_pos_covar);
  } else {
    x_est = FlightEstimator(x_est, constantsASTRA, z, loop_dT, Flight_P, ci.new_gps_packet, gps_vel_covar, gps_pos_covar);
  }

  Vector16 X = StateAUG(x_est.segment<13>(0), z.segment<3>(3));
  Vector3 TargetPos;
  TargetPos << ci.target_pos_north, ci.target_pos_west, ci.target_pos_up;
  Vector4 raw_co = ASTRAv2_Controller(TargetPos, X, constantsASTRA, loop_dT);
  if (ci.GND_val) {
    raw_co = Vector4::Zero();
  }
  last_thrust = raw_co(2);
  last_GND = ci.GND_val;

  float co_gimbal_yaw = raw_co(0) * 180 / M_PI;
  float co_gimbal_pitch = raw_co(1) * 180 / M_PI;

  float ALPHA = ideal_dT / tau;
  float new_gimbal_yaw = ALPHA * co_gimbal_yaw + (1.0f - ALPHA) * lastEMA(0);
  float new_gimbal_pitch = ALPHA * co_gimbal_pitch + (1.0f - ALPHA) * lastEMA(1);

  lastEMA(0) = new_gimbal_yaw;
  lastEMA(1) = new_gimbal_pitch;

  Controller_Output co;
  co.gimbal_yaw_deg = new_gimbal_yaw;
  co.gimbal_pitch_deg = new_gimbal_pitch;
  co.thrust_N = raw_co(2);
  co.roll_rad_sec_squared = raw_co(3);

  cs->x_est.q_vec_w = x_est(0);
  cs->x_est.q_vec_x = x_est(1);
  cs->x_est.q_vec_y = x_est(2);
  cs->x_est.q_vec_z = x_est(3);
  cs->x_est.est_pos_north = x_est(4);
  cs->x_est.est_pos_west = x_est(5);
  cs->x_est.est_pos_up = x_est(6);
  cs->x_est.est_vel_north = x_est(7);
  cs->x_est.est_vel_west = x_est(8);
  cs->x_est.est_vel_up = x_est(9);
  cs->x_est.gyro_bias_yaw = x_est(10);
  cs->x_est.gyro_bias_pitch = x_est(11);
  cs->x_est.gyro_bias_roll = x_est(12);
  cs->x_est.accel_bias_x = x_est(13);
  cs->x_est.accel_bias_y = x_est(14);
  cs->x_est.accel_bias_z = x_est(15);
  cs->x_est.mag_bias_x = x_est(16);
  cs->x_est.mag_bias_y = x_est(17);
  cs->x_est.mag_bias_z = x_est(18);
  return co;
}
} // namespace ControllerAndEstimator
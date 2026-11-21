#include "controller_and_estimator.h"
#include "matlab_funcs.h"

namespace ControllerAndEstimator {
constantsASTRA_t constantsASTRA;
Matrix15_15 Ground_P;
Matrix15_15 Flight_P;
Vector19 x_est;
Vector2 lastEMA;
Matrix9_4 dnf_X;
Matrix9_4 dnf_Y;
Vector9 xhat_pos;
Vector6 xhat_att;
Vector4 last_U_cmd;
float last_thrust;
bool last_GND;
const float tau = 0.01; // seconds - time constant for ema low pass filter applied to gimbal angles

// Hand off ground attitude/pos/vel/gyro-bias covariance and reset OU mag-bias slack for flight.
void init_flight_covariance() {
  Flight_P = Matrix15_15::Zero();
  Flight_P.block<12, 12>(0, 0) = Ground_P.block<12, 12>(0, 0);
  Flight_P.block<3, 3>(12, 12) = Matrix3_3::Identity() * (0.015f * 0.015f);
  x_est.segment<3>(16).setZero();
}

void init_controller_and_estimator_constants() {
  constantsASTRA.g = 9.8015;
  constantsASTRA.m = 1.1690;
  constantsASTRA.mag << 0.38535, 0.03198, -0.9222;
  constantsASTRA.rTB = 0.26f; // TODO: Check if true
  constantsASTRA.MaxThrust = 16.0f;
  constantsASTRA.J << 0.067f, 0.067f, 0.02f;
  constantsASTRA.K_I = Matrix3_3::Identity();
  constantsASTRA.K_I.diagonal() << 0.15f, 0.15f, 0.05f;
  constantsASTRA.Q = Q_gen();

  // Gain Matrices
  constantsASTRA.K_trans << 7.489237e-01, 0, 0, 1.434829e+00, 0, 0, 0, 7.489237e-01, 0, 0, 1.434829e+00, 0, 0, 0, 2.136482e+00, 0, 0, 2.972797e+00;

  constantsASTRA.K_rot << -4.736094e-01, 0, 0, -2.078351e-01, 0, 0, 0, -4.736094e-01, 0, 0, -2.078351e-01, 0, 0, 0, 4.945653e-01, 0, 0, 2.167021e-01;

  constantsASTRA.L_Thrust << 6.379708e-03, 0, 0, 2.006645e-03, 0, 0, 0, 5.369761e-03, -5.355184e-07, 0, 2.012529e-03, -3.013035e-09, 0, -5.456470e-07, 5.780200e-03, 0, 1.763092e-08, 2.007902e-03,
      2.277628e-04, 2.564385e-09, 0, 1.275894e-02, 0, 0, 0, 1.026969e-03, 2.566898e-06, 0, 1.118423e-02, 4.778304e-08, 0, -3.768781e-07, 3.236476e-04, 0, 3.945702e-08, 1.057488e-02, 7.326955e-04,
      8.160697e-09, 0, 2.032911e-02, -3.069875e-10, 0, 0, 2.895867e-03, 7.240287e-06, 0, 1.556194e-02, 1.274101e-07, 0, -9.800014e-07, 8.521664e-04, 0, 1.045619e-07, 1.393367e-02;

  constantsASTRA.L_Att << 9.278389e-03, 0, 0, 0, 1.106923e-02, 0, 0, 0, 1.106905e-02, 1.074992e-02, 0, 0, 0, 1.530486e-02, 0, 0, 0, 1.521536e-02;

  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.002f;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.050f;

  Ground_P = Matrix15_15::Identity();
  Ground_P.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.05f;
  Ground_P.block<3, 3>(3, 3) = Matrix3_3::Identity() * 1.0f;
  Ground_P.block<3, 3>(6, 6) = Matrix3_3::Identity() * 0.1f;
  Ground_P.block<3, 3>(9, 9) = Matrix3_3::Identity() * 0.001f;
  Ground_P(12, 12) = 0.0004f;
  Ground_P(13, 13) = 0.0004f;
  Ground_P(14, 14) = 0.01f;

  x_est = Vector19::Zero();
  x_est[0] = 1;
  init_flight_covariance();
  lastEMA = Vector2::Zero();
  dnf_X = Matrix9_4::Ones();
  dnf_Y = Matrix9_4::Ones();
  xhat_pos.setZero();
  xhat_att.setZero();
  last_U_cmd.setZero();
  last_thrust = 0;
  last_GND = true;

  ASTRAv2_Controller_reset();
}

Controller_Output get_controller_output(Controller_Input ci, float ideal_dT, float loop_dT, Controller_Internals *cs) {
  Vector15 z;
  // clang-format off
  z << ci.imu.accel_x, ci.imu.accel_y, ci.imu.accel_z, 
       ci.imu.gyro_yaw, ci.imu.gyro_pitch, ci.imu.gyro_roll,
       ci.mag.mag_x, ci.mag.mag_y, ci.mag.mag_z,
       ci.gps.pos.north, ci.gps.pos.west, ci.gps.pos.up, 
       ci.gps.vel.north, ci.gps.vel.west, ci.gps.vel.up;
  // clang-format on

  Vector15 X_trg;
  static_assert(decltype(X_trg)::RowsAtCompileTime == Trajectory::TRAJ_NUM_STATES);
  for (int i = 0; i < Trajectory::TRAJ_NUM_STATES; i++) {
    X_trg(i) = ci.X_trg[i];
  }

  Vector4 U_ff;
  static_assert(decltype(U_ff)::RowsAtCompileTime == Trajectory::TRAJ_NUM_INPUTS);
  for (int i = 0; i < Trajectory::TRAJ_NUM_INPUTS; i++) {
    U_ff(i) = ci.U_ff[i];
  }

  Vector9 imu = z.segment<9>(0);
  if (last_thrust < 1) { // prevent div by 0 in filter
    last_thrust = 9.8;
  }
  Vector9 filt_imu = DigitalNF(imu, ci.GND_val, last_thrust / constantsASTRA.MaxThrust * 100.0, ideal_dT, dnf_X, dnf_Y);
  z.segment<9>(0) = filt_imu;

  for (int i = 0; i < 9; i++) {
    cs->dnf_out[i] = z(i);
  }

  if (!ci.GND_val && last_GND) { // we left GND this frame
    init_flight_covariance();
    ASTRAv2_Controller_reset(); // reset integral gains in the controller itself
  }

  if (ci.GND_val) {
    x_est = GroundEstimator(x_est, constantsASTRA, z, loop_dT, Ground_P, ci.new_mag_packet, ci.new_gps_packet);
  } else {
    x_est = FlightEstimator(x_est, constantsASTRA, z, loop_dT, Flight_P, ci.new_mag_packet, ci.new_gps_packet);
  }

  // CHANGE: Bias corrected rates
  Vector3 omega_body = z.segment<3>(3) - x_est.segment<3>(10);
  Vector15 X_ctrl;
  X_ctrl << x_est.segment<10>(0), omega_body, 0.0f, 0.0f;

  // Run disturbance observers
  Vector6 Dist_LESO = Vector6::Zero();
  if (!ci.GND_val) {
    Vector3 a_dist = LESO_Position(ci.GND_val, X_ctrl, last_U_cmd, constantsASTRA.L_Thrust, constantsASTRA, loop_dT, xhat_pos);
    Vector3 u_dist = LESO_Attitude(ci.GND_val, X_ctrl, last_U_cmd, constantsASTRA.L_Att, constantsASTRA, loop_dT, xhat_att);
    Dist_LESO << a_dist, u_dist;
  }

  for (int i = 0; i < 6; i++) {
    cs->disturbance[i] = Dist_LESO(i);
  }

  // Main control loop
  Vector4 raw_co = ASTRAv2_Controller(ci.GND_val, X_ctrl, X_trg, U_ff, constantsASTRA, Dist_LESO, loop_dT, cs);
  last_U_cmd = raw_co;
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

  // Add the full estimated state
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
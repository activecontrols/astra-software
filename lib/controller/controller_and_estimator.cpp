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

Matrix3_3 get_pos_cov_mtx(GPS_State gps_state) {
  Matrix3_3 pos_cov_mtx;
  pos_cov_mtx(0, 0) = gps_state.posCovNN;
  pos_cov_mtx(0, 1) = -gps_state.posCovNE;
  pos_cov_mtx(0, 2) = -gps_state.posCovND;
  pos_cov_mtx(1, 0) = -gps_state.posCovNE;
  pos_cov_mtx(1, 1) = gps_state.posCovEE;
  pos_cov_mtx(1, 2) = gps_state.posCovED;
  pos_cov_mtx(2, 0) = -gps_state.posCovND;
  pos_cov_mtx(2, 1) = gps_state.posCovED;
  pos_cov_mtx(2, 2) = gps_state.posCovDD;
  return pos_cov_mtx;
}

Matrix3_3 get_vel_cov_mtx(GPS_State gps_state) {
  Matrix3_3 vel_cov_mtx;
  vel_cov_mtx(0, 0) = gps_state.velCovNN;
  vel_cov_mtx(0, 1) = -gps_state.velCovNE;
  vel_cov_mtx(0, 2) = -gps_state.velCovND;
  vel_cov_mtx(1, 0) = -gps_state.velCovNE;
  vel_cov_mtx(1, 1) = gps_state.velCovEE;
  vel_cov_mtx(1, 2) = gps_state.velCovED;
  vel_cov_mtx(2, 0) = -gps_state.velCovND;
  vel_cov_mtx(2, 1) = gps_state.velCovED;
  vel_cov_mtx(2, 2) = gps_state.velCovDD;
  return vel_cov_mtx;
}

void init_controller_and_estimator_constants() {
  constantsASTRA.g = 9.8015;
  constantsASTRA.m = 1.1690;
  constantsASTRA.mag << 0.38535, 0.03198, -0.9222;
  constantsASTRA.Q = Q_gen();

  constantsASTRA.K_Att << 1.728295e+00, -1.540789e-16, -5.757607e-16, 2.476386e-01, 1.840120e-17, 1.974565e-16, -4.714045e-01, -8.981678e-16, 2.808483e-16, //
      -7.665324e-16, 1.728295e+00, -6.661278e-16, -4.321400e-17, 2.476386e-01, -3.558853e-16, 2.782732e-16, -4.714045e-01, 1.755211e-16,                    //
      1.210242e-17, -4.327838e-16, 6.015416e+00, 1.574587e-17, -1.201731e-16, 2.493901e+00, 1.882484e-16, -8.462698e-17, -1.581139e+00;                     //

  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.050;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.100;

  P = Matrix18_18::Identity();
  P.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.1;
  P.block<3, 3>(9, 9) = Matrix3_3::Identity() * 0.001;
  P.block<3, 3>(12, 12) = Matrix3_3::Identity() * 0.00001;
  P.block<3, 3>(15, 15) = Matrix3_3::Identity() * 0.00001;

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
  z << ci.imu.accel_x, ci.imu.accel_y, ci.imu.accel_z, 
       ci.imu.gyro_yaw, ci.imu.gyro_pitch, ci.imu.gyro_roll,
       ci.mag.mag_x, ci.mag.mag_y, ci.mag.mag_z,
       ci.gps.pos.north, ci.gps.pos.west, ci.gps.pos.up, 
       ci.gps.vel.north, ci.gps.vel.west, ci.gps.vel.up;
  // clang-format on

  Vector9 imu = z.segment<9>(0);
  if (last_thrust < 1) { // prevent div by 0 in filter
    last_thrust = 9.8;
  }
  Vector9 filt_imu = DigitalNF(imu, ci.GND_val, last_thrust / 14.7 * 100.0, ideal_dT, dnf_X, dnf_Y);
  z.segment<9>(0) = filt_imu;

  for (int i = 0; i < 9; i++) {
    cs->filter_out[i] = z(i);
  }

  if (!ci.GND_val && last_GND) { // we left GND this frame
    Flight_P = P.block<9, 9>(0, 0);
    ASTRAv2_Controller_reset(); // reset integral gains in the controller itself
  }

  Matrix3_3 gps_vel_covar = get_vel_cov_mtx(ci.gps);
  Matrix3_3 gps_pos_covar = get_pos_cov_mtx(ci.gps);

  if (ci.GND_val) {
    x_est = GroundEstimator(x_est, constantsASTRA, z, loop_dT, P, ci.new_mag_packet, ci.new_gps_packet, gps_vel_covar, gps_pos_covar);
  } else {
    x_est = FlightEstimator(x_est, constantsASTRA, z, loop_dT, Flight_P, ci.new_gps_packet, gps_vel_covar, gps_pos_covar);
  }

  Vector16 X = StateAUG(x_est.segment<13>(0), z.segment<3>(3));
  Vector3 TargetPos;
  TargetPos << ci.target_pos_north, ci.target_pos_west, ci.target_pos_up;
  Vector4 raw_co = ASTRAv2_Controller(TargetPos, X, constantsASTRA, loop_dT, cs);
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
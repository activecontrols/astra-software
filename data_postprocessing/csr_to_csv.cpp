#include "csr_to_csv.h"
#include "controller_and_estimator.h"
#include <stdio.h>
#include "matlab_funcs.h"
#include "TrajectoryLogger.h"

const char SEPARATOR = ',';
bool first_field = true; // keep track of whether this is the first field in the current csv line we are outputting - tells us whether to prepend value with separator or not

void csv_header_log(FILE *out) {
  static const char HEADER[] =
      "accel_x,accel_y,accel_z,gyro_yaw,gyro_pitch,gyro_roll,mag_x,mag_y,mag_z,gps_pos_north,gps_pos_west,gps_pos_up,gps_vel_north,gps_vel_west,gps_vel_up,state_q_vec_w,state_q_vec_x,state_q_"
      "vec_y,state_q_vec_z,state_pos_north,state_pos_west,state_pos_up,state_vel_north,state_vel_west,state_vel_up,gyro_bias_yaw,gyro_bias_pitch,gyro_bias_roll,accel_bias_x,accel_bias_y,accel_"
      "bias_z,mag_bias_x,mag_bias_y,mag_bias_z,gimbal_yaw_raw,gimbal_pitch_raw,thrust_N,roll_rad_sec_squared,target_pos_north,target_pos_west,target_pos_up,elapsed_time,GND_flag,flight_armed,"
      "thrust_perc,diffy_perc,rtk_status,gps_hor_prec,gps_ver_prec,gps_sat_count,filtered_accel_x,filtered_accel_y,filtered_accel_z,filtered_gyro_x,filtered_gyro_y,filtered_gyro_z,filtered_mag_x,"
      "filtered_mag_y,filtered_mag_z,velocity_target_x,velocity_target_y,velocity_target_z,accel_target_x,accel_target_y,accel_target_z,attitude_target[0],attitude_target[1],attitude_target[2],"
      "attitude_target[3],velocity_integrator_x,velocity_integrator_y,velocity_integrator_z,attitude_integrator_x,attitude_integrator_y,attitude_integrator_z,posCovNN,posCovNE,posCovND,posCovEE,posCovED,"
      "posCovDD,velCovNN,velCovNE,velCovND,velCovEE,velCovED,velCovDD\n";
  // generated with gemini
  // subtract 1 from sizeof(HEADER) so we don't output the null terminator
  fwrite(HEADER, sizeof(HEADER) - 1, 1, out);
  return;
}

void csv_field_log(FILE *out, float value) {
  if (!first_field) {
    fputc(SEPARATOR, out);
  }
  fprintf(out, "%.8f", value);
  first_field = false;
  return;
}

void csv_end_line(FILE *out) {
  fputc('\n', out);
  first_field = true;
  return;
}

template <int m, int n>
void csv_log_matrix(FILE* f_csv_out, Eigen::Matrix<float, m, n> mat)
{
  for (int i = 0; i < mat.size(); ++i)
  {
    csv_field_log(f_csv_out, mat(i));
  }

  return;
}

void csv_log_array(FILE* f_csv_out, float* vals, int n)
{
  for (int i = 0; i < n; ++i)
  {
    csv_field_log(f_csv_out, vals[i]);
  }

  return;
}

void csr_to_csv(FILE* f_csv_out, telemetry_packet_t flight_packet, Controller_Internals internals) {
  // function calls generated with gemini
  csv_field_log(f_csv_out, flight_packet.ci.imu.accel_x);
  csv_field_log(f_csv_out, flight_packet.ci.imu.accel_y);
  csv_field_log(f_csv_out, flight_packet.ci.imu.accel_z);
  csv_field_log(f_csv_out, flight_packet.ci.imu.gyro_yaw);
  csv_field_log(f_csv_out, flight_packet.ci.imu.gyro_pitch);
  csv_field_log(f_csv_out, flight_packet.ci.imu.gyro_roll);
  csv_field_log(f_csv_out, flight_packet.ci.mag.mag_x);
  csv_field_log(f_csv_out, flight_packet.ci.mag.mag_y);
  csv_field_log(f_csv_out, flight_packet.ci.mag.mag_z);
  csv_field_log(f_csv_out, flight_packet.ci.gps.pos.north);
  csv_field_log(f_csv_out, flight_packet.ci.gps.pos.west);
  csv_field_log(f_csv_out, flight_packet.ci.gps.pos.up);
  csv_field_log(f_csv_out, flight_packet.ci.gps.vel.north);
  csv_field_log(f_csv_out, flight_packet.ci.gps.vel.west);
  csv_field_log(f_csv_out, flight_packet.ci.gps.vel.up);

  csv_field_log(f_csv_out, flight_packet.x_est.q_vec_w);
  csv_field_log(f_csv_out, flight_packet.x_est.q_vec_x);
  csv_field_log(f_csv_out, flight_packet.x_est.q_vec_y);
  csv_field_log(f_csv_out, flight_packet.x_est.q_vec_z);
  csv_field_log(f_csv_out, flight_packet.x_est.est_pos_north);
  csv_field_log(f_csv_out, flight_packet.x_est.est_pos_west);
  csv_field_log(f_csv_out, flight_packet.x_est.est_pos_up);
  csv_field_log(f_csv_out, flight_packet.x_est.est_vel_north);
  csv_field_log(f_csv_out, flight_packet.x_est.est_vel_west);
  csv_field_log(f_csv_out, flight_packet.x_est.est_vel_up);
  csv_field_log(f_csv_out, flight_packet.x_est.gyro_bias_yaw);
  csv_field_log(f_csv_out, flight_packet.x_est.gyro_bias_pitch);
  csv_field_log(f_csv_out, flight_packet.x_est.gyro_bias_roll);
  csv_field_log(f_csv_out, flight_packet.x_est.accel_bias_x);
  csv_field_log(f_csv_out, flight_packet.x_est.accel_bias_y);
  csv_field_log(f_csv_out, flight_packet.x_est.accel_bias_z);
  csv_field_log(f_csv_out, flight_packet.x_est.mag_bias_x);
  csv_field_log(f_csv_out, flight_packet.x_est.mag_bias_y);
  csv_field_log(f_csv_out, flight_packet.x_est.mag_bias_z);

  csv_field_log(f_csv_out, flight_packet.co.gimbal_yaw_deg);
  csv_field_log(f_csv_out, flight_packet.co.gimbal_pitch_deg);
  csv_field_log(f_csv_out, flight_packet.co.thrust_N);
  csv_field_log(f_csv_out, flight_packet.co.roll_rad_sec_squared);

  csv_field_log(f_csv_out, flight_packet.ci.target_pos_north);
  csv_field_log(f_csv_out, flight_packet.ci.target_pos_west);
  csv_field_log(f_csv_out, flight_packet.ci.target_pos_up);

  csv_field_log(f_csv_out, flight_packet.elapsed_time);
  csv_field_log(f_csv_out, flight_packet.ci.GND_val);
  csv_field_log(f_csv_out, flight_packet.flight_armed);
  csv_field_log(f_csv_out, flight_packet.thrust_perc);
  csv_field_log(f_csv_out, flight_packet.diffy_perc);
  csv_field_log(f_csv_out, flight_packet.rtk_status);
  csv_field_log(f_csv_out, flight_packet.gps_hor_prec);
  csv_field_log(f_csv_out, flight_packet.gps_ver_prec);
  csv_field_log(f_csv_out, flight_packet.gps_sat_count);

  csv_log_array(f_csv_out, internals.filter_out, 9);
  csv_log_array(f_csv_out, internals.VelTarget, 3);
  csv_log_array(f_csv_out, internals.AccelTarget, 3);
  csv_log_array(f_csv_out, internals.TargetAtt, 4);
  csv_log_array(f_csv_out, internals.VelErrorI, 3);
  csv_log_array(f_csv_out, internals.AttErrorI, 3);

  csv_field_log(f_csv_out, flight_packet.ci.gps.posCovNN);
  csv_field_log(f_csv_out, flight_packet.ci.gps.posCovNE);
  csv_field_log(f_csv_out, flight_packet.ci.gps.posCovND);
  csv_field_log(f_csv_out, flight_packet.ci.gps.posCovEE);
  csv_field_log(f_csv_out, flight_packet.ci.gps.posCovED);
  csv_field_log(f_csv_out, flight_packet.ci.gps.posCovDD);
  csv_field_log(f_csv_out, flight_packet.ci.gps.velCovNN);
  csv_field_log(f_csv_out, flight_packet.ci.gps.velCovNE);
  csv_field_log(f_csv_out, flight_packet.ci.gps.velCovND);
  csv_field_log(f_csv_out, flight_packet.ci.gps.velCovEE);
  csv_field_log(f_csv_out, flight_packet.ci.gps.velCovED);
  csv_field_log(f_csv_out, flight_packet.ci.gps.velCovDD);

  

  csv_end_line(f_csv_out);

  return;
}
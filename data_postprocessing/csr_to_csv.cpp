#include "csr_to_csv.h"
#include "controller_and_estimator.h"
#include <stdio.h>
#include "matlab_funcs.h"
#include "TrajectoryLogger.h"

const char SEPARATOR = ',';
bool first_field = true; // keep track of whether this is the first field in the current csv line we are outputting - tells us whether to prepend value with separator or not


std::string header = "";
bool header_complete = false;

std::string output_buf = "";

void csv_field_log(const char col_name[], float value) {
  if (!header_complete)
  {
    header += col_name;
    if (!first_field)
    {
      header += SEPARATOR;
    }
  }

  if (!first_field) {
    output_buf += SEPARATOR;
  }
  static char format_buf[32] {};
  snprintf(format_buf, sizeof(format_buf), "%.8f", value);
  output_buf += format_buf;

  first_field = false;
  return;
}

void csv_end_line(FILE *out) {

  if (!header_complete)
  {
    header_complete = true;
    header += '\n';
    fwrite(header.c_str(), header.size(), 1, out);
  }

  fwrite(output_buf.c_str(), output_buf.size(), 1, out);
  first_field = true;
  output_buf = "";
  return;
}

void csr_to_csv(FILE* f_csv_out, telemetry_packet_t flight_packet, Controller_Internals internals) {

  csv_field_log("accel_x", flight_packet.ci.imu.accel_x);
  csv_field_log("accel_y", flight_packet.ci.imu.accel_y);
  csv_field_log("accel_z", flight_packet.ci.imu.accel_z);

  csv_field_log("gyro_yaw", flight_packet.ci.imu.gyro_yaw);
  csv_field_log("gyro_pitch", flight_packet.ci.imu.gyro_pitch);
  csv_field_log("gyro_roll", flight_packet.ci.imu.gyro_roll);

  csv_field_log("mag_x", flight_packet.ci.mag.mag_x);
  csv_field_log("mag_y", flight_packet.ci.mag.mag_y);
  csv_field_log("mag_z", flight_packet.ci.mag.mag_z);

  csv_field_log("gps_pos_north", flight_packet.ci.gps.pos.north);
  csv_field_log("gps_pos_west", flight_packet.ci.gps.pos.west);
  csv_field_log("gps_pos_up", flight_packet.ci.gps.pos.up);

  csv_field_log("gps_vel_north", flight_packet.ci.gps.vel.north);
  csv_field_log("gps_vel_west", flight_packet.ci.gps.vel.west);
  csv_field_log("gps_vel_up", flight_packet.ci.gps.vel.up);

  csv_field_log("state_q_vec_w", flight_packet.x_est.q_vec_w);
  csv_field_log("state_q_vec_x", flight_packet.x_est.q_vec_x);
  csv_field_log("state_q_vec_y", flight_packet.x_est.q_vec_y);
  csv_field_log("state_q_vec_z", flight_packet.x_est.q_vec_z);

  csv_field_log("state_pos_north", flight_packet.x_est.est_pos_north);
  csv_field_log("state_pos_west", flight_packet.x_est.est_pos_west);
  csv_field_log("state_pos_up", flight_packet.x_est.est_pos_up);

  csv_field_log("state_vel_north", flight_packet.x_est.est_vel_north);
  csv_field_log("state_vel_west", flight_packet.x_est.est_vel_west);
  csv_field_log("state_vel_up", flight_packet.x_est.est_vel_up);

  csv_field_log("gyro_bias_yaw", flight_packet.x_est.gyro_bias_yaw);
  csv_field_log("gyro_bias_pitch", flight_packet.x_est.gyro_bias_pitch);
  csv_field_log("gyro_bias_roll", flight_packet.x_est.gyro_bias_roll);

  csv_field_log("accel_bias_x", flight_packet.x_est.accel_bias_x);
  csv_field_log("accel_bias_y", flight_packet.x_est.accel_bias_y);
  csv_field_log("accel_bias_z", flight_packet.x_est.accel_bias_z);

  csv_field_log("mag_bias_x", flight_packet.x_est.mag_bias_x);
  csv_field_log("mag_bias_y", flight_packet.x_est.mag_bias_y);
  csv_field_log("mag_bias_z", flight_packet.x_est.mag_bias_z);

  csv_field_log("gimbal_yaw_raw", flight_packet.co.gimbal_yaw_deg);
  csv_field_log("gimbal_pitch_raw", flight_packet.co.gimbal_pitch_deg);

  csv_field_log("thrust_N", flight_packet.co.thrust_N);
  csv_field_log("roll_rad_sec_squared", flight_packet.co.roll_rad_sec_squared);

  csv_field_log("target_pos_north", flight_packet.ci.target_pos_north);
  csv_field_log("target_pos_west", flight_packet.ci.target_pos_west);
  csv_field_log("target_pos_up", flight_packet.ci.target_pos_up);
  
  csv_field_log("elapsed_time", flight_packet.elapsed_time);
  csv_field_log("GND_flag", flight_packet.ci.GND_val);
  csv_field_log("flight_armed", flight_packet.flight_armed);
  csv_field_log("thrust_perc", flight_packet.thrust_perc);
  csv_field_log("diffy_perc", flight_packet.diffy_perc);
  csv_field_log("rtk_status", flight_packet.rtk_status);

  csv_field_log("gps_hor_prec", flight_packet.gps_hor_prec);
  csv_field_log("gps_ver_prec", flight_packet.gps_ver_prec);
  csv_field_log("gps_sat_count", flight_packet.gps_sat_count);

  csv_field_log("filtered_accel_x", internals.filter_out[0]);
  csv_field_log("filtered_accel_y", internals.filter_out[1]);
  csv_field_log("filtered_accel_z", internals.filter_out[2]);
  csv_field_log("filtered_gyro_x", internals.filter_out[3]);
  csv_field_log("filtered_gyro_y", internals.filter_out[4]);
  csv_field_log("filtered_gyro_z", internals.filter_out[5]);
  csv_field_log("filtered_mag_x", internals.filter_out[6]);
  csv_field_log("filtered_mag_y", internals.filter_out[7]);
  csv_field_log("filtered_mag_z", internals.filter_out[8]);

  csv_field_log("velocity_target_x", internals.VelTarget[0]);
  csv_field_log("velocity_target_y", internals.VelTarget[1]);
  csv_field_log("velocity_target_z", internals.VelTarget[2]);

  csv_field_log("accel_target_x", internals.AccelTarget[0]);
  csv_field_log("accel_target_y", internals.AccelTarget[1]);
  csv_field_log("accel_target_z", internals.AccelTarget[2]);
  
  csv_field_log("attitude_target[0]", internals.TargetAtt[0]);
  csv_field_log("attitude_target[1]", internals.TargetAtt[1]);
  csv_field_log("attitude_target[2]", internals.TargetAtt[2]);
  csv_field_log("attitude_target[3]", internals.TargetAtt[3]);

  csv_field_log("velocity_integrator_x", internals.VelErrorI[0]);
  csv_field_log("velocity_integrator_y", internals.VelErrorI[1]);
  csv_field_log("velocity_integrator_z", internals.VelErrorI[2]);

  csv_field_log("attitude_integrator_x", internals.AttErrorI[0]);
  csv_field_log("attitude_integrator_y", internals.AttErrorI[1]);
  csv_field_log("attitude_integrator_z", internals.AttErrorI[2]);

  csv_field_log("posCovNN", flight_packet.ci.gps.posCovNN);
  csv_field_log("posCovNE", flight_packet.ci.gps.posCovNE);
  csv_field_log("posCovND", flight_packet.ci.gps.posCovND);
  csv_field_log("posCovEE", flight_packet.ci.gps.posCovEE);
  csv_field_log("posCovED", flight_packet.ci.gps.posCovED);
  csv_field_log("posCovDD", flight_packet.ci.gps.posCovDD);

  csv_field_log("velCovNN", flight_packet.ci.gps.velCovNN);
  csv_field_log("velCovNE", flight_packet.ci.gps.velCovNE);
  csv_field_log("velCovND", flight_packet.ci.gps.velCovND);
  csv_field_log("velCovEE", flight_packet.ci.gps.velCovEE);
  csv_field_log("velCovED", flight_packet.ci.gps.velCovED);
  csv_field_log("velCovDD", flight_packet.ci.gps.velCovDD);

  csv_end_line(f_csv_out);

  return;
}
#define __packed __attribute__((__packed__)) // patch this in
#include "Trajectory.h"
#include "TrajectoryLogger.h"
#include "controller_and_estimator.h"
#include "flight_data.h"
#include <stdio.h>
#include <stdlib.h>

struct IMU_Calib {
  double gyro_bias[3];
  double accel_correction_bias[3];
  double accel_correction_gain[3];
};

struct Mag_Calib {
  float hard_x;
  float hard_y;
  float hard_z;
  float soft[3][3];
};

flight_packet_t fp; // allows this to persist across calls
float last_time;
float this_time;
Controller_Input ci;
traj_point_pos *traj;
Mag_Calib mcal;
IMU_Calib ical;

bool parse_log_entry(FILE *compressed_bin, FILE *reconstructed_bin) {
  uint8_t entry_type;
  size_t did_read = fread(&entry_type, sizeof(uint8_t), 1, compressed_bin);
  if (did_read != sizeof(uint8_t)) {
    return false;
  }

  // printf("Read log entry.\n");

  fp.GND_flag = false;
  fp.flight_armed = true;

  switch (entry_type) {

  case ENTRY_TRAJECTORY: {
    uint32_t len;
    fread(&len, sizeof(uint32_t), 1, compressed_bin);
    traj = (traj_point_pos *)malloc(sizeof(traj_point_pos) * len);
    fread(traj, sizeof(traj_point_pos), len, compressed_bin);
    break;
  }

  case ENTRY_LOOP_STATE: {
    LoopState ls;
    fread(&ls, sizeof(LoopState), 1, compressed_bin);
    this_time = ls.time;

    // start the CI entry
    ci.GND_val = false;
    ci.new_imu_packet = false;
    ci.new_gps_packet = false;
    ci.target_pos_north = traj[ls.phase].north;
    ci.target_pos_west = traj[ls.phase].west;
    ci.target_pos_up = traj[ls.phase].up;
    fp.target_pos_north = ci.target_pos_north;
    fp.target_pos_west = ci.target_pos_west;
    fp.target_pos_up = ci.target_pos_up;
    break;
  }

  case ENTRY_SENSOR: {
    SensorEntry se;
    fread(&se, sizeof(SensorEntry), 1, compressed_bin);

    // TODO - ci should just use this struct
    ci.new_imu_packet = true;
    ci.accel_x = se.accel_x;
    ci.accel_y = se.accel_y;
    ci.accel_z = se.accel_z;
    ci.gyro_yaw = se.gyro_yaw;
    ci.gyro_pitch = se.gyro_pitch;
    ci.gyro_roll = se.gyro_roll;
    ci.mag_x = se.mag_x;
    ci.mag_y = se.mag_y;
    ci.mag_z = se.mag_z;

    // TODO - flight packet should just use these structs
    fp.accel_x = ci.accel_x;
    fp.accel_y = ci.accel_y;
    fp.accel_z = ci.accel_z;
    fp.gyro_yaw = ci.gyro_yaw;
    fp.gyro_pitch = ci.gyro_pitch;
    fp.gyro_roll = ci.gyro_roll;
    fp.mag_x = ci.mag_x;
    fp.mag_y = ci.mag_y;
    fp.mag_z = ci.mag_z;
    break;
  }

  case ENTRY_GPS: {
    GpsEntry gps;
    fread(&gps, sizeof(GpsEntry), 1, compressed_bin);

    ci.new_gps_packet = true;
    ci.gps_pos_north = gps.gps_pos_north;
    ci.gps_pos_west = gps.gps_pos_west;
    ci.gps_pos_up = gps.gps_pos_up;
    ci.gps_vel_north = gps.gps_vel_north;
    ci.gps_vel_west = gps.gps_vel_west;
    ci.gps_vel_up = gps.gps_vel_up;

    // TODO - covariances

    fp.gps_pos_north = ci.gps_pos_north;
    fp.gps_pos_west = ci.gps_pos_west;
    fp.gps_pos_up = ci.gps_pos_up;
    fp.gps_vel_north = ci.gps_vel_north;
    fp.gps_vel_west = ci.gps_vel_west;
    fp.gps_vel_up = ci.gps_vel_up;
    break;
  }

  case ENTRY_CONTROLLER_OUT: {
    Controller_Output logged_co;
    fread(&logged_co, sizeof(Controller_Output), 1, compressed_bin);

    fp.gimbal_yaw_raw = logged_co.gimbal_yaw_deg;
    fp.gimbal_pitch_raw = logged_co.gimbal_pitch_deg;
    fp.thrust_N = logged_co.thrust_N;
    fp.roll_rad_sec_squared = logged_co.roll_rad_sec_squared;

    Controller_State cs;
    float loop_dT = this_time - last_time;
    last_time = this_time;
    float ideal_dT = 1 / 1000.0;

    Controller_Output constructed_co = ControllerAndEstimator::get_controller_output(ci, ideal_dT, loop_dT, &cs);
    // TODO - check constructed_co against real data

    // printf("Logged co: %f %f %f %f\n", logged_co.gimbal_pitch_deg, logged_co.gimbal_yaw_deg, logged_co.thrust_N, logged_co.roll_rad_sec_squared);
    // printf("Constructed co: %f %f %f %f\n", constructed_co.gimbal_pitch_deg, constructed_co.gimbal_yaw_deg, constructed_co.thrust_N, constructed_co.roll_rad_sec_squared);

    fp.state_q_vec_new = cs.state_q_vec_new;
    fp.state_q_vec_0 = cs.state_q_vec_0;
    fp.state_q_vec_1 = cs.state_q_vec_1;
    fp.state_q_vec_2 = cs.state_q_vec_2;
    fp.state_pos_north = cs.state_pos_north;
    fp.state_pos_west = cs.state_pos_west;
    fp.state_pos_up = cs.state_pos_up;
    fp.state_vel_north = cs.state_vel_north;
    fp.state_vel_west = cs.state_vel_west;
    fp.state_vel_up = cs.state_vel_up;
    fp.gyro_bias_yaw = cs.gyro_bias_yaw;
    fp.gyro_bias_pitch = cs.gyro_bias_pitch;
    fp.gyro_bias_roll = cs.gyro_bias_roll;
    fp.accel_bias_x = cs.accel_bias_x;
    fp.accel_bias_y = cs.accel_bias_y;
    fp.accel_bias_z = cs.accel_bias_z;
    fp.mag_bias_x = cs.mag_bias_x;
    fp.mag_bias_y = cs.mag_bias_y;
    fp.mag_bias_z = cs.mag_bias_z;
    fp.elapsed_time = this_time;

    fwrite(&fp, sizeof(fp), 1, reconstructed_bin);
    break;
  }

  case ENTRY_CALIB: { // TODO - use this data
    fread(&ical, sizeof(IMU_Calib), 1, compressed_bin);
    fread(&mcal, sizeof(Mag_Calib), 1, compressed_bin);
    break;
  }

  case ENTRY_X_EST: { // TODO - check x_est for drift
    fread((uint8_t *)(ControllerAndEstimator::x_est.data()), sizeof(ControllerAndEstimator::x_est(0)), ControllerAndEstimator::x_est.size(), compressed_bin);
    break;
  }

  case ENTRY_FLIGHT_P: {
    fread((uint8_t *)(ControllerAndEstimator::Flight_P.data()), sizeof(ControllerAndEstimator::Flight_P(0)), ControllerAndEstimator::Flight_P.size(), compressed_bin);
    break;
  }

  case 0xFF: {
    return false; // end of log reached
  }

  default: {
    printf("Error - invalid entry type.\n");
    break;
  }
  }

  return true;
}

int main(const int argc, const char* argv[]) {
  if (argc < 3)
  {
    printf("Usage:\n./app.exe <infile> <outfile>\n");
    return EXIT_FAILURE;
  }
  FILE *compressed_bin = fopen(argv[1], "rb");
  if (compressed_bin == NULL) {
    printf("Failed to open compressed log dump.");
    return EXIT_FAILURE;
  }

  FILE *reconstructed_bin = fopen(argv[2], "wb");
  if (reconstructed_bin == NULL) {
    fclose(compressed_bin);
    printf("Failed to create reconstructed flight log.");
    return EXIT_FAILURE;
  }

  last_time = 0;
  ControllerAndEstimator::init_controller_and_estimator_constants();

  while (parse_log_entry(compressed_bin, reconstructed_bin)) {
  };

  printf("Finished reading log.");
  fclose(compressed_bin);
  fclose(reconstructed_bin);
  return EXIT_SUCCESS;
}
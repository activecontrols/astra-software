#include "Trajectory.h"
#include "TrajectoryLogger.h"
#include "astra_structs.h"
#include "RollControl.h"
#include "controller_and_estimator.h"
#include <stdio.h>
#include <stdlib.h>
#include "csr_to_csv.h"

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

float last_time;
float this_time;
Controller_Input ci;
traj_point_pos *traj;
Mag_Calib mcal;
IMU_Calib ical;

bool parse_log_entry(FILE *compressed_bin, FILE *reconstructed_bin, FILE* csv_out) {
  uint8_t entry_type;
  size_t did_read = fread(&entry_type, sizeof(uint8_t), 1, compressed_bin);
  if (did_read != sizeof(uint8_t)) {
    return false;
  }

  // printf("Read log entry.\n");


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
    ci.new_mag_packet = false;
    ci.new_gps_packet = false;
    ci.target_pos_north = traj[ls.phase].north;
    ci.target_pos_west = traj[ls.phase].west;
    ci.target_pos_up = traj[ls.phase].up;
    break;
  }

  case ENTRY_IMU: {
    fread(&ci.imu, sizeof(IMU_State), 1, compressed_bin);
    break;
  }

  case ENTRY_MAG: {
    fread(&ci.mag, sizeof(MAG_State), 1, compressed_bin);
    ci.new_mag_packet = true;
    break;
  }

  case ENTRY_GPS: {
    fread(&ci.gps, sizeof(GPS_State), 1, compressed_bin);
    ci.new_gps_packet = true;
    break;
  }

  case ENTRY_CONTROLLER_OUT: {
    Controller_Output logged_co;
    fread(&logged_co, sizeof(Controller_Output), 1, compressed_bin);

    Controller_Internals cs;

    float loop_dT = this_time - last_time;
    last_time = this_time;
    float ideal_dT = 1 / 1000.0;

    Controller_Output constructed_co = ControllerAndEstimator::get_controller_output(ci, ideal_dT, loop_dT, &cs);
    // TODO - check constructed_co against real data

    // printf("Logged co: %f %f %f %f\n", logged_co.gimbal_pitch_deg, logged_co.gimbal_yaw_deg, logged_co.thrust_N, logged_co.roll_rad_sec_squared);
    // printf("Constructed co: %f %f %f %f\n", constructed_co.gimbal_pitch_deg, constructed_co.gimbal_yaw_deg, constructed_co.thrust_N, constructed_co.roll_rad_sec_squared);

    telemetry_packet_t fp;
    fp.ci = ci;
    fp.x_est = cs.x_est;
    fp.co = logged_co;
    fp.elapsed_time = this_time;
    fp.flight_armed = true;
    get_prop_perc(fp.co.thrust_N, fp.co.roll_rad_sec_squared, &fp.thrust_perc, &fp.diffy_perc); // use thrust table to find these values

    fwrite(&fp, sizeof(fp), 1, reconstructed_bin);

    if (csv_out)
    {
      csr_to_csv(csv_out, fp, cs);
    }
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

  FILE* csv_out = nullptr;

  if (argc == 4)
  {
    csv_out = fopen(argv[3], "wb");

    if (csv_out == NULL)
    {
      fclose(compressed_bin);
      fclose(reconstructed_bin);
      printf("Failed to create csv output file.");
      return EXIT_FAILURE;
    }
  }

  last_time = 0;
  ControllerAndEstimator::init_controller_and_estimator_constants();
  
  while (parse_log_entry(compressed_bin, reconstructed_bin, csv_out)) {
  };

  printf("Finished reading log.");

  fclose(compressed_bin);
  fclose(reconstructed_bin);

  if (csv_out != NULL)
  {
    fclose(csv_out);
  }

  return EXIT_SUCCESS;
}
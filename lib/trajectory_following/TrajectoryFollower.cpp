#include "TrajectoryFollower.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "FlashLogging.h"
#include "GPS.h"
#include "GimbalServos.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "SDCard.h"
#include "TrajectoryLoader.h"
#include "TrajectoryLogger.h"
#include "controller_and_estimator.h"
#include "elapsedMillis.h"
#include "flight_packet.h"
#include <Arduino.h>

#define TELEMETRY_INTERVAL_US 100000
#define COMMAND_INTERVAL_US 1000

namespace TrajectoryFollower {

bool kill_flag;
bool arm_flag;

void follow_trajectory() {
  bool has_left_ground = false;
  bool flight_armed = false;
  kill_flag = false;
  arm_flag = false;

  long counter = 0;

  GimbalServos::centerGimbal();
  Mag::beginMeasurement();

  Point last_gps_pos = {-1, -1, -1}; // first packet will be marked as new
  ControllerAndEstimator::init_controller_and_estimator_constants();
  Controller_State cs;
  flight_packet_t fp;

  while (!Mag::isMeasurementReady()) {
    CommsSerial.println("Waiting on mag...");
    delay(100);
  }
  while (!GPS::has_valid_recent_pos()) {
    GPS::pump_events();
    CommsSerial.println("Waiting on gps...");
    delay(100);
  }

  GPS::set_current_position_as_origin();

  TrajectoryLogger::log_calib_flash();

  elapsedMicros timer = elapsedMicros();
  unsigned long lasttelemetry = timer;
  unsigned long lastloop = timer;

  float last_time_s = timer / 1000000.0;
  double mx, my, mz;

  for (int i = 0; i < TrajectoryLoader::header.num_points; i++) {
    while (last_time_s < TrajectoryLoader::trajectory[i].time || !flight_armed) {
      while (CommsSerial.available()) {
        CommandRouter::receive_byte(CommsSerial.read());
      }

      if (kill_flag) {
        break;
      }

      if (arm_flag) {
        arm_flag = false; // only do this on rising edge
        flight_armed = true;
        timer = elapsedMicros();
        last_time_s = timer / 1000000.0;
        lasttelemetry = timer;
        lastloop = timer;
        counter = 0;
        GPS::set_current_position_as_origin();
      }

      Controller_Input ci;

      has_left_ground = has_left_ground | (TrajectoryLoader::trajectory[i].up > 0);

      IMU::Data imu_reading;
      IMU::IMUs[0].read_latest(&imu_reading);

      if (Mag::isMeasurementReady()) {
        ci.new_imu_packet = true;
        Mag::read_xyz_normalized(mx, my, mz);
        Mag::beginMeasurement();
      } else {
        ci.new_imu_packet = false;
      }

      GPS::pump_events();
      Point gps_rel_pos = GPS::get_rel_xyz_pos();
      GPS_Velocity gps_vel = GPS::get_velocity();

      ci.accel_x = -imu_reading.acc[2];
      ci.accel_y = imu_reading.acc[1];
      ci.accel_z = imu_reading.acc[0];
      ci.gyro_yaw = -imu_reading.gyro[2];
      ci.gyro_pitch = imu_reading.gyro[1];
      ci.gyro_roll = imu_reading.gyro[0];
      ci.mag_x = -mz;
      ci.mag_y = my;
      ci.mag_z = mx;
      ci.gps_pos_north = gps_rel_pos.north;
      ci.gps_pos_west = gps_rel_pos.west;
      ci.gps_pos_up = gps_rel_pos.up;
      ci.gps_vel_north = gps_vel.north;
      ci.gps_vel_west = gps_vel.west;
      ci.gps_vel_up = gps_vel.up;

      ci.target_pos_north = TrajectoryLoader::trajectory[i].north;
      ci.target_pos_west = TrajectoryLoader::trajectory[i].west;
      ci.target_pos_up = TrajectoryLoader::trajectory[i].up;

      if (gps_rel_pos.north != last_gps_pos.north || gps_rel_pos.west != last_gps_pos.west || gps_rel_pos.up != last_gps_pos.up) {
        ci.new_gps_packet = true;
        last_gps_pos = gps_rel_pos;
      } else {
        ci.new_gps_packet = false;
      }

      ci.GND_val = !has_left_ground;

      float time_s = timer / 1000000.0;
      float dT = time_s - last_time_s;

      Controller_Output co = ControllerAndEstimator::get_controller_output(ci, dT, &cs);
      float thrust_perc;
      float diffy_perc;
      Prop::get_prop_perc(co.thrust_N, co.roll_rad_sec_squared, &thrust_perc, &diffy_perc);

      if (flight_armed) {
        // Prop::set_throttle_roll(thrust_perc, diffy_perc); // TODO - re-enable this
        // GimbalServos::setGimbalAngle(co.gimbal_yaw_deg, -co.gimbal_pitch_deg);
      }

      TrajectoryLogger::log_trajectory_flash(timer, i, ci, co);

      if (timer - lasttelemetry > TELEMETRY_INTERVAL_US) {
        lasttelemetry = timer;

        fp.accel_x = cs.filter_out[0];
        fp.accel_y = cs.filter_out[1];
        fp.accel_z = cs.filter_out[2];
        fp.gyro_yaw = cs.filter_out[3];
        fp.gyro_pitch = cs.filter_out[4];
        fp.gyro_roll = cs.filter_out[5];
        fp.mag_x = cs.filter_out[6];
        fp.mag_y = cs.filter_out[7];
        fp.mag_z = cs.filter_out[8];
        fp.gps_pos_north = ci.gps_pos_north;
        fp.gps_pos_west = ci.gps_pos_west;
        fp.gps_pos_up = ci.gps_pos_up;
        fp.gps_vel_north = ci.gps_vel_north;
        fp.gps_vel_west = ci.gps_vel_west;
        fp.gps_vel_up = ci.gps_vel_up;

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

        fp.gimbal_yaw_raw = co.gimbal_yaw_deg;
        fp.gimbal_pitch_raw = co.gimbal_pitch_deg;
        fp.thrust_N = co.thrust_N;
        fp.roll_rad_sec_squared = co.roll_rad_sec_squared;

        fp.target_pos_north = ci.target_pos_north;
        fp.target_pos_west = ci.target_pos_west;
        fp.target_pos_up = ci.target_pos_up;

        fp.elapsed_time = timer / 1000000.0;
        fp.GND_flag = ci.GND_val;
        fp.flight_armed = flight_armed;
        fp.thrust_perc = thrust_perc;
        fp.diffy_perc = diffy_perc;
        fp.rtk_status = (GPS::ubx.pvt_solution.data->flags >> 6) & 0b11;

        CommandRouter::send_command("tr", fp);
      }
      counter++;

      unsigned long target_slp = COMMAND_INTERVAL_US - (timer - lastloop);
      delayMicroseconds(target_slp < COMMAND_INTERVAL_US ? target_slp : 0); // don't delay for too long
      lastloop += COMMAND_INTERVAL_US;
      last_time_s = time_s;
    }
    if (kill_flag) {
      break;
    }
  }

  Prop::stop();

  fp.flight_armed = false;
  CommandRouter::send_command("tr", fp);

  CommsSerial.printf("Finished %ld loop iterations.", counter);
}

// add relevant router cmds
void begin() {
  CommandRouter::add(start_flight_loop, "start_flight_loop");
  CommandRouter::add_flag(&kill_flag, "k", "terminate the flight loop early");
  CommandRouter::add_flag(&arm_flag, "arm", "start following a trajectory");
}

// prompt user for log file name, then follow trajectory
void start_flight_loop() {
  if (!TrajectoryLoader::loaded_trajectory) {
    CommsSerial.println("FAILURE: no trajectory loaded.");
    return;
  }

  // TODO - re-enable this!
  // if (!Logging::is_armed()) {
  //   Router::println("FAILURE: Flash logging is not armed. Use command log_arm.");
  //   return;
  // }

  CommsSerial.println("Flight loop starting. Type arm to start following trajectory.");
  follow_trajectory();
  CommsSerial.println("Finished following trajectory!");
}

} // namespace TrajectoryFollower
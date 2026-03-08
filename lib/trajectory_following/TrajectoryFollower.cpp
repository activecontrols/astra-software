#include "TrajectoryFollower.h"

#include "Arduino.h"
#include "FlashLogging.h"
#include "FlightCommands.h"
#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include "SDCard.h"
#include "TrajectoryLoader.h"
#include "TrajectoryLogger.h"
#include "controller_and_estimator.h"
#include "elapsedMillis.h"
#include "gimbal_servos.h"

#define TELEMETRY_INTERVAL_US 50000
#define COMMAND_INTERVAL_US 1000

namespace TrajectoryFollower {

/**
 * Follows a trajectory by interpolating between position values.
 */
void follow_trajectory() {
  bool has_left_ground = false;
  bool flight_armed = false;
  bool send_telemetry = false;

  long counter = 0;

  GimbalServos::centerGimbal();
  Mag::beginMeasurement();

  Point last_gps_pos = {-1, -1, -1}; // first packet will be marked as new
  ControllerAndEstimator::init_controller_and_estimator_constants();
  FlightCommands::reset();
  Controller_State cs;

  while (!Mag::isMeasurementReady()) {
    Router::println("Waiting on mag...");
    delay(100);
  }
  while (!GPS::has_valid_recent_pos()) {
    GPS::pump_events();
    Router::println("Waiting on gps...");
    delay(100);
  }

  GPS::set_current_position_as_origin();

  TrajectoryLogger::log_calib_flash();

  elapsedMicros timer = elapsedMicros();
  unsigned long lasttelemetry = timer;
  unsigned long lastloop = timer;

  float last_time_s = timer / 1000000.0;

  for (int i = 0; i < TrajectoryLoader::header.num_points; i++) {
    while (last_time_s < TrajectoryLoader::trajectory[i].time || !flight_armed) {
      while (Router::available()) {
        FlightCommands::encode(Router::read());
      }
      if (FlightCommands::kill_flag) {
        break;
      }
      if (FlightCommands::arm_flag) {
        FlightCommands::arm_flag = false; // only do this on rising edge
        flight_armed = true;
        timer = elapsedMicros();
        lasttelemetry = timer;
        lastloop = timer;
        counter = 0;
        GPS::set_current_position_as_origin();
      }

      if (timer - lasttelemetry > TELEMETRY_INTERVAL_US) {
        lasttelemetry = timer;
        send_telemetry = true;
      } else {
        send_telemetry = false;
      }

      Controller_Input ci;

      has_left_ground = has_left_ground | (TrajectoryLoader::trajectory[i].up > 0);

      IMU::Data imu_reading;
      double mx, my, mz;
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
      ci.mag_x = mx;
      ci.mag_y = my;
      ci.mag_z = -mz;
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
      // Prop::get_prop_perc(co.thrust_N, co.roll_rad_sec_squared, &thrust_perc, &diffy_perc);
      // Prop::set_throttle_roll(thrust_perc, diffy_perc);
      // GimbalServos::setGimbalAngle(-co.gimbal_yaw_deg, co.gimbal_pitch_deg);

      TrajectoryLogger::log_trajectory_flash(timer, i, ci, co);

      if (send_telemetry) {
        flight_packet_t fp;

        fp.accel_x = ci.accel_x;
        fp.accel_y = ci.accel_y;
        fp.accel_z = ci.accel_z;
        fp.gyro_yaw = ci.gyro_yaw;
        fp.gyro_pitch = ci.gyro_pitch;
        fp.gyro_roll = ci.gyro_roll;
        fp.mag_x = ci.mag_x;
        fp.mag_y = ci.mag_y;
        fp.mag_z = ci.mag_z;
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

        fp.gimbal_yaw_raw = co.gimbal_yaw_deg;
        fp.gimbal_pitch_raw = co.gimbal_pitch_deg;
        fp.thrust_N = co.thrust_N;
        fp.roll_N = co.roll_rad_sec_squared;

        fp.target_pos_north = ci.target_pos_north;
        fp.target_pos_west = ci.target_pos_west;
        fp.target_pos_up = ci.target_pos_up;

        FlightCommands::send_telemetry(fp);
      }
      counter++;

      unsigned long target_slp = COMMAND_INTERVAL_US - (timer - lastloop);
      delayMicroseconds(target_slp < COMMAND_INTERVAL_US ? target_slp : 0); // don't delay for too long
      lastloop += COMMAND_INTERVAL_US;
      last_time_s = time_s;
    }
    if (FlightCommands::kill_flag) {
      break;
    }
  }

  Prop::stop();

  // TODO - log to indicate flight disarm

  Router::print("Finished ");
  Router::print(counter);
  Router::println(" loop iterations.");
}

// add relevant router cmds
void begin() {
  Router::add({arm, "arm"});
}

// prompt user for log file name, then follow trajectory
void arm(const char *) {
  if (!TrajectoryLoader::loaded_trajectory) {
    Router::println("ARMING FAILURE: no trajectory loaded.");
    return;
  }

  // TODO - re-enable this!
  // if (!Logging::is_armed()) {
  //   Router::println("ARMING FAILURE: Flash logging is not armed. Use command log_arm.");
  //   return;
  // }

  // filenames use DOS 8.3 standard
  // Router::print("Enter log filename (1-8 chars + '.' + 3 chars): ");
  // char *log_file_name = Router::readline();
  // TrajectoryLogger::create_trajectory_log(log_file_name); // lower case files have issues on teensy

  Router::print("ARMING COMPLETE. Type `y` and press enter to confirm. ");

  follow_trajectory();

  Router::println("Finished following trajectory!");
  // TrajectoryLogger::close_trajectory_log();
}

} // namespace TrajectoryFollower
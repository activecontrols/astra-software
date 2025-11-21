#include "TrajectoryFollower.h"

#include "Arduino.h"
#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include "SDCard.h"
#include "TrajectoryLoader.h"
#include "TrajectoryLogger.h"
#include "controller.h"
#include "elapsedMillis.h"
#include "gimbal_servos.h"

#define LOG_INTERVAL_US 5000
#define COMMAND_INTERVAL_US 1000

namespace TrajectoryFollower {

/**
 * Follows a trajectory by interpolating between position values.
 */
void follow_trajectory() {
  elapsedMicros timer = elapsedMicros();
  unsigned long lastlog = timer;
  unsigned long lastloop = timer;
  bool has_left_ground = false;

  long counter = 0;

  GimbalServos::centerGimbal();
  Mag::beginMeasurement();

  Point last_gps_pos = {-1, -1, -1}; // first packet will be marked as new
  Controller::reset_controller_state();

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

  for (int i = 0; i < TrajectoryLoader::header.num_points; i++) {
    while (timer / 1000000.0 < TrajectoryLoader::trajectory[i].time) {
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

      ci.accel_x = imu_reading.acc[0];
      ci.accel_y = -imu_reading.acc[2];
      ci.accel_z = imu_reading.acc[1];
      ci.gyro_yaw = imu_reading.gyro[0];
      ci.gyro_pitch = -imu_reading.gyro[2];
      ci.gyro_roll = imu_reading.gyro[1];
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

      if (has_left_ground) {
        ci.GND_val = 0.0;
      } else {
        ci.GND_val = 1.0;
      }

      Controller_Output co = Controller::get_controller_output(ci);

      Prop::set_throttle_roll(co.thrust_N, co.roll_accel); // TODO - need to check/fix set units
      GimbalServos::setGimbalAngle(co.gimbal_yaw_deg, co.gimbal_pitch_deg);

      if (timer - lastlog > LOG_INTERVAL_US) {
        lastlog += LOG_INTERVAL_US;
        TrajectoryLogger::log_trajectory_csv(timer / 1000000.0, i, ci, co);
      }
      counter++;

      unsigned long target_slp = COMMAND_INTERVAL_US - (timer - lastloop);
      delayMicroseconds(target_slp < COMMAND_INTERVAL_US ? target_slp : 0); // don't delay for too long
      lastloop += COMMAND_INTERVAL_US;
    }
  }

  Prop::stop();

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

  // filenames use DOS 8.3 standard
  Router::print("Enter log filename (1-8 chars + '.' + 3 chars): ");
  String log_file_name = Router::read(50);
  TrajectoryLogger::create_trajectory_log(log_file_name.c_str()); // lower case files have issues on teensy

  Router::print("ARMING COMPLETE. Type `y` and press enter to confirm. ");
  String final_check_str = Router::read(50);
  if (final_check_str != "y") {
    Router::println("ARMING FAILURE: Cancelled by operator.");
    TrajectoryLogger::close_trajectory_log();
    return;
  }

  follow_trajectory();

  Router::println("Finished following trajectory!");
  TrajectoryLogger::close_trajectory_log();
}

} // namespace TrajectoryFollower
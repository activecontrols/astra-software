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
#include "astra_structs.h"
#include "controller_and_estimator.h"
#include "elapsedMillis.h"
#include <Arduino.h>
#include "fc_pins.h"

#define TELEMETRY_INTERVAL_US 100000
#define COMMAND_INTERVAL_US 1000
#define LOG_X_EST_INTERVAL_US 100000

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

  GPS_Point last_gps_pos = {-1, -1, -1}; // first packet will be marked as new
  ControllerAndEstimator::init_controller_and_estimator_constants();
  Controller_Internals cs;
  telemetry_packet_t fp;

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
  TrajectoryLogger::log_trajectory();

  elapsedMicros timer = elapsedMicros();
  unsigned long lasttelemetry = timer;
  unsigned long lastloop = timer;
  unsigned long lastlogx_est = timer;
  unsigned long led_on_time = 0;
  bool led_on = false;

  float last_time_s = timer / 1000000.0;
  float mx, my, mz;

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
        lastlogx_est = timer;
        counter = 0;
        GPS::set_current_position_as_origin();

        TrajectoryLogger::log_x_est(); // only log initial controller state, this is too much data to log every frame

        led_on_time = millis();
        led_on = true;
        digitalWrite(PIN_TEST_LED, LOW);
      }

      if (led_on && millis() - led_on_time > 500)
      {
        digitalWrite(PIN_TEST_LED, HIGH);
      }

      Controller_Input ci;

      has_left_ground = has_left_ground | (TrajectoryLoader::trajectory[i].up > 0);

      IMU::Data imu_reading;
      IMU::IMUs[0].read_latest(&imu_reading);

      if (Mag::isMeasurementReady()) {
        ci.new_imu_packet = true;
        Mag::read_xyz_calibrated(mx, my, mz);
        Mag::beginMeasurement();
      } else {
        ci.new_imu_packet = false;
      }

      GPS::pump_events();
      GPS_Point gps_rel_pos = GPS::get_rel_xyz_pos();
      GPS_Velocity gps_vel = GPS::get_velocity();

      GPS::get_pos_cov(ci.gps_pos_covar);
      GPS::get_vel_cov(ci.gps_vel_covar);

      ci.imu_mag_state.accel_x = -imu_reading.acc[2];
      ci.imu_mag_state.accel_y = imu_reading.acc[1];
      ci.imu_mag_state.accel_z = imu_reading.acc[0];
      ci.imu_mag_state.gyro_yaw = -imu_reading.gyro[2];
      ci.imu_mag_state.gyro_pitch = imu_reading.gyro[1];
      ci.imu_mag_state.gyro_roll = imu_reading.gyro[0];
      ci.imu_mag_state.mag_x = -mz;
      ci.imu_mag_state.mag_y = my;
      ci.imu_mag_state.mag_z = mx;
      ci.gps_pos = gps_rel_pos;
      ci.gps_vel = gps_vel;

      ci.target_pos_north = TrajectoryLoader::trajectory[i].north;
      ci.target_pos_west = TrajectoryLoader::trajectory[i].west;
      ci.target_pos_up = TrajectoryLoader::trajectory[i].up;

      ci.GND_val = !has_left_ground;

      if (ci.GND_val) { // GPS lock
        ci.gps_pos.north = 0;
        ci.gps_pos.west = 0;
        ci.gps_pos.up = 0.31;
        ci.gps_vel.north = 0;
        ci.gps_vel.west = 0;
        ci.gps_vel.up = 0;
        ci.new_gps_packet = true;
      } else if (gps_rel_pos.north != last_gps_pos.north || gps_rel_pos.west != last_gps_pos.west || gps_rel_pos.up != last_gps_pos.up) {
        ci.new_gps_packet = true;
        last_gps_pos = gps_rel_pos;
      } else {
        ci.new_gps_packet = false;
      }

      float time_s = timer / 1000000.0;
      float loop_dT = time_s - last_time_s;
      float ideal_dT = COMMAND_INTERVAL_US / 1000000.0;

      Controller_Output co = ControllerAndEstimator::get_controller_output(ci, ideal_dT, loop_dT, &cs);
      float thrust_perc;
      float diffy_perc;
      Prop::get_prop_perc(co.thrust_N, co.roll_rad_sec_squared, &thrust_perc, &diffy_perc);
      if (flight_armed) {
        Prop::set_throttle_roll(thrust_perc, diffy_perc);
        GimbalServos::setGimbalAngle(co.gimbal_pitch_deg, co.gimbal_yaw_deg);
      }

      if (flight_armed) // we only want to log flight data, not pre-flight
      {
        TrajectoryLogger::flash_log_sensor(time_s, i, ci, co);

        if (timer - lastlogx_est > LOG_X_EST_INTERVAL_US) {
          TrajectoryLogger::log_x_est();
          lastlogx_est = timer;
        }
      }

      if (timer - lasttelemetry > TELEMETRY_INTERVAL_US) {
        lasttelemetry = timer;

        fp.imu_mag_state.accel_x = cs.filter_out[0];
        fp.imu_mag_state.accel_y = cs.filter_out[1];
        fp.imu_mag_state.accel_z = cs.filter_out[2];
        fp.imu_mag_state.gyro_yaw = cs.filter_out[3];
        fp.imu_mag_state.gyro_pitch = cs.filter_out[4];
        fp.imu_mag_state.gyro_roll = cs.filter_out[5];
        fp.imu_mag_state.mag_x = cs.filter_out[6];
        fp.imu_mag_state.mag_y = cs.filter_out[7];
        fp.imu_mag_state.mag_z = cs.filter_out[8];
        fp.gps_pos = ci.gps_pos;
        fp.gps_vel = ci.gps_vel;

        fp.x_est = cs.x_est;
        fp.co = co;

        fp.target_pos_north = ci.target_pos_north;
        fp.target_pos_west = ci.target_pos_west;
        fp.target_pos_up = ci.target_pos_up;

        fp.elapsed_time = timer / 1000000.0;
        fp.GND_flag = ci.GND_val;
        fp.flight_armed = flight_armed;
        fp.thrust_perc = thrust_perc;
        fp.diffy_perc = diffy_perc;
        fp.rtk_status = (GPS::ubx.pvt_solution.data->flags >> 6) & 0b11;
        float hor, ver;
        int sats;
        GPS::get_gps_precision(&hor, &ver, &sats);
        fp.gps_hor_prec = hor;
        fp.gps_ver_prec = ver;
        fp.gps_sat_count = sats;

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

  digitalWrite(PIN_TEST_LED, HIGH);

  Logging::disarm();

  Prop::stop();

  fp.flight_armed = false;
  CommandRouter::send_command("tr", fp);

  CommsSerial.printf("Finished %ld loop iterations.\n", counter);
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

  if (!Logging::is_armed()) {
    CommsSerial.println("FAILURE: Flash logging is not armed. Use command log_arm.");
    return;
  }

  CommsSerial.println("Flight loop starting. Type arm to start following trajectory.");
  follow_trajectory();
  CommsSerial.println("Finished following trajectory!");
}

} // namespace TrajectoryFollower
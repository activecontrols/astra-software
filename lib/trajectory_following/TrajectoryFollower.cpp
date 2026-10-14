#include "TrajectoryFollower.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "FlashLogging.h"
#include "GPS.h"
#include "GimbalServos.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "RollControl.h"
#include "SDCard.h"
#include "TrajectoryLoader.h"
#include "TrajectoryLogger.h"
#include "astra_structs.h"
#include "controller_and_estimator.h"
#include "elapsedMillis.h"
#include "fc_pins.h"
#include <Arduino.h>

#define TELEMETRY_INTERVAL_US 100000
#define COMMAND_INTERVAL_US 1000
#define LOG_X_EST_INTERVAL_US 100000
#define HB_KILL_INTERVAL_MS 500 // if we don't see a heartbeat command every 0.5 seconds during flight, disable gimbal and props

namespace TrajectoryFollower {

bool kill_flag;
bool arm_flag;
bool auto_land_cmd;
unsigned long last_hb;
elapsedMicros timer;

void follow_trajectory() {
  bool has_left_ground = false;
  bool flight_armed = false;
  kill_flag = false;
  arm_flag = false;
  auto_land_cmd = false;

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
  // TODO: Check logging with new stack
  TrajectoryLogger::log_trajectory();

  timer = elapsedMicros();
  unsigned long lasttelemetry = timer;
  unsigned long lastloop = timer;
  unsigned long lastlogx_est = timer;
  unsigned long led_on_time = 0;
  bool led_on = false;
  bool auto_land = false;
  last_hb = timer;

  float last_time_s = timer / 1000000.0;
  float mx, my, mz;

  float traj_start_time_s = 0.0f;
  float traj_duration_s = TrajectoryLoader::nodes[TrajectoryLoader::NumNodes - 1].Time;
  bool in_flight = false;

  while (!kill_flag) {
    while (CommsSerial.available()) {
      CommandRouter::receive_byte(CommsSerial.read());
    }

    if (!auto_land && (auto_land_cmd || ((timer - last_hb) / 1000.0 > HB_KILL_INTERVAL_MS))) {
      if (flight_armed) {
        auto_land_cmd = false;
        CommsSerial.println("Autoland sequence activated!");
        auto_land = true;
      } else {
        CommsSerial.println("Radio disconnected pre-flight");
        kill_flag = true;
      }
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
      last_hb = timer;
      counter = 0;
      GPS::set_current_position_as_origin();

      TrajectoryLogger::log_x_est();
      TrajectoryLogger::log_flight_p(); // only log initial controller state, this is too much data to log every frame

      led_on_time = millis();
      led_on = true;
      digitalWrite(PIN_TEST_LED, LOW);
    }

    if (led_on && millis() - led_on_time > 500) {
      digitalWrite(PIN_TEST_LED, HIGH);
    }

    float time_s = timer / 1000000.0;
    float loop_dT = time_s - last_time_s;
    float ideal_dT = COMMAND_INTERVAL_US / 1000000.0;

    // Liftoff edge detection & flight timer
    if (flight_armed && !in_flight) {
      traj_start_time_s = time_s;
      in_flight = true;
      has_left_ground = true;
    }

    float t_flight = in_flight ? (time_s - traj_start_time_s) : 0.0f;

    // Trajectory end completion check
    if (in_flight && (t_flight >= traj_duration_s + 1.0f)) {
      CommsSerial.println("Trajectory completed successfully.");
      break;
    }

    Controller_Input ci;
    ci.GND_val = !in_flight;

    Vector15 X_trg;
    Vector4 U_ff;
    TrajectoryLoader::get_target(ci.GND_val, t_flight, X_trg, U_ff);

    ci.target_pos_north = X_trg(4);
    ci.target_pos_west = X_trg(5);
    ci.target_pos_up = X_trg(6);

    IMU::Data imu_reading;
    IMU::IMUs[0].read_latest(&imu_reading);

    if (Mag::isMeasurementReady()) {
      ci.new_mag_packet = true;
      Mag::read_xyz_calibrated(mx, my, mz);
      Mag::beginMeasurement();
    } else {
      ci.new_mag_packet = false;
    }

    GPS::pump_events();
    GPS_Point gps_rel_pos = GPS::get_rel_xyz_pos();
    GPS_Velocity gps_vel = GPS::get_velocity();
    GPS::fill_covariances(&ci.gps);

    ci.imu.accel_x = -imu_reading.acc[2];
    ci.imu.accel_y = imu_reading.acc[1];
    ci.imu.accel_z = imu_reading.acc[0];
    ci.imu.gyro_yaw = -imu_reading.gyro[2];
    ci.imu.gyro_pitch = imu_reading.gyro[1];
    ci.imu.gyro_roll = imu_reading.gyro[0];
    ci.mag.mag_x = -mz;
    ci.mag.mag_y = my;
    ci.mag.mag_z = mx;
    ci.gps.pos = gps_rel_pos;
    ci.gps.vel = gps_vel;

    if (ci.GND_val) { // GPS lock
      ci.gps.pos = {north : 0, west : 0, up : 0.31};
      ci.gps.vel = {north : 0, west : 0, up : 0};
      ci.new_gps_packet = true;
    } else if (gps_rel_pos.north != last_gps_pos.north || gps_rel_pos.west != last_gps_pos.west || gps_rel_pos.up != last_gps_pos.up) {
      ci.new_gps_packet = true;
      last_gps_pos = gps_rel_pos;
    } else {
      ci.new_gps_packet = false;
    }

    Controller_Output co = ControllerAndEstimator::get_controller_output(ci, X_trg, U_ff, ideal_dT, loop_dT, &cs);
    float thrust_perc;
    float diffy_perc;
    get_prop_perc(co.thrust_N, co.roll_rad_sec_squared, &thrust_perc, &diffy_perc);
    if (flight_armed) {
      Prop::set_throttle_roll(thrust_perc, diffy_perc);
      GimbalServos::setGimbalAngle(-co.gimbal_pitch_deg, co.gimbal_yaw_deg);
    }

    if (flight_armed) // we only want to log flight data, not pre-flight
    {
      TrajectoryLogger::flash_log_sensor(time_s, 0, ci, co);

      if (timer - lastlogx_est > LOG_X_EST_INTERVAL_US) {
        TrajectoryLogger::log_x_est();
        lastlogx_est = timer;
      }
    }

    if (timer - lasttelemetry > TELEMETRY_INTERVAL_US) {
      lasttelemetry = timer;

      fp.ci = ci;
      fp.ci.imu.accel_x = cs.filter_out[0];
      fp.ci.imu.accel_y = cs.filter_out[1];
      fp.ci.imu.accel_z = cs.filter_out[2];
      fp.ci.imu.gyro_yaw = cs.filter_out[3];
      fp.ci.imu.gyro_pitch = cs.filter_out[4];
      fp.ci.imu.gyro_roll = cs.filter_out[5];
      fp.ci.mag.mag_x = cs.filter_out[6];
      fp.ci.mag.mag_y = cs.filter_out[7];
      fp.ci.mag.mag_z = cs.filter_out[8];
      fp.x_est = cs.x_est;
      fp.co = co;

      fp.elapsed_time = timer / 1000000.0;
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

  digitalWrite(PIN_TEST_LED, HIGH);

  Logging::disarm();

  Prop::stop();

  fp.flight_armed = false;
  CommandRouter::send_command("tr", fp);

  CommsSerial.printf("Finished %ld loop iterations.\n", counter);
  if (kill_flag) {
    CommsSerial.println("Flight was terminated due to manual kill.");
  }
  if (auto_land) {
    CommsSerial.println("Autoland sequence was activated.");
  }

  TrajectoryLoader::begin(); // reset trajectory in case we modified it during flight
}

void heartbeat() {
  last_hb = timer;
}

// add relevant router cmds
void begin() {
  CommandRouter::add(start_flight_loop, "start_flight_loop");
  CommandRouter::add(heartbeat, "hb", "send every 0.5 seconds or flight will terminate");
  CommandRouter::add_flag(&kill_flag, "k", "terminate the flight loop early");
  CommandRouter::add_flag(&auto_land_cmd, "a", "trigger auto land sequence");
  CommandRouter::add_flag(&arm_flag, "arm", "start following a trajectory");
}

// prompt user for log file name, then follow trajectory
void start_flight_loop() {
  if (!TrajectoryLoader::isLoaded) {
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
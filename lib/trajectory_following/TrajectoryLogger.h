#pragma once

#include "controller_and_estimator.h"
#include <stdint.h>
#include <sys/cdefs.h>

const uint8_t ENTRY_SENSOR = 0;
const uint8_t ENTRY_GPS = 1;
const uint8_t ENTRY_X_EST = 2;
const uint8_t ENTRY_CALIB = 3;
const uint8_t ENTRY_CONTROLLER_OUT = 4;
const uint8_t ENTRY_LOOP_STATE = 5;
const uint8_t ENTRY_FLIGHT_P = 6;
const uint8_t ENTRY_TRAJECTORY = 7;

struct __packed LoopState {
  float time;
  uint8_t phase;
};

static_assert(sizeof(LoopState) == 5, "sizeof(LoopState) error");

static_assert(sizeof(IMU_MAG_State) == 36, "sizeof(SensorEntry) error");

struct __packed GpsEntry {

  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;

  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float posCovNN; // m^2
  float posCovNE; // m^2
  float posCovND; // m^2
  float posCovEE; // m^2
  float posCovED; // m^2
  float posCovDD; // m^2
  float velCovNN; // m^2/s^2
  float velCovNE; // m^2/s^2
  float velCovND; // m^2/s^2
  float velCovEE; // m^2/s^2
  float velCovED; // m^2/s^2
  float velCovDD; // m^2/s^2
};

static_assert(sizeof(GpsEntry) == 18 * 4, "sizeof(GpsEntry) error");

namespace TrajectoryLogger {

void flash_log_sensor(float time, int phase, const Controller_Input ci, const Controller_Output co);

void log_calib_flash();

void log_x_est();

void log_complete();

void send_flash_over_serial();

void log_trajectory();

void begin();
}; // namespace TrajectoryLogger

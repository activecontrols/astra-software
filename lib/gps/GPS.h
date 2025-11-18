#pragma once
#include <ArduinoEigenDense.h>

// Note - using matlab funcs based off of ASTRA simulation: f8cb7fa0f5e8ee347ce072c0007488427cedc874
// TODO - update matlab func commit

using Vector3 = Eigen::Matrix<float, 3, 1>;
using Vector4 = Eigen::Matrix<float, 4, 1>;
using Vector6 = Eigen::Matrix<float, 6, 1>;
using Vector12 = Eigen::Matrix<float, 12, 1>;
using Vector13 = Eigen::Matrix<float, 13, 1>;
using Vector15 = Eigen::Matrix<float, 15, 1>;
using Matrix3_3 = Eigen::Matrix<float, 3, 3>;
using Matrix4_4 = Eigen::Matrix<float, 4, 4>;
using Matrix4_12 = Eigen::Matrix<float, 4, 12>;
using Matrix6_6 = Eigen::Matrix<float, 6, 6>;
using Matrix6_12 = Eigen::Matrix<float, 6, 12>;
using Matrix12_6 = Eigen::Matrix<float, 12, 6>;
using Matrix12_12 = Eigen::Matrix<float, 12, 12>;
#include <Arduino.h>

#define GPS_UART Serial1

#define EARTH_RADIUS_M 6371001.0

struct GPS_Coord {
  double lat; // deg lat
  double lon; // deg lon
  double alt; // meters
};

struct GPS_Velocity {
  double north; // m/s velocity north
  double west;  // m/s velocity west
  double up;    // m/s velocity up
};

struct Point {
  double north; // meters
  double west;  // meters
  double up;    // meters
};

namespace GPS {

extern GPS_Coord origin;

// setup communication to GPS hardware
void begin();

// update GPS with new serial data, will be called periodically during flight loop
// all other functions use the state generated here and are non-blocking
void pump_events();

// check if the GPS is connected to satelites and has sent valid position data recently
bool has_valid_recent_pos();

// gets the latest lat/lon/alt sent by the GPS
// only safe to call if has_valid_recent_pos() = true
GPS_Coord get_lat_lon_alt();

// flag the current position as origin for use by get_rel_xyz_pos
// will be called once before flight
void set_current_position_as_origin();

// uses get_lat_lon_alt() and origin to get the relative north/west/up in meters
Point get_rel_xyz_pos();

// gets the latest lat/lon/alt vel sent by the GPS
// only safe to call if has_valid_recent_pos() = true
GPS_Velocity get_velocity();

// router functions
void print_gps_pos();
void print_rel_pos();

void get_vel_cov(Matrix3_3 &out);
void get_pos_cov(Matrix3_3 &out);

bool is_vel_cov_valid();
bool is_pos_cov_valid();

} // namespace GPS
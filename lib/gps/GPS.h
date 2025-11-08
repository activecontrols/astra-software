#pragma once
#include "TinyGPS++.h"
#include <Arduino.h>

#define GPS_UART Serial1

#define EARTH_RADIUS_M 6371000.0

struct GPS_Coord {
  double lat; // deg lat
  double lon; // deg lon
  double alt; // meters
};

struct Point {
  double north; // meters
  double west;  // meters
  double up;    // meters
};

namespace GPS {

extern GPS_Coord origin;
extern TinyGPSPlus gps;

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

// router functions
void print_gps_pos();
void print_rel_pos();

} // namespace GPS
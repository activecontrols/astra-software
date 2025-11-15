#include "GPS.h"
#include "Router.h"
#include "UBX.h"

// #define DEBUG_GPS_MSG

namespace GPS {
GPS_Coord origin;

UBX ubx;

double deg_to_rad(double degrees) {
  return degrees * (PI / 180.0);
}

void begin() {

  /*
  UBX-CFG-RATE - configure output rate
  UBX-NAV-PVT - position velocity time solution
  */

  GPS_UART.begin(38400, SERIAL_8N1); // https://content.u-blox.com/sites/default/files/documents/NEO-F9P-15B_DataSheet_UBX-22021920.pdf


#ifdef DEBUG_GPS_MSG
  Router::println("Undefine `DEBUG_GPS_MSG` to remove GPS prints.");
#endif

  Router::add({print_gps_pos, "gps_print_pos"});
  Router::add({print_rel_pos, "gps_print_rel_pos"});
  Router::add({set_current_position_as_origin, "gps_set_origin"});
  Router::add({pump_events, "pump_events"});
}

void pump_events() {
  while (GPS_UART.available() > 0) { // https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    char c = GPS_UART.read();
    ubx.encode(c);

#ifdef DEBUG_GPS_MSG
    Router::print(c);
#endif
  }

#ifdef DEBUG_GPS_MSG
  if (ubx.updated) {
    // convert from (degrees * 10^-7) to degrees
    double real_lat = ubx.pvt_solution.lat / 10000000.0;
    double real_lon = ubx.pvt_solution.lon / 10000000.0;

    // convert from mm/s to m/s
    double velocity_north = ubx.pvt_solution.velN * 1e-3;
    double velocity_east = ubx.pvt_solution.velE * 1e-3;
    double velocity_down = ubx.pvt_solution.velD * 1e-3;

    double altitude = ubx.pvt_solution.hMSL * 1e-3; // convert mm to meters

    Router::println("================");
    Router::printf("Satellite Count: %d\n", ubx.pvt_solution.numSV);
    Router::printf("Lat Lon: %d %d\n", ubx.pvt_solution.lat, ubx.pvt_solution.lon);
    Router::printf("Latitude  (deg):  %lf\n", real_lat);
    Router::printf("Longitude (deg): %lf\n", real_lon);
    Router::printf("Velocity North (m/s): %lf\n", velocity_north);
    Router::printf("Velocity East  (m/s): %lf\n", velocity_east);
    Router::printf("Velocity Down  (m/s): %lf\n", velocity_down);
    Router::printf("Altitude (m): %lf\n", altitude);
    Router::println("================");
  }
#endif
}

bool has_valid_recent_pos() {
  return ubx.isValid();
}

GPS_Coord get_lat_lon_alt() {
  return GPS_Coord{ubx.pvt_solution.lat / 10000000.0, ubx.pvt_solution.lon / 10000000.0, ubx.pvt_solution.hMSL / 1000.0};
}

GPS_Velocity get_velocity() {
  return GPS_Velocity{ubx.pvt_solution.velN / 1000.0, ubx.pvt_solution.velE / 1000.0, ubx.pvt_solution.velD / 1000.0};
}

void set_current_position_as_origin() {
  if (!has_valid_recent_pos()) {
    Router::print("Warning - tried to set current position as origin without valid GPS position.");
  } else {
    origin = get_lat_lon_alt();
  }
}

Point get_rel_xyz_pos() {
  GPS_Coord pos = get_lat_lon_alt();
  double origin_lat_rad = deg_to_rad(origin.lat);

  double dlat_rad = deg_to_rad(pos.lat - origin.lat);
  double dlon_rad = deg_to_rad(pos.lon - origin.lon);

  double north_m = EARTH_RADIUS_M * dlat_rad;
  double east_m = EARTH_RADIUS_M * std::cos(origin_lat_rad) * dlon_rad;
  double west_m = -east_m;
  double up_m = pos.alt - origin.alt;

  return Point{north : north_m, west : west_m, up : up_m};
}

void print_gps_pos() {
  if (!has_valid_recent_pos()) {
    Router::printf("GPS does not have valid position.");
  } else {
    GPS_Coord c = get_lat_lon_alt();
    Router::printf("GPS Lat:%.6f Lon:%.6f Alt:%.3f\n", c.lat, c.lon, c.alt);
  }
}

void print_rel_pos() {
  if (!has_valid_recent_pos()) {
    Router::printf("GPS does not have valid position.");
  } else {
    Point p = get_rel_xyz_pos();
    Router::printf("GPS relative position north:%.3f west:%.3f up:%.3f\n", p.north, p.west, p.up);
  }
}

} // namespace GPS
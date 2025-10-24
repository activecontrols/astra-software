#include "GPS.h"
#include "Router.h"

namespace GPS {
TinyGPSPlus gps;
GPS_Coord origin;

double deg_to_rad(double degrees) {
  return degrees * (PI / 180.0);
}

void begin() {
  GPS_UART.begin(38400, SERIAL_8N1); // https://content.u-blox.com/sites/default/files/documents/NEO-F9P-15B_DataSheet_UBX-22021920.pdf
}

// TODO GPS - this code was just for testing - please update to pass into tinygps++
void pump_events() {
  char msg[2];

  while (GPS_UART.available() > 0) { // https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    char c = GPS_UART.read();
    gps.encode(c);
    msg[0] = c;
    msg[1] = '\0';
    Router::print(msg);
  }
}

bool has_valid_recent_pos() {
  return gps.location.isValid() && gps.altitude.isValid();
}

GPS_Coord get_lat_lon_alt() {
  return GPS_Coord{gps.location.lat(), gps.location.lng(), gps.altitude.meters()};
}

void set_current_position_as_origin() {
  origin = get_lat_lon_alt();
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

} // namespace GPS
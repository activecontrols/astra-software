#include "GPS.h"
#include "Router.h"

namespace GPS {
TinyGPSPlus gps;
GPS_Coord origin;

double deg_to_rad(double degrees) {
  return degrees * (PI / 180.0);
}

double haversine_distance(const GPS_Coord &coord1, const GPS_Coord &coord2) {
  double lat1_rad = deg_to_rad(coord1.lat);
  double lon1_rad = deg_to_rad(coord1.lon);
  double lat2_rad = deg_to_rad(coord2.lat);
  double lon2_rad = deg_to_rad(coord2.lon);

  double dlat = lat2_rad - lat1_rad;
  double dlon = lon2_rad - lon1_rad;

  double a = std::pow(std::sin(dlat / 2), 2) + std::cos(lat1_rad) * std::cos(lat2_rad) * std::pow(std::sin(dlon / 2), 2);

  double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

  return EARTH_RADIUS_M * c;
}

Point convert_gps_to_point(const GPS_Coord &target, const GPS_Coord &origin) {
  double origin_lat_rad = deg_to_rad(origin.lat);

  double dlat_rad = deg_to_rad(target.lat - origin.lat);
  double dlon_rad = deg_to_rad(target.lon - origin.lon);

  double north_m = EARTH_RADIUS_M * dlat_rad;
  double east_m = EARTH_RADIUS_M * std::cos(origin_lat_rad) * dlon_rad;
  double west_m = -east_m;
  double up_m = target.alt - origin.alt;

  return Point{north : north_m, west : west_m, up : up_m};
}

void begin() {
  GPS_UART.begin(38400, SERIAL_8N1); // https://content.u-blox.com/sites/default/files/documents/NEO-F9P-15B_DataSheet_UBX-22021920.pdf
}

// TODO GPS - this code was just for testing - please update to pass into tinygps++
void pump_events() {
  char msg[2];

  while (GPS_UART.available() > 0) { // https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    gps.encode(GPS_UART.read());
    msg[0] = (char)GPS_UART.read();
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
  return convert_gps_to_point(get_lat_lon_alt(), origin);
}

} // namespace GPS
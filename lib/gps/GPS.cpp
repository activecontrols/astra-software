#include "GPS.h"
#include "Router.h"

namespace GPS {
TinyGPSPlus gps;
GPS_Coord origin;

void begin() {
  GPS_UART.begin(38400, SERIAL_8N1); // https://content.u-blox.com/sites/default/files/documents/NEO-F9P-15B_DataSheet_UBX-22021920.pdf
}

// TODO GPS - this code was just for testing - please update to pass into tinygps++
void pump_events() {
  char msg[2];

  while (GPS_UART.available()) { // https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    msg[0] = (char)GPS_UART.read();
    msg[1] = '\0';
    Router::print(msg);
  }
}

bool has_valid_recent_pos() {
  return gps.location.isValid() && gps.altitude.isValid();
}

GPS_Coord get_lat_lon_alt() {
  return GPS_Coord{static_cast<float>(gps.location.lat()), static_cast<float>(gps.location.lng()), static_cast<float>(gps.altitude.meters())};
}

void set_current_position_as_origin() {
  origin = get_lat_lon_alt();
}

Point get_rel_xyz_pos() {
  return Point{static_cast<float>(gps.location.lat()) - origin.lat, static_cast<float>(gps.location.lng()) - origin.lon, static_cast<float>(gps.altitude.meters()) - origin.alt};
}

} // namespace GPS
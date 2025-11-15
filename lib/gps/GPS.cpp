#include "GPS.h"
#include "Router.h"

#define DEBUG_GPS_MSG

namespace GPS {
TinyGPSPlus gps;
GPS_Coord origin;

double deg_to_rad(double degrees) {
  return degrees * (PI / 180.0);
}

uint8_t CK_A;
uint8_t CK_B;

int payload_position;

template <typename t> t ubx_read() {
  t ret = 0;
  for (unsigned int i = 0; i < sizeof(ret); ++i) {
    while (!GPS_UART.available());
    uint8_t val = GPS_UART.read();
    CK_A += val;
    CK_B += CK_A;
    *((uint8_t *)(&ret) + i) = val;
    ++payload_position;

#ifdef DEBUG_GPS_MSG
    Router::printf("0x%X ", val);
#endif
  }
  return ret;
}

void begin() {

  /*
  UBX-CFG-RATE - configure output rate
  UBX-NAV-PVT - position velocity time solution
  */

  GPS_UART.begin(38400, SERIAL_8N1); // https://content.u-blox.com/sites/default/files/documents/NEO-F9P-15B_DataSheet_UBX-22021920.pdf

  pump_events();

  while (1) {
#ifdef DEBUG_GPS_MSG
    Serial.println();
    Serial.println("Reading Packet");
#endif
    // check preamble
    while (!GPS_UART.available());
    uint8_t a = GPS_UART.read();
    while (!GPS_UART.available());
    uint8_t b = GPS_UART.peek();
#ifdef DEBUG_GPS_MSG
    Router::printf("0x%X 0x%X ", a, b);
#endif

    // synchronize with the start of the message frame
    if (!(a == 0xb5 && b == 0x62)) {
      continue;
    }
    // read the preamble second character if synced
    GPS_UART.read();

    // reset checksum
    CK_A = 0;
    CK_B = 0;

    uint8_t message_class = ubx_read<uint8_t>();

    uint8_t message_id = ubx_read<uint8_t>();

    uint16_t length = ubx_read<uint16_t>();

    Router::printf("\nlength: %d\n", length);

    // check if we even care what packet this is
    if (!(message_class == 0x01 && message_id == 0x07)) {
      // skip to where the next packet should start
      for (int i = 0; i < length + 2; ++i) {
        while (!GPS_UART.available());
        GPS_UART.read();
      }
      continue;
    }

    // parse payload
    payload_position = 0;
    while (payload_position < 23) {
      ubx_read<uint8_t>(); // skip to the good part
    }

    uint8_t numSV = ubx_read<uint8_t>(); // number of satellites

    int32_t lon = ubx_read<int32_t>();
    int32_t lat = ubx_read<int32_t>();

    while (payload_position < 48) {
      ubx_read<uint8_t>();
    }

    // north velocity, east velocity, down velocity
    int32_t velN = ubx_read<int32_t>();
    int32_t velE = ubx_read<int32_t>();
    int32_t velD = ubx_read<int32_t>();

    // skip to the end of the payload
    while (payload_position < length) {
      ubx_read<uint8_t>();
    }

    // verify checksum
    while (!GPS_UART.available());
    uint8_t msg_CK_A = GPS_UART.read();
    while (!GPS_UART.available());
    uint8_t msg_CK_B = GPS_UART.read();

    // output only if checksum was legit
    if (!(msg_CK_A == CK_A && msg_CK_B == CK_B)) {
      Router::println("Checksum Failed!!!");
      continue;
    }

    // convert to degrees from (degrees * 10^-7)
    double real_lat = lat / 10000000 + (lat % 10000000) / 10000000.0;
    double real_lon = lon / 10000000 + (lon % 10000000) / 10000000.0;

    // convert from mm/s to m/s
    double velocity_north = velN * 1e-3;
    double velocity_east = velE * 1e-3;
    double velocity_down = velD * 1e-3;

    Router::println("================");
    Router::printf("Satellite Count: %d\n", numSV);
    Router::printf("Lat Lon: %d %d\n", lat, lon);
    Router::printf("Latitude  (deg):  %lf\n", real_lat);
    Router::printf("Longitude (deg): %lf\n", real_lon);
    Router::printf("Velocity North (m/s): %lf\n", velocity_north);
    Router::printf("Velocity East  (m/s): %lf\n", velocity_east);
    Router::printf("Velocity Down  (m/s): %lf\n", velocity_down);
    Router::println("================");
  }


#ifdef DEBUG_GPS_MSG
  Router::println("Undefine `DEBUG_GPS_MSG` to remove GPS prints.");
#endif

  Router::add({print_gps_pos, "gps_print_pos"});
  Router::add({print_rel_pos, "gps_print_rel_pos"});
  Router::add({set_current_position_as_origin, "gps_set_origin"});
  Router::add({pump_events, "pump_events"});
}

void pump_events() {
  unsigned long last = millis();
  while (!Serial.available()) {
    while (GPS_UART.available() > 0) { // https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
      char c = GPS_UART.read();
      // gps.encode(c);

#ifdef DEBUG_GPS_MSG
      Router::print(c);
#endif
    }
  }
}

bool has_valid_recent_pos() {
  return gps.location.isValid() && gps.altitude.isValid();
}

GPS_Coord get_lat_lon_alt() {
  return GPS_Coord{gps.location.lat(), gps.location.lng(), gps.altitude.meters()};
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
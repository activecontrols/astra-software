#include "GPS.h"
#include "Router.h"
#include "UBX.h"
#include "fc_pins.h"

#define DEBUG_GPS_MSG

namespace GPS {
GPS_Coord origin;

UBX ubx;

double deg_to_rad(double degrees) {
  return degrees * (PI / 180.0);
}

void gps_speed_test() {
  unsigned long start_time = millis();

  while (millis() - start_time < 20000) {
    unsigned long start_micros = micros();
    pump_events();
    unsigned long delta_micros = micros() - start_micros;

    Router::printf("(%d, %d),", ubx.pvt_solution.updated, delta_micros);
    ubx.pvt_solution.updated = false;
    delay(1);
  }
  Router::println();
}

void output_gps_inf_cbk(uint8_t id, const char *msg) {
  Router::printf(" -GPS INFO- [%s]: %s\n", UBX::inf_message_name(id), msg);
}

// outputs gps info messages
void output_gps_inf() {
  void (*old_cbk)(uint8_t, const char *) = ubx.inf_msg_cbk;
  ubx.inf_msg_cbk = output_gps_inf_cbk;

  while (!external_uart.available()) {
    pump_events();

    delay(20);
  }

  while (external_uart.read() != '\n')
    ;
  ubx.inf_msg_cbk = old_cbk;
}

void print_gps_events() {
  while (!external_uart.available()) {
    pump_events();

    if (ubx.pvt_solution.updated && has_valid_recent_pos()) {
      ubx.pvt_solution.updated = false;

      // convert from (degrees * 10^-7) to degrees
      double real_lat = ubx.pvt_solution.data->lat / 10000000.0;
      double real_lon = ubx.pvt_solution.data->lon / 10000000.0;

      // convert from mm/s to m/s
      double velocity_north = ubx.pvt_solution.data->velN * 1e-3;
      double velocity_east = ubx.pvt_solution.data->velE * 1e-3;
      double velocity_down = ubx.pvt_solution.data->velD * 1e-3;

      double altitude = ubx.pvt_solution.data->hMSL * 1e-3; // convert mm to meters

      Router::println("================");
      Router::printf("Satellite Count: %d\n", ubx.pvt_solution.data->numSV);
      Router::printf("Lat Lon: %d %d\n", ubx.pvt_solution.data->lat, ubx.pvt_solution.data->lon);
      Router::printf("Latitude  (deg):  %lf\n", real_lat);
      Router::printf("Longitude (deg): %lf\n", real_lon);
      Router::printf("Velocity North (m/s): %lf\n", velocity_north);
      Router::printf("Velocity East  (m/s): %lf\n", velocity_east);
      Router::printf("Velocity Down  (m/s): %lf\n", velocity_down);
      Router::printf("Altitude (m): %lf\n", altitude);
      Router::printf("Horizontal Accuracy Estimate (m): %lf\n", ubx.pvt_solution.data->hAcc / 1000.0);
      Router::printf("Vertical Accuracy Estimate (m): %lf\n", ubx.pvt_solution.data->vAcc / 1000.0);

      Router::println("================");
    }

    if (ubx.cov.updated) {
      ubx.cov.updated = false;

      // output covariance matrices
      Router::print("================\n");
      Router::print("Position covariance (m^2)\n");
      Router::printf("%9.3f %9.3f %9.3f\n", ubx.cov.data->posCovNN, -ubx.cov.data->posCovNE, -ubx.cov.data->posCovND);
      Router::printf("%9.3f %9.3f %9.3f\n", -ubx.cov.data->posCovNE, ubx.cov.data->posCovEE, ubx.cov.data->posCovED);
      Router::printf("%9.3f %9.3f %9.3f\n", -ubx.cov.data->posCovND, ubx.cov.data->posCovED, ubx.cov.data->posCovDD);

      Router::print("Velocity Covariance (m^2/s^2)\n");
      Router::printf("%9.3f %9.3f %9.3f\n", ubx.cov.data->velCovNN, -ubx.cov.data->velCovNE, -ubx.cov.data->velCovND);
      Router::printf("%9.3f %9.3f %9.3f\n", -ubx.cov.data->velCovNE, ubx.cov.data->velCovEE, ubx.cov.data->velCovED);
      Router::printf("%9.3f %9.3f %9.3f\n", -ubx.cov.data->velCovND, ubx.cov.data->velCovED, ubx.cov.data->velCovDD);
      Router::print("================\n");
    }

    delay(10);
  }

  while (external_uart.read() != '\n')
    ;
}

void begin() {

  gps_uart.begin(38400, SERIAL_8N1); // https://content.u-blox.com/sites/default/files/documents/NEO-F9P-15B_DataSheet_UBX-22021920.pdf

#ifdef DEBUG_GPS_MSG
  Router::println("Undefine `DEBUG_GPS_MSG` to remove GPS prints.");
#endif

  ubx.inf_msg_cbk = output_gps_inf_cbk;

  Router::add({print_gps_pos, "gps_print_pos"});
  Router::add({print_rel_pos, "gps_print_rel_pos"});
  Router::add({set_current_position_as_origin, "gps_set_origin"});
  Router::add({pump_events, "pump_events"});
  Router::add({gps_speed_test, "gps_speed_test"});
  Router::add({print_gps_events, "print_gps_events"});
  Router::add({output_gps_inf, "gps_print_info"});
}

bool is_vel_cov_valid() {
  return (ubx.cov.data->velCovValid);
}

bool is_pos_cov_valid() {
  return (ubx.cov.data->posCovValid);
}

// get velocity covariance
void get_vel_cov(Matrix3_3 &out) {
  out(0, 0) = ubx.cov.data->velCovNN;
  out(0, 1) = -ubx.cov.data->velCovNE;
  out(0, 2) = -ubx.cov.data->velCovND;
  out(1, 0) = -ubx.cov.data->velCovNE;
  out(1, 1) = ubx.cov.data->velCovEE;
  out(1, 2) = ubx.cov.data->velCovED;
  out(2, 0) = -ubx.cov.data->velCovND;
  out(2, 1) = ubx.cov.data->velCovED;
  out(2, 2) = ubx.cov.data->velCovDD;
}

// get position covariance
void get_pos_cov(Matrix3_3 &out) {
  out(0, 0) = ubx.cov.data->posCovNN;
  out(0, 1) = -ubx.cov.data->posCovNE;
  out(0, 2) = -ubx.cov.data->posCovND;
  out(1, 0) = -ubx.cov.data->posCovNE;
  out(1, 1) = ubx.cov.data->posCovEE;
  out(1, 2) = ubx.cov.data->posCovED;
  out(2, 0) = -ubx.cov.data->posCovND;
  out(2, 1) = ubx.cov.data->posCovED;
  out(2, 2) = ubx.cov.data->posCovDD;
}

void pump_events() {
  while (gps_uart.available() > 0) { // https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    char c = gps_uart.read();
    ubx.encode(c);

  }

#ifdef DEBUG_GPS_MSG
  if (ubx.pvt_solution.updated) {
    // convert from (degrees * 10^-7) to degrees
    double real_lat = ubx.pvt_solution.data->lat / 10000000.0;
    double real_lon = ubx.pvt_solution.data->lon / 10000000.0;

    // convert from mm/s to m/s
    double velocity_north = ubx.pvt_solution.data->velN * 1e-3;
    double velocity_east = ubx.pvt_solution.data->velE * 1e-3;
    double velocity_down = ubx.pvt_solution.data->velD * 1e-3;

    double altitude = ubx.pvt_solution.data->hMSL * 1e-3; // convert mm to meters

    Router::println("================");
    Router::printf("Satellite Count: %d\n", ubx.pvt_solution.data->numSV);
    Router::printf("Lat Lon: %d %d\n", ubx.pvt_solution.data->lat, ubx.pvt_solution.data->lon);
    Router::printf("Latitude  (deg):  %lf\n", real_lat);
    Router::printf("Longitude (deg): %lf\n", real_lon);
    Router::printf("Velocity North (m/s): %lf\n", velocity_north);
    Router::printf("Velocity East  (m/s): %lf\n", velocity_east);
    Router::printf("Velocity Down  (m/s): %lf\n", velocity_down);
    Router::printf("Altitude (m): %lf\n", altitude);
    Router::printf("Horizontal Accuracy Estimate (m): %lf\n", ubx.pvt_solution.data->hAcc / 1000.0);
    Router::printf("Vertical Accuracy Estimate (m): %lf\n", ubx.pvt_solution.data->vAcc / 1000.0);
    Router::printf("carrSoln: %d\n", (ubx.pvt_solution.data->flags >> 6) & 0b11);
    Router::println("================");

    ubx.pvt_solution.updated = false;
  }
#endif
}

bool has_valid_recent_pos() {
  return ubx.pvt_solution.is_valid() && !(ubx.pvt_solution.data->flags3 & 1); // (check bit flag for invalid lat, long, hmsl)
}

GPS_Coord get_lat_lon_alt() {
  return GPS_Coord{ubx.pvt_solution.data->lat / 10000000.0, ubx.pvt_solution.data->lon / 10000000.0, ubx.pvt_solution.data->hMSL / 1000.0};
}

GPS_Velocity get_velocity() {
  return GPS_Velocity{ubx.pvt_solution.data->velN / 1000.0, -ubx.pvt_solution.data->velE / 1000.0, -ubx.pvt_solution.data->velD / 1000.0};
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

  double north_m = (EARTH_RADIUS_M + pos.alt) * dlat_rad;
  double east_m = (EARTH_RADIUS_M + pos.alt) * std::cos(origin_lat_rad) * dlon_rad;
  double west_m = -east_m;
  double up_m = pos.alt - origin.alt;

  return Point{north : north_m, west : west_m, up : up_m};
}

void print_gps_pos() {
  pump_events();
  if (!has_valid_recent_pos()) {
    Router::printf("GPS does not have valid position.\n");
  } else {
    GPS_Coord c = get_lat_lon_alt();
    Router::printf("GPS Lat:%.6f Lon:%.6f Alt:%.3f\n", c.lat, c.lon, c.alt);
  }
}

void print_rel_pos() {
  pump_events();
  if (!has_valid_recent_pos()) {
    Router::printf("GPS does not have valid position.\n");
  } else {
    Point p = get_rel_xyz_pos();
    Router::printf("GPS relative position north:%.3f west:%.3f up:%.3f\n", p.north, p.west, p.up);
  }
}

} // namespace GPS
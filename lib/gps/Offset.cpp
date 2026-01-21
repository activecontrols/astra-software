#include "Offset.h"
#include "GPS.h"
#include "Router.h"
#include "UBX.h"

GPS_Coord difference(double lat, double lon, double alt, GPS_Coord pos) {
  GPS_Coord diff;
  diff.lat = lat - pos.lat;
  diff.lon = lon - pos.lon;
  diff.alt = alt - pos.alt;
  return diff;
}

GPS_Coord add_offset(GPS_Coord rocket_pos, GPS_Coord diff) {
  GPS_Coord accurate_rocket;
  accurate_rocket.lat = diff.lat + rocket_pos.lat;
  accurate_rocket.lon = diff.lon + rocket_pos.lon;
  accurate_rocket.alt = diff.alt + rocket_pos.alt;
  return accurate_rocket;
}

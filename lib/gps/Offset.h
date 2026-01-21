#ifndef GPS_OFFSET_H
#define GPS_OFFSET_H

#include "GPS.h";

GPS_Coord difference(double lat, double lon, double alt, GPS_Coord pos);

GPS_Coord add_offset(double lat_offset, double lon_offset, double alt_offset, GPS_Coord pos);

#endif
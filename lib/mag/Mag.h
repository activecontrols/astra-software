#pragma once

#include <Arduino.h>

namespace Mag {

struct calibration {
  double hard_x;
  double hard_y;
  double hard_z;
  double soft[3][3];
};

extern const char *main_calib_name; // defined in mag.cpp as "mag_calib.bin"

void begin();
bool read_xyz(double &mx, double &my, double &mz);
bool read_xyz_normalized(double &mx, double &my, double &mz);
double get_heading();

bool isMeasurementReady();
bool beginMeasurement();
} // namespace Mag
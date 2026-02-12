#pragma once
#include "fc_pins.h"

namespace Mag {

extern const char *CALIB_FMT;

const int MAG_COUNT = sizeof(MAG_CS) / sizeof(MAG_CS[0]);

void begin();

bool read_xyz(double m[3], int mag_index);
bool read_xyz_all(double m[MAG_COUNT][3]);
bool read_normalized_fused(double m[3]);

bool read_xyz_normalized(double m[3], int mag_index);
bool read_xyz_normalized_all(double m[MAG_COUNT][3]);

double get_heading(int mag_index);

bool isMeasurementReady();
void beginMeasurement();
} // namespace Mag
#pragma once

namespace Mag {

extern const char *main_calib_name; // defined in mag.cpp as "mag_calib.bin"

void begin();
void read_xyz(double &mx, double &my, double &mz);
double get_heading();
} // namespace Mag
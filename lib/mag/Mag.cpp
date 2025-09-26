#include <Mag.h>

#include <Router.h>
#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h>

SFE_MMC5983MA mag;

namespace Mag {

void mag_rawprint(const char *) {
  double mx, my, mz;
  read_xyz(mx, my, mz);
  Router::println(String(mx, 6) + "," + String(my, 6) + "," + String(mz, 6));
}

void mag_heading(const char *) {
  //   double heading = get_heading();
  //   Router::println(String(heading, 3));

  int time = 60 * 10;
  while (time-- > 0) {
    double heading = get_heading();
    Router::println(String(heading, 3));
    delay(100);
  }
}

void init() {
  Wire.begin();
  // Initialize the magnetometer
  if (!mag.begin()) {
    Serial.println("Magnetometer not found");
    while (true)
      ;
  }
  mag.softReset();

  Router::add({mag_rawprint, "mag_raw"});
  Router::add({mag_heading, "mag_heading"});
}

void read_xyz(double &mx, double &my, double &mz) {
  uint32_t rawValueX = 0;
  uint32_t rawValueY = 0;
  uint32_t rawValueZ = 0;

  // Read all three channels simultaneously
  mag.getMeasurementXYZ(&rawValueX, &rawValueY, &rawValueZ);

  // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17 (131072).
  // Here we scale each field to +/- 1.0 to make it easier to calculate an approximate heading.
  //
  // Please note: to properly correct and calibrate the X, Y and Z channels, you need to determine true
  // offsets (zero points) and scale factors (gains) for all three channels. Futher details can be found at:
  // https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

  // todo: IG figure out calibration.

  mx = ((double)rawValueX - 131072.0) / 131072.0;
  my = ((double)rawValueY - 131072.0) / 131072.0;
  mz = ((double)rawValueZ - 131072.0) / 131072.0;
}

double get_heading() {
  double mx, my, mz;
  read_xyz(mx, my, mz);
  // Magnetic north is oriented with the Y axis
  // Convert the X and Y fields into heading using atan2 (Arc Tangent 2)
  double heading = atan2(mx, -my);

  // atan2 returns a value between +PI and -PI
  // Convert to degrees
  heading /= PI;
  heading *= 180;
  heading += 180;
  return heading;
}

} // namespace Mag
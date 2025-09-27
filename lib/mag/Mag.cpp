#include <Mag.h>

#include <Router.h>
#include <SDCard.h>
#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h>

SFE_MMC5983MA mag;

namespace Mag {

struct calibration {
  double hard_x;
  double hard_y;
  double hard_z;
  double soft[3][3];
};

// these come from final.m in mag_calib/
calibration matlab_calib = {-1714.23, -2777.08, 1604.81, {{1.0656, 0.0175, -0.0067}, {0.0175, 0.9805, 0.0141}, {-0.0067, 0.0141, 1.0678}}};
calibration identity_calib = {0.0, 0.0, 0.0, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

calibration calib = matlab_calib; // start with matlab calibration

void apply_calibration(double &x, double &y, double &z) {
  x -= calib.hard_x;
  y -= calib.hard_y;
  z -= calib.hard_z;

  double cal_x = calib.soft[0][0] * x + calib.soft[0][1] * y + calib.soft[0][2] * z;
  double cal_y = calib.soft[1][0] * x + calib.soft[1][1] * y + calib.soft[1][2] * z;
  double cal_z = calib.soft[2][0] * x + calib.soft[2][1] * y + calib.soft[2][2] * z;

  x = cal_x;
  y = cal_y;
  z = cal_z;
}

void get_centered_reading(int &mx, int &my, int &mz) {
  uint32_t rawValueX, rawValueY, rawValueZ;
  mag.getMeasurementXYZ(&rawValueX, &rawValueY, &rawValueZ);
  mx = (int)rawValueX - 131072;
  my = (int)rawValueY - 131072;
  mz = (int)rawValueZ - 131072;
}

void read_xyz(double &mx, double &my, double &mz) {
  int rx, ry, rz;
  get_centered_reading(rx, ry, rz);
  mx = (double)rx;
  my = (double)ry;
  mz = (double)rz;
  apply_calibration(mx, my, mz);

  double half = 131072.0; // all values after dividing are in +/- 1.0 range (sensor is 18 bit)
  mx /= half;
  my /= half;
  mz /= half;
}

double get_heading() {
  double mx, my, mz;
  read_xyz(mx, my, mz);
  // Magnetic north is oriented with the Y axis
  double heading = atan2(mx, -my);
  // atan2 returns a value between +PI and -PI
  // Convert to degrees
  heading /= PI;
  heading *= 180;
  heading += 180;
  return heading;
}

// Router commands -------------------------------------------------------------
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

void collect_calib_data(const char *filename) {
  if (filename == nullptr) {
    Router::print("Call with filename: mag_calib <filename>\n");
    return;
  }
  Router::println("Collecting calibration data...");
  Router::println("Move the sensor around in all orientations until done. Waiting 5 seconds to start...");
  delay(5000);
  // write calibration data to file
  File f = SDCard::open(filename, FILE_WRITE);
  if (!f) {
    Router::println("Error opening file for writing");
    return;
  }
  // record 2000 centered readings
  for (int i = 0; i < 2000; i++) {
    int mx, my, mz;
    get_centered_reading(mx, my, mz);
    f.println(String(mx) + "," + String(my) + "," + String(mz));
    delay(100);
    if (i % 10 == 0) { // every second, print progress
      Router::println(String(i) + " samples recorded " + String((i + 1) / 2000.0 * 100.0, 1) + "%");
    }
  }
  f.close();
  Router::println("Done writing calibration data to " + String(filename));
  Router::println("Use final.m in mag_calib/ to compute calibration parameters");
}

void print_calibration(const char *) {
  Router::println("Current calibration:");
  Router::println("Hard iron offsets:");
  Router::println("x: " + String(calib.hard_x, 4));
  Router::println("y: " + String(calib.hard_y, 4));
  Router::println("z: " + String(calib.hard_z, 4));
  Router::println("Soft iron correction matrix:");
  for (int i = 0; i < 3; i++) {
    Router::println(String(calib.soft[i][0], 4) + " " + String(calib.soft[i][1], 4) + " " + String(calib.soft[i][2], 4));
  }
}

void do_simple_calib(const char *) {
  // very simple calibration that just finds hard iron offsets
  // not as good as using final.m in mag_calib/
  Router::println("Doing simple calibration...");
  Router::println("Move the sensor around in all orientations until done. Waiting 5 seconds to start...");
  delay(5000);
  int min_x, min_y, min_z, max_x, max_y, max_z;

  calibration cnew; // new calibration

  for (int i = 0; i < 2000; i++) {
    int mx, my, mz;
    get_centered_reading(mx, my, mz);
    if (i == 0) {
      min_x = max_x = mx;
      min_y = max_y = my;
      min_z = max_z = mz;
    }

    if (mx < min_x)
      min_x = mx;
    if (my < min_y)
      min_y = my;
    if (mz < min_z)
      min_z = mz;

    if (mx > max_x)
      max_x = mx;
    if (my > max_y)
      max_y = my;
    if (mz > max_z)
      max_z = mz;

    delay(100);
    if (i % 10 == 0) { // every second, print progress
      Router::println(String(i) + " samples recorded " + String((i + 1) / 2000.0 * 100.0, 1) + "%");
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cnew.soft[i][j] = (i == j) ? 1.0 : 0.0; // identity matrix
    }
  }
  cnew.hard_x = ((double)min_x + max_x) / 2.0;
  cnew.hard_y = ((double)min_y + max_y) / 2.0;
  cnew.hard_z = ((double)min_z + max_z) / 2.0;
  Router::println("Simple calibration done.");
  calib = cnew; // set new calibration
  print_calibration(nullptr);
}

void reset_calib(const char *) {
  calib = matlab_calib;
  Router::println("Calibration reset to matlab calibration.");
  print_calibration(nullptr);
}

void no_calib(const char *) {
  calib = identity_calib;
  Router::println("Calibration set to identity (no calibration).");
  print_calibration(nullptr);
}
// -----------------------------------------------------------------------------

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
  Router::add({collect_calib_data, "mag_collect_calib"});
  Router::add({do_simple_calib, "mag_do_simple_calib"});
  Router::add({print_calibration, "mag_print_calib"});
  Router::add({reset_calib, "mag_reset_calib"});
  Router::add({no_calib, "mag_no_calib"});
  Router::println("Magnetometer initialized.");
}

} // namespace Mag
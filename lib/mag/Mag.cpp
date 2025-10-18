#include <Mag.h>

#include <Router.h>
#include <SDCard.h>
#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h>

const char *Mag::main_calib_name = "MCALIB.BIN";

SFE_MMC5983MA mag;

namespace Mag {

struct calibration {
  double hard_x;
  double hard_y;
  double hard_z;
  double soft[3][3];
};

// these come from final.m in mag_calib/
// calibration matlab_calib = {-1714.23, -2777.08, 1604.81, {{1.0656, 0.0175, -0.0067}, {0.0175, 0.9805, 0.0141}, {-0.0067, 0.0141, 1.0678}}};

calibration identity_calib = {0.0, 0.0, 0.0, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
calibration calib = identity_calib; // start with identity calibration

void apply_calibration(double &x, double &y, double &z, const calibration &c) {
  x -= c.hard_x;
  y -= c.hard_y;
  z -= c.hard_z;

  double cal_x = c.soft[0][0] * x + c.soft[0][1] * y + c.soft[0][2] * z;
  double cal_y = c.soft[1][0] * x + c.soft[1][1] * y + c.soft[1][2] * z;
  double cal_z = c.soft[2][0] * x + c.soft[2][1] * y + c.soft[2][2] * z;

  x = cal_x;
  y = cal_y;
  z = cal_z;
}

void apply_calibration(double &x, double &y, double &z) {
  apply_calibration(x, y, z, calib);
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

// does not modify input arrays
double calc_sphere_fit_goodness(const double *xs, const double *ys, const double *zs, int n, const calibration &c) {
  double avg_magnitude = 0.0;
  for (int i = 0; i < n; i++) {
    double x = xs[i], y = ys[i], z = zs[i];
    apply_calibration(x, y, z, c);
    avg_magnitude += sqrt(x * x + y * y + z * z);
  }
  avg_magnitude /= n;

  double stdev = 0.0;
  for (int i = 0; i < n; i++) {
    double x = xs[i], y = ys[i], z = zs[i];
    apply_calibration(x, y, z, c); // todo: yes this happens twice. other design choice is to copy arrays or modify etc which seems worse. this is not a hot path.
    double r = sqrt(x * x + y * y + z * z);
    double dr = r - avg_magnitude;
    stdev += dr * dr;
  }
  stdev /= n;
  stdev = sqrt(stdev);
  return 1 - stdev / avg_magnitude; // relative standard deviation
}

double read_x[1000], read_y[1000], read_z[1000];

// populates read_x, read_y, read_z with 1000 samples
void collect_samples() {
  Router::println("Collecting calibration data...");
  Router::println("Move the sensor around in all orientations until done. Waiting 5 seconds to start...");
  delay(5000);
  // record 1000 centered readings
  for (int i = 0; i < 1000; i++) {
    int mx, my, mz;
    get_centered_reading(mx, my, mz);
    read_x[i] = (double)mx;
    read_y[i] = (double)my;
    read_z[i] = (double)mz;
    delay(100);
    if (i % 10 == 0) { // every second, print progress
      Router::println(String(i) + " samples recorded " + String((i + 1) / 1000.0 * 100.0, 1) + "%");
    }
  }
  Router::println("Done recording calibration data.");
  double goodness = calc_sphere_fit_goodness(read_x, read_y, read_z, 1000, identity_calib);
  Router::println("Sphere fit goodness of raw data: " + String(goodness, 4));
  goodness = calc_sphere_fit_goodness(read_x, read_y, read_z, 1000, calib);
  Router::println("Sphere fit goodness of current calibration: " + String(goodness, 4));
}

// Router commands -------------------------------------------------------------
// void mag_rawprint(const char *) {
//   double mx, my, mz;
//   read_xyz(mx, my, mz);
//   Router::println(String(mx, 6) + "," + String(my, 6) + "," + String(mz, 6));
// }

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

void write_samples(const char *filename) {
  if (filename == nullptr) {
    Router::print("Call with filename to save to.\n");
    return;
  }
  collect_samples();
  // write raw data to file
  File f = SDCard::open(filename, FILE_WRITE | O_TRUNC | O_CREAT);
  if (!f) {
    return;
  }
  for (int i = 0; i < 1000; i++) {
    f.println(String(read_x[i]) + "," + String(read_y[i]) + "," + String(read_z[i]));
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

void hard_reset(const char *) {
  mag.performResetOperation();
  delay(100);
  Router::println("Magnetometer hard reset performed.");
}

void do_instant_calib(const char *) {
  hard_reset(nullptr);
  mag.performSetOperation();
  int sX, sY, sZ;
  get_centered_reading(sX, sY, sZ);
  get_centered_reading(sX, sY, sZ); // for some reason the demo does it twice apparently to avoid noise (??)
  Router::println("Set operation done. Values:");
  Router::println(String(sX) + "," + String(sY) + "," + String(sZ));

  mag.performResetOperation();
  int rX, rY, rZ;
  get_centered_reading(rX, rY, rZ);
  get_centered_reading(rX, rY, rZ);
  Router::println("Reset operation done. Values:");
  Router::println(String(rX) + "," + String(rY) + "," + String(rZ));

  // hard iron offset is average of set and reset
  calibration cnew;
  cnew.hard_x = (sX + rX) / 2.0;
  cnew.hard_y = (sY + rY) / 2.0;
  cnew.hard_z = (sZ + rZ) / 2.0;
  // no soft iron correction
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cnew.soft[i][j] = (i == j) ? 1.0 : 0.0; // identity matrix
    }
  }
  calib = cnew; // set new calibration
  Router::println("Instant calibration done.");
  print_calibration(nullptr);
}

void do_simple_calib(const char *) {
  hard_reset(nullptr);

  Router::println("Starting simple calibration.");

  collect_samples();
  // compute hard iron offsets as average of min and max
  auto min_max = [](const double *data, int n, double &minv, double &maxv) {
    minv = data[0];
    maxv = data[0];
    for (int i = 1; i < n; i++) {
      if (data[i] < minv)
        minv = data[i];
      if (data[i] > maxv)
        maxv = data[i];
    }
  };
  double min_x, min_y, min_z, max_x, max_y, max_z;
  min_max(read_x, 1000, min_x, max_x);
  min_max(read_y, 1000, min_y, max_y);
  min_max(read_z, 1000, min_z, max_z);

  calibration cnew; // new calibration
  cnew.hard_x = (min_x + max_x) / 2.0;
  cnew.hard_y = (min_y + max_y) / 2.0;
  cnew.hard_z = (min_z + max_z) / 2.0;
  // no soft iron correction
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cnew.soft[i][j] = (i == j) ? 1.0 : 0.0; // identity matrix
    }
  }

  double goodness = calc_sphere_fit_goodness(read_x, read_y, read_z, 1000, cnew);
  Router::println("Sphere fit goodness after simple calibration: " + String(goodness, 4));

  calib = cnew; // set new calibration
  Router::println("Simple calibration done.");
  print_calibration(nullptr);
}

void custom_calib(const char *) {
  auto parse_doubles = [](const String &str, double *vals, int count) { // sscanf doesnt handle doubles. 5 minutes of debugging resulted in that conclusion.
    size_t pos = 0;
    for (int i = 0; i < count; i++) {
      int next = str.indexOf(' ', pos);
      if (next == -1)
        next = str.length();
      if (pos >= str.length())
        return false;
      vals[i] = str.substring(pos, next).toDouble();
      pos = next + 1;
    }
    return true;
  };

  Router::println("Enter hard iron offsets separated by spaces (x y z): ");
  String line = Router::read(100);
  line.trim();

  double vals[3];
  if (!parse_doubles(line, vals, 3)) {
    Router::println("Error parsing input. Expected 3 numbers.");
    return;
  }
  calib.hard_x = vals[0];
  calib.hard_y = vals[1];
  calib.hard_z = vals[2];

  Router::println("Enter soft iron correction matrix (or hit enter for identity): ");
  String matline = Router::read(200);
  matline.trim();

  if (matline.length() == 0) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        calib.soft[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
    Router::println("Set soft iron correction matrix to identity.");
  } else {
    double m[9];
    if (!parse_doubles(matline, m, 9)) {
      Router::println("Error parsing input. Expected 9 numbers.");
      return;
    }
    for (int i = 0; i < 9; i++) {
      calib.soft[i / 3][i % 3] = m[i];
    }
    Router::println("Set soft iron correction matrix to input values.");
  }

  print_calibration(nullptr);
}

void show_centered_reading(const char *) {
  for (int i = 0; i < 100; i++) {
    int mx, my, mz;
    get_centered_reading(mx, my, mz);
    Router::println(String(mx) + "," + String(my) + "," + String(mz));
    delay(100);
  }
}

// // todo: change to load from file.
// void reset_calib(const char *) {
//   // calib = matlab_calib;
//   Router::println("Calibration reset to matlab calibration.");
//   print_calibration(nullptr);
// }

void save_calib(const char *filename) {
  if (filename == nullptr) {
    filename = main_calib_name;
  }
  Router::println("Saving calibration to " + String(filename));
  SDCard::write_bytes(filename, (uint8_t *)&calib, sizeof(calibration));
}

void load_calib(const char *filename) {
  if (filename == nullptr) {
    filename = main_calib_name;
  }
  Router::println("Loading calibration from " + String(filename));
  SDCard::load_bytes(filename, (uint8_t *)&calib, sizeof(calibration));
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
    while (true) {
      Router::println("Magnetometer not found, reboot once magnetometer connected...");
      delay(1000);
    }
  }
  mag.softReset();

  // Router::add({mag_rawprint, "mag_raw"});
  Router::add({mag_heading, "mag_heading"});

  Router::add({print_calibration, "mag_print_calib"});
  Router::add({write_samples, "mag_write_samples"});
  Router::add({do_simple_calib, "mag_do_simple_calib"}); // todo: add a way to save samples to a file when doing simple calib
  Router::add({do_instant_calib, "mag_do_instant_calib"});
  Router::add({custom_calib, "mag_custom_calib"});
  Router::add({hard_reset, "mag_hard_reset"});
  Router::add({show_centered_reading, "mag_show_centered"});
  Router::add({no_calib, "mag_no_calib"});
  Router::add({save_calib, "mag_save_calib"});
  Router::add({load_calib, "mag_load_calib"});

  do_instant_calib(nullptr);

  Router::println("Magnetometer initialized.");
}

} // namespace Mag
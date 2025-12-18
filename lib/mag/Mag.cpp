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

bool get_centered_reading(int &mx, int &my, int &mz) {
  uint32_t rawValueX, rawValueY, rawValueZ;
  if (!mag.measure(&rawValueX, &rawValueY, &rawValueZ)) {
    return false;
  }
  // mag.getMeasurementXYZ(&rawValueX, &rawValueY, &rawValueZ);

  mx = (int)rawValueX - 131072;
  my = (int)rawValueY - 131072;
  mz = (int)rawValueZ - 131072;
  return true;
}

bool isMeasurementReady() {
  return mag.isMeasurementReady();
}

bool beginMeasurement() {
  return mag.beginMeasurement();
}

// do not use this function in time-critical code
bool get_centered_reading_blocking(int &mx, int &my, int &mz) {
  mag.beginMeasurement();
  delay(1);
  while (!mag.isMeasurementReady()) {
    delay(1);
  }
  return get_centered_reading(mx, my, mz);
}

// values may be stale!!!
bool read_xyz(double &mx, double &my, double &mz) {
  int rx, ry, rz;

  if (!get_centered_reading(rx, ry, rz)) {
    return false;
  }
  mx = (double)rx;
  my = (double)ry;
  mz = (double)rz;
  apply_calibration(mx, my, mz);

  double half = 131072.0; // all values after dividing are in +/- 1.0 range (sensor is 18 bit)
  mx /= half;
  my /= half;
  mz /= half;
  return true;
}

bool read_xyz_normalized(double &mx, double &my, double &mz) {
  if (!read_xyz(mx, my, mz)) {
    return false;
  }

  double sqrt_ssq = sqrt(mx * mx + my * my + mz * mz);
  mx /= sqrt_ssq;
  my /= sqrt_ssq;
  mz /= sqrt_ssq;
  return true;
}

double get_heading() {
  double mx, my, mz;
  if (!read_xyz(mx, my, mz)) {
    return 0.0;
  }
  // Magnetic north is oriented with the Y axis
  double heading = atan2(mx, -my);
  // atan2 returns a value between +PI and -PI
  // Convert to degrees
  heading /= PI;
  heading *= 180;
  heading += 180;
  return heading;
}

// does not modify input arrays. returns a percentage goodness.
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
  return 100 * (1 - stdev / avg_magnitude); // relative standard deviation
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
    if (!get_centered_reading_blocking(mx, my, mz)) {
      Router::println("Mag read failed. collect_samples failed.");
      return;
    }
    read_x[i] = (double)mx;
    read_y[i] = (double)my;
    read_z[i] = (double)mz;
    delay(100);
    if (i % 10 == 0) { // every second, print progress
      Router::mprintln(i, " samples recorded ", i / 1000.0 * 100.0, "%");
    }
  }
  Router::println("Done recording calibration data.");
  double goodness = calc_sphere_fit_goodness(read_x, read_y, read_z, 1000, identity_calib);
  Router::mprintln("Sphere fit goodness of raw data: ", goodness, "%");
  goodness = calc_sphere_fit_goodness(read_x, read_y, read_z, 1000, calib);
  Router::mprintln("Sphere fit goodness of current calibration: ", goodness, "%");
}

// Router commands -------------------------------------------------------------
// void mag_rawprint() {
//   double mx, my, mz;
//   read_xyz(mx, my, mz);
//   Router::println(String(mx, 6) + "," + String(my, 6) + "," + String(mz, 6));
// }

void mag_heading() {
  while (!Serial.available()) {
    double heading = get_heading();
    Router::println(heading);
  }
}

void mag_test_read_time() {
  unsigned long read_time = 0;

  const int count = 5000;
  int count_non_stale = 0;
  mag.beginMeasurement();
  delay(1); // delay 1 ms to give the magnetometer time to measure before the first loop iteration

  unsigned long start_time = micros();
  for (int i = 0; i < count; ++i) {
    unsigned long read_start_micros = micros();
    if (mag.isMeasurementReady()) {
      double mx, my, mz;
      read_xyz(mx, my, mz);

      mag.beginMeasurement();

      ++count_non_stale;
    }

    unsigned long delta = micros() - read_start_micros;
    read_time += delta;

    if (delta < 1000) {
      delayMicroseconds(1000 - delta);
    }
  }
  double average_ms = (double)(micros() - start_time) / count / 1000.0;
  Router::printf("Test concluded.\nAverage loop time (ms): %lf\nAverage time spend reading per iteration (ms): %lf\nCount Reads: %d\n", average_ms, read_time / 1000.0 / count, count_non_stale);
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
    f.print(read_x[i]);
    f.print(",");
    f.print(read_y[i]);
    f.print(",");
    f.println(read_z[i]);
  }
  f.close();
  Router::mprintln("Done writing calibration data to ", filename);
  Router::println("Use final.m in mag_calib/ to compute calibration parameters");
}

void print_calibration() {
  Router::println("Current calibration:");
  Router::println("Hard iron offsets:");

  Router::printf("x: %.4f y: %.4f z: %.4f\n", calib.hard_x, calib.hard_y, calib.hard_z);

  Router::println("Soft iron correction matrix:");
  for (int i = 0; i < 3; i++) {
    Router::printf("%.4f %.4f %.4f\n", calib.soft[i][0], calib.soft[i][1], calib.soft[i][2]);
  }
}

void hard_reset() {
  mag.performResetOperation();
  delay(100);
  Router::println("Magnetometer hard reset performed.");
}

void do_instant_calib() {
  hard_reset();
  mag.performSetOperation();
  int sX, sY, sZ;
  bool success = get_centered_reading_blocking(sX, sY, sZ);
  success = success && get_centered_reading_blocking(sX, sY, sZ); // for some reason the demo does it twice apparently to avoid noise (??)
  if (!success) {
    Router::println("Set operation failed.");
    return;
  }
  Router::println("Set operation done. Values:");
  Router::mprintln(sX, ",", sY, ",", sZ);

  mag.performResetOperation();
  int rX, rY, rZ;
  success = get_centered_reading_blocking(rX, rY, rZ);
  success = success && get_centered_reading_blocking(rX, rY, rZ);

  if (!success) {
    Router::println("Reset operation on mag failed. Instant calibration failed.");
    return;
  }

  Router::println("Reset operation done. Values:");
  Router::mprintln(rX, ",", rY, ",", rZ);

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
  print_calibration();
}

void do_simple_calib() {
  hard_reset();

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
  Router::mprintln("Sphere fit goodness after simple calibration: ", goodness, "%");

  calib = cnew; // set new calibration
  Router::println("Simple calibration done.");
  print_calibration();
}

void custom_calib() {
  Router::println("Enter hard iron offsets separated by spaces (x y z): ");
  char *line = Router::read();

  double vals[3];
  if (!Router::parse_doubles(line, vals, 3)) {
    Router::println("Error parsing input. Expected 3 numbers.");
    return;
  }
  calib.hard_x = vals[0];
  calib.hard_y = vals[1];
  calib.hard_z = vals[2];

  Router::println("Enter soft iron correction matrix (or hit enter for identity): ");
  char *matline = Router::read();

  if (matline[0] == '\0') { // len is 0
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        calib.soft[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
    Router::println("Set soft iron correction matrix to identity.");
  } else {
    double m[9];
    if (!Router::parse_doubles(matline, m, 9)) {
      Router::println("Error parsing input. Expected 9 numbers.");
      return;
    }
    for (int i = 0; i < 9; i++) {
      calib.soft[i / 3][i % 3] = m[i];
    }
    Router::println("Set soft iron correction matrix to input values.");
  }

  print_calibration();
}

void show_centered_reading() {
  for (int i = 0; i < 100; i++) {
    int mx, my, mz;
    if (!get_centered_reading_blocking(mx, my, mz)) {
      Router::println("Get centered reading failed.");
      return;
    }
    Router::mprintln(mx, ",", my, ",", mz);
    delay(100);
  }
}

void show_normalized_reading() {
  mag.beginMeasurement();
  delay(1);
  while (!Serial.available()) {
    if (mag.isMeasurementReady()) {
      double mx, my, mz;
      if (read_xyz_normalized(mx, my, mz)) {
        Router::printf("%lf, %lf, %lf\n", mx, my, mz);
      }
      mag.beginMeasurement();
    }
    delay(5);
  }

  while (Serial.read() != '\n')
    ;
}

void save_calib(const char *filename) {
  if (filename == nullptr) {
    filename = main_calib_name;
  }
  Router::mprintln("Saving calibration to ", filename);
  SDCard::write_bytes(filename, (uint8_t *)&calib, sizeof(calibration));
}

void load_calib(const char *filename) {
  if (filename == nullptr) {
    filename = main_calib_name;
  }
  Router::mprintln("Loading calibration from ", filename);
  SDCard::load_bytes(filename, (uint8_t *)&calib, sizeof(calibration));
  print_calibration();
}

void no_calib() {
  calib = identity_calib;
  Router::println("Calibration set to identity (no calibration).");
  print_calibration();
}
// -----------------------------------------------------------------------------

void mag_record_test(const char *arg) {
  int old_filter_bw = mag.getFilterBandwidth();
  int new_filter_bw = atoi(arg);
  if (!new_filter_bw) {
    new_filter_bw = old_filter_bw;
  }

  mag.setFilterBandwidth(new_filter_bw);
  Serial.println("<<< CSV BEGIN >>>");
  Serial.println();
  Serial.println();
  Serial.print(new_filter_bw);
  Serial.print("\nTime (s),X reading,Y reading,Z reading\n");

  unsigned long start_micros = micros();

  while (!Serial.available()) {

    double x, y, z;
    x = y = z = 0;
    mag.beginMeasurement();
    delay(1);
    while (!mag.isMeasurementReady()) {
      delayMicroseconds(500);
    }

    double t = (micros() - start_micros) / 1000000.0;
    read_xyz_normalized(x, y, z);

    Router::print(t, 6);
    Router::print(',');
    Router::print(x, 8);
    Router::print(',');
    Router::print(y, 8);
    Router::print(',');
    Router::print(z, 8);
    Router::print('\n');
    delay(1);
  }

  while (Serial.read() != '\n')
    ;

  Serial.println();
  Serial.println();
  Serial.println("<<< CSV END >>>");

  mag.setFilterBandwidth(old_filter_bw);
}

void begin() {
  Wire.begin();
  Wire.setClock(400000);
  // Initialize the magnetometer
  if (!mag.begin()) {
    while (true) {
      Router::println("Magnetometer not found, reboot once magnetometer connected...");
      delay(1000);
    }
  }
  mag.softReset();

  // mag.setFilterBandwidth(800); // 0.5ms measurement time
  mag.setFilterBandwidth(400); // 2ms measurement time

  // supposedly this will prevent sensor drift
  mag.setPeriodicSetSamples(25);
  mag.enablePeriodicSet();

  // Router::add({mag_rawprint, "mag_raw"});
  Router::add({mag_heading, "mag_heading"});

  Router::add({print_calibration, "mag_print_calib"});
  Router::add({show_normalized_reading, "mag_show_normalized_reading"});
  Router::add({mag_test_read_time, "mag_test_read_time"});
  Router::add({write_samples, "mag_write_samples"});
  Router::add({do_simple_calib, "mag_do_simple_calib"}); // todo: add a way to save samples to a file when doing simple calib
  Router::add({do_instant_calib, "mag_do_instant_calib"});
  Router::add({custom_calib, "mag_custom_calib"});
  Router::add({hard_reset, "mag_hard_reset"});
  Router::add({show_centered_reading, "mag_show_centered"});
  Router::add({no_calib, "mag_no_calib"});
  Router::add({save_calib, "mag_save_calib"});
  Router::add({load_calib, "mag_load_calib"});
  Router::add({mag_record_test, "mag_record_test"});

  do_instant_calib();

  Router::println("Magnetometer initialized.");
}

} // namespace Mag
#include <Mag.h>

#include <Router.h>
#include <SDCard.h>
#include <SPI.h>

#include "fc_pins.h"

#include <SparkFun_MMC5983MA_Arduino_Library.h>

const char *Mag::CALIB_FMT = "MAG-%d";

namespace Mag {

SFE_MMC5983MA MAGs[MAG_COUNT];

struct calibration {
  double hard_x;
  double hard_y;
  double hard_z;
  double soft[3][3];
};

calibration identity_calib = {0.0, 0.0, 0.0, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
calibration calibs[MAG_COUNT]{};

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

void apply_calibration(double &x, double &y, double &z, int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return;

  apply_calibration(x, y, z, calibs[mag_index]);
}

bool get_centered_reading(int m[3], int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return false;

  uint32_t rawValueX, rawValueY, rawValueZ;

  if (!MAGs[mag_index].measure(&rawValueX, &rawValueY, &rawValueZ)) {
    return false;
  }

  m[0] = (int)rawValueX - 131072;
  m[1] = (int)rawValueY - 131072;
  m[2] = (int)rawValueZ - 131072;
  return true;
}

// check if all magnetometers are ready for measurement
bool isMeasurementReady() {
  for (int i = 0; i < MAG_COUNT; ++i) {
    if (!MAGs[i].isMeasurementReady())
      return false;
  }
  return true;
}

// begin measurement on all magnetometers
void beginMeasurement() {
  for (int i = 0; i < MAG_COUNT; ++i) {
    MAGs[i].beginMeasurement();
  }
  return;
}

// this function is REALLY slow btw !!
bool get_centered_reading_blocking(int m[3], int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return false;

  SFE_MMC5983MA &mag = MAGs[mag_index];

  mag.beginMeasurement();
  delay(1);
  while (!mag.isMeasurementReady()) {
    delay(1);
  }
  return get_centered_reading(m, mag_index);
}

// values may be stale!!! proper usage is to call beginMeasurement() and then isMeasurementReady() to make sure a new measurement is waiting to be read
bool read_xyz(double m[3], int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return false;

  int r[3]; // store raw, centered readings

  if (!get_centered_reading(r, mag_index)) {
    return false;
  }
  m[0] = (double)r[0];
  m[1] = (double)r[1];
  m[2] = (double)r[2];
  apply_calibration(m[0], m[1], m[2], mag_index);

  double half = 131072.0; // all values after dividing are in +/- 1.0 range (sensor is 18 bit)
  m[0] /= half;
  m[1] /= half;
  m[2] /= half;
  return true;
}

bool read_xyz_all(double m[MAG_COUNT][3]) {
  for (int i = 0; i < MAG_COUNT; ++i) {
    if (!read_xyz(m[i], i))
      return false;
  }

  return true;
}

// this function reads all magnetometers, normalizes their readings, and converts their readings into a single reading for each axis
bool read_normalized_fused(double m[3]) {
  double measurements[MAG_COUNT][3];

  if (!read_xyz_normalized_all(measurements)) {
    return false;
  }

  // average together the readings for each axis
  for (int i = 0; i < 3; ++i) {
    double accumulator = 0;
    for (int j = 0; j < MAG_COUNT; ++j) {
      accumulator += measurements[j][i];
    }

    m[i] = accumulator / MAG_COUNT;
  }

  return true;
}

bool read_xyz_normalized(double m[3], int mag_index) {
  if (!read_xyz(m, mag_index)) {
    return false;
  }

  double sqrt_ssq = sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  m[0] /= sqrt_ssq;
  m[1] /= sqrt_ssq;
  m[2] /= sqrt_ssq;
  return true;
}

bool read_xyz_normalized_all(double m[MAG_COUNT][3]) {
  for (int i = 0; i < MAG_COUNT; ++i) {
    if (!read_xyz_normalized(m[i], i))
      return false;
  }

  return true;
}

double get_heading(int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT) {
    return 0;
  }

  double m[3];
  if (!read_xyz(m, mag_index)) {
    return 0.0;
  }
  // Magnetic north is oriented with the Y axis
  double heading = atan2(m[0], -m[1]);
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
void collect_samples(int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return;

  Router::println("Collecting calibration data...");
  Router::println("Move the sensor around in all orientations until done. Waiting 5 seconds to start...");
  delay(5000);
  // record 1000 centered readings
  for (int i = 0; i < 1000; i++) {
    int m[3];
    if (!get_centered_reading_blocking(m, mag_index)) {
      Router::println("Mag read failed. collect_samples failed.");
      return;
    }
    read_x[i] = (double)m[0];
    read_y[i] = (double)m[1];
    read_z[i] = (double)m[2];
    delay(100);
    if (i % 10 == 0) { // every second, print progress
      Router::mprintln(i, " samples recorded ", i / 1000.0 * 100.0, "%");
    }
  }
  Router::println("Done recording calibration data.");
  double goodness = calc_sphere_fit_goodness(read_x, read_y, read_z, 1000, identity_calib);
  Router::mprintln("Sphere fit goodness of raw data: ", goodness, "%");
  goodness = calc_sphere_fit_goodness(read_x, read_y, read_z, 1000, calibs[mag_index]);
  Router::mprintln("Sphere fit goodness of current calibration: ", goodness, "%");
}

// Router commands -------------------------------------------------------------
// void mag_rawprint() {
//   double mx, my, mz;
//   read_xyz(mx, my, mz);
//   Router::println(String(mx, 6) + "," + String(my, 6) + "," + String(mz, 6));
// }

void mag_heading(const char *arg) {
  int mag_index = atoi(arg);

  if (mag_index < 0 || mag_index >= MAG_COUNT)
    mag_index = 0;

  Router::printf("Outputting mag heading measurement on mag %d. Press enter to continue...", mag_index);
  while (!Serial.available()) {
    double heading = get_heading(mag_index);
    Router::println(heading);
  }

  while (Serial.read() != '\n')
    ;
}

// test read time for all three mags as if they are in the main control loop; only time the portion of the code where the mag is being interacted with
void mag_test_read_time() {
  unsigned long read_time = 0;

  const int count = 5000;
  int count_non_stale = 0;
  beginMeasurement();
  delay(1); // delay 1 ms to give the magnetometer time to measure before the first loop iteration

  unsigned long start_time = micros();
  for (int i = 0; i < count; ++i) {
    unsigned long read_start_micros = micros();
    if (isMeasurementReady()) {
      double m[MAG_COUNT][3];
      read_xyz_all(m);

      beginMeasurement();

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

void write_samples(const char *args) {
  static char filename[30];
  int mag_index;
  if (sscanf(args, "%d %s", &mag_index, filename) != 2) {
    Router::print("Usage: mag_write_samples <mag_index> <filename>\n");
    return;
  }
  collect_samples(mag_index);
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

void print_calibration(int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    mag_index = 0;
  Router::printf("Current calibration for mag %d:\n", mag_index);
  Router::println("Hard iron offsets:");

  calibration &calib = calibs[mag_index];

  Router::printf("x: %.4f y: %.4f z: %.4f\n", calib.hard_x, calib.hard_y, calib.hard_z);

  Router::println("Soft iron correction matrix:");
  for (int i = 0; i < 3; i++) {
    Router::printf("%.4f %.4f %.4f\n", calib.soft[i][0], calib.soft[i][1], calib.soft[i][2]);
  }
}

void print_calibration_cmd(const char *args) {
  print_calibration(atoi(args));
}

void hard_reset() {
  for (int i = 0; i < MAG_COUNT; ++i) {
    MAGs[i].performResetOperation();
  }
  delay(100);
  Router::println("Magnetometer hard reset performed.");
}

void do_instant_calib(int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return;
  Router::printf("Performing instant calibration on magnetometer %d\n", mag_index);

  SFE_MMC5983MA &mag = MAGs[mag_index];
  mag.performResetOperation();
  mag.performSetOperation();
  int s[3];
  bool success = get_centered_reading_blocking(s, mag_index);
  success = success && get_centered_reading_blocking(s, mag_index); // for some reason the demo does it twice apparently to avoid noise (??)
  if (!success) {
    Router::println("Set operation failed.");
    return;
  }
  Router::println("Set operation done. Values:");
  Router::mprintln(s[0], ",", s[1], ",", s[2]);

  mag.performResetOperation();
  int r[3];
  success = get_centered_reading_blocking(r, mag_index);
  success = success && get_centered_reading_blocking(r, mag_index);

  if (!success) {
    Router::println("Reset operation on mag failed. Instant calibration failed.");
    return;
  }

  Router::println("Reset operation done. Values:");
  Router::mprintln(r[0], ",", r[1], ",", r[2]);

  // hard iron offset is average of set and reset
  calibration cnew;
  cnew.hard_x = (s[0] + r[0]) / 2.0;
  cnew.hard_y = (s[1] + r[1]) / 2.0;
  cnew.hard_z = (s[2] + r[2]) / 2.0;
  // no soft iron correction
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cnew.soft[i][j] = (i == j) ? 1.0 : 0.0; // identity matrix
    }
  }
  calibs[mag_index] = cnew; // set new calibration
  Router::println("Instant calibration done.");
  print_calibration(mag_index);
}

void do_instant_calib_cmd(const char *arg) {
  do_instant_calib(atoi(arg));
}

void do_simple_calib(const char *args) {
  int mag_index = atoi(args);

  if (mag_index < 0 || mag_index >= MAG_COUNT)
    mag_index = 0;

  MAGs[mag_index].performResetOperation();

  Router::println("Starting simple calibration.");

  collect_samples(mag_index);
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

  calibs[mag_index] = cnew; // set new calibration
  Router::println("Simple calibration done.");
  print_calibration(mag_index);
}

void custom_calib(const char *args) {
  int mag_index = atoi(args);

  if (mag_index < 0 || mag_index >= MAG_COUNT)
    mag_index = 0;

  Router::printf("Custom calib entry for mag %d\n", mag_index);
  Router::println("Enter hard iron offsets separated by spaces (x y z): ");
  char *line = Router::read();

  double vals[3];
  if (!Router::parse_doubles(line, vals, 3)) {
    Router::println("Error parsing input. Expected 3 numbers.");
    return;
  }

  calibration &calib = calibs[mag_index];
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

  print_calibration(mag_index);
}

void show_centered_reading(const char *args) {
  int mag_index = atoi(args);

  if (mag_index < 0 || mag_index >= MAG_COUNT)
    mag_index = 0;

  for (int i = 0; i < 100; i++) {
    int m[3];
    if (!get_centered_reading_blocking(m, mag_index)) {
      Router::println("Get centered reading failed.");
      return;
    }
    Router::mprintln(m[0], ",", m[1], ",", m[2]);
    delay(100);
  }
}

void show_normalized_reading(const char *args) {
  int mag_index = atoi(args);

  if (mag_index < 0 || mag_index >= MAG_COUNT)
    mag_index = 0;

  SFE_MMC5983MA &mag = MAGs[mag_index];
  while (!Serial.available()) {
    if (mag.isMeasurementReady()) {
      double m[3];
      if (read_xyz_normalized(m, mag_index)) {
        Router::printf("%lf, %lf, %lf\n", m[0], m[1], m[2]);
      }
      mag.beginMeasurement();
    }
    delay(5);
  }

  while (Serial.read() != '\n')
    ;
}

void save_calib(int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return;

  char filename[30];

  snprintf(filename, sizeof(filename), Mag::CALIB_FMT, mag_index);
  Router::mprintln("Saving calibration to ", filename);
  SDCard::write_bytes(filename, (uint8_t *)&calibs[mag_index], sizeof(calibration));
}

void save_calib_cmd(const char *arg) {
  if (arg == nullptr)
    return;
  save_calib(atoi(arg));
}

void load_calib(int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return;

  char filename[30];

  snprintf(filename, sizeof(filename), Mag::CALIB_FMT, mag_index);

  Router::mprintln("Loading calibration from ", filename);
  SDCard::load_bytes(filename, (uint8_t *)&calibs[mag_index], sizeof(calibration));
  print_calibration(mag_index);
}

void load_calib_cmd(const char *arg) {
  if (arg == nullptr)
    return;
  load_calib(atoi(arg));
}

void no_calib(int mag_index) {
  if (mag_index < 0 || mag_index >= MAG_COUNT)
    return;

  calibs[mag_index] = identity_calib;
  Router::printf("Mag %d calibration set to identity (no calibration).\n", mag_index);
  print_calibration(mag_index);
}

void no_calib_cmd(const char *arg) {
  no_calib(atoi(arg));
}
// -----------------------------------------------------------------------------

void mag_record_test(const char *arg) {
  int new_filter_bw;
  int mag_index;
  if (arg == nullptr || sscanf(arg, "%d %d", &mag_index, &new_filter_bw) != 2 || (mag_index < 0 || mag_index >= MAG_COUNT)) {
    Router::print("Invalid command usage. Proper usage: mag_record_test <mag_index> <new_filter_bw>");
  }

  SFE_MMC5983MA &mag = MAGs[mag_index];
  int old_filter_bw = mag.getFilterBandwidth();
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

    double m[3];
    mag.beginMeasurement();
    delay(1);
    while (!mag.isMeasurementReady()) {
      delayMicroseconds(500);
    }

    double t = (micros() - start_micros) / 1000000.0;
    read_xyz_normalized(m, mag_index);

    Router::print(t, 6);
    for (int i = 0; i < 3; ++i) {
      Router::print(',');
      Router::print(m[i], 8);
    }
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

void cmd_log_fused() {}

void begin() {

  // initialize each calibration and magnetometer sequentially
  for (int i = 0; i < MAG_COUNT; ++i) {

    calibs[i] = identity_calib; // set to identity calib for now, will be overwritten later when instant calib is done/or calib is loaded from file

    // if (!MAGs[i].begin(MAG_CS[i], fc_spi)) {
    //   while (true) {
    //     Router::printf("Magnetometer %d not found, reboot once magnetometer connected...\n", i);
    //     delay(1000);
    //   }
    // }

    while (!MAGs[i].begin(MAG_CS[i], fc_spi)) {
      Router::printf("Magnetometer %d not found, reboot once magnetometer connected...\n", i);
      delay(1000);
    }

    MAGs[i].softReset();

    // mag.setFilterBandwidth(800); // 0.5ms measurement time
    MAGs[i].setFilterBandwidth(400); // 2ms measurement time

    // supposedly this will prevent sensor drift
    MAGs[i].setPeriodicSetSamples(25);
    MAGs[i].enablePeriodicSet();

    do_instant_calib(i);
  }

  // Router::add({mag_rawprint, "mag_raw"});
  Router::add({mag_heading, "mag_heading"});

  Router::add({print_calibration_cmd, "mag_print_calib"});
  Router::add({show_normalized_reading, "mag_show_normalized_reading"});
  Router::add({mag_test_read_time, "mag_test_read_time"});
  Router::add({write_samples, "mag_write_samples"});
  Router::add({do_simple_calib, "mag_do_simple_calib"}); // todo: add a way to save samples to a file when doing simple calib
  Router::add({do_instant_calib_cmd, "mag_do_instant_calib"});
  Router::add({custom_calib, "mag_custom_calib"});
  Router::add({hard_reset, "mag_hard_reset"});
  Router::add({show_centered_reading, "mag_show_centered"});
  Router::add({no_calib_cmd, "mag_no_calib"});
  Router::add({save_calib_cmd, "mag_save_calib"});
  Router::add({load_calib_cmd, "mag_load_calib"});
  Router::add({mag_record_test, "mag_record_test"});

  Router::println("Magnetometer initialized.");
}

} // namespace Mag
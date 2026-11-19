#include <Mag.h>

#include <CommandRouter.h>
#include <CommsSerial.h>
#include <Mag_Calib.h>
#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h>

const char *Mag::main_calib_name = "MCALIB.BIN";

SFE_MMC5983MA mag;

namespace Mag {

calibration identity_calib = {0.0, 0.0, 0.0, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
calibration calib = identity_calib; // start with identity calibration

void apply_calibration(float &x, float &y, float &z, const calibration &c) {
  x -= c.hard_x;
  y -= c.hard_y;
  z -= c.hard_z;

  float cal_x = c.soft[0][0] * x + c.soft[0][1] * y + c.soft[0][2] * z;
  float cal_y = c.soft[1][0] * x + c.soft[1][1] * y + c.soft[1][2] * z;
  float cal_z = c.soft[2][0] * x + c.soft[2][1] * y + c.soft[2][2] * z;

  x = cal_x;
  y = cal_y;
  z = cal_z;
}

void apply_calibration(float &x, float &y, float &z) {
  apply_calibration(x, y, z, calib);
}

bool get_centered_reading(float &mx, float &my, float &mz) {
  uint32_t rawValueX, rawValueY, rawValueZ;
  if (!mag.measure(&rawValueX, &rawValueY, &rawValueZ)) {
    return false;
  }

  // this normalization (based on mag register width) keeps soft iron to be basically identity matrix
  mx = ((int)rawValueX - 131072) / 16384.0;
  my = ((int)rawValueY - 131072) / 16384.0;
  mz = ((int)rawValueZ - 131072) / 16384.0;

  return true;
}

bool isMeasurementReady() {
  return mag.isMeasurementReady();
}

bool beginMeasurement() {
  return mag.beginMeasurement();
}

// do not use this function in time-critical code
bool get_centered_reading_blocking(float &mx, float &my, float &mz) {
  mag.beginMeasurement();
  delay(1);
  while (!mag.isMeasurementReady()) {
    delay(1);
  }
  return get_centered_reading(mx, my, mz);
}

// values may be stale!!!
bool read_xyz_calibrated(float &mx, float &my, float &mz) {
  if (!get_centered_reading(mx, my, mz)) {
    return false;
  }
  apply_calibration(mx, my, mz);
  return true;
}

float get_heading() {
  float mx, my, mz;
  if (!read_xyz_calibrated(mx, my, mz)) {
    return 0.0;
  }
  // Magnetic north is oriented with the Y axis
  float heading = atan2(mx, -my);
  // atan2 returns a value between +PI and -PI
  // Convert to degrees
  heading /= PI;
  heading *= 180;
  heading += 180;
  return heading;
}

void mag_heading() {
  while (!CommsSerial.available()) {
    float heading = get_heading();
    CommsSerial.println(heading);
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
      float mx, my, mz;
      read_xyz_calibrated(mx, my, mz);

      mag.beginMeasurement();

      ++count_non_stale;
    }

    unsigned long delta = micros() - read_start_micros;
    read_time += delta;

    if (delta < 1000) {
      delayMicroseconds(1000 - delta);
    }
  }
  float average_ms = (float)(micros() - start_time) / count / 1000.0;
  CommsSerial.printf("Test concluded.\nAverage loop time (ms): %f\nAverage time spend reading per iteration (ms): %f\nCount Reads: %d\n", average_ms, read_time / 1000.0 / count, count_non_stale);
}

void print_calibration() {
  CommsSerial.println("Current calibration:");
  CommsSerial.println("Hard iron offsets:");

  CommsSerial.printf("x: %.4f y: %.4f z: %.4f\n", calib.hard_x, calib.hard_y, calib.hard_z);

  CommsSerial.println("Soft iron correction matrix:");
  for (int i = 0; i < 3; i++) {
    CommsSerial.printf("%.4f %.4f %.4f\n", calib.soft[i][0], calib.soft[i][1], calib.soft[i][2]);
  }
}

void hard_reset() {
  mag.performResetOperation();
  delay(100);
  CommsSerial.println("Magnetometer hard reset performed.");
}

void do_instant_calib() {
  hard_reset();
  mag.performSetOperation();
  float sX, sY, sZ;
  bool success = get_centered_reading_blocking(sX, sY, sZ);
  success = success && get_centered_reading_blocking(sX, sY, sZ); // for some reason the demo does it twice apparently to avoid noise (??)
  if (!success) {
    CommsSerial.println("Set operation failed.");
    return;
  }
  CommsSerial.println("Set operation done. Values:");
  CommsSerial.mprintln(sX, ",", sY, ",", sZ);

  mag.performResetOperation();
  float rX, rY, rZ;
  success = get_centered_reading_blocking(rX, rY, rZ);
  success = success && get_centered_reading_blocking(rX, rY, rZ);

  if (!success) {
    CommsSerial.println("Reset operation on mag failed. Instant calibration failed.");
    return;
  }

  CommsSerial.println("Reset operation done. Values:");
  CommsSerial.mprintln(rX, ",", rY, ",", rZ);

  // hard iron offset is average of set and reset
  calibration cnew;
  cnew.hard_x = (sX + rX) / 2.0;
  cnew.hard_y = (sY + rY) / 2.0;
  cnew.hard_z = (sZ + rZ) / 2.0;

  // no soft iron correction
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      cnew.soft[r][c] = identity_calib.soft[r][c];
    }
  }

  calib = cnew; // set new calibration
  CommsSerial.println("Instant calibration done.");
  print_calibration();
}

void do_simple_calib() {
  hard_reset();

  float read_x[CALIB_SAMPLE_COUNT], read_y[CALIB_SAMPLE_COUNT], read_z[CALIB_SAMPLE_COUNT];

  CommsSerial.println("Starting simple calibration.");
  CommsSerial.println("Collecting calibration data...");
  CommsSerial.println("Move the sensor around in all orientations until done. Waiting 5 seconds to start...");
  delay(5000);
  // record 1000 centered readings
  for (int i = 0; i < CALIB_SAMPLE_COUNT; i++) {
    if (!get_centered_reading_blocking(read_x[i], read_y[i], read_z[i])) {
      CommsSerial.println("Mag read failed. collect_samples failed.");
      return;
    }
    delay(50);
    if (i % 10 == 0) { // every second, print progress
      CommsSerial.mprintln(i, " samples recorded ", i / 1000.0 * 100.0, "%");
    }
  }
  CommsSerial.println("Done recording calibration data.");

  calib = MagCalib(read_x, read_y, read_z); // set new calibration
  CommsSerial.println("Simple calibration done.");
  print_calibration();
}

void custom_calib() {
  CommsSerial.println("Enter hard iron offsets separated by spaces (x y z): ");
  float vals[3];
  if (CommsSerial.scanf("%f %f %f", &vals[0], &vals[1], &vals[2]) != 3) {
    CommsSerial.println("Error parsing input. Expected 3 numbers.");
    return;
  }

  calib.hard_x = vals[0];
  calib.hard_y = vals[1];
  calib.hard_z = vals[2];

  CommsSerial.println("Enter soft iron correction matrix (or hit enter for identity): ");
  float m[9];
  if (CommsSerial.scanf("%f %f %f %f %f %f %f %f %f", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8], &m[9]) != 9) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        calib.soft[i][j] = identity_calib.soft[i][j];
      }
    }
    CommsSerial.println("Set soft iron correction matrix to identity.");
  } else {
    for (int i = 0; i < 9; i++) {
      calib.soft[i / 3][i % 3] = m[i];
    }
    CommsSerial.println("Set soft iron correction matrix to input values.");
  }

  print_calibration();
}

void show_centered_reading() {
  for (int i = 0; i < 100; i++) {
    float mx, my, mz;
    if (!get_centered_reading_blocking(mx, my, mz)) {
      CommsSerial.println("Get centered reading failed.");
      return;
    }
    CommsSerial.mprintln(mx, ",", my, ",", mz);
    delay(100);
  }
}

void show_normalized_reading() {
  mag.beginMeasurement();
  delay(1);
  while (!CommsSerial.available()) {
    if (mag.isMeasurementReady()) {
      float mx, my, mz;
      if (read_xyz_calibrated(mx, my, mz)) {
        CommsSerial.printf("%f, %f, %f\n", mx, my, mz);
      }
      mag.beginMeasurement();
    }
    delay(5);
  }

  while (CommsSerial.read() != '\n')
    ;
}

void no_calib() {
  calib = identity_calib;
  CommsSerial.println("Calibration set to identity (no calibration).");
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
  CommsSerial.println("<<< CSV BEGIN >>>");
  CommsSerial.println();
  CommsSerial.println();
  CommsSerial.print(new_filter_bw);
  CommsSerial.print("\nTime (s),X reading,Y reading,Z reading\n");

  unsigned long start_micros = micros();

  while (!CommsSerial.available()) {

    float x, y, z;
    x = y = z = 0;
    mag.beginMeasurement();
    delay(1);
    while (!mag.isMeasurementReady()) {
      delayMicroseconds(500);
    }

    float t = (micros() - start_micros) / 1000000.0;
    read_xyz_calibrated(x, y, z);

    CommsSerial.print(t, 6);
    CommsSerial.print(',');
    CommsSerial.print(x, 8);
    CommsSerial.print(',');
    CommsSerial.print(y, 8);
    CommsSerial.print(',');
    CommsSerial.print(z, 8);
    CommsSerial.print('\n');
    delay(1);
  }

  while (CommsSerial.read() != '\n')
    ;

  CommsSerial.println();
  CommsSerial.println();
  CommsSerial.println("<<< CSV END >>>");

  mag.setFilterBandwidth(old_filter_bw);
}

void begin() {
  Wire.begin();
  Wire.setClock(400000);
  // Initialize the magnetometer
  if (!mag.begin()) {
    while (true) {
      CommsSerial.println("Magnetometer not found, reboot once magnetometer connected...");
      delay(1000);
    }
  }
  mag.softReset();

  // mag.setFilterBandwidth(800); // 0.5ms measurement time
  mag.setFilterBandwidth(400); // 2ms measurement time

  // supposedly this will prevent sensor drift
  mag.setPeriodicSetSamples(25);
  mag.enablePeriodicSet();

  // CommandRouter::add(mag_rawprint, "mag_raw");
  CommandRouter::add(mag_heading, "mag_heading");

  CommandRouter::add(print_calibration, "mag_print_calib");
  CommandRouter::add(show_normalized_reading, "mag_show_normalized_reading");
  CommandRouter::add(mag_test_read_time, "mag_test_read_time");
  // CommandRouter::add({write_samples, "mag_write_samples");
  CommandRouter::add(do_simple_calib, "mag_do_simple_calib"); // todo: add a way to save samples to a file when doing simple calib
  CommandRouter::add(do_instant_calib, "mag_do_instant_calib");
  CommandRouter::add(custom_calib, "mag_custom_calib");
  CommandRouter::add(hard_reset, "mag_hard_reset");
  CommandRouter::add(show_centered_reading, "mag_show_centered");
  CommandRouter::add(no_calib, "mag_no_calib");
  // CommandRouter::add(save_calib, "mag_save_calib");
  // CommandRouter::add(load_calib, "mag_load_calib");
  CommandRouter::add(mag_record_test, "mag_record_test");

  do_instant_calib();

  CommsSerial.println("Magnetometer initialized.");
}

} // namespace Mag
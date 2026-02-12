#pragma once
#include "./Invn/Drivers/Icm406xx/Sensor_Event.h"
#include <SPI.h>

#include "fc_pins.h"

class IMU {
public:
  struct Measurement {
    double acc[3];  // units are function dependent
    double gyro[3]; // units are function dependent
  };

  // IMU count depends on the number of CS pins defined in fc_pins.h
  static const int IMU_COUNT = sizeof(IMU_CS) / sizeof(IMU_CS[0]);

  IMU();
  ~IMU();

  static void begin();
  static void calibrate_gyroscope_all();
  static void read_latest_all(Measurement output[IMU_COUNT]);
  static void read_latest_fused(Measurement *output);

private:
  struct SPI_Interface {
    int cs;
    SPIClass *spi;
  };

  struct Calib {
    double gyro_bias[3];             // deg/s
    double accel_correction_bias[3]; // g's
    double accel_correction_gain[3];
  };

  void read_latest(Measurement *output);
  void read_latest_no_calib(Measurement *output);

  void load_calib(const char *);
  void write_calib(const char *);

  void clear_calib();
  void load_custom_calib();

  static IMU IMUs[IMU_COUNT];

  Calib calib;

  int init(int cs, SPIClass *spi);

  int enable_accel();
  int enable_gyro();

  int disable_accelerometer();
  int disable_gyro();

  int read_accel_config(uint8_t *value);

  void read_latest_accel_raw(int16_t *);
  void read_latest_gyro_raw(int16_t *);

  int write_reg_mask(uint8_t addr, uint8_t mask, uint8_t val);

  SPI_Interface spi_interface;

  void *inv_icm;

  static void begin_transaction(SPI_Interface *);
  static void end_transaction(SPI_Interface *);

  static int read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
  static int write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

  // router command declarations
  static void cmd_calibrate_gyro();
  static void cmd_log_accel_for_calibration(const char *param);
  static void cmd_imu_log();
  static void cmd_load_custom_calib(const char *arg);
  static void cmd_load_calib(const char *arg);
  static void cmd_write_calib(const char *arg);
  static void cmd_output_calib(const char *arg);
  static void cmd_imu_speed_test();
  static void cmd_imu_log_fused();
};

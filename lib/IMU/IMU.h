#pragma once
#include "./Invn/Drivers/Icm406xx/Sensor_Event.h"
#include <SPI.h>

#define IMU_COUNT 1

namespace IMU {

struct Calib {
  double gyro_bias[3];
  double accel_correction_bias[3];
  double accel_correction_gain[3];
};

struct Data {
  double acc[3];  // units are function dependent
  double gyro[3]; // units are function dependent
};

struct SPI_Interface {
  int cs;
  SPIClass *spi;
};

int begin();
void calibrate_gyroscope();

class Sensor {
public:
  Sensor(int cs, SPIClass *spi);
  ~Sensor();
  int init();
  void read_latest(Data *output);
  void read_latest_no_calib(Data *output);

  int enable_accel();
  int enable_gyro();

  int disable_accelerometer();
  int disable_gyro();

  int read_accel_config(uint8_t *value);
  void read_latest_raw(int16_t *acc, int16_t *gyro);

  void calibrate_gyro();

  // void load_calib(const char *);
  // void write_calib(const char *);

  void clear_calib();
  void load_custom_calib();

  Calib calib;

  void read_latest_accel_raw(int16_t *);
  void read_latest_gyro_raw(int16_t *);

private:
  int write_reg_mask(uint8_t addr, uint8_t mask, uint8_t val);

  SPI_Interface spi_interface;

  void *inv_icm;
};

extern Sensor IMUs[IMU_COUNT];
} // namespace IMU

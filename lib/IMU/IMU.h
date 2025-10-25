#pragma once
#include "./Invn/Drivers/Icm406xx/Sensor_Event.h"
#include <SPI.h>

// TODO: add function to enable/disable gyro, accel
class IMU {
public:
  struct sensor_data {
    double acc[3];
    double gyro[3];
  };

  IMU(int cs, arduino::MbedSPI *spi);
  ~IMU();
  int begin();
  void read_latest(sensor_data *output);
  void read_latest_raw(sensor_data *output);

  int enable_accel();
  int enable_gyro();

  int disable_accelerometer();
  int disable_gyro();

  int read_accel_config(uint8_t *value);

  void calibrate_gyro();

  void load_calib(const char *);
  void write_calib(const char *);

  struct IMU_Calib {
    double gyro_bias[3];
    double accel_correction_bias[3];
    double accel_correction_gain[3];
  };

  void clear_calib();

  void load_custom_calib();

  struct SPI_Interface {
    int cs;
    arduino::MbedSPI *spi;
  };

  IMU_Calib calib;

private:
  void read_latest_accel_raw(int16_t *);
  void read_latest_gyro_raw(int16_t *);

  SPI_Interface spi_interface;

  void *inv_icm;
};

#pragma once
#include "./Invn/Drivers/Icm406xx/Sensor_Event.h"
#include <SPI.h>

#define IMU_COUNT 1

namespace IMU {

class Sensor {
public:
  struct Data {
    double acc[3];  // units are function dependent
    double gyro[3]; // units are function dependent
  };

  Sensor(int cs, arduino::MbedSPI *spi);
  ~Sensor();
  int begin();
  void read_latest(Data *output);
  void read_latest_no_calib(Data *output);

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

Sensor IMUs[IMU_COUNT];

int begin();
void calibrate_gyroscope();
void cmd_calibrate_gyro(const char *);
void cmd_imu_log(const char *);
void cmd_load_custom_calib(const char*);
void cmd_load_calib(const char*);
void cmd_write_calib(const char*);
void cmd_output_calib(const char*);

void cmd_log_accel_for_calibration(const char *_);
} // namespace IMU
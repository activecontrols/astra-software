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
  int read_fifo(sensor_data *output);
  int read_latest(sensor_data *output);

  int enable_accel();
  int enable_gyro();

  int disable_accelerometer();
  int disable_gyro();

  int read_accel_config(uint8_t* value);

  struct SPI_Interface {
    int cs;
    arduino::MbedSPI *spi;
  };

private:
  SPI_Interface spi_interface;

  void *inv_icm;

  struct acc {
    int64_t accel[3];
    int64_t gyro[3];
    uint16_t count;
  };

  acc accumulator;

  static void fifo_callback(void *ctx, inv_icm406xx_sensor_event_t *event);
};

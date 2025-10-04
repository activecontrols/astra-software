#include "GPS.h"
#include "Mag.h"
#include "Router.h"
#include "Prop.h"
#include "IMU.h"
#include "Router.h"
#include <Arduino.h>
#include <SPI.h>

#define IMU_CS D6

IMU imu(IMU_CS, &SPI);

int packets_read = 0;

void fixedpoint_to_float(int16_t *, float *, double, const uint8_t);

void read_sensor_packet(const char *_) {
  static char output_s[500];
  static char sens_mask_str[9];
  double* accel;
  double* gyro;
  float temperature_c;
  IMU::sensor_data sensor_data;
  sens_mask_str[8] = '\0';

  unsigned long start = micros();

  imu.read_latest(&sensor_data);
  
  unsigned long delta = micros() - start;


  accel = sensor_data.acc;
  gyro = sensor_data.gyro;
  snprintf(output_s, sizeof(output_s) - 1,
           "Accel (g):.............. [%6.4lf, %6.4lf, %6.4lf]\n"
           "Gyro (dps):............. [%9.2lf, %9.2lf, %9.2lf]\n"
           "imu.read_fifo time (us): %lu",
           accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], delta);

  Router::println(output_s);
}

void test_read_time(const char *_){
  static char output_s[100];
  const int count = 1000;
  IMU::sensor_data data;

  unsigned long start = micros();
  for (int i = 0; i < count; ++i){
    if (imu.read_latest(&data)){
      Router::println("Error occurred...");
      return;
    }
  }
  unsigned long delta = micros() - start;

  unsigned long average_delta = (delta + count / 2) / count;

  snprintf(output_s, sizeof(output_s) - 1,
    "Average time (us): %lu",
    average_delta
  );

  Router::println(output_s);
}

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
}

void help(const char *args) {
  // ignore args
  Router::print_all_cmds();
}

void setup() {
  Router::begin();
  Router::println("Controller started.");

  Prop::begin();
  Mag::begin();
  GPS::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({help, "help"});
  Router::add({read_sensor_packet, "read_sensor_packet"});
  Router::add({test_read_time, "test_read_time"});

  SPI.begin();


  delay(3000);

  int error{0};
  error |= imu.begin();
  error |= imu.enable_accel();
  error |= imu.enable_gyro();

  if (error) {
    Router::println("Error while initializing IMU and enabling accel/gyro.");
  } else {
    Router::println("IMU Initialized. Accel and gyro enabled.");
  }

  uint8_t config;

  error = imu.read_accel_config(&config);

  Router::println(std::string("Accel FSR config: ") + std::to_string(config >> 5) + "\n"); // this should output 3 for +-4 g fsr

}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}


void fixedpoint_to_float(int16_t *in, float *out, double sensitivity, const uint8_t dim) {
  double scale = 1.0 / sensitivity;

  for (int i = 0; i < dim; ++i)
    out[i] = scale * in[i];
}

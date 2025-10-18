#include "GPS.h"
#include "Mag.h"
#include "Router.h"
#include "Prop.h"
#include "IMU.h"
#include "Router.h"
#include <Arduino.h>
#include <SPI.h>
#include <cmath>

#define IMU_CS D6

IMU imu(IMU_CS, &SPI);

void calibrate_gyro(const char* _)
{
  imu.calibrate_gyro();

  // output gyro biases

  char output[200];
  snprintf(output, sizeof(output),
           "Gyro Biases (LSBs):  [%6.2d, %6.2d, %6.2d]\n"
           "Gyro Biases (deg/s): [%6.2f, %6.2f, %6.2f]",
           imu.gyro_bias[0], imu.gyro_bias[1], imu.gyro_bias[2], imu.gyro_bias[0] / 65.5, imu.gyro_bias[1] / 65.5, imu.gyro_bias[2] / 65.5);

  Router::println(output);
  return;
}

void calibrate_accel(const char* arg)
{
  int axis = std::atoi(arg);
  imu.calibrate_accel_axis(axis);
}

// simple trapezoidal integrator for the gyro
void test_integrator(const char*_)
{
  double pos[3];
  char outbuf[100];
  IMU::sensor_data new_data;
  IMU::sensor_data last_data;
  memset(pos, 0, sizeof(pos));

  unsigned long last_time = micros();
  unsigned long next_output_time = last_time;
  // js keep doing ts until the user presses enter

  imu.read_latest(&last_data);
  while (!Serial.available())
  {
    // read angular velocity
    imu.read_latest(&new_data);

    unsigned long new_time = micros();
    double delta_time = (new_time - last_time) * (1E-6);

    for (int i = 0; i < 3; ++i)
    {
      pos[i] = std::fmod(pos[i] + 0.5 * (new_data.gyro[i] + last_data.gyro[i]) * delta_time, 360.0);
      if (pos[i] < 0)
      {
        pos[i] += 360.0;
      }
    }

    // output every 50ms
    if (new_time > next_output_time)
    {
      next_output_time += 50000;
      snprintf(outbuf, sizeof(outbuf),
        "[%6.2lf, %6.2lf, %6.2lf]",
        pos[0], pos[1], pos[2]
      );
      Router::println(outbuf);
    }

    last_time = new_time;
    delay(1);
  }

  return;
}

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
           "Read time (us): %lu",
           accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], delta);

  Router::println(output_s);
}



void test_read_time(const char *_){
  static char output_s[100];
  const int count = 1000;
  IMU::sensor_data data;

  unsigned long start = micros();
  for (int i = 0; i < count; ++i){
    imu.read_latest(&data);
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
  Router::add({test_integrator, "test_integrator"});
  Router::add({calibrate_gyro, "calibrate_gyro"});
  Router::add({calibrate_accel, "calibrate_accel"});

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
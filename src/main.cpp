#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include "TrajectoryFollower.h"
#include "TrajectoryLoader.h"
#include "gimbal_servos.h"
#include <Arduino.h>

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
}

void characterize_grid() {
  float deg_min = -7.5;
  float deg_max = 7.5;
  const float interp_time = 2;

  float a = -7.5;

  IMU::Data last_packet;

  Router::print("Angle A (deg), Angle B (deg), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro Y (deg/s), Gyro Z (deg/s)\n");

  for (float b = -7.5; b <= 7.5; b += 0.2) {
    unsigned long start_time = micros();

    while (1) {

      IMU::IMUs[0].read_latest(&last_packet);

      Router::printf("%.2f, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1],
                     last_packet.gyro[2]);

      unsigned long now_time = micros();
      double t = (now_time - start_time) / 1000000.0;
      if (t > interp_time)
        break;

      a = (deg_max - deg_min) * (t / interp_time) + deg_min;

      GimbalServos::setGimbalAngle(a, b);
      delay(1);
    }

    float temp = deg_min;
    deg_min = deg_max;
    deg_max = temp;
  }
}

void characterize_frequency(const char *param) {

  IMU::Data last_packet;

  Router::print("Time (s), Frequency (Hz), Angle A (deg), Angle B (deg), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro Y (deg/s), Gyro Z (deg/s)\n");
  float a = 0;
  float f = 0;

  GimbalServos::setGimbalAngle(a, 0);
  delay(1000);

  unsigned long start_time = micros();

  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time) * 1e-6;
    IMU::IMUs[0].read_latest(&last_packet);

    Router::printf("%.5lf, %.5lf, %.2f, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, f, a, 0, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0],
                   last_packet.gyro[1], last_packet.gyro[2]);

    a = sin(t * t);
    f = t / PI;

    GimbalServos::setGimbalAngle(a, 0);

    delay(1);
  }

  while (Serial.read() != '\n')
    ;
}

void circle_test() {
  IMU::Data last_packet;

  const double amplitude = 7.5;
  const double period = 6; // seconds

  float a = amplitude;
  float b = 0;

  GimbalServos::setGimbalAngle(a, b);
  delay(1000);

  unsigned long start_time = micros();
  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time) * 1e-6;

    IMU::IMUs[0].read_latest(&last_packet);

    Router::printf("%.5lf, %.2f, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1],
                   last_packet.gyro[2]);

    a = amplitude * cos(t * 2 * PI / period);
    b = amplitude * sin(t * 2 * PI / period);
    GimbalServos::setGimbalAngle(a, b);

    delay(1);
  }

  while (Serial.read() != '\n')
    ;
}

void setup() {
  delay(3000);
  Router::begin();
  Router::println("Controller started.");

  SPI.begin();

  // Prop::begin();
  // Mag::begin();
  // GPS::begin();
  IMU::begin();
  GimbalServos::init();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
  Router::add({characterize_grid, "grid"});
  Router::add({characterize_frequency, "frequency"});
  Router::add({circle_test, "circle"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
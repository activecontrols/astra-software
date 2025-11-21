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

/*
Sample IMU Calibration:

-0.914 -0.589 0.625
0.001285802273694 1.963703053573527e-04 0.024535158578756
0.996176127990454 0.999260555553662 0.996176127990454
*/

void roll_test(const char *args) {
  Prop::cmd_set(args);

  IMU::Data last_imu;

  for (int i = 0; i < 3; ++i)
    Serial.print('\n');
  Serial.flush();

  delayMicroseconds(2000);

  Serial.print("<<<  START  >>>\n");

  Serial.print("Time (s),Acceleration X (m/s^2),Acceleration Y (m/s^2),Acceleration Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");

  delayMicroseconds(2000);

  unsigned long start_time = micros();
  unsigned long last_time = start_time;
  // run until the user presses enter
  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time) * 1e-6;
    IMU::IMUs[0].read_latest(&last_imu);

    Serial.print(t, 6);
    for (int i = 0; i < 3; ++i) {
      Serial.print(',');
      Serial.print(last_imu.acc[i], 6);
    }

    for (int i = 0; i < 3; ++i) {
      Serial.print(',');
      Serial.print(last_imu.gyro[i], 10);
    }

    Serial.print('\n');
    
    unsigned long delta_us = now_time - last_time;
    if (delta_us < 1000)
      delayMicroseconds(1000 - delta_us);
    last_time = now_time;
  }
  Serial.readStringUntil('\n');
  Serial.print("<<<  END  >>>\n");

  for (int i = 0; i < 3; ++i)
    Serial.print('\n');
  Serial.flush();

  // stop props
  Prop::cmd_set_both("0 0");
}

void setup() {
  delay(3000);
  Router::begin();
  Router::println("Controller started.");

  Prop::begin();
  // Mag::begin();
  // GPS::begin();
  IMU::begin();
  // GimbalServos::init();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
  Router::add({roll_test, "roll_test"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
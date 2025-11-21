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

-0.995 -0.582 0.655
0.001285802273694 1.963703053573527e-04 0.024535158578756
0.996176127990454 0.999260555553662 0.996176127990454
*/

void roll_test(const char *args) {
  Prop::cmd_set(args);

  IMU::Data last_imu;

  unsigned long start_time = micros();

  Router::print("<<<  START  >>>\n");

  Router::printf("Thrust % (prop 1, 2): (%.2f, %.2f)\n", Prop::get_throttle_1(), Prop::get_throttle_2());

  Router::printf("Time (s), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s)\n");

  // run until the user presses enter
  while (!Serial.available()) {
    double t = (micros() - start_time) * 1e-6;
    IMU::IMUs[0].read_latest(&last_imu);

    Router::printf("%lf, %lf, %lf, %lf, %.10lf, %.10lf, %.10lf\n", t, last_imu.acc[0], last_imu.acc[1], last_imu.acc[2], last_imu.gyro[0], last_imu.gyro[1], last_imu.gyro[2]);

    delay(1);
  }
  Serial.readStringUntil('\n');
  Router::print("<<<  END  >>>\n");

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
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
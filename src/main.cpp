#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include "TrajectoryFollower.h"
#include "TrajectoryLoader.h"
#include "controller_and_estimator.h"
#include "gimbal_servos.h"
#include <Arduino.h>

#include "sample_data.h"

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
}

void setup() {
  delay(3000);
  // SPI.begin(); // spi is a shared interface, so we always begin here
  Router::begin();
  // Router::println("Controller started.");

  // // TODO - configure CS somewhere else!
  // pinMode(PB7, OUTPUT);
  // pinMode(PB3, OUTPUT);
  // pinMode(PB5, OUTPUT);
  // pinMode(PC12, OUTPUT);
  // pinMode(PE7, OUTPUT);
  // pinMode(PE4, OUTPUT);

  // digitalWrite(PB7, HIGH);
  // digitalWrite(PB3, HIGH);
  // digitalWrite(PB5, HIGH);
  // digitalWrite(PC12, HIGH);
  // digitalWrite(PE7, HIGH);
  // digitalWrite(PE4, HIGH);

  // Prop::begin();
  // // Mag::begin();
  // GPS::begin();
  // // IMU::begin();
  // GimbalServos::begin();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  // Router::add({ping, "ping"}); // example registration
  // Router::add({Router::print_all_cmds, "help"});

  Router::println("Starting benchmark...");
  long long start = micros();

  ControllerAndEstimator::init_controller_and_estimator_constants();
  Controller_Output co;

  volatile float co1, co2, co3, co4; // ensure compiler doesn't optimize these out

  for (int i = 0; i < MAX_IDX; i++) {
    co = ControllerAndEstimator::get_controller_output(Vector15(z_arr[i]), Vector3(target_pos_arr[i]), GND_arr[i] == 1.0, true, true, dT);
    co1 = co.gimbal_pitch_deg;
    co2 = co.gimbal_yaw_deg;
    co3 = co.thrust_N;
    co4 = co.roll_rad_sec_squared;
  }

  long long end = micros();
  Router::printf("Finished benchmark in %ld us.\n", end - start);
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
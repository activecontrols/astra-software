#include "Coder.h"
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

void test_coder() {
  if (Coder::test()) {
    Router::println("Coder test passed.");
  } else {
    Router::println("Coder test failed.");
  }
}

void setup() {
  delay(3000);
  SPI.begin(); // spi is a shared interface, so we always begin here
  Router::begin();
  Router::println("Controller started.");

  Prop::begin();
  Mag::begin();
  GPS::begin();
  IMU::begin();
  GimbalServos::begin();
  TrajectoryLoader::begin();
  TrajectoryFollower::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
  Router::add({test_coder, "test_coder"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
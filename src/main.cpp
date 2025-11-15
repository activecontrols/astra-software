#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include "gimbal_servos.h"
#include <Arduino.h>

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
}

void setup() {
  delay(3000);
  Router::begin();
  Router::println("Controller started.");

  // Prop::begin();
  // Mag::begin();
  GPS::begin();
  IMU::begin();
  GimbalServos::init();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
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

void setup() {
  delay(3000);
  SPI.begin(); // spi is a shared interface, so we always begin here
  Router::begin();
  Router::println("Controller started.");

  // TODO - configure CS somewhere else!
  pinMode(PB7, OUTPUT);
  pinMode(PB3, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PC12, OUTPUT);
  pinMode(PE7, OUTPUT);
  pinMode(PE4, OUTPUT);

  digitalWrite(PB7, HIGH);
  digitalWrite(PB3, HIGH);
  digitalWrite(PB5, HIGH);
  digitalWrite(PC12, HIGH);
  digitalWrite(PE7, HIGH);
  digitalWrite(PE4, HIGH);

  // Prop::begin();
  // // Mag::begin();
  // GPS::begin();
  // IMU::begin();
  // GimbalServos::begin();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  Flash::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
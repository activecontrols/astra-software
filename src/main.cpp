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
  digitalWrite(PB7, HIGH);
  digitalWrite(PC12, HIGH);

  digitalWrite(PD7, HIGH); // MAG CS 1
  digitalWrite(PE7, HIGH); // MAG CS 2
  digitalWrite(PE4, HIGH); // MAG CS 3

  digitalWrite(PB3, HIGH); // IMU CS 3
  digitalWrite(PB4, HIGH); // IMU CS 2
  digitalWrite(PB5, HIGH); // IMU CS 1

  // TODO - configure CS somewhere else!
  pinMode(PB7, OUTPUT);
  pinMode(PC12, OUTPUT);
  pinMode(PE7, OUTPUT);
  pinMode(PE4, OUTPUT);

  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PD7, OUTPUT);

  pinMode(PB6, OUTPUT);

  // Prop::begin();
  // // Mag::begin();
  // GPS::begin();
  // IMU::begin();
  // GimbalServos::begin();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  // Flash::begin();

  Logging::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
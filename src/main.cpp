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

#define RADIO_SERIAL Serial1
#define RADIO_RATE 57600

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

  // Prop::begin();
  Mag::begin();
  // GPS::begin();
  // IMU::begin();
  // GimbalServos::init();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});

  // RADIO_SERIAL.begin(RADIO_RATE);
  // Router::println("Radio serial started.");
  // while (true) {
  //   // Router::println("Radio serial started.");
  //   RADIO_SERIAL.println("Hello radio.");
  //   // delay(1000);
  // }
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
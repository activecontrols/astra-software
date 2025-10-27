#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include <Arduino.h>
#include <Mag.h>
#include <SPI.h>
#include <cmath>

#include "SDCard.h"

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

  SPI.begin();

  delay(3000);

  SDCard::begin();
  int error = IMU::begin();


  if (error) {
    Router::println("Error while initializing IMU and enabling accel/gyro.");
  } else {
    Router::println("IMU Initialized. Accel and gyro enabled.");
  }
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
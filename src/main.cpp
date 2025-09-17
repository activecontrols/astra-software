// #include <Arduino.h>
#include <SPI.h>
#include "Router.h"
#include "ICM40609D.h"

ICM40609D icm(D6);

void print_imu_temp(){
  double temp = icm.get_temp_c();

  Router::info(std::to_string(temp));
}

void ping() {
  Router::info("pong");
}

void help() {
  Router::print_all_cmds();
}

void setup() {

  Router::begin();
  Router::info("Controller started.");

  Router::add({ping, "ping"}); // example registration
  Router::add({help, "help"});

  Router::add({print_imu_temp, "print_imu_temp"});

  SPI.begin();
  icm.begin();
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
// #include <Arduino.h>
#include "Router.h"
#include "gimbal_servos.h"

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
  gimbal_servos::centerServos();
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
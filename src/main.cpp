#include "GPS.h"
#include "Mag.h"
#include "Router.h"
#include "Prop.h"
#include "IMU.h"

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
  
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
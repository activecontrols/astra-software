// #include <Arduino.h>
#include "Router.h"

void ping(const char* args) {
  Router::info("pong");
  Router::info_no_newline("args: ");
  Router::info(args == nullptr ? "null" : args);
}

void help(const char* args) {
  // ignore args
  Router::print_all_cmds();
}

void setup() {
  Router::begin();
  Router::info("Controller started.");
  Router::add({ping, "ping"}); // example registration
  Router::add({help, "help"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
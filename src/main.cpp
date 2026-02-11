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

struct my_sample_packet {
  float a;
  float b;
  int c;
  char msg[3];
};

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

  my_sample_packet tx = {}; // ensure any padding bytes get init to 0
  tx.a = 5.3;
  tx.b = 6.3;
  tx.c = 7;
  tx.msg[0] = 'a';
  tx.msg[1] = 'b';
  tx.msg[2] = '\0';
  Router::print_compressed("#a>", tx);
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
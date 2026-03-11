#include "CommandRouter.h"
#include "CommsSerial.h"
#include "FlashLogging.h"
#include "GPS.h"
#include "GimbalServos.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "SPI.h"
#include "TrajectoryFollower.h"
#include "TrajectoryLoader.h"
#include <Arduino.h>

CommsSerial_t<HardwareSerial> HW_CommsSerial(Serial6);
CommsSerial_t<USBSerial> USB_CommsSerial;

void ping(const char *args) {
  CommsSerial.println("pong");
  CommsSerial.print("args: ");
  CommsSerial.println(args == nullptr ? "null" : args);
}

void setup() {
  delay(3000);
  SPI.begin(); // spi is a shared interface, so we always begin here
  CommsSerial.begin(115200);
  CommsSerial.println("Controller started.");

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

  CommandRouter::begin();
  Prop::begin();
  Mag::begin();
  GPS::begin();
  IMU::begin();
  GimbalServos::begin();
  TrajectoryLoader::begin();
  TrajectoryFollower::begin();
  Logging::begin();

  CommandRouter::add(ping, "ping"); // example registration
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}
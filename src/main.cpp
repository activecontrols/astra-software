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
#include "TrajectoryLogger.h"
#include "gimbal_servos.h"

#include <Arduino.h>

CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
CommsSerial_t<USBSerial> USB_CommsSerial;

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
}

void cmd_emi_test() {
  if (!Prop::is_armed()) {
    Prop::arm();
  }
  Prop::set_throttle(50.0, 50.0);

  Mag::beginMeasurement();

  while (!COMMS_SERIAL.available()) {
    while (!Mag::isMeasurementReady) {
      delay(1);
    }

    double x, y, z;

    Mag::read_xyz(x, y, z);
    Mag::beginMeasurement();

    COMMS_SERIAL.print(x, 4);
    COMMS_SERIAL.print(',');
    COMMS_SERIAL.print(y, 4);
    COMMS_SERIAL.print(',');
    COMMS_SERIAL.print(z, 4);
    COMMS_SERIAL.print('\n');
  }
  return;
}

void setup() {
  delay(3000);
  SPI.begin(); // spi is a shared interface, so we always begin here
  CommsSerial.begin(57600);
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
  TrajectoryLogger::begin();

  CommandRouter::add(ping, "ping"); // example registration
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}
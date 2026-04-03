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

#define RADIO_SERIAL Serial6

void cmd_emi_test() {
  if (!Prop::is_armed()) {
    Prop::arm();
  }

  Serial6.begin(57600);
  Prop::set_throttle(0.0, 0.0);

  Mag::beginMeasurement();

  COMMS_SERIAL.print("Time (s),Throttle %,Mag X,Mag Y,Mag Z,Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");


  uint8_t b = 0;
  unsigned long start_t = micros();
  double last_t = start_t * 1e-6;
  while (!COMMS_SERIAL.available()) {
    while (!Mag::isMeasurementReady) {
      delay(1);
    }

    double x, y, z;

    double t = (micros() - start_t) * 1e-6;

    float throttle = 0.0 + 50.0 * (t > 5.0); // throttle on props after 5 seconds

    Prop::set_throttle(throttle, throttle);

    IMU::Data imu_measurement;

    IMU::IMUs[0].read_latest(&imu_measurement);

    Mag::read_xyz(x, y, z);
    Mag::beginMeasurement();

    COMMS_SERIAL.print(t, 4);
    COMMS_SERIAL.print(',');
    COMMS_SERIAL.print(throttle, 2);
    COMMS_SERIAL.print(',');
    COMMS_SERIAL.print(x, 4);
    COMMS_SERIAL.print(',');
    COMMS_SERIAL.print(y, 4);
    COMMS_SERIAL.print(',');
    COMMS_SERIAL.print(z, 4);

    for (int i = 0; i < 3; ++i) {
      COMMS_SERIAL.print(',');
      COMMS_SERIAL.print(imu_measurement.acc[i], 5);
    }

    for (int i = 0; i < 3; ++i) {
      COMMS_SERIAL.print(',');
      COMMS_SERIAL.print(imu_measurement.gyro[i], 5);
    }

    COMMS_SERIAL.print('\n');

    // send incremental data over radio link to also test emi
    for (int i = 0; i < (int)((t - last_t) * 5000); ++i)
    {
      RADIO_SERIAL.write(b++);
    }

    last_t = t;
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
  // GPS::begin();
  IMU::begin();
  // GimbalServos::begin();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  // Logging::begin();

  CommandRouter::add(ping, "ping"); // example registration
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}
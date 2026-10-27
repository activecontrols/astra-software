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
#include "fc_pins.h"
#include <Arduino.h>

CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
CommsSerial_t<USBSerial> USB_CommsSerial;

void ping(const char *args) {
  CommsSerial.println("pong");
  CommsSerial.print("args: ");
  CommsSerial.println(args == nullptr ? "null" : args);
}

struct __attribute__((packed)) imu_entry_t {
  const uint16_t marker = 0xABCD;
  uint32_t time_us;
  int16_t gyro[3];  // x y z
  int16_t accel[3]; // x y z
};

void cmd_vibe_test(void) {
  if (!Logging::is_armed()) {
    CommsSerial.println("Error: flash is not armed");
    return;
  }

  CommsSerial.println("Test running.");
  uint32_t start_us = micros();
  // test runs until we fill up flash or until user enters "X"
  for (uint32_t i = 0; true; ++i) {
    bool do_break = false;

    while (CommsSerial.available()) {
      if (CommsSerial.read() == 'X') {
        do_break = true;
        break;
      }
    }

    if (do_break)
      break;

    uint32_t now_us = micros();
    uint32_t elapsed_us = now_us - start_us;
    IMU::Data imu_data;

    imu_entry_t entry = {.time_us = elapsed_us};

    int16_t gyro_tmp[3];
    int16_t accel_tmp[3];

    IMU::IMUs[0].read_latest_raw(accel_tmp, gyro_tmp);

    memcpy(entry.gyro, gyro_tmp, sizeof(entry.gyro));
    memcpy(entry.accel, accel_tmp, sizeof(entry.accel));

    if (!Logging::write(entry)) {
      break; // break if we reach the end of flash / write fails
    }

    if (i % 2000 == 0) {
      CommsSerial.printf("%u samples collected\n", i);
    }

    uint32_t target_elapsed = i * 250; // target 4KHz loop time
    uint32_t current_us = micros() - start_us;
    if (target_elapsed > current_us) {
      delayMicroseconds(target_elapsed - current_us);
    }
  }

  Logging::complete();

  CommsSerial.println("Test complete.");
}

void setup() {
  delay(3000);
  SPI.begin(); // spi is a shared interface, so we always begin here
  HW_CommsSerial.begin(921600);
  USB_CommsSerial.begin(57600);
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

  pinMode(PIN_TEST_LED, OUTPUT);
  digitalWrite(PIN_TEST_LED, HIGH);

  CommandRouter::begin();
  // Prop::begin();
  // Mag::begin();
  // GPS::begin();
  IMU::begin();
  // GimbalServos::begin();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();
  Logging::begin();
  TrajectoryLogger::begin();

  CommandRouter::add(ping, "ping"); // example registration
  CommandRouter::add(cmd_vibe_test, "vibe_test");
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}
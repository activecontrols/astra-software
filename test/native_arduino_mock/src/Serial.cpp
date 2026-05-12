#include "Serial.h"

HardwareSerial::HardwareSerial(int rx, int tx) {}
HardwareSerial::HardwareSerial() {}

void HardwareSerial::begin(unsigned long baud, uint8_t cfg) {};

int HardwareSerial::available() {
  return 1;
}

int HardwareSerial::read() {
  return 0;
}

size_t HardwareSerial::print(float, int) {
  return 0;
}

size_t HardwareSerial::println() {
  return 0;
}

size_t HardwareSerial::write(const char *buffer, size_t size) {
  return 0;
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
  return 0;
}

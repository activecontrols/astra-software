#pragma once

#include "stdint.h"
#define SERIAL_8N1 0x06

class HardwareSerial {
public:
  HardwareSerial(int rx, int tx);
  HardwareSerial();
  void begin(unsigned long baud, uint8_t cfg = SERIAL_8N1);
  int available();
  int read();
  size_t print(float, int);
  template <typename T> size_t print(T in) {
    return 0;
  };
  size_t println();
  template <typename T> size_t println(T in) {
    return 0;
  };
  template <typename T> size_t write(T in) {
    return 0;
  };
  size_t write(const char *buffer, size_t size);
  size_t write(const uint8_t *buffer, size_t size);
};

// treat USB Serial identically to HardwareSerial
class USBSerial : public HardwareSerial {};
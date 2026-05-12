#pragma once

#include <stdint.h>

class Servo {
public:
  uint8_t attach(int pin);
  void writeMicroseconds(int value);
};
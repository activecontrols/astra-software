#pragma once

#include <stdint.h>

class TwoWire {
public:
  void begin();
  void setClock(uint32_t);

  void beginTransmission(uint8_t);
  uint8_t endTransmission(void);
  template <typename T> size_t write(T in) {
    return 0;
  };
  uint8_t requestFrom(uint8_t, uint8_t);
  int read();
};

extern TwoWire Wire;
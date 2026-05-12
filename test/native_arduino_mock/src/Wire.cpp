#include "Wire.h"

void TwoWire::begin() {}
void TwoWire::setClock(uint32_t) {}
void TwoWire::beginTransmission(uint8_t) {}

uint8_t TwoWire::endTransmission(void) {
  return 0;
}

uint8_t TwoWire::requestFrom(uint8_t, uint8_t) {
  return 0;
}

int TwoWire::read() {
  return 0;
}

TwoWire Wire;
#include "SPI.h"

void SPIClass::begin() {}
void SPIClass::beginTransaction(SPISettings settings) {}
void SPIClass::endTransaction() {}
uint8_t SPIClass::transfer(uint8_t data) {
  return 0;
};

void SPIClass::transfer(void *buf, size_t count) {}

SPIClass SPI;
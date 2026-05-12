#pragma once
#include "stdint.h"

typedef enum {
  SPI_MODE0 = 0,
  SPI_MODE1 = 1,
  SPI_MODE2 = 2,
  SPI_MODE3 = 3,
} SPIMode;

enum BitOrder { LSBFIRST = 0, MSBFIRST = 1 };

#define SPI_SPEED_CLOCK_DEFAULT 4000000

class SPISettings {
public:
  constexpr SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) : clockFreq(clock), bitOrder(bitOrder), dataMode((SPIMode)dataMode) {}
  constexpr SPISettings(uint32_t clock, BitOrder bitOrder, SPIMode dataMode) : clockFreq(clock), bitOrder(bitOrder), dataMode(dataMode) {}
  constexpr SPISettings() : clockFreq(SPI_SPEED_CLOCK_DEFAULT), bitOrder(MSBFIRST), dataMode(SPI_MODE0) {}

  bool operator==(const SPISettings &rhs) const {
    if ((this->clockFreq == rhs.clockFreq) && (this->bitOrder == rhs.bitOrder) && (this->dataMode == rhs.dataMode)) {
      return true;
    }
    return false;
  }

  bool operator!=(const SPISettings &rhs) const {
    return !(*this == rhs);
  }

private:
  uint32_t clockFreq; // specifies the spi bus maximum clock speed
  BitOrder bitOrder;  // bit order (MSBFirst or LSBFirst)
  SPIMode dataMode;   // one of the data mode

  friend class SPIClass;
};

class SPIClass {
public:
  void begin();
  void beginTransaction(SPISettings settings);
  void endTransaction();
  uint8_t transfer(uint8_t data);
  void transfer(void *buf, size_t count);
};

extern SPIClass SPI;
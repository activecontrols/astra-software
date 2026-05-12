#include "stdint.h"

namespace Logging {
void begin() {}
void complete() {}
void write(uint8_t *data, unsigned int len) {};
bool is_armed() {}
void disarm() {}
} // namespace Logging

namespace Flash {
bool read(uint32_t addr, uint32_t len, uint8_t *out) {
  return true;
}
} // namespace Flash

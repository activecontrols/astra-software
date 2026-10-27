#pragma once

#include <cstdint>

#include "controller_and_estimator.h"

#include "IMU.h"
#include "Mag.h"

namespace Logging {

void begin();

void complete();

bool write(uint8_t *data, unsigned int len);

template <typename t> bool write(t x) {
  return write((uint8_t *)&x, sizeof(x));
}

bool is_armed();

void disarm();
}; // namespace Logging
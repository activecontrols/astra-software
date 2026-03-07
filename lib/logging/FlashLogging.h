#pragma once

#include <cstdint>

#include "controller_and_estimator.h"

#include "IMU.h"
#include "Mag.h"

namespace Logging {

void begin();

void complete();

void write(uint8_t *data, unsigned int len);

bool is_armed();
}; // namespace Logging
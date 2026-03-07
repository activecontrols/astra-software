#pragma once

#include <cstdint>

#include "controller_and_estimator.h"

#include "Mag.h"
#include "IMU.h"


namespace Logging
{

void begin();

void complete();

void write(uint8_t *data, unsigned int len);
};
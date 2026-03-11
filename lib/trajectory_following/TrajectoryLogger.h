#pragma once

#include "controller_and_estimator.h"

namespace TrajectoryLogger {

void log_trajectory_flash(float time, int phase, const Controller_Input ci, const Controller_Output co);
void log_calib_flash();

}; // namespace TrajectoryLogger

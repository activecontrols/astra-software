#pragma once

#include "controller_and_estimator.h"

namespace TrajectoryLogger {
// void create_trajectory_log(const char *filename);
// void log_trajectory_csv(float time, int phase, Controller_Input ci, Controller_Output co);
// void close_trajectory_log();

void log_trajectory_flash(float time, int phase, const Controller_Input ci, const Controller_Output co);

void log_calib_flash();

void log_controller_state();
}; // namespace TrajectoryLogger

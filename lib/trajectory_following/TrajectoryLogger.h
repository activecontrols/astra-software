#ifndef TRAJECTORY_LOGGER_H
#define TRAJECTORY_LOGGER_H

namespace TrajectoryLogger {
void create_trajectory_log(const char *filename);
void log_trajectory_csv(float time, int phase, float x, float y, float z);
void close_trajectory_log();

}; // namespace TrajectoryLogger

#endif
#include "TrajectoryFollower.h"

#include "Arduino.h"
#include "Router.h"
#include "SDCard.h"
#include "TrajectoryLoader.h"
#include "TrajectoryLogger.h"
#include "controller.h"
#include "elapsedMillis.h"

#define LOG_INTERVAL_US 5000
#define COMMAND_INTERVAL_US 1000

namespace TrajectoryFollower {

/**
 * Performs linear interpolation between two values.
 * @param a The starting value.
 * @param b The ending value.
 * @param t0 The starting time.
 * @param t1 The ending time.
 * @param t The current time.
 * @return The interpolated value at the current time.
 */
float lerp(float a, float b, float t0, float t1, float t) {
  if (t <= t0)
    return a;
  if (t >= t1)
    return b;
  if (t0 == t1)
    return b; // immediately get to b
  return a + (b - a) * ((t - t0) / (t1 - t0));
}

/**
 * Follows a trajectory by interpolating between position values.
 */
void follow_trajectory() {
  lerp_point_pos *lpt = TrajectoryLoader::lerp_pos_trajectory;
  elapsedMicros timer = elapsedMicros();
  unsigned long lastlog = timer;
  unsigned long lastloop = timer;

  long counter = 0;

  for (int i = 0; i < TrajectoryLoader::header.num_points - 1; i++) {
    while (timer / 1000000.0 < lpt[i + 1].time) {
      float seconds = timer / 1000000.0;
      float x_pos = lerp(lpt[i].x, lpt[i + 1].x, lpt[i].time, lpt[i + 1].time, seconds);
      float y_pos = lerp(lpt[i].y, lpt[i + 1].y, lpt[i].time, lpt[i + 1].time, seconds);
      float z_pos = lerp(lpt[i].z, lpt[i + 1].z, lpt[i].time, lpt[i + 1].time, seconds);

      Controller_Input ci;
      // TODO - fill this with sensor readings
      Controller_Output co = Controller::get_controller_output(ci);
      // TODO - call prop and gimbal servos

      if (timer - lastlog > LOG_INTERVAL_US) {
        lastlog += LOG_INTERVAL_US;
        TrajectoryLogger::log_trajectory_csv(seconds, i, x_pos, y_pos, z_pos);
      }
      counter++;

      unsigned long target_slp = COMMAND_INTERVAL_US - (timer - lastloop);
      delayMicroseconds(target_slp < COMMAND_INTERVAL_US ? target_slp : 0); // don't delay for too long
      lastloop += COMMAND_INTERVAL_US;
    }
  }
  Router::print("Finished ");
  Router::print(counter);
  Router::println(" loop iterations.");
}

// add relevant router cmds
void begin() {
  Router::add({arm, "arm"});
}

// prompt user for log file name, then follow trajectory
void arm(const char *) {
  if (!TrajectoryLoader::loaded_trajectory) {
    Router::println("ARMING FAILURE: no trajectory loaded.");
    return;
  }

  // filenames use DOS 8.3 standard
  Router::print("Enter log filename (1-8 chars + '.' + 3 chars): ");
  String log_file_name = Router::read(50);
  TrajectoryLogger::create_trajectory_log(log_file_name.c_str()); // lower case files have issues on teensy

  Router::print("ARMING COMPLETE. Type `y` and press enter to confirm. ");
  String final_check_str = Router::read(50);
  if (final_check_str != "y") {
    Router::println("ARMING FAILURE: Cancelled by operator.");
    TrajectoryLogger::close_trajectory_log();
    return;
  }

  follow_trajectory();

  Router::println("Finished following trajectory!");
  TrajectoryLogger::close_trajectory_log();
}

} // namespace TrajectoryFollower
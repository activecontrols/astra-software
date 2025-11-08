#include "TrajectoryLogger.h"

#include "CString.h"
#include "Router.h"
#include "SDCard.h"

namespace TrajectoryLogger {

File positionLogFile;
CString<400> telemCSV;

#define LOG_HEADER ("time,phase,north,west,up")

// logs time, phase, and position data in .csv format
int print_counter = 0;
void log_trajectory_csv(float time, int phase, Controller_Input ci, Controller_Output co) {
  telemCSV.clear();
  telemCSV << time << "," << phase << "," << ci.target_pos_north << "," << ci.target_pos_west << "," << ci.target_pos_up;

  positionLogFile.println(telemCSV.str);
  positionLogFile.flush();

  print_counter++;
  if (print_counter % 10 == 0) {
    telemCSV.clear();
    telemCSV << time << "  " << ci.target_pos_north << "  " << ci.target_pos_west << "  " << ci.target_pos_up;
    telemCSV.print();
  }
}

// creates a log file for the current trajectory and prints csv header
void create_trajectory_log(const char *filename) {
  positionLogFile = SDCard::open(filename, FILE_WRITE);
  positionLogFile.println(LOG_HEADER);
}

// close and flush the log file
void close_trajectory_log() {
  positionLogFile.flush();
  positionLogFile.close();
}

} // namespace TrajectoryLogger
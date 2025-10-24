#include "TrajectoryLogger.h"

#include "CString.h"
#include "Router.h"
#include "SDCard.h"

namespace TrajectoryLogger {

File positionLogFile;
CString<400> telemCSV;

#define LOG_HEADER ("time,phase,x,y,z")

// logs time, phase, and position data in .csv format
int print_counter = 0;
void log_trajectory_csv(float time, int phase, float x, float y, float z) {
  telemCSV.clear();
  telemCSV << time << "," << phase << "," << x << "," << y << "," << z;

  positionLogFile.println(telemCSV.str);
  positionLogFile.flush();

  print_counter++;
  if (print_counter % 10 == 0) {
    telemCSV.clear();
    telemCSV << time << "  " << x << "  " << y << "  " << z;
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
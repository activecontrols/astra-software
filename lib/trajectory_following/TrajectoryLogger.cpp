#include "TrajectoryLogger.h"
#include "FlashLogging.h"

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
    Router::print(telemCSV.str);
  }
}

struct type_identifier {
  uint8_t type; // 0 for trajectory log, 1 for gps log
  float time;
  int phase;
};

struct sensorWriting : type_identifier {

  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;

  float mag_x;
  float mag_y;
  float mag_z;
};

struct gpsWriting : type_identifier {

  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;

  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;
};

void log_trajectory_flash(float time, int phase, Controller_Input ci, Controller_Output co) {

  type_identifier idSensor{};
  idSensor.type = 0;
  idSensor.time = time;
  idSensor.phase = phase;

  sensorWriting sensorData{};
  sensorData.type = idSensor.type;
  sensorData.time = idSensor.time;
  sensorData.phase = idSensor.phase;
  sensorData.accel_x = ci.accel_x;
  sensorData.accel_y = ci.accel_y;
  sensorData.accel_z = ci.accel_z;
  sensorData.gyro_yaw = ci.gyro_yaw;
  sensorData.gyro_pitch = ci.gyro_pitch;
  sensorData.gyro_roll = ci.gyro_roll;
  sensorData.mag_x = ci.mag_x;
  sensorData.mag_y = ci.mag_y;
  sensorData.mag_z = ci.mag_z;

  Logging::write((uint8_t *)&sensorData, sizeof(sensorData));

  if (ci.new_gps_packet) {
    type_identifier idGPS{};
    idGPS.type = 1;
    idGPS.time = time;
    idGPS.phase = phase;

    gpsWriting gpsData{};
    gpsData.type = idGPS.type;
    gpsData.time = idGPS.time;
    gpsData.phase = idGPS.phase;

    gpsData.gps_pos_north = ci.gps_pos_north;
    gpsData.gps_pos_west = ci.gps_pos_west;
    gpsData.gps_pos_up = ci.gps_pos_up;

    gpsData.gps_vel_north = ci.gps_vel_north;
    gpsData.gps_vel_west = ci.gps_vel_west;
    gpsData.gps_vel_up = ci.gps_vel_up;

    Logging::write((uint8_t *)&gpsData, sizeof(gpsData));
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
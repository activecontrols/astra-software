#include "TrajectoryLogger.h"
#include "FlashLogging.h"

#include "CString.h"
#include "GPS.h"
#include "Router.h"
#include "SDCard.h"

namespace TrajectoryLogger {

File positionLogFile;
CString<400> telemCSV;

#define LOG_HEADER ("time,phase,north,west,up")

// // logs time, phase, and position data in .csv format
// int print_counter = 0;
// void log_trajectory_csv(float time, int phase, Controller_Input ci, Controller_Output co) {
//   telemCSV.clear();
//   telemCSV << time << "," << phase << "," << ci.target_pos_north << "," << ci.target_pos_west << "," << ci.target_pos_up;

//   positionLogFile.println(telemCSV.str);
//   positionLogFile.flush();

//   print_counter++;
//   if (print_counter % 10 == 0) {
//     telemCSV.clear();
//     telemCSV << time << "  " << ci.target_pos_north << "  " << ci.target_pos_west << "  " << ci.target_pos_up;
//     Router::print(telemCSV.str);
//   }
// }

struct __packed MeasurementFlags {
  bool new_imu_packet : 1;
  bool new_gps_packet : 1;
};

static_assert(sizeof(MeasurementFlags) == 1, "sizeof(MeasurementFlags) error");

struct __packed EntryBase {
  MeasurementFlags flags;
  float time;
  uint8_t phase;
};

static_assert(sizeof(EntryBase) == 6, "sizeof(EntryBase) error");

struct __packed SensorEntry {

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

static_assert(sizeof(SensorEntry) == 36, "sizeof(SensorEntry) error");

struct __packed GpsEntry {

  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;

  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float posCovNN; // m^2
  float posCovNE; // m^2
  float posCovND; // m^2
  float posCovEE; // m^2
  float posCovED; // m^2
  float posCovDD; // m^2
  float velCovNN; // m^2/s^2
  float velCovNE; // m^2/s^2
  float velCovND; // m^2/s^2
  float velCovEE; // m^2/s^2
  float velCovED; // m^2/s^2
  float velCovDD; // m^2/s^2
};

static_assert(sizeof(GpsEntry) == 18 * 4, "sizeof(GpsEntry) error");

void log_trajectory_flash(float time, int phase, Controller_Input ci, Controller_Output co) {

  MeasurementFlags flags{};
  flags.new_gps_packet = ci.new_gps_packet;
  flags.new_imu_packet = ci.new_imu_packet;

  EntryBase entryBase{};
  entryBase.flags = flags;
  entryBase.time = time;
  entryBase.phase = phase;

  Logging::write((uint8_t *)&entryBase, sizeof(entryBase));

  if (ci.new_imu_packet) {
    SensorEntry sensorData{};
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
  }

  if (ci.new_gps_packet) {
    GpsEntry gpsData{};
    gpsData.gps_pos_north = ci.gps_pos_north;
    gpsData.gps_pos_west = ci.gps_pos_west;
    gpsData.gps_pos_up = ci.gps_pos_up;

    gpsData.gps_vel_north = ci.gps_vel_north;
    gpsData.gps_vel_west = ci.gps_vel_west;
    gpsData.gps_vel_up = ci.gps_vel_up;

    gpsData.posCovNN = GPS::ubx.cov.data->posCovNN;
    gpsData.posCovNE = GPS::ubx.cov.data->posCovNE;
    gpsData.posCovND = GPS::ubx.cov.data->posCovND;
    gpsData.posCovEE = GPS::ubx.cov.data->posCovEE;
    gpsData.posCovED = GPS::ubx.cov.data->posCovED;
    gpsData.posCovDD = GPS::ubx.cov.data->posCovDD;

    gpsData.velCovNN = GPS::ubx.cov.data->velCovNN;
    gpsData.velCovNE = GPS::ubx.cov.data->velCovNE;
    gpsData.velCovND = GPS::ubx.cov.data->velCovND;
    gpsData.velCovEE = GPS::ubx.cov.data->velCovEE;
    gpsData.velCovED = GPS::ubx.cov.data->velCovED;
    gpsData.velCovDD = GPS::ubx.cov.data->velCovDD;

    Logging::write((uint8_t *)&gpsData, sizeof(gpsData));
  }

  static_assert(sizeof(co) == 16);
  // log controller output
  Logging::write((uint8_t *)&co, sizeof(co));

  return;
}

// write IMU calib, MAG calib to flash
void log_calib_flash() {
  // some checks to make sure the compiler isn't adding weird padding
  static_assert(sizeof(IMU::Calib) == 8 * 9);
  static_assert(sizeof(Mag::calibration) == 12 * 8);
  static_assert(IMU_COUNT == 1);

  // write imu calibration to flash
  Logging::write((uint8_t *)&IMU::IMUs[0].calib, sizeof(IMU::IMUs[0].calib));

  // write mag calibration to flash
  Logging::write((uint8_t *)&Mag::calib, sizeof(Mag::calib));

  return;
}

// creates a log file for the current trajectory and prints csv header
// void create_trajectory_log(const char *filename) {
//   positionLogFile = SDCard::open(filename, FILE_WRITE);
//   positionLogFile.println(LOG_HEADER);
// }

// // close and flush the log file
// void close_trajectory_log() {
//   positionLogFile.flush();
//   positionLogFile.close();
// }

} // namespace TrajectoryLogger
#include "TrajectoryLogger.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "FlashLogging.h"
#include "GPS.h"
#include "SDCard.h"
#include "TrajectoryLoader.h"

#include "flash.h"

namespace TrajectoryLogger {

void log_x_est() {
  Logging::write(ENTRY_X_EST);

  // log x_est
  Logging::write((uint8_t *)(ControllerAndEstimator::x_est.data()), sizeof(ControllerAndEstimator::x_est(0)) * (ControllerAndEstimator::x_est.size()));

  return;
}

void log_trajectory() {
  Logging::write(ENTRY_TRAJECTORY);

  // next uint32_t specifies length in # of points
  uint32_t len = TrajectoryLoader::header.num_points;

  static_assert(sizeof(traj_point_pos) == 16, "sizeof(traj_point_pos) != 16");

  Logging::write(len);

  Logging::write((uint8_t *)(TrajectoryLoader::trajectory), sizeof(traj_point_pos) * len);
  return;
}

void log_flight_p() {
  Logging::write(ENTRY_FLIGHT_P);

  Logging::write((uint8_t *)(ControllerAndEstimator::Flight_P.data()), sizeof(ControllerAndEstimator::Flight_P(0)) * (ControllerAndEstimator::Flight_P.size()));

  return;
}

void flash_log_sensor(float time, int phase, Controller_Input ci, Controller_Output co) {

  Logging::write(ENTRY_LOOP_STATE);

  LoopState loopState{};
  loopState.time = time;
  loopState.phase = phase;

  Logging::write((uint8_t *)&loopState, sizeof(loopState));

  if (ci.new_imu_packet) {
    Logging::write(ENTRY_SENSOR);

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
    Logging::write(ENTRY_GPS);

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

  Logging::write(ENTRY_CONTROLLER_OUT);
  // log controller output
  Logging::write((uint8_t *)&co, sizeof(co));

  return;
}

// write IMU calib, MAG calib to flash
void log_calib_flash() {
  Logging::write(ENTRY_CALIB);

  // some checks to make sure the compiler isn't adding weird padding
  static_assert(sizeof(IMU::Calib) == 8 * 9);
  static_assert(sizeof(Mag::calibration) == 12 * 4);
  static_assert(IMU_COUNT == 1);

  // write imu calibration to flash (we are only using IMU 0 for now)
  Logging::write((uint8_t *)&IMU::IMUs[0].calib, sizeof(IMU::IMUs[0].calib));

  // write mag calibration to flash
  Logging::write((uint8_t *)&Mag::calib, sizeof(Mag::calib));

  return;
}

void log_complete() {
  Logging::complete();
}

void send_flash_over_serial() {
  uint8_t last_page[PAGE_SIZE];
  uint32_t addr = 0;
  int idx = 0;
  while (1) {

    Flash::read(addr, PAGE_SIZE, last_page);

    USB_CommsSerial.write(last_page, sizeof(last_page));

    bool end = true;

    uint8_t cs_A = 0;
    uint8_t cs_B = 0;
    for (int i = 0; i < sizeof(last_page); ++i) {
      cs_A += last_page[i];
      cs_B += cs_A;
      if (last_page[i] != 0xFF) {
        end = false;
      }
    }

    USB_CommsSerial.write(cs_A);
    USB_CommsSerial.write(cs_B);

    if (end) {
      USB_CommsSerial.write('k');
      break;
    }

    USB_CommsSerial.write('c');

    idx %= PAGE_SIZE;

    addr += PAGE_SIZE;

    // wait for serial to send a c before sending the next page
    while (!USB_CommsSerial.available()) {
    }

    char c;

    do {
      c = USB_CommsSerial.read();
    } while (!(c == 'k' || c == 'c'));

    if (c == 'k') {
      return;
    }
  }

  return;
}

void begin() {
  CommandRouter::add(send_flash_over_serial, "dump_flash", "use this with traj_following/dump.py");
}

} // namespace TrajectoryLogger
#pragma once

#include <Arduino.h>

// TODO - after merging with fc/multisensor branch, remove these defs
#define IMU_COUNT 3
#define MAG_COUNT 3

struct __packed LOG_IMU_CALIB {
  double gyro_bias[3];
  double accel_bias[3];
  double accel_gain[3];
};

struct __packed LOG_MAG_CALIB {
  double hard_x;
  double hard_y;
  double hard_z;
  double soft[3][3];
};

struct __packed Log_Header {
  uint8_t imu_count {IMU_COUNT};
  uint8_t mag_count {MAG_COUNT};
  LOG_MAG_CALIB mag_calib [MAG_COUNT];
  LOG_IMU_CALIB imu_calib [IMU_COUNT];
};
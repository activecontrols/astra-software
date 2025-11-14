#include "gimbal_servos.h"
#include "Router.h"
#include "mbed.h"
#include "portenta_pins.h"

namespace GimbalServos {

// Min and Max pulse for servo (changes depending on the servo)
#define MIN_SERVO_MICROS 800
#define MAX_SERVO_MICROS 2450

mbed::PwmOut yaw_servo(digitalPinToPinName(YAW_SERVO_PIN));
mbed::PwmOut pitch_servo(digitalPinToPinName(PITCH_SERVO_PIN));

#define INTERPOLATION_TABLE_LENGTH 31 // max length of all tables - set to enable passing tables to functions

// maps v from (min_in, max_in) to (min_out, max_out)
float linear_interpolation(float v, float min_in, float max_in, float min_out, float max_out) {
  return (v - min_in) / (max_in - min_in) * (max_out - min_out) + min_out;
}

// interpolates based on given value and table
float clamped_table_interplolation(float v, float table[2][INTERPOLATION_TABLE_LENGTH], int table_length) {
  if (v < table[0][0]) {
    return table[1][0]; // if starting value is below min, return min
  }
  for (int i = 0; i < table_length - 1; i++) {
    if (table[0][i] <= v && v < table[0][i + 1]) {
      return linear_interpolation(v, table[0][i], table[0][i + 1], table[1][i], table[1][i + 1]);
    }
  }
  return table[1][table_length - 1]; // if starting value is above max, return max
}

// Creates the lookup table for gimble-servo reference values for angle phi
// clang-format off
#define yaw_gs_table_len 10
float yaw_gs_table[2][INTERPOLATION_TABLE_LENGTH] = {
  {-44.666, -41.166, -37.83, -34.62, -31.508, -28.472, -25.499, -22.575, -19.689, -16.835, -14.003, -11.189, -8.387, -5.591, -2.797, 0, 2.802, 5.614, 8.439, 11.281, 14.143, 17.029, 19.94, 22.882, 25.855, 28.865, 31.912, 35.001, 38.134, 41.314, 44.542}, 
  {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};

// Creates the lookup table for gimble-servo reference values for angle theta
#define pitch_gs_table_len 8
float pitch_gs_table[2][INTERPOLATION_TABLE_LENGTH] = {
  {-58.465, -53.028, -48.186, -43.725, -39.528, -35.524, -31.663, -27.912, -24.247, -20.65, -17.108, -13.612, -10.156, -6.735, -3.35, 0, 3.311, 6.578, 9.795, 12.953, 16.042, 19.051, 21.967, 24.776, 27.466, 30.023, 32.432, 34.682, 36.762, 38.66, 40.37},
  {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -13, -14, -15}};

// clang-format on

int calc_servo_pulsewidth(float angle) {
  return linear_interpolation(angle, -90, 90, MIN_SERVO_MICROS, MAX_SERVO_MICROS);
}

void setGimbalAngle(float yaw, float pitch) {
  float servo_yaw_angle = clamped_table_interplolation(yaw, yaw_gs_table, yaw_gs_table_len);
  float servo_pitch_angle = clamped_table_interplolation(pitch, pitch_gs_table, pitch_gs_table_len);

  Router::printf("Outputted Yaw Angle: %.2f\n", servo_yaw_angle);
  Router::printf("Outputted Pitch Angle: %.2f\n", servo_pitch_angle);

  yaw_servo.pulsewidth_us(calc_servo_pulsewidth(servo_yaw_angle));
  pitch_servo.pulsewidth_us(calc_servo_pulsewidth(servo_pitch_angle));
}

void centerGimbal() {
  setGimbalAngle(0, 0);
}

void setGimbalAngleCmd(const char *args) {
  float yaw_angle;
  float pitch_angle;
  sscanf(args, "%f %f", &yaw_angle, &pitch_angle);
  setGimbalAngle(yaw_angle, pitch_angle);
}

void init() {
  yaw_servo.period_ms(20);
  pitch_servo.period_ms(20);
  centerGimbal();
  Router::add({setGimbalAngleCmd, "gimbal_set_angle_yp"});
  Router::add({centerGimbal, "gimbal_center"});
}
} // namespace GimbalServos

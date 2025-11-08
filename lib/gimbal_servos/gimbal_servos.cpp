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

#define INTERPOLATION_TABLE_LENGTH 30 // max length of all tables - set to enable passing tables to functions

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
  {-12.34, -9.5, -6.67, -3.86, -1.09, 4.29, 6.88, 9.39, 11.81, 14.13}, 
  {-25, -20, -15, -10, -5, 5, 10, 15, 20, 25}};

// Creates the lookup table for gimble-servo reference values for angle theta
#define pitch_gs_table_len 8
float pitch_gs_table[2][INTERPOLATION_TABLE_LENGTH] = {
  {-13.28, -10.15, -6.99, -0.58, 2.64, 5.86, 9.06, 12.22}, 
  {15, 10, 5, -5, -10, -15, -20, -25}};
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

void centerGimbalCmd(const char *) {
  centerGimbal();
}

void init() {
  yaw_servo.period_ms(20);
  pitch_servo.period_ms(20);
  centerGimbal();
  Router::add({setGimbalAngleCmd, "gimbal_set_angle_yp"});
  Router::add({centerGimbalCmd, "gimbal_center"});
}
} // namespace GimbalServos

#include "gimbal_servos.h"
#include "Router.h"
#include "mbed.h"
#include "portenta_pins.h"

namespace GimbalServos {

// Min and Max pulse for servo (changes depending on the servo)
#define MIN_SERVO_MICROS 800
#define MAX_SERVO_MICROS 2450

float bottom_offset{0};
float top_offset{0};

mbed::PwmOut bottom_servo(digitalPinToPinName(BOTTOM_SERVO_PIN));
mbed::PwmOut top_servo(digitalPinToPinName(TOP_SERVO_PIN));

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

// Creates the lookup table for gimble-servo reference values for angle bottom
// clang-format off
#define bottom_gs_table_len 31
float bottom_gs_table[2][INTERPOLATION_TABLE_LENGTH] = {
  {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
  {-44.666, -41.166, -37.83, -34.62, -31.508, -28.472, -25.499, -22.575, -19.689, -16.835, -14.003, -11.189, -8.387, -5.591, -2.797, 0, 2.802, 5.614, 8.439, 11.281, 14.143, 17.029, 19.94, 22.882, 25.855, 28.865, 31.912, 35.001, 38.134, 41.314, 44.542}};

// Creates the lookup table for gimble-servo reference values for angle top
#define top_gs_table_len 31
float top_gs_table[2][INTERPOLATION_TABLE_LENGTH] = {
  {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
  {40.37, 38.66, 36.762, 34.682, 32.432, 30.023, 27.466, 24.776, 21.967, 19.051, 16.042, 12.953, 9.795, 6.578, 3.311, 0, -3.35, -6.735, -10.156, -13.612, -17.108, -20.65, -24.247, -27.912, -31.663, -35.524, -39.528, -43.725, -48.186, -53.028, -58.465}};

// clang-format on

int calc_servo_pulsewidth(float angle) {
  return linear_interpolation(angle, -90, 90, MIN_SERVO_MICROS, MAX_SERVO_MICROS);
}

float model(const float p[6], float gA, float gB) {
  return p[0] + p[1] * gA + p[2] * gB + p[3] * gA * gA + p[4] * gA * gB + p[5] * gB * gB;
}

void setGimbalAngle(float bottom, float top) {
  float a;
  float b;

  setGimbalAngle(bottom, top, &a, &b);
}

void setGimbalAngle(float bottom, float top, float *out_bottom, float *out_top) {

  // const float top_c[] = {5.107797, 3.680462, 0.080192, 0.047730, 0.000961, -0.000481};
  // const float bottom_c[] = {0.838017, -0.106973, 2.856349, 0.001929, -0.002516, -0.015256};

  const float top_c[] = {9.367508, 3.973589, -0.214943, 0.026955, -0.003850, 0.000172};
  const float bottom_c[] = {1.331478, 0.010510, 3.032240, 0.003421, 0.003297, 0.004364};
  // float servo_bottom_angle = clamped_table_interplolation(bottom, bottom_gs_table, bottom_gs_table_len) + bottom_offset;
  // float servo_top_angle = clamped_table_interplolation(top, top_gs_table, top_gs_table_len) + top_offset;

  // Router::printf("Outputted Bottom Angle: %.2f\n", servo_bottom_angle);
  // Router::printf("Outputted Top Angle: %.2f\n", servo_top_angle);

  float servo_bottom_angle = model(bottom_c, top, bottom) + bottom_offset;
  float servo_top_angle = model(top_c, top, bottom) + top_offset;

  servo_bottom_angle = min(max(servo_bottom_angle, -60), 60);
  servo_top_angle = min(max(servo_top_angle, -60), 60);

  bottom_servo.pulsewidth_us(calc_servo_pulsewidth(servo_bottom_angle));
  top_servo.pulsewidth_us(calc_servo_pulsewidth(servo_top_angle));

  *out_bottom = servo_bottom_angle;
  *out_top = servo_top_angle;
}

void setServoAngle(float servo_bottom_angle, float servo_top_angle) {
  bottom_servo.pulsewidth_us(calc_servo_pulsewidth(servo_bottom_angle));
  top_servo.pulsewidth_us(calc_servo_pulsewidth(servo_top_angle));
}

void centerGimbal() {
  setGimbalAngle(0, 0);
}

void setGimbalAngleCmd(const char *args) {
  double bottom_top_angle[2];
  if (!Router::parse_doubles(args, bottom_top_angle, 2)) {
    Router::printf("Usage: gimbal_set_angle_bt 0.00 0.00\n");
    return;
  }
  setGimbalAngle(bottom_top_angle[0], bottom_top_angle[1]);
}

void set_offset_top(const char *param) {
  float offset = atof(param);

  top_offset = offset;
  centerGimbal();
}

void set_offset_bottom(const char *param) {
  float offset = atof(param);

  bottom_offset = offset;
  centerGimbal();
}

void cmd_set_servo_angle(const char *args) {
  double bottom_top_angle[2];
  if (!Router::parse_doubles(args, bottom_top_angle, 2)) {
    Router::printf("Usage: gimbal_set_angle_bt 0.00 0.00\n");
    return;
  }
  setServoAngle(bottom_top_angle[0], bottom_top_angle[1]);
}

void init() {
  bottom_servo.period_ms(20);
  top_servo.period_ms(20);
  centerGimbal();
  Router::add({setGimbalAngleCmd, "gimbal_set_angle_bt"});
  Router::add({centerGimbal, "gimbal_center"});
  Router::add({set_offset_top, "gimbal_top_offset"});
  Router::add({set_offset_bottom, "gimbal_bottom_offset"});
  Router::add({cmd_set_servo_angle, "set_servo_angle"});
}
} // namespace GimbalServos

#include "gimbal_servos.h"
#include "Router.h"
#include "fc_pins.h"

namespace GimbalServos {

// Min and Max pulse for servo (changes depending on the servo)
#define MIN_SERVO_MICROS 500
#define MAX_SERVO_MICROS 2500

#define INNER_SERVO_CHANNEL 1
#define OUTER_SERVO_CHANNEL 2

HardwareTimer *gimbal_servos;

// maps v from (min_in, max_in) to (min_out, max_out)
float linear_interpolation(float v, float min_in, float max_in, float min_out, float max_out) {
  return (v - min_in) / (max_in - min_in) * (max_out - min_out) + min_out;
}

int calc_servo_pulsewidth(float angle) {
  return linear_interpolation(angle, -135, 135, MIN_SERVO_MICROS, MAX_SERVO_MICROS);
}

void setGimbalAngle(float inner, float outer) {
  float servo_inner_angle = inner + 16;
  float servo_outer_angle = outer + 5;

  // Router::printf("Outputted Inner Angle: %.2f\n", servo_inner_angle);
  // Router::printf("Outputted Outer Angle: %.2f\n", servo_outer_angle);

  // Set duty cycle
  gimbal_servos->setCaptureCompare(INNER_SERVO_CHANNEL, calc_servo_pulsewidth(servo_inner_angle), MICROSEC_COMPARE_FORMAT); // 1500 us pulse
  gimbal_servos->setCaptureCompare(OUTER_SERVO_CHANNEL, calc_servo_pulsewidth(servo_outer_angle), MICROSEC_COMPARE_FORMAT); // 1500 us pulse
}

void centerGimbal() {
  setGimbalAngle(0, 0);
}

void setGimbalAngleCmd(const char *args) {
  double inner_outer_angle[2];
  if (!Router::parse_doubles(args, inner_outer_angle, 2)) {
    Router::printf("Usage: gimbal_set_angle_inout 0.00 0.00\n");
    return;
  }
  setGimbalAngle(inner_outer_angle[0], inner_outer_angle[1]);
}

void begin() {
  pinMode(INNER_SERVO_PIN, OUTPUT);
  pinMode(OUTER_SERVO_CHANNEL, OUTPUT);

  gimbal_servos = new HardwareTimer(TIM3);

  // Set PWM period directly
  gimbal_servos->setOverflow(20000, MICROSEC_FORMAT); // 20,000 us = 20 ms period (50 Hz)

  // Configure PWM channel
  gimbal_servos->setMode(INNER_SERVO_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, INNER_SERVO_PIN);
  gimbal_servos->setMode(OUTER_SERVO_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, OUTER_SERVO_CHANNEL);

  centerGimbal();
  gimbal_servos->resume();

  Router::add({setGimbalAngleCmd, "gimbal_set_angle_inout"});
  Router::add({centerGimbal, "gimbal_center"});
}
} // namespace GimbalServos

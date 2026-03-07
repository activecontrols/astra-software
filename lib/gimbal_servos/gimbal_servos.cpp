#include "gimbal_servos.h"
#include "Router.h"
#include "Servo.h"
#include "fc_pins.h"

namespace GimbalServos {

// Min and Max pulse for servo (changes depending on the servo)
#define MIN_SERVO_MICROS 500
#define MAX_SERVO_MICROS 2500

Servo inner_servo;
Servo outer_servo;

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

  inner_servo.writeMicroseconds(calc_servo_pulsewidth(servo_inner_angle));
  outer_servo.writeMicroseconds(calc_servo_pulsewidth(servo_outer_angle));
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
  inner_servo.attach(INNER_SERVO_PIN);
  outer_servo.attach(OUTER_SERVO_PIN);
  centerGimbal();
  Router::add({setGimbalAngleCmd, "gimbal_set_angle_inout"});
  Router::add({centerGimbal, "gimbal_center"});
}
} // namespace GimbalServos

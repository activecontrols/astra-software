#include <Arduino.h>
#include <gimbal_servos.h>
#include "Portenta_H7_ISR_Servo.h"
#include <Router.h>

// Min and Max pulse for servo (changes depending on the servo)
#define MIN_MICROS 800
#define MAX_MICROS 2450

#define SERVO_PIN_1 D6
#define SERVO_PIN_2 D7

#define INTERPOLATION_TABLE_LENGTH 30 // max length of all tables - set to enable passing tables to functions

namespace {
int servoPitch = -1;
int servoRoll = -1;
} // namespace

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
#define phi_gs_table_len 10
float phi_gs_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {-12.34, -9.5, -6.67, -3.86, -1.09, 4.29, 6.88, 9.39, 11.81, 14.13},
    {-25, -20, -15, -10, -5, 5, 10, 15, 20, 25}};

// Creates the lookup table for gimble-servo reference values for angle theta
#define theta_gs_table_len 8
float theta_gs_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {12.22, 9.06, 5.86, 2.64, -0.58, -6.99, -10.15, -13.28},
    {-25, -20, -15, -10, -5, 5, 10, 15}};

void gimbal_servos::centerServos() {
  if (servoPitch >= 0)
    Portenta_H7_ISR_Servos.setPosition(servoPitch, 90);

  if (servoRoll >= 0)
    Portenta_H7_ISR_Servos.setPosition(servoRoll, 90);
}

void gimbal_servos::setServoAngle(float phi, float theta) {
  int servo_phi = clamped_table_interplolation(phi, phi_gs_table, phi_gs_table_len);
  int servo_theta = clamped_table_interplolation(theta, theta_gs_table, theta_gs_table_len);

  Serial.print("Outputted Phi Angle: ");
  Serial.println(servo_phi);
  Serial.print("Outputted Theta Angle: ");
  Serial.println(servo_theta);

  Portenta_H7_ISR_Servos.setPosition(servoPitch, servo_phi);
  Portenta_H7_ISR_Servos.setPosition(servoRoll, servo_theta);
}

void gimbal_servos::init() {
  Serial.begin(115200);
  Serial.println("Starting feature/gimbal_servos");
  centerServos();

  servoPitch = Portenta_H7_ISR_Servos.setupServo(SERVO_PIN_1, MIN_MICROS, MAX_MICROS);
  servoRoll = Portenta_H7_ISR_Servos.setupServo(SERVO_PIN_2, MIN_MICROS, MAX_MICROS);
}

/*
void loop()
{
    float phi = 0.0;
    float theta = 0.0;
    //setServoAngle(phi, theta);
}
    */
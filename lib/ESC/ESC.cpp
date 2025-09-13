#include "ESC.h"
#include <Arduino.h>
#include <Servo.h>

#define THROTTLE_MIN (0)
#define THROTTLE_MAX (1)

const int pulseMin = 1020;
const int pulseMax = 1980;

ESC::ESC(int pin) {
  this->pin = pin;
  this->armed = false;
  this->throttle = 0;
  this->lastWriteTime = 0;
  pinMode(pin, OUTPUT);
  writeMicroseconds(pulseMin);
}

void ESC::arm() {
  writeMicroseconds(pulseMin);
  delay(2000);
  writeMicroseconds(pulseMax);
  delay(2000);
  writeMicroseconds(pulseMin);
  delay(2000);
  this->armed = true;
}

void ESC::setThrottle(float throttle) {
  if (throttle < THROTTLE_MIN) {
    throttle = THROTTLE_MIN;
  } else if (throttle > THROTTLE_MAX) {
    throttle = THROTTLE_MAX;
  }
  this->throttle = throttle;
}
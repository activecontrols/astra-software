#pragma once

#include "Serial.h"
#include "pins.h" // raw mapping of PA# to ints

// Arduino implicity includes these
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define PIN_SERIAL_RX PG9 // from variant file
#define PIN_SERIAL_TX PG14

#define INPUT 0x0
#define OUTPUT 0x1

#define LOW 0x0
#define HIGH 0x1

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#include <algorithm>
using std::max;
using std::min;
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void digitalWrite(int, int);
void pinMode(int, int);
void delay(int);
void delayMicroseconds(int);
uint32_t millis(void);
uint32_t micros(void);
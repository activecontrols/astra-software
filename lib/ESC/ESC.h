#ifndef ESC_H

#define ESC_H

#include <Arduino.h>
#include <Servo.h>

class ESC {
    public:
        ESC(int pin); 
        void arm(); 
        void setThrottle(float throttle);
}
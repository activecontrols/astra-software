#include <Arduino.h>
#include <gimbal_servos.h>
#include "Portenta_H7_ISR_Servo.h"


//Min and Max pulse for servo (changes depending on the servo)
#define MIN_MICROS      800  
#define MAX_MICROS      2450


#define SERVO_PIN_1 D6
#define SERVO_PIN_2 D7

namespace {
    int servoPitch = -1;
    int servoRoll = -1;
}


void gimbal_servos::centerServos()
{
    if (servoPitch >= 0) 
        Portenta_H7_ISR_Servos.setPosition(servoPitch, 90);

    if (servoRoll >= 0)    
        Portenta_H7_ISR_Servos.setPosition(servoRoll, 90);
}

void gimbal_servos::setServoAngle(float phi, float theta){
    Portenta_H7_ISR_Servos.setPosition(servoPitch, (int) phi);
    Portenta_H7_ISR_Servos.setPosition(servoRoll, (int) theta);
}


void gimbal_servos::init()
{
    Serial.begin(115200);
    Serial.println("Starting feature/gimbal_servos");

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
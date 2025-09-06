#include <Arduino.h>
#include "STM32_ISR_Servo.h"

#define STM_TIMER TIMER_SERVO


//Min and Max pulse for servo (changes depending on the servo)
#define MIN_MICROS      800  
#define MAX_MICROS      2450


#define SERVO_PIN_1 D6
#define SERVO_PIN_2 D7


int servoPitch = -1;
int servoRoll = -1;



void centerServos()
{
    if (servoPitch >= 0) 
        STM_ISR_Servos.setposition(servoPitch, 90);

    if (servoRoll >= 0)    
        STM_ISR_Servos.setposition(servoRoll, 90);
}

void setAngle(float phi, float theta){
    STM_ISR_Servos.setposition(servoPitch, (int) phi);
    STM_ISR_Servos.setposition(servoRoll, (int) roll);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting feature/gimbal_servos");

    STM32_ISR_Servos.useTimer(STM_TIMER);

    servoPitch = STM32_ISR_Servos.setupServo(SERVO_PIN_1, MIN_MICROS, MAX_MICROS);
    servoRoll = STM32_ISR_Servos.setupServo(SERVO_PIN_2, MIN_MICROS, MAX_MICROS);

    centerServos()
}

void loop()
{
    float phi = 0.0, theta = 0.0;
    setAngle(phi, theta);
}
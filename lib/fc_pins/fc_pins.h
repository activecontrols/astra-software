#pragma once
#include <Arduino.h>

// TODO - all of these need to be updated
#define BOTTOM_SERVO_PIN PH15
#define TOP_SERVO_PIN PH1
#define PROP2_PIN PH11
#define SD_CARD_CS PG7
#define RADIO_TX PC6
#define RADIO_RX PC7
#define PROP1_PIN PA8

const uint32_t IMU_CS[] = {PB5, PB4, PB3};
const uint32_t MAG_CS[] = {PD7, PE7, PE4};
#pragma once
#include "SPI.h"
#include <Arduino.h>

// PWM PINS
#define PROP1_PIN PD14
#define PROP2_PIN PD15
#define BOTTOM_SERVO_PIN PC6 // servo 1
#define TOP_SERVO_PIN PC7    // servo 2

// SPI PINS
#define SD_CARD_CS PC12
#define IMU_CS PB4 // TODO - break out into 3 IMUs
// TODO - add mags

#define FC_SCK PB10
#define FC_MOSI PC3_C
#define FC_MISO PC2_C
extern SPIClass fc_spi;

#define EXTERNAL_UART_RX PG9
#define EXTERNAL_UART_TX PG14
extern HardwareSerial external_uart;

#define GPS_UART_RX PA10
#define GPS_UART_TX PA9
extern HardwareSerial gps_uart;

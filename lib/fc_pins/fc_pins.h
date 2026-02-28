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

#define GPS_UART_RX PA10
#define GPS_UART_TX PA9

#define PIN_QSPI_FLASH_MISO PD12
#define PIN_QSPI_FLASH_MOSI PD11
#define PIN_QSPI_FLASH_CLK PF10
#define PIN_QSPI_FLASH_CS PG6
#define PIN_QSPI_FLASH_RST_IO3 PD13
#define PIN_QSPI_FLASH_WP_IO2 PF7
#include "fc_pins.h"
#include "SPI.h"
#include <Arduino.h>

HardwareSerial external_uart(EXTERNAL_UART_RX, EXTERNAL_UART_TX);
HardwareSerial gps_uart(GPS_UART_RX, GPS_UART_TX);
SPIClass fc_spi(FC_MOSI, FC_MISO, FC_SCK);
#include "GPS.h"
#include "Router.h"

namespace GPS {
GPS_Coord origin;

void begin() {
  GPS_UART.begin(38400, SERIAL_8N1); // https://content.u-blox.com/sites/default/files/documents/NEO-F9P-15B_DataSheet_UBX-22021920.pdf
}

// TODO GPS - this code was just for testing - please update to pass into tinygps++
void pump_events() {
  char msg[2];

  while (GPS_UART.available()) { // https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    msg[0] = (char)GPS_UART.read();
    msg[1] = '\0';
    Router::print(msg);
  }
}
} // namespace GPS
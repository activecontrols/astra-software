#include "GPS.h"

namespace FlightCommands {
bool escaped;
#define MAX_FLIGHT_CMD_LEN 256
uint8_t flight_command_buffer[MAX_FLIGHT_CMD_LEN];
int flight_command_buffer_pos;
bool kill_flag;
bool arm_flag;

#define ESCAPE_CHAR '\\'
#define END_CHAR '\n'

void reset() {
  escaped = false;
  flight_command_buffer_pos = 0;
  kill_flag = false;
}

void process_cmd(uint8_t *cmd_buf, int cmd_len) {
  if (cmd_len == 1 && cmd_buf[0] == 'k') { // "k\n"
    kill_flag = true;
  }
  if (cmd_len == 1 && cmd_buf[0] == 'y') { // "y\n"
    arm_flag = true;
  }
  if (cmd_len > 4 && cmd_buf[0] == 'r' && cmd_buf[1] == 't' && cmd_buf[2] == 'k' && cmd_buf[3] == ':') { // "rtk:***"
    for (int i = 4; i < cmd_len; i++) {
      gps_uart.write(flight_command_buffer[i]);
    }
  }
}

void encode(uint8_t c) {
  if (c == END_CHAR && !escaped) { // true end
    process_cmd(flight_command_buffer, flight_command_buffer_pos);
  } else if (c == ESCAPE_CHAR && !escaped) { // true escape start
    escaped = true;
  } else { // we might still be escaped here but if it isn't followed by a special character we will just ignore it...
    flight_command_buffer[flight_command_buffer_pos] = c;
    flight_command_buffer_pos++;
    escaped = false;
    if (flight_command_buffer_pos == MAX_FLIGHT_CMD_LEN) {
      flight_command_buffer_pos = 0; // something went wrong and we never saw a non-escaped END_CHAR so just go back to 0 and try again
    }
  }
}

} // namespace FlightCommands

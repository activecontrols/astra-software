#include "flight_commands.h"
#include "flight_data.h"
#include <stdint.h>
#include <stdio.h> // use the built in serial monitor
#include <string.h>

bool escaped;
#define MAX_FLIGHT_CMD_LEN 1024
uint8_t flight_command_buffer[MAX_FLIGHT_CMD_LEN];
int flight_command_buffer_pos;

int current_buf_len = 0;
char concat_msg_buf[OUT_BUF_SIZE];

void reset() {
  escaped = false;
  flight_command_buffer_pos = 0;
}

void process_cmd(uint8_t *cmd_buf, int cmd_len) {
  if (cmd_len == sizeof(active_packet) + 3 && cmd_buf[0] == 't' && cmd_buf[1] == 'r' && cmd_buf[2] == ' ') { // "tr:***"
    memcpy(&active_packet, &cmd_buf[3], sizeof(active_packet));
    commit_packet();
    reply_with_heartbeat();
  } else {
    cmd_buf[cmd_len] = '\0';
    printf("%s\n", cmd_buf);
    int msg_len = cmd_len + 1; // +1 for \n

    if (current_buf_len > 0) {
      concat_msg_buf[current_buf_len - 1] = '\n';
    }
    if (current_buf_len + msg_len >= OUT_BUF_SIZE) {
      int half = current_buf_len / 2;
      // Shift second half to the front
      memmove(concat_msg_buf, concat_msg_buf + half, current_buf_len - half + 1); // +1 for the '\0'
      current_buf_len = half;
    }
    strncat(concat_msg_buf, (char *)cmd_buf, OUT_BUF_SIZE - current_buf_len - 1);
    strncat(concat_msg_buf, "\n", OUT_BUF_SIZE - current_buf_len - 1);
    current_buf_len += msg_len;
    concat_msg_buf[current_buf_len - 1] = '\0';
  }
}

// TODO - merge these with the functions in CommandRouter if possible
void flight_command_encode(uint8_t c) {
  if (c == END_CHAR && !escaped) { // true end
    process_cmd(flight_command_buffer, flight_command_buffer_pos);
    flight_command_buffer_pos = 0;
  } else if (c == CR_CHAR && !escaped) { // true carriage return
    // do nothing - we aren't a typewriter, no need to carriage return
  } else if (c == BACKSPACE_CHAR && !escaped) { // true backspace
    if (flight_command_buffer_pos > 0) {
      flight_command_buffer_pos -= 1;
    }
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

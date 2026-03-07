#pragma once
#include "stdint.h"

namespace FlightCommands {
extern bool kill_flag;
extern bool arm_flag;

void reset();
void process_cmd(uint8_t *cmd_buf, int cmd_len);
void encode(uint8_t c);
} // namespace FlightCommands

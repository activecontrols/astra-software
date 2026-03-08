#ifndef ASTRA_GS_FLIGHT_COMMANDS_H
#define ASTRA_GS_FLIGHT_COMMANDS_H

#include <stdint.h>

#define ESCAPE_CHAR '\\'
#define END_CHAR '\n'

void flight_command_encode(uint8_t c);
#endif
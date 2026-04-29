#pragma once
#include <stdio.h>
#include "matlab_funcs.h"
#include "TrajectoryLogger.h"
#include "astra_structs.h"

void csr_to_csv(FILE* f_csv_out, telemetry_packet_t flight_packet, Controller_Internals internals);

void csv_header_log(FILE *out);

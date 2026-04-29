#pragma once
#include <stdio.h>
#include "flight_data.h"
#include "matlab_funcs.h"
#include "TrajectoryLogger.h"


void csr_to_csv(FILE* f_csv_out, flight_packet_t flight_packet, Controller_Intermediates intermediates, GpsEntry gps);

void csv_header_log(FILE *out);

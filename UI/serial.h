#ifndef ASTRA_GS_SERIAL_H
#define ASTRA_GS_SERIAL_H

typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;
} state_packet_t;

extern state_packet_t state_packet;
void poll_serial();
void init_serial();
void deinit_serial();

#endif

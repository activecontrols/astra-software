#include <windows.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"

HANDLE hSerial;
state_packet_t state_packet;
#define BUF_SIZE 4096
#define T_BUF_SIZE 1024

// your message callback
void handle_message(const char *msg) {
  if (msg[0] == '>') {
    sscanf(msg, ">%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
           &state_packet.accel_x, &state_packet.accel_y, &state_packet.accel_z, &state_packet.gyro_yaw, &state_packet.gyro_pitch, &state_packet.gyro_roll, &state_packet.mag_x, &state_packet.mag_y, &state_packet.mag_z, &state_packet.gps_pos_north, &state_packet.gps_pos_west, &state_packet.gps_pos_up, &state_packet.gps_vel_north, &state_packet.gps_vel_west, &state_packet.gps_vel_up,
           &state_packet.state_q_vec_0, &state_packet.state_q_vec_1, &state_packet.state_q_vec_2, &state_packet.state_pos_north, &state_packet.state_pos_west, &state_packet.state_pos_up, &state_packet.state_vel_north, &state_packet.state_vel_west, &state_packet.state_vel_up, &state_packet.state_0, &state_packet.state_1, &state_packet.state_2, &state_packet.state_3, &state_packet.state_4, &state_packet.state_5,
           &state_packet.gimbal_yaw_deg, &state_packet.gimbal_pitch_deg, &state_packet.thrust_N, &state_packet.roll_N,
           &state_packet.target_pos_north, &state_packet.target_pos_west, &state_packet.target_pos_up);
  }
}

// reads from serial, splits by '\n', calls your callback
void poll_serial() {
  static char buffer[BUF_SIZE];
  static int buf_len = 0;

  char temp[T_BUF_SIZE];
  DWORD bytes_read = 0;

  if (!ReadFile(hSerial, temp, sizeof(temp), &bytes_read, NULL)) {
    printf("ReadFile error %lu\n", GetLastError());
    return;
  }

  if (bytes_read == 0)
    return;

  // append new bytes to buffer
  for (DWORD i = 0; i < bytes_read; i++) {
    char c = temp[i];

    if (c == '\n') {
      buffer[buf_len] = '\0'; // terminate message
      handle_message(buffer); // <-- YOUR CALLBACK
      buf_len = 0;            // reset buffer
    } else if (buf_len < BUF_SIZE - 1) {
      buffer[buf_len++] = c;
    }
  }
}

void init_serial() {
  hSerial = CreateFileA(
      "\\\\.\\COM15",
      GENERIC_READ | GENERIC_WRITE,
      0, NULL, OPEN_EXISTING,
      0, NULL);

  if (hSerial == INVALID_HANDLE_VALUE) {
    printf("Error opening serial port\n");
    return;
  }

  // --- configure port ---
  DCB dcb = {0};
  dcb.DCBlength = sizeof(dcb);

  GetCommState(hSerial, &dcb);
  dcb.BaudRate = CBR_115200;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;

  if (!SetCommState(hSerial, &dcb)) {
    printf("SetCommState error\n");
    return;
  }

  // --- set timeouts (non-blocking with small wait) ---
  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 10;
  timeouts.ReadTotalTimeoutConstant = 10;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  SetCommTimeouts(hSerial, &timeouts);

  printf("Listening...\n");
}

void deinit_serial() {
  CloseHandle(hSerial);
}
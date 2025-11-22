#include "serial.h"
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <windows.h>

HANDLE hSerial;
state_packet_t state_packet;
#define BUF_SIZE 4096
#define T_BUF_SIZE 1024

#define MAX_MSGS 32
#define MAX_MSG_LEN 1024

char msg_buffer[MAX_MSGS][MAX_MSG_LEN];
int msg_count = 0;
char concat_msg_buf[OUT_BUF_SIZE];

FILE *log_file;

void concat_messages(char *out) {}

// your message callback
void handle_message(const char *msg) {
  fprintf(log_file, "%s", msg);
  if (msg[0] == '>' && msg[1] == 'a') {
    sscanf(msg, ">a %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &state_packet.accel_x, &state_packet.accel_y, &state_packet.accel_z, &state_packet.gyro_yaw, &state_packet.gyro_pitch,
           &state_packet.gyro_roll, &state_packet.mag_x, &state_packet.mag_y, &state_packet.mag_z, &state_packet.gps_pos_north, &state_packet.gps_pos_west, &state_packet.gps_pos_up,
           &state_packet.gps_vel_north, &state_packet.gps_vel_west, &state_packet.gps_vel_up);
  } else if (msg[0] == '>' && msg[1] == 'b') {
    sscanf(msg, ">b %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &state_packet.state_q_vec_new, &state_packet.state_q_vec_0, &state_packet.state_q_vec_1, &state_packet.state_q_vec_2,
           &state_packet.state_pos_north, &state_packet.state_pos_west, &state_packet.state_pos_up, &state_packet.state_vel_north, &state_packet.state_vel_west, &state_packet.state_vel_up,
           &state_packet.state_0, &state_packet.state_1, &state_packet.state_2, &state_packet.state_3, &state_packet.state_4, &state_packet.state_5);
  } else if (msg[0] == '>' && msg[1] == 'c') {
    sscanf(msg, ">c %f %f %f %f %f %f %f", &state_packet.gimbal_yaw_deg, &state_packet.gimbal_pitch_deg, &state_packet.thrust_N, &state_packet.roll_N, &state_packet.target_pos_north,
           &state_packet.target_pos_west, &state_packet.target_pos_up);
  } else if (msg[0] == '>' && msg[1] == 'd') {
    sscanf(msg, ">d %f %f %f %f %f", &state_packet.elapsed_time, &state_packet.GND_flag, &state_packet.flight_armed, &state_packet.thrust_perc, &state_packet.diffy_perc);
  } else {
    for (int i = MAX_MSGS - 1; i > 0; i--) {
      strcpy(msg_buffer[i], msg_buffer[i - 1]);
    }

    // insert new message at front
    strncpy(msg_buffer[0], msg, MAX_MSG_LEN - 1);
    msg_buffer[0][MAX_MSG_LEN - 1] = '\0';

    if (msg_count < MAX_MSGS)
      msg_count++;

    concat_msg_buf[0] = '\0'; // start empty
    for (int i = 0; i < msg_count; i++) {
      strncat(concat_msg_buf, msg_buffer[i], OUT_BUF_SIZE - strlen(concat_msg_buf) - 1);
      strncat(concat_msg_buf, "\n", OUT_BUF_SIZE - strlen(concat_msg_buf) - 1);
    }
  }
}

void write_serial(const char *msg) {
  fprintf(log_file, "%s\n", msg);
  DWORD bytes_written = 0;

  int len = (int)strlen(msg);
  if (len == 0)
    return;

  if (!WriteFile(hSerial, msg, len, &bytes_written, NULL)) {
    printf("WriteFile error %lu\n", GetLastError());
    return;
  }

  // Optionally: ensure newline at the end
  WriteFile(hSerial, "\n", 1, &bytes_written, NULL);
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
  std::time_t now = std::time(nullptr);
  std::tm tm_buf;
  localtime_s(&tm_buf, &now);
  char timestamp[64];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", &tm_buf);
  char filename[128];
  snprintf(filename, sizeof(filename), "logs/log_%s.txt", timestamp);
  log_file = fopen(filename, "w");

  hSerial = CreateFileA("\\\\.\\COM9", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

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
  fclose(log_file);
  CloseHandle(hSerial);
}
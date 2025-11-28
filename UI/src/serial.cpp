#include "serial.h"
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <windows.h>

// the core logic in the file is for continually updating this vector as messages come in
state_packet_t state_packet;

#define MODE_SERIAL_INPUT 1
#define MODE_FILE_INPUT 2
HANDLE hSerial;
FILE *input_file;
FILE *log_file; // should only be used if serial is active
int input_mode; // SERIAL or FILE

// this manages ingoing and outgoing serial buffers
#define BUF_SIZE 4096
#define T_BUF_SIZE 1024

int current_buf_len = 0;
char concat_msg_buf[OUT_BUF_SIZE];

// your message callback
void handle_message(char *msg) {
  if (input_mode == MODE_SERIAL_INPUT) {
    fprintf(log_file, "%s", msg); // TODO - this is not printing the newline correctly
  }

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
    // TODO - need to check this over to make sure no memory issues
    int msg_len = strlen(msg) + 1; // +1 for \n
    if (current_buf_len > 0) {
      concat_msg_buf[current_buf_len - 1] = '\n';
    }

    if (current_buf_len + msg_len >= OUT_BUF_SIZE) {
      int half = current_buf_len / 2;
      // Shift second half to the front
      memmove(concat_msg_buf, concat_msg_buf + half, current_buf_len - half + 1); // +1 for the '\0'
      current_buf_len = half;
    }

    strncat(concat_msg_buf, msg, OUT_BUF_SIZE - current_buf_len - 1);
    strncat(concat_msg_buf, "\n", OUT_BUF_SIZE - current_buf_len - 1);
    current_buf_len += msg_len;
    concat_msg_buf[current_buf_len - 1] = '\0';
  }
}

void write_serial(const char *msg) {
  if (input_mode == MODE_SERIAL_INPUT) {
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
  } else {
    // ignore this - can't write messages if not in serial input mode
  }
}

// reads from serial, splits by '\n', calls your callback
void poll_serial() {
  static char buffer[BUF_SIZE];
  static int buf_len = 0;

  if (input_mode == MODE_SERIAL_INPUT) {
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
  } else { // TODO - make this less cursed
    int write_idx = 0;
    while (true) {
      int c = fgetc(input_file);
      if (c == EOF) {
        return;
      }
      buffer[write_idx] = c;
      write_idx++;
      if (write_idx >= BUF_SIZE) {
        return;
      }
      if (c == '\r' || c == '\n') {
        buffer[write_idx - 1] = '\0';
        handle_message(buffer);
        return;
      }
    }
  }
}

bool init_serial(char *com_port) {
  if (strncmp(com_port, "COM", 3) == 0) {
    input_mode = MODE_SERIAL_INPUT;
  } else {
    input_mode = MODE_FILE_INPUT;
  }

  if (input_mode == MODE_SERIAL_INPUT) {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf;
    localtime_s(&tm_buf, &now);
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", &tm_buf);
    char filename[128];
    snprintf(filename, sizeof(filename), "logs/log_%s.txt", timestamp);
    log_file = fopen(filename, "w");

    char portName[20];
    snprintf(portName, sizeof(portName), "\\\\.\\%s", com_port);
    hSerial = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
      printf("Error opening serial port - make sure port name is correct and all other serial monitors are closed.\n");
      return false;
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
      return false;
    }

    // --- set timeouts (non-blocking with small wait) ---
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutConstant = 10;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    SetCommTimeouts(hSerial, &timeouts);

    printf("Listening...\n");
    return true;
  } else { // file input
    input_file = fopen(com_port, "r");
    if (input_file == NULL) {
      printf("Couldn't open input file, make sure path is correct.\n");
      return EXIT_FAILURE;
    }
    return true;
  }
}

void deinit_serial() {
  if (input_mode == MODE_SERIAL_INPUT) {
    fclose(log_file);
    CloseHandle(hSerial);
  } else {
    fclose(input_file);
  }
}
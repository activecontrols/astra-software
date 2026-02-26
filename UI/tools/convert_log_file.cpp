// converts old ASTRA logs to the new binary format
// in general, a starting point for porting log files

// gcc .\tools\convert_log_file.cpp -Iinclude -o .\tools\convert_log_file.exe
// .\tools\convert_log_file.exe sample_flight.txt sample_flight.bin

#include <flight_data.h>
#include <stdio.h>
#include <stdlib.h>

#define BUF_SIZE 4096

flight_packet_t active_packet;

void parse_packet(char *msg, FILE *output_file) {
  if (msg[0] == '>' && msg[1] == 'a') {
    sscanf(msg, ">a %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &active_packet.accel_x, &active_packet.accel_y, &active_packet.accel_z, &active_packet.gyro_yaw, &active_packet.gyro_pitch,
           &active_packet.gyro_roll, &active_packet.mag_x, &active_packet.mag_y, &active_packet.mag_z, &active_packet.gps_pos_north, &active_packet.gps_pos_west, &active_packet.gps_pos_up,
           &active_packet.gps_vel_north, &active_packet.gps_vel_west, &active_packet.gps_vel_up);
  } else if (msg[0] == '>' && msg[1] == 'b') {
    sscanf(msg, ">b %f %f %f %f %f %f %f %f %f %f", &active_packet.state_q_vec_new, &active_packet.state_q_vec_0, &active_packet.state_q_vec_1, &active_packet.state_q_vec_2,
           &active_packet.state_pos_north, &active_packet.state_pos_west, &active_packet.state_pos_up, &active_packet.state_vel_north, &active_packet.state_vel_west, &active_packet.state_vel_up);
  } else if (msg[0] == '>' && msg[1] == 'c') {
    sscanf(msg, ">c %f %f %f %f %f %f %f", &active_packet.gimbal_yaw_raw, &active_packet.gimbal_pitch_raw, &active_packet.thrust_N, &active_packet.roll_N, &active_packet.target_pos_north,
           &active_packet.target_pos_west, &active_packet.target_pos_up);

    // commit packet
    fwrite(&active_packet, sizeof(active_packet), 1, output_file);
  }
}

int main(int argc, char **argv) {
  if (argc != 3) {
    printf("Usage: ./convert_log_file.exe old_in.txt new_out.bin");
    return EXIT_FAILURE;
  }

  FILE *input_file = fopen(argv[1], "r");
  if (input_file == NULL) {
    printf("Failed to open input file.");
    return EXIT_FAILURE;
  }

  FILE *output_file = fopen(argv[2], "wb");
  if (output_file == NULL) {
    printf("Failed to open output file.");
    fclose(input_file);
    return EXIT_FAILURE;
  }

  // scan the old file
  char buffer[BUF_SIZE];
  int write_idx = 0;
  while (true) {
    int c = fgetc(input_file);
    if (c == EOF) {
      break;
    }
    buffer[write_idx] = c;
    write_idx++;
    if (write_idx >= BUF_SIZE) {
      printf("Internal buffer overflow while reading.");
      return EXIT_FAILURE;
    }
    if (c == '\r' || c == '\n') {
      buffer[write_idx - 1] = '\0';
      parse_packet(buffer, output_file);
      write_idx = 0;
    }
  }

  fclose(input_file);
  fclose(output_file);
}
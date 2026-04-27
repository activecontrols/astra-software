#include "flight_data.h"
#include "flight_commands.h"
#include "flight_data_state.h"
#include "platform_win.h"

flight_history_t FlightHistory;
telemetry_packet_t active_packet; // may be partially filled
flight_data_state_t FlightDataState;

void init_flight_data() {
  FlightHistory = {};

  FlightHistory.write_pos = 0;
  FlightHistory.read_start_pos = FlightHistory.write_pos;
  FlightHistory.read_end_pos = FlightHistory.read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}

void deinit_flight_data() {
  if (FlightDataState.input_file != NULL) {
    fclose(FlightDataState.input_file);
    FlightDataState.input_file = NULL;
  }

  FlightDataState.fv_serial.close();
  FlightDataState.rtk_serial.close();
}

void commit_packet() {
  FlightHistory.accel_x[FlightHistory.write_pos] = active_packet.ci.imu.accel_x;
  FlightHistory.accel_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.imu.accel_x;
  FlightHistory.accel_y[FlightHistory.write_pos] = active_packet.ci.imu.accel_y;
  FlightHistory.accel_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.imu.accel_y;
  FlightHistory.accel_z[FlightHistory.write_pos] = active_packet.ci.imu.accel_z;
  FlightHistory.accel_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.imu.accel_z;
  FlightHistory.gyro_yaw[FlightHistory.write_pos] = active_packet.ci.imu.gyro_yaw;
  FlightHistory.gyro_yaw[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.imu.gyro_yaw;
  FlightHistory.gyro_pitch[FlightHistory.write_pos] = active_packet.ci.imu.gyro_pitch;
  FlightHistory.gyro_pitch[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.imu.gyro_pitch;
  FlightHistory.gyro_roll[FlightHistory.write_pos] = active_packet.ci.imu.gyro_roll;
  FlightHistory.gyro_roll[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.imu.gyro_roll;
  FlightHistory.mag_x[FlightHistory.write_pos] = active_packet.ci.mag.mag_x;
  FlightHistory.mag_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.mag.mag_x;
  FlightHistory.mag_y[FlightHistory.write_pos] = active_packet.ci.mag.mag_y;
  FlightHistory.mag_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.mag.mag_y;
  FlightHistory.mag_z[FlightHistory.write_pos] = active_packet.ci.mag.mag_z;
  FlightHistory.mag_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.ci.mag.mag_z;
  FlightHistory.gps_pos_north = active_packet.ci.gps.pos.north;
  FlightHistory.gps_pos_west = active_packet.ci.gps.pos.west;
  FlightHistory.gps_pos_up = active_packet.ci.gps.pos.up;
  FlightHistory.gps_vel_north = active_packet.ci.gps.vel.north;
  FlightHistory.gps_vel_west = active_packet.ci.gps.vel.west;
  FlightHistory.gps_vel_up = active_packet.ci.gps.vel.up;

  FlightHistory.state_q_vec_new = active_packet.x_est.q_vec_w;
  FlightHistory.state_q_vec_0 = active_packet.x_est.q_vec_x;
  FlightHistory.state_q_vec_1 = active_packet.x_est.q_vec_y;
  FlightHistory.state_q_vec_2 = active_packet.x_est.q_vec_z;
  FlightHistory.state_pos_north = active_packet.x_est.est_pos_north;
  FlightHistory.state_pos_west = active_packet.x_est.est_pos_west;
  FlightHistory.state_pos_up = active_packet.x_est.est_pos_up;
  FlightHistory.state_vel_north = active_packet.x_est.est_vel_north;
  FlightHistory.state_vel_west = active_packet.x_est.est_vel_west;
  FlightHistory.state_vel_up = active_packet.x_est.est_vel_up;
  FlightHistory.gyro_bias_yaw[FlightHistory.write_pos] = active_packet.x_est.gyro_bias_yaw;
  FlightHistory.gyro_bias_yaw[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.gyro_bias_yaw;
  FlightHistory.gyro_bias_pitch[FlightHistory.write_pos] = active_packet.x_est.gyro_bias_pitch;
  FlightHistory.gyro_bias_pitch[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.gyro_bias_pitch;
  FlightHistory.gyro_bias_roll[FlightHistory.write_pos] = active_packet.x_est.gyro_bias_roll;
  FlightHistory.gyro_bias_roll[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.gyro_bias_roll;
  FlightHistory.accel_bias_x[FlightHistory.write_pos] = active_packet.x_est.accel_bias_x;
  FlightHistory.accel_bias_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.accel_bias_x;
  FlightHistory.accel_bias_y[FlightHistory.write_pos] = active_packet.x_est.accel_bias_y;
  FlightHistory.accel_bias_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.accel_bias_y;
  FlightHistory.accel_bias_z[FlightHistory.write_pos] = active_packet.x_est.accel_bias_z;
  FlightHistory.accel_bias_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.accel_bias_z;
  FlightHistory.mag_bias_x[FlightHistory.write_pos] = active_packet.x_est.mag_bias_x;
  FlightHistory.mag_bias_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.mag_bias_x;
  FlightHistory.mag_bias_y[FlightHistory.write_pos] = active_packet.x_est.mag_bias_y;
  FlightHistory.mag_bias_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.mag_bias_y;
  FlightHistory.mag_bias_z[FlightHistory.write_pos] = active_packet.x_est.mag_bias_z;
  FlightHistory.mag_bias_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.x_est.mag_bias_z;

  FlightHistory.gimbal_yaw_raw = active_packet.co.gimbal_yaw_deg;
  FlightHistory.gimbal_pitch_raw = active_packet.co.gimbal_pitch_deg;
  FlightHistory.thrust_N = active_packet.co.thrust_N;
  FlightHistory.roll_rad_sec_squared = active_packet.co.roll_rad_sec_squared;

  FlightHistory.target_pos_north = active_packet.ci.target_pos_north;
  FlightHistory.target_pos_west = active_packet.ci.target_pos_west;
  FlightHistory.target_pos_up = active_packet.ci.target_pos_up;

  FlightHistory.elapsed_time = active_packet.elapsed_time;
  FlightHistory.GND_flag = active_packet.ci.GND_val;
  FlightHistory.flight_armed = active_packet.flight_armed;
  FlightHistory.thrust_perc = active_packet.thrust_perc;
  FlightHistory.diffy_perc = active_packet.diffy_perc;
  FlightHistory.rtk_status = active_packet.rtk_status;
  FlightHistory.gps_hor_prec = active_packet.gps_hor_prec;
  FlightHistory.gps_ver_prec = active_packet.gps_ver_prec;
  FlightHistory.gps_sat_count = active_packet.gps_sat_count;

  FlightHistory.write_pos += 1;
  FlightHistory.write_pos %= FLIGHT_HISTORY_LENGTH;
  FlightHistory.read_start_pos = FlightHistory.write_pos;
  FlightHistory.read_end_pos = FlightHistory.read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}

void load_flight_replay() {
  if (FlightDataState.input_file != NULL) {
    fclose(FlightDataState.input_file);
    FlightDataState.input_file = NULL;
  }

  FlightDataState.input_file = fopen(FlightDataState.selected_file_path, "rb");
  FlightDataState.file_read_progress = 0;

  FlightDataState.file_reading_paused = true;
  FlightDataState.replay_pause_offset_us = 0;

  // determine file length
  if (FlightDataState.input_file != NULL) {
    FlightDataState.file_length = 0;
    while (true) {
      size_t read_size = fread(&active_packet, sizeof(active_packet), 1, FlightDataState.input_file);
      if (read_size == 1) {
        FlightDataState.file_length += 1;
      } else {
        break;
      }
    }
    fseek(FlightDataState.input_file, 0, SEEK_SET);
  }
}

#define RTK_READ_SIZE 256
#define RTK_WRITE_SIZE 64
#define FV_SERIAL_READ_SIZE 1024
char fv_read_buf[FV_SERIAL_READ_SIZE];
char rtk_read_buf[RTK_READ_SIZE];
char rtk_write_buf[RTK_WRITE_SIZE];
int rtk_write_pos = 0;

void flight_data_periodic() {
  if (FlightDataState.data_input_mode == MODE_SERIAL_INPUT) {
    // RTK forwarding
    if (FlightDataState.rtk_serial.is_open() && FlightDataState.fv_serial.is_open()) {
      int rtk_bytes_read = FlightDataState.rtk_serial.read(rtk_read_buf, RTK_READ_SIZE);

      if (rtk_bytes_read > 0) {
        for (int i = 0; i < rtk_bytes_read; i++) {
          if (rtk_read_buf[i] == ESCAPE_CHAR || rtk_read_buf[i] == END_CHAR || rtk_read_buf[i] == CR_CHAR || rtk_read_buf[i] == BACKSPACE_CHAR) {
            rtk_write_buf[rtk_write_pos] = ESCAPE_CHAR;
            rtk_write_pos++;
          }
          rtk_write_buf[rtk_write_pos] = rtk_read_buf[i];
          rtk_write_pos++;
          if (rtk_write_pos >= RTK_WRITE_SIZE - 1) {
            // escape newlines and backslashes
            FlightDataState.fv_serial.write("rtk ", 4);
            FlightDataState.fv_serial.write(rtk_write_buf, rtk_write_pos, true);
            rtk_write_pos = 0;
          }
        }
      }
    }

    // Flight data parsing
    if (FlightDataState.fv_serial.is_open()) {
      int fv_bytes_read = FlightDataState.fv_serial.read(fv_read_buf, FV_SERIAL_READ_SIZE);
      for (int i = 0; i < fv_bytes_read; i++) {
        flight_command_encode(fv_read_buf[i]);
      }
    }

  } else {
    if (FlightDataState.input_file != NULL && !FlightDataState.file_reading_paused) {
      uint64_t now_time = get_time_us();
      float replay_time = (now_time - FlightDataState.replay_play_start_us + FlightDataState.replay_pause_offset_us) * 1e-6;
      while (1) {
        size_t read_size = fread(&active_packet, sizeof(active_packet), 1, FlightDataState.input_file);
        if (read_size == 1) {
          FlightDataState.file_read_progress += 1;
          commit_packet();
          if (FlightHistory.elapsed_time > replay_time) {
            break; // break from the loop once the replay has caught up with real time
          }
        } else {
          FlightDataState.file_reading_paused = true; // pause replay as we have reached the end and don't want this loop called anymore
          break;
        }
      }
    }
  }
}

void reply_with_heartbeat() {
  write_to_serial_port(&FlightDataState.fv_serial, &FlightDataState.fv_serial_port_open, "hb", 2, true);
}

void write_serial_to_fv(const char *msg) {
  if (FlightDataState.data_input_mode == MODE_SERIAL_INPUT && FlightDataState.fv_serial.is_open()) {
    size_t len = strlen(msg);
    if (len != 0) {
      // write to the flight vehicle, end with newline
      FlightDataState.fv_serial.write(msg, len, true);
    }

  } else {
    printf("Failed to send serial message - make sure serial input mode is active and flight vehicle serial port is open.\n");
  }
}

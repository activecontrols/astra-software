#include "flight_data.h"
#include "flight_data_state.h"
#include "platform_win.h"

flight_history_t FlightHistory;
flight_packet_t active_packet; // may be partially filled
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
  close_serial_port(&FlightDataState.fv_serial);
  close_serial_port(&FlightDataState.rtk_serial);
}

void commit_packet() {
  FlightHistory.accel_x[FlightHistory.write_pos] = active_packet.accel_x;
  FlightHistory.accel_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_x;
  FlightHistory.accel_y[FlightHistory.write_pos] = active_packet.accel_y;
  FlightHistory.accel_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_y;
  FlightHistory.accel_z[FlightHistory.write_pos] = active_packet.accel_z;
  FlightHistory.accel_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.accel_z;
  FlightHistory.gyro_yaw[FlightHistory.write_pos] = active_packet.gyro_yaw;
  FlightHistory.gyro_yaw[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.gyro_yaw;
  FlightHistory.gyro_pitch[FlightHistory.write_pos] = active_packet.gyro_pitch;
  FlightHistory.gyro_pitch[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.gyro_pitch;
  FlightHistory.gyro_roll[FlightHistory.write_pos] = active_packet.gyro_roll;
  FlightHistory.gyro_roll[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.gyro_roll;
  FlightHistory.mag_x[FlightHistory.write_pos] = active_packet.mag_x;
  FlightHistory.mag_x[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.mag_x;
  FlightHistory.mag_y[FlightHistory.write_pos] = active_packet.mag_y;
  FlightHistory.mag_y[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.mag_y;
  FlightHistory.mag_z[FlightHistory.write_pos] = active_packet.mag_z;
  FlightHistory.mag_z[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.mag_z;
  FlightHistory.gps_pos_north = active_packet.gps_pos_north;
  FlightHistory.gps_pos_west = active_packet.gps_pos_north;
  FlightHistory.gps_pos_up = active_packet.gps_pos_north;
  FlightHistory.gps_vel_north = active_packet.gps_pos_north;
  FlightHistory.gps_vel_west = active_packet.gps_pos_north;
  FlightHistory.gps_vel_up = active_packet.gps_pos_north;

  FlightHistory.state_q_vec_new = active_packet.state_q_vec_new;
  FlightHistory.state_q_vec_0 = active_packet.state_q_vec_0;
  FlightHistory.state_q_vec_1 = active_packet.state_q_vec_1;
  FlightHistory.state_q_vec_2 = active_packet.state_q_vec_2;

  FlightHistory.state_pos_north = active_packet.state_pos_north;
  FlightHistory.state_pos_west = active_packet.state_pos_west;
  FlightHistory.state_pos_up = active_packet.state_pos_up;
  FlightHistory.state_vel_north = active_packet.state_vel_north;
  FlightHistory.state_vel_west = active_packet.state_vel_west;
  FlightHistory.state_vel_up = active_packet.state_vel_up;

  FlightHistory.gimbal_yaw_raw = active_packet.gimbal_yaw_raw;
  FlightHistory.gimbal_pitch_raw = active_packet.gimbal_pitch_raw;
  FlightHistory.thrust_N = active_packet.thrust_N;
  FlightHistory.roll_N = active_packet.roll_N;

  FlightHistory.target_pos_north = active_packet.target_pos_north;
  FlightHistory.target_pos_west = active_packet.target_pos_west;
  FlightHistory.target_pos_up = active_packet.target_pos_up;

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

  // determine file length
  if (FlightDataState.input_file != NULL) {
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

void flight_data_periodic() {
  if (FlightDataState.data_input_mode == MODE_SERIAL_INPUT) {
    if (FlightDataState.fv_serial_port_open && FlightDataState.fv_serial == INVALID_HANDLE_VALUE && FlightDataState.fv_serial_idx < FlightDataState.ports.size()) {
      open_serial_port(&FlightDataState.fv_serial, FlightDataState.ports[FlightDataState.fv_serial_idx].portName.c_str());
      if (FlightDataState.fv_serial == INVALID_HANDLE_VALUE) {
        FlightDataState.fv_serial_port_open = false; // tell the user we failed to open the port
      }
    }

    if (FlightDataState.rtk_serial_port_open && FlightDataState.rtk_serial == INVALID_HANDLE_VALUE && FlightDataState.rtk_serial_idx < FlightDataState.ports.size()) {
      open_serial_port(&FlightDataState.rtk_serial, FlightDataState.ports[FlightDataState.rtk_serial_idx].portName.c_str());
      if (FlightDataState.rtk_serial == INVALID_HANDLE_VALUE) {
        FlightDataState.rtk_serial_port_open = false; // tell the user we failed to open the port
      }
    }

    if (!FlightDataState.fv_serial_port_open && FlightDataState.fv_serial != INVALID_HANDLE_VALUE) {
      close_serial_port(&FlightDataState.fv_serial);
    }

    if (!FlightDataState.rtk_serial_port_open && FlightDataState.rtk_serial != INVALID_HANDLE_VALUE) {
      close_serial_port(&FlightDataState.rtk_serial);
    }

    // RTK forwarding
    if (FlightDataState.rtk_serial_port_open && FlightDataState.fv_serial_port_open) {
    }

    // Flight data parsing
    if (FlightDataState.fv_serial_port_open) {
    }

  } else {
    if (FlightDataState.input_file != NULL && !FlightDataState.file_reading_paused) {
      size_t read_size = fread(&active_packet, sizeof(active_packet), 1, FlightDataState.input_file);
      if (read_size == 1) {
        FlightDataState.file_read_progress += 1;
        commit_packet();
      }
    }
  }
}

void write_serial_to_fv(const char *msg) {
  if (FlightDataState.data_input_mode == MODE_SERIAL_INPUT && FlightDataState.fv_serial_port_open) {
    size_t len = strlen(msg);
    if (len != 0) {
      // write to the flight vehicle, end with newline
      write_to_serial_port(&FlightDataState.fv_serial, &FlightDataState.fv_serial_port_open, msg, len, true);
    }

  } else {
    printf("Failed to send serial message - make sure serial input mode is active and flight vehicle serial port is open.\n");
  }
}

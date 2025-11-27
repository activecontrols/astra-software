#ifndef ASTRA_GS_SERIAL_H
#define ASTRA_GS_SERIAL_H

#define OUT_BUF_SIZE 32768 // adjust as needed
extern char concat_msg_buf[OUT_BUF_SIZE];

typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;
  float mag_x;
  float mag_y;
  float mag_z;
  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;
  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float state_q_vec_new;
  float state_q_vec_0;
  float state_q_vec_1;
  float state_q_vec_2;
  float state_pos_north;
  float state_pos_west;
  float state_pos_up;
  float state_vel_north; // TODO - vis this
  float state_vel_west;
  float state_vel_up;
  float state_0; // TODO - vis this
  float state_1;
  float state_2;
  float state_3;
  float state_4;
  float state_5;

  float gimbal_yaw_deg;
  float gimbal_pitch_deg;
  float thrust_N;
  float roll_N;

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;

  float elapsed_time;
  float GND_flag;
  float flight_armed;
  float thrust_perc;
  float diffy_perc;
} state_packet_t;

extern state_packet_t state_packet;
void poll_serial();
void write_serial(const char *msg);
bool init_serial(char *com_port);
void deinit_serial();

#endif

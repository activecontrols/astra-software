#include "GPS.h"
#include "Mag.h"
#include "Router.h"
#include "Prop.h"
#include "IMU.h"
#include <SPI.h>
#include <Arduino.h>


#define IMU_CS D6

void fifo_callback(Sensor_Event*);
IMU imu(IMU_CS, &SPI, &fifo_callback);

Sensor_Event last_event;
int packets_read = 0;

void fixedpoint_to_float(int16_t *, float *, const uint8_t, const uint8_t, float);


void read_sensor_packet(const char* _){
  static char output_s[500];
  static char sens_mask_str[9];
  float accel[3];
  float gyro[3];
  float temperature_c;
  sens_mask_str[8] = '\0';

  imu.read_fifo();

  // sensor mask has (lower 4) bits set to convey whether or not gyro, accel, temperature, and/or time data are valid
  // convert sensor mask to string in binary representationo
  for (int i = 0; i < 8; ++i){
    sens_mask_str[i] = ((last_event.sensor_mask >> (7 - i)) & 1) + '0';
  }


  // gyro defaults to fsr of 2000dps
  // accel defaults to fsr of 4g
  fixedpoint_to_float(last_event.accel, accel, 16, 3, 4.0);
  fixedpoint_to_float(last_event.gyro, gyro, 16, 3, 2000.0);

  temperature_c = last_event.temperature / 2.07 + 25; // conversion formula from datasheet to convert to celsius
  
  snprintf(output_s, sizeof(output_s) - 1,
    "Sensor_mask:..........%s\n"
    "Timestamp_fsync:......%hu\n"
    "Accel (g):............[%6.4f, %6.4f, %6.4f]\n"
    "Gyro (dps):...........[%9.2f, %9.2f, %9.2f]\n"
    "Temperature (C):......%.3f\n"
    "Packets read so far:..%d\n",
    sens_mask_str,
    last_event.timestamp_fsync,
    accel[0], accel[1], accel[2],
    gyro[0], gyro[1], gyro[2],
    temperature_c,
    packets_read
  );

  Router::println(output_s);
}

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
}

void help(const char *args) {
  // ignore args
  Router::print_all_cmds();
}

void setup() {
  Router::begin();
  Router::println("Controller started.");

  Prop::begin();
  Mag::begin();
  GPS::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({help, "help"});
  Router::add({read_sensor_packet, "read_sensor_packet"});

  SPI.begin();
  
  int error{0};
  error |= imu.begin();
  error |= imu.enable_accel();
  error |= imu.enable_gyro();
  if (error){
    Router::println("Error while initializing IMU and enabling accel/gyro.");
  }
  else{
    Router::println("IMU Initialized. Accel and gyro enabled.");
  }
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}

 
void fifo_callback(Sensor_Event *event){
  last_event = *event;
  ++packets_read;
}


void fixedpoint_to_float(int16_t *in, float *out, const uint8_t fxp_shift, const uint8_t dim, float sf)
{
	int i;
	float scale = 1.f / (1 << fxp_shift) * sf;

	for (i = 0; i < dim; i++)
		out[i] = scale * in[i];
}

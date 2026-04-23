#include "CommandRouter.h"
#include "CommsSerial.h"
#include "FlashLogging.h"
#include "GPS.h"
#include "GimbalServos.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "SPI.h"
#include "TrajectoryFollower.h"
#include "TrajectoryLoader.h"
#include "TrajectoryLogger.h"
#include <Arduino.h>

CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
CommsSerial_t<USBSerial> USB_CommsSerial;

void ping(const char *args) {
  CommsSerial.println("pong");
  CommsSerial.print("args: ");
  CommsSerial.println(args == nullptr ? "null" : args);
}

void characterize_grid(const char *args) {
  bool flip = args[0] == 'r';
  const float deg_limit = 10;
  float deg_min = -deg_limit;
  float deg_max = deg_limit;
  const float interp_time = 1;
  const float step_size = 0.5;

  const unsigned long delay_time = 3000000;

  float a = deg_min;
  float b = -deg_limit;

  IMU::Data last_packet;

  GimbalServos::centerGimbal();
  delay(500);

  CommsSerial.printf("<<< LOG BEGIN >>>\n");

  CommsSerial.print("_grid_.csv\n");
  CommsSerial.print("Time (s),Angle A (deg),Angle B (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");

  unsigned long test_start_time = micros();

  while (micros() - test_start_time < delay_time) {
    IMU::IMUs[0].read_latest(&last_packet);

    double t = (micros() - test_start_time) * 1e-6;

    CommsSerial.printf("%.5lf, %.2f, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1],
                   last_packet.gyro[2]);

    delay(1);
  }

  for (; b <= deg_limit && !Serial.available(); b += step_size) {
    unsigned long start_time = micros();

    while (1) {

      IMU::IMUs[0].read_latest(&last_packet);

      unsigned long now_time = micros();
      double t = (now_time - start_time) * 1e-6;

      CommsSerial.printf("%.5lf, %.2f, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", (now_time - test_start_time) * 1e-6, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2],
                     last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

      if (t > interp_time)
        break;

      a = (deg_max - deg_min) * (t / interp_time) + deg_min;

      if (flip)
        GimbalServos::setGimbalAngle(b, a);
      else
        GimbalServos::setGimbalAngle(a, b);
      delay(1);
    }

    // swap min and max target angles to zig zag
    float temp = deg_min;
    deg_min = deg_max;
    deg_max = temp;
  }

  while (1) {
    int a = Serial.read();
    if (a == '\n' || a == -1)
      break;
  }

  CommsSerial.printf("<<< LOG END >>>\n");
}

void characterize_frequency(const char *param) {

  bool axis_top = param[0] == 'u';
  const double start_delay = 3; // 3 seconds

  const float amplitude = 7;
  IMU::Data last_packet;

  CommsSerial.printf("<<< LOG BEGIN >>>\n");

  CommsSerial.printf("_frequency_%s.csv\n", axis_top ? "top" : "bottom");

  CommsSerial.print("Time (s),Angle A (deg),Angle B (deg),Frequency (Hz),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");
  float a = 0;
  float f = 0;

  GimbalServos::setGimbalAngle(a, 0);
  delay(1000);

  unsigned long start_time = micros();

  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time - start_delay) * 1e-6;
    IMU::IMUs[0].read_latest(&last_packet);

    CommsSerial.printf("%.5lf, %.2f, %.2f, %.5lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, a, 0, f, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0],
                   last_packet.gyro[1], last_packet.gyro[2]);

    if (t < start_delay) {
      delay(1);
      continue;
    }

    a = amplitude * sin(t * t);
    f = t / PI;

    if (axis_top)
      GimbalServos::setGimbalAngle(0, a);
    else
      GimbalServos::setGimbalAngle(a, 0);

    delay(1);
  }

  while (Serial.read() != '\n')
    ;

  CommsSerial.print("\n\n<<< LOG END >>>\n");
}

void circle_test(const char *arg) {
  // this should be how to paramaterize a circle???
  // R = circle radius
  // L = prop length
  // a = angle a
  // b = angle b

  // x = Lsin a = Rcost
  // y = Lsin b = Rsint

  // a = arcsin(R/L cost)
  // b = arcsin(R/L sint)

  // k = R/L (must be |k| <= 1)

  const double amplitude = 7;

  const double k = sin(amplitude * PI / 180.0);

  float a = 0;
  float b = 0;

  float servo_bottom, servo_top;

  GimbalServos::setGimbalAngle(a, b);

  delay(1000);

  unsigned long start_time = micros();

  double f = 2.0;

  if (strlen(arg) > 0) {
    f = atof(arg);
  }

  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time) * 1e-6;

    float w = 2 * PI * f;
    a = amplitude * cos(w * t);
    b = amplitude * sin(w * t);

    GimbalServos::setGimbalAngle(a, b);

    delay(1);
  }

  while (Serial.read() != '\n')
    ;
}

void step_test(const char *param) {
  const double period = 3;
  const float amplitude = 7;

  float angle = 0;
  const double delay_time = 3;

  bool upper = param[0] == 'u';

  CommsSerial.printf("<<< LOG BEGIN >>>\n");

  CommsSerial.printf("_step_%s.csv\n", upper ? "top" : "bottom");

  CommsSerial.printf("Time (s),Angle %s (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n", upper ? "upper" : "lower");

  unsigned long start_time = micros();

  IMU::Data last_packet;

  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time) * 1e-6 - delay_time;

    if (t >= 0)
      angle = fmod(t, 2 * period) < period ? amplitude : -amplitude; // square wave

    if (upper) {
      GimbalServos::setGimbalAngle(0, angle);
    } else {
      GimbalServos::setGimbalAngle(angle, 0);
    }

    IMU::IMUs[0].read_latest(&last_packet);

    CommsSerial.printf("%.5lf, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, angle, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1],
                   last_packet.gyro[2]);

    delay(1);
  }

  while (Serial.read() != '\n')
    ;

  CommsSerial.print("\n\n<<< LOG END >>>\n");
}

void servo_characterization(const char *args) {
  bool flip = args[0] == 'r';
  const float deg_limit = 40;
  float deg_min = -deg_limit;
  float deg_max = deg_limit;
  const float wait_time = 1;
  const float step_size = 5;

  const unsigned long delay_time = 3000000;

  float a = deg_min;
  float b = deg_min;

  IMU::Data last_packet;

  GimbalServos::setGimbalAngle(a, b);
  delay(500);

  CommsSerial.printf("<<< LOG BEGIN >>>\n");

  CommsSerial.print("_servo_grid_.csv\n");
  CommsSerial.print("Time (s),Servo Angle Bottom (deg),Servo Angle Top (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");

  unsigned long test_start_time = micros();

  while (micros() - test_start_time < delay_time) {
    IMU::IMUs[0].read_latest(&last_packet);

    double t = (micros() - test_start_time) * 1e-6;

    CommsSerial.printf("%.5lf,%.2f,%.2f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", t, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1],
                   last_packet.gyro[2]);

    delay(1);
  }

  for (; b <= deg_limit && !Serial.available(); b += step_size) {
    unsigned long start_time = micros();

    while (!Serial.available()) {

      IMU::IMUs[0].read_latest(&last_packet);

      unsigned long now_time = micros();
      double t = (now_time - start_time) * 1e-6;

      CommsSerial.printf("%.5lf,%.2f,%.2f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", (now_time - test_start_time) * 1e-6, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2],
                     last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

      if (t > (2 * deg_limit + 1) * wait_time / step_size) {
        break;
      }

      a = step_size * ((int)(t / wait_time)) * (deg_min > deg_max ? -1 : 1) + deg_min;
      GimbalServos::setGimbalAngle(a, b);
      // GimbalServos::setGimbalAngle(a, b);
      delay(1);
    }

    // swap min and max target angles to zig zag
    float temp = deg_min;
    deg_min = deg_max;
    deg_max = temp;
  }

  while (1) {
    int a = Serial.read();
    if (a == '\n' || a == -1)
      break;
  }

  CommsSerial.printf("<<< LOG END >>>\n");
}

void step_grid(const char *args) {
  bool flip = args[0] == 'r';
  const float deg_limit = 10;
  float deg_min = -deg_limit;
  float deg_max = deg_limit;
  const float wait_time = 1;
  const float step_size = 2;

  const unsigned long delay_time = 3000000;

  float a = 0;
  float b = 0;

  IMU::Data last_packet;

  float servo_bottom, servo_top;

  GimbalServos::setGimbalAngle(a, b);
  delay(1000);

  CommsSerial.printf("<<< LOG BEGIN >>>\n");

  CommsSerial.print("_gimbal_grid_.csv\n");
  CommsSerial.print("Time (s),Gimbal Angle Bottom (deg),Gimbal Angle Top (deg),Servo Angle Bottom (deg),Servo Angle Top (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y "
                "(rad/s),Gyro Z (rad/s)\n");

  unsigned long test_start_time = micros();

  while (micros() - test_start_time < delay_time) {
    IMU::IMUs[0].read_latest(&last_packet);

    double t = (micros() - test_start_time) * 1e-6;

    CommsSerial.printf("%.5lf,%.2f,%.2f,%.4f,%.4f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", t, a, b, servo_bottom, servo_top, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2],
                   last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

    delay(1);
  }

  for (b = deg_min; b <= deg_limit && !Serial.available(); b += step_size) {
    unsigned long start_time = micros();

    while (!Serial.available()) {

      IMU::IMUs[0].read_latest(&last_packet);

      unsigned long now_time = micros();
      double t = (now_time - start_time) * 1e-6;

      CommsSerial.printf("%.5lf,%.2f,%.2f,%.4f,%.4f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", (now_time - test_start_time) * 1e-6, a, b, servo_bottom, servo_top, last_packet.acc[0],
                     last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

      if (t > (2 * deg_limit + 1) * wait_time / step_size) {
        break;
      }

      a = step_size * ((int)(t / wait_time)) * (deg_min > deg_max ? -1 : 1) + deg_min;
      // GimbalServos::setGimbalAngle(a, b);
      GimbalServos::setGimbalAngle(a, b);
      delay(1);
    }

    // swap min and max target angles to zig zag
    float temp = deg_min;
    deg_min = deg_max;
    deg_max = temp;
  }

  while (1) {
    int a = Serial.read();
    if (a == '\n' || a == -1)
      break;
  }

  CommsSerial.printf("<<< LOG END >>>\n");
}

void setup() {
  delay(3000);
  SPI.begin(); // spi is a shared interface, so we always begin here
  CommsSerial.begin(57600);
  CommsSerial.println("Controller started.");

  // TODO - configure CS somewhere else!
  digitalWrite(PB7, HIGH);
  digitalWrite(PC12, HIGH);

  digitalWrite(PD7, HIGH); // MAG CS 1
  digitalWrite(PE7, HIGH); // MAG CS 2
  digitalWrite(PE4, HIGH); // MAG CS 3

  digitalWrite(PB3, HIGH); // IMU CS 3
  digitalWrite(PB4, HIGH); // IMU CS 2
  digitalWrite(PB5, HIGH); // IMU CS 1

  // TODO - configure CS somewhere else!
  pinMode(PB7, OUTPUT);
  pinMode(PC12, OUTPUT);
  pinMode(PE7, OUTPUT);
  pinMode(PE4, OUTPUT);

  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PD7, OUTPUT);

  pinMode(PB6, OUTPUT);

  CommandRouter::begin();
  // Prop::begin();
  // Mag::begin();
  // GPS::begin();
  // IMU::begin();
  GimbalServos::begin();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();
  // Logging::begin();
  // TrajectoryLogger::begin();

  CommandRouter::add(ping, "ping"); // example registration

  // CommandRouter::add(Router::print_all_cmds, "help");
  CommandRouter::add(characterize_grid, "grid");
  CommandRouter::add(characterize_frequency, "frequency");
  CommandRouter::add(circle_test, "circle");
  CommandRouter::add(step_test, "step");
  CommandRouter::add(servo_characterization, "servo_characterization");
  CommandRouter::add(step_grid, "step_grid");
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}
#include "GPS.h"
#include "IMU.h"
#include "Mag.h"
#include "Prop.h"
#include "Router.h"
#include "TrajectoryFollower.h"
#include "TrajectoryLoader.h"
#include "gimbal_servos.h"
#include <Arduino.h>

void ping(const char *args) {
  Router::println("pong");
  Router::print("args: ");
  Router::println(args == nullptr ? "null" : args);
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

  Router::printf("<<< LOG BEGIN >>>\n");

  Router::print("_grid_.csv\n");
  Router::print("Time (s),Angle A (deg),Angle B (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");

  unsigned long test_start_time = micros();

  while (micros() - test_start_time < delay_time)
  {
    IMU::IMUs[0].read_latest(&last_packet);

    double t = (micros() - test_start_time) * 1e-6;

    Router::printf("%.5lf, %.2f, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0],
                   last_packet.gyro[1], last_packet.gyro[2]);

    delay(1);
  }

  for (; b <= deg_limit && !Serial.available(); b += step_size) {
    unsigned long start_time = micros();

    while (1) {

      IMU::IMUs[0].read_latest(&last_packet);

      unsigned long now_time = micros();
      double t = (now_time - start_time) * 1e-6;

      Router::printf("%.5lf, %.2f, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", (now_time - test_start_time) * 1e-6, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0],
                     last_packet.gyro[1], last_packet.gyro[2]);

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

  while (1)
  {
    int a = Serial.read();
    if (a == '\n' || a == -1) break;
  }

  Router::printf("<<< LOG END >>>\n");
}

void characterize_frequency(const char *param) {

  bool axis_top = param[0] == 'u';
  const double start_delay = 3; // 3 seconds

  const float amplitude = 5;
  IMU::Data last_packet;

  Router::printf("<<< LOG BEGIN >>>\n");

  Router::printf("_frequency_%s.csv\n", axis_top ? "top" : "bottom");

  Router::print("Time (s),Angle A (deg),Angle B (deg),Frequency (Hz),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");
  float a = 0;
  float f = 0;

  GimbalServos::setGimbalAngle(a, 0);
  delay(1000);

  unsigned long start_time = micros();

  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time - start_delay) * 1e-6;
    IMU::IMUs[0].read_latest(&last_packet);

    Router::printf("%.5lf, %.2f, %.2f, %.5lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, a, 0, f, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0],
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

  Router::print("\n\n<<< LOG END >>>\n");
}

void circle_test() {
  IMU::Data last_packet;

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

  const double amplitude = 10;

  const double k = sin(amplitude * PI / 180.0);

  float a = 0;
  float b = 0;

  float servo_bottom, servo_top;

  GimbalServos::setGimbalAngle(a, b, &servo_bottom, &servo_top);

  Router::printf("<<< LOG BEGIN >>>\n");

  Router::printf("_circle_test.csv\n");

  Router::print("Time (s),Gimbal Angle Bottom (deg),Gimbal Angle Top (deg),Servo Angle Bottom (deg),Servo Angle Top (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y "
                "(rad/s),Gyro Z (rad/s)\n");
  delay(1000);

  unsigned long start_time = micros();

  while (!Serial.available())
  {
    unsigned long now_time = micros();
    double t = (now_time - start_time) * 1e-6;

    if (t > 3) break;

    IMU::IMUs[0].read_latest(&last_packet);

    Router::printf("%.5lf,%.2f,%.2f,%.4f,%.4f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", t, a, b, servo_bottom, servo_top, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2],
                   last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

    delay(1);
  }

  while (!Serial.available()) {
    unsigned long now_time = micros();
    double t = (now_time - start_time) * 1e-6;

    IMU::IMUs[0].read_latest(&last_packet);

    Router::printf("%.5lf,%.2f,%.2f,%.4f,%.4f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", t, a, b, servo_bottom, servo_top, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2],
                   last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

    float deg = (int)(t - 3.0) * 10;
    a = asin(k * cos(deg * PI / 180.0)) * 180 / PI;
    b = asin(k * sin(deg * PI / 180.0)) * 180 / PI;

    GimbalServos::setGimbalAngle(a, b, &servo_bottom, &servo_top);

    delay(1);
  }

  while (Serial.read() != '\n')
    ;

  Router::print("\n\n<<< LOG END >>>\n");
}

void step_test(const char *param) {
  const double period = 3;
  const float amplitude = 10;

  float angle = 0;
  const double delay_time = 3;

  bool upper = param[0] == 'u';

  Router::printf("<<< LOG BEGIN >>>\n");

  Router::printf("_step_%s.csv\n", upper ? "top" : "bottom");

  Router::printf("Time (s),Angle %s (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n", upper ? "upper" : "lower");

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

    Router::printf("%.5lf, %.2f, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf, %.15lf\n", t, angle, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1],
                   last_packet.gyro[2]);

    delay(1);
  }

  while (Serial.read() != '\n')
    ;

  Router::print("\n\n<<< LOG END >>>\n");
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

  GimbalServos::setServoAngle(a, b);
  delay(500);

  Router::printf("<<< LOG BEGIN >>>\n");

  Router::print("_servo_grid_.csv\n");
  Router::print("Time (s),Servo Angle Bottom (deg),Servo Angle Top (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");


  unsigned long test_start_time = micros();

  while (micros() - test_start_time < delay_time) {
    IMU::IMUs[0].read_latest(&last_packet);

    double t = (micros() - test_start_time) * 1e-6;

    Router::printf("%.5lf,%.2f,%.2f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", t, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0],
                   last_packet.gyro[1], last_packet.gyro[2]);

    delay(1);
  }

  for (; b <= deg_limit && !Serial.available(); b += step_size) {
    unsigned long start_time = micros();

    while (!Serial.available()) {

      IMU::IMUs[0].read_latest(&last_packet);

      unsigned long now_time = micros();
      double t = (now_time - start_time) * 1e-6;

      Router::printf("%.5lf,%.2f,%.2f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", (now_time - test_start_time) * 1e-6, a, b, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2],
                     last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

      if (t > (2 * deg_limit + 1) * wait_time / step_size)
      {
        break;
      }

      a = step_size * ((int)(t / wait_time)) * (deg_min > deg_max ? -1 : 1) + deg_min;
      GimbalServos::setServoAngle(a, b);
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

  Router::printf("<<< LOG END >>>\n");
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

  GimbalServos::setGimbalAngle(a, b, &servo_bottom, &servo_top);
  delay(1000);

  Router::printf("<<< LOG BEGIN >>>\n");

  Router::print("_gimbal_grid_.csv\n");
  Router::print("Time (s),Gimbal Angle Bottom (deg),Gimbal Angle Top (deg),Servo Angle Bottom (deg),Servo Angle Top (deg),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s)\n");

  unsigned long test_start_time = micros();

  while (micros() - test_start_time < delay_time) {
    IMU::IMUs[0].read_latest(&last_packet);

    double t = (micros() - test_start_time) * 1e-6;

    Router::printf("%.5lf,%.2f,%.2f,%.4f,%.4f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", t, a, b, servo_bottom, servo_top, last_packet.acc[0], last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1],
                   last_packet.gyro[2]);

    delay(1);
  }

  for (b = deg_min; b <= deg_limit && !Serial.available(); b += step_size) {
    unsigned long start_time = micros();

    while (!Serial.available()) {

      IMU::IMUs[0].read_latest(&last_packet);

      unsigned long now_time = micros();
      double t = (now_time - start_time) * 1e-6;

      Router::printf("%.5lf,%.2f,%.2f,%.4f,%.4f,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf,%.15lf\n", (now_time - test_start_time) * 1e-6, a, b, servo_bottom, servo_top, last_packet.acc[0],
                     last_packet.acc[1], last_packet.acc[2], last_packet.gyro[0], last_packet.gyro[1], last_packet.gyro[2]);

      if (t > (2 * deg_limit + 1) * wait_time / step_size) {
        break;
      }

      a = step_size * ((int)(t / wait_time)) * (deg_min > deg_max ? -1 : 1) + deg_min;
      // GimbalServos::setServoAngle(a, b);
      GimbalServos::setGimbalAngle(a, b, &servo_bottom, &servo_top);
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

  Router::printf("<<< LOG END >>>\n");
}

void setup() {
  delay(3000);
  Router::begin();
  Router::println("Controller started.");

  SPI.begin();

  // Prop::begin();
  // Mag::begin();
  // GPS::begin();
  IMU::begin();
  GimbalServos::init();
  // TrajectoryLoader::begin();
  // TrajectoryFollower::begin();

  Router::add({ping, "ping"}); // example registration
  Router::add({Router::print_all_cmds, "help"});
  Router::add({characterize_grid, "grid"});
  Router::add({characterize_frequency, "frequency"});
  Router::add({circle_test, "circle"});
  Router::add({step_test, "step"});
  Router::add({servo_characterization, "servo_characterization"});
  Router::add({step_grid, "step_grid"});
}

void loop() {
  Router::run(); // loop only runs once, since there is an internal loop in Router::run()
}
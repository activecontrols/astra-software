#include "Servo.h"
#include "portenta_pins.h"
#include <Prop.h>
#include <Router.h>

#include <Arduino.h>

#define MIN_PULSE 1000
#define MAX_PULSE 2000
#define ARM_TIME 5000

namespace Prop {

Servo esc1;
Servo esc2;

int current_throttle_1 = MIN_PULSE;
int current_throttle_2 = MIN_PULSE;
bool armed = false;

#define THRUST_MEASUREMENT_QTY 9
#define ROLL_MEASUREMENT_QTY 4

// maps thrust in N to thrust %
float thrust_table[2][THRUST_MEASUREMENT_QTY] = {{0, 1.96029, 3.4305075, 4.900725, 6.3709425, 8.3312325, 9.80145, 13.2319575, 15.68232}, // Thrust (N)
                                                 {0, 20, 30, 40, 50, 60, 70, 80, 90}};                                                   // Thurst (%)

// contains diffy %
float roll_table_roll_perc[ROLL_MEASUREMENT_QTY] = {-5, 0, 2.5, 5};

// for each thrust in %, contains rolls in rad/sec^2 in order of roll_table_roll_perc
float roll_table[THRUST_MEASUREMENT_QTY][ROLL_MEASUREMENT_QTY] = {
    {-100, 0, 100, 100},                                    // 0%
    {-0.390878977, -0.097806543, 0.024223127, 0.204754574}, // 20%
    {-0.339431146, -0.115873367, 0.083320491, 0.218522341}, // 30%
    {-0.179999844, -0.0842441, 0.113443593, 0.391775541},   // 40%
    {-0.027414388, -0.120395976, 0.08948976, 0.447323613},  // 50%
    {-0.031847838, -0.086742267, 0.1, 0.6},                 // 60%
    {-0.051756141, 0.018706414, 0.263005152, 0.840379736},  // 70%
    {-0.131469118, 0.202871203, 0.437694149, 1.184869617},  // 80%
    {-0.202587043, 0.022354158, 0.59466497, 1.231386159}    // 90%
};

// maps v from (min_in, max_in) to (min_out, max_out)
float linear_interpolation(float v, float min_in, float max_in, float min_out, float max_out) {
  return (v - min_in) / (max_in - min_in) * (max_out - min_out) + min_out;
}

// maps thrust in newtons and roll in rad/sec^2 to the primary thrust percentage and differential percentage
void get_prop_perc(float thrust_N, float roll_accel, float *thrust_perc, float *diffy_perc) {
  int thrust_idx_low = 0;
  int thrust_idx_high = 0;
  float thrust_lerp = 0; // 0 to 1

  if (thrust_N < thrust_table[0][0]) {
    thrust_idx_low = 0;
    thrust_idx_high = 0;
    thrust_lerp = 0;
    *thrust_perc = thrust_table[1][0];
  } else if (thrust_N >= thrust_table[0][THRUST_MEASUREMENT_QTY - 1]) {
    thrust_idx_low = THRUST_MEASUREMENT_QTY - 1;
    thrust_idx_high = THRUST_MEASUREMENT_QTY - 1;
    thrust_lerp = 0;
    *thrust_perc = thrust_table[1][THRUST_MEASUREMENT_QTY - 1];
  } else {
    for (int i = 0; i < THRUST_MEASUREMENT_QTY - 1; i++) {
      if (thrust_table[0][i] <= thrust_N && thrust_N < thrust_table[0][i + 1]) {
        thrust_idx_low = i;
        thrust_idx_high = i + 1;
        thrust_lerp = linear_interpolation(thrust_N, thrust_table[0][i], thrust_table[0][i + 1], 0, 1);
        *thrust_perc = linear_interpolation(thrust_N, thrust_table[0][i], thrust_table[0][i + 1], thrust_table[1][i], thrust_table[1][i + 1]);
      }
    }
  }

  float roll_low = 0;
  float roll_high = 0;

  if (roll_accel < roll_table[thrust_idx_low][0]) {
    roll_low = roll_table_roll_perc[0];
  } else if (roll_accel >= roll_table[thrust_idx_low][ROLL_MEASUREMENT_QTY - 1]) {
    roll_low = roll_table_roll_perc[ROLL_MEASUREMENT_QTY - 1];
  } else {
    for (int i = 0; i < ROLL_MEASUREMENT_QTY - 1; i++) {
      if (roll_table[thrust_idx_low][i] <= roll_accel && roll_accel < roll_table[thrust_idx_low][i + 1]) {
        roll_low = linear_interpolation(roll_accel, roll_table[thrust_idx_low][i], roll_table[thrust_idx_low][i + 1], roll_table_roll_perc[i], roll_table_roll_perc[i + 1]);
      }
    }
  }

  if (roll_accel < roll_table[thrust_idx_high][0]) {
    roll_high = roll_table_roll_perc[0];
  } else if (roll_accel >= roll_table[thrust_idx_high][ROLL_MEASUREMENT_QTY - 1]) {
    roll_high = roll_table_roll_perc[ROLL_MEASUREMENT_QTY - 1];
  } else {
    for (int i = 0; i < ROLL_MEASUREMENT_QTY - 1; i++) {
      if (roll_table[thrust_idx_high][i] <= roll_accel && roll_accel < roll_table[thrust_idx_high][i + 1]) {
        roll_high = linear_interpolation(roll_accel, roll_table[thrust_idx_high][i], roll_table[thrust_idx_high][i + 1], roll_table_roll_perc[i], roll_table_roll_perc[i + 1]);
      }
    }
  }

  *diffy_perc = linear_interpolation(thrust_lerp, 0, 1, roll_low, roll_high);
}

void set_throttle_micros(int prop1_us, int prop2_us) {
  prop1_us = constrain(prop1_us, MIN_PULSE, MAX_PULSE);
  prop2_us = constrain(prop2_us, MIN_PULSE, MAX_PULSE);

  current_throttle_1 = prop1_us;
  current_throttle_2 = prop2_us;

  esc1.writeMicroseconds(prop1_us);
  esc2.writeMicroseconds(prop2_us);
}

void set_throttle_roll(float overall_pct, float differential) {
  set_throttle(overall_pct + differential / 2.0, overall_pct - differential / 2.0);
}

void set_throttle(float prop1_pct, float prop2_pct) {
  prop1_pct = constrain(prop1_pct, 0, 100);
  prop2_pct = constrain(prop2_pct, 0, 100);

  int m1_us = MIN_PULSE + (MAX_PULSE - MIN_PULSE) * prop1_pct / 100.0;
  int m2_us = MIN_PULSE + (MAX_PULSE - MIN_PULSE) * prop2_pct / 100.0;

  set_throttle_micros(m1_us, m2_us);
}

void stop() {
  set_throttle(0, 0);
}

void arm() {
  esc1.writeMicroseconds(MIN_PULSE);
  esc2.writeMicroseconds(MIN_PULSE);
  delay(ARM_TIME);
  armed = true;
}

int get_throttle_1() {
  return current_throttle_1;
}

int get_throttle_2() {
  return current_throttle_2;
}

bool is_armed() {
  return armed;
}

// Router commands -------------------------------------------------------------

void cmd_set_both(const char *args) {
  double vals[2];
  if (!Router::parse_doubles(args, vals, 2)) {
    Router::println("Usage: prop_set_both <prop1 %> <prop2 %>");
    return;
  }

  set_throttle(vals[0], vals[1]);
  Router::mprintln("Throttle set to: ", vals[0], "% ", vals[1], "%");
}

void cmd_set(const char *args) {
  double vals[2];
  if (!Router::parse_doubles(args, vals, 2)) {
    if (!Router::parse_doubles(args, vals, 1)) {
      Router::println("Usage: prop_set <throttle %> [<roll %>]");
      return;
    }
    vals[1] = 0.0; // default roll to 0
  }

  set_throttle_roll(vals[0], vals[1]);
  Router::mprintln("Throttle set to: ", vals[0], "% ", " differential ", vals[1], "%");
}

void cmd_stop() {
  stop();
  Router::println("Stopped.");
}

void cmd_arm() {
  Router::println("Arming...");
  arm();
  Router::println("Armed.");
}

void cmd_status() {
  Router::mprintln("Manually armed: ", armed ? "Yes" : "No");
  float p1 = (current_throttle_1 - MIN_PULSE) * 100.0 / (MAX_PULSE - MIN_PULSE);
  float p2 = (current_throttle_2 - MIN_PULSE) * 100.0 / (MAX_PULSE - MIN_PULSE);
  Router::mprintln("prop 1: ", p1, "%");
  Router::mprintln("prop 2: ", p2, "%");
}

void cmd_ramp_test(const char *args) {
  int max_pct = 30; // default 30
  if (args && strlen(args) > 0) {
    max_pct = atoi(args);
  }

  Router::mprintln("Ramping to ", max_pct, "% over 5 seconds...");

  for (int i = 0; i <= max_pct; i++) {
    set_throttle(i, i);
    delay(5000 / max_pct);
  }

  Router::println("Holding for 2 seconds...");
  delay(2000);

  Router::println("Ramping down...");
  for (int i = max_pct; i >= 0; i--) {
    set_throttle(i, i);
    delay(2000 / max_pct);
  }

  stop();
  Router::println("Ramp test complete.");
}

void cmd_contra_test() {
  Router::println("Testing contra...");
  float base = 30.0;
  float maxdiff = 20.0;
  for (int i = 0; i <= 100; i++) {
    set_throttle_roll(base, maxdiff * sin(2 * PI * i / 100.0)); // one full period.
    delay(100);
  }
  stop();
  Router::println("Contra test complete.");
}

// -----------------------------------------------------------------------------

void begin() {
  esc1.attach(PROP1_PIN);
  esc2.attach(PROP2_PIN);
  set_throttle_micros(MIN_PULSE, MIN_PULSE); // this is pretty much arming assuming no commands are run for a couple seconds

  Router::add({cmd_arm, "prop_arm"});
  Router::add({cmd_stop, "prop_stop"});
  Router::add({cmd_set_both, "prop_set_both"});
  Router::add({cmd_set, "prop_set"});
  Router::add({cmd_status, "prop_status"});
  Router::add({cmd_ramp_test, "prop_ramp"});
  Router::add({cmd_contra_test, "prop_contra"});

  Router::mprintln("Initialized on pins ", PROP1_PIN, " and ", PROP2_PIN, ".");
  Router::println("Run 'prop_arm' to arm ESCs.");
}

} // namespace Prop
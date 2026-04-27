#include "Prop.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "fc_pins.h"
#include <Servo.h>

#define MIN_PULSE 1000
#define MAX_PULSE 2000
#define ARM_TIME 5000

namespace Prop {

Servo esc1;
Servo esc2;

int current_throttle_1 = MIN_PULSE;
int current_throttle_2 = MIN_PULSE;
bool armed = false;

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
  double prop1;
  double prop2;
  if (sscanf(args, "%lf %lf", &prop1, &prop2) != 2) {
    CommsSerial.println("Usage: prop_set_both <prop1 %> <prop2 %>");
    return;
  }

  set_throttle(prop1, prop2);
  CommsSerial.mprintln("Throttle set to: ", prop1, "% ", prop2, "%");
}

void cmd_set(const char *args) {
  double overall;
  double differential;
  if (sscanf(args, "%lf %lf", &overall, &differential) != 2) {
    if (sscanf(args, "%lf", &overall) != 1) {
      CommsSerial.println("Usage: prop_set <throttle %> [<roll %>]");
      return;
    }
    differential = 0.0; // default roll to 0
  }

  set_throttle_roll(overall, differential);
  CommsSerial.mprintln("Throttle set to: ", overall, "% ", " differential ", differential, "%");
}

void cmd_stop() {
  stop();
  CommsSerial.println("Stopped.");
}

void cmd_arm() {
  CommsSerial.println("Arming...");
  arm();
  CommsSerial.println("Armed.");
}

void cmd_status() {
  CommsSerial.mprintln("Manually armed: ", armed ? "Yes" : "No");
  float p1 = (current_throttle_1 - MIN_PULSE) * 100.0 / (MAX_PULSE - MIN_PULSE);
  float p2 = (current_throttle_2 - MIN_PULSE) * 100.0 / (MAX_PULSE - MIN_PULSE);
  CommsSerial.mprintln("prop 1: ", p1, "%");
  CommsSerial.mprintln("prop 2: ", p2, "%");
}

void cmd_ramp_test(const char *args) {
  int max_pct = 30; // default 30
  if (args && strlen(args) > 0) {
    max_pct = atoi(args);
  }

  CommsSerial.mprintln("Ramping to ", max_pct, "% over 5 seconds...");

  for (int i = 0; i <= max_pct; i++) {
    set_throttle(i, i);
    delay(5000 / max_pct);
  }

  CommsSerial.println("Holding for 2 seconds...");
  delay(2000);

  CommsSerial.println("Ramping down...");
  for (int i = max_pct; i >= 0; i--) {
    set_throttle(i, i);
    delay(2000 / max_pct);
  }

  stop();
  CommsSerial.println("Ramp test complete.");
}

void cmd_contra_test() {
  CommsSerial.println("Testing contra...");
  float base = 30.0;
  float maxdiff = 20.0;
  for (int i = 0; i <= 100; i++) {
    set_throttle_roll(base, maxdiff * sin(2 * PI * i / 100.0)); // one full period.
    delay(100);
  }
  stop();
  CommsSerial.println("Contra test complete.");
}

// -----------------------------------------------------------------------------

void begin() {
  esc1.attach(PROP1_PIN);
  esc2.attach(PROP2_PIN);
  set_throttle_micros(MIN_PULSE, MIN_PULSE); // this is pretty much arming assuming no commands are run for a couple seconds

  CommandRouter::add(cmd_arm, "prop_arm");
  CommandRouter::add(cmd_stop, "prop_stop");
  CommandRouter::add(cmd_set_both, "prop_set_both");
  CommandRouter::add(cmd_set, "prop_set");
  CommandRouter::add(cmd_status, "prop_status");
  CommandRouter::add(cmd_ramp_test, "prop_ramp");
  CommandRouter::add(cmd_contra_test, "prop_contra");

  // CommandRouter::mprintln("Initialized on pins ", PROP1_PIN, " and ", PROP2_PIN, ".");
  // CommandRouter::println("Run 'prop_arm' to arm ESCs.");
}

} // namespace Prop
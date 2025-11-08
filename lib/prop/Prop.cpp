#include <Prop.h>
#include <Router.h>

#include <mbed.h>
#include <Arduino.h>

#define PROP1_PIN D4
#define PROP2_PIN D2
#define MIN_PULSE 1000
#define MAX_PULSE 2000
#define ARM_TIME 5000

namespace Prop {

mbed::PwmOut esc1(digitalPinToPinName(PROP1_PIN));
mbed::PwmOut esc2(digitalPinToPinName(PROP2_PIN));

int current_throttle_1 = MIN_PULSE;
int current_throttle_2 = MIN_PULSE;
bool armed = false;

auto parse_doubles = [](const String &str, double *vals, int count) { // sscanf doesnt handle doubles. 5 minutes of debugging resulted in that conclusion.
  size_t pos = 0;
  for (int i = 0; i < count; i++) {
    int next = str.indexOf(' ', pos);
    if (next == -1)
      next = str.length();
    if (pos >= str.length())
      return false;
    vals[i] = str.substring(pos, next).toDouble();
    pos = next + 1;
  }
  return true;
};

void set_throttle_micros(int prop1_us, int prop2_us) {
  prop1_us = constrain(prop1_us, MIN_PULSE, MAX_PULSE);
  prop2_us = constrain(prop2_us, MIN_PULSE, MAX_PULSE);

  current_throttle_1 = prop1_us;
  current_throttle_2 = prop2_us;

  esc1.pulsewidth_us(prop1_us);
  esc2.pulsewidth_us(prop2_us);
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
  esc1.pulsewidth_us(MIN_PULSE);
  esc2.pulsewidth_us(MIN_PULSE);
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
  String argstr = String(args);
  if (!parse_doubles(argstr, vals, 2)) {
    Router::println("Usage: prop_set_both <prop1 %> <prop2 %>");
    return;
  }

  set_throttle(vals[0], vals[1]);
  Router::mprintln("Throttle set to: ", vals[0], "% ", vals[1], "%");
}

void cmd_set(const char *args) {
  double vals[2];
  String argstr = String(args);
  if (!parse_doubles(argstr, vals, 2)) {
    if (!parse_doubles(argstr, vals, 1)) {
      Router::println("Usage: prop_set <throttle %> [<roll %>]");
      return;
    }
    vals[1] = 0.0; // default roll to 0
  }

  set_throttle_roll(vals[0], vals[1]);
  Router::mprintln("Throttle set to: ", vals[0], "% ", " differential ", vals[1], "%");
}

void cmd_stop(const char *) {
  stop();
  Router::println("Stopped.");
}

void cmd_arm(const char *) {
  Router::println("Arming...");
  arm();
  Router::println("Armed.");
}

void cmd_status(const char *) {
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

void cmd_contra_test(const char *) {
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
  esc1.period_ms(20);
  esc2.period_ms(20);
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
#pragma once

namespace Prop {
void begin();

void get_prop_perc(float thrust_N, float roll_accel, float *thrust_perc, float *diffy_perc);

void set_throttle(float motor1_pct, float motor2_pct);
void set_throttle_roll(float overall_pct, float differential);

void stop();
void arm();

int get_throttle_1();
int get_throttle_2();
bool is_armed();
} // namespace Prop
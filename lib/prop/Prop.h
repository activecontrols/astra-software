#pragma once

namespace Prop {
void begin();

// void set_throttle(int motor1_us, int motor2_us);
void set_throttle(float motor1_pct, float motor2_pct);
void stop();
void arm();

int get_throttle_1();
int get_throttle_2();
bool is_armed();
}
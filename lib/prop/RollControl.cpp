#include "RollControl.h"

#define THRUST_MEASUREMENT_QTY 9
#define ROLL_MEASUREMENT_QTY 4

// maps thrust in N to thrust %
float thrust_table[2][THRUST_MEASUREMENT_QTY] = {{0, 1.96029, 3.4305075, 4.900725, 6.3709425, 8.3312325, 9.80145, 13.2319575, 14.7}, // Thrust (N)
                                                 {0, 20, 30, 40, 50, 60, 70, 80, 100}};                                              // Thurst (%)

// contains diffy %
float roll_table_roll_perc[ROLL_MEASUREMENT_QTY] = {-30, -10, 10, 30};

// for each thrust in %, contains rolls in rad/sec^2 in order of roll_table_roll_perc
float roll_table[THRUST_MEASUREMENT_QTY][ROLL_MEASUREMENT_QTY] = {
    {-100, -100, 100, 100}, // 0%
    {-0.4, -0.2, 0.2, 0.4}, // 20%
    {-0.4, -0.2, 0.2, 0.4}, // 30%
    {-0.4, -0.2, 0.2, 0.4}, // 40%
    {-0.4, -0.2, 0.2, 0.4}, // 50%
    {-0.4, -0.2, 0.2, 0.4}, // 60%
    {-0.4, -0.2, 0.2, 0.4}, // 70%
    {-0.4, -0.2, 0.2, 0.4}, // 80%
    {-0.4, -0.2, 0.2, 0.4}  // 90%
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

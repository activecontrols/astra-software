
#include "SDCard.h"
#include "matlab_funcs.h"
#include <Arduino.h>
#include <ArduinoEigenDense.h>

#define CURRENT_TRAJECTORY_VERSION 1
#define MAX_TRAJ_NODES 150

struct TrajectoryNode {
  float Time;
  float X_trg[15];
  float U_ff[4];
};

namespace TrajectoryLoader {
extern TrajectoryNode nodes[MAX_TRAJ_NODES];
extern int NumNodes;
extern bool isLoaded;

void begin();
bool load_trajectory_csv(const char *filename);
void load_trajectory_cmd();
void get_target(bool GND, float t, Vector15 &X_trg_out, Vector4 &U_ff_out);
} // namespace TrajectoryLoader
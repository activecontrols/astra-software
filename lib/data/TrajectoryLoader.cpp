//
// Pablo Plata - 10/13/26

#include "TrajectoryLoader.h"
#include "CommandRouter.h"
#include "SDCard.h"

namespace TrajectoryLoader {

TrajectoryNode nodes[MAX_TRAJ_NODES];
bool isLoaded;
int NumNodes = 0;

void begin() {
  // Do we need some sort of default here or something??
  CommandRouter::add(load_trajectory_cmd, "load_traj", "Load trajectory .csv from SD card");
}

void load_trajectory_cmd() {
  CommsSerial.println("Enter CSV filename");
  char *filename = CommsSerial.readline();
  load_trajectory_csv(filename);
}

bool load_trajectory_csv(const char *filename) {
  File file = SDCard::open(filename, FILE_READ);
  CommsSerial.println("Loading trajectory");
  NumNodes = 0;
  char line[384];
  file.readBytesUntil('\n', line, sizeof(line));
  while (file.available()) {
    int bytes = file.readBytesUntil('\n', line, sizeof(line) - 1);
    if (bytes <= 0)
      continue;
    line[bytes] = '\0';

    if (line[0] == '\r' || line[0] == '\n' || line[0] == '\0')
      continue;

    // Time
    char *token = strtok(line, ",");
    if (!token)
      continue;
    nodes[NumNodes].Time = strtof(token, NULL);

    // Target States
    for (int i = 0; i < 15; i++) {
      token = strtok(NULL, ",");
      nodes[NumNodes].X_trg(i) = token ? strtof(token, NULL) : 0.0f;
    }

    // Feedforward Inputs
    for (int i = 0; i < 4; i++) {
      token = strtok(NULL, ",");
      nodes[NumNodes].U_ff(i) = token ? strtof(token, NULL) : 0.0f;
    }

    NumNodes++;
  }
  file.close();
  if (NumNodes == 0) {
    CommsSerial.println("ERROR, No valid rows found in CSV.");
    isLoaded = false;
    return false;
  }
  isLoaded = true;
  CommsSerial.println("Succesfully Loaded trajectory with duration");
  CommsSerial.println(nodes[NumNodes - 1].Time);
}

void get_target(bool GND, float t, Vector15 &X_trg_out, Vector4 &U_ff_out) {
  if (GND || !isLoaded) {
    X_trg_out.setZero();
    X_trg_out(0) = 1.0f;
    U_ff_out.setZero();
    return;
  }

  // Clamp post-flight
  if (t >= nodes[NumNodes - 1].Time) {
    X_trg_out = nodes[NumNodes - 1].X_trg;
    U_ff_out = nodes[NumNodes - 1].U_ff;
    return;
  }

  // Linear interp
  for (int k = 0; k < NumNodes - 1; k++) {
    if (t >= nodes[k].Time && t <= nodes[k + 1].Time) {
      float dt = nodes[k + 1].Time - nodes[k].Time;
      float alpha = (dt > 1e-5f) ? ((t - nodes[k].Time) / dt) : 0.0f;

      X_trg_out = (1.0f - alpha) * nodes[k].X_trg + alpha * nodes[k + 1].X_trg;
      U_ff_out = (1.0f - alpha) * nodes[k].U_ff + alpha * nodes[k + 1].U_ff;

      // Normalize
      float q_norm = X_trg_out.segment<4>(0).norm();
      if (q_norm > 1e-6f) {
        X_trg_out.segment<4>(0) /= q_norm;
      }
      return;
    }
  }
}

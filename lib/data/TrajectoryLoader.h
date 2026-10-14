/*
 *  TrajectoryLoader.h
 *
 *  Created on: 2024-06-10 by Ishan Goel
 *  Description: This file contains the declaration of the Loader class, which  provides functions to load configurations
 *  and trajectories from either serial communication or an SD card, as well as write trajectories to an SD card.
 */

#pragma once

// TODO: Is this code redundant? Seems like Trajectory.h does all of this already.
#include "SDCard.h"
#include "Trajectory.h"

namespace TrajectoryLoader {
extern TrajectoryNode nodes[MAX_TRAJ_NODES];
extern int NumNodes;
extern bool isLoaded;

void begin();
bool load_trajectory_csv(const char *filename);
void load_trajectory_cmd();
void get_target(bool GND, float t, Vector15 &X_trg_out, Vector4 &U_ff_out);
}; // namespace TrajectoryLoader

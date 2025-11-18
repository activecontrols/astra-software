/*
 *  TrajectoryLoader.h
 *
 *  Created on: 2024-06-10 by Ishan Goel
 *  Description: This file contains the declaration of the Loader class, which  provides functions to load configurations
 *  and trajectories from either serial communication or an SD card, as well as write trajectories to an SD card.
 */

#pragma once

#include "SDCard.h"
#include "Trajectory.h"

namespace TrajectoryLoader {

extern trajectory_header header;
extern traj_point_pos *trajectory;
extern bool loaded_trajectory;

void begin(); // registers loader functions with the router
bool load_trajectory_sd(const char *filename);

// triggered by comms
void load_trajectory_serial(const char *);
void load_trajectory_sd_cmd(const char *);
void write_trajectory_sd(const char *);

void load_trajectory_generic(bool serial, File *f);
}; // namespace TrajectoryLoader

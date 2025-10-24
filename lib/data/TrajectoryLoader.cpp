//
// Created by Ishan Goel on 6/10/24.
//

#include "TrajectoryLoader.h"
#include "Router.h"
#include "SDCard.h"

namespace TrajectoryLoader {

trajectory_header header;
lerp_point_pos *lerp_pos_trajectory;
bool loaded_trajectory;

void begin() {
  Router::add({load_trajectory_serial, "load_trajectory_serial"});
  Router::add({load_trajectory_sd_cmd, "load_trajectory_sd"});
  Router::add({write_trajectory_sd, "write_trajectory_sd"});
}

void load_trajectory_generic(bool serial, File *f) {

  if (loaded_trajectory) {
    free(lerp_pos_trajectory);
    lerp_pos_trajectory = NULL;
  }

  auto receive = [=](char *buf, unsigned int len) {
    if (serial)
      Router::receive(buf, len);
    else
      f->read(buf, len);
  };

  receive((char *)&header, sizeof(header));

  if (header.version != CURRENT_TRAJECTORYH_VERSION) {
    Router::println("ERROR! Attempted to load a trajectory from an older version. Aborting.");
    return;
  }

  // load lerp points in from serial
  lerp_pos_trajectory = (lerp_point_pos *)(calloc(header.num_points, sizeof(lerp_point_pos)));
  receive((char *)lerp_pos_trajectory, sizeof(lerp_point_pos) * header.num_points);
  Router::print("Loaded trajectory with: ");
  Router::print(header.num_points);
  Router::println(" points");

  for (int i = 0; i < header.num_points; i++) {
    Router::print("Point: ");
    Router::print(lerp_pos_trajectory[i].time);
    Router::print(" sec | X ");
    Router::print(lerp_pos_trajectory[i].x);
    Router::print(" meters | Y ");
    Router::print(lerp_pos_trajectory[i].y);
    Router::print(" meters | Z ");
    Router::print(lerp_pos_trajectory[i].z);
    Router::println(" meters.");
  }

  loaded_trajectory = true;
}

void load_trajectory_serial(const char *) {
  Router::println("Preparing to load trajectory!");
  load_trajectory_generic(true, nullptr);
  Router::println("Loaded trajectory!");
}

void load_trajectory_sd_cmd(const char *) {
  // filenames use DOS 8.3 standard
  Router::print("Enter filename: ");
  String filename = Router::read(50);
  File f = SDCard::open(filename.c_str(), FILE_READ);
  if (f) {
    load_trajectory_generic(false, &f);
    f.close();
  } else {
    Router::println("File not found.");
    return;
  }
  Router::println("Loaded trajectory!");
}

bool load_trajectory_sd(const char *filename) {
  File f = SDCard::open(filename, FILE_READ);
  if (f) {
    load_trajectory_generic(false, &f);
    f.close();
  } else {
    Router::println("File not found.");
    return false;
  }
  return loaded_trajectory;
}

void write_trajectory_sd(const char *) {
  // filenames use DOS 8.3 standard
  Router::print("Enter filename: ");
  String filename = Router::read(50);
  File f = SDCard::open(filename.c_str(), FILE_WRITE);
  if (!f) {
    Router::println("File not found.");
    return;
  }
  f.write((char *)&header, sizeof(header));
  f.write((char *)lerp_pos_trajectory, sizeof(lerp_point_pos) * header.num_points);

  f.close();
  Router::println("Wrote trajectory!");
}

}
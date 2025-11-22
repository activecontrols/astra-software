//
// Created by Ishan Goel on 6/10/24.
//

#include "TrajectoryLoader.h"
#include "Router.h"
#include "SDCard.h"

namespace TrajectoryLoader {

trajectory_header header;
traj_point_pos *trajectory;
bool loaded_trajectory;

void begin() {
  Router::add({load_trajectory_serial, "load_trajectory_serial"});
  Router::add({load_trajectory_sd_cmd, "load_trajectory_sd"});
  Router::add({write_trajectory_sd, "write_trajectory_sd"});

  loaded_trajectory = true;
  trajectory = (traj_point_pos *)malloc(sizeof(traj_point_pos) * 2);
  trajectory[0] = {.time = 0, .north = 0, .west = 0, .up = 0};
  trajectory[1] = {.time = 10, .north = 0, .west = 0, .up = 0};
}

void load_trajectory_generic(bool serial, File *f) {

  if (loaded_trajectory) {
    free(trajectory);
    trajectory = NULL;
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
  trajectory = (traj_point_pos *)(calloc(header.num_points, sizeof(traj_point_pos)));
  receive((char *)trajectory, sizeof(traj_point_pos) * header.num_points);
  Router::print("Loaded trajectory with: ");
  Router::print(header.num_points);
  Router::println(" points");

  for (int i = 0; i < header.num_points; i++) {
    Router::print("Point: ");
    Router::print(trajectory[i].time);
    Router::print(" sec | NORTH ");
    Router::print(trajectory[i].north);
    Router::print(" meters | WEST ");
    Router::print(trajectory[i].west);
    Router::print(" meters | UP ");
    Router::print(trajectory[i].up);
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
  f.write((char *)trajectory, sizeof(traj_point_pos) * header.num_points);

  f.close();
  Router::println("Wrote trajectory!");
}

} // namespace TrajectoryLoader
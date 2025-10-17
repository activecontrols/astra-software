//
// Created by Ishan Goel on 6/10/24.
//

#include "Loader.h"
#include "Router.h"
#include "SDCard.h"

trajectory_header Loader::header;
lerp_point_pos *Loader::lerp_pos_curve;
bool Loader::loaded_curve;

void Loader::begin() {
  Router::add({load_curve_serial, "load_curve_serial"});
  Router::add({load_curve_sd_cmd, "load_curve_sd"});
  Router::add({write_curve_sd, "write_curve_sd"});
}

void Loader::load_curve_generic(bool serial, File *f) {

  if (loaded_curve) {
    free(lerp_pos_curve);
    lerp_pos_curve = NULL;
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
  lerp_pos_curve = (lerp_point_pos *)(calloc(header.num_points, sizeof(lerp_point_pos)));
  receive((char *)lerp_pos_curve, sizeof(lerp_point_pos) * header.num_points);
  Router::print("Loaded trajectory with: ");
  Router::print(header.num_points);
  Router::println(" points");

  for (int i = 0; i < header.num_points; i++) {
    Router::print("Point: ");
    Router::print(lerp_pos_curve[i].time);
    Router::print(" sec | X ");
    Router::print(lerp_pos_curve[i].x);
    Router::print(" units | Y ");
    Router::print(lerp_pos_curve[i].y);
    Router::print(" units | Z ");
    Router::print(lerp_pos_curve[i].z);
    Router::println(" units.");
  }

  loaded_curve = true;
}

void Loader::load_curve_serial(const char *) {
  Router::println("Preparing to load trajectory!");
  load_curve_generic(true, nullptr);
  Router::println("Loaded trajectory!");
}

void Loader::load_curve_sd_cmd(const char *) {
  // filenames use DOS 8.3 standard
  Router::print("Enter filename: ");
  String filename = Router::read(50);
  File f = SDCard::open(filename.c_str(), FILE_READ);
  if (f) {
    load_curve_generic(false, &f);
    f.close();
  } else {
    Router::println("File not found.");
    return;
  }
  Router::println("Loaded trajectory!");
}

bool Loader::load_curve_sd(const char *filename) {
  File f = SDCard::open(filename, FILE_READ);
  if (f) {
    load_curve_generic(false, &f);
    f.close();
  } else {
    Router::println("File not found.");
    return false;
  }
  return loaded_curve;
}

void Loader::write_curve_sd(const char *) {
  // filenames use DOS 8.3 standard
  Router::print("Enter filename: ");
  String filename = Router::read(50);
  File f = SDCard::open(filename.c_str(), FILE_WRITE);
  if (!f) {
    Router::println("File not found.");
    return;
  }
  f.write((char *)&header, sizeof(header));
  f.write((char *)lerp_pos_curve, sizeof(lerp_point_pos) * header.num_points);

  f.close();
  Router::println("Wrote trajectory!");
}
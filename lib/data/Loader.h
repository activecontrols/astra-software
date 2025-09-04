
// #pragma once

// #include <SD.h>
// #include <Curve.h>

// class Loader {
// public:
//   static curve_header header;
//   static lerp_point_angle *lerp_angle_curve;
//   static lerp_point_thrust *lerp_thrust_curve;
//   static bool loaded_curve;

//   static void begin(); // registers loader functions with the router
//   Loader() = delete;   // prevent instantiation
//   static bool load_curve_sd(const char *filename);

//   static void save_pt_zero();
//   static void restore_pt_zero();

// private:
//   // triggered by comms
//   static void
//   load_curve_serial();
//   static void load_curve_sd_cmd();
//   static void write_curve_sd();

//   static void load_curve_generic(bool serial, File *f);
// };
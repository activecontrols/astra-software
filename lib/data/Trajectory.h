#define CURRENT_TRAJECTORYH_VERSION 1 // UPDATE THIS BIT IF THE STRUCT IS CHANGED - it will invalidate files created in older version

typedef struct {
  float time;      // seconds since start
  float x; // X coordinate
  float y; // Y coordinate
  float z; // z coordinate
} lerp_point_pos;

typedef struct {
  int version = CURRENT_TRAJECTORYH_VERSION;
  char curve_label[50]; // max 49 char string label
  int num_points;
} trajectory_header;
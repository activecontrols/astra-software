#define CURRENT_TRAJECTORYH_VERSION 1 // UPDATE THIS BIT IF THE STRUCT IS CHANGED - it will invalidate files created in older version

typedef struct {
  float time;  // seconds since start
  float north; // north from origin (m)
  float west;  // west from origin (m)
  float up;    // up from origin (m)
} traj_point_pos;

typedef struct {
  int version = CURRENT_TRAJECTORYH_VERSION;
  char trajectory_label[50]; // max 49 char string label
  int num_points;
} trajectory_header;
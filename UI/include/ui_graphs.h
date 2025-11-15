#ifndef ASTRA_GS_UI_GRAPHS_H
#define ASTRA_GS_UI_GRAPHS_H

typedef struct {
  const char *render_title;
  const char *plot_title;
  const char *y1_label;
  const char *y2_label;
  const char *y3_label;
  double y_max;
  double y_min;
} scrolling_line_chart_arg_t;

typedef struct {
  const char *render_title;
  const char *plot_title;
  const char *x_axis_label;
  const char *y_axis_label;
  ImPlotCond axis_lock;
  double min;
  double max;
} top_down_pos_plot_arg_t;

typedef struct {
  const char *render_title;
  const char *plot_title;
  double alt_max;
  double hor_min;
  double hor_max;
} drop_pos_target_arg_t;

void scrolling_line_chart(scrolling_line_chart_arg_t arg, float history[3][2000], int write_idx);
void rotatable_cube(ImVec4 q);
void top_down_position_plot(top_down_pos_plot_arg_t arg, float x, float y);
void drop_position_target_plot(drop_pos_target_arg_t arg, ImVec4 state, ImVec4 target);

#endif

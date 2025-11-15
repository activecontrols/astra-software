#include "implot.h"
#include "implot3d.h"
#include "ui_components.h"
#include "ui_graphs.h"

void scrolling_line_chart(scrolling_line_chart_arg_t arg, float history[3][2000], int &write_idx, float y1, float y2, float y3) {
  history[0][write_idx] = y1;
  history[1][write_idx] = y2;
  history[2][write_idx] = y3;
  history[0][write_idx + 1000] = y1;
  history[1][write_idx + 1000] = y2;
  history[2][write_idx + 1000] = y3;

  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 175))) { // width = fill, height auto
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1000, ImPlotCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, arg.y_min, arg.y_max);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::PlotLine(arg.y1_label, &history[0][write_idx], 1000);
    ImPlot::PlotLine(arg.y2_label, &history[1][write_idx], 1000);
    ImPlot::PlotLine(arg.y3_label, &history[2][write_idx], 1000);
    ImPlot::EndPlot();
  }

  write_idx--;
  write_idx += 1000;
  write_idx %= 1000;
}

ImVec4 cube_verts[8] = {{-1, -1, -1, 0}, {1, -1, -1, 0}, {1, 1, -1, 0}, {-1, 1, -1, 0}, {-1, -1, 1, 0}, {1, -1, 1, 0}, {1, 1, 1, 0}, {-1, 1, 1, 0}};
int cube_edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

ImVec4 quatRot(ImVec4 q, ImVec4 vtx) {
  ImVec4 out;
  out.x = vtx.x * (1 - 2 * (q.y * q.y + q.z * q.z)) + vtx.y * (2 * (q.x * q.y + q.w * q.z)) + vtx.z * (2 * (q.x * q.z - q.w * q.y));
  out.y = vtx.x * (2 * (q.x * q.y - q.w * q.z)) + vtx.y * (1 - 2 * (q.x * q.x + q.z * q.z)) + vtx.z * (2 * (q.y * q.z + q.w * q.x));
  out.z = vtx.x * (2 * (q.x * q.z + q.w * q.y)) + vtx.y * (2 * (q.y * q.z - q.w * q.x)) + vtx.z * (1 - 2 * (q.x * q.x + q.y * q.y));
  return out;
}

void rotatable_cube(ImVec4 q) {
  // rotate cube vertices
  ImVec4 rot[8];
  for (int i = 0; i < 8; i++) {
    rot[i] = quatRot(q, cube_verts[i]);
  }

  if (ImPlot3D::BeginPlot("##Cube3D")) {
    ImPlot3D::SetupAxesLimits(-2, 2, -2, 2, -2, 2);

    for (int i = 0; i < 12; i++) {
      ImVec4 a = rot[cube_edges[i][0]];
      ImVec4 b = rot[cube_edges[i][1]];
      // Draw line segment
      double xs[2] = {-a.y, -b.y};
      double ys[2] = {-a.x, -b.x};
      double zs[2] = {a.z, b.z};
      ImPlot3D::PlotLine("##edge", xs, ys, zs, 2);
    }

    ImPlot3D::EndPlot();
  }
}

void top_down_position_plot(top_down_pos_plot_arg_t arg, float x, float y) {
  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 250), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes(arg.x_axis_label, arg.y_axis_label);
    ImPlot::SetupAxesLimits(arg.min, arg.max, arg.min, arg.max, arg.axis_lock);

    ImPlot::PlotScatter(arg.point_name, &x, &y, 1);

    ImPlot::EndPlot();
  }
}

void drop_position_target_plot(drop_pos_target_arg_t arg, ImVec4 state, ImVec4 target) {
  centered_text(arg.plot_title);
  if (ImPlot3D::BeginPlot(arg.render_title)) {
    double cs_x[2] = {state.x, state.x};
    double cs_y[2] = {state.y, state.y};
    double cs_z[2] = {0, state.z};

    double target_x[2] = {target.x, target.x};
    double target_y[2] = {target.y, target.y};
    double target_z[2] = {0, target.z};

    ImPlot3D::SetupAxes("West (m)", "North (m)", "Up (m)");
    ImPlot3D::SetupAxisLimits(ImAxis3D_Z, 0, arg.alt_max);
    ImPlot3D::SetupAxisLimits(ImAxis3D_X, arg.hor_min, arg.hor_max);
    ImPlot3D::SetupAxisLimits(ImAxis3D_Y, arg.hor_min, arg.hor_max);

    ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Circle, 5, ImPlot3D::GetColormapColor(0, ImPlot3DColormap_Deep));
    ImPlot3D::PlotScatter("State", cs_x, cs_y, cs_z, 2);

    ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Diamond, 5, ImPlot3D::GetColormapColor(1, ImPlot3DColormap_Deep));
    ImPlot3D::PlotScatter("Target", target_x, target_y, target_z, 2);

    ImPlot3D::SetNextLineStyle(ImPlot3D::GetColormapColor(0, ImPlot3DColormap_Deep), 2);
    ImPlot3D::PlotLine("State", cs_x, cs_y, cs_z, 2);

    ImPlot3D::SetNextLineStyle(ImPlot3D::GetColormapColor(1, ImPlot3DColormap_Deep), 2);
    ImPlot3D::PlotLine("Target", target_x, target_y, target_z, 2);

    ImPlot3D::EndPlot();
  }
}

void top_down_vector_plot(top_down_vector_arg_t arg, float x, float y) {
  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("West (m/s)", "North (m/s)");
    ImPlot::SetupAxesLimits(arg.min, arg.max, arg.min, arg.max);

    double gps_x[2] = {0, x};
    double gps_y[2] = {0, y};
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 4.0f);
    ImPlot::PlotLine(arg.line_name, gps_x, gps_y, 2);
    ImPlot::PopStyleVar();

    ImPlot::EndPlot();
  }
}

void top_down_position_target_plot(top_down_pos_target_arg_t arg, float state_x, float state_y, float target_x, float target_y) {
  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("West (m)", "North (m)");
    ImPlot::SetupAxesLimits(arg.min, arg.max, arg.min, arg.max);
    ImPlot::PlotScatter("State", &state_x, &state_y, 1);
    ImPlot::PlotScatter("Target", &target_x, &target_y, 1);
    ImPlot::EndPlot();
  }
}
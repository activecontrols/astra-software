#include "ui_graphs.h"
#include "implot.h"
#include "implot3d.h"
#include "ui_components.h"

ImVec4 axis_colors[3] = {{1.0, 0.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, {0.0, 0.0, 1.0, 1.0}};

void scrolling_line_chart(scrolling_line_chart_arg_t arg, float y1[FLIGHT_HISTORY_LENGTH], float y2[FLIGHT_HISTORY_LENGTH], float y3[FLIGHT_HISTORY_LENGTH]) {
  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 175))) { // width = fill, height auto
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1000, ImPlotCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, arg.y_min, arg.y_max);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);

    ImPlotSpec spec;
    spec.LineColor = axis_colors[0];
    ImPlot::PlotLine(arg.y1_label, &y1[FlightHistory.read_start_pos], FLIGHT_HISTORY_LENGTH, 1, 0, spec);
    spec.LineColor = axis_colors[1];
    ImPlot::PlotLine(arg.y2_label, &y2[FlightHistory.read_start_pos], FLIGHT_HISTORY_LENGTH, 1, 0, spec);
    spec.LineColor = axis_colors[2];
    ImPlot::PlotLine(arg.y3_label, &y3[FlightHistory.read_start_pos], FLIGHT_HISTORY_LENGTH, 1, 0, spec);
    ImPlot::EndPlot();
  }
}

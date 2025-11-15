#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "implot3d.h"
#include "math.h"
#include <string>

#include <iostream>

#include "serial.h"

ImFont *panel_header_font;
ImFont *large_font;

#define DO_COUNT

bool ui_do_state[DO_COUNT] = {false, false, false, false, false};

// Helper to adjust brightness
ImU32 AdjustBrightness(ImU32 color, float factor) {
  ImVec4 c = ImGui::ColorConvertU32ToFloat4(color);
  c.x = ImClamp(c.x * factor, 0.0f, 1.0f);
  c.y = ImClamp(c.y * factor, 0.0f, 1.0f);
  c.z = ImClamp(c.z * factor, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(c);
}

bool daq_button(const char *label, const ImVec2 &size, ImU32 color, float rounding = 10.0f) {
  ImGui::PushStyleColor(ImGuiCol_Button, color);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, AdjustBrightness(color, 1.2));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, AdjustBrightness(color, 0.7));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, rounding);

  bool clicked = ImGui::Button(label, size);

  ImGui::PopStyleVar();
  ImGui::PopStyleColor(3);

  return clicked;
}

void toggle_button(const char *label, const ImVec2 &size, ImU32 active_color, ImU32 deactive_color, bool *control_var, float rounding = 10.0f) {
  ImU32 color;
  ImU32 clicked_color;
  if (*control_var) {
    color = active_color;
    clicked_color = deactive_color;
  } else {
    color = deactive_color;
    clicked_color = active_color;
  }

  ImGui::PushStyleColor(ImGuiCol_Button, color);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, AdjustBrightness(color, 1.2));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, AdjustBrightness(color, 1.2)); // stops flickering
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, rounding);

  bool clicked = ImGui::Button(label, size);

  ImGui::PopStyleVar();
  ImGui::PopStyleColor(3);

  if (clicked) {
    std::cout << "toggle" << std::endl;
    *control_var = !(*control_var);
  }
}

void centered_text(const char *text) {
  ImGuiStyle &style = ImGui::GetStyle();

  float size = ImGui::CalcTextSize(text).x + style.FramePadding.x * 2.0f;
  float avail = ImGui::GetContentRegionAvail().x;

  float off = (avail - size) * 0.5;
  if (off > 0.0f)
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + off);
  ImGui::PushFont(large_font);
  ImGui::Text(text);
  ImGui::PopFont();
}

typedef struct {
  const char *render_title;
  const char *plot_title;
  const char *y1_label;
  const char *y2_label;
  const char *y3_label;
  double y_max;
  double y_min;
} scrolling_line_chart_arg_t;

void scrolling_line_chart(scrolling_line_chart_arg_t arg, float history[3][2000], int write_idx) {
  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 200))) { // width = fill, height auto
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1000, ImPlotCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, arg.y_min, arg.y_max);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::PlotLine(arg.y1_label, &history[0][write_idx], 1000);
    ImPlot::PlotLine(arg.y2_label, &history[1][write_idx], 1000);
    ImPlot::PlotLine(arg.y3_label, &history[2][write_idx], 1000);
    ImPlot::EndPlot();
  }
}

float acc_history[3][2000];
float gyro_history[3][2000];
int write_idx = 999;

void live_sensor_panel() {
  acc_history[0][write_idx] = state_packet.accel_x;
  acc_history[1][write_idx] = state_packet.accel_y;
  acc_history[2][write_idx] = state_packet.accel_z;
  acc_history[0][write_idx + 1000] = state_packet.accel_x;
  acc_history[1][write_idx + 1000] = state_packet.accel_y;
  acc_history[2][write_idx + 1000] = state_packet.accel_z;

  gyro_history[0][write_idx] = state_packet.gyro_yaw;
  gyro_history[1][write_idx] = state_packet.gyro_pitch;
  gyro_history[2][write_idx] = state_packet.gyro_roll;
  gyro_history[0][write_idx + 1000] = state_packet.gyro_yaw;
  gyro_history[1][write_idx + 1000] = state_packet.gyro_pitch;
  gyro_history[2][write_idx + 1000] = state_packet.gyro_roll;

  scrolling_line_chart_arg_t imu_acc;
  imu_acc.plot_title = "IMU Accel";
  imu_acc.render_title = "##IMU Accel";
  imu_acc.y1_label = "x";
  imu_acc.y2_label = "y";
  imu_acc.y3_label = "z";
  imu_acc.y_max = 15;
  imu_acc.y_min = -5;
  scrolling_line_chart(imu_acc, acc_history, write_idx);

  scrolling_line_chart_arg_t imu_gyro;
  imu_gyro.plot_title = "IMU Gyro";
  imu_gyro.render_title = "##IMU Gyro";
  imu_gyro.y1_label = "yaw";
  imu_gyro.y2_label = "pitch";
  imu_gyro.y3_label = "roll";
  imu_gyro.y_max = 0.5;
  imu_gyro.y_min = -0.5;
  scrolling_line_chart(imu_gyro, gyro_history, write_idx);

  if (ImGui::BeginTable("live_sensor_table", 3, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);

    centered_text("GPS Position");
    if (ImPlot::BeginPlot("##GPS Position", ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
      ImPlot::SetupAxes("West (m)", "North (m)");
      ImPlot::SetupAxesLimits(-5, 5, -5, 5);

      // Draw single point
      float gps_x[1] = {state_packet.gps_pos_west};
      float gps_y[1] = {state_packet.gps_pos_north};
      float target_x[1] = {state_packet.target_pos_west};
      float target_y[1] = {state_packet.target_pos_north};
      ImPlot::PlotScatter("GPS", gps_x, gps_y, 1);
      ImPlot::PlotScatter("Target", target_x, target_y, 1);

      ImPlot::EndPlot();
    }

    ImGui::TableSetColumnIndex(1);

    centered_text("GPS Velocity");
    if (ImPlot::BeginPlot("##GPS Velocity", ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
      ImPlot::SetupAxes("West (m/s)", "North (m/s)");
      ImPlot::SetupAxesLimits(-5, 5, -5, 5);

      // Draw single point
      double gps_x[2] = {0, state_packet.gps_vel_west};
      double gps_y[2] = {0, state_packet.gps_vel_north};
      ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 4.0f);
      ImPlot::PlotLine("##GPS Velocity", gps_x, gps_y, 2);
      ImPlot::PopStyleVar();

      ImPlot::EndPlot();
    }

    ImGui::TableSetColumnIndex(2);

    centered_text("Altitude");
    ImGui::Text("     GPS: %5.2f m", state_packet.gps_pos_up);
    ImGui::Text("  Target: %5.2f m", state_packet.target_pos_up);
    ImGui::Dummy(ImVec2(0, 50)); // Add vertical spacing

    centered_text("Vert Velocity");
    ImGui::Text("     GPS: %5.2f m/s", state_packet.gps_vel_up);

    ImGui::EndTable();
  }
  write_idx--;
  write_idx += 1000;
  write_idx %= 1000;
}

void serial_control_panel() {
  static char inputBuffer[128] = ""; // buffer for text input
  ImGui::InputTextWithHint("##serial_input", "enter serial command", inputBuffer, IM_ARRAYSIZE(inputBuffer));
  ImGui::SameLine();
  if (daq_button("Send", ImVec2(-1, 0), IM_COL32(33, 112, 69, 255))) {
    // Do something with inputBuffer
    printf("You entered: %s\n", inputBuffer);
    inputBuffer[0] = '\0';
  }

  static char outputBuffer[128] = ">help\nping\npong"; // buffer for text input
  ImGui::InputTextMultiline("##serial_output", outputBuffer, IM_ARRAYSIZE(outputBuffer));
}

void controller_state_panel() {
  if (ImGui::BeginTable("control_state_table", 3, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);

    centered_text("CS 1");
    if (ImPlot3D::BeginPlot("##CS1")) {
      float x = 10;
      float y = 11;
      float z = 12;

      ImPlot3D::PlotScatter("##point", &x, &y, &z, 1);
      ImPlot3D::EndPlot();
    }

    ImGui::TableSetColumnIndex(1);

    centered_text("CS 2");
    if (ImPlot3D::BeginPlot("##CS2")) {
      float x = 10;
      float y = 11;
      float z = 12;

      ImPlot3D::PlotScatter("##point", &x, &y, &z, 1);
      ImPlot3D::EndPlot();
    }

    ImGui::TableSetColumnIndex(2);

    centered_text("CS 3");
    if (ImPlot3D::BeginPlot("##CS3")) {
      float x = 10;
      float y = 11;
      float z = 12;

      ImPlot3D::PlotScatter("##point", &x, &y, &z, 1);
      ImPlot3D::EndPlot();
    }

    ImGui::EndTable();
  }
}

void controls_output_panel() {
  if (ImGui::BeginTable("controls_table", 2, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    ImGui::Text("Target Thrust: %5.2f N", state_packet.thrust_N);
    ImGui::Text("Target Roll: %5.2f rad/s^2", state_packet.roll_N);

    ImGui::TableSetColumnIndex(1);

    centered_text("Gimbal Position");
    if (ImPlot::BeginPlot("##Gimbal State", ImVec2(-1, 300), ImPlotFlags_NoLegend)) {
      ImPlot::SetupAxes("Yaw (deg)", "Pitch (deg)");
      ImPlot::SetupAxesLimits(-15, 15, -15, 15);

      // Draw single point
      float xs[1] = {state_packet.gimbal_yaw_deg * 180 / 3.1415};
      float ys[1] = {state_packet.gimbal_pitch_deg * 180 / 3.1415};
      ImPlot::PlotScatter("Gimbal", xs, ys, 1);

      ImPlot::EndPlot();
    }

    ImGui::EndTable();
  }
}

void render_loop() {
  ImGuiIO &io = ImGui::GetIO();
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(io.DisplaySize);

  ImGui::Begin("MainWindow", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus);

  if (ImGui::BeginTable("main_split", 2, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextColumn(); // LEFT

    ImGui::BeginChild("live_sensor_subpanel", ImVec2(0, 800), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Live Sensor Data");
    ImGui::PopFont();
    live_sensor_panel();
    ImGui::EndChild();

    ImGui::BeginChild("serial_control_subpanel", ImVec2(0, 0), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Serial Monitor");
    ImGui::PopFont();
    serial_control_panel();
    ImGui::EndChild();

    ImGui::TableNextColumn(); // RIGHT

    ImGui::BeginChild("controller_state_subpanel", ImVec2(0, 500), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Controller State");
    ImGui::PopFont();
    controller_state_panel();
    ImGui::EndChild();

    ImGui::BeginChild("controller_output_subpanel", ImVec2(0, 0), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Controller Output");
    ImGui::PopFont();
    controls_output_panel();
    ImGui::EndChild();

    ImGui::EndTable();
  }

  ImGui::End();
}
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
} scrolling_line_chart_arg_t;

void scrolling_line_chart(scrolling_line_chart_arg_t arg, float history[3][2000], int write_idx) {
  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 200))) { // width = fill, height auto
    ImPlot::PlotLine("x", &history[0][write_idx], 1000);
    ImPlot::PlotLine("y", &history[1][write_idx], 1000);
    ImPlot::PlotLine("z", &history[2][write_idx], 1000);
    ImPlot::EndPlot();
  }
}

float acc_history[3][2000];
int write_idx = 999;
int acc_counter = 0;

void live_sensor_panel() {
  acc_history[0][write_idx] = state_packet.acc_x;
  acc_history[1][write_idx] = state_packet.acc_y;
  acc_history[2][write_idx] = state_packet.acc_z;
  acc_history[0][write_idx + 1000] = state_packet.acc_x;
  acc_history[1][write_idx + 1000] = state_packet.acc_y;
  acc_history[2][write_idx + 1000] = state_packet.acc_z;
  acc_counter++;

  scrolling_line_chart_arg_t imu_acc;
  imu_acc.plot_title = "IMU Accel";
  imu_acc.render_title = "##IMU Accel";
  imu_acc.y1_label = "x";
  imu_acc.y2_label = "y";
  imu_acc.y3_label = "z";
  scrolling_line_chart(imu_acc, acc_history, write_idx);

  write_idx--;
  write_idx += 1000;
  write_idx %= 1000;

  scrolling_line_chart_arg_t imu_gyro;
  imu_gyro.plot_title = "IMU Gyro";
  imu_gyro.render_title = "##IMU Gyro";
  imu_gyro.y1_label = "x";
  imu_gyro.y2_label = "y";
  imu_gyro.y3_label = "z";
  scrolling_line_chart(imu_gyro, acc_history, write_idx);

  if (ImGui::BeginTable("live_sensor_table", 3, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);

    centered_text("Mag Vec");
    if (ImPlot3D::BeginPlot("##MAG Vec")) {
      float x = 10;
      float y = 11;
      float z = 12;
      ImPlot3D::PlotScatter("##point", &x, &y, &z, 1);
      ImPlot3D::EndPlot();
    }

    ImGui::TableSetColumnIndex(1);

    centered_text("GPS Pos");
    if (ImPlot3D::BeginPlot("##GPS Pos")) {
      float x = 10;
      float y = 11;
      float z = 12;
      ImPlot3D::PlotScatter("##point", &x, &y, &z, 1);
      ImPlot3D::EndPlot();
    }

    ImGui::TableSetColumnIndex(2);

    centered_text("GPS Vel");
    if (ImPlot3D::BeginPlot("##GPS Vel")) {
      float x = 10;
      float y = 11;
      float z = 12;
      ImPlot3D::PlotScatter("##point", &x, &y, &z, 1);
      ImPlot3D::EndPlot();
    }

    ImGui::EndTable();
  }
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
    ImGui::Text("Target Thrust: 0.00 lbf");
    ImGui::Text("Target Roll: 0.00 deg/s^2");

    ImGui::TableSetColumnIndex(1);

    float gx = -5; // e.g., -15 to +15
    float gy = 5;  // e.g., -15 to +15

    centered_text("Gimbal Position");
    if (ImPlot::BeginPlot("##Gimbal State", ImVec2(-1, 300), ImPlotFlags_NoLegend)) {
      ImPlot::SetupAxes("Yaw (deg)", "Pitch (deg)"); //, ImPlotAxisFlags_NoGridLines, ImPlotAxisFlags_NoGridLines);
      ImPlot::SetupAxesLimits(-15, 15, -15, 15, ImPlotCond_Always);

      // Draw single point
      float xs[1] = {gx};
      float ys[1] = {gy};
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
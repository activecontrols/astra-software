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

void toggle_button(const char *labelActive, const ImVec2 &size, ImU32 active_color, ImU32 deactive_color, bool *control_var, float rounding = 10.0f) {
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

  bool clicked = ImGui::SmallButton(labelActive);

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
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 175))) { // width = fill, height auto
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
float mag_history[3][2000];
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

  mag_history[0][write_idx] = state_packet.mag_x;
  mag_history[1][write_idx] = state_packet.mag_y;
  mag_history[2][write_idx] = state_packet.mag_z;
  mag_history[0][write_idx + 1000] = state_packet.mag_x;
  mag_history[1][write_idx + 1000] = state_packet.mag_y;
  mag_history[2][write_idx + 1000] = state_packet.mag_z;

  scrolling_line_chart_arg_t imu_acc;
  imu_acc.plot_title = "IMU Accel";
  imu_acc.render_title = "##IMU Accel";
  imu_acc.y1_label = "x";
  imu_acc.y2_label = "y";
  imu_acc.y3_label = "z";
  imu_acc.y_max = 15;
  imu_acc.y_min = -5;

  scrolling_line_chart_arg_t imu_gyro;
  imu_gyro.plot_title = "IMU Gyro";
  imu_gyro.render_title = "##IMU Gyro";
  imu_gyro.y1_label = "yaw";
  imu_gyro.y2_label = "pitch";
  imu_gyro.y3_label = "roll";
  imu_gyro.y_max = 0.5;
  imu_gyro.y_min = -0.5;

  scrolling_line_chart_arg_t mag;
  mag.plot_title = "Mag";
  mag.render_title = "##Mag";
  mag.y1_label = "x";
  mag.y2_label = "y";
  mag.y3_label = "z";
  mag.y_max = 1.5;
  mag.y_min = -1.5;

  if (ImGui::BeginTable("live_sensor_table", 2, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    scrolling_line_chart(imu_acc, acc_history, write_idx);

    ImGui::TableSetColumnIndex(1);
    scrolling_line_chart(imu_gyro, gyro_history, write_idx);

    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    scrolling_line_chart(mag, mag_history, write_idx);

    ImGui::TableSetColumnIndex(1);

    ImGui::EndTable();
  }

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
  if (daq_button("Send", ImVec2(175, 0), IM_COL32(33, 112, 69, 255))) {
    // Do something with inputBuffer
    write_serial(inputBuffer);
    printf("You entered: %s\n", inputBuffer);
    inputBuffer[0] = '\0';
  }

  ImGui::InputTextMultiline("##serial_output", concat_msg_buf, IM_ARRAYSIZE(concat_msg_buf), ImVec2(800, 100));
}

ImVec4 verts[8] = {
    {-1, -1, -1, 0}, {1, -1, -1, 0}, {1, 1, -1, 0}, {-1, 1, -1, 0}, // bottom
    {-1, -1, 1, 0},  {1, -1, 1, 0},  {1, 1, 1, 0},  {-1, 1, 1, 0}   // top
};
int edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom square
    {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top square
    {0, 4}, {1, 5}, {2, 6}, {3, 7}  // verticals
};

ImVec4 quatRot(ImVec4 q, ImVec4 vtx) {
  ImVec4 out;
  out.x = vtx.x * (1 - 2 * (q.y * q.y + q.z * q.z)) + vtx.y * (2 * (q.x * q.y + q.w * q.z)) + vtx.z * (2 * (q.x * q.z - q.w * q.y));
  out.y = vtx.x * (2 * (q.x * q.y - q.w * q.z)) + vtx.y * (1 - 2 * (q.x * q.x + q.z * q.z)) + vtx.z * (2 * (q.y * q.z + q.w * q.x));
  out.z = vtx.x * (2 * (q.x * q.z + q.w * q.y)) + vtx.y * (2 * (q.y * q.z - q.w * q.x)) + vtx.z * (1 - 2 * (q.x * q.x + q.y * q.y));
  return out;
}

void DrawCube3D(ImVec4 q) {
  // rotate cube vertices
  ImVec4 rot[8];
  for (int i = 0; i < 8; i++) {
    rot[i] = quatRot(q, verts[i]);
  }

  if (ImPlot3D::BeginPlot("##Cube3D")) {
    ImPlot3D::SetupAxesLimits(-2, 2, -2, 2, -2, 2);

    for (int i = 0; i < 12; i++) {
      ImVec4 a = rot[edges[i][0]];
      ImVec4 b = rot[edges[i][1]];
      // Draw line segment
      double xs[2] = {-a.y, -b.y};
      double ys[2] = {-a.x, -b.x};
      double zs[2] = {a.z, b.z};
      ImPlot3D::PlotLine("##edge", xs, ys, zs, 2);
    }

    ImPlot3D::EndPlot();
  }
}

void controller_state_panel() {
  if (ImGui::BeginTable("control_state_table", 2, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);

    centered_text("Estimated Pos");
    if (ImPlot3D::BeginPlot("##CS Pos")) {
      double cs_x[2] = {state_packet.state_pos_west, state_packet.state_pos_west};
      double cs_y[2] = {state_packet.state_pos_north, state_packet.state_pos_north};
      double cs_z[2] = {0, state_packet.state_pos_up};

      double target_x[2] = {state_packet.target_pos_west, state_packet.target_pos_west};
      double target_y[2] = {state_packet.target_pos_north, state_packet.target_pos_north};
      double target_z[2] = {0, state_packet.target_pos_up};

      ImPlot3D::SetupAxes("West (m)", "North (m)", "Up (m)");
      ImPlot3D::SetupAxisLimits(ImAxis3D_Z, 0, 5);
      ImPlot3D::SetupAxisLimits(ImAxis3D_X, -5, 5);
      ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -5, 5);

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

    ImGui::TableSetColumnIndex(1);

    centered_text("Orientation");
    ImVec4 q;
    q.x = state_packet.state_q_vec_0;
    q.y = state_packet.state_q_vec_1;
    q.z = state_packet.state_q_vec_2;
    q.w = sqrt(1 - q.x * q.x - q.y * q.y - q.z * q.z);
    DrawCube3D(q);

    ImGui::EndTable();
  }
}

void controls_output_panel() {
  if (ImGui::BeginTable("controls_table", 2, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    ImGui::Text("  Target Thrust: %5.2f N", state_packet.thrust_N);
    ImGui::Text("    Target Roll: %5.2f rad/s^2", state_packet.roll_N);

    ImGui::TableSetColumnIndex(1);

    centered_text("Gimbal Position");
    if (ImPlot::BeginPlot("##Gimbal State", ImVec2(-1, 250), ImPlotFlags_NoLegend)) {
      ImPlot::SetupAxes("Yaw (deg)", "Pitch (deg)");
      ImPlot::SetupAxesLimits(-15, 15, -15, 15, ImPlotCond_Always);

      // Draw single point
      float xs[1] = {state_packet.gimbal_yaw_deg * 180 / 3.1415};
      float ys[1] = {state_packet.gimbal_pitch_deg * 180 / 3.1415};
      ImPlot::PlotScatter("Gimbal", xs, ys, 1);

      ImPlot::EndPlot();
    }

    ImGui::EndTable();
  }
}

void system_state_panel() {
  ImGui::Text("Elasped Time: %5.2f s", state_packet.elapsed_time);

  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.0f);
  ImVec4 on_gnd = ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f);
  ImVec4 in_air = ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f);
  if (state_packet.GND_flag > 0) {
    ImGui::PushStyleColor(ImGuiCol_FrameBg, on_gnd);
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, on_gnd);
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, on_gnd);
  } else {
    ImGui::PushStyleColor(ImGuiCol_FrameBg, in_air);
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, in_air);
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, in_air);
  }

  ImGui::SetNextItemWidth(200.0f); // pixels
  ImGui::BeginDisabled();          // prevent editing
  ImGui::InputText("##dummy", (char *)"    GND Flag", ImGuiInputTextFlags_ReadOnly);
  ImGui::EndDisabled();
  ImGui::PopStyleColor(3);
  ImGui::PopStyleVar();
}

bool thrusterBool = false;

void ground_control_panel() {
  if (ImGui::BeginTable("controls_table", 4, ImGuiTableFlags_Resizable)) {
    ImU32 activeCol = ImGui::ColorConvertFloat4ToU32(ImVec4(8.0 / 255.0, 255.0 / 255.0, 156.0 / 255.0, 1.0f));
    ImU32 deactiveCol = ImGui::ColorConvertFloat4ToU32(ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f));

    ImGui::TableNextColumn();
    ImGui::BeginChild("COPV_Manifold_subpanel", ImVec2(0, 0), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("COPV Manifold");
    toggle_button("Open Valve 1 ", ImVec2(175, 0), activeCol, deactiveCol, &thrusterBool);
    ImGui::PopFont();

    ImGui::EndChild();

    ImGui::BeginChild("low_pressure_subpanel", ImVec2(0, 300), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Low Pressure Circuit");
    ImGui::PopFont();
    ImGui::EndChild();

    ImGui::TableNextColumn();

    ImGui::BeginChild("tank_rcs_subpanel", ImVec2(0, 300), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Tank set & RCS Circuit");
    ImGui::PopFont();
    ImGui::EndChild();

    ImGui::BeginChild("purge_circuit_subpanel", ImVec2(0, 300), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Purge Circuit");
    ImGui::PopFont();
    ImGui::EndChild();

    ImGui::TableNextColumn();
    ImGui::BeginChild("liquid_oxygen_subpanel", ImVec2(0, 0), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Liquid Oxygen Tank");
    ImGui::PopFont();
    ImGui::EndChild();

    ImGui::TableNextColumn();
    ImGui::BeginChild("isopropyl_alc_subpanel", ImVec2(0, 0), true);
    ImGui::PushFont(panel_header_font);
    ImGui::SeparatorText("Isopropyl Alcohol Tank");
    ImGui::PopFont();
    ImGui::EndChild();
    ImGui::EndTable();
  }
}

int page = 1;

void render_loop() {
  ImGuiIO &io = ImGui::GetIO();
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(io.DisplaySize);

  if (ImGui::Begin("MainWindow", nullptr,
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus |
                       ImGuiWindowFlags_NoNavFocus)) {
    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Windows")) {
        if (ImGui::MenuItem("Sensor")) {
          page = 1;
        }
        if (ImGui::MenuItem("Live Doc")) {
          page = 2;
        }
        if (ImGui::MenuItem("Controls")) {
          page = 3;
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }
    if (page == 1) {
      centered_text("Sensors");
      if (ImGui::BeginTable("main_split", 2, ImGuiTableFlags_Resizable)) {
        ImGui::TableNextColumn(); // LEFT

        ImGui::BeginChild("live_sensor_subpanel", ImVec2(0, 750), true);
        ImGui::PushFont(panel_header_font);
        ImGui::SeparatorText("Live Sensor Data");
        ImGui::PopFont();
        live_sensor_panel();
        ImGui::EndChild();

        ImGui::BeginChild("system_state_subpanel", ImVec2(0, 0), true);
        ImGui::PushFont(panel_header_font);
        ImGui::SeparatorText("System State");
        ImGui::PopFont();
        system_state_panel();
        ImGui::EndChild();

        ImGui::TableNextColumn(); // RIGHT

        ImGui::BeginChild("controller_state_subpanel", ImVec2(0, 500), true);
        ImGui::PushFont(panel_header_font);
        ImGui::SeparatorText("Controller State");
        ImGui::PopFont();
        controller_state_panel();
        ImGui::EndChild();

        ImGui::BeginChild("controller_output_subpanel", ImVec2(0, 350), true);
        ImGui::PushFont(panel_header_font);
        ImGui::SeparatorText("Controller Output");
        ImGui::PopFont();
        controls_output_panel();
        ImGui::EndChild();

        ImGui::EndTable();
      }
    }
    if (page == 2) {
      if (ImGui::BeginTable("main_split", 2, ImGuiTableFlags_Resizable)) {
        ImGui::TableNextColumn();

        ImGui::EndTable();
      }
    }
    if (page == 3) {
      if (ImGui::BeginTable("main_split", 1)) {
        ImGui::TableNextColumn();

        ImGui::BeginChild("ground_control_subpanel", ImVec2(0, 0), true);
        ImGui::PushFont(panel_header_font);
        ImGui::SeparatorText("Control");
        ImGui::PopFont();
        ground_control_panel();
        ImGui::EndChild();

        ImGui::BeginChild("serial_control_subpanel", ImVec2(0, 200), true);
        ImGui::PushFont(panel_header_font);
        ImGui::SeparatorText("Serial Monitor");
        ImGui::PopFont();
        serial_control_panel();
        ImGui::EndChild();
        ImGui::EndTable();
      }
    }

    ImGui::End();
  }
}

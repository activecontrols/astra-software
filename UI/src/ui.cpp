#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "implot3d.h"
#include "math.h"
#include <string>

#include <iostream>

#include "ui_components.h"
#include "ui_graphs.h"
#include "serial.h"

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
  if (rounded_button("Send", ImVec2(175, 0), IM_COL32(33, 112, 69, 255))) {
    // Do something with inputBuffer
    write_serial(inputBuffer);
    printf("You entered: %s\n", inputBuffer);
    inputBuffer[0] = '\0';
  }

  ImGui::InputTextMultiline("##serial_output", concat_msg_buf, IM_ARRAYSIZE(concat_msg_buf), ImVec2(800, 100));
}

void controller_state_panel() {
  if (ImGui::BeginTable("control_state_table", 2, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);

    drop_pos_target_arg_t estimated_pos_plot;
    estimated_pos_plot.plot_title = "Estimated Pos";
    estimated_pos_plot.render_title = "##Estimated Pos";
    estimated_pos_plot.alt_max = 5;
    estimated_pos_plot.hor_min = -5;
    estimated_pos_plot.hor_max = 5;

    ImVec4 state;
    state.x = state_packet.state_pos_west;
    state.y = state_packet.state_pos_north;
    state.z = state_packet.state_pos_up;
    ImVec4 target;
    target.x = state_packet.target_pos_west;
    target.y = state_packet.target_pos_north;
    target.z = state_packet.target_pos_up;
    drop_position_target_plot(estimated_pos_plot, state, target);

    ImGui::TableSetColumnIndex(1);

    centered_text("Orientation");
    ImVec4 q;
    q.x = state_packet.state_q_vec_0;
    q.y = state_packet.state_q_vec_1;
    q.z = state_packet.state_q_vec_2;
    q.w = sqrt(1 - q.x * q.x - q.y * q.y - q.z * q.z);
    rotatable_cube(q);

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

    top_down_pos_plot_arg_t gimbal_pos_plot;
    gimbal_pos_plot.plot_title = "Gimbal Position";
    gimbal_pos_plot.render_title = "##Gimbal Position";
    gimbal_pos_plot.x_axis_label = "Yaw (deg)";
    gimbal_pos_plot.y_axis_label = "Pitch (deg)";
    gimbal_pos_plot.axis_lock = ImPlotCond_Always;
    gimbal_pos_plot.min = -15;
    gimbal_pos_plot.max = 15;
    top_down_position_plot(gimbal_pos_plot, state_packet.gimbal_yaw_deg * 180 / 3.1415, state_packet.gimbal_pitch_deg * 180 / 3.1415);

    ImGui::EndTable();
  }
}

void system_state_panel() {
  ImGui::Text("Elasped Time: %5.2f s", state_packet.elapsed_time);
  colored_flag("    GND Flag", state_packet.GND_flag, ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f), ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f));
}

void render_loop() {
  ImGuiIO &io = ImGui::GetIO();
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(io.DisplaySize);

  ImGui::Begin("MainWindow", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus);

  if (ImGui::BeginTable("main_split", 2, ImGuiTableFlags_Resizable)) {
    ImGui::TableNextColumn();
    panel("Live Sensor Data", ImVec2(0, 750), live_sensor_panel);
    panel("Serial Monitor", ImVec2(0, 0), serial_control_panel);
    ImGui::TableNextColumn();
    panel("Controller State", ImVec2(0, 500), controller_state_panel);
    panel("Controller Output", ImVec2(0, 350), controls_output_panel);
    panel("System State", ImVec2(0, 0), system_state_panel);
    ImGui::EndTable();
  }

  ImGui::End();
}
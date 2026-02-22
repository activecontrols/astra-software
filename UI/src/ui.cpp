#include "ui.h"
#include "flight_data.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "implot3d.h"
#include "ui_components.h"
#include "ui_graphs.h"

void imu_accel_panel() {
  ImGui::Begin(IMU_ACCEL_PANEL);

  scrolling_line_chart_arg_t imu_acc;
  imu_acc.plot_title = "IMU Accel";
  imu_acc.render_title = "##IMU Accel";
  imu_acc.y1_label = "x";
  imu_acc.y2_label = "y";
  imu_acc.y3_label = "z";
  imu_acc.y_max = 15;
  imu_acc.y_min = -5;

  scrolling_line_chart(imu_acc, FlightHistory.accel_x, FlightHistory.accel_y, FlightHistory.accel_z);

  ImGui::End();
}

void imu_gyro_panel() {
  ImGui::Begin(IMU_GYRO_PANEL);

  scrolling_line_chart_arg_t imu_gyro;
  imu_gyro.plot_title = "IMU Gyro";
  imu_gyro.render_title = "##IMU Gyro";
  imu_gyro.y1_label = "yaw";
  imu_gyro.y2_label = "pitch";
  imu_gyro.y3_label = "roll";
  imu_gyro.y_max = 0.5;
  imu_gyro.y_min = -0.5;

  scrolling_line_chart(imu_gyro, FlightHistory.gyro_yaw, FlightHistory.gyro_pitch, FlightHistory.gyro_roll);

  ImGui::End();
}

void mag_panel() {
  ImGui::Begin(MAG_PANEL);

  scrolling_line_chart_arg_t mag;
  mag.plot_title = "Mag";
  mag.render_title = "##Mag";
  mag.y1_label = "x";
  mag.y2_label = "y";
  mag.y3_label = "z";
  mag.y_max = 1.5;
  mag.y_min = -1.5;
  scrolling_line_chart(mag, FlightHistory.mag_x, FlightHistory.mag_y, FlightHistory.mag_z);

  ImGui::End();
}

void gps_pos_panel() {
  ImGui::Begin(GPS_POS_PANEL);

  float state_x = -FlightHistory.gps_pos_west;
  float state_y = FlightHistory.gps_pos_north;
  float target_x = -FlightHistory.target_pos_west;
  float target_y = FlightHistory.target_pos_north;

  centered_text("GPS Position");
  if (ImPlot::BeginPlot("##GPS Position", ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("East (m)", "North (m)");
    ImPlot::SetupAxesLimits(-5, 5, -5, 5);
    ImPlot::PlotScatter("State", &state_x, &state_y, 1);
    ImPlot::PlotScatter("Target", &target_x, &target_y, 1);
    ImPlot::EndPlot();
  }

  ImGui::End();
}

void gps_vel_panel() {
  ImGui::Begin(GPS_VEL_PANEL);

  centered_text("GPS Velocity");
  if (ImPlot::BeginPlot("##GPS Velocity", ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("East (m/s)", "North (m/s)");
    ImPlot::SetupAxesLimits(-5, 5, -5, 5);

    double gps_x[2] = {0, -FlightHistory.gps_vel_west};
    double gps_y[2] = {0, FlightHistory.gps_vel_north};
    ImPlotSpec spec;
    spec.LineWeight = 4;
    ImPlot::PlotLine("##GPS Velocity", gps_x, gps_y, 2, spec);

    ImPlot::EndPlot();
  }

  ImGui::End();
}

void gps_vert_panel() {
  ImGui::Begin(GPS_VERT_PANEL);

  centered_text("Altitude");
  ImGui::Text("     GPS: %5.2f m", FlightHistory.gps_pos_up);
  ImGui::Text("  Target: %5.2f m", FlightHistory.target_pos_up);
  ImGui::Dummy(ImVec2(0, 50)); // Add vertical spacing
  centered_text("Vert Velocity");
  ImGui::Text("     GPS: %5.2f m/s", FlightHistory.gps_vel_up);

  ImGui::End();
}

void estimated_pos_panel() {
  ImGui::Begin(EST_POS_PANEL);

  centered_text("Estimated Pos");
  if (ImPlot3D::BeginPlot("##Estimated Pos", ImVec2(-1, 500))) {
    double cs_x[2] = {-FlightHistory.state_pos_west, -FlightHistory.state_pos_west};
    double cs_y[2] = {FlightHistory.state_pos_north, FlightHistory.state_pos_north};
    double cs_z[2] = {0, FlightHistory.state_pos_up};

    double target_x[2] = {-FlightHistory.target_pos_west, -FlightHistory.target_pos_west};
    double target_y[2] = {FlightHistory.target_pos_north, FlightHistory.target_pos_north};
    double target_z[2] = {0, FlightHistory.target_pos_up};

    ImPlot3D::SetupAxes("East (m)", "North (m)", "Up (m)");
    ImPlot3D::SetupAxisLimits(ImAxis3D_Z, 0, 5);
    ImPlot3D::SetupAxisLimits(ImAxis3D_X, -5, 5);
    ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -5, 5);

    ImPlot3DSpec marker_spec;
    marker_spec.Marker = ImPlot3DMarker_Circle;
    marker_spec.MarkerSize = 5;
    marker_spec.MarkerFillColor = ImPlot3D::GetColormapColor(0, ImPlot3DColormap_Deep);
    ImPlot3D::PlotScatter("State", cs_x, cs_y, cs_z, 2, marker_spec);

    marker_spec.Marker = ImPlot3DMarker_Diamond;
    marker_spec.MarkerSize = 5;
    marker_spec.MarkerFillColor = ImPlot3D::GetColormapColor(1, ImPlot3DColormap_Deep);
    ImPlot3D::PlotScatter("Target", target_x, target_y, target_z, 2, marker_spec);

    ImPlot3DSpec line_spec;
    line_spec.LineColor = ImPlot3D::GetColormapColor(0, ImPlot3DColormap_Deep);
    line_spec.LineWeight = 2;
    ImPlot3D::PlotLine("State", cs_x, cs_y, cs_z, 2, line_spec);

    line_spec.LineColor = ImPlot3D::GetColormapColor(1, ImPlot3DColormap_Deep);
    line_spec.LineWeight = 2;
    ImPlot3D::PlotLine("Target", target_x, target_y, target_z, 2, line_spec);

    ImPlot3D::EndPlot();
  }

  ImGui::End();
}

void estimated_orientation_panel() {
  ImGui::Begin(EST_ROT_PANEL);

  centered_text("Orientation");
  ImVec4 q;
  q.x = FlightHistory.state_q_vec_0;
  q.y = FlightHistory.state_q_vec_1;
  q.z = FlightHistory.state_q_vec_2;
  q.w = FlightHistory.state_q_vec_new;
  rotatable_cube_plot(q);

  ImGui::End();
}

void controller_output_panel() {
  ImGui::Begin(CONTROLLER_OUTPUT_PANEL);

  centered_text("Controller Output");
  ImGui::Text("  Target Thrust: %5.2f N", FlightHistory.thrust_N);
  ImGui::Text("    Target Roll: %5.2f rad/s^2", FlightHistory.roll_N);
  ImGui::Text("         Thrust: %5.2f %%", 0); // TODO - these
  ImGui::Text("   Differential: %5.2f %%", 0);

  ImGui::End();
}

void gimbal_output_panel() {
  ImGui::Begin(GIMBAL_OUTPUT_PANEL);

  float x = FlightHistory.gimbal_yaw_raw * 180 / 3.1415;
  float y = FlightHistory.gimbal_pitch_raw * 180 / 3.1415;

  centered_text("Gimbal Command");
  if (ImPlot::BeginPlot("##Gimbal Command", ImVec2(-1, 250), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("Yaw (deg)", "Pitch (deg)");
    ImPlot::SetupAxesLimits(-20, 20, -20, 20, ImPlotCond_Always);
    ImPlot::PlotScatter("##Gimbal", &x, &y, 1);
    ImPlot::EndPlot();
  }

  ImGui::End();
}

char concat_msg_buf[1000];
void serial_control_panel() {
  ImGui::Begin(SERIAL_CONTROL_PANEL);

  ImGui::PushFont(panel_header_font);
  ImGui::SeparatorText("Serial Monitor");
  ImGui::PopFont();

  ImGuiInputTextFlags flags = ImGuiInputTextFlags_EnterReturnsTrue;
  static char inputBuffer[128] = ""; // buffer for text input
  bool should_send = false;

  if (ImGui::InputTextWithHint("##serial_input", "enter serial command", inputBuffer, IM_ARRAYSIZE(inputBuffer), flags)) {
    should_send = true;
  }

  bool text_box_active = ImGui::IsItemActive();
  ImGui::SameLine();
  if (rounded_button("Send", ImVec2(175, 0), IM_COL32(33, 112, 69, 255))) {
    should_send = true;
  }

  if (should_send) {
    ImGui::ActivateItemByID(ImGui::GetID("##serial_input"));
  }

  if (should_send) {
    // write_serial(inputBuffer);
    printf("You entered: %s\n", inputBuffer);
    inputBuffer[0] = '\0';
  }

  if (!text_box_active && ImGui::IsKeyPressed(ImGuiKey_K)) {
    // write_serial("k");
    printf("You entered: %s\n", "k");
  }

  ImGui::InputTextMultiline("##serial_output", concat_msg_buf, IM_ARRAYSIZE(concat_msg_buf), ImVec2(800, 100), ImGuiInputTextFlags_ReadOnly);

  // autoscroll code - // TODO - only enable if autoscroll enabled
  // ImGuiContext &g = *GImGui;
  // const char *child_window_name = NULL;
  // ImFormatStringToTempBuffer(&child_window_name, NULL, "%s/%s_%08X", g.CurrentWindow->Name, "##serial_output", ImGui::GetID("##serial_output"));
  // ImGuiWindow *child_window = ImGui::FindWindowByName(child_window_name);
  // ImGui::SetScrollY(child_window, child_window->ScrollMax.y);

  ImGui::End();
}

void build_dock_layout(ImGuiID dockspace_id) {
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

  ImGuiID left_panel = dockspace_id;
  ImGuiID right_panel = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, 0.5, nullptr, &left_panel);

  ImGuiID live_sensor_panel = left_panel;
  ImGuiID bottom_left_panel = ImGui::DockBuilderSplitNode(left_panel, ImGuiDir_Down, 1.0 / 3.0, nullptr, &live_sensor_panel);

  ImGuiID live_sensor_top = live_sensor_panel;
  ImGuiID live_sensor_middle = ImGui::DockBuilderSplitNode(live_sensor_panel, ImGuiDir_Down, 2.0 / 3.0, nullptr, &live_sensor_top);
  ImGuiID live_sensor_bottom = ImGui::DockBuilderSplitNode(live_sensor_middle, ImGuiDir_Down, 0.5, nullptr, &live_sensor_middle);

  ImGuiID live_sensor_tl = live_sensor_top;
  ImGuiID live_sensor_tr = ImGui::DockBuilderSplitNode(live_sensor_top, ImGuiDir_Right, 0.5, nullptr, &live_sensor_tl);
  ImGuiID live_sensor_ml = live_sensor_middle;
  ImGuiID live_sensor_mr = ImGui::DockBuilderSplitNode(live_sensor_middle, ImGuiDir_Right, 0.5, nullptr, &live_sensor_ml);
  ImGuiID live_sensor_bl = live_sensor_bottom;
  ImGuiID live_sensor_br = ImGui::DockBuilderSplitNode(live_sensor_bottom, ImGuiDir_Right, 0.5, nullptr, &live_sensor_bl);

  ImGuiID top_right_panel = right_panel;
  ImGuiID bottom_right_panel = ImGui::DockBuilderSplitNode(right_panel, ImGuiDir_Down, 0.5, nullptr, &top_right_panel);

  ImGuiID left_top_right_panel = top_right_panel;
  ImGuiID right_top_right_panel = ImGui::DockBuilderSplitNode(top_right_panel, ImGuiDir_Right, 0.5, nullptr, &left_top_right_panel);

  ImGuiID left_bottom_right_panel = bottom_right_panel;
  ImGuiID right_bottom_right_panel = ImGui::DockBuilderSplitNode(bottom_right_panel, ImGuiDir_Right, 0.5, nullptr, &left_bottom_right_panel);

  ImGui::DockBuilderDockWindow(IMU_ACCEL_PANEL, live_sensor_tl);
  ImGui::DockBuilderDockWindow(IMU_GYRO_PANEL, live_sensor_tr);
  ImGui::DockBuilderDockWindow(MAG_PANEL, live_sensor_ml);
  ImGui::DockBuilderDockWindow(GPS_VERT_PANEL, live_sensor_mr);
  ImGui::DockBuilderDockWindow(GPS_POS_PANEL, live_sensor_bl);
  ImGui::DockBuilderDockWindow(GPS_VEL_PANEL, live_sensor_br);
  ImGui::DockBuilderDockWindow(SERIAL_CONTROL_PANEL, bottom_left_panel);

  ImGui::DockBuilderDockWindow(EST_POS_PANEL, left_top_right_panel);
  ImGui::DockBuilderDockWindow(EST_ROT_PANEL, right_top_right_panel);
  ImGui::DockBuilderDockWindow(CONTROLLER_OUTPUT_PANEL, left_bottom_right_panel);
  ImGui::DockBuilderDockWindow(GIMBAL_OUTPUT_PANEL, right_bottom_right_panel);

  ImGui::DockBuilderFinish(dockspace_id);
}

void render_loop() {
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

  const ImGuiViewport *vp = ImGui::GetMainViewport();

  ImGui::SetNextWindowPos(vp->Pos);
  ImGui::SetNextWindowSize(vp->Size);
  ImGui::SetNextWindowViewport(vp->ID);

  ImGui::Begin("MainWindow", nullptr, flags);

  ImGuiID dockspace_id = ImGui::GetID("MyDockspace");
  ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_AutoHideTabBar);

  static bool first_time = true;
  if (first_time || ImGui::IsKeyPressed(ImGuiKey_R)) {
    build_dock_layout(dockspace_id);
    first_time = false;
  }

  // create each window
  imu_accel_panel();
  imu_gyro_panel();
  mag_panel();
  gps_pos_panel();
  gps_vel_panel();
  gps_vert_panel();
  estimated_pos_panel();
  estimated_orientation_panel();
  controller_output_panel();
  gimbal_output_panel();

  serial_control_panel();

  ImGui::End();
}

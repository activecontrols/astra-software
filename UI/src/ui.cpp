#include "ui.h"
#include "flight_data.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "implot3d.h"
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

const char *names[9] = {IMU_ACCEL_PANEL, IMU_GYRO_PANEL, MAG_PANEL, "", "", "", "", "", ""};

void build_dock_layout(ImGuiID dockspace_id) {
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

  ImGuiID cols[3];
  ImGuiID rows[3][3];

  // Split into 3 columns
  cols[0] = dockspace_id;
  cols[1] = ImGui::DockBuilderSplitNode(cols[0], ImGuiDir_Right, 2.0f / 3.0f, nullptr, &cols[0]);
  cols[2] = ImGui::DockBuilderSplitNode(cols[1], ImGuiDir_Right, 0.5f, nullptr, &cols[1]);

  // Split each column into 3 rows
  for (int c = 0; c < 3; c++) {
    rows[c][0] = cols[c];
    rows[c][1] = ImGui::DockBuilderSplitNode(rows[c][0], ImGuiDir_Down, 2.0f / 3.0f, nullptr, &rows[c][0]);
    rows[c][2] = ImGui::DockBuilderSplitNode(rows[c][1], ImGuiDir_Down, 0.5f, nullptr, &rows[c][1]);
  }

  // Dock example plot windows
  int idx = 0;
  for (int c = 0; c < 3; c++) {
    for (int r = 0; r < 3; r++) {
      ImGui::DockBuilderDockWindow(names[c * 3 + r], rows[c][r]);
    }
  }

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
  ImGui::DockSpace(dockspace_id, ImVec2(0, 0));

  static bool first_time = true;
  if (first_time || ImGui::IsKeyPressed(ImGuiKey_R)) {
    build_dock_layout(dockspace_id);
    first_time = false;
  }

  // create each window
  imu_accel_panel();
  imu_gyro_panel();
  mag_panel();

  ImGui::End();
}

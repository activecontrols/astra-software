#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "math.h"
#include <string>

#include <iostream>

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

bool daq_button(const char *label, const ImVec2 &size,
                ImU32 color, float rounding = 10.0f) {
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

void control_panel() {
  ImGui::Text("Control Panel");

  if (ImGui::BeginTable("start_abort", 2)) {
    ImGui::TableNextColumn();
    if (daq_button("START", ImVec2(-1, 50), IM_COL32(33, 112, 69, 255))) {
      std::cout << "START" << std::endl;
    };
    ImGui::TableNextColumn();
    if (daq_button("ABORT", ImVec2(-1, 50), IM_COL32(204, 0, 0, 255))) {
      std::cout << "END" << std::endl;
    };
    ImGui::EndTable();
  }

  if (ImGui::BeginTable("do_control", 5)) {
    ImGui::TableNextRow();
    for (int i = 0; i < 5; i++) {
      ImGui::TableSetColumnIndex(i);
      ImGui::PushID(i);
      std::string do_name = "DO " + std::to_string(i + 1);
      toggle_button(do_name.c_str(), ImVec2(-1, 50), IM_COL32(33, 112, 69, 255), IM_COL32(50, 50, 50, 255), &ui_do_state[i]);
      ImGui::PopID();
    }
    ImGui::EndTable();
  }

  ImGui::Dummy(ImVec2(0, 100)); // Add vertical spacing

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

void plot_panel() {
  for (int i = 0; i < 3; i++) {
    std::string plot_name = "Plot " + std::to_string(i + 1);
    if (ImPlot::BeginPlot(plot_name.c_str(), ImVec2(-1, 0))) { // width = fill, height auto
      // Example: simple sine wave
      static float xs[1000], ys[1000];
      for (int j = 0; j < 1000; ++j) {
        xs[j] = j * 0.01f;
        ys[j] = sin(xs[j] + i); // different phase for each plot
      }
      ImPlot::PlotLine("Sine", xs, ys, 1000);
      ImPlot::EndPlot();
    }
  }
}

void render_loop() {
  ImGuiIO &io = ImGui::GetIO();
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(io.DisplaySize);

  ImGui::Begin("MainWindow", nullptr,
               ImGuiWindowFlags_NoTitleBar |
                   ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse |
                   ImGuiWindowFlags_NoBringToFrontOnFocus |
                   ImGuiWindowFlags_NoNavFocus);

  // Split into 2 sections: left (buttons) and right (plots)
  ImGui::Columns(2, nullptr, true); // true = resizable
  control_panel();
  ImGui::NextColumn(); // move to right column
  plot_panel();

  ImGui::End();
}

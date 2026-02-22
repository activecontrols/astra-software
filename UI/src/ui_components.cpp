#include "ui_components.h"

ImFont *panel_header_font;
ImFont *large_font;

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
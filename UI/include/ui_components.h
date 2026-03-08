#ifndef ASTRA_GS_UI_COMPONENTS_H
#define ASTRA_GS_UI_COMPONENTS_H

#include "imgui.h"

extern ImFont *panel_header_font;
extern ImFont *large_font;

void centered_text(const char *text);
bool rounded_button(const char *label, const ImVec2 &size, ImU32 color, float rounding = 10.0f);
void colored_flag(char *text, bool state, ImVec4 on_color, ImVec4 off_color, char *imgui_id);

#endif
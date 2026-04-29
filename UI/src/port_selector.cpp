#include "port_selector.h"
#include "flight_data_state.h"
#include "imgui.h"
#include "platform_win.h"

PortSelector::PortSelector(const char *name, const char *combo_id, const char *checkbox_id) {
  // copy in case they point to volatile data
  _name = (char *)malloc(strlen(name) + 1);
  strcpy(_name, name);

  _combo_id = (char *)malloc(strlen(combo_id) + 1);
  strcpy(_combo_id, combo_id);

  _checkbox_id = (char *)malloc(strlen(checkbox_id) + 1);
  strcpy(_checkbox_id, checkbox_id);
}

void PortSelector::render() {
  // port name
  ImGui::Text(_name);
  ImGui::SameLine(150);
  // port dropdown

  if (ImGui::BeginCombo(_combo_id, _index < FlightDataState.ports.size() ? FlightDataState.ports[_index].friendlyName.c_str() : "No Serial Ports Found")) {
    for (int i = 0; i < FlightDataState.ports.size(); i++) {
      ComPortInfo port = FlightDataState.ports[i];
      const bool is_selected = (_index == i);
      if (ImGui::Selectable(port.friendlyName.c_str(), is_selected)) {
        _index = i;
        close();
      }

      // Set the initial focus when opening the combo (scrolling to selection)
      if (is_selected)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }

  // port checkbox
  ImGui::SameLine();

  bool enabled = is_open();
  if (ImGui::Checkbox(_checkbox_id, &enabled)) {
    if (!enabled) {
      close();
    } else if (_index < FlightDataState.ports.size()) {
      active_port = open_serial_port(FlightDataState.ports[_index].portName.c_str());
    }
  }

  return;
}

void PortSelector::close() {
  delete active_port;
  active_port = nullptr;
}

bool PortSelector::is_open() {
  return active_port && active_port->is_open();
}

// return 0 on error
int PortSelector::write(const char *data, unsigned int len, bool end_with_newline) {
  if (!is_open()) {
    return 0;
  }

  return active_port->write(data, len, end_with_newline);
}

// return # of bytes read, -1 on error
int PortSelector::read(char *data, unsigned int max_len) {
  if (!is_open()) {
    return -1;
  }
  int read_count = active_port->read(data, max_len);
  return read_count;
}

PortSelector::~PortSelector() {
  free(_name);
  free(_combo_id);
  free(_checkbox_id);
  return;
}


// multiple panels will eventually need this component so it's placed here
void render_port_refresh_button()
{
  if (ImGui::Button("\uE117")) {
    FlightDataState.ports = enumerate_ports();
  }
}
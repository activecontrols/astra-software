#pragma once

#include "Serial.h"

class PortSelector {
public:
  PortSelector(const char *name, const char *combo_id, const char *checkbox_id);

  void render();

  bool is_open();

  void close();

  ~PortSelector();

  // return 0 on error
  int write(const char *data, unsigned int len, bool end_with_newline = false);

  // return # of bytes read, -1 on error
  int read(char *data, unsigned int max_len);

private:
  Serial *active_port = nullptr;
  char *_name;
  char *_combo_id;
  char *_checkbox_id;
  int _index = 0;
};


// renders a refresh button for refreshing the available ports list in FlightDataState
void render_port_refresh_button();
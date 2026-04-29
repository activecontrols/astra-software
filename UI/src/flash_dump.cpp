#include "flash_dump.h"
#include "csr_trigger.h"
#include "flight_data_state.h"
#include "imgui.h"
#include "platform_win.h"
#include "port_selector.h"

namespace FlashDump {
bool _in_progress = false;
const char *result_message = nullptr;

unsigned int bytes_downloaded = 0;

PortSelector flash_usb_port("Flash: ", "##dump_combo", "##dump_checkbox");

PortSelector &selected_download_port = flash_usb_port;

FILE *fout;

char out_path[MAX_PATH];

const unsigned int PAGE_SIZE = 256;
uint8_t recv_buf[PAGE_SIZE + 3];
unsigned int recv_buf_pos = 0;

uint64_t last_byte_time = 0;

bool is_in_progress() {
  return _in_progress;
}

void finish() {
  _in_progress = false;
  fclose(fout);
}

void update() {
  if (!is_in_progress()) {
    return;
  }

  // timeout of 5s
  if (get_time_us() - last_byte_time > 5'000'000) {
    result_message = "ERROR: CONNECTION TIMED OUT";
    selected_download_port.write("k", 1, false);
    finish();
    return;
  }

  if (!selected_download_port.is_open()) {
    result_message = "ERROR: PORT CLOSED";
    finish();
    return;
  }

  int n_bytes = selected_download_port.read((char *)(recv_buf + recv_buf_pos), sizeof(recv_buf) - recv_buf_pos);

  if (n_bytes > 0) {
    last_byte_time = get_time_us();
    recv_buf_pos += n_bytes;
  }

  if (recv_buf_pos == sizeof(recv_buf)) {
    // checksum calculation
    uint8_t cs_A = 0;
    uint8_t cs_B = 0;
    for (int i = 0; i < PAGE_SIZE; ++i) {
      cs_A += recv_buf[i];
      cs_B += cs_A;
    }

    if (cs_A != recv_buf[PAGE_SIZE] || cs_B != recv_buf[PAGE_SIZE + 1]) {
      result_message = "ERROR: CHECKSUM FAILED";
      finish();
      return;
    }

    bytes_downloaded += PAGE_SIZE;

    fwrite(recv_buf, PAGE_SIZE, 1, fout);

    if (recv_buf[PAGE_SIZE + 2] == 'k') {
      result_message = "Finished Successfully";
      finish();

      // automatically switch to csr menu and set input path to the dump output
      FlightDataState.data_input_mode = MODE_CSR_TRIGGER;
      strcpy(CSRTrigger::input_fp, out_path);
      return;
    }

    selected_download_port.write("c", 1, false);
    recv_buf_pos = 0;
  }

  return;
}

void render() {
  if (is_in_progress()) {
    // render dump progress (how many bytes downloaded)
    ImGui::Text("%u BYTES DOWNLOADED", bytes_downloaded);
  } else {

    // serial port selections
    FlightDataState.fv_serial.render(); // fv serial selector
    ImGui::SameLine();
    render_port_refresh_button(); // port refresh button

    flash_usb_port.render();

    if (FlightDataState.fv_serial.is_open() && ImGui::Button("Begin")) {
      static const char *specs[] = {"*.bin", "*.*"};
      static const char *labels[] = {"Binary File", "All Files"};
      SaveFileDialog(out_path, specs, labels, 2, 0);

      fout = fopen(out_path, "wb");

      if (fout) {
        // begin by sending start command
        const char start_cmd[] = "\ndump_flash";
        if (FlightDataState.fv_serial.write(start_cmd, sizeof(start_cmd) - 1, true)) {
          // if send succeeded, switch to in-progress state
          _in_progress = true;
          recv_buf_pos = 0;
          bytes_downloaded = 0;

          // if dump port hasn't been chosen, assume we are dumping over fv serial
          selected_download_port = flash_usb_port.is_open() ? flash_usb_port : FlightDataState.fv_serial;
          last_byte_time = get_time_us();
        } else {
          fclose(fout);
        }
      }
    }

    if (result_message) {
      ImGui::Text("%s", result_message);
    }
  }
}
}; // namespace FlashDump
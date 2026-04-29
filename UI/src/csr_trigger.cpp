#include "csr_trigger.h"
#include "flight_data_state.h"
#include "imgui.h"
#include "platform_win.h"
#include "flight_data.h"

namespace CSRTrigger {
bool _in_progress = false;

char input_fp[MAX_PATH]{};
char output_fp[MAX_PATH]{};
char output_csv_fp[MAX_PATH]{};

bool output_csv = false;

enum Stage { MAKE = 0, CSR = 1 };

Stage stage = Stage::MAKE;
void *current_handle = nullptr;
char *success_message = nullptr;

void render() {
  if (in_progress()) {
    // just display a status
    if (stage == Stage::MAKE) {
      ImGui::Text("Building CSR...");
    } else {
      ImGui::Text("Running CSR...");
    }
  } else {
    // select input file
    if (ImGui::Button("Choose Input File")) {
      OpenFileDialog(input_fp);
    }

    if (input_fp[0]) {
      ImGui::Text("%s", get_filename_from_path(input_fp));
    }

    // select output file
    if (ImGui::Button("Choose CSR Output")) {
      const char *specs[] = {"*.bin", "*.*"};
      const char *labels[] = {"Binary File", "All Files"};
      SaveFileDialog(output_fp, specs, labels, 2, 0);
    }

    if (output_fp[0]) {
      ImGui::Text("%s", get_filename_from_path(output_fp));
    }

    // optional - select csv output file
    ImGui::Checkbox("Output CSV", &output_csv);

    if (output_csv) {
      if (ImGui::Button("Choose CSV Output")) {
        const char *specs[] = {"*.csv", "*.*"};
        const char *labels[] = {"Comma Separated Values File", "All Files"};
        SaveFileDialog(output_csv_fp, specs, labels, 2, 0);
      }

      if (output_csv_fp[0]) {
        ImGui::Text("%s", get_filename_from_path(output_csv_fp));
      }
    }

    if (input_fp[0] && output_fp[0] && (!output_csv || output_csv_fp[0])) {
      if (ImGui::Button("Begin CSR")) {
        _in_progress = true;

        stage = Stage::MAKE;
        // trigger a CSR build
        current_handle = spawn_csr_make();
      }
    }

    if (success_message) {
      ImGui::Text("%s", success_message);
    }
  }
}

bool in_progress() {
  return _in_progress;
}

void update() {
  if (!in_progress()) {
    return;
  }

  Process_Status status = check_process_status(current_handle);

  if (status.running) {
    return;
  }

  close_process(current_handle);
  current_handle = nullptr;

  if (stage == Stage::MAKE) {
    if (status.exit_code == 0) // build succeeded
    {
      stage = Stage::CSR;
      current_handle = spawn_csr(input_fp, output_fp, output_csv ? output_csv_fp : nullptr);
    } else {
      success_message = "ERROR: BUILD FAILED";
      _in_progress = false;
      return;
    }
  } else {                     // stage == Stage::CSR
    if (status.exit_code == 0) // csr succeeded
    {
      success_message = "CSR Finished Successfully";

      FlightDataState.data_input_mode = MODE_FILE_INPUT;
      strcpy(FlightDataState.selected_file_path, output_fp);
      load_flight_replay();
    } else {
      success_message = "ERROR: CSR FAILED";
    }
    _in_progress = false;
    return;
  }
}

}; // namespace CSRTrigger
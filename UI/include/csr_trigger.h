#pragma once
#include "platform_win.h"

// abstract:
// - file input for flight log
// - needs to trigger CSR by first calling make (to ensure csr executable is updated)
//   - fail if CSR build fails
// - call csr executable
//   - pass flight log file path, output file path, and optional csv file path
// - wait for csr to complete
//   - fail if csr gives error code
// - switch to flight replay window and set the input file path to the csr output
namespace CSRTrigger {
extern char input_fp[MAX_PATH];
void render();

bool in_progress();

void update();

}; // namespace CSRTrigger
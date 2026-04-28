#ifndef ASTRA_GS_PLATFORM_WIN_H
#define ASTRA_GS_PLATFORM_WIN_H

#include "flight_data_state.h" // for ComPortInfo

void OpenFileDialog(char *path);
const char *get_filename_from_path(const char *full_path);
std::vector<ComPortInfo> enumerate_ports();

Serial *open_serial_port(const char *name);
unsigned long long get_time_us();
void platform_begin();

void SaveFileDialog(char *path);
void SaveFileDialog(char *path, const char *file_specs[], const char *spec_names[], unsigned int n_specs, unsigned int default_spec_idx);

// triggers a build of the csr code
void* spawn_csr_make();

// spawns csr process
void* spawn_csr(const char* input_fp, const char* output_fp, const char* output_csv_fp = nullptr);


struct Process_Status
{
    bool running;
    int exit_code;
};
Process_Status check_process_status(void *handle);

void close_process(void* handle);

#endif
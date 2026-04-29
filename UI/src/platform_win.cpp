// windows specific code (non-UI) code
// will need to be re-written for porting to linux/macos
// file dialog
// serial port interface

#include "platform_win.h"
#include <windows.h>

#include <devguid.h>
#include <regstr.h>
#include <setupapi.h>
#include <shlwapi.h>  // file path management
#include <shobjidl.h> // IFileOpenDialog

#include "serial.h"
#include <cstdint>

uint64_t lpFrequency;

void _open_serial_port(HANDLE *hSerial, const char *com_port);
void _close_serial_port(HANDLE *hSerial);
void _read_from_serial_port(HANDLE *hSerial, bool *open_flag, char *read_buf, size_t MAX_READ_LEN, int *bytes_read);
void _write_to_serial_port(HANDLE *hSerial, bool *open_flag, const char *msg, size_t len, bool end_with_newline);

class WindowsSerial : Serial {
private:
  HANDLE handle;
  bool _is_open;

public:
  WindowsSerial(HANDLE h) {
    handle = h;
    _is_open = true;
  }

  bool is_open() {
    return _is_open;
  }

  int write(const char *data, unsigned int len, bool end_with_newline = false) {
    if (!_is_open) {
      return 0;
    }
    _write_to_serial_port(&handle, &_is_open, data, len, end_with_newline);
    if (_is_open) {
      return 1;
    }
    return 0;
  }

  int read(char *data, unsigned int max_len) {
    if (!_is_open) {
      return 0;
    }
    int bytes_read = -1;
    _read_from_serial_port(&handle, &_is_open, data, max_len, &bytes_read);

    if (!_is_open) {
      return -1;
    }
    return bytes_read;
  }

  void close() {
    if (!_is_open) {
      return;
    }
    _close_serial_port(&handle);
    _is_open = false;
    return;
  }

  ~WindowsSerial() {
    close();
  }
};

// will fill path if file is chosen
// TODO - restrict file type
void OpenFileDialog(char *path) {
  path[0] = '\0';

  HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
  if (FAILED(hr)) {
    return;
  }

  IFileOpenDialog *pFileOpen = nullptr;

  hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL, IID_IFileOpenDialog, reinterpret_cast<void **>(&pFileOpen));

  if (SUCCEEDED(hr)) {
    hr = pFileOpen->Show(NULL);

    if (SUCCEEDED(hr)) {
      IShellItem *pItem;
      hr = pFileOpen->GetResult(&pItem);

      if (SUCCEEDED(hr)) {
        PWSTR pszFilePath = nullptr;
        hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);

        if (SUCCEEDED(hr)) {
          WideCharToMultiByte(CP_UTF8, 0, pszFilePath, -1, path, MAX_PATH, NULL, NULL);
          CoTaskMemFree(pszFilePath);
        }

        pItem->Release();
      }
    }

    pFileOpen->Release();
  }

  CoUninitialize();
}

void SaveFileDialog(char *path, const char *file_specs[], const char *spec_names[], unsigned int n_specs, unsigned int default_spec_idx) {
  path[0] = '\0';

  HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
  if (FAILED(hr)) {
    return;
  }

  IFileSaveDialog *pFileSave = nullptr;

  hr = CoCreateInstance(CLSID_FileSaveDialog, NULL, CLSCTX_ALL, IID_IFileSaveDialog, reinterpret_cast<void **>(&pFileSave));

  // convert types to array of win32 structs
  COMDLG_FILTERSPEC *filter_spec = (COMDLG_FILTERSPEC *)malloc(n_specs * sizeof(*filter_spec));

  for (int i = 0; i < n_specs; ++i) {
    int n = MultiByteToWideChar(CP_UTF8, 0, spec_names[i], -1, (LPWSTR)filter_spec[i].pszName, 0);
    filter_spec[i].pszName = (LPCWSTR)malloc(n * sizeof(*(filter_spec[i].pszName)));
    MultiByteToWideChar(CP_UTF8, 0, spec_names[i], -1, (LPWSTR)filter_spec[i].pszName, n);

    n = MultiByteToWideChar(CP_UTF8, 0, file_specs[i], -1, (LPWSTR)filter_spec[i].pszSpec, 0);
    filter_spec[i].pszSpec = (LPCWSTR)malloc(n * sizeof(*(filter_spec[i].pszSpec)));
    MultiByteToWideChar(CP_UTF8, 0, file_specs[i], -1, (LPWSTR)filter_spec[i].pszSpec, n);
  }

  hr = pFileSave->SetFileTypes(n_specs, filter_spec);

  if (SUCCEEDED(hr)) {
    hr = pFileSave->SetFileTypeIndex(default_spec_idx);
    if (SUCCEEDED(hr)) {
      hr = pFileSave->SetDefaultExtension(filter_spec[default_spec_idx].pszSpec);
      if (SUCCEEDED(hr)) {
        hr = pFileSave->Show(NULL);

        if (SUCCEEDED(hr)) {
          IShellItem *pItem;
          hr = pFileSave->GetResult(&pItem);

          if (SUCCEEDED(hr)) {
            PWSTR pszFilePath = nullptr;
            hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);

            if (SUCCEEDED(hr)) {
              WideCharToMultiByte(CP_UTF8, 0, pszFilePath, -1, path, MAX_PATH, NULL, NULL);
              CoTaskMemFree(pszFilePath);
            }

            pItem->Release();
          }
        }

        pFileSave->Release();
      }

      CoUninitialize();
    }
  }

  for (int i = 0; i < n_specs; ++i) {
    free((void *)filter_spec[i].pszName);
    free((void *)filter_spec[i].pszSpec);
  }

  free(filter_spec);
  return;
}

void SaveFileDialog(char *path) {
  static const char *default_spec[] = {"*.*"};
  static const char *default_label[] = {"any"};
  SaveFileDialog(path, default_spec, default_label, 1, 0);
  return;
}

const char *get_filename_from_path(const char *full_path) {
  const char *filename = PathFindFileNameA(full_path);
  return filename;
}

std::vector<ComPortInfo> enumerate_ports() {
  std::vector<ComPortInfo> ports;

  HDEVINFO deviceInfoSet = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);

  if (deviceInfoSet == INVALID_HANDLE_VALUE)
    return ports;

  SP_DEVINFO_DATA devInfoData;
  devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

  for (DWORD i = 0; SetupDiEnumDeviceInfo(deviceInfoSet, i, &devInfoData); i++) {
    char friendlyName[256];
    if (SetupDiGetDeviceRegistryPropertyA(deviceInfoSet, &devInfoData, SPDRP_FRIENDLYNAME, NULL, (PBYTE)friendlyName, sizeof(friendlyName), NULL)) {
      std::string fn = friendlyName;

      // Extract COM port name from "(COMx)"
      size_t start = fn.find("(COM");
      size_t end = fn.find(")", start);

      if (start != std::string::npos && end != std::string::npos) {
        std::string port = fn.substr(start + 1, end - start - 1);
        ports.push_back({port, fn});
      }
    }
  }

  SetupDiDestroyDeviceInfoList(deviceInfoSet);
  return ports;
}

Serial *open_serial_port(const char *name) {
  HANDLE h;
  _open_serial_port(&h, name);
  if (h == INVALID_HANDLE_VALUE) {
    return nullptr;
  }
  // is this cursed idk? don't classes store deconstructor in their vtable?
  return (Serial *)new WindowsSerial(h);
}

void _open_serial_port(HANDLE *hSerial, const char *com_port) {
  char full_port_name[20];
  snprintf(full_port_name, sizeof(full_port_name), "\\\\.\\%s", com_port);
  *hSerial = CreateFileA(full_port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

  if (*hSerial == INVALID_HANDLE_VALUE) {
    printf("Error opening serial port - make sure port name is correct and all other serial monitors are closed.\n");
    return;
  }

  // --- configure port ---
  DCB dcb = {0};
  dcb.DCBlength = sizeof(dcb);

  GetCommState(*hSerial, &dcb);
  dcb.BaudRate = CBR_57600; // TODO - set radio baud rate
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;

  dcb.fBinary = TRUE;                   // Must be TRUE for Win32
  dcb.fParity = FALSE;                  // Disable parity checking
  dcb.fOutxCtsFlow = FALSE;             // Disable CTS flow control (CRITICAL)
  dcb.fOutxDsrFlow = FALSE;             // Disable DSR flow control (CRITICAL)
  dcb.fDtrControl = DTR_CONTROL_ENABLE; // Ensure DTR is on
  dcb.fDsrSensitivity = FALSE;
  dcb.fOutX = FALSE; // Disable XON/XOFF
  dcb.fInX = FALSE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE; // Ensure RTS is on

  if (!SetCommState(*hSerial, &dcb)) {
    printf("SetCommState error while opening serial port.\n");
    _close_serial_port(hSerial);
  }

  // --- set timeouts (non-blocking with small wait) ---
  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 2; // wait for 2 ms
  timeouts.ReadTotalTimeoutConstant = 10;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  if (!SetCommTimeouts(*hSerial, &timeouts)) {
    printf("SetCommTimeouts error while opening serial port.\n");
    _close_serial_port(hSerial);
  }

  printf("Serial port opened!\n");
}

void _close_serial_port(HANDLE *hSerial) {
  if (*hSerial != INVALID_HANDLE_VALUE) {
    printf("Serial port closed.\n");
    CloseHandle(*hSerial);
  }
  *hSerial = INVALID_HANDLE_VALUE;
}

void _read_from_serial_port(HANDLE *hSerial, bool *open_flag, char *read_buf, size_t MAX_READ_LEN, int *bytes_read) {
  DWORD dword_bytes_read = 0;

  if (!ReadFile(*hSerial, read_buf, MAX_READ_LEN, &dword_bytes_read, NULL)) {
    printf("ReadFile error - closing serial port.\n");
    _close_serial_port(hSerial);
    *open_flag = false;
    *bytes_read = 0;
    return;
  }

  *bytes_read = dword_bytes_read;
}

void _write_to_serial_port(HANDLE *hSerial, bool *open_flag, const char *msg, size_t len, bool end_with_newline) {
  DWORD bytes_written = 0;
  if (!WriteFile(*hSerial, msg, len, &bytes_written, NULL)) {
    printf("WriteFile error - closing serial port.\n");
    _close_serial_port(hSerial);
    *open_flag = false;
    return;
  }

  if (end_with_newline) {
    char newline = '\n';

    if (!WriteFile(*hSerial, &newline, 1, &bytes_written, NULL)) {
      printf("WriteFile error - closing serial port.\n");
      _close_serial_port(hSerial);
      *open_flag = false;
      return;
    }
  }
}

// get time in microseconds, not synced to any external reference
unsigned long long get_time_us() {
  LARGE_INTEGER temp;
  QueryPerformanceCounter(&temp);
  return temp.QuadPart * 1000000 / lpFrequency; // todo - is potential overflow here concerning? nahhh
}

void platform_begin() {
  LARGE_INTEGER temp;
  QueryPerformanceFrequency(&temp); // per windows docs - this value doesn't change and can be cached
  lpFrequency = temp.QuadPart;
}

void *spawn_csr_make() {
  PROCESS_INFORMATION *proc_info = new PROCESS_INFORMATION;
  STARTUPINFO si{};
  si.cb = sizeof(si);

  char cmd[] = "mingw32-make.exe";
  bool success = CreateProcessA(nullptr, cmd, NULL, NULL, false, NORMAL_PRIORITY_CLASS | CREATE_NEW_PROCESS_GROUP, NULL, "..\\data_postprocessing", &si, proc_info);

  if (!success) {
    int error = GetLastError();
    printf("Failed to start process: %d\n", error);
  }
  return proc_info;
}

void *spawn_csr(const char *input_fp, const char *output_fp, const char *output_csv_fp) {
  PROCESS_INFORMATION *proc_info = new PROCESS_INFORMATION;
  STARTUPINFO si{};
  si.cb = sizeof(si);

  static char cmd_str[1024];

  if (output_csv_fp) {
    snprintf(cmd_str, sizeof(cmd_str), "\"..\\data_postprocessing\\app.exe\" \"%s\" \"%s\"", input_fp, output_fp);
  } else {
    snprintf(cmd_str, sizeof(cmd_str), "\"..\\data_postprocessing\\app.exe\" \"%s\" \"%s\" \"%s\"", input_fp, output_fp, output_csv_fp);
  }

  bool success = CreateProcessA(nullptr, cmd_str, NULL, NULL, false, ABOVE_NORMAL_PRIORITY_CLASS | CREATE_NEW_PROCESS_GROUP, NULL, nullptr, &si, proc_info);

  return proc_info;
}

Process_Status check_process_status(void *handle) {
  Process_Status res{};
  PROCESS_INFORMATION *proc_info = (PROCESS_INFORMATION *)(handle);
  DWORD code;
  bool succeeded = GetExitCodeProcess(proc_info->hProcess, &code);

  if (!succeeded) {
    printf("GetExitCodeProcess failed: %d\n", GetLastError());
    return res;
  }

  if (code == STILL_ACTIVE) {
    res.running = true;
    return res;
  } else {
    res.running = false;
    res.exit_code = code;
    return res;
  }
}

void close_process(void *handle) {
  PROCESS_INFORMATION *hproc = (PROCESS_INFORMATION *)handle;

  CloseHandle(hproc->hProcess);
  CloseHandle(hproc->hThread);
  delete hproc;
  return;
}
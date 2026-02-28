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

void open_serial_port(HANDLE *hSerial, const char *com_port) {
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
  dcb.BaudRate = CBR_115200;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;

  if (!SetCommState(*hSerial, &dcb)) {
    printf("SetCommState error while opening serial port.\n");
    close_serial_port(hSerial);
  }

  // --- set timeouts (non-blocking with small wait) ---
  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 10;
  timeouts.ReadTotalTimeoutConstant = 10;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  if (!SetCommTimeouts(*hSerial, &timeouts)) {
    printf("SetCommTimeouts error while opening serial port.\n");
    close_serial_port(hSerial);
  }

  printf("Serial port opened!\n");
}

void close_serial_port(HANDLE *hSerial) {
  if (*hSerial != INVALID_HANDLE_VALUE) {
    printf("Serial port closed.\n");
    CloseHandle(*hSerial);
  }
  *hSerial = INVALID_HANDLE_VALUE;
}

void read_from_serial_port(HANDLE *hSerial, bool *open_flag, char *read_buf, size_t MAX_READ_LEN, int *bytes_read) {
  DWORD dword_bytes_read = 0;

  if (!ReadFile(*hSerial, read_buf, MAX_READ_LEN, &dword_bytes_read, NULL)) {
    printf("ReadFile error - closing serial port.\n");
    close_serial_port(hSerial);
    *open_flag = false;
    *bytes_read = 0;
    return;
  }

  *bytes_read = dword_bytes_read;
}

void write_to_serial_port(HANDLE *hSerial, bool *open_flag, const char *msg, size_t len, bool end_with_newline) {
  DWORD bytes_written = 0;
  if (!WriteFile(*hSerial, msg, len, &bytes_written, NULL)) {
    printf("WriteFile error - closing serial port.\n");
    close_serial_port(hSerial);
    *open_flag = false;
    return;
  }

  char newline = '\n';

  if (!WriteFile(*hSerial, &newline, 1, &bytes_written, NULL)) {
    printf("WriteFile error - closing serial port.\n");
    close_serial_port(hSerial);
    *open_flag = false;
    return;
  }
}
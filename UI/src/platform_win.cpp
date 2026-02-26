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
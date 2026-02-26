// windows specific code (non-UI) code
// will need to be re-written for porting to linux/macos
// file dialog
// serial port interface

#include "platform_win.h"
#include <shlwapi.h>  // file path management
#include <shobjidl.h> // IFileOpenDialog
#include <windows.h>

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

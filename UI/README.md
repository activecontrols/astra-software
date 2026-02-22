# Building the UI

Just want the latest .exe? - go to https://github.com/activecontrols/astra-software/actions/workflows/ui_build.yml

## Setting up gcc on Windows

Follow the instructions here, including updating PATH: https://code.visualstudio.com/docs/cpp/config-mingw

mingw provides the infrastructure needed to install and run linux tools natively on windows. (Note: as opposed to WSL, which isn't useful here because we are building a windows executable).

After restarting VS Code, `gcc`, `gdb`, and `g++` should work from the VS Code terminal.

## Building

At this point, running `mingw32-make` in the UI folder should build the project.
The relative path to each .cpp file is needed for the build are specified in the makefile. This structure is re-created in the build folder.
So `imgui/backends/imgui_impl_dx11.cpp` gets compiled to `build/imgui/backends/imgui_impl_dx11.o`. **Any new source files must be added to the makefile.**

Note: the makefile only detects changes in `.cpp` files. If you change a `.h` file, the `.cpp` files that rely on it **will not be rebuilt unless you also modify them.** If needed, just run `mingw32-make clean` to clear out all build artifacts so that all files will be rebuilt with the next time `make` is called.

## Intellisense

Intellisense should "just work" once you have mingw installed (along with https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) and you have the `UI` folder open in VSCode (not the outer folder that has ASTRA embedded code). If it doesn't, make sure that you can run `g++` from the command line. If something isn't working, look at [c_cpp_properties.json](.vscode/c_cpp_properties.json) and the intellisense status in the bottom right of the VSCode window.
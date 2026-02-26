CXX      := g++
CXXFLAGS := -Iimgui -Iimgui/backends -Iimplot -Iimplot3d -Iinclude -MMD -MP
LDFLAGS  := -luser32 -lgdi32 -ld3d11 -ldxgi -ld3dcompiler -ldwmapi -lole32 -luuid

# Sources
IMGUI_SRC := imgui\imgui.cpp \
             imgui\imgui_draw.cpp \
             imgui\imgui_widgets.cpp \
             imgui\imgui_tables.cpp \
             imgui\backends\imgui_impl_win32.cpp \
             imgui\backends\imgui_impl_dx11.cpp \
             implot\implot.cpp \
             implot\implot_items.cpp \
             implot3d\implot3d.cpp \
             implot3d\implot3d_items.cpp

APP_SRC   := main.cpp ui.cpp ui_components.cpp ui_graphs.cpp flight_data.cpp platform_win.cpp
SRC       := $(APP_SRC) $(IMGUI_SRC)
BUILD_FOLDER := build

# Objects
OBJ := $(patsubst %.cpp,$(BUILD_FOLDER)/%.o,$(SRC))

# Output
TARGET    := app.exe

all: $(TARGET)

# Link step
$(TARGET): $(OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BUILD_FOLDER):
	mkdir $(BUILD_FOLDER)
	mkdir $(BUILD_FOLDER)\imgui
	mkdir $(BUILD_FOLDER)\imgui\backends
	mkdir $(BUILD_FOLDER)\implot
	mkdir $(BUILD_FOLDER)\implot3d

# Compile step
$(BUILD_FOLDER)/%.o: src/%.cpp | $(BUILD_FOLDER)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile step (imgui)
$(BUILD_FOLDER)/%.o: %.cpp | $(BUILD_FOLDER)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@if exist $(BUILD_FOLDER) rmdir /s /q $(BUILD_FOLDER)
	@del /Q $(TARGET)

# Include dependency files
-include $(OBJ:.o=.d)
CXX      := g++
CXXFLAGS := -Iimgui -Iimgui/backends -Iimplot
LDFLAGS  := -luser32 -lgdi32 -ld3d11 -ldxgi -ld3dcompiler -ldwmapi

# Sources
IMGUI_SRC := imgui/imgui.cpp \
             imgui/imgui_draw.cpp \
             imgui/imgui_widgets.cpp \
             imgui/imgui_tables.cpp \
             imgui/backends/imgui_impl_win32.cpp \
             imgui/backends/imgui_impl_dx11.cpp \
             implot/implot.cpp \
             implot/implot_items.cpp

APP_SRC   := main.cpp ui.cpp
SRC       := $(APP_SRC) $(IMGUI_SRC)

# Objects
OBJ       := $(SRC:.cpp=.o)

# Output
TARGET    := app.exe

all: $(TARGET)

# Link step
$(TARGET): $(OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

# Compile step
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	del /Q $(OBJ) $(TARGET) 2>nul || true
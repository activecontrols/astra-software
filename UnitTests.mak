CXX      := g++
CC       := gcc
CXXFLAGS := $(addprefix -I, $(filter-out lib/README,$(wildcard lib/*/))) -Itest/native_arduino_mock/include -Iinclude -Idata_postprocessing/eigen -MMD -MP

# the majority of this is just windows libraries
# the -static is because we were pulling in the wrong C++ .dlls (from the RUST ESP32 project), resulting in crazy crashes
# LDFLAGS  := -luser32 -lgdi32 -ld3d11 -ldxgi -ld3dcompiler -ldwmapi -lole32 -luuid -lshlwapi -lsetupapi -static

# Sources

SRC           := main.cpp run_tests.cpp $(wildcard test/native_arduino_mock/src/*.cpp)
LIB_SRC       := $(filter-out lib/logging/%, $(wildcard lib/*/*.cpp))
LIB_C_SRC     := $(wildcard lib/IMU/Invn/*/*.c)  $(wildcard lib/IMU/Invn/Drivers/*/*.c)
LIB_FOLDERS   := $(sort $(dir $(LIB_SRC))) $(sort $(dir $(LIB_C_SRC))) # removes dups
BUILD_FOLDER  := test/build
BUILD_FOLDERS := $(BUILD_FOLDER) $(addprefix $(BUILD_FOLDER)/,$(LIB_FOLDERS))

# Objects
OBJ := $(patsubst %.cpp,$(BUILD_FOLDER)/%.o,$(notdir $(SRC))) $(patsubst %.cpp,$(BUILD_FOLDER)/%.o,$(LIB_SRC)) $(patsubst %.c,$(BUILD_FOLDER)/%.o,$(LIB_C_SRC))

# Output
TARGET := app.exe

all: $(TARGET)

# Link step
$(TARGET): $(OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BUILD_FOLDERS):
	mkdir "$@"

# Compile step
$(BUILD_FOLDER)/%.o: src/%.cpp | $(BUILD_FOLDERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_FOLDER)/%.o: test/native_arduino_mock/src/%.cpp | $(BUILD_FOLDERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_FOLDER)/%.o: test/%.cpp | $(BUILD_FOLDERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_FOLDER)/%.o: %.cpp | $(BUILD_FOLDERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_FOLDER)/%.o: %.c | $(BUILD_FOLDERS)
	$(CC) $(CXXFLAGS) -c $< -o $@


clean:
	@if exist $(BUILD_FOLDER) rmdir /s /q $(BUILD_FOLDER)
	@del /Q $(TARGET)

# Include dependency files
-include $(OBJ:.o=.d)
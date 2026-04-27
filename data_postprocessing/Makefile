CXX      := g++
CXXFLAGS := -I../lib/trajectory_following/ -I../lib/controller/ -I../lib/data/ -I../lib/prop -I../include -I./ -Ieigen/ -MMD -MP
LDFLAGS  := 

VPATH     := ../lib/controller ../lib/prop
APP_SRC   := controller_state_reconstruction.cpp $(wildcard ../lib/controller/*.cpp) ../lib/prop/RollControl.cpp
SRC       := $(APP_SRC)
BUILD_FOLDER := build

# Objects
OBJ := $(patsubst %.cpp,$(BUILD_FOLDER)/%.o,$(notdir $(SRC)))

# Output
TARGET    := app.exe

all: $(TARGET)

# Link step
$(TARGET): $(OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BUILD_FOLDER):
	mkdir $(BUILD_FOLDER)

# Compile step
$(BUILD_FOLDER)/%.o: %.cpp | $(BUILD_FOLDER)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@if exist $(BUILD_FOLDER) rmdir /s /q $(BUILD_FOLDER)
	@del /Q $(TARGET)

# Include dependency files
-include $(OBJ:.o=.d)
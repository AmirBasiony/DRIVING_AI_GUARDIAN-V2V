CC := gcc 
CXX := g++
PYTHON := python3
LD := ld
AR := ar

# Directories
WORK_DIR := $(PWD)
SRC_DIR := $(WORK_DIR)
BUILD_DIR := $(WORK_DIR)
LIB_DIR := $(WORK_DIR)/lib

# C/C++ source files
C_SOURCES := $(wildcard $(SRC_DIR)/*.c) \
             $(wildcard $(SRC_DIR)/Algo/*.c) \
             $(wildcard $(SRC_DIR)/ipcMsg/*.c) \
             $(wildcard $(SRC_DIR)/connections/*.c) \
             $(wildcard $(SRC_DIR)/Threads/*.c) \
             $(wildcard $(SRC_DIR)/HAL/*.c) \
             $(wildcard $(SRC_DIR)/action/*.c) \
             $(wildcard $(SRC_DIR)/MQTT/*.c)
             
CPP_SOURCES := $(wildcard $(SRC_DIR)/*.cpp) \
               $(wildcard $(SRC_DIR)/Algo/*.cpp)

# Object files
C_OBJECTS := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(C_SOURCES))
CPP_OBJECTS := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(CPP_SOURCES))

# Python source files
PY_SOURCES := $(wildcard $(SRC_DIR)/*.py)

# Include paths
INCLUDES := -I$(WORK_DIR) \
            -I$(WORK_DIR)/INC \
            -I$(WORK_DIR)/Infrastructure \
            -I$(WORK_DIR)/ipcMsg \
            -I$(WORK_DIR)/connections \
            -I$(WORK_DIR)/Algo \
            -I$(WORK_DIR)/Threads \
            -I$(WORK_DIR)/action \
            -I$(WORK_DIR)/MQTT \
            -I/usr/local/include  # Add this line for Paho MQTT include path

# Compiler flags
CFLAGS := -Wall -Wextra $(INCLUDES) -DUSE_SYSCALL
CXXFLAGS := $(CFLAGS)

# Linker flags
LDFLAGS := -L/usr/local/lib -lpaho-mqtt3c -lm -lrt
# Output executable name
TARGET := Calculation_Process

# Targets and rules
all: $(BUILD_DIR)/$(TARGET)
	@echo "\n -----------> BUILD COMPLETED :) <----------- \n"

$(BUILD_DIR)/$(TARGET): $(C_OBJECTS) $(CPP_OBJECTS)
	$(CC) $(C_OBJECTS) $(CPP_OBJECTS) $(LDFLAGS) -o $@

# $(BUILD_DIR)/$(TARGET): $(C_OBJECTS) $(CPP_OBJECTS)
# 	$(CC) $(LDFLAGS) $^ -o $@ -L$(LIB_DIR) -lm

# Compile C source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

# Compile C++ source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Python files do not need compilation
$(BUILD_DIR)/%.py: $(SRC_DIR)/%.py
	cp $< $@

# Clean individual object files
clean:
	rm -rf $(C_OBJECTS) $(CPP_OBJECTS) $(TARGET)
	@echo "\n -----------> Deleting all object files COMPLETED :) <----------- \n"

.PHONY: all clean




# Makefile for Vocabs2 - Velocity Obstacle for Drones Simulator

# Compiler and flags
CC := g++
CFLAGS := -std=c++23 -Wall -Wextra -O3 -finline-functions -fopenmp -mavx2 -mfma
LDFLAGS := -lm

# Source files
SRC_DIR := src
SRC_FILES := \
	$(SRC_DIR)/main.cpp \
	$(SRC_DIR)/drone.cpp \
	$(SRC_DIR)/flightplan.cpp \
	$(SRC_DIR)/comms.cpp \
	$(SRC_DIR)/math2d.cpp \
	$(SRC_DIR)/math3d.cpp \
	$(SRC_DIR)/file_system.cpp

# Build directory
BUILD_DIR := build
TARGET := $(BUILD_DIR)/vocabs2

# Test source files
TEST_SRC_FILES := \
	$(SRC_DIR)/test.cpp \
	$(SRC_DIR)/test_vec2.cpp \
	$(SRC_DIR)/test_flightplan.cpp

TEST_TARGET := $(BUILD_DIR)/vocabs2_test

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SRC_FILES)
	@echo "Building Vocabs2..."
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	@rm -f *.d  # Clean up dependency files after build

$(TEST_TARGET): $(TEST_SRC_FILES) $(SRC_FILES)
	@echo "Building Vocabs2 tests..."
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -DTEST $^ $(LDFLAGS) -o $@

clean:
	@echo "Cleaning build files..."
	rm -rf $(BUILD_DIR)
	rm -f *.d

cleandeps:
	@echo "Cleaning dependency files..."
	rm -f *.d

run: $(TARGET)
	@echo "Running Vocabs2..."
	./$(TARGET) A 0.0 0.0 20.0

test: $(TEST_TARGET)
	@echo "Running tests..."
	./$(TEST_TARGET)

# Individual test targets
test_vec2: $(TEST_TARGET)
	@echo "Running vec2 tests..."
	./$(TEST_TARGET) --vec2

test_flightplan: $(TEST_TARGET)
	@echo "Running flightplan tests..."
	./$(TEST_TARGET) --flightplan

# Debug build
debug: CFLAGS += -g -O0 -DDEBUG
debug: all

# Release build (optimized)
release: CFLAGS := -std=c23 -O3 -finline-functions -fopenmp
release: all

# Dependency tracking (disabled for now to avoid .d file generation)
# -include $(SRC_FILES:.c=.d)

# %.d: %.c
# 	@$(CC) -M $(CFLAGS) $< > $@.tmp
# 	@mv -f $@.tmp $@
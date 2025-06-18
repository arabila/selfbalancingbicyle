# Simple Makefile for Little Bicycle V2 Controller
# Based on Jonah Zander's Bachelor Thesis (2023)

# Webots paths for macOS
WEBOTS_HOME = /Applications/Webots.app/Contents
WEBOTS_CONTROLLER_LIB = $(WEBOTS_HOME)/lib/controller
WEBOTS_INCLUDE = $(WEBOTS_HOME)/include/controller/c

# Compiler settings
CC = gcc
CFLAGS = -Wall -O2 -I$(WEBOTS_INCLUDE)
LDFLAGS = -L$(WEBOTS_CONTROLLER_LIB) -lController -lm

# Target and source
TARGET = little_bicycle_P_V2
SOURCES = little_bicycle_P_V2.c

# Check if Webots is installed
$(info Checking Webots installation...)
ifeq ($(wildcard $(WEBOTS_HOME)),)
    $(error Webots not found at $(WEBOTS_HOME))
endif

# Default target
all: $(TARGET)

# Build the controller
$(TARGET): $(SOURCES)
	@echo "Building controller: $(TARGET)"
	@echo "Using Webots at: $(WEBOTS_HOME)"
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) $(LDFLAGS)
	@echo "Build complete!"

# Clean build files
clean:
	rm -f $(TARGET) $(TARGET).exe *.o

# Debug info
info:
	@echo "=== Build Configuration ==="
	@echo "WEBOTS_HOME: $(WEBOTS_HOME)"
	@echo "WEBOTS_INCLUDE: $(WEBOTS_INCLUDE)"
	@echo "WEBOTS_CONTROLLER_LIB: $(WEBOTS_CONTROLLER_LIB)"
	@echo "CC: $(CC)"
	@echo "CFLAGS: $(CFLAGS)"
	@echo "LDFLAGS: $(LDFLAGS)"
	@echo "TARGET: $(TARGET)"
	@echo "SOURCES: $(SOURCES)"

# Test if libraries are available
test:
	@echo "Testing Webots libraries..."
	@ls -la $(WEBOTS_INCLUDE)/webots/ 2>/dev/null || echo "Headers not found"
	@ls -la $(WEBOTS_CONTROLLER_LIB)/ 2>/dev/null || echo "Libraries not found"

.PHONY: all clean info test 
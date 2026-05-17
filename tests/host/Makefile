.POSIX:

# Host self-test for the Fusion savestate Phase 1 protocol skeleton.
# Builds with the system compiler; no ESP-IDF, no FreeRTOS dependency.

CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra -Wpedantic -Wshadow \
           -Wno-unused-parameter -Werror

ROOT    := ../..
MAIN    := $(ROOT)/main
CRC     := $(ROOT)/components/crc

INCLUDES := -I$(MAIN) -I$(CRC)

SRCS := \
  $(MAIN)/fusion_savestate.c \
  $(MAIN)/fpga_state_port.c \
  $(MAIN)/savestate_storage.c \
  $(MAIN)/fusion_pathe_pkg.c \
  $(CRC)/crc8_sae_j1850.c \
  fusion_savestate_selftest.c \
  fusion_pathe_pkg_selftest.c

BUILD := build
BIN   := $(BUILD)/fusion_savestate_selftest

all: $(BIN)

$(BIN): $(SRCS) | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRCS) -o $(BIN)

$(BUILD):
	mkdir -p $(BUILD)

run: $(BIN)
	$(BIN)

clean:
	rm -rf $(BUILD)

.PHONY: all run clean

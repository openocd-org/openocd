# SPDX-License-Identifier: GPL-2.0-or-later

# Espressif Xtensa Makefile to compile flasher stub wrapper
# Copyright (C) 2023 Espressif Systems Ltd.

# Prefix for Espressif xtensa cross compilers (can include a directory path)
CROSS ?= xtensa-esp32-elf-

APP_ARCH := xtensa
APP_CHIP_PATH := $(shell pwd)
SRCS := $(APP_CHIP_PATH)/esp_xtensa_stub_tramp_win.S

BIN2C = ../../../../../src/helper/bin2char.sh
BUILD_DIR = build

APP = esp_xtensa_stub_tramp_win
APP_OBJ = $(BUILD_DIR)/$(APP).o
APP_BIN = $(BUILD_DIR)/$(APP).bin
APP_CODE = $(APP).inc

.PHONY: all clean

all: $(BUILD_DIR) $(APP_OBJ) $(APP_CODE)

$(BUILD_DIR):
	$(Q) mkdir $@

$(APP_OBJ): $(SRCS)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc -c $(CFLAGS)  -o $@ $^

$(APP_CODE): $(APP_OBJ)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)objcopy -O binary -j.text $^ $(APP_BIN)
	$(Q) $(BIN2C) < $(APP_BIN) > $@

clean:
	$(Q) rm -rf $(BUILD_DIR)

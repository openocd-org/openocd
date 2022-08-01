# SPDX-License-Identifier: GPL-2.0-or-later

# ESP32 Makefile to compile the SoC reset program
# Copyright (C) 2022 Espressif Systems Ltd.

# Pass V=1 to see the commands being executed by make
ifneq ("$(V)","1")
Q = @
endif

BIN2C = ../../../../../src/helper/bin2char.sh

APP = cpu_reset_handler

BUILD_DIR = build

APP_OBJ = $(BUILD_DIR)/$(APP).o
APP_BIN = $(BUILD_DIR)/$(APP)_code.bin
APP_CODE = $(APP)_code.inc

CFLAGS += -mtext-section-literals

.PHONY: all cleanxten

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

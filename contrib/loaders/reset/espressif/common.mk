# ESP32 Makefile to compile the SoC reset program
# Copyright (C) 2022 Espressif Systems Ltd.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

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

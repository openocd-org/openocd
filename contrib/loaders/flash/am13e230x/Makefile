# SPDX-License-Identifier: GPL-2.0-or-later

# Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/

BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-
GCC     = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

FLAGS  = -mcpu=cortex-m33 -mthumb -Os
FLAGS += -ffunction-sections -fdata-sections
FLAGS += -g -gdwarf-3 -gstrict-dwarf
FLAGS += -Wall -fno-strict-aliasing -fno-builtin
FLAGS += -nostdlib -nostartfiles

CFLAGS = -c -I.

OBJS := \
	startup.o \
	main.o \
	flash.o

all: am13e230x_algo.inc

%.o: %.c
	@echo 'Building: $<'
	$(GCC) $(FLAGS) $(CFLAGS) -o "$@" "$<"

am13e230x_algo.out: $(OBJS)
	@echo 'Linking: $@'
	$(GCC) $(FLAGS) -o $@ $(OBJS) -Wl,-T"am13e230x.lds"

%.bin: %.out
	@echo 'Objcopy: $@'
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	@echo 'Bin2Char: $@'
	$(BIN2C) < $< > $@
	rm $< $*.out

clean:
	rm -rf *.o *.d *.out *.bin *.inc *.map

.PRECIOUS: %.bin

.PHONY: all clean

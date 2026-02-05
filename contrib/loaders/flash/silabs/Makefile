# SPDX-License-Identifier: GPL-2.0-or-later

BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump


AFLAGS = -static -nostartfiles -mlittle-endian -Wa,-EL
CFLAGS = -c -mthumb -nostdlib -nostartfiles -Os -g -fPIC

all: silabs_s0_s1.inc silabs_s2.inc

.PHONY: clean

%.elf: %.S
	$(CC) $(AFLAGS) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc

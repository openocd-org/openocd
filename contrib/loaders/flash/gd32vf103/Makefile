BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= riscv-none-embed-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS = -march=rv32i -mabi=ilp32 -static -nostartfiles -nostdlib -Os -g -fPIC

all: gd32vf103.inc

.PHONY: clean

%.elf: %.c
	$(CC) $(CFLAGS) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc

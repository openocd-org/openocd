BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS = -static -nostartfiles -mlittle-endian -Wa,-EL

all: erase.inc write.inc

.PHONY: clean

.INTERMEDIATE: erase.elf write.elf

erase.elf write.elf: fm4.h

%.elf: %.S
	$(CC) $(CFLAGS) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc

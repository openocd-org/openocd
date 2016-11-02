BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS = -static -nostartfiles -mlittle-endian -Wa,-EL
LDFLAGS = -Tdebug_handler.ld

all: debug_handler.inc

.PHONY: clean

.INTERMEDIATE: debug_handler.elf

debug_handler.elf: protocol.h

%.elf: %.S
	$(CC) $(CFLAGS) $(LDFLAGS) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc

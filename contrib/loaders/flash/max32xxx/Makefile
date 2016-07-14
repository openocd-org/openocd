BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-
AS      = $(CROSS_COMPILE)as
OBJCOPY = $(CROSS_COMPILE)objcopy

all: max32xxx.inc

%.elf: %.s
	$(AS) $< -o $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.bin *.inc

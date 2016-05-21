BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-
AS      = $(CROSS_COMPILE)as
OBJCOPY = $(CROSS_COMPILE)objcopy

AFLAGS = -EL

all: kinetis_ke_flash.inc kinetis_ke_watchdog.inc

%.elf: %.s
	$(AS) $(AFLAGS) $< -o $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.bin *.inc

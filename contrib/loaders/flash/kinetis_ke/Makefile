BIN2C = ../../../../src/helper/bin2char.sh

ARM_CROSS_COMPILE ?= arm-none-eabi-
ARM_AS      ?= $(ARM_CROSS_COMPILE)as
ARM_OBJCOPY ?= $(ARM_CROSS_COMPILE)objcopy

arm: kinetis_ke_flash.inc kinetis_ke_watchdog.inc

kinetis_ke_%.elf: kinetis_ke_%.s
	$(ARM_AS) $< -o $@

kinetis_ke_%.bin: kinetis_ke_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

kinetis_ke_%.inc: kinetis_ke_%.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.bin *.inc

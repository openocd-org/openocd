BIN2C = ../../../src/helper/bin2char.sh

ARM_CROSS_COMPILE ?= arm-none-eabi-
ARM_AS      ?= $(ARM_CROSS_COMPILE)as
ARM_OBJCOPY ?= $(ARM_CROSS_COMPILE)objcopy

arm: armv7m_erase_check.inc armv7m_0_erase_check.inc

armv7m_%.elf: armv7m_%.s
	$(ARM_AS) $< -o $@

armv7m_%.bin: armv7m_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

armv7m_%.inc: armv7m_%.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.bin *.inc

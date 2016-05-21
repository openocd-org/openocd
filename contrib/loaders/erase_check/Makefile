BIN2C = ../../../src/helper/bin2char.sh

ARM_CROSS_COMPILE ?= arm-none-eabi-
ARM_AS      ?= $(ARM_CROSS_COMPILE)as
ARM_OBJCOPY ?= $(ARM_CROSS_COMPILE)objcopy

ARM_AFLAGS = -EL

arm: armv4_5_erase_check.inc armv7m_erase_check.inc armv7m_0_erase_check.inc

armv4_5_%.elf: armv4_5_%.s
	$(ARM_AS) $(ARM_AFLAGS) $< -o $@

armv4_5_%.bin: armv4_5_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

armv4_5_%.inc: armv4_5_%.bin
	$(BIN2C) < $< > $@

armv7m_%.elf: armv7m_%.s
	$(ARM_AS) $(ARM_AFLAGS) $< -o $@

armv7m_%.bin: armv7m_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

armv7m_%.inc: armv7m_%.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.bin *.inc

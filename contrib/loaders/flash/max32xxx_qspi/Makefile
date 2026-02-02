# SPDX-License-Identifier: GPL-2.0-or-later

TARGET=max32xxx_qspi
ENTRY=algo_write
OFFSET=0x20005000
BIN2C = ../../../../src/helper/bin2char.sh

%.o: %.s
	arm-none-eabi-as -mthumb -mcpu=cortex-m4 ${<} -o ${@}

%.o: %.c
	arm-none-eabi-gcc -mthumb -mcpu=cortex-m4 -Wa,-mimplicit-it=thumb -Os -Wall -c ${<} -o ${@}

%.bin: %.elf
	arm-none-eabi-objcopy -O binary ${<} ${@}

%.inc: %.bin
	$(BIN2C) < $< > $@

all: $(TARGET).inc

dump: $(TARGET).o
	arm-none-eabi-objdump -d -s ${<}

$(TARGET).elf: $(TARGET).o
	arm-none-eabi-ld -nostdlib -T max32xxx_qspi.ld ${<} -o ${@}
	arm-none-eabi-size ${@}

clean:
	rm -rf $(TARGET).bin $(TARGET).elf $(TARGET).o $(TARGET).inc

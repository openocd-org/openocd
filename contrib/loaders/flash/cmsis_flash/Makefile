BIN2C = ../../../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS =  -c -mthumb -mcpu=cortex-m0plus -Os

all: program_page.inc

.PHONY: clean

.INTERMEDIATE: program_page.o

program_page.o: program_page.c
	$(CC) $(CFLAGS) -DMXS40_VARIANT=MXS40_VARIANT_PSOC6A_2M -Wall -Wextra -Wa,-adhln=$*.lst $< -o $@

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.lst *.bin *.inc

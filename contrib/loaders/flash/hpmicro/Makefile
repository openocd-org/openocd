# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2023 HPMicro
#
BIN2C = ../../../../src/helper/bin2char.sh


PROJECT=hpm_xpi_flash
CROSS_COMPILE ?= riscv32-unknown-elf-
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump
LD=$(CROSS_COMPILE)ld
LDSCRIPT=linker.ld

OPT=-O3

ASFLAGS=
CFLAGS=$(OPT) -fomit-frame-pointer -Wall
LDFLAGS=-nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(PROJECT).map -static -Wl,--gc-sections
OBJS=$(ASRC:.S=.o) $(SRC:.c=.o)

SRC=openocd_flash_algo.c
ASRC=func_table.S

all: $(OBJS) $(PROJECT).elf $(PROJECT).bin $(PROJECT).lst $(PROJECT).inc

%o: %c
	@$(CC) -c $(CFLAGS) -I . $< -o $@

%o: %S
	@$(AS) -c $(ASFLAGS) -I . $< -o $@

%elf: $(OBJS)
	@$(CC) $(OBJS) $(LDFLAGS) -o $@

%lst: %elf
	@$(OBJDUMP) -h -S $< > $@

%bin: %elf
	@$(OBJCOPY) -Obinary $< $@

%inc: %bin
	$(BIN2C) < $< > $@

clean:
	@-rm -f *.o *.elf *.lst *.bin *.inc

.PHONY:	all clean

.INTERMEDIATE: $(patsubst %.S,%.o,$(SRCS)) $(patsubst %.S,%.elf,$(SRCS)) $(patsubst %.S,%.bin,$(SRCS))

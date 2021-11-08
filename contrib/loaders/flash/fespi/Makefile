BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= riscv64-unknown-elf-

RISCV_CC=$(CROSS_COMPILE)gcc
RISCV_OBJCOPY=$(CROSS_COMPILE)objcopy
RISCV_OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS = -nostdlib -nostartfiles -Wall -Werror -Os -fPIC -Wunused-result -g
RISCV32_CFLAGS = -march=rv32e -mabi=ilp32e $(CFLAGS)
RISCV64_CFLAGS = -march=rv64i -mabi=lp64 $(CFLAGS)

all: riscv32_fespi.inc riscv64_fespi.inc

.PHONY: clean

# .c -> .o
riscv32_%.o:  riscv_%.c
	$(RISCV_CC) -c $(RISCV32_CFLAGS) $^ -o $@

riscv64_%.o:  riscv_%.c
	$(RISCV_CC) -c $(RISCV64_CFLAGS) $< -o $@

# .S -> .o
riscv32_%.o:  riscv_%.S
	$(RISCV_CC) -c $(RISCV32_CFLAGS) $^ -o $@

riscv64_%.o:  riscv_%.S
	$(RISCV_CC) -c $(RISCV64_CFLAGS) $^ -o $@

# .o -> .elf
riscv32_%.elf:	riscv32_%.o riscv32_wrapper.o
	$(RISCV_CC) -T riscv.lds $(RISCV32_CFLAGS) $^ -o $@

riscv64_%.elf:	riscv64_%.o riscv64_wrapper.o
	$(RISCV_CC) -T riscv.lds $(RISCV64_CFLAGS) $^ -o $@

# .elf -> .bin
%.bin: %.elf
	$(RISCV_OBJCOPY) -Obinary $< $@

# .bin -> .inc
%.inc: %.bin
	$(BIN2C) < $< > $@

# utility
%.lst: %.elf
	$(RISCV_OBJDUMP) -S $< > $@

clean:
	-rm -f *.elf *.o *.lst *.bin *.inc

BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-
GCC     = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

FLAGS  = -mthumb -Os -ffunction-sections -fdata-sections -g -gdwarf-3
FLAGS += -gstrict-dwarf -Wall -fno-strict-aliasing --asm

CFLAGS = -c -I.

CC26X0_CFLAGS  = -mcpu=cortex-m3 -DDEVICE_CC26X0

CC26X2_CFLAGS  = -mcpu=cortex-m4 -DDEVICE_CC26X2

CC26X0_OBJS := \
cc26x0/flashloader.o \
cc26x0/main.o \
cc26x0/startup.o \
cc26x0/flash.o

CC26X2_OBJS := \
cc26x2/flashloader.o \
cc26x2/main.o \
cc26x2/startup.o \
cc26x2/flash.o

all: cc26x0_algo.inc cc26x2_algo.inc

cc26x0/%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	$(GCC) $(FLAGS) $(CFLAGS) $(CC26X0_CFLAGS) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

cc26x2/%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	$(GCC) $(FLAGS) $(CFLAGS) $(CC26X2_CFLAGS) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

cc26x0_algo.out: $(CC26X0_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	$(GCC) $(FLAGS) -o$@ $(CC26X0_OBJS) -Wl,-T"cc26x0/cc26x0r2f.lds"
	@echo 'Finished building target: $@'
	@echo ' '

cc26x2_algo.out: $(CC26X2_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	$(GCC) $(FLAGS) -o$@ $(CC26X2_OBJS) -Wl,-T"cc26x2/cc26x2r1f.lds"
	@echo 'Finished building target: $@'
	@echo ' '

%.bin: %.out
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Objcopy Utility'
	$(OBJCOPY) -Obinary $< $@
	@echo 'Finished building target: $@'
	@echo ' '

%.inc: %.bin
	@echo 'Building target: $@'
	@echo 'Invoking Bin2Char Script'
	$(BIN2C) < $< > $@
	rm $< $*.out
	@echo 'Finished building target: $@'
	@echo ' '

clean:
	@echo 'Cleaning Targets and Build Artifacts'
	rm -rf *.inc *.bin *.out *.map
	rm -rf cc26x0/*.o cc26x0/*.d
	rm -rf cc26x2/*.o cc26x2/*.d
	@echo 'Finished clean'
	@echo ' '

.PRECIOUS: %.bin

.PHONY: all clean

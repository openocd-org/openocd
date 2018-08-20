BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-
GCC     = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

FLAGS = -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb

CFLAGS  = -c -DNO_MSP_CLASSIC_DEFINES -Dgcc -Wall -ffunction-sections
CFLAGS += -fdata-sections -std=c99 -O4

LDFLAGS = -lc -lnosys -Wl,--gc-sections

MSP432E4X_OBJS := \
msp432e4x/driverlib.o \
msp432e4x/main_msp432e4x.o \
msp432e4x/startup_msp432e4.o

MSP432P401X_OBJS := \
msp432p401x/driverlib.o \
msp432p401x/main_msp432p401x.o \
msp432p401x/startup_msp432p4.o

MSP432P411X_OBJS := \
msp432p411x/driverlib.o \
msp432p411x/main_msp432p411x.o \
msp432p411x/startup_msp432p4.o

all: msp432e4x_algo.inc msp432p401x_algo.inc msp432p411x_algo.inc

msp432e4x/%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	$(GCC) -D__MSP432E4X__ $(FLAGS) $(CFLAGS) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

msp432p401x/%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	$(GCC) -D__MSP432P401X__ $(FLAGS) $(CFLAGS) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

msp432p411x/%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	$(GCC) -D__MSP432P411X__ $(FLAGS) $(CFLAGS) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

msp432e4x_algo.out: $(MSP432E4X_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	$(GCC) $(FLAGS) $(LDFLAGS) -o$@ $(MSP432E4X_OBJS) -Tmsp432e4x/msp432e4x.lds
	@echo 'Finished building target: $@'
	@echo ' '

msp432p401x_algo.out: $(MSP432P401X_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	$(GCC) $(FLAGS) $(LDFLAGS) -o$@ $(MSP432P401X_OBJS) -Tmsp432p401x/msp432p401x.lds
	@echo 'Finished building target: $@'
	@echo ' '

msp432p411x_algo.out: $(MSP432P411X_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	$(GCC) $(FLAGS) $(LDFLAGS) -o$@ $(MSP432P411X_OBJS) -Tmsp432p411x/msp432p411x.lds
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
	rm -rf msp432e4x/*.o msp432e4x/*.d
	rm -rf msp432p401x/*.o msp432p401x/*.d
	rm -rf msp432p411x/*.o msp432p411x/*.d
	@echo 'Finished clean'
	@echo ' '

.PRECIOUS: %.bin

.PHONY: all clean

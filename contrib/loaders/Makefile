.PHONY: arm clean-arm

all: arm stm8

common_dirs = \
	checksum \
	erase_check \
	watchdog

ARM_CROSS_COMPILE ?= arm-none-eabi-

arm_dirs = \
	flash/fm4 \
	flash/kinetis_ke \
	flash/max32xxx \
	flash/xmc1xxx \
	debug/xscale

arm:
	for d in $(common_dirs); do \
		$(MAKE) -C $$d arm; \
	done
	for d in $(arm_dirs); do \
		$(MAKE) -C $$d all CROSS_COMPILE=$(ARM_CROSS_COMPILE); \
	done

clean-arm:
	for d in $(arm_dirs); do \
		$(MAKE) -C $$d clean; \
	done

clean: clean-arm
	for d in $(common_dirs); do \
		$(MAKE) -C $$d clean; \
	done

stm8:
	$(MAKE) -C erase_check stm8

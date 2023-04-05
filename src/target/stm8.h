/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
*   OpenOCD STM8 target driver
*   Copyright (C) 2017  Ake Rehnman
*   ake.rehnman(at)gmail.com
*/

#ifndef OPENOCD_TARGET_STM8_H
#define OPENOCD_TARGET_STM8_H

struct target;

#define STM8_COMMON_MAGIC	0x53544D38U
#define STM8_NUM_CORE_REGS 6

struct stm8_common {
	unsigned int common_magic;

	void *arch_info;
	struct reg_cache *core_cache;
	uint32_t core_regs[STM8_NUM_CORE_REGS];

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	bool swim_configured;
	bool bp_scanned;
	uint8_t num_hw_bpoints;
	uint8_t num_hw_bpoints_avail;
	struct stm8_comparator *hw_break_list;
	uint32_t blocksize;
	uint32_t flashstart;
	uint32_t flashend;
	uint32_t eepromstart;
	uint32_t eepromend;
	uint32_t optionstart;
	uint32_t optionend;
	bool enable_step_irq;

	bool enable_stm8l;
	uint32_t flash_cr2;
	uint32_t flash_ncr2;
	uint32_t flash_iapsr;
	uint32_t flash_dukr;
	uint32_t flash_pukr;

	/* cc value used for interrupt flags restore */
	uint32_t cc;
	bool cc_valid;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned int num);
	int (*write_core_reg)(struct target *target, unsigned int num);
};

static inline struct stm8_common *
target_to_stm8(struct target *target)
{
	return target->arch_info;
}

#endif /* OPENOCD_TARGET_STM8_H */

/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009-2011 by Mathias Kuester                            *
 *   mkdorg@users.sourceforge.net                                          *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_DSP563XX_H
#define OPENOCD_TARGET_DSP563XX_H

#include <jtag/jtag.h>
#include <target/dsp563xx_once.h>

#define DSP563XX_NUMCOREREGS	54
#define DSP563XX_NUMONCEREGS	25

struct mcu_jtag {
	struct jtag_tap *tap;
};

enum breakpoint_usage {
	BPU_NONE = 0,
	BPU_BREAKPOINT,
	BPU_WATCHPOINT
};

struct hardware_breakpoint {
	enum breakpoint_usage used;
};

struct dsp563xx_common {
	struct mcu_jtag jtag_info;
	struct reg_cache *core_cache;
	uint32_t core_regs[DSP563XX_NUMCOREREGS];
	struct once_reg once_regs[DSP563XX_NUMONCEREGS];

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, int num);
	int (*write_core_reg)(struct target *target, int num);

	struct hardware_breakpoint hardware_breakpoint[1];

	/*Were the hardware breakpoints cleared on startup?*/
	bool hardware_breakpoints_cleared;
};

struct dsp563xx_core_reg {
	uint32_t num;
	const char *name;
	uint32_t size;
	uint8_t eame;
	uint32_t instr_mask;
	struct target *target;
	struct dsp563xx_common *dsp563xx_common;
};

static inline struct dsp563xx_common *target_to_dsp563xx(struct target *target)
{
	return target->arch_info;
}

#endif /* OPENOCD_TARGET_DSP563XX_H */

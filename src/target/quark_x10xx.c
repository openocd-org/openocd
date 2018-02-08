/*
 * Copyright(c) 2013-2016 Intel Corporation.
 *
 * Adrian Burns (adrian.burns@intel.com)
 * Thomas Faust (thomas.faust@intel.com)
 * Ivan De Cesaris (ivan.de.cesaris@intel.com)
 * Julien Carreno (julien.carreno@intel.com)
 * Jeffrey Maxwell (jeffrey.r.maxwell@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information:
 * Intel Corporation
 */

/*
 * @file
 * Debugger for Intel Quark SoC X1000
 * Intel Quark X10xx is the first product in the Quark family of SoCs.
 * It is an IA-32 (Pentium x86 ISA) compatible SoC. The core CPU in the
 * X10xx is codenamed Lakemont. Lakemont version 1 (LMT1) is used in X10xx.
 * The CPU TAP (Lakemont TAP) is used for software debug and the CLTAP is
 * used for SoC level operations.
 * Useful docs are here: https://communities.intel.com/community/makers/documentation
 * Intel Quark SoC X1000 OpenOCD/GDB/Eclipse App Note (web search for doc num 330015)
 * Intel Quark SoC X1000 Debug Operations User Guide (web search for doc num 329866)
 * Intel Quark SoC X1000 Datasheet (web search for doc num 329676)
 *
 * This file implements any Quark SoC specific features such as resetbreak (TODO)
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>

#include "target.h"
#include "target_type.h"
#include "lakemont.h"
#include "x86_32_common.h"

static int quark_x10xx_target_create(struct target *t, Jim_Interp *interp)
{
	struct x86_32_common *x86_32 = calloc(1, sizeof(*x86_32));

	if (!x86_32)
		return ERROR_FAIL;

	x86_32_common_init_arch_info(t, x86_32);
	lakemont_init_arch_info(t, x86_32);
	x86_32->core_type = LMT1;

	return ERROR_OK;
}

struct target_type quark_x10xx_target = {
	.name			= "quark_x10xx",

	/* Quark X1000 SoC */
	.target_create		= quark_x10xx_target_create,

	/* lakemont probemode specific code */
	.arch_state		= lakemont_arch_state,
	.assert_reset		= lakemont_reset_assert,
	.deassert_reset		= lakemont_reset_deassert,
	.halt			= lakemont_halt,
	.init_target		= lakemont_init_target,
	.poll			= lakemont_poll,
	.resume			= lakemont_resume,
	.step			= lakemont_step,

	/* common x86 code */
	.add_breakpoint		= x86_32_common_add_breakpoint,
	.add_watchpoint		= x86_32_common_add_watchpoint,
	.commands		= x86_32_command_handlers,
	.get_gdb_reg_list	= x86_32_get_gdb_reg_list,
	.mmu			= x86_32_common_mmu,
	.read_memory		= x86_32_common_read_memory,
	.read_phys_memory	= x86_32_common_read_phys_mem,
	.remove_breakpoint	= x86_32_common_remove_breakpoint,
	.remove_watchpoint	= x86_32_common_remove_watchpoint,
	.virt2phys		= x86_32_common_virt2phys,
	.write_memory		= x86_32_common_write_memory,
	.write_phys_memory	= x86_32_common_write_phys_mem,
};

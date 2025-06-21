// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *                                                                         *
 *   Cortex-M3(tm) TRM, ARM DDI 0337E (r1p1) and 0337G (r2p0)              *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "breakpoints.h"
#include "cortex_m.h"
#include "target_request.h"
#include "target_type.h"
#include "arm_adi_v5.h"
#include "arm_disassembler.h"
#include "register.h"
#include "arm_opcodes.h"
#include "arm_semihosting.h"
#include "smp.h"
#include <helper/nvp.h>
#include <helper/time_support.h>
#include <rtt/rtt.h>

/* NOTE:  most of this should work fine for the Cortex-M1 and
 * Cortex-M0 cores too, although they're ARMv6-M not ARMv7-M.
 * Some differences:  M0/M1 doesn't have FPB remapping or the
 * DWT tracing/profiling support.  (So the cycle counter will
 * not be usable; the other stuff isn't currently used here.)
 *
 * Although there are some workarounds for errata seen only in r0p0
 * silicon, such old parts are hard to find and thus not much tested
 * any longer.
 */

/* Timeout for register r/w */
#define DHCSR_S_REGRDY_TIMEOUT (500)

/* Supported Cortex-M Cores */
static const struct cortex_m_part_info cortex_m_parts[] = {
	{
		.impl_part = CORTEX_M0_PARTNO,
		.name = "Cortex-M0",
		.arch = ARM_ARCH_V6M,
	},
	{
		.impl_part = CORTEX_M0P_PARTNO,
		.name = "Cortex-M0+",
		.arch = ARM_ARCH_V6M,
	},
	{
		.impl_part = CORTEX_M1_PARTNO,
		.name = "Cortex-M1",
		.arch = ARM_ARCH_V6M,
	},
	{
		.impl_part = CORTEX_M3_PARTNO,
		.name = "Cortex-M3",
		.arch = ARM_ARCH_V7M,
		.flags = CORTEX_M_F_TAR_AUTOINCR_BLOCK_4K,
	},
	{
		.impl_part = CORTEX_M4_PARTNO,
		.name = "Cortex-M4",
		.arch = ARM_ARCH_V7M,
		.flags = CORTEX_M_F_HAS_FPV4 | CORTEX_M_F_TAR_AUTOINCR_BLOCK_4K,
	},
	{
		.impl_part = CORTEX_M7_PARTNO,
		.name = "Cortex-M7",
		.arch = ARM_ARCH_V7M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
	{
		.impl_part = CORTEX_M23_PARTNO,
		.name = "Cortex-M23",
		.arch = ARM_ARCH_V8M,
	},
	{
		.impl_part = CORTEX_M33_PARTNO,
		.name = "Cortex-M33",
		.arch = ARM_ARCH_V8M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
	{
		.impl_part = CORTEX_M35P_PARTNO,
		.name = "Cortex-M35P",
		.arch = ARM_ARCH_V8M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
	{
		.impl_part = CORTEX_M52_PARTNO,
		.name = "Cortex-M52",
		.arch = ARM_ARCH_V8M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
	{
		.impl_part = CORTEX_M55_PARTNO,
		.name = "Cortex-M55",
		.arch = ARM_ARCH_V8M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
	{
		.impl_part = CORTEX_M85_PARTNO,
		.name = "Cortex-M85",
		.arch = ARM_ARCH_V8M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
	{
		.impl_part = STAR_MC1_PARTNO,
		.name = "STAR-MC1",
		.arch = ARM_ARCH_V8M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
	{
		.impl_part = INFINEON_SLX2_PARTNO,
		.name = "Infineon-SLx2",
		.arch = ARM_ARCH_V8M,
	},
	{
		.impl_part = REALTEK_M200_PARTNO,
		.name = "Real-M200 (KM0)",
		.arch = ARM_ARCH_V8M,
	},
	{
		.impl_part = REALTEK_M300_PARTNO,
		.name = "Real-M300 (KM4)",
		.arch = ARM_ARCH_V8M,
		.flags = CORTEX_M_F_HAS_FPV5,
	},
};

/* forward declarations */
static int cortex_m_store_core_reg_u32(struct target *target,
		uint32_t num, uint32_t value);
static void cortex_m_dwt_free(struct target *target);

/** DCB DHCSR register contains S_RETIRE_ST and S_RESET_ST bits cleared
 *  on a read. Call this helper function each time DHCSR is read
 *  to preserve S_RESET_ST state in case of a reset event was detected.
 */
static inline void cortex_m_cumulate_dhcsr_sticky(struct cortex_m_common *cortex_m,
		uint32_t dhcsr)
{
	cortex_m->dcb_dhcsr_cumulated_sticky |= dhcsr;
}

/** Read DCB DHCSR register to cortex_m->dcb_dhcsr and cumulate
 * sticky bits in cortex_m->dcb_dhcsr_cumulated_sticky
 */
static int cortex_m_read_dhcsr_atomic_sticky(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);

	int retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DHCSR,
				&cortex_m->dcb_dhcsr);
	if (retval != ERROR_OK)
		return retval;

	cortex_m_cumulate_dhcsr_sticky(cortex_m, cortex_m->dcb_dhcsr);
	return ERROR_OK;
}

static int cortex_m_load_core_reg_u32(struct target *target,
		uint32_t regsel, uint32_t *value)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int retval;
	uint32_t dcrdr, tmp_value;
	int64_t then;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we have to save/restore the DCB_DCRDR when used */
	if (target->dbg_msg_enabled) {
		retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DCRDR, &dcrdr);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRSR, regsel);
	if (retval != ERROR_OK)
		return retval;

	/* check if value from register is ready and pre-read it */
	then = timeval_ms();
	while (1) {
		retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DHCSR,
								 &cortex_m->dcb_dhcsr);
		if (retval != ERROR_OK)
			return retval;
		retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DCRDR,
										&tmp_value);
		if (retval != ERROR_OK)
			return retval;
		cortex_m_cumulate_dhcsr_sticky(cortex_m, cortex_m->dcb_dhcsr);
		if (cortex_m->dcb_dhcsr & S_REGRDY)
			break;
		cortex_m->slow_register_read = true; /* Polling (still) needed. */
		if (timeval_ms() > then + DHCSR_S_REGRDY_TIMEOUT) {
			LOG_TARGET_ERROR(target, "Timeout waiting for DCRDR transfer ready");
			return ERROR_TIMEOUT_REACHED;
		}
		keep_alive();
	}

	*value = tmp_value;

	if (target->dbg_msg_enabled) {
		/* restore DCB_DCRDR - this needs to be in a separate
		 * transaction otherwise the emulated DCC channel breaks */
		if (retval == ERROR_OK)
			retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DCRDR, dcrdr);
	}

	return retval;
}

static int cortex_m_slow_read_all_regs(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	const unsigned int num_regs = armv7m->arm.core_cache->num_regs;

	/* Opportunistically restore fast read, it'll revert to slow
	 * if any register needed polling in cortex_m_load_core_reg_u32(). */
	cortex_m->slow_register_read = false;

	for (unsigned int reg_id = 0; reg_id < num_regs; reg_id++) {
		struct reg *r = &armv7m->arm.core_cache->reg_list[reg_id];
		if (r->exist) {
			int retval = armv7m->arm.read_core_reg(target, r, reg_id, ARM_MODE_ANY);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	if (!cortex_m->slow_register_read)
		LOG_TARGET_DEBUG(target, "Switching back to fast register reads");

	return ERROR_OK;
}

static int cortex_m_queue_reg_read(struct target *target, uint32_t regsel,
		uint32_t *reg_value, uint32_t *dhcsr)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int retval;

	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRSR, regsel);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DHCSR, dhcsr);
	if (retval != ERROR_OK)
		return retval;

	return mem_ap_read_u32(armv7m->debug_ap, DCB_DCRDR, reg_value);
}

static int cortex_m_fast_read_all_regs(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int retval;
	uint32_t dcrdr;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we have to save/restore the DCB_DCRDR when used */
	bool dbg_msg_enabled = target->dbg_msg_enabled;
	if (dbg_msg_enabled) {
		retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DCRDR, &dcrdr);
		if (retval != ERROR_OK)
			return retval;
	}

	const unsigned int num_regs = armv7m->arm.core_cache->num_regs;
	const unsigned int n_r32 = ARMV7M_LAST_REG - ARMV7M_CORE_FIRST_REG + 1
							   + ARMV7M_FPU_LAST_REG - ARMV7M_FPU_FIRST_REG + 1;
	/* we need one 32-bit word for each register except FP D0..D15, which
	 * need two words */
	uint32_t r_vals[n_r32];
	uint32_t dhcsr[n_r32];

	unsigned int wi = 0; /* write index to r_vals and dhcsr arrays */
	unsigned int reg_id; /* register index in the reg_list, ARMV7M_R0... */
	for (reg_id = 0; reg_id < num_regs; reg_id++) {
		struct reg *r = &armv7m->arm.core_cache->reg_list[reg_id];
		if (!r->exist)
			continue;	/* skip non existent registers */

		if (r->size <= 8) {
			/* Any 8-bit or shorter register is unpacked from a 32-bit
			 * container register. Skip it now. */
			continue;
		}

		uint32_t regsel = armv7m_map_id_to_regsel(reg_id);
		retval = cortex_m_queue_reg_read(target, regsel, &r_vals[wi],
										 &dhcsr[wi]);
		if (retval != ERROR_OK)
			return retval;
		wi++;

		assert(r->size == 32 || r->size == 64);
		if (r->size == 32)
			continue;	/* done with 32-bit register */

		assert(reg_id >= ARMV7M_FPU_FIRST_REG && reg_id <= ARMV7M_FPU_LAST_REG);
		/* the odd part of FP register (S1, S3...) */
		retval = cortex_m_queue_reg_read(target, regsel + 1, &r_vals[wi],
											 &dhcsr[wi]);
		if (retval != ERROR_OK)
			return retval;
		wi++;
	}

	assert(wi <= n_r32);

	retval = dap_run(armv7m->debug_ap->dap);
	if (retval != ERROR_OK)
		return retval;

	if (dbg_msg_enabled) {
		/* restore DCB_DCRDR - this needs to be in a separate
		 * transaction otherwise the emulated DCC channel breaks */
		retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DCRDR, dcrdr);
		if (retval != ERROR_OK)
			return retval;
	}

	bool not_ready = false;
	for (unsigned int i = 0; i < wi; i++) {
		if ((dhcsr[i] & S_REGRDY) == 0) {
			not_ready = true;
			LOG_TARGET_DEBUG(target, "Register %u was not ready during fast read", i);
		}
		cortex_m_cumulate_dhcsr_sticky(cortex_m, dhcsr[i]);
	}

	if (not_ready) {
		/* Any register was not ready,
		 * fall back to slow read with S_REGRDY polling */
		return ERROR_TIMEOUT_REACHED;
	}

	LOG_TARGET_DEBUG(target, "read %u 32-bit registers", wi);

	unsigned int ri = 0; /* read index from r_vals array */
	for (reg_id = 0; reg_id < num_regs; reg_id++) {
		struct reg *r = &armv7m->arm.core_cache->reg_list[reg_id];
		if (!r->exist)
			continue;	/* skip non existent registers */

		r->dirty = false;

		unsigned int reg32_id;
		uint32_t offset;
		if (armv7m_map_reg_packing(reg_id, &reg32_id, &offset)) {
			/* Unpack a partial register from 32-bit container register */
			struct reg *r32 = &armv7m->arm.core_cache->reg_list[reg32_id];

			/* The container register ought to precede all regs unpacked
			 * from it in the reg_list. So the value should be ready
			 * to unpack */
			assert(r32->valid);
			buf_cpy(r32->value + offset, r->value, r->size);

		} else {
			assert(r->size == 32 || r->size == 64);
			buf_set_u32(r->value, 0, 32, r_vals[ri++]);

			if (r->size == 64) {
				assert(reg_id >= ARMV7M_FPU_FIRST_REG && reg_id <= ARMV7M_FPU_LAST_REG);
				/* the odd part of FP register (S1, S3...) */
				buf_set_u32(r->value + 4, 0, 32, r_vals[ri++]);
			}
		}
		r->valid = true;
	}
	assert(ri == wi);

	return retval;
}

static int cortex_m_store_core_reg_u32(struct target *target,
		uint32_t regsel, uint32_t value)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int retval;
	uint32_t dcrdr;
	int64_t then;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we have to save/restore the DCB_DCRDR when used */
	if (target->dbg_msg_enabled) {
		retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DCRDR, &dcrdr);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRDR, value);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRSR, regsel | DCRSR_WNR);
	if (retval != ERROR_OK)
		return retval;

	/* check if value is written into register */
	then = timeval_ms();
	while (1) {
		retval = cortex_m_read_dhcsr_atomic_sticky(target);
		if (retval != ERROR_OK)
			return retval;
		if (cortex_m->dcb_dhcsr & S_REGRDY)
			break;
		if (timeval_ms() > then + DHCSR_S_REGRDY_TIMEOUT) {
			LOG_TARGET_ERROR(target, "Timeout waiting for DCRDR transfer ready");
			return ERROR_TIMEOUT_REACHED;
		}
		keep_alive();
	}

	if (target->dbg_msg_enabled) {
		/* restore DCB_DCRDR - this needs to be in a separate
		 * transaction otherwise the emulated DCC channel breaks */
		if (retval == ERROR_OK)
			retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DCRDR, dcrdr);
	}

	return retval;
}

static int cortex_m_write_debug_halt_mask(struct target *target,
	uint32_t mask_on, uint32_t mask_off)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;

	/* mask off status bits */
	cortex_m->dcb_dhcsr &= ~((0xFFFFul << 16) | mask_off);
	/* create new register mask */
	cortex_m->dcb_dhcsr |= DBGKEY | C_DEBUGEN | mask_on;

	return mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DHCSR, cortex_m->dcb_dhcsr);
}

static int cortex_m_set_maskints(struct target *target, bool mask)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	if (!!(cortex_m->dcb_dhcsr & C_MASKINTS) != mask)
		return cortex_m_write_debug_halt_mask(target, mask ? C_MASKINTS : 0, mask ? 0 : C_MASKINTS);
	else
		return ERROR_OK;
}

static int cortex_m_set_maskints_for_halt(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	switch (cortex_m->isrmasking_mode) {
		case CORTEX_M_ISRMASK_AUTO:
			/* interrupts taken at resume, whether for step or run -> no mask */
			return cortex_m_set_maskints(target, false);

		case CORTEX_M_ISRMASK_OFF:
			/* interrupts never masked */
			return cortex_m_set_maskints(target, false);

		case CORTEX_M_ISRMASK_ON:
			/* interrupts always masked */
			return cortex_m_set_maskints(target, true);

		case CORTEX_M_ISRMASK_STEPONLY:
			/* interrupts masked for single step only -> mask now if MASKINTS
			 * erratum, otherwise only mask before stepping */
			return cortex_m_set_maskints(target, cortex_m->maskints_erratum);
	}
	return ERROR_OK;
}

static int cortex_m_set_maskints_for_run(struct target *target)
{
	switch (target_to_cm(target)->isrmasking_mode) {
		case CORTEX_M_ISRMASK_AUTO:
			/* interrupts taken at resume, whether for step or run -> no mask */
			return cortex_m_set_maskints(target, false);

		case CORTEX_M_ISRMASK_OFF:
			/* interrupts never masked */
			return cortex_m_set_maskints(target, false);

		case CORTEX_M_ISRMASK_ON:
			/* interrupts always masked */
			return cortex_m_set_maskints(target, true);

		case CORTEX_M_ISRMASK_STEPONLY:
			/* interrupts masked for single step only -> no mask */
			return cortex_m_set_maskints(target, false);
	}
	return ERROR_OK;
}

static int cortex_m_set_maskints_for_step(struct target *target)
{
	switch (target_to_cm(target)->isrmasking_mode) {
		case CORTEX_M_ISRMASK_AUTO:
			/* the auto-interrupt should already be done -> mask */
			return cortex_m_set_maskints(target, true);

		case CORTEX_M_ISRMASK_OFF:
			/* interrupts never masked */
			return cortex_m_set_maskints(target, false);

		case CORTEX_M_ISRMASK_ON:
			/* interrupts always masked */
			return cortex_m_set_maskints(target, true);

		case CORTEX_M_ISRMASK_STEPONLY:
			/* interrupts masked for single step only -> mask */
			return cortex_m_set_maskints(target, true);
	}
	return ERROR_OK;
}

static int cortex_m_clear_halt(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	int retval;

	/* clear step if any */
	cortex_m_write_debug_halt_mask(target, C_HALT, C_STEP);

	/* Read Debug Fault Status Register */
	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_DFSR, &cortex_m->nvic_dfsr);
	if (retval != ERROR_OK)
		return retval;

	/* Clear Debug Fault Status */
	retval = mem_ap_write_atomic_u32(armv7m->debug_ap, NVIC_DFSR, cortex_m->nvic_dfsr);
	if (retval != ERROR_OK)
		return retval;
	LOG_TARGET_DEBUG(target, "NVIC_DFSR 0x%" PRIx32 "", cortex_m->nvic_dfsr);

	return ERROR_OK;
}

static int cortex_m_single_step_core(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	int retval;

	/* Mask interrupts before clearing halt, if not done already.  This avoids
	 * Erratum 377497 (fixed in r1p0) where setting MASKINTS while clearing
	 * HALT can put the core into an unknown state.
	 */
	if (!(cortex_m->dcb_dhcsr & C_MASKINTS)) {
		retval = cortex_m_write_debug_halt_mask(target, C_MASKINTS, 0);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = cortex_m_write_debug_halt_mask(target, C_STEP, C_HALT);
	if (retval != ERROR_OK)
		return retval;
	LOG_TARGET_DEBUG(target, "single step");

	/* restore dhcsr reg */
	cortex_m_clear_halt(target);

	return ERROR_OK;
}

static int cortex_m_enable_fpb(struct target *target)
{
	int retval = target_write_u32(target, FP_CTRL, 3);
	if (retval != ERROR_OK)
		return retval;

	/* check the fpb is actually enabled */
	uint32_t fpctrl;
	retval = target_read_u32(target, FP_CTRL, &fpctrl);
	if (retval != ERROR_OK)
		return retval;

	if (fpctrl & 1)
		return ERROR_OK;

	return ERROR_FAIL;
}

static int cortex_m_endreset_event(struct target *target)
{
	int retval;
	uint32_t dcb_demcr;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	struct adiv5_dap *swjdp = cortex_m->armv7m.arm.dap;
	struct cortex_m_fp_comparator *fp_list = cortex_m->fp_comparator_list;
	struct cortex_m_dwt_comparator *dwt_list = cortex_m->dwt_comparator_list;

	/* REVISIT The four debug monitor bits are currently ignored... */
	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DEMCR, &dcb_demcr);
	if (retval != ERROR_OK)
		return retval;
	LOG_TARGET_DEBUG(target, "DCB_DEMCR = 0x%8.8" PRIx32 "", dcb_demcr);

	/* this register is used for emulated dcc channel */
	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DCRDR, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = cortex_m_read_dhcsr_atomic_sticky(target);
	if (retval != ERROR_OK)
		return retval;

	if (!(cortex_m->dcb_dhcsr & C_DEBUGEN)) {
		/* Enable debug requests */
		retval = cortex_m_write_debug_halt_mask(target, 0, C_HALT | C_STEP | C_MASKINTS);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Restore proper interrupt masking setting for running CPU. */
	cortex_m_set_maskints_for_run(target);

	/* Enable features controlled by ITM and DWT blocks, and catch only
	 * the vectors we were told to pay attention to.
	 *
	 * Target firmware is responsible for all fault handling policy
	 * choices *EXCEPT* explicitly scripted overrides like "vector_catch"
	 * or manual updates to the NVIC SHCSR and CCR registers.
	 */
	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DEMCR, TRCENA | armv7m->demcr);
	if (retval != ERROR_OK)
		return retval;

	/* Paranoia: evidently some (early?) chips don't preserve all the
	 * debug state (including FPB, DWT, etc) across reset...
	 */

	/* Enable FPB */
	retval = cortex_m_enable_fpb(target);
	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to enable the FPB");
		return retval;
	}

	cortex_m->fpb_enabled = true;

	/* Restore FPB registers */
	for (unsigned int i = 0; i < cortex_m->fp_num_code + cortex_m->fp_num_lit; i++) {
		retval = target_write_u32(target, fp_list[i].fpcr_address, fp_list[i].fpcr_value);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Restore DWT registers */
	for (unsigned int i = 0; i < cortex_m->dwt_num_comp; i++) {
		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 0,
				dwt_list[i].comp);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 4,
				dwt_list[i].mask);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 8,
				dwt_list[i].function);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = dap_run(swjdp);
	if (retval != ERROR_OK)
		return retval;

	register_cache_invalidate(armv7m->arm.core_cache);

	/* TODO: invalidate also working areas (needed in the case of detected reset).
	 * Doing so will require flash drivers to test if working area
	 * is still valid in all target algo calling loops.
	 */

	/* make sure we have latest dhcsr flags */
	retval = cortex_m_read_dhcsr_atomic_sticky(target);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int cortex_m_examine_debug_reason(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);

	/* THIS IS NOT GOOD, TODO - better logic for detection of debug state reason
	 * only check the debug reason if we don't know it already */

	if ((target->debug_reason != DBG_REASON_DBGRQ)
		&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {
		if (cortex_m->nvic_dfsr & DFSR_BKPT) {
			target->debug_reason = DBG_REASON_BREAKPOINT;
			if (cortex_m->nvic_dfsr & DFSR_DWTTRAP)
				target->debug_reason = DBG_REASON_WPTANDBKPT;
		} else if (cortex_m->nvic_dfsr & DFSR_DWTTRAP)
			target->debug_reason = DBG_REASON_WATCHPOINT;
		else if (cortex_m->nvic_dfsr & DFSR_VCATCH)
			target->debug_reason = DBG_REASON_BREAKPOINT;
		else if (cortex_m->nvic_dfsr & DFSR_EXTERNAL)
			target->debug_reason = DBG_REASON_DBGRQ;
		else	/* HALTED */
			target->debug_reason = DBG_REASON_UNDEFINED;
	}

	return ERROR_OK;
}

static int cortex_m_examine_exception_reason(struct target *target)
{
	uint32_t shcsr = 0, except_sr = 0, cfsr = -1, except_ar = -1;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	int retval;

	retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_SHCSR, &shcsr);
	if (retval != ERROR_OK)
		return retval;
	switch (armv7m->exception_number) {
		case 2:	/* NMI */
			break;
		case 3:	/* Hard Fault */
			retval = mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_HFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			if (except_sr & 0x40000000) {
				retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &cfsr);
				if (retval != ERROR_OK)
					return retval;
			}
			break;
		case 4:	/* Memory Management */
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_MMFAR, &except_ar);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 5:	/* Bus Fault */
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_BFAR, &except_ar);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 6:	/* Usage Fault */
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_CFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 7:	/* Secure Fault */
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_SFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_SFAR, &except_ar);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 11:	/* SVCall */
			break;
		case 12:	/* Debug Monitor */
			retval = mem_ap_read_u32(armv7m->debug_ap, NVIC_DFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 14:	/* PendSV */
			break;
		case 15:	/* SysTick */
			break;
		default:
			except_sr = 0;
			break;
	}
	retval = dap_run(swjdp);
	if (retval == ERROR_OK)
		LOG_TARGET_DEBUG(target, "%s SHCSR 0x%" PRIx32 ", SR 0x%" PRIx32
			", CFSR 0x%" PRIx32 ", AR 0x%" PRIx32,
			armv7m_exception_string(armv7m->exception_number),
			shcsr, except_sr, cfsr, except_ar);
	return retval;
}

/* Errata 3092511 workaround
 * Cortex-M7 can halt in an incorrect address when breakpoint
 * and exception occurs simultaneously */
static int cortex_m_erratum_check_breakpoint(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	struct arm *arm = &armv7m->arm;

	uint32_t pc = buf_get_u32(arm->pc->value, 0, 32);

	/* To reduce the workaround processing cost we assume FPB is in sync
	 * with OpenOCD breakpoints. If the target app writes to FPB
	 * OpenOCD will resume after the break set by app */
	struct breakpoint *bkpt = breakpoint_find(target, pc);
	if (bkpt) {
		LOG_TARGET_DEBUG(target, "Erratum 3092511: breakpoint confirmed");
		return ERROR_OK;
	}
	if (pc >= 0xe0000000u)
		/* not executable area, do not read instruction @ pc */
		return ERROR_OK;

	uint16_t insn;
	int retval = target_read_u16(target, pc, &insn);
	if (retval != ERROR_OK)
		return ERROR_OK;	/* do not propagate the error, just avoid workaround */

	if ((insn & 0xff00) == (ARMV5_T_BKPT(0) & 0xff00)) {
		LOG_TARGET_DEBUG(target, "Erratum 3092511: breakpoint embedded in code confirmed");
		return ERROR_OK;
	}
	LOG_TARGET_DEBUG(target, "Erratum 3092511: breakpoint not found, proceed with resume");
	return ERROR_TARGET_HALTED_DO_RESUME;
}

static int cortex_m_debug_entry(struct target *target)
{
	uint32_t xpsr;
	int retval;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	struct arm *arm = &armv7m->arm;
	struct reg *r;

	LOG_TARGET_DEBUG(target, " ");

	/* Do this really early to minimize the window where the MASKINTS erratum
	 * can pile up pending interrupts. */
	cortex_m_set_maskints_for_halt(target);

	cortex_m_clear_halt(target);

	retval = cortex_m_read_dhcsr_atomic_sticky(target);
	if (retval != ERROR_OK)
		return retval;

	retval = armv7m->examine_debug_reason(target);
	if (retval != ERROR_OK)
		return retval;

	/* examine PE security state */
	uint32_t dscsr = 0;
	if (armv7m->arm.arch == ARM_ARCH_V8M) {
		retval = mem_ap_read_u32(armv7m->debug_ap, DCB_DSCSR, &dscsr);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Load all registers to arm.core_cache */
	if (!cortex_m->slow_register_read) {
		retval = cortex_m_fast_read_all_regs(target);
		if (retval == ERROR_TIMEOUT_REACHED) {
			cortex_m->slow_register_read = true;
			LOG_TARGET_DEBUG(target, "Switched to slow register read");
		}
	}

	if (cortex_m->slow_register_read)
		retval = cortex_m_slow_read_all_regs(target);

	if (retval != ERROR_OK)
		return retval;

	r = arm->cpsr;
	xpsr = buf_get_u32(r->value, 0, 32);

	/* Are we in an exception handler */
	if (xpsr & 0x1FF) {
		armv7m->exception_number = (xpsr & 0x1FF);

		arm->core_mode = ARM_MODE_HANDLER;
		arm->map = armv7m_msp_reg_map;
	} else {
		unsigned int control = buf_get_u32(arm->core_cache
				->reg_list[ARMV7M_CONTROL].value, 0, 3);

		/* is this thread privileged? */
		arm->core_mode = control & 1
			? ARM_MODE_USER_THREAD
			: ARM_MODE_THREAD;

		/* which stack is it using? */
		if (control & 2)
			arm->map = armv7m_psp_reg_map;
		else
			arm->map = armv7m_msp_reg_map;

		armv7m->exception_number = 0;
	}

	if (armv7m->exception_number)
		cortex_m_examine_exception_reason(target);

	bool secure_state = (dscsr & DSCSR_CDS) == DSCSR_CDS;
	LOG_TARGET_DEBUG(target, "entered debug state in core mode: %s at PC 0x%" PRIx32
			", cpu in %s state, target->state: %s",
		arm_mode_name(arm->core_mode),
		buf_get_u32(arm->pc->value, 0, 32),
		secure_state ? "Secure" : "Non-Secure",
		target_state_name(target));

	/* Errata 3092511 workaround
	 * Cortex-M7 can halt in an incorrect address when breakpoint
	 * and exception occurs simultaneously */
	if (cortex_m->incorrect_halt_erratum
			&& armv7m->exception_number
			&& cortex_m->nvic_dfsr == (DFSR_BKPT | DFSR_HALTED)) {
		retval = cortex_m_erratum_check_breakpoint(target);
		if (retval != ERROR_OK)
			return retval;
	}

	if (armv7m->post_debug_entry) {
		retval = armv7m->post_debug_entry(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int cortex_m_poll_one(struct target *target)
{
	int detected_failure = ERROR_OK;
	int retval = ERROR_OK;
	enum target_state prev_target_state = target->state;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;

	/* Read from Debug Halting Control and Status Register */
	retval = cortex_m_read_dhcsr_atomic_sticky(target);
	if (retval != ERROR_OK) {
		target->state = TARGET_UNKNOWN;
		return retval;
	}

	/* Recover from lockup.  See ARMv7-M architecture spec,
	 * section B1.5.15 "Unrecoverable exception cases".
	 */
	if (cortex_m->dcb_dhcsr & S_LOCKUP) {
		LOG_TARGET_ERROR(target, "clearing lockup after double fault");
		cortex_m_write_debug_halt_mask(target, C_HALT, 0);
		target->debug_reason = DBG_REASON_DBGRQ;

		/* We have to execute the rest (the "finally" equivalent, but
		 * still throw this exception again).
		 */
		detected_failure = ERROR_FAIL;

		/* refresh status bits */
		retval = cortex_m_read_dhcsr_atomic_sticky(target);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cortex_m->dcb_dhcsr_cumulated_sticky & S_RESET_ST) {
		cortex_m->dcb_dhcsr_cumulated_sticky &= ~S_RESET_ST;
		if (target->state != TARGET_RESET) {
			target->state = TARGET_RESET;
			LOG_TARGET_INFO(target, "external reset detected");
			/* In case of an unexpected S_RESET_ST set TARGET_RESET state
			 * and keep it until the next poll to allow its detection */
			return ERROR_OK;
		}

		/* refresh status bits */
		retval = cortex_m_read_dhcsr_atomic_sticky(target);
		if (retval != ERROR_OK)
			return retval;

		/* If still under reset, quit and re-check at next poll */
		if (cortex_m->dcb_dhcsr_cumulated_sticky & S_RESET_ST) {
			cortex_m->dcb_dhcsr_cumulated_sticky &= ~S_RESET_ST;
			return ERROR_OK;
		}

		/* S_RESET_ST was expected (in a reset command). Continue processing
		 * to quickly get out of TARGET_RESET state */
	}

	if (target->state == TARGET_RESET) {
		/* Cannot switch context while running so endreset is
		 * called with target->state == TARGET_RESET
		 */
		LOG_TARGET_DEBUG(target, "Exit from reset with dcb_dhcsr 0x%" PRIx32,
			cortex_m->dcb_dhcsr);
		retval = cortex_m_endreset_event(target);
		if (retval != ERROR_OK) {
			target->state = TARGET_UNKNOWN;
			return retval;
		}
		target->state = TARGET_RUNNING;
		prev_target_state = TARGET_RUNNING;
	}

	if (cortex_m->dcb_dhcsr & S_HALT) {
		target->state = TARGET_HALTED;

		if ((prev_target_state == TARGET_RUNNING) || (prev_target_state == TARGET_RESET)) {
			retval = cortex_m_debug_entry(target);

			/* Errata 3092511 workaround
			 * Cortex-M7 can halt in an incorrect address when breakpoint
			 * and exception occurs simultaneously */
			if (retval == ERROR_TARGET_HALTED_DO_RESUME) {
				struct arm *arm = &armv7m->arm;
				LOG_TARGET_INFO(target, "Resuming after incorrect halt @ PC 0x%08" PRIx32
					", ARM Cortex-M7 erratum 3092511",
					buf_get_u32(arm->pc->value, 0, 32));
				/* We don't need to restore registers, just restart the core */
				cortex_m_set_maskints_for_run(target);
				retval = cortex_m_write_debug_halt_mask(target, 0, C_HALT);
				if (retval != ERROR_OK)
					return retval;

				target->debug_reason = DBG_REASON_NOTHALTED;
				/* registers are now invalid */
				register_cache_invalidate(armv7m->arm.core_cache);

				target->state = TARGET_RUNNING;
				return ERROR_OK;
			}

			/* arm_semihosting needs to know registers, don't run if debug entry returned error */
			if (retval == ERROR_OK && arm_semihosting(target, &retval) != 0)
				return retval;

			if (target->smp) {
				LOG_TARGET_DEBUG(target, "postpone target event 'halted'");
				target->smp_halt_event_postponed = true;
			} else {
				/* regardless of errors returned in previous code update state */
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
		}
		if (prev_target_state == TARGET_DEBUG_RUNNING) {
			retval = cortex_m_debug_entry(target);

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
		if (retval != ERROR_OK)
			return retval;
	}

	if (target->state == TARGET_UNKNOWN) {
		/* Check if processor is retiring instructions or sleeping.
		 * Unlike S_RESET_ST here we test if the target *is* running now,
		 * not if it has been running (possibly in the past). Instructions are
		 * typically processed much faster than OpenOCD polls DHCSR so S_RETIRE_ST
		 * is read always 1. That's the reason not to use dcb_dhcsr_cumulated_sticky.
		 */
		if (cortex_m->dcb_dhcsr & S_RETIRE_ST || cortex_m->dcb_dhcsr & S_SLEEP) {
			target->state = TARGET_RUNNING;
			retval = ERROR_OK;
		}
	}

	/* Check that target is truly halted, since the target could be resumed externally */
	if ((prev_target_state == TARGET_HALTED) && !(cortex_m->dcb_dhcsr & S_HALT)) {
		/* registers are now invalid */
		register_cache_invalidate(armv7m->arm.core_cache);

		target->state = TARGET_RUNNING;
		LOG_TARGET_WARNING(target, "external resume detected");
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		retval = ERROR_OK;
	}

	/* Did we detect a failure condition that we cleared? */
	if (detected_failure != ERROR_OK)
		retval = detected_failure;
	return retval;
}

static int cortex_m_halt_one(struct target *target);

static int cortex_m_smp_halt_all(struct list_head *smp_targets)
{
	int retval = ERROR_OK;
	struct target_list *head;

	foreach_smp_target(head, smp_targets) {
		struct target *curr = head->target;
		if (!target_was_examined(curr))
			continue;
		if (curr->state == TARGET_HALTED)
			continue;

		int ret2 = cortex_m_halt_one(curr);
		if (retval == ERROR_OK)
			retval = ret2;	/* store the first error code ignore others */
	}
	return retval;
}

static int cortex_m_smp_post_halt_poll(struct list_head *smp_targets)
{
	int retval = ERROR_OK;
	struct target_list *head;

	foreach_smp_target(head, smp_targets) {
		struct target *curr = head->target;
		if (!target_was_examined(curr))
			continue;
		/* skip targets that were already halted */
		if (curr->state == TARGET_HALTED)
			continue;

		int ret2 = cortex_m_poll_one(curr);
		if (retval == ERROR_OK)
			retval = ret2;	/* store the first error code ignore others */
	}
	return retval;
}

static int cortex_m_poll_smp(struct list_head *smp_targets)
{
	int retval = ERROR_OK;
	struct target_list *head;
	bool halted = false;

	foreach_smp_target(head, smp_targets) {
		struct target *curr = head->target;
		if (curr->smp_halt_event_postponed) {
			halted = true;
			break;
		}
	}

	if (halted) {
		retval = cortex_m_smp_halt_all(smp_targets);

		int ret2 = cortex_m_smp_post_halt_poll(smp_targets);
		if (retval == ERROR_OK)
			retval = ret2;	/* store the first error code ignore others */

		foreach_smp_target(head, smp_targets) {
			struct target *curr = head->target;
			if (!curr->smp_halt_event_postponed)
				continue;

			curr->smp_halt_event_postponed = false;
			if (curr->state == TARGET_HALTED) {
				LOG_TARGET_DEBUG(curr, "sending postponed target event 'halted'");
				target_call_event_callbacks(curr, TARGET_EVENT_HALTED);
			}
		}
		/* There is no need to set gdb_service->target
		 * as hwthread_update_threads() selects an interesting thread
		 * by its own
		 */
	}
	return retval;
}

static int cortex_m_poll(struct target *target)
{
	int retval = cortex_m_poll_one(target);

	if (target->smp) {
		struct target_list *last;
		last = list_last_entry(target->smp_targets, struct target_list, lh);
		if (target == last->target)
			/* After the last target in SMP group has been polled
			 * check for postponed halted events and eventually halt and re-poll
			 * other targets */
			cortex_m_poll_smp(target->smp_targets);
	}
	return retval;
}

static int cortex_m_halt_one(struct target *target)
{
	int retval;
	LOG_TARGET_DEBUG(target, "target->state: %s", target_state_name(target));

	if (!target_was_examined(target)) {
		LOG_TARGET_ERROR(target, "target non examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	if (target->state == TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_TARGET_WARNING(target, "target was in unknown state when halt was requested");

	/* Write to Debug Halting Control and Status Register */
	retval = cortex_m_write_debug_halt_mask(target, C_HALT, 0);

	/* Do this really early to minimize the window where the MASKINTS erratum
	 * can pile up pending interrupts. */
	cortex_m_set_maskints_for_halt(target);

	target->debug_reason = DBG_REASON_DBGRQ;

	return retval;
}

static int cortex_m_halt(struct target *target)
{
	if (target->smp)
		return cortex_m_smp_halt_all(target->smp_targets);
	else
		return cortex_m_halt_one(target);
}

static int cortex_m_soft_reset_halt(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	int retval, timeout = 0;

	/* on single cortex_m MCU soft_reset_halt should be avoided as same functionality
	 * can be obtained by using 'reset halt' and 'cortex_m reset_config vectreset'.
	 * As this reset only uses VC_CORERESET it would only ever reset the cortex_m
	 * core, not the peripherals */
	LOG_TARGET_DEBUG(target, "soft_reset_halt is discouraged, please use 'reset halt' instead.");

	if (!cortex_m->vectreset_supported) {
		LOG_TARGET_ERROR(target, "VECTRESET is not supported on this Cortex-M core");
		return ERROR_FAIL;
	}

	/* Set C_DEBUGEN */
	retval = cortex_m_write_debug_halt_mask(target, 0, C_STEP | C_MASKINTS);
	if (retval != ERROR_OK)
		return retval;

	/* Enter debug state on reset; restore DEMCR in endreset_event() */
	retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DEMCR,
			TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
	if (retval != ERROR_OK)
		return retval;

	/* Request a core-only reset */
	retval = mem_ap_write_atomic_u32(armv7m->debug_ap, NVIC_AIRCR,
			AIRCR_VECTKEY | AIRCR_VECTRESET);
	if (retval != ERROR_OK)
		return retval;
	target->state = TARGET_RESET;

	/* registers are now invalid */
	register_cache_invalidate(cortex_m->armv7m.arm.core_cache);

	while (timeout < 100) {
		retval = cortex_m_read_dhcsr_atomic_sticky(target);
		if (retval == ERROR_OK) {
			retval = mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_DFSR,
					&cortex_m->nvic_dfsr);
			if (retval != ERROR_OK)
				return retval;
			if ((cortex_m->dcb_dhcsr & S_HALT)
				&& (cortex_m->nvic_dfsr & DFSR_VCATCH)) {
				LOG_TARGET_DEBUG(target, "system reset-halted, DHCSR 0x%08" PRIx32 ", DFSR 0x%08" PRIx32,
						cortex_m->dcb_dhcsr, cortex_m->nvic_dfsr);
				cortex_m_poll(target);
				/* FIXME restore user's vector catch config */
				return ERROR_OK;
			} else {
				LOG_TARGET_DEBUG(target, "waiting for system reset-halt, "
					"DHCSR 0x%08" PRIx32 ", %d ms",
					cortex_m->dcb_dhcsr, timeout);
			}
		}
		timeout++;
		alive_sleep(1);
	}

	return ERROR_OK;
}

void cortex_m_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint) {
		if (!breakpoint->is_set)
			cortex_m_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

static int cortex_m_restore_one(struct target *target, bool current,
	target_addr_t *address, bool handle_breakpoints, bool debug_execution)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;
	struct reg *r;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);
		cortex_m_enable_breakpoints(target);
		cortex_m_enable_watchpoints(target);
	}

	if (debug_execution) {
		r = armv7m->arm.core_cache->reg_list + ARMV7M_PRIMASK;

		/* Disable interrupts */
		/* We disable interrupts in the PRIMASK register instead of
		 * masking with C_MASKINTS.  This is probably the same issue
		 * as Cortex-M3 Erratum 377493 (fixed in r1p0):  C_MASKINTS
		 * in parallel with disabled interrupts can cause local faults
		 * to not be taken.
		 *
		 * This breaks non-debug (application) execution if not
		 * called from armv7m_start_algorithm() which saves registers.
		 */
		buf_set_u32(r->value, 0, 1, 1);
		r->dirty = true;
		r->valid = true;

		/* Make sure we are in Thumb mode, set xPSR.T bit */
		/* armv7m_start_algorithm() initializes entire xPSR register.
		 * This duplicity handles the case when cortex_m_resume()
		 * is used with the debug_execution flag directly,
		 * not called through armv7m_start_algorithm().
		 */
		r = armv7m->arm.cpsr;
		buf_set_u32(r->value, 24, 1, 1);
		r->dirty = true;
		r->valid = true;
	}

	/* current = true: continue on current pc, otherwise continue at <address> */
	r = armv7m->arm.pc;
	if (!current) {
		buf_set_u32(r->value, 0, 32, *address);
		r->dirty = true;
		r->valid = true;
	}

	/* if we halted last time due to a bkpt instruction
	 * then we have to manually step over it, otherwise
	 * the core will break again */

	if (!breakpoint_find(target, buf_get_u32(r->value, 0, 32))
		&& !debug_execution)
		armv7m_maybe_skip_bkpt_inst(target, NULL);

	resume_pc = buf_get_u32(r->value, 0, 32);
	if (current)
		*address = resume_pc;

	int retval = armv7m_restore_context(target);
	if (retval != ERROR_OK)
		return retval;

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_TARGET_DEBUG(target, "unset breakpoint at " TARGET_ADDR_FMT " (ID: %" PRIu32 ")",
				breakpoint->address,
				breakpoint->unique_id);
			retval = cortex_m_unset_breakpoint(target, breakpoint);
			if (retval == ERROR_OK)
				retval = cortex_m_single_step_core(target);
			int ret2 = cortex_m_set_breakpoint(target, breakpoint);
			if (retval != ERROR_OK)
				return retval;
			if (ret2 != ERROR_OK)
				return ret2;
		}
	}

	return ERROR_OK;
}

static int cortex_m_restart_one(struct target *target, bool debug_execution)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);

	/* Restart core */
	cortex_m_set_maskints_for_run(target);
	cortex_m_write_debug_halt_mask(target, 0, C_HALT);

	target->debug_reason = DBG_REASON_NOTHALTED;
	/* registers are now invalid */
	register_cache_invalidate(armv7m->arm.core_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
	}

	return ERROR_OK;
}

static int cortex_m_restore_smp(struct target *target, bool handle_breakpoints)
{
	struct target_list *head;
	target_addr_t address;
	foreach_smp_target(head, target->smp_targets) {
		struct target *curr = head->target;
		/* skip calling target */
		if (curr == target)
			continue;
		if (!target_was_examined(curr))
			continue;
		/* skip running targets */
		if (curr->state == TARGET_RUNNING)
			continue;

		int retval = cortex_m_restore_one(curr, true, &address,
			handle_breakpoints, false);
		if (retval != ERROR_OK)
			return retval;

		retval = cortex_m_restart_one(curr, false);
		if (retval != ERROR_OK)
			return retval;

		LOG_TARGET_DEBUG(curr, "SMP resumed at " TARGET_ADDR_FMT, address);
	}
	return ERROR_OK;
}

static int cortex_m_resume(struct target *target, bool current,
	   target_addr_t address, bool handle_breakpoints, bool debug_execution)
{
	int retval = cortex_m_restore_one(target, current, &address,
		handle_breakpoints, debug_execution);
	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "context restore failed, aborting resume");
		return retval;
	}

	if (target->smp && !debug_execution) {
		retval = cortex_m_restore_smp(target, handle_breakpoints);
		if (retval != ERROR_OK)
			LOG_TARGET_WARNING(target, "resume of a SMP target failed, trying to resume current one");
	}

	cortex_m_restart_one(target, debug_execution);
	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "resume failed");
		return retval;
	}

	LOG_TARGET_DEBUG(target, "%sresumed at " TARGET_ADDR_FMT,
					debug_execution ? "debug " : "", address);

	return ERROR_OK;
}

/* int irqstepcount = 0; */
static int cortex_m_step(struct target *target, bool current,
		target_addr_t address, bool handle_breakpoints)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	struct breakpoint *breakpoint = NULL;
	struct reg *pc = armv7m->arm.pc;
	bool bkpt_inst_found = false;
	int retval;
	bool isr_timed_out = false;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Just one of SMP cores will step. Set the gdb control
	 * target to current one or gdb miss gdb-end event */
	if (target->smp && target->gdb_service)
		target->gdb_service->target = target;

	/* current = true: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u32(pc->value, 0, 32, address);
		pc->dirty = true;
		pc->valid = true;
	}

	uint32_t pc_value = buf_get_u32(pc->value, 0, 32);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, pc_value);
		if (breakpoint)
			cortex_m_unset_breakpoint(target, breakpoint);
	}

	armv7m_maybe_skip_bkpt_inst(target, &bkpt_inst_found);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	armv7m_restore_context(target);

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* if no bkpt instruction is found at pc then we can perform
	 * a normal step, otherwise we have to manually step over the bkpt
	 * instruction - as such simulate a step */
	if (!bkpt_inst_found) {
		if (cortex_m->isrmasking_mode != CORTEX_M_ISRMASK_AUTO) {
			/* Automatic ISR masking mode off: Just step over the next
			 * instruction, with interrupts on or off as appropriate. */
			cortex_m_set_maskints_for_step(target);
			cortex_m_write_debug_halt_mask(target, C_STEP, C_HALT);
		} else {
			/* Process interrupts during stepping in a way they don't interfere
			 * debugging.
			 *
			 * Principle:
			 *
			 * Set a temporary break point at the current pc and let the core run
			 * with interrupts enabled. Pending interrupts get served and we run
			 * into the breakpoint again afterwards. Then we step over the next
			 * instruction with interrupts disabled.
			 *
			 * If the pending interrupts don't complete within time, we leave the
			 * core running. This may happen if the interrupts trigger faster
			 * than the core can process them or the handler doesn't return.
			 *
			 * If no more breakpoints are available we simply do a step with
			 * interrupts enabled.
			 *
			 */

			/* 2012-09-29 ph
			 *
			 * If a break point is already set on the lower half word then a break point on
			 * the upper half word will not break again when the core is restarted. So we
			 * just step over the instruction with interrupts disabled.
			 *
			 * The documentation has no information about this, it was found by observation
			 * on STM32F1 and STM32F2. Proper explanation welcome. STM32F0 doesn't seem to
			 * suffer from this problem.
			 *
			 * To add some confusion: pc_value has bit 0 always set, while the breakpoint
			 * address has it always cleared. The former is done to indicate thumb mode
			 * to gdb.
			 *
			 */
			if ((pc_value & 0x02) && breakpoint_find(target, pc_value & ~0x03)) {
				LOG_TARGET_DEBUG(target, "Stepping over next instruction with interrupts disabled");
				cortex_m_write_debug_halt_mask(target, C_HALT | C_MASKINTS, 0);
				cortex_m_write_debug_halt_mask(target, C_STEP, C_HALT);
				/* Re-enable interrupts if appropriate */
				cortex_m_write_debug_halt_mask(target, C_HALT, 0);
				cortex_m_set_maskints_for_halt(target);
			} else {

				/* Set a temporary break point */
				if (breakpoint) {
					retval = cortex_m_set_breakpoint(target, breakpoint);
				} else {
					enum breakpoint_type type = BKPT_HARD;
					if (cortex_m->fp_rev == 0 && pc_value > 0x1FFFFFFF) {
						/* FPB rev.1 cannot handle such addr, try BKPT instr */
						type = BKPT_SOFT;
					}
					retval = breakpoint_add(target, pc_value, 2, type);
				}

				bool tmp_bp_set = (retval == ERROR_OK);

				/* No more breakpoints left, just do a step */
				if (!tmp_bp_set) {
					cortex_m_set_maskints_for_step(target);
					cortex_m_write_debug_halt_mask(target, C_STEP, C_HALT);
					/* Re-enable interrupts if appropriate */
					cortex_m_write_debug_halt_mask(target, C_HALT, 0);
					cortex_m_set_maskints_for_halt(target);
				} else {
					/* Start the core */
					LOG_TARGET_DEBUG(target, "Starting core to serve pending interrupts");
					int64_t t_start = timeval_ms();
					cortex_m_set_maskints_for_run(target);
					cortex_m_write_debug_halt_mask(target, 0, C_HALT | C_STEP);

					/* Wait for pending handlers to complete or timeout */
					do {
						retval = cortex_m_read_dhcsr_atomic_sticky(target);
						if (retval != ERROR_OK) {
							target->state = TARGET_UNKNOWN;
							return retval;
						}
						isr_timed_out = ((timeval_ms() - t_start) > 500);
					} while (!((cortex_m->dcb_dhcsr & S_HALT) || isr_timed_out));

					/* only remove breakpoint if we created it */
					if (breakpoint)
						cortex_m_unset_breakpoint(target, breakpoint);
					else {
						/* Remove the temporary breakpoint */
						breakpoint_remove(target, pc_value);
					}

					if (isr_timed_out) {
						LOG_TARGET_DEBUG(target, "Interrupt handlers didn't complete within time, "
							"leaving target running");
					} else {
						/* Step over next instruction with interrupts disabled */
						cortex_m_set_maskints_for_step(target);
						cortex_m_write_debug_halt_mask(target,
							C_HALT | C_MASKINTS,
							0);
						cortex_m_write_debug_halt_mask(target, C_STEP, C_HALT);
						/* Re-enable interrupts if appropriate */
						cortex_m_write_debug_halt_mask(target, C_HALT, 0);
						cortex_m_set_maskints_for_halt(target);
					}
				}
			}
		}
	}

	retval = cortex_m_read_dhcsr_atomic_sticky(target);
	if (retval != ERROR_OK)
		return retval;

	/* registers are now invalid */
	register_cache_invalidate(armv7m->arm.core_cache);

	if (breakpoint)
		cortex_m_set_breakpoint(target, breakpoint);

	if (isr_timed_out) {
		/* Leave the core running. The user has to stop execution manually. */
		target->debug_reason = DBG_REASON_NOTHALTED;
		target->state = TARGET_RUNNING;
		return ERROR_OK;
	}

	LOG_TARGET_DEBUG(target, "target stepped dcb_dhcsr = 0x%" PRIx32
		" nvic_icsr = 0x%" PRIx32,
		cortex_m->dcb_dhcsr, cortex_m->nvic_icsr);

	retval = cortex_m_debug_entry(target);
	if (retval != ERROR_OK && retval != ERROR_TARGET_HALTED_DO_RESUME)
		return retval;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	LOG_TARGET_DEBUG(target, "target stepped dcb_dhcsr = 0x%" PRIx32
		" nvic_icsr = 0x%" PRIx32,
		cortex_m->dcb_dhcsr, cortex_m->nvic_icsr);

	return ERROR_OK;
}

static int cortex_m_assert_reset(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	enum cortex_m_soft_reset_config reset_config = cortex_m->soft_reset_config;

	LOG_TARGET_DEBUG(target, "target->state: %s,%s examined",
		target_state_name(target),
		target_was_examined(target) ? "" : " not");

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		/* allow scripts to override the reset event */

		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
		register_cache_invalidate(cortex_m->armv7m.arm.core_cache);
		target->state = TARGET_RESET;

		return ERROR_OK;
	}

	/* some cores support connecting while srst is asserted
	 * use that mode if it has been configured */

	bool srst_asserted = false;

	if ((jtag_reset_config & RESET_HAS_SRST) &&
		((jtag_reset_config & RESET_SRST_NO_GATING)
		 || (!armv7m->debug_ap && !target->defer_examine))) {
		/* If we have no debug_ap, asserting SRST is the only thing
		 * we can do now */
		adapter_assert_reset();
		srst_asserted = true;
	}

	/* TODO: replace the hack calling target_examine_one()
	 * as soon as a better reset framework is available */
	if (!target_was_examined(target) && !target->defer_examine
		&& srst_asserted && (jtag_reset_config & RESET_SRST_NO_GATING)) {
		LOG_TARGET_DEBUG(target, "Trying to re-examine under reset");
		target_examine_one(target);
	}

	/* We need at least debug_ap to go further.
	 * Inform user and bail out if we don't have one. */
	if (!armv7m->debug_ap) {
		if (srst_asserted) {
			if (target->reset_halt)
				LOG_TARGET_ERROR(target, "Debug AP not available, will not halt after reset!");

			/* Do not propagate error: reset was asserted, proceed to deassert! */
			target->state = TARGET_RESET;
			register_cache_invalidate(cortex_m->armv7m.arm.core_cache);
			return ERROR_OK;

		} else {
			LOG_TARGET_ERROR(target, "Debug AP not available, reset NOT asserted!");
			return ERROR_FAIL;
		}
	}

	/* Enable debug requests */
	int retval = cortex_m_read_dhcsr_atomic_sticky(target);

	/* Store important errors instead of failing and proceed to reset assert */

	if (retval != ERROR_OK || !(cortex_m->dcb_dhcsr & C_DEBUGEN))
		retval = cortex_m_write_debug_halt_mask(target, 0, C_HALT | C_STEP | C_MASKINTS);

	/* If the processor is sleeping in a WFI or WFE instruction, the
	 * C_HALT bit must be asserted to regain control */
	if (retval == ERROR_OK && (cortex_m->dcb_dhcsr & S_SLEEP))
		retval = cortex_m_write_debug_halt_mask(target, C_HALT, 0);

	mem_ap_write_u32(armv7m->debug_ap, DCB_DCRDR, 0);
	/* Ignore less important errors */

	if (!target->reset_halt) {
		/* Set/Clear C_MASKINTS in a separate operation */
		cortex_m_set_maskints_for_run(target);

		/* clear any debug flags before resuming */
		cortex_m_clear_halt(target);

		/* clear C_HALT in dhcsr reg */
		cortex_m_write_debug_halt_mask(target, 0, C_HALT);
	} else {
		/* Halt in debug on reset; endreset_event() restores DEMCR.
		 *
		 * REVISIT catching BUSERR presumably helps to defend against
		 * bad vector table entries.  Should this include MMERR or
		 * other flags too?
		 */
		int retval2;
		retval2 = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DEMCR,
				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
		target->debug_reason = DBG_REASON_DBGRQ;
		if (retval != ERROR_OK || retval2 != ERROR_OK)
			LOG_TARGET_INFO(target, "AP write error, reset will not halt");
	}

	if (jtag_reset_config & RESET_HAS_SRST) {
		/* default to asserting srst */
		if (!srst_asserted)
			adapter_assert_reset();

		/* srst is asserted, ignore AP access errors */
		retval = ERROR_OK;
	} else {
		/* Use a standard Cortex-M software reset mechanism.
		 * We default to using VECTRESET.
		 * This has the disadvantage of not resetting the peripherals, so a
		 * reset-init event handler is needed to perform any peripheral resets.
		 */
		if (!cortex_m->vectreset_supported
				&& reset_config == CORTEX_M_RESET_VECTRESET) {
			reset_config = CORTEX_M_RESET_SYSRESETREQ;
			LOG_TARGET_WARNING(target, "VECTRESET is not supported on this Cortex-M core, using SYSRESETREQ instead.");
			LOG_TARGET_WARNING(target, "Set 'cortex_m reset_config sysresetreq'.");
		}

		LOG_TARGET_DEBUG(target, "Using Cortex-M %s", (reset_config == CORTEX_M_RESET_SYSRESETREQ)
			? "SYSRESETREQ" : "VECTRESET");

		if (reset_config == CORTEX_M_RESET_VECTRESET) {
			LOG_TARGET_WARNING(target, "Only resetting the Cortex-M core, use a reset-init event "
				"handler to reset any peripherals or configure hardware srst support.");
		}

		int retval3;
		retval3 = mem_ap_write_atomic_u32(armv7m->debug_ap, NVIC_AIRCR,
				AIRCR_VECTKEY | ((reset_config == CORTEX_M_RESET_SYSRESETREQ)
				? AIRCR_SYSRESETREQ : AIRCR_VECTRESET));
		if (retval3 != ERROR_OK)
			LOG_TARGET_DEBUG(target, "Ignoring AP write error right after reset");

		retval3 = dap_dp_init_or_reconnect(armv7m->debug_ap->dap);
		if (retval3 != ERROR_OK) {
			LOG_TARGET_ERROR(target, "DP initialisation failed");
			/* The error return value must not be propagated in this case.
			 * SYSRESETREQ or VECTRESET have been possibly triggered
			 * so reset processing should continue */
		} else {
			/* I do not know why this is necessary, but it
			 * fixes strange effects (step/resume cause NMI
			 * after reset) on LM3S6918 -- Michael Schwingen
			 */
			uint32_t tmp;
			mem_ap_read_atomic_u32(armv7m->debug_ap, NVIC_AIRCR, &tmp);
		}
	}

	target->state = TARGET_RESET;
	jtag_sleep(50000);

	register_cache_invalidate(cortex_m->armv7m.arm.core_cache);

	return retval;
}

static int cortex_m_deassert_reset(struct target *target)
{
	struct armv7m_common *armv7m = &target_to_cm(target)->armv7m;

	LOG_TARGET_DEBUG(target, "target->state: %s,%s examined",
		target_state_name(target),
		target_was_examined(target) ? "" : " not");

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	/* deassert reset lines */
	if (jtag_reset_config & RESET_HAS_SRST)
		adapter_deassert_reset();


	if ((jtag_reset_config & RESET_HAS_SRST) &&
		!(jtag_reset_config & RESET_SRST_NO_GATING) &&
		armv7m->debug_ap) {

		int retval = dap_dp_init_or_reconnect(armv7m->debug_ap->dap);
		if (retval != ERROR_OK) {
			LOG_TARGET_ERROR(target, "DP initialisation failed");
			return retval;
		}
	}

	return ERROR_OK;
}

int cortex_m_set_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	unsigned int fp_num = 0;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct cortex_m_fp_comparator *comparator_list = cortex_m->fp_comparator_list;

	if (breakpoint->is_set) {
		LOG_TARGET_WARNING(target, "breakpoint (BPID: %" PRIu32 ") already set", breakpoint->unique_id);
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		uint32_t fpcr_value;
		while (comparator_list[fp_num].used && (fp_num < cortex_m->fp_num_code))
			fp_num++;
		if (fp_num >= cortex_m->fp_num_code) {
			LOG_TARGET_ERROR(target, "Can not find free FPB Comparator!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint_hw_set(breakpoint, fp_num);
		fpcr_value = breakpoint->address | 1;
		if (cortex_m->fp_rev == 0) {
			if (breakpoint->address > 0x1FFFFFFF) {
				LOG_TARGET_ERROR(target, "Cortex-M Flash Patch Breakpoint rev.1 "
						"cannot handle HW breakpoint above address 0x1FFFFFFE");
				return ERROR_FAIL;
			}
			uint32_t hilo;
			hilo = (breakpoint->address & 0x2) ? FPCR_REPLACE_BKPT_HIGH : FPCR_REPLACE_BKPT_LOW;
			fpcr_value = (fpcr_value & 0x1FFFFFFC) | hilo | 1;
		} else if (cortex_m->fp_rev > 1) {
			LOG_TARGET_ERROR(target, "Unhandled Cortex-M Flash Patch Breakpoint architecture revision");
			return ERROR_FAIL;
		}
		comparator_list[fp_num].used = true;
		comparator_list[fp_num].fpcr_value = fpcr_value;
		target_write_u32(target, comparator_list[fp_num].fpcr_address,
			comparator_list[fp_num].fpcr_value);
		LOG_TARGET_DEBUG(target, "fpc_num %i fpcr_value 0x%" PRIx32 "",
			fp_num,
			comparator_list[fp_num].fpcr_value);
		if (!cortex_m->fpb_enabled) {
			LOG_TARGET_DEBUG(target, "FPB wasn't enabled, do it now");
			retval = cortex_m_enable_fpb(target);
			if (retval != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to enable the FPB");
				return retval;
			}

			cortex_m->fpb_enabled = true;
		}
	} else if (breakpoint->type == BKPT_SOFT) {
		uint8_t code[4];

		/* NOTE: on ARMv6-M and ARMv7-M, BKPT(0xab) is used for
		 * semihosting; don't use that.  Otherwise the BKPT
		 * parameter is arbitrary.
		 */
		buf_set_u32(code, 0, 32, ARMV5_T_BKPT(0x11));
		retval = target_read_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				code);
		if (retval != ERROR_OK)
			return retval;
		breakpoint->is_set = true;
	}

	LOG_TARGET_DEBUG(target, "BPID: %" PRIu32 ", Type: %d, Address: " TARGET_ADDR_FMT " Length: %d (n=%u)",
		breakpoint->unique_id,
		(int)(breakpoint->type),
		breakpoint->address,
		breakpoint->length,
		(breakpoint->type == BKPT_SOFT) ? 0 : breakpoint->number);

	return ERROR_OK;
}

int cortex_m_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct cortex_m_fp_comparator *comparator_list = cortex_m->fp_comparator_list;

	if (!breakpoint->is_set) {
		LOG_TARGET_WARNING(target, "breakpoint not set");
		return ERROR_OK;
	}

	LOG_TARGET_DEBUG(target, "BPID: %" PRIu32 ", Type: %d, Address: " TARGET_ADDR_FMT " Length: %d (n=%u)",
		breakpoint->unique_id,
		(int)(breakpoint->type),
		breakpoint->address,
		breakpoint->length,
		(breakpoint->type == BKPT_SOFT) ? 0 : breakpoint->number);

	if (breakpoint->type == BKPT_HARD) {
		unsigned int fp_num = breakpoint->number;
		if (fp_num >= cortex_m->fp_num_code) {
			LOG_TARGET_DEBUG(target, "Invalid FP Comparator number in breakpoint");
			return ERROR_OK;
		}
		comparator_list[fp_num].used = false;
		comparator_list[fp_num].fpcr_value = 0;
		target_write_u32(target, comparator_list[fp_num].fpcr_address,
			comparator_list[fp_num].fpcr_value);
	} else {
		/* restore original instruction (kept in target endianness) */
		retval = target_write_memory(target, breakpoint->address & 0xFFFFFFFE,
					breakpoint->length, 1,
					breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;
	}
	breakpoint->is_set = false;

	return ERROR_OK;
}

int cortex_m_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (breakpoint->length == 3) {
		LOG_TARGET_DEBUG(target, "Using a two byte breakpoint for 32bit Thumb-2 request");
		breakpoint->length = 2;
	}

	if ((breakpoint->length != 2)) {
		LOG_TARGET_INFO(target, "only breakpoints of two bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return cortex_m_set_breakpoint(target, breakpoint);
}

int cortex_m_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (!breakpoint->is_set)
		return ERROR_OK;

	return cortex_m_unset_breakpoint(target, breakpoint);
}

static int cortex_m_set_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	unsigned int dwt_num = 0;
	struct cortex_m_common *cortex_m = target_to_cm(target);

	/* REVISIT Don't fully trust these "not used" records ... users
	 * may set up breakpoints by hand, e.g. dual-address data value
	 * watchpoint using comparator #1; comparator #0 matching cycle
	 * count; send data trace info through ITM and TPIU; etc
	 */
	struct cortex_m_dwt_comparator *comparator;

	for (comparator = cortex_m->dwt_comparator_list;
		comparator->used && dwt_num < cortex_m->dwt_num_comp;
		comparator++, dwt_num++)
		continue;
	if (dwt_num >= cortex_m->dwt_num_comp) {
		LOG_TARGET_ERROR(target, "Can not find free DWT Comparator");
		return ERROR_FAIL;
	}
	comparator->used = true;
	watchpoint_set(watchpoint, dwt_num);

	comparator->comp = watchpoint->address;
	target_write_u32(target, comparator->dwt_comparator_address + 0,
		comparator->comp);

	if ((cortex_m->dwt_devarch & 0x1FFFFF) != DWT_DEVARCH_ARMV8M_V2_0
			&& (cortex_m->dwt_devarch & 0x1FFFFF) != DWT_DEVARCH_ARMV8M_V2_1) {
		uint32_t mask = 0, temp;

		/* watchpoint params were validated earlier */
		temp = watchpoint->length;
		while (temp) {
			temp >>= 1;
			mask++;
		}
		mask--;

		comparator->mask = mask;
		target_write_u32(target, comparator->dwt_comparator_address + 4,
			comparator->mask);

		switch (watchpoint->rw) {
		case WPT_READ:
			comparator->function = 5;
			break;
		case WPT_WRITE:
			comparator->function = 6;
			break;
		case WPT_ACCESS:
			comparator->function = 7;
			break;
		}
	} else {
		uint32_t data_size = watchpoint->length >> 1;
		comparator->mask = (watchpoint->length >> 1) | 1;

		switch (watchpoint->rw) {
		case WPT_ACCESS:
			comparator->function = 4;
			break;
		case WPT_WRITE:
			comparator->function = 5;
			break;
		case WPT_READ:
			comparator->function = 6;
			break;
		}
		comparator->function = comparator->function | (1 << 4) |
				(data_size << 10);
	}

	target_write_u32(target, comparator->dwt_comparator_address + 8,
		comparator->function);

	LOG_TARGET_DEBUG(target, "Watchpoint (ID %d) DWT%d 0x%08" PRIx32 " 0x%" PRIx32 " 0x%05" PRIx32,
		watchpoint->unique_id, dwt_num,
		comparator->comp, comparator->mask, comparator->function);
	return ERROR_OK;
}

static int cortex_m_unset_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct cortex_m_dwt_comparator *comparator;

	if (!watchpoint->is_set) {
		LOG_TARGET_WARNING(target, "watchpoint (wpid: %d) not set",
			watchpoint->unique_id);
		return ERROR_OK;
	}

	unsigned int dwt_num = watchpoint->number;

	LOG_TARGET_DEBUG(target, "Watchpoint (ID %d) DWT%u address: " TARGET_ADDR_FMT " clear",
		watchpoint->unique_id, dwt_num,
		watchpoint->address);

	if (dwt_num >= cortex_m->dwt_num_comp) {
		LOG_TARGET_DEBUG(target, "Invalid DWT Comparator number in watchpoint");
		return ERROR_OK;
	}

	comparator = cortex_m->dwt_comparator_list + dwt_num;
	comparator->used = false;
	comparator->function = 0;
	target_write_u32(target, comparator->dwt_comparator_address + 8,
		comparator->function);

	watchpoint->is_set = false;

	return ERROR_OK;
}

int cortex_m_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);

	if (cortex_m->dwt_comp_available < 1) {
		LOG_TARGET_DEBUG(target, "no comparators?");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* REVISIT This DWT may well be able to watch for specific data
	 * values.  Requires comparator #1 to set DATAVMATCH and match
	 * the data, and another comparator (DATAVADDR0) matching addr.
	 *
	 * NOTE: hardware doesn't support data value masking, so we'll need
	 * to check that mask is zero
	 */
	if (watchpoint->mask != WATCHPOINT_IGNORE_DATA_VALUE_MASK) {
		LOG_TARGET_DEBUG(target, "watchpoint value masks not supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* hardware allows address masks of up to 32K */
	unsigned int mask;

	for (mask = 0; mask < 16; mask++) {
		if ((1u << mask) == watchpoint->length)
			break;
	}
	if (mask == 16) {
		LOG_TARGET_DEBUG(target, "unsupported watchpoint length");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	if (watchpoint->address & ((1 << mask) - 1)) {
		LOG_TARGET_DEBUG(target, "watchpoint address is unaligned");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	cortex_m->dwt_comp_available--;
	LOG_TARGET_DEBUG(target, "dwt_comp_available: %d", cortex_m->dwt_comp_available);

	return ERROR_OK;
}

int cortex_m_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);

	/* REVISIT why check? DWT can be updated with core running ... */
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->is_set)
		cortex_m_unset_watchpoint(target, watchpoint);

	cortex_m->dwt_comp_available++;
	LOG_TARGET_DEBUG(target, "dwt_comp_available: %d", cortex_m->dwt_comp_available);

	return ERROR_OK;
}

static int cortex_m_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	if (target->debug_reason != DBG_REASON_WATCHPOINT)
		return ERROR_FAIL;

	struct cortex_m_common *cortex_m = target_to_cm(target);

	for (struct watchpoint *wp = target->watchpoints; wp; wp = wp->next) {
		if (!wp->is_set)
			continue;

		unsigned int dwt_num = wp->number;
		struct cortex_m_dwt_comparator *comparator = cortex_m->dwt_comparator_list + dwt_num;

		uint32_t dwt_function;
		int retval = target_read_u32(target, comparator->dwt_comparator_address + 8, &dwt_function);
		if (retval != ERROR_OK)
			return ERROR_FAIL;

		/* check the MATCHED bit */
		if (dwt_function & BIT(24)) {
			*hit_watchpoint = wp;
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

void cortex_m_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	/* set any pending watchpoints */
	while (watchpoint) {
		if (!watchpoint->is_set)
			cortex_m_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

static int cortex_m_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);

	if (armv7m->arm.arch == ARM_ARCH_V6M) {
		/* armv6m does not handle unaligned memory access */
		if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
			return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	return mem_ap_read_buf(armv7m->debug_ap, buffer, size, count, address);
}

static int cortex_m_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);

	if (armv7m->arm.arch == ARM_ARCH_V6M) {
		/* armv6m does not handle unaligned memory access */
		if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
			return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	return mem_ap_write_buf(armv7m->debug_ap, buffer, size, count, address);
}

static int cortex_m_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	armv7m_build_reg_cache(target);
	arm_semihosting_init(target);
	return ERROR_OK;
}

void cortex_m_deinit_target(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);

	if (!armv7m->is_hla_target && armv7m->debug_ap)
		dap_put_ap(armv7m->debug_ap);

	free(cortex_m->fp_comparator_list);

	cortex_m_dwt_free(target);
	armv7m_free_reg_cache(target);

	free(target->private_config);
	free(cortex_m);
}

int cortex_m_profiling(struct target *target, uint32_t *samples,
			      uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds)
{
	struct timeval timeout, now;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint32_t reg_value;
	int retval;

	retval = target_read_u32(target, DWT_PCSR, &reg_value);
	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Error while reading PCSR");
		return retval;
	}
	if (reg_value == 0) {
		LOG_TARGET_INFO(target, "PCSR sampling not supported on this processor.");
		return target_profiling_default(target, samples, max_num_samples, num_samples, seconds);
	}

	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, seconds, 0);

	LOG_TARGET_INFO(target, "Starting Cortex-M profiling. Sampling DWT_PCSR as fast as we can...");

	/* Make sure the target is running */
	target_poll(target);
	if (target->state == TARGET_HALTED)
		retval = target_resume(target, true, 0, false, false);

	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Error while resuming target");
		return retval;
	}

	uint32_t sample_count = 0;

	for (;;) {
		if (armv7m && armv7m->debug_ap) {
			uint32_t read_count = max_num_samples - sample_count;
			if (read_count > 1024)
				read_count = 1024;

			retval = mem_ap_read_buf_noincr(armv7m->debug_ap,
						(void *)&samples[sample_count],
						4, read_count, DWT_PCSR);
			sample_count += read_count;
		} else {
			target_read_u32(target, DWT_PCSR, &samples[sample_count++]);
		}

		if (retval != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Error while reading PCSR");
			return retval;
		}


		gettimeofday(&now, NULL);
		if (sample_count >= max_num_samples || timeval_compare(&now, &timeout) > 0) {
			LOG_TARGET_INFO(target, "Profiling completed. %" PRIu32 " samples.", sample_count);
			break;
		}
	}

	*num_samples = sample_count;
	return retval;
}


/* REVISIT cache valid/dirty bits are unmaintained.  We could set "valid"
 * on r/w if the core is not running, and clear on resume or reset ... or
 * at least, in a post_restore_context() method.
 */

struct dwt_reg_state {
	struct target *target;
	uint32_t addr;
	uint8_t value[4];		/* scratch/cache */
};

static int cortex_m_dwt_get_reg(struct reg *reg)
{
	struct dwt_reg_state *state = reg->arch_info;

	uint32_t tmp;
	int retval = target_read_u32(state->target, state->addr, &tmp);
	if (retval != ERROR_OK)
		return retval;

	buf_set_u32(state->value, 0, 32, tmp);
	return ERROR_OK;
}

static int cortex_m_dwt_set_reg(struct reg *reg, uint8_t *buf)
{
	struct dwt_reg_state *state = reg->arch_info;

	return target_write_u32(state->target, state->addr,
			buf_get_u32(buf, 0, reg->size));
}

struct dwt_reg {
	uint32_t addr;
	const char *name;
	unsigned int size;
};

static const struct dwt_reg dwt_base_regs[] = {
	{ DWT_CTRL, "dwt_ctrl", 32, },
	/* NOTE that Erratum 532314 (fixed r2p0) affects CYCCNT:  it wrongly
	 * increments while the core is asleep.
	 */
	{ DWT_CYCCNT, "dwt_cyccnt", 32, },
	/* plus some 8 bit counters, useful for profiling with TPIU */
};

static const struct dwt_reg dwt_comp[] = {
#define DWT_COMPARATOR(i) \
		{ DWT_COMP0 + 0x10 * (i), "dwt_" #i "_comp", 32, }, \
		{ DWT_MASK0 + 0x10 * (i), "dwt_" #i "_mask", 4, }, \
		{ DWT_FUNCTION0 + 0x10 * (i), "dwt_" #i "_function", 32, }
	DWT_COMPARATOR(0),
	DWT_COMPARATOR(1),
	DWT_COMPARATOR(2),
	DWT_COMPARATOR(3),
	DWT_COMPARATOR(4),
	DWT_COMPARATOR(5),
	DWT_COMPARATOR(6),
	DWT_COMPARATOR(7),
	DWT_COMPARATOR(8),
	DWT_COMPARATOR(9),
	DWT_COMPARATOR(10),
	DWT_COMPARATOR(11),
	DWT_COMPARATOR(12),
	DWT_COMPARATOR(13),
	DWT_COMPARATOR(14),
	DWT_COMPARATOR(15),
#undef DWT_COMPARATOR
};

static const struct reg_arch_type dwt_reg_type = {
	.get = cortex_m_dwt_get_reg,
	.set = cortex_m_dwt_set_reg,
};

static void cortex_m_dwt_addreg(struct target *t, struct reg *r, const struct dwt_reg *d)
{
	struct dwt_reg_state *state;

	state = calloc(1, sizeof(*state));
	if (!state)
		return;
	state->addr = d->addr;
	state->target = t;

	r->name = d->name;
	r->size = d->size;
	r->value = state->value;
	r->arch_info = state;
	r->type = &dwt_reg_type;
	r->exist = true;
}

static void cortex_m_dwt_setup(struct cortex_m_common *cm, struct target *target)
{
	uint32_t dwtcr;
	struct reg_cache *cache;
	struct cortex_m_dwt_comparator *comparator;
	int reg;

	target_read_u32(target, DWT_CTRL, &dwtcr);
	LOG_TARGET_DEBUG(target, "DWT_CTRL: 0x%" PRIx32, dwtcr);
	if (!dwtcr) {
		LOG_TARGET_DEBUG(target, "no DWT");
		return;
	}

	target_read_u32(target, DWT_DEVARCH, &cm->dwt_devarch);
	LOG_TARGET_DEBUG(target, "DWT_DEVARCH: 0x%" PRIx32, cm->dwt_devarch);

	cm->dwt_num_comp = (dwtcr >> 28) & 0xF;
	cm->dwt_comp_available = cm->dwt_num_comp;
	cm->dwt_comparator_list = calloc(cm->dwt_num_comp,
			sizeof(struct cortex_m_dwt_comparator));
	if (!cm->dwt_comparator_list) {
fail0:
		cm->dwt_num_comp = 0;
		LOG_TARGET_ERROR(target, "out of mem");
		return;
	}

	cache = calloc(1, sizeof(*cache));
	if (!cache) {
fail1:
		free(cm->dwt_comparator_list);
		goto fail0;
	}
	cache->name = "Cortex-M DWT registers";
	cache->num_regs = 2 + cm->dwt_num_comp * 3;
	cache->reg_list = calloc(cache->num_regs, sizeof(*cache->reg_list));
	if (!cache->reg_list) {
		free(cache);
		goto fail1;
	}

	for (reg = 0; reg < 2; reg++)
		cortex_m_dwt_addreg(target, cache->reg_list + reg,
			dwt_base_regs + reg);

	comparator = cm->dwt_comparator_list;
	for (unsigned int i = 0; i < cm->dwt_num_comp; i++, comparator++) {
		int j;

		comparator->dwt_comparator_address = DWT_COMP0 + 0x10 * i;
		for (j = 0; j < 3; j++, reg++)
			cortex_m_dwt_addreg(target, cache->reg_list + reg,
				dwt_comp + 3 * i + j);

		/* make sure we clear any watchpoints enabled on the target */
		target_write_u32(target, comparator->dwt_comparator_address + 8, 0);
	}

	*register_get_last_cache_p(&target->reg_cache) = cache;
	cm->dwt_cache = cache;

	LOG_TARGET_DEBUG(target, "DWT dwtcr 0x%" PRIx32 ", comp %d, watch%s",
		dwtcr, cm->dwt_num_comp,
		(dwtcr & (0xf << 24)) ? " only" : "/trigger");

	/* REVISIT:  if num_comp > 1, check whether comparator #1 can
	 * implement single-address data value watchpoints ... so we
	 * won't need to check it later, when asked to set one up.
	 */
}

static void cortex_m_dwt_free(struct target *target)
{
	struct cortex_m_common *cm = target_to_cm(target);
	struct reg_cache *cache = cm->dwt_cache;

	free(cm->dwt_comparator_list);
	cm->dwt_comparator_list = NULL;
	cm->dwt_num_comp = 0;

	if (cache) {
		register_unlink_cache(&target->reg_cache, cache);

		if (cache->reg_list) {
			for (size_t i = 0; i < cache->num_regs; i++)
				free(cache->reg_list[i].arch_info);
			free(cache->reg_list);
		}
		free(cache);
	}
	cm->dwt_cache = NULL;
}

static bool cortex_m_has_tz(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint32_t dauthstatus;

	if (armv7m->arm.arch != ARM_ARCH_V8M)
		return false;

	int retval = target_read_u32(target, DAUTHSTATUS, &dauthstatus);
	if (retval != ERROR_OK) {
		LOG_TARGET_WARNING(target, "Error reading DAUTHSTATUS register");
		return false;
	}
	return (dauthstatus & DAUTHSTATUS_SID_MASK) != 0;
}


#define MVFR0          0xE000EF40
#define MVFR0_SP_MASK  0x000000F0
#define MVFR0_SP       0x00000020
#define MVFR0_DP_MASK  0x00000F00
#define MVFR0_DP       0x00000200

#define MVFR1          0xE000EF44
#define MVFR1_MVE_MASK 0x00000F00
#define MVFR1_MVE_I    0x00000100
#define MVFR1_MVE_F    0x00000200

static int cortex_m_find_mem_ap(struct adiv5_dap *swjdp,
		struct adiv5_ap **debug_ap)
{
	if (dap_find_get_ap(swjdp, AP_TYPE_AHB3_AP, debug_ap) == ERROR_OK)
		return ERROR_OK;

	return dap_find_get_ap(swjdp, AP_TYPE_AHB5_AP, debug_ap);
}

int cortex_m_examine(struct target *target)
{
	int retval;
	uint32_t cpuid, fpcr;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct adiv5_dap *swjdp = cortex_m->armv7m.arm.dap;
	struct armv7m_common *armv7m = target_to_armv7m(target);

	/* hla_target shares the examine handler but does not support
	 * all its calls */
	if (!armv7m->is_hla_target) {
		if (!armv7m->debug_ap) {
			if (cortex_m->apsel == DP_APSEL_INVALID) {
				/* Search for the MEM-AP */
				retval = cortex_m_find_mem_ap(swjdp, &armv7m->debug_ap);
				if (retval != ERROR_OK) {
					LOG_TARGET_ERROR(target, "Could not find MEM-AP to control the core");
					return retval;
				}
			} else {
				armv7m->debug_ap = dap_get_ap(swjdp, cortex_m->apsel);
				if (!armv7m->debug_ap) {
					LOG_TARGET_ERROR(target, "Cannot get AP");
					return ERROR_FAIL;
				}
			}
		}

		armv7m->debug_ap->memaccess_tck = 8;

		retval = mem_ap_init(armv7m->debug_ap);
		if (retval != ERROR_OK)
			return retval;
	}

	if (!target_was_examined(target)) {
		target_set_examined(target);

		/* Read from Device Identification Registers */
		retval = target_read_u32(target, CPUID, &cpuid);
		if (retval != ERROR_OK)
			return retval;

		/* Inspect implementer/part to look for recognized cores  */
		unsigned int impl_part = cpuid & (ARM_CPUID_IMPLEMENTER_MASK | ARM_CPUID_PARTNO_MASK);

		for (unsigned int n = 0; n < ARRAY_SIZE(cortex_m_parts); n++) {
			if (impl_part == cortex_m_parts[n].impl_part) {
				cortex_m->core_info = &cortex_m_parts[n];
				break;
			}
		}

		if (!cortex_m->core_info) {
			LOG_TARGET_ERROR(target, "Cortex-M CPUID: 0x%x is unrecognized", cpuid);
			return ERROR_FAIL;
		}

		armv7m->arm.arch = cortex_m->core_info->arch;

		LOG_TARGET_INFO(target, "%s r%" PRId8 "p%" PRId8 " processor detected",
				cortex_m->core_info->name,
				(uint8_t)((cpuid >> 20) & 0xf),
				(uint8_t)((cpuid >> 0) & 0xf));

		cortex_m->maskints_erratum = false;
		cortex_m->incorrect_halt_erratum = false;
		if (impl_part == CORTEX_M7_PARTNO) {
			uint8_t rev, patch;
			rev = (cpuid >> 20) & 0xf;
			patch = (cpuid >> 0) & 0xf;
			if ((rev == 0) && (patch < 2)) {
				LOG_TARGET_WARNING(target, "Erratum 702596: single stepping may enter pending exception handler!");
				cortex_m->maskints_erratum = true;
			}
			/* TODO: add revision check when a Cortex-M7 revision with fixed 3092511 is out */
			LOG_TARGET_WARNING(target, "Erratum 3092511: Cortex-M7 can halt in an incorrect address when breakpoint and exception occurs simultaneously");
			cortex_m->incorrect_halt_erratum = true;
			if (armv7m->is_hla_target)
				LOG_TARGET_WARNING(target, "No erratum 3092511 workaround on hla adapter");
			else
				LOG_TARGET_INFO(target, "The erratum 3092511 workaround will resume after an incorrect halt");
		}
		LOG_TARGET_DEBUG(target, "cpuid: 0x%8.8" PRIx32 "", cpuid);

		if (cortex_m->core_info->flags & CORTEX_M_F_HAS_FPV4) {
			uint32_t mvfr0;
			target_read_u32(target, MVFR0, &mvfr0);

			if ((mvfr0 & MVFR0_SP_MASK) == MVFR0_SP) {
				LOG_TARGET_DEBUG(target, "%s floating point feature FPv4_SP found",
						cortex_m->core_info->name);
				armv7m->fp_feature = FPV4_SP;
			}
		} else if (cortex_m->core_info->flags & CORTEX_M_F_HAS_FPV5) {
			uint32_t mvfr0, mvfr1;
			target_read_u32(target, MVFR0, &mvfr0);
			target_read_u32(target, MVFR1, &mvfr1);

			if ((mvfr0 & MVFR0_DP_MASK) == MVFR0_DP) {
				if ((mvfr1 & MVFR1_MVE_MASK) == MVFR1_MVE_F) {
					LOG_TARGET_DEBUG(target, "%s floating point feature FPv5_DP + MVE-F found",
							cortex_m->core_info->name);
					armv7m->fp_feature = FPV5_MVE_F;
				} else {
					LOG_TARGET_DEBUG(target, "%s floating point feature FPv5_DP found",
							cortex_m->core_info->name);
					armv7m->fp_feature = FPV5_DP;
				}
			} else if ((mvfr0 & MVFR0_SP_MASK) == MVFR0_SP) {
				LOG_TARGET_DEBUG(target, "%s floating point feature FPv5_SP found",
						cortex_m->core_info->name);
				armv7m->fp_feature = FPV5_SP;
			} else if ((mvfr1 & MVFR1_MVE_MASK) == MVFR1_MVE_I) {
				LOG_TARGET_DEBUG(target, "%s floating point feature MVE-I found",
						cortex_m->core_info->name);
				armv7m->fp_feature = FPV5_MVE_I;
			}
		}

		/* VECTRESET is supported only on ARMv7-M cores */
		cortex_m->vectreset_supported = armv7m->arm.arch == ARM_ARCH_V7M;

		/* Check for FPU, otherwise mark FPU register as non-existent */
		if (armv7m->fp_feature == FP_NONE)
			for (size_t idx = ARMV7M_FPU_FIRST_REG; idx <= ARMV7M_FPU_LAST_REG; idx++)
				armv7m->arm.core_cache->reg_list[idx].exist = false;

		/* TODO: MVE can be present without floating points. Revisit this test */
		if (armv7m->fp_feature != FPV5_MVE_F && armv7m->fp_feature != FPV5_MVE_I)
			armv7m->arm.core_cache->reg_list[ARMV8M_VPR].exist = false;

		if (!cortex_m_has_tz(target))
			for (size_t idx = ARMV8M_FIRST_REG; idx <= ARMV8M_LAST_REG; idx++)
				armv7m->arm.core_cache->reg_list[idx].exist = false;

		if (!armv7m->is_hla_target) {
			if (cortex_m->core_info->flags & CORTEX_M_F_TAR_AUTOINCR_BLOCK_4K)
				/* Cortex-M3/M4 have 4096 bytes autoincrement range,
				 * s. ARM IHI 0031C: MEM-AP 7.2.2 */
				armv7m->debug_ap->tar_autoincr_block = (1 << 12);
		}

		retval = target_read_u32(target, DCB_DHCSR, &cortex_m->dcb_dhcsr);
		if (retval != ERROR_OK)
			return retval;

		/*  Don't cumulate sticky S_RESET_ST at the very first read of DHCSR
		 *  as S_RESET_ST may indicate a reset that happened long time ago
		 *  (most probably the power-on reset before OpenOCD was started).
		 *  As we are just initializing the debug system we do not need
		 *  to call cortex_m_endreset_event() in the following poll.
		 */
		if (!cortex_m->dcb_dhcsr_sticky_is_recent) {
			cortex_m->dcb_dhcsr_sticky_is_recent = true;
			if (cortex_m->dcb_dhcsr & S_RESET_ST) {
				LOG_TARGET_DEBUG(target, "reset happened some time ago, ignore");
				cortex_m->dcb_dhcsr &= ~S_RESET_ST;
			}
		}
		cortex_m_cumulate_dhcsr_sticky(cortex_m, cortex_m->dcb_dhcsr);

		if (!(cortex_m->dcb_dhcsr & C_DEBUGEN)) {
			/* Enable debug requests */
			uint32_t dhcsr = (cortex_m->dcb_dhcsr | C_DEBUGEN) & ~(C_HALT | C_STEP | C_MASKINTS);

			retval = target_write_u32(target, DCB_DHCSR, DBGKEY | (dhcsr & 0x0000FFFFUL));
			if (retval != ERROR_OK)
				return retval;
			cortex_m->dcb_dhcsr = dhcsr;
		}

		/* Configure trace modules */
		retval = target_write_u32(target, DCB_DEMCR, TRCENA | armv7m->demcr);
		if (retval != ERROR_OK)
			return retval;

		/* Configure ITM */
		armv7m_trace_itm_config(target);

		/* NOTE: FPB and DWT are both optional. */

		/* Setup FPB */
		target_read_u32(target, FP_CTRL, &fpcr);
		/* bits [14:12] and [7:4] */
		cortex_m->fp_num_code = ((fpcr >> 8) & 0x70) | ((fpcr >> 4) & 0xF);
		cortex_m->fp_num_lit = (fpcr >> 8) & 0xF;
		/* Detect flash patch revision, see RM DDI 0403E.b page C1-817.
		   Revision is zero base, fp_rev == 1 means Rev.2 ! */
		cortex_m->fp_rev = (fpcr >> 28) & 0xf;
		free(cortex_m->fp_comparator_list);
		cortex_m->fp_comparator_list = calloc(
				cortex_m->fp_num_code + cortex_m->fp_num_lit,
				sizeof(struct cortex_m_fp_comparator));
		cortex_m->fpb_enabled = fpcr & 1;
		for (unsigned int i = 0; i < cortex_m->fp_num_code + cortex_m->fp_num_lit; i++) {
			cortex_m->fp_comparator_list[i].type =
				(i < cortex_m->fp_num_code) ? FPCR_CODE : FPCR_LITERAL;
			cortex_m->fp_comparator_list[i].fpcr_address = FP_COMP0 + 4 * i;

			/* make sure we clear any breakpoints enabled on the target */
			target_write_u32(target, cortex_m->fp_comparator_list[i].fpcr_address, 0);
		}
		LOG_TARGET_DEBUG(target, "FPB fpcr 0x%" PRIx32 ", numcode %i, numlit %i",
			fpcr,
			cortex_m->fp_num_code,
			cortex_m->fp_num_lit);

		/* Setup DWT */
		cortex_m_dwt_free(target);
		cortex_m_dwt_setup(cortex_m, target);

		/* These hardware breakpoints only work for code in flash! */
		LOG_TARGET_INFO(target, "target has %d breakpoints, %d watchpoints",
			cortex_m->fp_num_code,
			cortex_m->dwt_num_comp);
	}

	return ERROR_OK;
}

static int cortex_m_dcc_read(struct target *target, uint8_t *value, uint8_t *ctrl)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint16_t dcrdr;
	uint8_t buf[2];
	int retval;

	retval = mem_ap_read_buf_noincr(armv7m->debug_ap, buf, 2, 1, DCB_DCRDR);
	if (retval != ERROR_OK)
		return retval;

	dcrdr = target_buffer_get_u16(target, buf);
	*ctrl = (uint8_t)dcrdr;
	*value = (uint8_t)(dcrdr >> 8);

	LOG_TARGET_DEBUG(target, "data 0x%x ctrl 0x%x", *value, *ctrl);

	/* write ack back to software dcc register
	 * signify we have read data */
	if (dcrdr & (1 << 0)) {
		target_buffer_set_u16(target, buf, 0);
		retval = mem_ap_write_buf_noincr(armv7m->debug_ap, buf, 2, 1, DCB_DCRDR);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int cortex_m_target_request_data(struct target *target,
	uint32_t size, uint8_t *buffer)
{
	uint8_t data;
	uint8_t ctrl;
	uint32_t i;

	for (i = 0; i < (size * 4); i++) {
		int retval = cortex_m_dcc_read(target, &data, &ctrl);
		if (retval != ERROR_OK)
			return retval;
		buffer[i] = data;
	}

	return ERROR_OK;
}

static int cortex_m_handle_target_request(void *priv)
{
	struct target *target = priv;
	if (!target_was_examined(target))
		return ERROR_OK;

	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING) {
		uint8_t data;
		uint8_t ctrl;
		int retval;

		retval = cortex_m_dcc_read(target, &data, &ctrl);
		if (retval != ERROR_OK)
			return retval;

		/* check if we have data */
		if (ctrl & (1 << 0)) {
			uint32_t request;

			/* we assume target is quick enough */
			request = data;
			for (int i = 1; i <= 3; i++) {
				retval = cortex_m_dcc_read(target, &data, &ctrl);
				if (retval != ERROR_OK)
					return retval;
				request |= ((uint32_t)data << (i * 8));
			}
			target_request(target, request);
		}
	}

	return ERROR_OK;
}

static int cortex_m_init_arch_info(struct target *target,
	struct cortex_m_common *cortex_m, struct adiv5_dap *dap)
{
	struct armv7m_common *armv7m = &cortex_m->armv7m;

	armv7m_init_arch_info(target, armv7m);

	/* default reset mode is to use srst if fitted
	 * if not it will use CORTEX_M_RESET_VECTRESET */
	cortex_m->soft_reset_config = CORTEX_M_RESET_VECTRESET;

	armv7m->arm.dap = dap;

	/* register arch-specific functions */
	armv7m->examine_debug_reason = cortex_m_examine_debug_reason;

	armv7m->post_debug_entry = NULL;

	armv7m->pre_restore_context = NULL;

	armv7m->load_core_reg_u32 = cortex_m_load_core_reg_u32;
	armv7m->store_core_reg_u32 = cortex_m_store_core_reg_u32;

	target_register_timer_callback(cortex_m_handle_target_request, 1,
		TARGET_TIMER_TYPE_PERIODIC, target);

	return ERROR_OK;
}

static int cortex_m_target_create(struct target *target)
{
	struct adiv5_private_config *pc;

	pc = (struct adiv5_private_config *)target->private_config;
	if (adiv5_verify_config(pc) != ERROR_OK)
		return ERROR_FAIL;

	struct cortex_m_common *cortex_m = calloc(1, sizeof(struct cortex_m_common));
	if (!cortex_m) {
		LOG_TARGET_ERROR(target, "No memory creating target");
		return ERROR_FAIL;
	}

	cortex_m->common_magic = CORTEX_M_COMMON_MAGIC;
	cortex_m->apsel = pc->ap_num;

	cortex_m_init_arch_info(target, cortex_m, pc->dap);

	return ERROR_OK;
}

/*--------------------------------------------------------------------------*/

static int cortex_m_verify_pointer(struct command_invocation *cmd,
	struct cortex_m_common *cm)
{
	if (!is_cortex_m_with_dap_access(cm)) {
		command_print(cmd, "target is not a Cortex-M");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/*
 * Only stuff below this line should need to verify that its target
 * is a Cortex-M with available DAP access (not a HLA adapter).
 */

COMMAND_HANDLER(handle_cortex_m_vector_catch_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct armv7m_common *armv7m = &cortex_m->armv7m;
	uint32_t demcr = 0;
	int retval;

	static const struct {
		char name[10];
		unsigned int mask;
	} vec_ids[] = {
		{ "hard_err",   VC_HARDERR, },
		{ "int_err",    VC_INTERR, },
		{ "bus_err",    VC_BUSERR, },
		{ "state_err",  VC_STATERR, },
		{ "chk_err",    VC_CHKERR, },
		{ "nocp_err",   VC_NOCPERR, },
		{ "mm_err",     VC_MMERR, },
		{ "reset",      VC_CORERESET, },
	};

	retval = cortex_m_verify_pointer(CMD, cortex_m);
	if (retval != ERROR_OK)
		return retval;

	if (!target_was_examined(target)) {
		LOG_TARGET_ERROR(target, "Target not examined yet");
		return ERROR_FAIL;
	}

	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DEMCR, &demcr);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC > 0) {
		unsigned int catch = 0;

		if (CMD_ARGC == 1) {
			if (strcmp(CMD_ARGV[0], "all") == 0) {
				catch = VC_HARDERR | VC_INTERR | VC_BUSERR
					| VC_STATERR | VC_CHKERR | VC_NOCPERR
					| VC_MMERR | VC_CORERESET;
				goto write;
			} else if (strcmp(CMD_ARGV[0], "none") == 0)
				goto write;
		}
		while (CMD_ARGC-- > 0) {
			unsigned int i;
			for (i = 0; i < ARRAY_SIZE(vec_ids); i++) {
				if (strcmp(CMD_ARGV[CMD_ARGC], vec_ids[i].name) != 0)
					continue;
				catch |= vec_ids[i].mask;
				break;
			}
			if (i == ARRAY_SIZE(vec_ids)) {
				LOG_TARGET_ERROR(target, "No Cortex-M vector '%s'", CMD_ARGV[CMD_ARGC]);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		}
write:
		/* For now, armv7m->demcr only stores vector catch flags. */
		armv7m->demcr = catch;

		demcr &= ~0xffff;
		demcr |= catch;

		/* write, but don't assume it stuck (why not??) */
		retval = mem_ap_write_u32(armv7m->debug_ap, DCB_DEMCR, demcr);
		if (retval != ERROR_OK)
			return retval;
		retval = mem_ap_read_atomic_u32(armv7m->debug_ap, DCB_DEMCR, &demcr);
		if (retval != ERROR_OK)
			return retval;

		/* FIXME be sure to clear DEMCR on clean server shutdown.
		 * Otherwise the vector catch hardware could fire when there's
		 * no debugger hooked up, causing much confusion...
		 */
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(vec_ids); i++) {
		command_print(CMD, "%9s: %s", vec_ids[i].name,
			(demcr & vec_ids[i].mask) ? "catch" : "ignore");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cortex_m_mask_interrupts_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	int retval;

	static const struct nvp nvp_maskisr_modes[] = {
		{ .name = "auto", .value = CORTEX_M_ISRMASK_AUTO },
		{ .name = "off", .value = CORTEX_M_ISRMASK_OFF },
		{ .name = "on", .value = CORTEX_M_ISRMASK_ON },
		{ .name = "steponly", .value = CORTEX_M_ISRMASK_STEPONLY },
		{ .name = NULL, .value = -1 },
	};
	const struct nvp *n;


	retval = cortex_m_verify_pointer(CMD, cortex_m);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "Error: target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (CMD_ARGC > 0) {
		n = nvp_name2value(nvp_maskisr_modes, CMD_ARGV[0]);
		if (!n->name)
			return ERROR_COMMAND_SYNTAX_ERROR;
		cortex_m->isrmasking_mode = n->value;
		cortex_m_set_maskints_for_halt(target);
	}

	n = nvp_value2name(nvp_maskisr_modes, cortex_m->isrmasking_mode);
	command_print(CMD, "cortex_m interrupt mask %s", n->name);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cortex_m_reset_config_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	int retval;
	char *reset_config;

	retval = cortex_m_verify_pointer(CMD, cortex_m);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC > 0) {
		if (strcmp(*CMD_ARGV, "sysresetreq") == 0)
			cortex_m->soft_reset_config = CORTEX_M_RESET_SYSRESETREQ;

		else if (strcmp(*CMD_ARGV, "vectreset") == 0) {
			if (target_was_examined(target)
					&& !cortex_m->vectreset_supported)
				LOG_TARGET_WARNING(target, "VECTRESET is not supported on your Cortex-M core!");
			else
				cortex_m->soft_reset_config = CORTEX_M_RESET_VECTRESET;

		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	switch (cortex_m->soft_reset_config) {
		case CORTEX_M_RESET_SYSRESETREQ:
			reset_config = "sysresetreq";
			break;

		case CORTEX_M_RESET_VECTRESET:
			reset_config = "vectreset";
			break;

		default:
			reset_config = "unknown";
			break;
	}

	command_print(CMD, "cortex_m reset_config %s", reset_config);

	return ERROR_OK;
}

static const struct command_registration cortex_m_exec_command_handlers[] = {
	{
		.name = "maskisr",
		.handler = handle_cortex_m_mask_interrupts_command,
		.mode = COMMAND_EXEC,
		.help = "mask cortex_m interrupts",
		.usage = "['auto'|'on'|'off'|'steponly']",
	},
	{
		.name = "vector_catch",
		.handler = handle_cortex_m_vector_catch_command,
		.mode = COMMAND_EXEC,
		.help = "configure hardware vectors to trigger debug entry",
		.usage = "['all'|'none'|('bus_err'|'chk_err'|...)*]",
	},
	{
		.name = "reset_config",
		.handler = handle_cortex_m_reset_config_command,
		.mode = COMMAND_ANY,
		.help = "configure software reset handling",
		.usage = "['sysresetreq'|'vectreset']",
	},
	{
		.chain = smp_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration cortex_m_command_handlers[] = {
	{
		.chain = armv7m_command_handlers,
	},
	{
		.chain = armv7m_trace_command_handlers,
	},
	/* START_DEPRECATED_TPIU */
	{
		.chain = arm_tpiu_deprecated_command_handlers,
	},
	/* END_DEPRECATED_TPIU */
	{
		.name = "cortex_m",
		.mode = COMMAND_EXEC,
		.help = "Cortex-M command group",
		.usage = "",
		.chain = cortex_m_exec_command_handlers,
	},
	{
		.chain = rtt_target_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type cortexm_target = {
	.name = "cortex_m",

	.poll = cortex_m_poll,
	.arch_state = armv7m_arch_state,

	.target_request_data = cortex_m_target_request_data,

	.halt = cortex_m_halt,
	.resume = cortex_m_resume,
	.step = cortex_m_step,

	.assert_reset = cortex_m_assert_reset,
	.deassert_reset = cortex_m_deassert_reset,
	.soft_reset_halt = cortex_m_soft_reset_halt,

	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = armv7m_get_gdb_reg_list,

	.read_memory = cortex_m_read_memory,
	.write_memory = cortex_m_write_memory,
	.checksum_memory = armv7m_checksum_memory,
	.blank_check_memory = armv7m_blank_check_memory,

	.run_algorithm = armv7m_run_algorithm,
	.start_algorithm = armv7m_start_algorithm,
	.wait_algorithm = armv7m_wait_algorithm,

	.add_breakpoint = cortex_m_add_breakpoint,
	.remove_breakpoint = cortex_m_remove_breakpoint,
	.add_watchpoint = cortex_m_add_watchpoint,
	.remove_watchpoint = cortex_m_remove_watchpoint,
	.hit_watchpoint = cortex_m_hit_watchpoint,

	.commands = cortex_m_command_handlers,
	.target_create = cortex_m_target_create,
	.target_jim_configure = adiv5_jim_configure,
	.init_target = cortex_m_init_target,
	.examine = cortex_m_examine,
	.deinit_target = cortex_m_deinit_target,

	.profiling = cortex_m_profiling,
};

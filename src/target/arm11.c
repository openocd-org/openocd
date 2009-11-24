/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
 *   Michael Bruck                                                         *
 *                                                                         *
 *   Copyright (C) 2008,2009 Oyvind Harboe oyvind.harboe@zylin.com         *
 *                                                                         *
 *   Copyright (C) 2008 Georg Acher <acher@in.tum.de>                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "etm.h"
#include "breakpoints.h"
#include "arm11_dbgtap.h"
#include "arm_simulator.h"
#include "time_support.h"
#include "target_type.h"
#include "algorithm.h"
#include "register.h"


#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

static bool arm11_config_memwrite_burst = true;
static bool arm11_config_memwrite_error_fatal = true;
static uint32_t arm11_vcr = 0;
static bool arm11_config_step_irq_enable = false;
static bool arm11_config_hardware_step = false;

enum arm11_regtype
{
	ARM11_REGISTER_CORE,
	ARM11_REGISTER_CPSR,

	ARM11_REGISTER_FX,
	ARM11_REGISTER_FPS,

	ARM11_REGISTER_FIQ,
	ARM11_REGISTER_SVC,
	ARM11_REGISTER_ABT,
	ARM11_REGISTER_IRQ,
	ARM11_REGISTER_UND,
	ARM11_REGISTER_MON,

	ARM11_REGISTER_SPSR_FIQ,
	ARM11_REGISTER_SPSR_SVC,
	ARM11_REGISTER_SPSR_ABT,
	ARM11_REGISTER_SPSR_IRQ,
	ARM11_REGISTER_SPSR_UND,
	ARM11_REGISTER_SPSR_MON,

	/* debug regs */
	ARM11_REGISTER_DSCR,
	ARM11_REGISTER_WDTR,
	ARM11_REGISTER_RDTR,
};


struct arm11_reg_defs
{
	char *					name;
	uint32_t						num;
	int						gdb_num;
	enum arm11_regtype		type;
};

/* update arm11_regcache_ids when changing this */
static const struct arm11_reg_defs arm11_reg_defs[] =
{
	{"r0",	0,	0,	ARM11_REGISTER_CORE},
	{"r1",	1,	1,	ARM11_REGISTER_CORE},
	{"r2",	2,	2,	ARM11_REGISTER_CORE},
	{"r3",	3,	3,	ARM11_REGISTER_CORE},
	{"r4",	4,	4,	ARM11_REGISTER_CORE},
	{"r5",	5,	5,	ARM11_REGISTER_CORE},
	{"r6",	6,	6,	ARM11_REGISTER_CORE},
	{"r7",	7,	7,	ARM11_REGISTER_CORE},
	{"r8",	8,	8,	ARM11_REGISTER_CORE},
	{"r9",	9,	9,	ARM11_REGISTER_CORE},
	{"r10",	10,	10,	ARM11_REGISTER_CORE},
	{"r11",	11,	11,	ARM11_REGISTER_CORE},
	{"r12",	12,	12,	ARM11_REGISTER_CORE},
	{"sp",	13,	13,	ARM11_REGISTER_CORE},
	{"lr",	14,	14,	ARM11_REGISTER_CORE},
	{"pc",	15,	15,	ARM11_REGISTER_CORE},

	{"cpsr",	0,	25,	ARM11_REGISTER_CPSR},

	/* Debug Registers */
	{"dscr",	0,	-1,	ARM11_REGISTER_DSCR},
	{"wdtr",	0,	-1,	ARM11_REGISTER_WDTR},
	{"rdtr",	0,	-1,	ARM11_REGISTER_RDTR},
};

enum arm11_regcache_ids
{
	ARM11_RC_R0,
	ARM11_RC_RX			= ARM11_RC_R0,

	ARM11_RC_R1,
	ARM11_RC_R2,
	ARM11_RC_R3,
	ARM11_RC_R4,
	ARM11_RC_R5,
	ARM11_RC_R6,
	ARM11_RC_R7,
	ARM11_RC_R8,
	ARM11_RC_R9,
	ARM11_RC_R10,
	ARM11_RC_R11,
	ARM11_RC_R12,
	ARM11_RC_R13,
	ARM11_RC_SP			= ARM11_RC_R13,
	ARM11_RC_R14,
	ARM11_RC_LR			= ARM11_RC_R14,
	ARM11_RC_R15,
	ARM11_RC_PC			= ARM11_RC_R15,

	ARM11_RC_CPSR,

	ARM11_RC_DSCR,
	ARM11_RC_WDTR,
	ARM11_RC_RDTR,

	ARM11_RC_MAX,
};

static int arm11_on_enter_debug_state(struct arm11_common *arm11);
static int arm11_step(struct target *target, int current,
		uint32_t address, int handle_breakpoints);
/* helpers */
static int arm11_build_reg_cache(struct target *target);
static int arm11_set_reg(struct reg *reg, uint8_t *buf);
static int arm11_get_reg(struct reg *reg);


/** Check and if necessary take control of the system
 *
 * \param arm11		Target state variable.
 * \param dscr		If the current DSCR content is
 *					available a pointer to a word holding the
 *					DSCR can be passed. Otherwise use NULL.
 */
static int arm11_check_init(struct arm11_common *arm11, uint32_t *dscr)
{
	uint32_t			dscr_local_tmp_copy;

	if (!dscr)
	{
		dscr = &dscr_local_tmp_copy;

		CHECK_RETVAL(arm11_read_DSCR(arm11, dscr));
	}

	if (!(*dscr & ARM11_DSCR_MODE_SELECT))
	{
		LOG_DEBUG("Bringing target into debug mode");

		*dscr |= ARM11_DSCR_MODE_SELECT;		/* Halt debug-mode */
		arm11_write_DSCR(arm11, *dscr);

		/* add further reset initialization here */

		arm11->simulate_reset_on_next_halt = true;

		if (*dscr & ARM11_DSCR_CORE_HALTED)
		{
			/** \todo TODO: this needs further scrutiny because
			  * arm11_on_enter_debug_state() never gets properly called.
			  * As a result we don't read the actual register states from
			  * the target.
			  */

			arm11->target->state	= TARGET_HALTED;
			arm11->target->debug_reason	= arm11_get_DSCR_debug_reason(*dscr);
		}
		else
		{
			arm11->target->state	= TARGET_RUNNING;
			arm11->target->debug_reason	= DBG_REASON_NOTHALTED;
		}

		arm11_sc7_clear_vbw(arm11);
	}

	return ERROR_OK;
}



#define R(x) \
	(arm11->reg_values[ARM11_RC_##x])

/** Save processor state.
  *
  * This is called when the HALT instruction has succeeded
  * or on other occasions that stop the processor.
  *
  */
static int arm11_on_enter_debug_state(struct arm11_common *arm11)
{
	int retval;

	/* REVISIT entire cache should already be invalid !!! */
	register_cache_invalidate(arm11->arm.core_cache);

	for (size_t i = 0; i < ARRAY_SIZE(arm11->reg_values); i++)
	{
		arm11->reg_list[i].valid	= 1;
		arm11->reg_list[i].dirty	= 0;
	}

	/* Save DSCR */
	CHECK_RETVAL(arm11_read_DSCR(arm11, &R(DSCR)));

	/* Save wDTR */

	if (R(DSCR) & ARM11_DSCR_WDTR_FULL)
	{
		arm11_add_debug_SCAN_N(arm11, 0x05, ARM11_TAP_DEFAULT);

		arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

		struct scan_field	chain5_fields[3];

		arm11_setup_field(arm11, 32, NULL, &R(WDTR),	chain5_fields + 0);
		arm11_setup_field(arm11,  1, NULL, NULL,		chain5_fields + 1);
		arm11_setup_field(arm11,  1, NULL, NULL,		chain5_fields + 2);

		arm11_add_dr_scan_vc(ARRAY_SIZE(chain5_fields), chain5_fields, TAP_DRPAUSE);
	}
	else
	{
		arm11->reg_list[ARM11_RC_WDTR].valid	= 0;
	}


	/* DSCR: set ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE */
	/* ARM1176 spec says this is needed only for wDTR/rDTR's "ITR mode", but not to issue ITRs
	   ARM1136 seems to require this to issue ITR's as well */

	uint32_t new_dscr = R(DSCR) | ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE;

	/* this executes JTAG queue: */

	arm11_write_DSCR(arm11, new_dscr);


	/* From the spec:
	   Before executing any instruction in debug state you have to drain the write buffer.
	   This ensures that no imprecise Data Aborts can return at a later point:*/

	/** \todo TODO: Test drain write buffer. */

#if 0
	while (1)
	{
		/* MRC p14,0,R0,c5,c10,0 */
		//	arm11_run_instr_no_data1(arm11, /*0xee150e1a*/0xe320f000);

		/* mcr	   15, 0, r0, cr7, cr10, {4} */
		arm11_run_instr_no_data1(arm11, 0xee070f9a);

		uint32_t dscr = arm11_read_DSCR(arm11);

		LOG_DEBUG("DRAIN, DSCR %08x", dscr);

		if (dscr & ARM11_DSCR_STICKY_IMPRECISE_DATA_ABORT)
		{
			arm11_run_instr_no_data1(arm11, 0xe320f000);

			dscr = arm11_read_DSCR(arm11);

			LOG_DEBUG("DRAIN, DSCR %08x (DONE)", dscr);

			break;
		}
	}
#endif

	retval = arm_dpm_read_current_registers(&arm11->dpm);
	if (retval != ERROR_OK)
		LOG_ERROR("DPM REG READ -- fail %d", retval);

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* save r0 - r14 */

	/** \todo TODO: handle other mode registers */

	for (size_t i = 0; i < 15; i++)
	{
		/* MCR p14,0,R?,c0,c5,0 */
		retval = arm11_run_instr_data_from_core(arm11, 0xEE000E15 | (i << 12), &R(RX + i), 1);
		if (retval != ERROR_OK)
			return retval;
	}

	/* save rDTR */

	/* check rDTRfull in DSCR */

	if (R(DSCR) & ARM11_DSCR_RDTR_FULL)
	{
		/* MRC p14,0,R0,c0,c5,0 (move rDTR -> r0 (-> wDTR -> local var)) */
		retval = arm11_run_instr_data_from_core_via_r0(arm11, 0xEE100E15, &R(RDTR));
		if (retval != ERROR_OK)
			return retval;
	}
	else
	{
		arm11->reg_list[ARM11_RC_RDTR].valid	= 0;
	}

	/* save CPSR */

	/* MRS r0,CPSR (move CPSR -> r0 (-> wDTR -> local var)) */
	retval = arm11_run_instr_data_from_core_via_r0(arm11, 0xE10F0000, &R(CPSR));
	if (retval != ERROR_OK)
		return retval;

	/* save PC */

	/* MOV R0,PC (move PC -> r0 (-> wDTR -> local var)) */
	retval = arm11_run_instr_data_from_core_via_r0(arm11, 0xE1A0000F, &R(PC));
	if (retval != ERROR_OK)
		return retval;

	/* adjust PC depending on ARM state */

	if (R(CPSR) & ARM11_CPSR_J)	/* Java state */
	{
		arm11->reg_values[ARM11_RC_PC] -= 0;
	}
	else if (R(CPSR) & ARM11_CPSR_T)	/* Thumb state */
	{
		arm11->reg_values[ARM11_RC_PC] -= 4;
	}
	else					/* ARM state */
	{
		arm11->reg_values[ARM11_RC_PC] -= 8;
	}

	if (arm11->simulate_reset_on_next_halt)
	{
		arm11->simulate_reset_on_next_halt = false;

		LOG_DEBUG("Reset c1 Control Register");

		/* Write 0 (reset value) to Control register 0 to disable MMU/Cache etc. */

		/* MCR p15,0,R0,c1,c0,0 */
		retval = arm11_run_instr_data_to_core_via_r0(arm11, 0xee010f10, 0);
		if (retval != ERROR_OK)
			return retval;

	}

	retval = arm11_run_instr_data_finish(arm11);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/** Restore processor state
  *
  * This is called in preparation for the RESTART function.
  *
  */
static int arm11_leave_debug_state(struct arm11_common *arm11)
{
	int retval;

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/** \todo TODO: handle other mode registers */

	/* restore R1 - R14 */

	for (unsigned i = 1; i < 15; i++)
	{
		if (!arm11->reg_list[ARM11_RC_RX + i].dirty)
			continue;

		/* MRC p14,0,r?,c0,c5,0 */
		arm11_run_instr_data_to_core1(arm11,
				0xee100e15 | (i << 12), R(RX + i));

		//	LOG_DEBUG("RESTORE R%u %08x", i, R(RX + i));
	}

	retval = arm11_run_instr_data_finish(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* spec says clear wDTR and rDTR; we assume they are clear as
	   otherwise our programming would be sloppy */
	{
		uint32_t DSCR;

		CHECK_RETVAL(arm11_read_DSCR(arm11, &DSCR));

		if (DSCR & (ARM11_DSCR_RDTR_FULL | ARM11_DSCR_WDTR_FULL))
		{
			/*
			The wDTR/rDTR two registers that are used to send/receive data to/from
			the core in tandem with corresponding instruction codes that are
			written into the core. The RDTR FULL/WDTR FULL flag indicates that the
			registers hold data that was written by one side (CPU or JTAG) and not
			read out by the other side.
			*/
			LOG_ERROR("wDTR/rDTR inconsistent (DSCR %08" PRIx32 ")", DSCR);
			return ERROR_FAIL;
		}
	}

/* DEBUG for now, trust "new" code only for shadowed registers */
retval = arm_dpm_write_dirty_registers(&arm11->dpm);

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* restore original wDTR */

	if ((R(DSCR) & ARM11_DSCR_WDTR_FULL) || arm11->reg_list[ARM11_RC_WDTR].dirty)
	{
		/* MCR p14,0,R0,c0,c5,0 */
		retval = arm11_run_instr_data_to_core_via_r0(arm11, 0xee000e15, R(WDTR));
		if (retval != ERROR_OK)
			return retval;
	}

	/* restore CPSR */

	/* MSR CPSR,R0*/
	retval = arm11_run_instr_data_to_core_via_r0(arm11, 0xe129f000, R(CPSR));
	if (retval != ERROR_OK)
		return retval;


	/* restore PC */

	/* MOV PC,R0 */
	retval = arm11_run_instr_data_to_core_via_r0(arm11, 0xe1a0f000, R(PC));
	if (retval != ERROR_OK)
		return retval;


	/* restore R0 */

	/* MRC p14,0,r0,c0,c5,0 */
	arm11_run_instr_data_to_core1(arm11, 0xee100e15, R(R0));

	retval = arm11_run_instr_data_finish(arm11);
	if (retval != ERROR_OK)
		return retval;

/* DEBUG use this when "new" code is really managing core registers */
// retval = arm_dpm_write_dirty_registers(&arm11->dpm);

	register_cache_invalidate(arm11->arm.core_cache);

	/* restore DSCR */

	arm11_write_DSCR(arm11, R(DSCR));

	/* restore rDTR */

	if (R(DSCR) & ARM11_DSCR_RDTR_FULL || arm11->reg_list[ARM11_RC_RDTR].dirty)
	{
		arm11_add_debug_SCAN_N(arm11, 0x05, ARM11_TAP_DEFAULT);

		arm11_add_IR(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

		struct scan_field	chain5_fields[3];

		uint8_t			Ready		= 0;	/* ignored */
		uint8_t			Valid		= 0;	/* ignored */

		arm11_setup_field(arm11, 32, &R(RDTR),	NULL, chain5_fields + 0);
		arm11_setup_field(arm11,  1, &Ready,	NULL, chain5_fields + 1);
		arm11_setup_field(arm11,  1, &Valid,	NULL, chain5_fields + 2);

		arm11_add_dr_scan_vc(ARRAY_SIZE(chain5_fields), chain5_fields, TAP_DRPAUSE);
	}

	return ERROR_OK;
}

/* poll current target status */
static int arm11_poll(struct target *target)
{
	int retval;
	struct arm11_common *arm11 = target_to_arm11(target);
	uint32_t	dscr;

	CHECK_RETVAL(arm11_read_DSCR(arm11, &dscr));

	LOG_DEBUG("DSCR %08" PRIx32 "", dscr);

	CHECK_RETVAL(arm11_check_init(arm11, &dscr));

	if (dscr & ARM11_DSCR_CORE_HALTED)
	{
		if (target->state != TARGET_HALTED)
		{
			enum target_state old_state = target->state;

			LOG_DEBUG("enter TARGET_HALTED");
			target->state			= TARGET_HALTED;
			target->debug_reason	= arm11_get_DSCR_debug_reason(dscr);
			retval = arm11_on_enter_debug_state(arm11);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target,
				old_state == TARGET_DEBUG_RUNNING ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED);
		}
	}
	else
	{
		if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING)
		{
			LOG_DEBUG("enter TARGET_RUNNING");
			target->state			= TARGET_RUNNING;
			target->debug_reason	= DBG_REASON_NOTHALTED;
		}
	}

	return ERROR_OK;
}
/* architecture specific status reply */
static int arm11_arch_state(struct target *target)
{
	int retval;

	retval = armv4_5_arch_state(target);

	/* REVISIT also display ARM11-specific MMU and cache status ... */

	return retval;
}

/* target request support */
static int arm11_target_request_data(struct target *target,
		uint32_t size, uint8_t *buffer)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

/* target execution control */
static int arm11_halt(struct target *target)
{
	struct arm11_common *arm11 = target_to_arm11(target);

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	if (target->state == TARGET_UNKNOWN)
	{
		arm11->simulate_reset_on_next_halt = true;
	}

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	arm11_add_IR(arm11, ARM11_HALT, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	uint32_t dscr;

	int i = 0;
	while (1)
	{
		CHECK_RETVAL(arm11_read_DSCR(arm11, &dscr));

		if (dscr & ARM11_DSCR_CORE_HALTED)
			break;


		long long then = 0;
		if (i == 1000)
		{
			then = timeval_ms();
		}
		if (i >= 1000)
		{
			if ((timeval_ms()-then) > 1000)
			{
				LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
				return ERROR_FAIL;
			}
		}
		i++;
	}

	arm11_on_enter_debug_state(arm11);

	enum target_state old_state	= target->state;

	target->state		= TARGET_HALTED;
	target->debug_reason	= arm11_get_DSCR_debug_reason(dscr);

	CHECK_RETVAL(
		target_call_event_callbacks(target,
			old_state == TARGET_DEBUG_RUNNING ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED));

	return ERROR_OK;
}

static int arm11_resume(struct target *target, int current,
		uint32_t address, int handle_breakpoints, int debug_execution)
{
	//	  LOG_DEBUG("current %d  address %08x  handle_breakpoints %d  debug_execution %d",
	//	current, address, handle_breakpoints, debug_execution);

	struct arm11_common *arm11 = target_to_arm11(target);

	LOG_DEBUG("target->state: %s",
		target_state_name(target));


	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!current)
		R(PC) = address;

	LOG_DEBUG("RESUME PC %08" PRIx32 "%s", R(PC), !current ? "!" : "");

	/* clear breakpoints/watchpoints and VCR*/
	arm11_sc7_clear_vbw(arm11);

	if (!debug_execution)
		target_free_all_working_areas(target);

	/* Set up breakpoints */
	if (handle_breakpoints)
	{
		/* check if one matches PC and step over it if necessary */

		struct breakpoint *	bp;

		for (bp = target->breakpoints; bp; bp = bp->next)
		{
			if (bp->address == R(PC))
			{
				LOG_DEBUG("must step over %08" PRIx32 "", bp->address);
				arm11_step(target, 1, 0, 0);
				break;
			}
		}

		/* set all breakpoints */

		unsigned brp_num = 0;

		for (bp = target->breakpoints; bp; bp = bp->next)
		{
			struct arm11_sc7_action	brp[2];

			brp[0].write	= 1;
			brp[0].address	= ARM11_SC7_BVR0 + brp_num;
			brp[0].value	= bp->address;
			brp[1].write	= 1;
			brp[1].address	= ARM11_SC7_BCR0 + brp_num;
			brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5) | (0 << 14) | (0 << 16) | (0 << 20) | (0 << 21);

			arm11_sc7_run(arm11, brp, ARRAY_SIZE(brp));

			LOG_DEBUG("Add BP %d at %08" PRIx32, brp_num,
					bp->address);

			brp_num++;
		}

		arm11_sc7_set_vcr(arm11, arm11_vcr);
	}

	arm11_leave_debug_state(arm11);

	arm11_add_IR(arm11, ARM11_RESTART, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	int i = 0;
	while (1)
	{
		uint32_t dscr;

		CHECK_RETVAL(arm11_read_DSCR(arm11, &dscr));

		LOG_DEBUG("DSCR %08" PRIx32 "", dscr);

		if (dscr & ARM11_DSCR_CORE_RESTARTED)
			break;


		long long then = 0;
		if (i == 1000)
		{
			then = timeval_ms();
		}
		if (i >= 1000)
		{
			if ((timeval_ms()-then) > 1000)
			{
				LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
				return ERROR_FAIL;
			}
		}
		i++;
	}

	if (!debug_execution)
	{
		target->state			= TARGET_RUNNING;
		target->debug_reason	= DBG_REASON_NOTHALTED;

		CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));
	}
	else
	{
		target->state			= TARGET_DEBUG_RUNNING;
		target->debug_reason	= DBG_REASON_NOTHALTED;

		CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));
	}

	return ERROR_OK;
}

static int arm11_step(struct target *target, int current,
		uint32_t address, int handle_breakpoints)
{
	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct arm11_common *arm11 = target_to_arm11(target);

	if (!current)
		R(PC) = address;

	LOG_DEBUG("STEP PC %08" PRIx32 "%s", R(PC), !current ? "!" : "");


	/** \todo TODO: Thumb not supported here */

	uint32_t	next_instruction;

	CHECK_RETVAL(arm11_read_memory_word(arm11, R(PC), &next_instruction));

	/* skip over BKPT */
	if ((next_instruction & 0xFFF00070) == 0xe1200070)
	{
		R(PC) += 4;
		arm11->reg_list[ARM11_RC_PC].valid = 1;
		arm11->reg_list[ARM11_RC_PC].dirty = 0;
		LOG_DEBUG("Skipping BKPT");
	}
	/* skip over Wait for interrupt / Standby */
	/* mcr	15, 0, r?, cr7, cr0, {4} */
	else if ((next_instruction & 0xFFFF0FFF) == 0xee070f90)
	{
		R(PC) += 4;
		arm11->reg_list[ARM11_RC_PC].valid = 1;
		arm11->reg_list[ARM11_RC_PC].dirty = 0;
		LOG_DEBUG("Skipping WFI");
	}
	/* ignore B to self */
	else if ((next_instruction & 0xFEFFFFFF) == 0xeafffffe)
	{
		LOG_DEBUG("Not stepping jump to self");
	}
	else
	{
		/** \todo TODO: check if break-/watchpoints make any sense at all in combination
		* with this. */

		/** \todo TODO: check if disabling IRQs might be a good idea here. Alternatively
		* the VCR might be something worth looking into. */


		/* Set up breakpoint for stepping */

		struct arm11_sc7_action	brp[2];

		brp[0].write	= 1;
		brp[0].address	= ARM11_SC7_BVR0;
		brp[1].write	= 1;
		brp[1].address	= ARM11_SC7_BCR0;

		if (arm11_config_hardware_step)
		{
			/* Hardware single stepping ("instruction address
			 * mismatch") is used if enabled.  It's not quite
			 * exactly "run one instruction"; "branch to here"
			 * loops won't break, neither will some other cases,
			 * but it's probably the best default.
			 *
			 * Hardware single stepping isn't supported on v6
			 * debug modules.  ARM1176 and v7 can support it...
			 *
			 * FIXME Thumb stepping likely needs to use 0x03
			 * or 0xc0 byte masks, not 0x0f.
			 */
			 brp[0].value	= R(PC);
			 brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5)
					| (0 << 14) | (0 << 16) | (0 << 20)
					| (2 << 21);
		} else
		{
			/* Sets a breakpoint on the next PC, as calculated
			 * by instruction set simulation.
			 *
			 * REVISIT stepping Thumb on ARM1156 requires Thumb2
			 * support from the simulator.
			 */
			uint32_t next_pc;
			int retval;

			retval = arm_simulate_step(target, &next_pc);
			if (retval != ERROR_OK)
				return retval;

			brp[0].value	= next_pc;
			brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5)
					| (0 << 14) | (0 << 16) | (0 << 20)
					| (0 << 21);
		}

		CHECK_RETVAL(arm11_sc7_run(arm11, brp, ARRAY_SIZE(brp)));

		/* resume */


		if (arm11_config_step_irq_enable)
			R(DSCR) &= ~ARM11_DSCR_INTERRUPTS_DISABLE;		/* should be redundant */
		else
			R(DSCR) |= ARM11_DSCR_INTERRUPTS_DISABLE;


		CHECK_RETVAL(arm11_leave_debug_state(arm11));

		arm11_add_IR(arm11, ARM11_RESTART, TAP_IDLE);

		CHECK_RETVAL(jtag_execute_queue());

		/* wait for halt */
		int i = 0;
		while (1)
		{
			uint32_t dscr;

			CHECK_RETVAL(arm11_read_DSCR(arm11, &dscr));

			LOG_DEBUG("DSCR %08" PRIx32 "e", dscr);

			if ((dscr & (ARM11_DSCR_CORE_RESTARTED | ARM11_DSCR_CORE_HALTED)) ==
				(ARM11_DSCR_CORE_RESTARTED | ARM11_DSCR_CORE_HALTED))
				break;

			long long then = 0;
			if (i == 1000)
			{
				then = timeval_ms();
			}
			if (i >= 1000)
			{
				if ((timeval_ms()-then) > 1000)
				{
					LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}
			i++;
		}

		/* clear breakpoint */
		arm11_sc7_clear_vbw(arm11);

		/* save state */
		CHECK_RETVAL(arm11_on_enter_debug_state(arm11));

	    /* restore default state */
		R(DSCR) &= ~ARM11_DSCR_INTERRUPTS_DISABLE;

	}

	//	  target->state		= TARGET_HALTED;
	target->debug_reason	= DBG_REASON_SINGLESTEP;

	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	return ERROR_OK;
}

static int arm11_assert_reset(struct target *target)
{
	int retval;
	struct arm11_common *arm11 = target_to_arm11(target);

	retval = arm11_check_init(arm11, NULL);
	if (retval != ERROR_OK)
		return retval;

	target->state = TARGET_UNKNOWN;

	/* we would very much like to reset into the halted, state,
	 * but resetting and halting is second best... */
	if (target->reset_halt)
	{
		CHECK_RETVAL(target_halt(target));
	}


	/* srst is funny. We can not do *anything* else while it's asserted
	 * and it has unkonwn side effects. Make sure no other code runs
	 * meanwhile.
	 *
	 * Code below assumes srst:
	 *
	 * - Causes power-on-reset (but of what parts of the system?). Bug
	 * in arm11?
	 *
	 * - Messes us TAP state without asserting trst.
	 *
	 * - There is another bug in the arm11 core. When you generate an access to
	 * external logic (for example ddr controller via AHB bus) and that block
	 * is not configured (perhaps it is still held in reset), that transaction
	 * will never complete. This will hang arm11 core but it will also hang
	 * JTAG controller. Nothing, short of srst assertion will bring it out of
	 * this.
	 *
	 * Mysteries:
	 *
	 * - What should the PC be after an srst reset when starting in the halted
	 * state?
	 */

	jtag_add_reset(0, 1);
	jtag_add_reset(0, 0);

	/* How long do we have to wait? */
	jtag_add_sleep(5000);

	/* un-mess up TAP state */
	jtag_add_tlr();

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}

static int arm11_deassert_reset(struct target *target)
{
	return ERROR_OK;
}

static int arm11_soft_reset_halt(struct target *target)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

/* target memory access
 * size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
 * count: number of items of <size>
 *
 * arm11_config_memrw_no_increment - in the future we may want to be able
 * to read/write a range of data to a "port". a "port" is an action on
 * read memory address for some peripheral.
 */
static int arm11_read_memory_inner(struct target *target,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer,
		bool arm11_config_memrw_no_increment)
{
	/** \todo TODO: check if buffer cast to uint32_t* and uint16_t* might cause alignment problems */
	int retval;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("ADDR %08" PRIx32 "  SIZE %08" PRIx32 "  COUNT %08" PRIx32 "", address, size, count);

	struct arm11_common *arm11 = target_to_arm11(target);

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* MRC p14,0,r0,c0,c5,0 */
	retval = arm11_run_instr_data_to_core1(arm11, 0xee100e15, address);
	if (retval != ERROR_OK)
		return retval;

	switch (size)
	{
	case 1:
		/** \todo TODO: check if dirty is the right choice to force a rewrite on arm11_resume() */
		arm11->reg_list[ARM11_RC_R1].dirty = 1;

		for (size_t i = 0; i < count; i++)
		{
			/* ldrb    r1, [r0], #1 */
			/* ldrb    r1, [r0] */
			arm11_run_instr_no_data1(arm11,
					!arm11_config_memrw_no_increment ? 0xe4d01001 : 0xe5d01000);

			uint32_t res;
			/* MCR p14,0,R1,c0,c5,0 */
			arm11_run_instr_data_from_core(arm11, 0xEE001E15, &res, 1);

			*buffer++ = res;
		}

		break;

	case 2:
		{
			arm11->reg_list[ARM11_RC_R1].dirty = 1;

			for (size_t i = 0; i < count; i++)
			{
				/* ldrh    r1, [r0], #2 */
				arm11_run_instr_no_data1(arm11,
					!arm11_config_memrw_no_increment ? 0xe0d010b2 : 0xe1d010b0);

				uint32_t res;

				/* MCR p14,0,R1,c0,c5,0 */
				arm11_run_instr_data_from_core(arm11, 0xEE001E15, &res, 1);

				uint16_t svalue = res;
				memcpy(buffer + i * sizeof(uint16_t), &svalue, sizeof(uint16_t));
			}

			break;
		}

	case 4:
		{
		uint32_t instr = !arm11_config_memrw_no_increment ? 0xecb05e01 : 0xed905e00;
		/** \todo TODO: buffer cast to uint32_t* causes alignment warnings */
		uint32_t *words = (uint32_t *)buffer;

		/* LDC p14,c5,[R0],#4 */
		/* LDC p14,c5,[R0] */
		arm11_run_instr_data_from_core(arm11, instr, words, count);
		break;
		}
	}

	return arm11_run_instr_data_finish(arm11);
}

static int arm11_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	return arm11_read_memory_inner(target, address, size, count, buffer, false);
}

/*
* arm11_config_memrw_no_increment - in the future we may want to be able
* to read/write a range of data to a "port". a "port" is an action on
* read memory address for some peripheral.
*/
static int arm11_write_memory_inner(struct target *target,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer,
		bool arm11_config_memrw_no_increment)
{
	int retval;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("ADDR %08" PRIx32 "  SIZE %08" PRIx32 "  COUNT %08" PRIx32 "", address, size, count);

	struct arm11_common *arm11 = target_to_arm11(target);

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* MRC p14,0,r0,c0,c5,0 */
	retval = arm11_run_instr_data_to_core1(arm11, 0xee100e15, address);
	if (retval != ERROR_OK)
		return retval;

	/* burst writes are not used for single words as those may well be
	 * reset init script writes.
	 *
	 * The other advantage is that as burst writes are default, we'll
	 * now exercise both burst and non-burst code paths with the
	 * default settings, increasing code coverage.
	 */
	bool burst = arm11_config_memwrite_burst && (count > 1);

	switch (size)
	{
	case 1:
		{
			arm11->reg_list[ARM11_RC_R1].dirty = 1;

			for (size_t i = 0; i < count; i++)
			{
				/* MRC p14,0,r1,c0,c5,0 */
				retval = arm11_run_instr_data_to_core1(arm11, 0xee101e15, *buffer++);
				if (retval != ERROR_OK)
					return retval;

				/* strb    r1, [r0], #1 */
				/* strb    r1, [r0] */
				retval = arm11_run_instr_no_data1(arm11,
					!arm11_config_memrw_no_increment ? 0xe4c01001 : 0xe5c01000);
				if (retval != ERROR_OK)
					return retval;
			}

			break;
		}

	case 2:
		{
			arm11->reg_list[ARM11_RC_R1].dirty = 1;

			for (size_t i = 0; i < count; i++)
			{
				uint16_t value;
				memcpy(&value, buffer + i * sizeof(uint16_t), sizeof(uint16_t));

				/* MRC p14,0,r1,c0,c5,0 */
				retval = arm11_run_instr_data_to_core1(arm11, 0xee101e15, value);
				if (retval != ERROR_OK)
					return retval;

				/* strh    r1, [r0], #2 */
				/* strh    r1, [r0] */
				retval = arm11_run_instr_no_data1(arm11,
					!arm11_config_memrw_no_increment ? 0xe0c010b2 : 0xe1c010b0);
				if (retval != ERROR_OK)
					return retval;
			}

			break;
		}

	case 4: {
		uint32_t instr = !arm11_config_memrw_no_increment ? 0xeca05e01 : 0xed805e00;

		/** \todo TODO: buffer cast to uint32_t* causes alignment warnings */
		uint32_t *words = (uint32_t*)buffer;

		if (!burst)
		{
			/* STC p14,c5,[R0],#4 */
			/* STC p14,c5,[R0]*/
			retval = arm11_run_instr_data_to_core(arm11, instr, words, count);
			if (retval != ERROR_OK)
				return retval;
		}
		else
		{
			/* STC p14,c5,[R0],#4 */
			/* STC p14,c5,[R0]*/
			retval = arm11_run_instr_data_to_core_noack(arm11, instr, words, count);
			if (retval != ERROR_OK)
				return retval;
		}

		break;
	}
	}

	/* r0 verification */
	if (!arm11_config_memrw_no_increment)
	{
		uint32_t r0;

		/* MCR p14,0,R0,c0,c5,0 */
		retval = arm11_run_instr_data_from_core(arm11, 0xEE000E15, &r0, 1);
		if (retval != ERROR_OK)
			return retval;

		if (address + size * count != r0)
		{
			LOG_ERROR("Data transfer failed. Expected end "
					"address 0x%08x, got 0x%08x",
					(unsigned) (address + size * count),
					(unsigned) r0);

			if (burst)
				LOG_ERROR("use 'arm11 memwrite burst disable' to disable fast burst mode");

			if (arm11_config_memwrite_error_fatal)
				return ERROR_FAIL;
		}
	}

	return arm11_run_instr_data_finish(arm11);
}

static int arm11_write_memory(struct target *target,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	return arm11_write_memory_inner(target, address, size, count, buffer, false);
}

/* write target memory in multiples of 4 byte, optimized for writing large quantities of data */
static int arm11_bulk_write_memory(struct target *target,
		uint32_t address, uint32_t count, uint8_t *buffer)
{
	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return arm11_write_memory(target, address, 4, count, buffer);
}

/* target break-/watchpoint control
* rw: 0 = write, 1 = read, 2 = access
*/
static int arm11_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct arm11_common *arm11 = target_to_arm11(target);

#if 0
	if (breakpoint->type == BKPT_SOFT)
	{
		LOG_INFO("sw breakpoint requested, but software breakpoints not enabled");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
#endif

	if (!arm11->free_brps)
	{
		LOG_DEBUG("no breakpoint unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->length != 4)
	{
		LOG_DEBUG("only breakpoints of four bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	arm11->free_brps--;

	return ERROR_OK;
}

static int arm11_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct arm11_common *arm11 = target_to_arm11(target);

	arm11->free_brps++;

	return ERROR_OK;
}

static int arm11_add_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

static int arm11_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

static int arm11_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm11_common *arm11;

	if (target->tap == NULL)
		return ERROR_FAIL;

	if (target->tap->ir_length != 5)
	{
		LOG_ERROR("'target arm11' expects IR LENGTH = 5");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	arm11 = calloc(1, sizeof *arm11);
	if (!arm11)
		return ERROR_FAIL;

	armv4_5_init_arch_info(target, &arm11->arm);

	arm11->target = target;

	arm11->jtag_info.tap = target->tap;
	arm11->jtag_info.scann_size = 5;
	arm11->jtag_info.scann_instr = ARM11_SCAN_N;
	/* cur_scan_chain == 0 */
	arm11->jtag_info.intest_instr = ARM11_INTEST;

	return ERROR_OK;
}

static int arm11_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	/* Initialize anything we can set up without talking to the target */

	/* FIXME Switch to use the standard build_reg_cache() not custom
	 * code.  Do it from examine(), after we check whether we're
	 * an arm1176 and thus support the Secure Monitor mode.
	 */
	return arm11_build_reg_cache(target);
}

/* talk to the target and set things up */
static int arm11_examine(struct target *target)
{
	int retval;
	char *type;
	struct arm11_common *arm11 = target_to_arm11(target);
	uint32_t didr, device_id;
	uint8_t implementor;

	/* FIXME split into do-first-time and do-every-time logic ... */

	/* check IDCODE */

	arm11_add_IR(arm11, ARM11_IDCODE, ARM11_TAP_DEFAULT);

	struct scan_field		idcode_field;

	arm11_setup_field(arm11, 32, NULL, &device_id, &idcode_field);

	arm11_add_dr_scan_vc(1, &idcode_field, TAP_DRPAUSE);

	/* check DIDR */

	arm11_add_debug_SCAN_N(arm11, 0x00, ARM11_TAP_DEFAULT);

	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	struct scan_field		chain0_fields[2];

	arm11_setup_field(arm11, 32, NULL, &didr, chain0_fields + 0);
	arm11_setup_field(arm11,  8, NULL, &implementor, chain0_fields + 1);

	arm11_add_dr_scan_vc(ARRAY_SIZE(chain0_fields), chain0_fields, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	switch (device_id & 0x0FFFF000)
	{
	case 0x07B36000:
		type = "ARM1136";
		break;
	case 0x07B56000:
		type = "ARM1156";
		break;
	case 0x07B76000:
		arm11->arm.core_type = ARM_MODE_MON;
		type = "ARM1176";
		break;
	default:
		LOG_ERROR("'target arm11' expects IDCODE 0x*7B*7****");
		return ERROR_FAIL;
	}
	LOG_INFO("found %s", type);

	/* unlikely this could ever fail, but ... */
	switch ((didr >> 16) & 0x0F) {
	case ARM11_DEBUG_V6:
	case ARM11_DEBUG_V61:		/* supports security extensions */
		break;
	default:
		LOG_ERROR("Only ARM v6 and v6.1 debug supported.");
		return ERROR_FAIL;
	}

	arm11->brp = ((didr >> 24) & 0x0F) + 1;
	arm11->wrp = ((didr >> 28) & 0x0F) + 1;

	/** \todo TODO: reserve one brp slot if we allow breakpoints during step */
	arm11->free_brps = arm11->brp;
	arm11->free_wrps = arm11->wrp;

	LOG_DEBUG("IDCODE %08" PRIx32 " IMPLEMENTOR %02x DIDR %08" PRIx32,
			device_id, implementor, didr);

	/* as a side-effect this reads DSCR and thus
	 * clears the ARM11_DSCR_STICKY_PRECISE_DATA_ABORT / Sticky Precise Data Abort Flag
	 * as suggested by the spec.
	 */

	retval = arm11_check_init(arm11, NULL);
	if (retval != ERROR_OK)
		return retval;

	/* Build register cache "late", after target_init(), since we
	 * want to know if this core supports Secure Monitor mode.
	 */
	if (!target_was_examined(target)) {
		arm11_dpm_init(arm11, didr);
		retval = arm_dpm_setup(&arm11->dpm);
	}

	/* ETM on ARM11 still uses original scanchain 6 access mode */
	if (arm11->arm.etm && !target_was_examined(target)) {
		*register_get_last_cache_p(&target->reg_cache) =
			etm_build_reg_cache(target, &arm11->jtag_info,
					arm11->arm.etm);
		retval = etm_setup(target);
	}

	target_set_examined(target);

	return ERROR_OK;
}


/** Load a register that is marked !valid in the register cache */
static int arm11_get_reg(struct reg *reg)
{
	struct target * target = ((struct arm11_reg_state *)reg->arch_info)->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/** \todo TODO: Check this. We assume that all registers are fetched at debug entry. */

#if 0
	struct arm11_common *arm11 = target_to_arm11(target);
	const struct arm11_reg_defs *arm11_reg_info = arm11_reg_defs + ((struct arm11_reg_state *)reg->arch_info)->def_index;
#endif

	return ERROR_OK;
}

/** Change a value in the register cache */
static int arm11_set_reg(struct reg *reg, uint8_t *buf)
{
	struct target *target = ((struct arm11_reg_state *)reg->arch_info)->target;
	struct arm11_common *arm11 = target_to_arm11(target);
//	const struct arm11_reg_defs *arm11_reg_info = arm11_reg_defs + ((struct arm11_reg_state *)reg->arch_info)->def_index;

	arm11->reg_values[((struct arm11_reg_state *)reg->arch_info)->def_index] = buf_get_u32(buf, 0, 32);
	reg->valid	= 1;
	reg->dirty	= 1;

	return ERROR_OK;
}

static const struct reg_arch_type arm11_reg_type = {
	.get = arm11_get_reg,
	.set = arm11_set_reg,
};

static int arm11_build_reg_cache(struct target *target)
{
	struct arm11_common *arm11 = target_to_arm11(target);
	struct reg_cache *cache;
	struct reg *reg_list;
	struct arm11_reg_state *arm11_reg_states;

	cache = calloc(1, sizeof *cache);
	reg_list = calloc(ARM11_REGCACHE_COUNT, sizeof *reg_list);
	arm11_reg_states = calloc(ARM11_REGCACHE_COUNT,
			sizeof *arm11_reg_states);
	if (!cache || !reg_list || !arm11_reg_states) {
		free(cache);
		free(reg_list);
		free(arm11_reg_states);
		return ERROR_FAIL;
	}

	arm11->reg_list	= reg_list;

	/* Build the process context cache */
	cache->name		= "arm11 registers";
	cache->next		= NULL;
	cache->reg_list	= reg_list;
	cache->num_regs	= ARM11_REGCACHE_COUNT;

	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	(*cache_p) = cache;

	arm11->core_cache = cache;
//	  armv7m->process_context = cache;

	size_t i;

	/* Not very elegant assertion */
	if (ARM11_REGCACHE_COUNT != ARRAY_SIZE(arm11->reg_values) ||
		ARM11_REGCACHE_COUNT != ARRAY_SIZE(arm11_reg_defs) ||
		ARM11_REGCACHE_COUNT != ARM11_RC_MAX)
	{
		LOG_ERROR("BUG: arm11->reg_values inconsistent (%d %u %u %d)",
				ARM11_REGCACHE_COUNT,
				(unsigned) ARRAY_SIZE(arm11->reg_values),
				(unsigned) ARRAY_SIZE(arm11_reg_defs),
				ARM11_RC_MAX);
		/* FIXME minimally, use a build_bug_on(X) mechanism;
		 * runtime exit() here is bad!
		 */
		exit(-1);
	}

	for (i = 0; i < ARM11_REGCACHE_COUNT; i++)
	{
		struct reg *						r	= reg_list			+ i;
		const struct arm11_reg_defs *	rd	= arm11_reg_defs	+ i;
		struct arm11_reg_state *			rs	= arm11_reg_states	+ i;

		r->name				= rd->name;
		r->size				= 32;
		r->value			= (uint8_t *)(arm11->reg_values + i);
		r->dirty			= 0;
		r->valid			= 0;
		r->type = &arm11_reg_type;
		r->arch_info		= rs;

		rs->def_index		= i;
		rs->target			= target;
	}

	return ERROR_OK;
}

/* FIXME all these BOOL_WRAPPER things should be modifying
 * per-instance state, not shared state; ditto the vector
 * catch register support.  Scan chains with multiple cores
 * should be able to say "work with this core like this,
 * that core like that".  Example, ARM11 MPCore ...
 */

#define ARM11_BOOL_WRAPPER(name, print_name)	\
		COMMAND_HANDLER(arm11_handle_bool_##name) \
		{ \
			return CALL_COMMAND_HANDLER(handle_command_parse_bool, \
					&arm11_config_##name, print_name); \
		}

ARM11_BOOL_WRAPPER(memwrite_burst, "memory write burst mode")
ARM11_BOOL_WRAPPER(memwrite_error_fatal, "fatal error mode for memory writes")
ARM11_BOOL_WRAPPER(step_irq_enable, "IRQs while stepping")
ARM11_BOOL_WRAPPER(hardware_step, "hardware single step")

COMMAND_HANDLER(arm11_handle_vcr)
{
	switch (CMD_ARGC) {
	case 0:
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], arm11_vcr);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	LOG_INFO("VCR 0x%08" PRIx32 "", arm11_vcr);
	return ERROR_OK;
}

static const uint32_t arm11_coproc_instruction_limits[] =
{
	15,				/* coprocessor */
	7,				/* opcode 1 */
	15,				/* CRn */
	15,				/* CRm */
	7,				/* opcode 2 */
	0xFFFFFFFF,		/* value */
};

static int arm11_mrc_inner(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm,
		uint32_t *value, bool read)
{
	int retval;
	struct arm11_common *arm11 = target_to_arm11(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_FAIL;
	}

	uint32_t instr = 0xEE000010	|
		(cpnum <<  8) |
		(op1 << 21) |
		(CRn << 16) |
		(CRm <<  0) |
		(op2 <<  5);

	if (read)
		instr |= 0x00100000;

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	if (read)
	{
		retval = arm11_run_instr_data_from_core_via_r0(arm11, instr, value);
		if (retval != ERROR_OK)
			return retval;
	}
	else
	{
		retval = arm11_run_instr_data_to_core_via_r0(arm11, instr, *value);
		if (retval != ERROR_OK)
			return retval;
	}

	return arm11_run_instr_data_finish(arm11);
}

static int arm11_mrc(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t *value)
{
	return arm11_mrc_inner(target, cpnum, op1, op2, CRn, CRm, value, true);
}

static int arm11_mcr(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t value)
{
	return arm11_mrc_inner(target, cpnum, op1, op2, CRn, CRm, &value, false);
}

static int arm11_register_commands(struct command_context *cmd_ctx)
{
	struct command *top_cmd, *mw_cmd;

	armv4_5_register_commands(cmd_ctx);

	top_cmd = register_command(cmd_ctx, NULL, "arm11",
			NULL, COMMAND_ANY, NULL);

	/* "hardware_step" is only here to check if the default
	 * simulate + breakpoint implementation is broken.
	 * TEMPORARY! NOT DOCUMENTED!
	 */
	register_command(cmd_ctx, top_cmd, "hardware_step",
			arm11_handle_bool_hardware_step, COMMAND_ANY,
			"DEBUG ONLY - Hardware single stepping"
				" (default: disabled)");

	mw_cmd = register_command(cmd_ctx, top_cmd, "memwrite",
			NULL, COMMAND_ANY, NULL);
	register_command(cmd_ctx, mw_cmd, "burst",
			arm11_handle_bool_memwrite_burst, COMMAND_ANY,
			"Enable/Disable non-standard but fast burst mode"
				" (default: enabled)");
	register_command(cmd_ctx, mw_cmd, "error_fatal",
			arm11_handle_bool_memwrite_error_fatal, COMMAND_ANY,
			"Terminate program if transfer error was found"
				" (default: enabled)");

	register_command(cmd_ctx, top_cmd, "step_irq_enable",
			arm11_handle_bool_step_irq_enable, COMMAND_ANY,
			"Enable interrupts while stepping"
				" (default: disabled)");
	register_command(cmd_ctx, top_cmd, "vcr",
			arm11_handle_vcr, COMMAND_ANY,
			"Control (Interrupt) Vector Catch Register");

	return etm_register_commands(cmd_ctx);
}

/** Holds methods for ARM11xx targets. */
struct target_type arm11_target = {
	.name =			"arm11",

	.poll =			arm11_poll,
	.arch_state =		arm11_arch_state,

	.target_request_data =	arm11_target_request_data,

	.halt =			arm11_halt,
	.resume =		arm11_resume,
	.step =			arm11_step,

	.assert_reset =		arm11_assert_reset,
	.deassert_reset =	arm11_deassert_reset,
	.soft_reset_halt =	arm11_soft_reset_halt,

	.get_gdb_reg_list =	armv4_5_get_gdb_reg_list,

	.read_memory =		arm11_read_memory,
	.write_memory =		arm11_write_memory,

	.bulk_write_memory =	arm11_bulk_write_memory,

	.checksum_memory =	arm_checksum_memory,
	.blank_check_memory =	arm_blank_check_memory,

	.add_breakpoint =	arm11_add_breakpoint,
	.remove_breakpoint =	arm11_remove_breakpoint,
	.add_watchpoint =	arm11_add_watchpoint,
	.remove_watchpoint =	arm11_remove_watchpoint,

	.run_algorithm =	armv4_5_run_algorithm,

	.register_commands =	arm11_register_commands,
	.target_create =	arm11_target_create,
	.init_target =		arm11_init_target,
	.examine =		arm11_examine,

	.mrc =			arm11_mrc,
	.mcr =			arm11_mcr,
};

/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
 *                                                                         *
 *   Copyright (C) 2008 Oyvind Harboe oyvind.harboe@zylin.com              *
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

#include "arm11.h"
#include "jtag.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

#if 0
#define FNC_INFO	LOG_DEBUG("-")
#else
#define FNC_INFO
#endif

#if 1
#define FNC_INFO_NOTIMPLEMENTED do { LOG_DEBUG("NOT IMPLEMENTED"); /*exit(-1);*/ } while (0)
#else
#define FNC_INFO_NOTIMPLEMENTED
#endif

static void arm11_on_enter_debug_state(arm11_common_t * arm11);

bool	arm11_config_memwrite_burst		= true;
bool	arm11_config_memwrite_error_fatal	= true;
u32	arm11_vcr				= 0;

#define ARM11_HANDLER(x)	\
	.x				= arm11_##x

target_type_t arm11_target =
{
	.name			= "arm11",

	ARM11_HANDLER(poll),
	ARM11_HANDLER(arch_state),

	ARM11_HANDLER(target_request_data),

	ARM11_HANDLER(halt),
	ARM11_HANDLER(resume),
	ARM11_HANDLER(step),

	ARM11_HANDLER(assert_reset),
	ARM11_HANDLER(deassert_reset),
	ARM11_HANDLER(soft_reset_halt),

	ARM11_HANDLER(get_gdb_reg_list),

	ARM11_HANDLER(read_memory),
	ARM11_HANDLER(write_memory),

	ARM11_HANDLER(bulk_write_memory),

	ARM11_HANDLER(checksum_memory),

	ARM11_HANDLER(add_breakpoint),
	ARM11_HANDLER(remove_breakpoint),
	ARM11_HANDLER(add_watchpoint),
	ARM11_HANDLER(remove_watchpoint),

	ARM11_HANDLER(run_algorithm),

	ARM11_HANDLER(register_commands),
	ARM11_HANDLER(target_create),
	ARM11_HANDLER(init_target),
	ARM11_HANDLER(examine),
	ARM11_HANDLER(quit),
};

int arm11_regs_arch_type = -1;


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


typedef struct arm11_reg_defs_s
{
	char *			name;
	u32				num;
	int				gdb_num;
	enum arm11_regtype		type;
} arm11_reg_defs_t;

/* update arm11_regcache_ids when changing this */
static const arm11_reg_defs_t arm11_reg_defs[] =
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

#if ARM11_REGCACHE_FREGS
	{"f0",	0,	16,	ARM11_REGISTER_FX},
	{"f1",	1,	17,	ARM11_REGISTER_FX},
	{"f2",	2,	18,	ARM11_REGISTER_FX},
	{"f3",	3,	19,	ARM11_REGISTER_FX},
	{"f4",	4,	20,	ARM11_REGISTER_FX},
	{"f5",	5,	21,	ARM11_REGISTER_FX},
	{"f6",	6,	22,	ARM11_REGISTER_FX},
	{"f7",	7,	23,	ARM11_REGISTER_FX},
	{"fps",	0,	24,	ARM11_REGISTER_FPS},
#endif

	{"cpsr",	0,	25,	ARM11_REGISTER_CPSR},

#if ARM11_REGCACHE_MODEREGS
	{"r8_fiq",	8,	-1,	ARM11_REGISTER_FIQ},
	{"r9_fiq",	9,	-1,	ARM11_REGISTER_FIQ},
	{"r10_fiq",	10,	-1,	ARM11_REGISTER_FIQ},
	{"r11_fiq",	11,	-1,	ARM11_REGISTER_FIQ},
	{"r12_fiq",	12,	-1,	ARM11_REGISTER_FIQ},
	{"r13_fiq",	13,	-1,	ARM11_REGISTER_FIQ},
	{"r14_fiq",	14,	-1,	ARM11_REGISTER_FIQ},
	{"spsr_fiq", 0,	-1,	ARM11_REGISTER_SPSR_FIQ},

	{"r13_svc",	13,	-1,	ARM11_REGISTER_SVC},
	{"r14_svc",	14,	-1,	ARM11_REGISTER_SVC},
	{"spsr_svc", 0,	-1,	ARM11_REGISTER_SPSR_SVC},

	{"r13_abt",	13,	-1,	ARM11_REGISTER_ABT},
	{"r14_abt",	14,	-1,	ARM11_REGISTER_ABT},
	{"spsr_abt", 0,	-1,	ARM11_REGISTER_SPSR_ABT},

	{"r13_irq",	13,	-1,	ARM11_REGISTER_IRQ},
	{"r14_irq",	14,	-1,	ARM11_REGISTER_IRQ},
	{"spsr_irq", 0,	-1,	ARM11_REGISTER_SPSR_IRQ},

	{"r13_und",	13,	-1,	ARM11_REGISTER_UND},
	{"r14_und",	14,	-1,	ARM11_REGISTER_UND},
	{"spsr_und", 0,	-1,	ARM11_REGISTER_SPSR_UND},

	/* ARM1176 only */
	{"r13_mon",	13,	-1,	ARM11_REGISTER_MON},
	{"r14_mon",	14,	-1,	ARM11_REGISTER_MON},
	{"spsr_mon", 0,	-1,	ARM11_REGISTER_SPSR_MON},
#endif

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

#if ARM11_REGCACHE_FREGS
	ARM11_RC_F0,
	ARM11_RC_FX			= ARM11_RC_F0,
	ARM11_RC_F1,
	ARM11_RC_F2,
	ARM11_RC_F3,
	ARM11_RC_F4,
	ARM11_RC_F5,
	ARM11_RC_F6,
	ARM11_RC_F7,
	ARM11_RC_FPS,
#endif

	ARM11_RC_CPSR,

#if ARM11_REGCACHE_MODEREGS
	ARM11_RC_R8_FIQ,
	ARM11_RC_R9_FIQ,
	ARM11_RC_R10_FIQ,
	ARM11_RC_R11_FIQ,
	ARM11_RC_R12_FIQ,
	ARM11_RC_R13_FIQ,
	ARM11_RC_R14_FIQ,
	ARM11_RC_SPSR_FIQ,

	ARM11_RC_R13_SVC,
	ARM11_RC_R14_SVC,
	ARM11_RC_SPSR_SVC,

	ARM11_RC_R13_ABT,
	ARM11_RC_R14_ABT,
	ARM11_RC_SPSR_ABT,

	ARM11_RC_R13_IRQ,
	ARM11_RC_R14_IRQ,
	ARM11_RC_SPSR_IRQ,

	ARM11_RC_R13_UND,
	ARM11_RC_R14_UND,
	ARM11_RC_SPSR_UND,

	ARM11_RC_R13_MON,
	ARM11_RC_R14_MON,
	ARM11_RC_SPSR_MON,
#endif

	ARM11_RC_DSCR,
	ARM11_RC_WDTR,
	ARM11_RC_RDTR,

	ARM11_RC_MAX,
};

#define ARM11_GDB_REGISTER_COUNT	26

u8 arm11_gdb_dummy_fp_value[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

reg_t arm11_gdb_dummy_fp_reg =
{
	"GDB dummy floating-point register", arm11_gdb_dummy_fp_value, 0, 1, 96, NULL, 0, NULL, 0
};

u8 arm11_gdb_dummy_fps_value[] = {0, 0, 0, 0};

reg_t arm11_gdb_dummy_fps_reg =
{
	"GDB dummy floating-point status register", arm11_gdb_dummy_fps_value, 0, 1, 32, NULL, 0, NULL, 0
};



/** Check and if necessary take control of the system
 *
 * \param arm11		Target state variable.
 * \param dscr		If the current DSCR content is
 *				available a pointer to a word holding the
 *				DSCR can be passed. Otherwise use NULL.
 */
void arm11_check_init(arm11_common_t * arm11, u32 * dscr)
{
	FNC_INFO;

	u32			dscr_local_tmp_copy;

	if (!dscr)
	{
	dscr = &dscr_local_tmp_copy;
	*dscr = arm11_read_DSCR(arm11);
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
		  * arm11_on_enter_debug_state() never gets properly called
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
}



#define R(x) \
	(arm11->reg_values[ARM11_RC_##x])

/** Save processor state.
  *
  * This is called when the HALT instruction has succeeded
  * or on other occasions that stop the processor.
  *
  */
static void arm11_on_enter_debug_state(arm11_common_t * arm11)
{
	FNC_INFO;

	{size_t i;
	for(i = 0; i < asizeof(arm11->reg_values); i++)
	{
	arm11->reg_list[i].valid	= 1;
	arm11->reg_list[i].dirty	= 0;
	}}

	/* Save DSCR */

	R(DSCR) = arm11_read_DSCR(arm11);

	/* Save wDTR */

	if (R(DSCR) & ARM11_DSCR_WDTR_FULL)
	{
	arm11_add_debug_SCAN_N(arm11, 0x05, -1);

	arm11_add_IR(arm11, ARM11_INTEST, -1);

	scan_field_t	chain5_fields[3];

	arm11_setup_field(arm11, 32, NULL, &R(WDTR),	chain5_fields + 0);
	arm11_setup_field(arm11,  1, NULL, NULL,	chain5_fields + 1);
	arm11_setup_field(arm11,  1, NULL, NULL,	chain5_fields + 2);

	arm11_add_dr_scan_vc(asizeof(chain5_fields), chain5_fields, TAP_DRPAUSE);
	}
	else
	{
	arm11->reg_list[ARM11_RC_WDTR].valid	= 0;
	}


	/* DSCR: set ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE */
	/* ARM1176 spec says this is needed only for wDTR/rDTR's "ITR mode", but not to issue ITRs
	   ARM1136 seems to require this to issue ITR's as well */

	u32 new_dscr = R(DSCR) | ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE;

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

	u32 dscr = arm11_read_DSCR(arm11);

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

	arm11_run_instr_data_prepare(arm11);

	/* save r0 - r14 */

	/** \todo TODO: handle other mode registers */

	{size_t i;
	for (i = 0; i < 15; i++)
	{
	/* MCR p14,0,R?,c0,c5,0 */
	arm11_run_instr_data_from_core(arm11, 0xEE000E15 | (i << 12), &R(RX + i), 1);
	}}

	/* save rDTR */

	/* check rDTRfull in DSCR */

	if (R(DSCR) & ARM11_DSCR_RDTR_FULL)
	{
	/* MRC p14,0,R0,c0,c5,0 (move rDTR -> r0 (-> wDTR -> local var)) */
	arm11_run_instr_data_from_core_via_r0(arm11, 0xEE100E15, &R(RDTR));
	}
	else
	{
	arm11->reg_list[ARM11_RC_RDTR].valid	= 0;
	}

	/* save CPSR */

	/* MRS r0,CPSR (move CPSR -> r0 (-> wDTR -> local var)) */
	arm11_run_instr_data_from_core_via_r0(arm11, 0xE10F0000, &R(CPSR));

	/* save PC */

	/* MOV R0,PC (move PC -> r0 (-> wDTR -> local var)) */
	arm11_run_instr_data_from_core_via_r0(arm11, 0xE1A0000F, &R(PC));

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
	arm11_run_instr_data_to_core_via_r0(arm11, 0xee010f10, 0);

	}

	arm11_run_instr_data_finish(arm11);

	arm11_dump_reg_changes(arm11);
}

void arm11_dump_reg_changes(arm11_common_t * arm11)
{
	{size_t i;
	for(i = 0; i < ARM11_REGCACHE_COUNT; i++)
	{
	if (!arm11->reg_list[i].valid)
	{
		if (arm11->reg_history[i].valid)
		LOG_INFO("%8s INVALID	 (%08x)", arm11_reg_defs[i].name, arm11->reg_history[i].value);
	}
	else
	{
		if (arm11->reg_history[i].valid)
		{
		if (arm11->reg_history[i].value != arm11->reg_values[i])
			LOG_INFO("%8s %08x (%08x)", arm11_reg_defs[i].name, arm11->reg_values[i], arm11->reg_history[i].value);
		}
		else
		{
		LOG_INFO("%8s %08x (INVALID)", arm11_reg_defs[i].name, arm11->reg_values[i]);
		}
	}
	}}
}

/** Restore processor state
  *
  * This is called in preparation for the RESTART function.
  *
  */
void arm11_leave_debug_state(arm11_common_t * arm11)
{
	FNC_INFO;

	arm11_run_instr_data_prepare(arm11);

	/** \todo TODO: handle other mode registers */

	/* restore R1 - R14 */
	{size_t i;
	for (i = 1; i < 15; i++)
	{
	if (!arm11->reg_list[ARM11_RC_RX + i].dirty)
		continue;

	/* MRC p14,0,r?,c0,c5,0 */
	arm11_run_instr_data_to_core1(arm11, 0xee100e15 | (i << 12), R(RX + i));

//	LOG_DEBUG("RESTORE R" ZU " %08x", i, R(RX + i));
	}}

	arm11_run_instr_data_finish(arm11);

	/* spec says clear wDTR and rDTR; we assume they are clear as
	   otherwise our programming would be sloppy */

	{
	u32 DSCR = arm11_read_DSCR(arm11);

	if (DSCR & (ARM11_DSCR_RDTR_FULL | ARM11_DSCR_WDTR_FULL))
	{
		LOG_ERROR("wDTR/rDTR inconsistent (DSCR %08x)", DSCR);
	}
	}

	arm11_run_instr_data_prepare(arm11);

	/* restore original wDTR */

	if ((R(DSCR) & ARM11_DSCR_WDTR_FULL) || arm11->reg_list[ARM11_RC_WDTR].dirty)
	{
	/* MCR p14,0,R0,c0,c5,0 */
	arm11_run_instr_data_to_core_via_r0(arm11, 0xee000e15, R(WDTR));
	}

	/* restore CPSR */

	/* MSR CPSR,R0*/
	arm11_run_instr_data_to_core_via_r0(arm11, 0xe129f000, R(CPSR));

	/* restore PC */

	/* MOV PC,R0 */
	arm11_run_instr_data_to_core_via_r0(arm11, 0xe1a0f000, R(PC));

	/* restore R0 */

	/* MRC p14,0,r0,c0,c5,0 */
	arm11_run_instr_data_to_core1(arm11, 0xee100e15, R(R0));

	arm11_run_instr_data_finish(arm11);

	/* restore DSCR */

	arm11_write_DSCR(arm11, R(DSCR));

	/* restore rDTR */

	if (R(DSCR) & ARM11_DSCR_RDTR_FULL || arm11->reg_list[ARM11_RC_RDTR].dirty)
	{
	arm11_add_debug_SCAN_N(arm11, 0x05, -1);

	arm11_add_IR(arm11, ARM11_EXTEST, -1);

	scan_field_t	chain5_fields[3];

	u8			Ready		= 0;	/* ignored */
	u8			Valid		= 0;	/* ignored */

	arm11_setup_field(arm11, 32, &R(RDTR),	NULL, chain5_fields + 0);
	arm11_setup_field(arm11,  1, &Ready,	NULL, chain5_fields + 1);
	arm11_setup_field(arm11,  1, &Valid,	NULL, chain5_fields + 2);

	arm11_add_dr_scan_vc(asizeof(chain5_fields), chain5_fields, TAP_DRPAUSE);
	}

	arm11_record_register_history(arm11);
}

void arm11_record_register_history(arm11_common_t * arm11)
{
	{size_t i;
	for(i = 0; i < ARM11_REGCACHE_COUNT; i++)
	{
	arm11->reg_history[i].value	= arm11->reg_values[i];
	arm11->reg_history[i].valid	= arm11->reg_list[i].valid;

	arm11->reg_list[i].valid	= 0;
	arm11->reg_list[i].dirty	= 0;
	}}
}


/* poll current target status */
int arm11_poll(struct target_s *target)
{
	FNC_INFO;

	arm11_common_t * arm11 = target->arch_info;

	if (arm11->trst_active)
	return ERROR_OK;

	u32	dscr = arm11_read_DSCR(arm11);

	LOG_DEBUG("DSCR %08x", dscr);

	arm11_check_init(arm11, &dscr);

	if (dscr & ARM11_DSCR_CORE_HALTED)
	{
	if (target->state != TARGET_HALTED)
	{
		enum target_state old_state = target->state;

		LOG_DEBUG("enter TARGET_HALTED");
		target->state		= TARGET_HALTED;
		target->debug_reason	= arm11_get_DSCR_debug_reason(dscr);
		arm11_on_enter_debug_state(arm11);

		target_call_event_callbacks(target,
		old_state == TARGET_DEBUG_RUNNING ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED);
	}
	}
	else
	{
	if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING)
	{
		LOG_DEBUG("enter TARGET_RUNNING");
		target->state		= TARGET_RUNNING;
		target->debug_reason	= DBG_REASON_NOTHALTED;
	}
	}

	return ERROR_OK;
}
/* architecture specific status reply */
int arm11_arch_state(struct target_s *target)
{
	FNC_INFO_NOTIMPLEMENTED;

	return ERROR_OK;
}

/* target request support */
int arm11_target_request_data(struct target_s *target, u32 size, u8 *buffer)
{
	FNC_INFO_NOTIMPLEMENTED;

	return ERROR_OK;
}

/* target execution control */
int arm11_halt(struct target_s *target)
{
	int retval = ERROR_OK;

	FNC_INFO;

	arm11_common_t * arm11 = target->arch_info;

	LOG_DEBUG("target->state: %s",
		  Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name );

	if (target->state == TARGET_UNKNOWN)
	{
	arm11->simulate_reset_on_next_halt = true;
	}

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (arm11->trst_active)
	{
	arm11->halt_requested = true;
	return ERROR_OK;
	}

	arm11_add_IR(arm11, ARM11_HALT, TAP_IDLE);

	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	u32 dscr;

	while (1)
	{
	dscr = arm11_read_DSCR(arm11);

	if (dscr & ARM11_DSCR_CORE_HALTED)
		break;
	}

	arm11_on_enter_debug_state(arm11);

	enum target_state old_state	= target->state;

	target->state		= TARGET_HALTED;
	target->debug_reason	= arm11_get_DSCR_debug_reason(dscr);

	if((retval = target_call_event_callbacks(target,
			old_state == TARGET_DEBUG_RUNNING ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED)) != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}

int arm11_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	int retval = ERROR_OK;

	FNC_INFO;

//	  LOG_DEBUG("current %d  address %08x  handle_breakpoints %d  debug_execution %d",
//	current, address, handle_breakpoints, debug_execution);

	arm11_common_t * arm11 = target->arch_info;

	LOG_DEBUG("target->state: %s",
		  Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name );


	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!current)
	R(PC) = address;

	LOG_INFO("RESUME PC %08x%s", R(PC), !current ? "!" : "");

	/* clear breakpoints/watchpoints and VCR*/
	arm11_sc7_clear_vbw(arm11);

	/* Set up breakpoints */
	if (!debug_execution)
	{
	/* check if one matches PC and step over it if necessary */

	breakpoint_t *	bp;

	for (bp = target->breakpoints; bp; bp = bp->next)
	{
		if (bp->address == R(PC))
		{
		LOG_DEBUG("must step over %08x", bp->address);
		arm11_step(target, 1, 0, 0);
		break;
		}
	}

	/* set all breakpoints */

	size_t		brp_num = 0;

	for (bp = target->breakpoints; bp; bp = bp->next)
	{
		arm11_sc7_action_t	brp[2];

		brp[0].write	= 1;
		brp[0].address	= ARM11_SC7_BVR0 + brp_num;
		brp[0].value	= bp->address;
		brp[1].write	= 1;
		brp[1].address	= ARM11_SC7_BCR0 + brp_num;
		brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5) | (0 << 14) | (0 << 16) | (0 << 20) | (0 << 21);

		arm11_sc7_run(arm11, brp, asizeof(brp));

		LOG_DEBUG("Add BP " ZU " at %08x", brp_num, bp->address);

		brp_num++;
	}

	arm11_sc7_set_vcr(arm11, arm11_vcr);
	}

	arm11_leave_debug_state(arm11);

	arm11_add_IR(arm11, ARM11_RESTART, TAP_IDLE);

	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	while (1)
	{
	u32 dscr = arm11_read_DSCR(arm11);

	LOG_DEBUG("DSCR %08x", dscr);

	if (dscr & ARM11_DSCR_CORE_RESTARTED)
		break;
	}

	if (!debug_execution)
	{
		target->state		= TARGET_RUNNING;
		target->debug_reason	= DBG_REASON_NOTHALTED;
		if((retval = target_call_event_callbacks(target, TARGET_EVENT_RESUMED)) != ERROR_OK)
		{
			return retval;
		}
	}
	else
	{
		target->state		= TARGET_DEBUG_RUNNING;
		target->debug_reason	= DBG_REASON_NOTHALTED;
		if((retval = target_call_event_callbacks(target, TARGET_EVENT_RESUMED)) != ERROR_OK)
		{
			return retval;
		}
	}

	return ERROR_OK;
}

int arm11_step(struct target_s *target, int current, u32 address, int handle_breakpoints)
{
	int retval = ERROR_OK;

	FNC_INFO;

	LOG_DEBUG("target->state: %s",
		  Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name );

	if (target->state != TARGET_HALTED)
	{
	LOG_WARNING("target was not halted");
	return ERROR_TARGET_NOT_HALTED;
	}

	arm11_common_t * arm11 = target->arch_info;

	if (!current)
	R(PC) = address;

	LOG_INFO("STEP PC %08x%s", R(PC), !current ? "!" : "");

	/** \todo TODO: Thumb not supported here */

	u32	next_instruction;

	arm11_read_memory_word(arm11, R(PC), &next_instruction);

	/* skip over BKPT */
	if ((next_instruction & 0xFFF00070) == 0xe1200070)
	{
	R(PC) += 4;
	arm11->reg_list[ARM11_RC_PC].valid = 1;
	arm11->reg_list[ARM11_RC_PC].dirty = 0;
	LOG_INFO("Skipping BKPT");
	}
	/* skip over Wait for interrupt / Standby */
	/* mcr	15, 0, r?, cr7, cr0, {4} */
	else if ((next_instruction & 0xFFFF0FFF) == 0xee070f90)
	{
	R(PC) += 4;
	arm11->reg_list[ARM11_RC_PC].valid = 1;
	arm11->reg_list[ARM11_RC_PC].dirty = 0;
	LOG_INFO("Skipping WFI");
	}
	/* ignore B to self */
	else if ((next_instruction & 0xFEFFFFFF) == 0xeafffffe)
	{
	LOG_INFO("Not stepping jump to self");
	}
	else
	{
	/** \todo TODO: check if break-/watchpoints make any sense at all in combination
	  * with this. */

	/** \todo TODO: check if disabling IRQs might be a good idea here. Alternatively
	  * the VCR might be something worth looking into. */


	/* Set up breakpoint for stepping */

	arm11_sc7_action_t	brp[2];

	brp[0].write	= 1;
	brp[0].address	= ARM11_SC7_BVR0;
	brp[0].value	= R(PC);
	brp[1].write	= 1;
	brp[1].address	= ARM11_SC7_BCR0;
	brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5) | (0 << 14) | (0 << 16) | (0 << 20) | (2 << 21);

	arm11_sc7_run(arm11, brp, asizeof(brp));

	/* resume */

	arm11_leave_debug_state(arm11);

	arm11_add_IR(arm11, ARM11_RESTART, TAP_IDLE);

	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	/** \todo TODO: add a timeout */

	/* wait for halt */

	while (1)
	{
		u32 dscr = arm11_read_DSCR(arm11);

		LOG_DEBUG("DSCR %08x", dscr);

		if ((dscr & (ARM11_DSCR_CORE_RESTARTED | ARM11_DSCR_CORE_HALTED)) ==
		(ARM11_DSCR_CORE_RESTARTED | ARM11_DSCR_CORE_HALTED))
		break;
	}

	/* clear breakpoint */
	arm11_sc7_clear_vbw(arm11);

	/* save state */
	arm11_on_enter_debug_state(arm11);
	}

//	  target->state		= TARGET_HALTED;
	target->debug_reason	= DBG_REASON_SINGLESTEP;

	if((retval = target_call_event_callbacks(target, TARGET_EVENT_HALTED)) != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}

/* target reset control */
int arm11_assert_reset(struct target_s *target)
{
	FNC_INFO;

#if 0
	/* assert reset lines */
	/* resets only the DBGTAP, not the ARM */

	jtag_add_reset(1, 0);
	jtag_add_sleep(5000);

	arm11_common_t * arm11 = target->arch_info;
	arm11->trst_active = true;
#endif

	if (target->reset_halt)
	{
		int retval;
		if ((retval = target_halt(target))!=ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int arm11_deassert_reset(struct target_s *target)
{
	FNC_INFO;

#if 0
	LOG_DEBUG("target->state: %s",
		  Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name );


	/* deassert reset lines */
	jtag_add_reset(0, 0);

	arm11_common_t * arm11 = target->arch_info;
	arm11->trst_active = false;

	if (arm11->halt_requested)
	return arm11_halt(target);
#endif

	return ERROR_OK;
}

int arm11_soft_reset_halt(struct target_s *target)
{
	FNC_INFO_NOTIMPLEMENTED;

	return ERROR_OK;
}

/* target register access for gdb */
int arm11_get_gdb_reg_list(struct target_s *target, struct reg_s **reg_list[], int *reg_list_size)
{
	FNC_INFO;

	arm11_common_t * arm11 = target->arch_info;

	*reg_list_size	= ARM11_GDB_REGISTER_COUNT;
	*reg_list		= malloc(sizeof(reg_t*) * ARM11_GDB_REGISTER_COUNT);

	{size_t i;
	for (i = 16; i < 24; i++)
	{
	(*reg_list)[i] = &arm11_gdb_dummy_fp_reg;
	}}

	(*reg_list)[24] = &arm11_gdb_dummy_fps_reg;

	{size_t i;
	for (i = 0; i < ARM11_REGCACHE_COUNT; i++)
	{
	if (arm11_reg_defs[i].gdb_num == -1)
		continue;

	(*reg_list)[arm11_reg_defs[i].gdb_num] = arm11->reg_list + i;
	}}

	return ERROR_OK;
}

/* target memory access
* size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
* count: number of items of <size>
*/
int arm11_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	/** \todo TODO: check if buffer cast to u32* and u16* might cause alignment problems */

	FNC_INFO;

	if (target->state != TARGET_HALTED)
	{
	LOG_WARNING("target was not halted");
	return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("ADDR %08x  SIZE %08x  COUNT %08x", address, size, count);

	arm11_common_t * arm11 = target->arch_info;

	arm11_run_instr_data_prepare(arm11);

	/* MRC p14,0,r0,c0,c5,0 */
	arm11_run_instr_data_to_core1(arm11, 0xee100e15, address);

	switch (size)
	{
	case 1:
	/** \todo TODO: check if dirty is the right choice to force a rewrite on arm11_resume() */
	arm11->reg_list[ARM11_RC_R1].dirty = 1;

	{size_t i;
	for (i = 0; i < count; i++)
	{
		/* ldrb    r1, [r0], #1 */
		arm11_run_instr_no_data1(arm11, 0xe4d01001);

		u32 res;
		/* MCR p14,0,R1,c0,c5,0 */
		arm11_run_instr_data_from_core(arm11, 0xEE001E15, &res, 1);

		*buffer++ = res;
	}}

	break;

	case 2:
	{
	arm11->reg_list[ARM11_RC_R1].dirty = 1;

	u16 * buf16 = (u16*)buffer;

	{size_t i;
	for (i = 0; i < count; i++)
	{
		/* ldrh    r1, [r0], #2 */
		arm11_run_instr_no_data1(arm11, 0xe0d010b2);

		u32 res;

		/* MCR p14,0,R1,c0,c5,0 */
		arm11_run_instr_data_from_core(arm11, 0xEE001E15, &res, 1);

		*buf16++ = res;
	}}

	break;
	}

	case 4:

	/* LDC p14,c5,[R0],#4 */
	arm11_run_instr_data_from_core(arm11, 0xecb05e01, (u32 *)buffer, count);
	break;
	}

	arm11_run_instr_data_finish(arm11);

	return ERROR_OK;
}

int arm11_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	FNC_INFO;

	if (target->state != TARGET_HALTED)
	{
	LOG_WARNING("target was not halted");
	return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("ADDR %08x  SIZE %08x  COUNT %08x", address, size, count);

	arm11_common_t * arm11 = target->arch_info;

	arm11_run_instr_data_prepare(arm11);

	/* MRC p14,0,r0,c0,c5,0 */
	arm11_run_instr_data_to_core1(arm11, 0xee100e15, address);

	switch (size)
	{
	case 1:
	{
	arm11->reg_list[ARM11_RC_R1].dirty = 1;

	{size_t i;
	for (i = 0; i < count; i++)
	{
		/* MRC p14,0,r1,c0,c5,0 */
		arm11_run_instr_data_to_core1(arm11, 0xee101e15, *buffer++);

		/* strb    r1, [r0], #1 */
		arm11_run_instr_no_data1(arm11, 0xe4c01001);
	}}

	break;
	}

	case 2:
	{
	arm11->reg_list[ARM11_RC_R1].dirty = 1;

	u16 * buf16 = (u16*)buffer;

	{size_t i;
	for (i = 0; i < count; i++)
	{
		/* MRC p14,0,r1,c0,c5,0 */
		arm11_run_instr_data_to_core1(arm11, 0xee101e15, *buf16++);

		/* strh    r1, [r0], #2 */
		arm11_run_instr_no_data1(arm11, 0xe0c010b2);
	}}

	break;
	}

	case 4:
	/** \todo TODO: check if buffer cast to u32* might cause alignment problems */

	if (!arm11_config_memwrite_burst)
	{
		/* STC p14,c5,[R0],#4 */
		arm11_run_instr_data_to_core(arm11, 0xeca05e01, (u32 *)buffer, count);
	}
	else
	{
		/* STC p14,c5,[R0],#4 */
		arm11_run_instr_data_to_core_noack(arm11, 0xeca05e01, (u32 *)buffer, count);
	}

	break;
	}

#if 1
	/* r0 verification */
	{
	u32 r0;

	/* MCR p14,0,R0,c0,c5,0 */
	arm11_run_instr_data_from_core(arm11, 0xEE000E15, &r0, 1);

	if (address + size * count != r0)
	{
		LOG_ERROR("Data transfer failed. (%d)", (r0 - address) - size * count);

		if (arm11_config_memwrite_burst)
		LOG_ERROR("use 'arm11 memwrite burst disable' to disable fast burst mode");

		if (arm11_config_memwrite_error_fatal)
			return ERROR_FAIL;
	}
	}
#endif

	arm11_run_instr_data_finish(arm11);

	return ERROR_OK;
}


/* write target memory in multiples of 4 byte, optimized for writing large quantities of data */
int arm11_bulk_write_memory(struct target_s *target, u32 address, u32 count, u8 *buffer)
{
	FNC_INFO;

	if (target->state != TARGET_HALTED)
	{
	LOG_WARNING("target was not halted");
	return ERROR_TARGET_NOT_HALTED;
	}

	return arm11_write_memory(target, address, 4, count, buffer);
}

int arm11_checksum_memory(struct target_s *target, u32 address, u32 count, u32* checksum)
{
	FNC_INFO_NOTIMPLEMENTED;

	return ERROR_OK;
}

/* target break-/watchpoint control
* rw: 0 = write, 1 = read, 2 = access
*/
int arm11_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	FNC_INFO;

	arm11_common_t * arm11 = target->arch_info;

#if 0
	if (breakpoint->type == BKPT_SOFT)
	{
	LOG_INFO("sw breakpoint requested, but software breakpoints not enabled");
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
#endif

	if (!arm11->free_brps)
	{
	LOG_INFO("no breakpoint unit available for hardware breakpoint");
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->length != 4)
	{
	LOG_INFO("only breakpoints of four bytes length supported");
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	arm11->free_brps--;

	return ERROR_OK;
}

int arm11_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	FNC_INFO;

	arm11_common_t * arm11 = target->arch_info;

	arm11->free_brps++;

	return ERROR_OK;
}

int arm11_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	FNC_INFO_NOTIMPLEMENTED;

	return ERROR_OK;
}

int arm11_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	FNC_INFO_NOTIMPLEMENTED;

	return ERROR_OK;
}

// HACKHACKHACK - FIXME mode/state
/* target algorithm support */
int arm11_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params,
			int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point,
			int timeout_ms, void *arch_info)
{
		arm11_common_t *arm11 = target->arch_info;
	armv4_5_algorithm_t *arm11_algorithm_info = arch_info;
//	enum armv4_5_state core_state = arm11->core_state;
//	enum armv4_5_mode core_mode = arm11->core_mode;
	u32 context[16];
	u32 cpsr;
	int exit_breakpoint_size = 0;
	int i;
	int retval = ERROR_OK;
		LOG_DEBUG("Running algorithm");

	if (arm11_algorithm_info->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		LOG_ERROR("current target isn't an ARMV4/5 target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	// FIXME
//	if (armv4_5_mode_to_number(arm11->core_mode)==-1)
//		return ERROR_FAIL;

	// Save regs
	for (i = 0; i < 16; i++)
	{
		context[i] = buf_get_u32((u8*)(&arm11->reg_values[i]),0,32);
		LOG_DEBUG("Save %i: 0x%x",i,context[i]);
	}

	cpsr = buf_get_u32((u8*)(arm11->reg_values+ARM11_RC_CPSR),0,32);
	LOG_DEBUG("Save CPSR: 0x%x", cpsr);

	for (i = 0; i < num_mem_params; i++)
	{
		target_write_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
	}

	// Set register parameters
	for (i = 0; i < num_reg_params; i++)
	{
		reg_t *reg = register_get_by_name(arm11->core_cache, reg_params[i].reg_name, 0);
		if (!reg)
		{
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			exit(-1);
		}

		if (reg->size != reg_params[i].size)
		{
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size", reg_params[i].reg_name);
			exit(-1);
		}
		arm11_set_reg(reg,reg_params[i].value);
//		printf("%i: Set %s =%08x\n", i, reg_params[i].reg_name,val);
	}

	exit_breakpoint_size = 4;

/*	arm11->core_state = arm11_algorithm_info->core_state;
	if (arm11->core_state == ARMV4_5_STATE_ARM)
				exit_breakpoint_size = 4;
	else if (arm11->core_state == ARMV4_5_STATE_THUMB)
		exit_breakpoint_size = 2;
	else
	{
		LOG_ERROR("BUG: can't execute algorithms when not in ARM or Thumb state");
		exit(-1);
	}
*/
	if (arm11_algorithm_info->core_mode != ARMV4_5_MODE_ANY)
	{
		LOG_DEBUG("setting core_mode: 0x%2.2x", arm11_algorithm_info->core_mode);
		buf_set_u32(arm11->reg_list[ARM11_RC_CPSR].value, 0, 5, arm11_algorithm_info->core_mode);
		arm11->reg_list[ARM11_RC_CPSR].dirty = 1;
		arm11->reg_list[ARM11_RC_CPSR].valid = 1;
	}

	if ((retval = breakpoint_add(target, exit_point, exit_breakpoint_size, BKPT_HARD)) != ERROR_OK)
	{
		LOG_ERROR("can't add breakpoint to finish algorithm execution");
		retval = ERROR_TARGET_FAILURE;
		goto restore;
	}

	// no debug, otherwise breakpoint is not set
	if((retval = target_resume(target, 0, entry_point, 1, 0)) != ERROR_OK)
	{
		return retval;
	}

	if((retval = target_wait_state(target, TARGET_HALTED, timeout_ms)) != ERROR_OK)
	{
		return retval;
	}

	if (target->state != TARGET_HALTED)
	{
		if ((retval=target_halt(target))!=ERROR_OK)
			return retval;
		if ((retval=target_wait_state(target, TARGET_HALTED, 500))!=ERROR_OK)
		{
			return retval;
		}
		retval = ERROR_TARGET_TIMEOUT;
		goto del_breakpoint;
	}

	if (buf_get_u32(arm11->reg_list[15].value, 0, 32) != exit_point)
	{
		LOG_WARNING("target reentered debug state, but not at the desired exit point: 0x%4.4x",
			buf_get_u32(arm11->reg_list[15].value, 0, 32));
		retval = ERROR_TARGET_TIMEOUT;
		goto del_breakpoint;
	}

	for (i = 0; i < num_mem_params; i++)
	{
		if (mem_params[i].direction != PARAM_OUT)
			target_read_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
	}

	for (i = 0; i < num_reg_params; i++)
	{
		if (reg_params[i].direction != PARAM_OUT)
		{
			reg_t *reg = register_get_by_name(arm11->core_cache, reg_params[i].reg_name, 0);
			if (!reg)
			{
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				exit(-1);
			}

			if (reg->size != reg_params[i].size)
			{
				LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size", reg_params[i].reg_name);
				exit(-1);
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

del_breakpoint:
	breakpoint_remove(target, exit_point);

restore:
	// Restore context
	for (i = 0; i < 16; i++)
	{
		LOG_DEBUG("restoring register %s with value 0x%8.8x",
			 arm11->reg_list[i].name, context[i]);
		arm11_set_reg(&arm11->reg_list[i], (u8*)&context[i]);
	}
	LOG_DEBUG("restoring CPSR with value 0x%8.8x", cpsr);
	arm11_set_reg(&arm11->reg_list[ARM11_RC_CPSR], (u8*)&cpsr);

//	arm11->core_state = core_state;
//	arm11->core_mode = core_mode;

	return retval;
}

int arm11_target_create(struct target_s *target, Jim_Interp *interp)
{
	int retval = ERROR_OK;
	FNC_INFO;

	NEW(arm11_common_t, arm11, 1);

	arm11->target = target;

	/* prepare JTAG information for the new target */
	arm11->jtag_info.tap	= target->tap;
	arm11->jtag_info.scann_size	= 5;

	if((retval = arm_jtag_setup_connection(&arm11->jtag_info)) != ERROR_OK)
	{
		return retval;
	}

	if (target->tap==NULL)
		return ERROR_FAIL;

	if (target->tap->ir_length != 5)
	{
		LOG_ERROR("'target arm11' expects IR LENGTH = 5");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	target->arch_info = arm11;

	return ERROR_OK;
}

int arm11_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	/* Initialize anything we can set up without talking to the target */
	return arm11_build_reg_cache(target);

}

/* talk to the target and set things up */
int arm11_examine(struct target_s *target)
{
	FNC_INFO;
	int retval;

	arm11_common_t * arm11 = target->arch_info;

	/* check IDCODE */

	arm11_add_IR(arm11, ARM11_IDCODE, -1);

	scan_field_t		idcode_field;

	arm11_setup_field(arm11, 32, NULL, &arm11->device_id, &idcode_field);

	arm11_add_dr_scan_vc(1, &idcode_field, TAP_DRPAUSE);

	/* check DIDR */

	arm11_add_debug_SCAN_N(arm11, 0x00, -1);

	arm11_add_IR(arm11, ARM11_INTEST, -1);

	scan_field_t		chain0_fields[2];

	arm11_setup_field(arm11, 32, NULL,	&arm11->didr,		chain0_fields + 0);
	arm11_setup_field(arm11,  8, NULL,	&arm11->implementor,	chain0_fields + 1);

	arm11_add_dr_scan_vc(asizeof(chain0_fields), chain0_fields, TAP_IDLE);

	if ((retval=jtag_execute_queue())!=ERROR_OK)
		return retval;


	switch (arm11->device_id & 0x0FFFF000)
	{
	case 0x07B36000:	LOG_INFO("found ARM1136"); break;
	case 0x07B56000:	LOG_INFO("found ARM1156"); break;
	case 0x07B76000:	LOG_INFO("found ARM1176"); break;
	default:
	{
		LOG_ERROR("'target arm11' expects IDCODE 0x*7B*7****");
		return ERROR_FAIL;
	}
	}

	arm11->debug_version = (arm11->didr >> 16) & 0x0F;

	if (arm11->debug_version != ARM11_DEBUG_V6 &&
	arm11->debug_version != ARM11_DEBUG_V61)
	{
	LOG_ERROR("Only ARMv6 v6 and v6.1 architectures supported.");
	return ERROR_FAIL;
	}


	arm11->brp	= ((arm11->didr >> 24) & 0x0F) + 1;
	arm11->wrp	= ((arm11->didr >> 28) & 0x0F) + 1;

	/** \todo TODO: reserve one brp slot if we allow breakpoints during step */
	arm11->free_brps = arm11->brp;
	arm11->free_wrps = arm11->wrp;

	LOG_DEBUG("IDCODE %08x IMPLEMENTOR %02x DIDR %08x",
	arm11->device_id,
	arm11->implementor,
	arm11->didr);

	/* as a side-effect this reads DSCR and thus
	 * clears the ARM11_DSCR_STICKY_PRECISE_DATA_ABORT / Sticky Precise Data Abort Flag
	 * as suggested by the spec.
	 */

	arm11_check_init(arm11, NULL);

	target->type->examined = 1;

	return ERROR_OK;
}

int arm11_quit(void)
{
	FNC_INFO_NOTIMPLEMENTED;

	return ERROR_OK;
}

/** Load a register that is marked !valid in the register cache */
int arm11_get_reg(reg_t *reg)
{
	FNC_INFO;

	target_t * target = ((arm11_reg_state_t *)reg->arch_info)->target;

	if (target->state != TARGET_HALTED)
	{
	LOG_WARNING("target was not halted");
	return ERROR_TARGET_NOT_HALTED;
	}

	/** \todo TODO: Check this. We assume that all registers are fetched at debug entry. */

#if 0
	arm11_common_t *arm11 = target->arch_info;
	const arm11_reg_defs_t * arm11_reg_info = arm11_reg_defs + ((arm11_reg_state_t *)reg->arch_info)->def_index;
#endif

	return ERROR_OK;
}

/** Change a value in the register cache */
int arm11_set_reg(reg_t *reg, u8 *buf)
{
	FNC_INFO;

	target_t * target = ((arm11_reg_state_t *)reg->arch_info)->target;
	arm11_common_t *arm11 = target->arch_info;
//	  const arm11_reg_defs_t * arm11_reg_info = arm11_reg_defs + ((arm11_reg_state_t *)reg->arch_info)->def_index;

	arm11->reg_values[((arm11_reg_state_t *)reg->arch_info)->def_index] = buf_get_u32(buf, 0, 32);
	reg->valid	= 1;
	reg->dirty	= 1;

	return ERROR_OK;
}

int arm11_build_reg_cache(target_t *target)
{
	arm11_common_t *arm11 = target->arch_info;

	NEW(reg_cache_t,		cache,			1);
	NEW(reg_t,			reg_list,		ARM11_REGCACHE_COUNT);
	NEW(arm11_reg_state_t,	arm11_reg_states,	ARM11_REGCACHE_COUNT);

	if (arm11_regs_arch_type == -1)
		arm11_regs_arch_type = register_reg_arch_type(arm11_get_reg, arm11_set_reg);

	register_init_dummy(&arm11_gdb_dummy_fp_reg);
	register_init_dummy(&arm11_gdb_dummy_fps_reg);

	arm11->reg_list	= reg_list;

	/* Build the process context cache */
	cache->name		= "arm11 registers";
	cache->next		= NULL;
	cache->reg_list	= reg_list;
	cache->num_regs	= ARM11_REGCACHE_COUNT;

	reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
	(*cache_p) = cache;

	arm11->core_cache = cache;
//	  armv7m->process_context = cache;

	size_t i;

	/* Not very elegant assertion */
	if (ARM11_REGCACHE_COUNT != asizeof(arm11->reg_values) ||
	ARM11_REGCACHE_COUNT != asizeof(arm11_reg_defs) ||
	ARM11_REGCACHE_COUNT != ARM11_RC_MAX)
	{
	LOG_ERROR("BUG: arm11->reg_values inconsistent (%d " ZU " " ZU " %d)", ARM11_REGCACHE_COUNT, asizeof(arm11->reg_values), asizeof(arm11_reg_defs), ARM11_RC_MAX);
	exit(-1);
	}

	for (i = 0; i < ARM11_REGCACHE_COUNT; i++)
	{
	reg_t *				r	= reg_list		+ i;
	const arm11_reg_defs_t *	rd	= arm11_reg_defs	+ i;
	arm11_reg_state_t *		rs	= arm11_reg_states	+ i;

	r->name			= rd->name;
	r->size			= 32;
	r->value		= (u8 *)(arm11->reg_values + i);
	r->dirty		= 0;
	r->valid		= 0;
	r->bitfield_desc	= NULL;
	r->num_bitfields	= 0;
	r->arch_type		= arm11_regs_arch_type;
	r->arch_info		= rs;

	rs->def_index		= i;
	rs->target		= target;
	}
	return ERROR_OK;
}

int arm11_handle_bool(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, bool * var, char * name)
{
	if (argc == 0)
	{
	LOG_INFO("%s is %s.", name, *var ? "enabled" : "disabled");
	return ERROR_OK;
	}

	if (argc != 1)
	return ERROR_COMMAND_SYNTAX_ERROR;

	switch (args[0][0])
	{
	case '0':	/* 0 */
	case 'f':	/* false */
	case 'F':
	case 'd':	/* disable */
	case 'D':
	*var = false;
	break;

	case '1':	/* 1 */
	case 't':	/* true */
	case 'T':
	case 'e':	/* enable */
	case 'E':
	*var = true;
	break;
	}

	LOG_INFO("%s %s.", *var ? "Enabled" : "Disabled", name);

	return ERROR_OK;
}

#define BOOL_WRAPPER(name, print_name)	\
int arm11_handle_bool_##name(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc) \
{ \
	return arm11_handle_bool(cmd_ctx, cmd, args, argc, &arm11_config_##name, print_name); \
}

#define RC_TOP(name, descr, more)  \
{ \
	command_t * new_cmd = register_command(cmd_ctx, top_cmd, name, NULL, COMMAND_ANY, descr);  \
	command_t * top_cmd = new_cmd; \
	more \
}

#define RC_FINAL(name, descr, handler)	\
	register_command(cmd_ctx, top_cmd, name, handler, COMMAND_ANY, descr);

#define RC_FINAL_BOOL(name, descr, var)  \
	register_command(cmd_ctx, top_cmd, name, arm11_handle_bool_##var, COMMAND_ANY, descr);

BOOL_WRAPPER(memwrite_burst,		"memory write burst mode")
BOOL_WRAPPER(memwrite_error_fatal,	"fatal error mode for memory writes")

int arm11_handle_vcr(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
	arm11_vcr = strtoul(args[0], NULL, 0);
	}
	else if (argc != 0)
	{
	return ERROR_COMMAND_SYNTAX_ERROR;
	}

	LOG_INFO("VCR 0x%08X", arm11_vcr);
	return ERROR_OK;
}

const u32 arm11_coproc_instruction_limits[] =
{
	15,			/* coprocessor */
	7,			/* opcode 1 */
	15,			/* CRn */
	15,			/* CRm */
	7,			/* opcode 2 */
	0xFFFFFFFF,		/* value */
};

const char arm11_mrc_syntax[] = "Syntax: mrc <jtag_target> <coprocessor> <opcode 1> <CRn> <CRm> <opcode 2>. All parameters are numbers only.";
const char arm11_mcr_syntax[] = "Syntax: mcr <jtag_target> <coprocessor> <opcode 1> <CRn> <CRm> <opcode 2> <32bit value to write>. All parameters are numbers only.";

arm11_common_t * arm11_find_target(const char * arg)
{
	jtag_tap_t *tap;
	target_t * t;

	tap = jtag_TapByString( arg );
	if( !tap ){
		return NULL;
	}

	for (t = all_targets; t; t = t->next){
		if( t->tap == tap ){
			if( 0 == strcmp(t->type->name,"arm11")){
				arm11_common_t * arm11 = t->arch_info;
				return arm11;
			}
		}
	}
	return 0;
}

int arm11_handle_mrc_mcr(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, bool read)
{
	if (argc != (read ? 6 : 7))
	{
	LOG_ERROR("Invalid number of arguments. %s", read ? arm11_mrc_syntax : arm11_mcr_syntax);
	return -1;
	}

	arm11_common_t * arm11 = arm11_find_target(args[0]);

	if (!arm11)
	{
	LOG_ERROR("Parameter 1 is not a the JTAG chain position of an ARM11 device. %s",
		read ? arm11_mrc_syntax : arm11_mcr_syntax);

	return -1;

	}

	if (arm11->target->state != TARGET_HALTED)
	{
	LOG_WARNING("target was not halted");
	return ERROR_TARGET_NOT_HALTED;
	}

	u32	values[6];

	{size_t i;
	for (i = 0; i < (read ? 5 : 6); i++)
	{
	values[i] = strtoul(args[i + 1], NULL, 0);

	if (values[i] > arm11_coproc_instruction_limits[i])
	{
		LOG_ERROR("Parameter %ld out of bounds (%d max). %s",
			  (long)(i + 2), arm11_coproc_instruction_limits[i],
		read ? arm11_mrc_syntax : arm11_mcr_syntax);
		return -1;
	}
	}}

	u32 instr = 0xEE000010	|
	(values[0] <<  8) |
	(values[1] << 21) |
	(values[2] << 16) |
	(values[3] <<  0) |
	(values[4] <<  5);

	if (read)
	instr |= 0x00100000;

	arm11_run_instr_data_prepare(arm11);

	if (read)
	{
	u32 result;
	arm11_run_instr_data_from_core_via_r0(arm11, instr, &result);

	LOG_INFO("MRC p%d, %d, R0, c%d, c%d, %d = 0x%08x (%d)",
		values[0], values[1], values[2], values[3], values[4], result, result);
	}
	else
	{
	arm11_run_instr_data_to_core_via_r0(arm11, instr, values[5]);

	LOG_INFO("MRC p%d, %d, R0 (#0x%08x), c%d, c%d, %d",
		values[0], values[1],
		values[5],
		values[2], values[3], values[4]);
	}

	arm11_run_instr_data_finish(arm11);


	return ERROR_OK;
}

int arm11_handle_mrc(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return arm11_handle_mrc_mcr(cmd_ctx, cmd, args, argc, true);
}

int arm11_handle_mcr(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return arm11_handle_mrc_mcr(cmd_ctx, cmd, args, argc, false);
}

int arm11_register_commands(struct command_context_s *cmd_ctx)
{
	FNC_INFO;

	command_t * top_cmd = NULL;

	RC_TOP(			"arm11",	"arm11 specific commands",

	RC_TOP(			"memwrite",	"Control memory write transfer mode",

		RC_FINAL_BOOL(	"burst",	"Enable/Disable non-standard but fast burst mode (default: enabled)",
						memwrite_burst)

		RC_FINAL_BOOL(	"error_fatal",
						"Terminate program if transfer error was found (default: enabled)",
						memwrite_error_fatal)
	)

	RC_FINAL(		"vcr",		"Control (Interrupt) Vector Catch Register",
						arm11_handle_vcr)

	RC_FINAL(		"mrc",		"Read Coprocessor register",
						arm11_handle_mrc)

	RC_FINAL(		"mcr",		"Write Coprocessor register",
						arm11_handle_mcr)
	)

	return ERROR_OK;
}

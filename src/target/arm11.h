/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
 *   Michael Bruck                                                         *
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

#ifndef ARM11_H
#define ARM11_H

#include "target.h"
#include "register.h"
#include "jtag.h"

#define asizeof(x)	(sizeof(x) / sizeof((x)[0]))

#define NEW(type, variable, items)			\
	type * variable = calloc(1, sizeof(type) * items)

/* For MinGW use 'I' prefix to print size_t (instead of 'z') */
/* Except if __USE_MINGW_ANSI_STDIO is defined with MinGW    */

#if (!defined(__MSVCRT__) || defined(__USE_MINGW_ANSI_STDIO))
#define ZU		"%zu"
#else
#define ZU		"%Iu"
#endif

#define ARM11_REGCACHE_MODEREGS		0
#define ARM11_REGCACHE_FREGS		0

#define ARM11_REGCACHE_COUNT		(20 +					\
					 23 * ARM11_REGCACHE_MODEREGS +			\
					  9 * ARM11_REGCACHE_FREGS)

#define ARM11_TAP_DEFAULT			TAP_INVALID


#define CHECK_RETVAL(action)								\
do {														\
	int __retval = (action);								\
															\
	if (__retval != ERROR_OK)								\
	{														\
		LOG_DEBUG("error while calling \"" # action "\"");	\
		return __retval;									\
	}														\
															\
} while (0)


typedef struct arm11_register_history_s
{
	uint32_t		value;
	uint8_t		valid;
}arm11_register_history_t;

enum arm11_debug_version
{
	ARM11_DEBUG_V6			= 0x01,
	ARM11_DEBUG_V61			= 0x02,
	ARM11_DEBUG_V7			= 0x03,
	ARM11_DEBUG_V7_CP14		= 0x04,
};

typedef struct arm11_common_s
{
	target_t *	target;		/**< Reference back to the owner */

	/** \name Processor type detection */
	/*@{*/

	uint32_t		device_id;		/**< IDCODE readout				*/
	uint32_t		didr;			/**< DIDR readout (debug capabilities)	*/
	uint8_t		implementor;	/**< DIDR Implementor readout		*/

	size_t	brp;			/**< Number of Breakpoint Register Pairs from DIDR	*/
	size_t	wrp;			/**< Number of Watchpoint Register Pairs from DIDR	*/

	enum arm11_debug_version
		debug_version;		/**< ARM debug architecture from DIDR	*/
	/*@}*/

	uint32_t		last_dscr;		/**< Last retrieved DSCR value;
							     Use only for debug message generation		*/

	bool	simulate_reset_on_next_halt;	/**< Perform cleanups of the ARM state on next halt */

	/** \name Shadow registers to save processor state */
	/*@{*/

	reg_t *	reg_list;							/**< target register list */
	uint32_t		reg_values[ARM11_REGCACHE_COUNT];	/**< data for registers */

	/*@}*/

	arm11_register_history_t
		reg_history[ARM11_REGCACHE_COUNT];	/**< register state before last resume */

	size_t	free_brps;				/**< keep track of breakpoints allocated by arm11_add_breakpoint() */
	size_t	free_wrps;				/**< keep track of breakpoints allocated by arm11_add_watchpoint() */

	// GA
	reg_cache_t *core_cache;
} arm11_common_t;


/**
 * ARM11 DBGTAP instructions
 *
 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0301f/I1006229.html
 */
enum arm11_instructions
{
	ARM11_EXTEST    = 0x00,
	ARM11_SCAN_N    = 0x02,
	ARM11_RESTART   = 0x04,
	ARM11_HALT	    = 0x08,
	ARM11_INTEST    = 0x0C,
	ARM11_ITRSEL    = 0x1D,
	ARM11_IDCODE    = 0x1E,
	ARM11_BYPASS    = 0x1F,
};

enum arm11_dscr
{
	ARM11_DSCR_CORE_HALTED									= 1 << 0,
	ARM11_DSCR_CORE_RESTARTED								= 1 << 1,

	ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_MASK					= 0x0F << 2,
	ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_HALT					= 0x00 << 2,
	ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_BREAKPOINT				= 0x01 << 2,
	ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_WATCHPOINT				= 0x02 << 2,
	ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_BKPT_INSTRUCTION		= 0x03 << 2,
	ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_EDBGRQ					= 0x04 << 2,
	ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_VECTOR_CATCH			= 0x05 << 2,

	ARM11_DSCR_STICKY_PRECISE_DATA_ABORT					= 1 << 6,
	ARM11_DSCR_STICKY_IMPRECISE_DATA_ABORT					= 1 << 7,
	ARM11_DSCR_INTERRUPTS_DISABLE							= 1 << 11,
	ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE				= 1 << 13,
	ARM11_DSCR_MODE_SELECT									= 1 << 14,
	ARM11_DSCR_WDTR_FULL									= 1 << 29,
	ARM11_DSCR_RDTR_FULL									= 1 << 30,
};

enum arm11_cpsr
{
	ARM11_CPSR_T				= 1 << 5,
	ARM11_CPSR_J				= 1 << 24,
};

enum arm11_sc7
{
	ARM11_SC7_NULL				= 0,
	ARM11_SC7_VCR				= 7,
	ARM11_SC7_PC				= 8,
	ARM11_SC7_BVR0				= 64,
	ARM11_SC7_BCR0				= 80,
	ARM11_SC7_WVR0				= 96,
	ARM11_SC7_WCR0				= 112,
};

typedef struct arm11_reg_state_s
{
	uint32_t				def_index;
	target_t *			target;
} arm11_reg_state_t;

int arm11_register_commands(struct command_context_s *cmd_ctx);

/* internals */

void arm11_setup_field			(arm11_common_t * arm11, int num_bits, void * in_data, void * out_data, scan_field_t * field);
void arm11_add_IR				(arm11_common_t * arm11, uint8_t instr, tap_state_t state);
int arm11_add_debug_SCAN_N		(arm11_common_t * arm11, uint8_t chain, tap_state_t state);
void arm11_add_debug_INST		(arm11_common_t * arm11, uint32_t inst, uint8_t * flag, tap_state_t state);
int arm11_read_DSCR				(arm11_common_t * arm11, uint32_t *dscr);
int arm11_write_DSCR			(arm11_common_t * arm11, uint32_t dscr);

enum target_debug_reason arm11_get_DSCR_debug_reason(uint32_t dscr);

int arm11_run_instr_data_prepare			(arm11_common_t * arm11);
int arm11_run_instr_data_finish				(arm11_common_t * arm11);
int arm11_run_instr_no_data					(arm11_common_t * arm11, uint32_t * opcode, size_t count);
int arm11_run_instr_no_data1				(arm11_common_t * arm11, uint32_t opcode);
int arm11_run_instr_data_to_core			(arm11_common_t * arm11, uint32_t opcode, uint32_t * data, size_t count);
int arm11_run_instr_data_to_core_noack		(arm11_common_t * arm11, uint32_t opcode, uint32_t * data, size_t count);
int arm11_run_instr_data_to_core1			(arm11_common_t * arm11, uint32_t opcode, uint32_t data);
int arm11_run_instr_data_from_core			(arm11_common_t * arm11, uint32_t opcode, uint32_t * data, size_t count);
int arm11_run_instr_data_from_core_via_r0	(arm11_common_t * arm11, uint32_t opcode, uint32_t * data);
int arm11_run_instr_data_to_core_via_r0	(arm11_common_t * arm11, uint32_t opcode, uint32_t data);

int arm11_add_dr_scan_vc(int num_fields, scan_field_t *fields, tap_state_t state);
int arm11_add_ir_scan_vc(int num_fields, scan_field_t *fields, tap_state_t state);

/** Used to make a list of read/write commands for scan chain 7
 *
 *  Use with arm11_sc7_run()
 */
typedef struct arm11_sc7_action_s
{
	bool	write;				/**< Access mode: true for write, false for read.	*/
	uint8_t		address;			/**< Register address mode. Use enum #arm11_sc7		*/
	uint32_t		value;				/**< If write then set this to value to be written.
									 In read mode this receives the read value when the
									 function returns.					*/
} arm11_sc7_action_t;

int arm11_sc7_run(arm11_common_t * arm11, arm11_sc7_action_t * actions, size_t count);

/* Mid-level helper functions */
void arm11_sc7_clear_vbw(arm11_common_t * arm11);
void arm11_sc7_set_vcr(arm11_common_t * arm11, uint32_t value);

int arm11_read_memory_word(arm11_common_t * arm11, uint32_t address, uint32_t * result);

#endif /* ARM11_H */

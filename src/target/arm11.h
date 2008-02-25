/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
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
#include "embeddedice.h"
#include "arm_jtag.h"


#define bool	int
#define true	1
#define false	0

#define asizeof(x)	(sizeof(x) / sizeof((x)[0]))

#define NEW(type, variable, items) \
    type * variable = malloc(sizeof(type) * items)


#define ARM11_REGCACHE_MODEREGS		0
#define ARM11_REGCACHE_FREGS		0

#define ARM11_REGCACHE_COUNT		(20 +					\
					 23 * ARM11_REGCACHE_MODEREGS +		\
					  9 * ARM11_REGCACHE_FREGS)


typedef struct arm11_register_history_s
{
    u32	    value;
    u8	    valid;
}arm11_register_history_t;



typedef struct arm11_common_s
{
    target_t *	target;

    arm_jtag_t	jtag_info;

    /** \name Processor type detection */
    /*@{*/

    u32		device_id;	    /**< IDCODE readout				*/
    u32		didr;		    /**< DIDR readout (debug capabilities)	*/
    u8		implementor;	    /**< DIDR Implementor readout		*/

    size_t	brp;		    /**< Number of Breakpoint Register Pairs	*/
    size_t	wrp;		    /**< Number of Watchpoint Register Pairs	*/

    /*@}*/


    u32		last_dscr;	    /**< Last retrieved DSCR value;
				     *   Can be used to detect changes		*/

    u8		trst_active;
    u8		halt_requested;

    /** \name Shadow registers to save processor state */
    /*@{*/

    reg_t *	reg_list;				/**< target register list */
    u32		reg_values[ARM11_REGCACHE_COUNT];	/**< data for registers */

    /*@}*/

    arm11_register_history_t
		reg_history[ARM11_REGCACHE_COUNT];	/**< register state before last resume */


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
    ARM11_DSCR_CORE_HALTED				= 1 << 0,
    ARM11_DSCR_CORE_RESTARTED				= 1 << 1,

    ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_MASK		= 0x0F << 2,
    ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_HALT		= 0x00 << 2,
    ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_BREAKPOINT		= 0x01 << 2,
    ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_WATCHPOINT		= 0x02 << 2,
    ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_BKPT_INSTRUCTION	= 0x03 << 2,
    ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_EDBGRQ		= 0x04 << 2,
    ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_VECTOR_CATCH	= 0x05 << 2,

    ARM11_DSCR_STICKY_PRECISE_DATA_ABORT		= 1 << 6,
    ARM11_DSCR_STICKY_IMPRECISE_DATA_ABORT		= 1 << 7,
    ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE		= 1 << 13,
    ARM11_DSCR_MODE_SELECT				= 1 << 14,
    ARM11_DSCR_WDTR_FULL				= 1 << 29,
    ARM11_DSCR_RDTR_FULL				= 1 << 30,
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
    u32				def_index;
    target_t *			target;
} arm11_reg_state_t;




/* poll current target status */
int arm11_poll(struct target_s *target);
/* architecture specific status reply */
int arm11_arch_state(struct target_s *target);

/* target request support */
int arm11_target_request_data(struct target_s *target, u32 size, u8 *buffer);

/* target execution control */
int arm11_halt(struct target_s *target);
int arm11_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution);
int arm11_step(struct target_s *target, int current, u32 address, int handle_breakpoints);

/* target reset control */
int arm11_assert_reset(struct target_s *target);
int arm11_deassert_reset(struct target_s *target);
int arm11_soft_reset_halt(struct target_s *target);
int arm11_prepare_reset_halt(struct target_s *target);

/* target register access for gdb */
int arm11_get_gdb_reg_list(struct target_s *target, struct reg_s **reg_list[], int *reg_list_size);

/* target memory access 
* size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
* count: number of items of <size>
*/
int arm11_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm11_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);

/* write target memory in multiples of 4 byte, optimized for writing large quantities of data */
int arm11_bulk_write_memory(struct target_s *target, u32 address, u32 count, u8 *buffer);

int arm11_checksum_memory(struct target_s *target, u32 address, u32 count, u32* checksum);

/* target break-/watchpoint control 
* rw: 0 = write, 1 = read, 2 = access
*/
int arm11_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int arm11_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int arm11_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
int arm11_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint);

/* target algorithm support */
int arm11_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_param, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info);

int arm11_register_commands(struct command_context_s *cmd_ctx);
int arm11_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target);
int arm11_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm11_quit(void);


/* helpers */
void arm11_build_reg_cache(target_t *target);


/* internals */

void arm11_setup_field		(arm11_common_t * arm11, int num_bits, void * in_data, void * out_data, scan_field_t * field);
void arm11_add_IR		(arm11_common_t * arm11, u8 instr, enum tap_state state);
void arm11_add_debug_SCAN_N	(arm11_common_t * arm11, u8 chain, enum tap_state state);
void arm11_add_debug_INST	(arm11_common_t * arm11, u32 inst, u8 * flag, enum tap_state state);
u32  arm11_read_DSCR		(arm11_common_t * arm11);
void arm11_write_DSCR		(arm11_common_t * arm11, u32 dscr);

enum target_debug_reason arm11_get_DSCR_debug_reason(u32 dscr);

void arm11_run_instr_data_prepare		(arm11_common_t * arm11);
void arm11_run_instr_data_finish		(arm11_common_t * arm11);
void arm11_run_instr_no_data			(arm11_common_t * arm11, u32 * opcode, size_t count);
void arm11_run_instr_no_data1			(arm11_common_t * arm11, u32 opcode);
void arm11_run_instr_data_to_core		(arm11_common_t * arm11, u32 opcode, u32 * data, size_t count);
void arm11_run_instr_data_to_core1		(arm11_common_t * arm11, u32 opcode, u32 data);
void arm11_run_instr_data_from_core		(arm11_common_t * arm11, u32 opcode, u32 * data, size_t count);
void arm11_run_instr_data_from_core_via_r0	(arm11_common_t * arm11, u32 opcode, u32 * data);
void arm11_run_instr_data_to_core_via_r0	(arm11_common_t * arm11, u32 opcode, u32 data);


typedef struct arm11_sc7_action_s
{
    bool    write;
    u8	    address;
    u32	    value;
} arm11_sc7_action_t;

void arm11_sc7_run(arm11_common_t * arm11, arm11_sc7_action_t * actions, size_t count);
void arm11_sc7_clear_bw(arm11_common_t * arm11);



#endif /* ARM11_H */

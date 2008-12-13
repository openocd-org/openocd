/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#ifndef TARGET_H
#define TARGET_H

#include "register.h"
#include "breakpoints.h"
#include "algorithm.h"
#include "trace.h"

#include "command.h"
#include "types.h"

#include <sys/time.h>
#include <time.h>

struct reg_s;
struct command_context_s;
/*
 * TARGET_UNKNOWN = 0: we don't know anything about the target yet
 * TARGET_RUNNING = 1: the target is executing user code
 * TARGET_HALTED  = 2: the target is not executing code, and ready to talk to the
 * debugger. on an xscale it means that the debug handler is executing
 * TARGET_RESET   = 3: the target is being held in reset (only a temporary state,
 * not sure how this is used with all the recent changes)
 * TARGET_DEBUG_RUNNING = 4: the target is running, but it is executing code on
 * behalf of the debugger (e.g. algorithm for flashing) */

enum target_state
{
	TARGET_UNKNOWN = 0,
	TARGET_RUNNING = 1,
	TARGET_HALTED = 2,
	TARGET_RESET = 3,
	TARGET_DEBUG_RUNNING = 4,
};

extern const Jim_Nvp nvp_target_state[];

enum nvp_assert {
	NVP_DEASSERT,
	NVP_ASSERT,
};

extern const Jim_Nvp nvp_assert[];

enum target_reset_mode
{
	RESET_UNKNOWN = 0,
	RESET_RUN = 1,		/* reset and let target run */
	RESET_HALT = 2,		/* reset and halt target out of reset */
	RESET_INIT = 3,		/* reset and halt target out of reset, then run init script */
};

extern const Jim_Nvp nvp_reset_mode[];

enum target_debug_reason
{
	DBG_REASON_DBGRQ = 0,
	DBG_REASON_BREAKPOINT = 1,
	DBG_REASON_WATCHPOINT = 2,
	DBG_REASON_WPTANDBKPT = 3,
	DBG_REASON_SINGLESTEP = 4,
	DBG_REASON_NOTHALTED = 5,
	DBG_REASON_UNDEFINED = 6
};

extern const Jim_Nvp nvp_target_debug_reason[];

enum target_endianess
{
	TARGET_ENDIAN_UNKNOWN=0,
	TARGET_BIG_ENDIAN = 1, TARGET_LITTLE_ENDIAN = 2
};

extern const Jim_Nvp nvp_target_endian[];

struct target_s;

typedef struct working_area_s
{
	u32 address;
	u32 size;
	int free;
	u8 *backup;
	struct working_area_s **user;
	struct working_area_s *next;
} working_area_t;

typedef struct target_type_s
{
	char *name;

	int examined;

	/* poll current target status */
	int (*poll)(struct target_s *target);
	/* Invoked only from target_arch_state().
	 * Issue USER() w/architecture specific status.  */
	int (*arch_state)(struct target_s *target);

	/* target request support */
	int (*target_request_data)(struct target_s *target, u32 size, u8 *buffer);

	/* halt will log a warning, but return ERROR_OK if the target is already halted. */
	int (*halt)(struct target_s *target);
	int (*resume)(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution);
	int (*step)(struct target_s *target, int current, u32 address, int handle_breakpoints);

	/* target reset control. assert reset can be invoked when OpenOCD and
	 * the target is out of sync.
	 *
	 * A typical example is that the target was power cycled while OpenOCD
	 * thought the target was halted or running.
	 *
	 * assert_reset() can therefore make no assumptions whatsoever about the
	 * state of the target
	 *
	 * Before assert_reset() for the target is invoked, a TRST/tms and
	 * chain validation is executed. TRST should not be asserted
	 * during target assert unless there is no way around it due to
	 * the way reset's are configured.
	 *
	 */
	int (*assert_reset)(struct target_s *target);
	int (*deassert_reset)(struct target_s *target);
	int (*soft_reset_halt_imp)(struct target_s *target);
	int (*soft_reset_halt)(struct target_s *target);

	/* target register access for gdb.
	 *
	 * Danger! this function will succeed even if the target is running
	 * and return a register list with dummy values.
	 *
	 * The reason is that GDB connection will fail without a valid register
	 * list, however it is after GDB is connected that monitor commands can
	 * be run to properly initialize the target
	 */
	int (*get_gdb_reg_list)(struct target_s *target, struct reg_s **reg_list[], int *reg_list_size);

	/* target memory access
	* size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
	* count: number of items of <size>
	*/
	int (*read_memory_imp)(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
	int (*read_memory)(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
	int (*write_memory_imp)(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
	int (*write_memory)(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);

	/* write target memory in multiples of 4 byte, optimized for writing large quantities of data */
	int (*bulk_write_memory)(struct target_s *target, u32 address, u32 count, u8 *buffer);

	int (*checksum_memory)(struct target_s *target, u32 address, u32 count, u32* checksum);
	int (*blank_check_memory)(struct target_s *target, u32 address, u32 count, u32* blank);

	/*
	 * target break-/watchpoint control
	 * rw: 0 = write, 1 = read, 2 = access
	 *
	 * Target must be halted while this is invoked as this
	 * will actually set up breakpoints on target.
	 *
	 * The breakpoint hardware will be set up upon adding the first breakpoint.
	 *
	 * Upon GDB connection all breakpoints/watchpoints are cleared.
	 */
	int (*add_breakpoint)(struct target_s *target, breakpoint_t *breakpoint);

	/* remove breakpoint. hw will only be updated if the target is currently halted.
	 * However, this method can be invoked on unresponsive targets.
	 */
	int (*remove_breakpoint)(struct target_s *target, breakpoint_t *breakpoint);
	int (*add_watchpoint)(struct target_s *target, watchpoint_t *watchpoint);
	/* remove watchpoint. hw will only be updated if the target is currently halted.
	 * However, this method can be invoked on unresponsive targets.
	 */
	int (*remove_watchpoint)(struct target_s *target, watchpoint_t *watchpoint);

	/* target algorithm support */
	int (*run_algorithm_imp)(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_param, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info);
	int (*run_algorithm)(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_param, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info);

	int (*register_commands)(struct command_context_s *cmd_ctx);

	/* called when target is created */
	int (*target_create)( struct target_s *target, Jim_Interp *interp );

	/* called for various config parameters */
	/* returns JIM_CONTINUE - if option not understood */
	/* otherwise: JIM_OK, or JIM_ERR, */
	int (*target_jim_configure)( struct target_s *target, Jim_GetOptInfo *goi );

	/* target commands specifically handled by the target */
	/* returns JIM_OK, or JIM_ERR, or JIM_CONTINUE - if option not understood */
	int (*target_jim_commands)( struct target_s *target, Jim_GetOptInfo *goi );

	/* invoked after JTAG chain has been examined & validated. During
	 * this stage the target is examined and any additional setup is
	 * performed.
	 *
	 * invoked every time after the jtag chain has been validated/examined
	 */
	int (*examine)(struct target_s *target);
	/* Set up structures for target.
	 *
	 * It is illegal to talk to the target at this stage as this fn is invoked
	 * before the JTAG chain has been examined/verified
	 * */
	int (*init_target)(struct command_context_s *cmd_ctx, struct target_s *target);
	int (*quit)(void);

	int (*virt2phys)(struct target_s *target, u32 address, u32 *physical);
	int (*mmu)(struct target_s *target, int *enabled);

} target_type_t;

/* forward decloration */
typedef struct target_event_action_s target_event_action_t;

typedef struct target_s
{
	target_type_t *type;				/* target type definition (name, access functions) */
	const char *cmd_name;				/* tcl Name of target */
	int target_number;					/* generaly, target index but may not be in order */
	jtag_tap_t *tap;					/* where on the jtag chain is this */
	const char *variant;				/* what varient of this chip is it? */
	target_event_action_t *event_action;

	int reset_halt;						/* attempt resetting the CPU into the halted mode? */
	u32 working_area;					/* working area (initialized RAM). Evaluated
										 * upon first allocation from virtual/physical address. */
	u32 working_area_virt;				/* virtual address */
	u32 working_area_phys;				/* physical address */
	u32 working_area_size;				/* size in bytes */
	u32 backup_working_area;			/* whether the content of the working area has to be preserved */
	struct working_area_s *working_areas;/* list of allocated working areas */
	enum target_debug_reason debug_reason;/* reason why the target entered debug state */
	enum target_endianess endianness;	/* target endianess */
	enum target_state state;			/* the current backend-state (running, halted, ...) */
	struct reg_cache_s *reg_cache;		/* the first register cache of the target (core regs) */
	struct breakpoint_s *breakpoints;	/* list of breakpoints */
	struct watchpoint_s *watchpoints;	/* list of watchpoints */
	struct trace_s *trace_info;			/* generic trace information */
	struct debug_msg_receiver_s *dbgmsg;/* list of debug message receivers */
	u32 dbg_msg_enabled;				/* debug message status */
	void *arch_info;					/* architecture specific information */
	struct target_s *next;				/* next target in list */

	int display;						/* display async info in telnet session. Do not display
										 * lots of halted/resumed info when stepping in debugger. */
} target_t;

enum target_event
{
	/* LD historical names
	 * - Prior to the great TCL change
	 * - June/July/Aug 2008
	 * - Duane Ellis */
	TARGET_EVENT_OLD_gdb_program_config,
	TARGET_EVENT_OLD_pre_reset,
	TARGET_EVENT_OLD_post_reset,
	TARGET_EVENT_OLD_pre_resume,

	/* allow GDB to do stuff before others handle the halted event,
	 * this is in lieu of defining ordering of invocation of events,
	 * which would be more complicated */
	TARGET_EVENT_EARLY_HALTED,
	TARGET_EVENT_HALTED,		/* target entered debug state from normal execution or reset */
	TARGET_EVENT_RESUMED,		/* target resumed to normal execution */
	TARGET_EVENT_RESUME_START,
	TARGET_EVENT_RESUME_END,

	TARGET_EVENT_GDB_START, /* debugger started execution (step/run) */
	TARGET_EVENT_GDB_END, /* debugger stopped execution (step/run) */

	TARGET_EVENT_RESET_START,
	TARGET_EVENT_RESET_ASSERT_PRE,
	TARGET_EVENT_RESET_ASSERT_POST,
	TARGET_EVENT_RESET_DEASSERT_PRE,
	TARGET_EVENT_RESET_DEASSERT_POST,
	TARGET_EVENT_RESET_HALT_PRE,
	TARGET_EVENT_RESET_HALT_POST,
	TARGET_EVENT_RESET_WAIT_PRE,
	TARGET_EVENT_RESET_WAIT_POST,
	TARGET_EVENT_RESET_INIT,
	TARGET_EVENT_RESET_END,

	TARGET_EVENT_DEBUG_HALTED,	/* target entered debug state, but was executing on behalf of the debugger */
	TARGET_EVENT_DEBUG_RESUMED, /* target resumed to execute on behalf of the debugger */

	TARGET_EVENT_EXAMINE_START,
	TARGET_EVENT_EXAMINE_END,

	TARGET_EVENT_GDB_ATTACH,
	TARGET_EVENT_GDB_DETACH,

	TARGET_EVENT_GDB_FLASH_ERASE_START,
	TARGET_EVENT_GDB_FLASH_ERASE_END,
	TARGET_EVENT_GDB_FLASH_WRITE_START,
	TARGET_EVENT_GDB_FLASH_WRITE_END,
};

extern const Jim_Nvp nvp_target_event[];

struct target_event_action_s {
	enum target_event event;
	Jim_Obj *body;
	int has_percent;
	target_event_action_t *next;
 };

typedef struct target_event_callback_s
{
	int (*callback)(struct target_s *target, enum target_event event, void *priv);
	void *priv;
	struct target_event_callback_s *next;
} target_event_callback_t;

typedef struct target_timer_callback_s
{
	int (*callback)(void *priv);
	int time_ms;
	int periodic;
	struct timeval when;
	void *priv;
	struct target_timer_callback_s *next;
} target_timer_callback_t;

extern int target_register_commands(struct command_context_s *cmd_ctx);
extern int target_register_user_commands(struct command_context_s *cmd_ctx);
extern int target_init(struct command_context_s *cmd_ctx);
extern int target_examine(void);
extern int handle_target(void *priv);
extern int target_process_reset(struct command_context_s *cmd_ctx, enum target_reset_mode reset_mode);

extern int target_register_event_callback(int (*callback)(struct target_s *target, enum target_event event, void *priv), void *priv);
extern int target_unregister_event_callback(int (*callback)(struct target_s *target, enum target_event event, void *priv), void *priv);
extern int target_poll(target_t *target);
extern int target_resume(target_t *target, int current, u32 address, int handle_breakpoints, int debug_execution);
extern int target_halt(target_t *target);
extern int target_call_event_callbacks(target_t *target, enum target_event event);

/* The period is very approximate, the callback can happen much more often
 * or much more rarely than specified
 */
extern int target_register_timer_callback(int (*callback)(void *priv), int time_ms, int periodic, void *priv);
extern int target_unregister_timer_callback(int (*callback)(void *priv), void *priv);
extern int target_call_timer_callbacks(void);
/* invoke this to ensure that e.g. polling timer callbacks happen before
 * a syncrhonous command completes.
 */
extern int target_call_timer_callbacks_now(void);

extern target_t* get_current_target(struct command_context_s *cmd_ctx);
extern int get_num_by_target(target_t *query_target);
extern target_t* get_target_by_num(int num);

extern int target_write_buffer(struct target_s *target, u32 address, u32 size, u8 *buffer);
extern int target_read_buffer(struct target_s *target, u32 address, u32 size, u8 *buffer);
extern int target_checksum_memory(struct target_s *target, u32 address, u32 size, u32* crc);
extern int target_blank_check_memory(struct target_s *target, u32 address, u32 size, u32* blank);
extern int target_wait_state(target_t *target, enum target_state state, int ms);

/* DANGER!!!!!
 *
 * if "area" passed in to target_alloc_working_area() points to a memory
 * location that goes out of scope (e.g. a pointer on the stack), then
 * the caller of target_alloc_working_area() is responsible for invoking
 * target_free_working_area() before "area" goes out of scope.
 *
 * target_free_all_working_areas() will NULL out the "area" pointer
 * upon resuming or resetting the CPU.
 *
 */
extern int target_alloc_working_area(struct target_s *target, u32 size, working_area_t **area);
extern int target_free_working_area(struct target_s *target, working_area_t *area);
extern int target_free_working_area_restore(struct target_s *target, working_area_t *area, int restore);
extern void target_free_all_working_areas(struct target_s *target);
extern void target_free_all_working_areas_restore(struct target_s *target, int restore);

extern target_t *all_targets;

extern target_event_callback_t *target_event_callbacks;
extern target_timer_callback_t *target_timer_callbacks;

extern u32 target_buffer_get_u32(target_t *target, u8 *buffer);
extern u16 target_buffer_get_u16(target_t *target, u8 *buffer);
extern u8  target_buffer_get_u8 (target_t *target, u8 *buffer);
extern void target_buffer_set_u32(target_t *target, u8 *buffer, u32 value);
extern void target_buffer_set_u16(target_t *target, u8 *buffer, u16 value);
extern void target_buffer_set_u8 (target_t *target, u8 *buffer, u8  value);

int target_read_u32(struct target_s *target, u32 address, u32 *value);
int target_read_u16(struct target_s *target, u32 address, u16 *value);
int target_read_u8(struct target_s *target, u32 address, u8 *value);
int target_write_u32(struct target_s *target, u32 address, u32 value);
int target_write_u16(struct target_s *target, u32 address, u16 value);
int target_write_u8(struct target_s *target, u32 address, u8 value);

/* Issues USER() statements with target state information */
int target_arch_state(struct target_s *target);

void target_handle_event( target_t *t, enum target_event e);
void target_all_handle_event( enum target_event e );

#define ERROR_TARGET_INVALID	(-300)
#define ERROR_TARGET_INIT_FAILED (-301)
#define ERROR_TARGET_TIMEOUT	(-302)
#define ERROR_TARGET_NOT_HALTED (-304)
#define ERROR_TARGET_FAILURE	(-305)
#define ERROR_TARGET_UNALIGNED_ACCESS	(-306)
#define ERROR_TARGET_DATA_ABORT	(-307)
#define ERROR_TARGET_RESOURCE_NOT_AVAILABLE	(-308)
#define ERROR_TARGET_TRANSLATION_FAULT	(-309)
#define ERROR_TARGET_NOT_RUNNING (-310)
#define ERROR_TARGET_NOT_EXAMINED (-311)

extern const Jim_Nvp nvp_error_target[];
extern const char *target_strerror_safe( int err );

#endif /* TARGET_H */

#ifndef TARGET_TYPE_H
#define TARGET_TYPE_H

#include "types.h"

struct target_s;

struct target_type_s
{
	/**
	 * Name of the target.  Do @b not access this field directly, use
	 * target_get_name() instead.
	 */
	char *name;

	/**
	 * Indicates whether this target has been examined.
	 *
	 * Do @b not access this field directly, use target_was_examined()
	 * target_set_examined(), and target_reset_examined().
	 */
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

	/**
	 * Target register access for GDB.  Do @b not call this function
	 * directly, use target_get_gdb_reg_list() instead.
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
	/**
	 * Target memory read callback.  Do @b not call this function
	 * directly, use target_read_memory() instead.
	 */
	int (*read_memory)(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
	int (*write_memory_imp)(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
	/**
	 * Target memory write callback.  Do @b not call this function
	 * directly, use target_write_memory() instead.
	 */
	int (*write_memory)(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);

	/**
	 * Write target memory in multiples of 4 bytes, optimized for
	 * writing large quantities of data.  Do @b not call this
	 * function directly, use target_bulk_write_memory() instead.
	 */
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
	/**
	 * Target algorithm support.  Do @b not call this method directly,
	 * use target_run_algorithm() instead.
	 */
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

};

#endif // TARGET_TYPE_H

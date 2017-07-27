#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include "log.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/time_support.h"
#include "riscv.h"
#include "gdb_regs.h"
#include "rtos/rtos.h"

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

/**
 * Code structure
 *
 * At the bottom of the stack are the OpenOCD JTAG functions:
 * 		jtag_add_[id]r_scan
 * 		jtag_execute_query
 * 		jtag_add_runtest
 *
 * There are a few functions to just instantly shift a register and get its
 * value:
 * 		dtmcontrol_scan
 * 		idcode_scan
 * 		dbus_scan
 *
 * Because doing one scan and waiting for the result is slow, most functions
 * batch up a bunch of dbus writes and then execute them all at once. They use
 * the scans "class" for this:
 * 		scans_new
 * 		scans_delete
 * 		scans_execute
 * 		scans_add_...
 * Usually you new(), call a bunch of add functions, then execute() and look
 * at the results by calling scans_get...()
 *
 * Optimized functions will directly use the scans class above, but slightly
 * lazier code will use the cache functions that in turn use the scans
 * functions:
 * 		cache_get...
 * 		cache_set...
 * 		cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

// Constants for legacy SiFive hardware breakpoints.
#define CSR_BPCONTROL_X			(1<<0)
#define CSR_BPCONTROL_W			(1<<1)
#define CSR_BPCONTROL_R			(1<<2)
#define CSR_BPCONTROL_U			(1<<3)
#define CSR_BPCONTROL_S			(1<<4)
#define CSR_BPCONTROL_H			(1<<5)
#define CSR_BPCONTROL_M			(1<<6)
#define CSR_BPCONTROL_BPMATCH	(0xf<<7)
#define CSR_BPCONTROL_BPACTION	(0xff<<11)

#define DEBUG_ROM_START         0x800
#define DEBUG_ROM_RESUME        (DEBUG_ROM_START + 4)
#define DEBUG_ROM_EXCEPTION     (DEBUG_ROM_START + 8)
#define DEBUG_RAM_START         0x400

#define SETHALTNOT				0x10c

/*** JTAG registers. ***/

#define DTMCONTROL					0x10
#define DTMCONTROL_DBUS_RESET		(1<<16)
#define DTMCONTROL_IDLE				(7<<10)
#define DTMCONTROL_ADDRBITS			(0xf<<4)
#define DTMCONTROL_VERSION			(0xf)

#define DBUS						0x11
#define DBUS_OP_START				0
#define DBUS_OP_SIZE				2
typedef enum {
	DBUS_OP_NOP = 0,
	DBUS_OP_READ = 1,
	DBUS_OP_WRITE = 2
} dbus_op_t;
typedef enum {
	DBUS_STATUS_SUCCESS = 0,
	DBUS_STATUS_FAILED = 2,
	DBUS_STATUS_BUSY = 3
} dbus_status_t;
#define DBUS_DATA_START				2
#define DBUS_DATA_SIZE				34
#define DBUS_ADDRESS_START			36

typedef enum {
	RE_OK,
	RE_FAIL,
	RE_AGAIN
} riscv_error_t;

typedef enum slot {
	SLOT0,
	SLOT1,
	SLOT_LAST,
} slot_t;

/*** Debug Bus registers. ***/

#define DMCONTROL				0x10
#define DMCONTROL_INTERRUPT		(((uint64_t)1)<<33)
#define DMCONTROL_HALTNOT		(((uint64_t)1)<<32)
#define DMCONTROL_BUSERROR		(7<<19)
#define DMCONTROL_SERIAL		(3<<16)
#define DMCONTROL_AUTOINCREMENT	(1<<15)
#define DMCONTROL_ACCESS		(7<<12)
#define DMCONTROL_HARTID		(0x3ff<<2)
#define DMCONTROL_NDRESET		(1<<1)
#define DMCONTROL_FULLRESET		1

#define DMINFO					0x11
#define DMINFO_ABUSSIZE			(0x7fU<<25)
#define DMINFO_SERIALCOUNT		(0xf<<21)
#define DMINFO_ACCESS128		(1<<20)
#define DMINFO_ACCESS64			(1<<19)
#define DMINFO_ACCESS32			(1<<18)
#define DMINFO_ACCESS16			(1<<17)
#define DMINFO_ACCESS8			(1<<16)
#define DMINFO_DRAMSIZE			(0x3f<<10)
#define DMINFO_AUTHENTICATED	(1<<5)
#define DMINFO_AUTHBUSY			(1<<4)
#define DMINFO_AUTHTYPE			(3<<2)
#define DMINFO_VERSION			3

/*** Info about the core being debugged. ***/

#define DBUS_ADDRESS_UNKNOWN	0xffff
#define WALL_CLOCK_TIMEOUT		2

// gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
// its source tree. We must interpret the numbers the same here.
enum {
	REG_XPR0 = 0,
	REG_XPR31 = 31,
	REG_PC = 32,
	REG_FPR0 = 33,
	REG_FPR31 = 64,
	REG_CSR0 = 65,
	REG_MSTATUS = CSR_MSTATUS + REG_CSR0,
	REG_CSR4095 = 4160,
	REG_PRIV = 4161,
	REG_COUNT
};

#define MAX_HWBPS			16
#define DRAM_CACHE_SIZE		16

uint8_t ir_dtmcontrol[1] = {DTMCONTROL};
struct scan_field select_dtmcontrol = {
	.in_value = NULL,
	.out_value = ir_dtmcontrol
};
uint8_t ir_dbus[1] = {DBUS};
struct scan_field select_dbus = {
	.in_value = NULL,
	.out_value = ir_dbus
};
uint8_t ir_idcode[1] = {0x1};
struct scan_field select_idcode = {
	.in_value = NULL,
	.out_value = ir_idcode
};

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4];

	buf_set_u32(out_value, 0, 32, out);

	jtag_add_ir_scan(target->tap, &select_dtmcontrol, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = out_value;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCONTROL: 0x%x -> 0x%x", out, in);

	return in;
}

static struct target_type *get_target_type(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	switch (info->dtm_version) {
		case 0:
			return &riscv011_target;
		case 1:
			return &riscv013_target;
		default:
			LOG_ERROR("Unsupported DTM version: %d", info->dtm_version);
			return NULL;
	}
}

static int riscv_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("riscv_init_target()");
	target->arch_info = calloc(1, sizeof(riscv_info_t));
	if (!target->arch_info)
		return ERROR_FAIL;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	riscv_info_init(target, info);
	info->cmd_ctx = cmd_ctx;

	select_dtmcontrol.num_bits = target->tap->ir_length;
	select_dbus.num_bits = target->tap->ir_length;
	select_idcode.num_bits = target->tap->ir_length;

	return ERROR_OK;
}

static void riscv_deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	struct target_type *tt = get_target_type(target);
	tt->deinit_target(target);
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	free(info);
	target->arch_info = NULL;
}

static int oldriscv_halt(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->halt(target);
}

static void trigger_from_breakpoint(struct trigger *trigger,
		const struct breakpoint *breakpoint)
{
	trigger->address = breakpoint->address;
	trigger->length = breakpoint->length;
	trigger->mask = ~0LL;
	trigger->read = false;
	trigger->write = false;
	trigger->execute = true;
	// unique_id is unique across both breakpoints and watchpoints.
	trigger->unique_id = breakpoint->unique_id;
}

static int maybe_add_trigger_t1(struct target *target, unsigned hartid,
		struct trigger *trigger, uint64_t tdata1)
{
	RISCV_INFO(r);

	const uint32_t bpcontrol_x = 1<<0;
	const uint32_t bpcontrol_w = 1<<1;
	const uint32_t bpcontrol_r = 1<<2;
	const uint32_t bpcontrol_u = 1<<3;
	const uint32_t bpcontrol_s = 1<<4;
	const uint32_t bpcontrol_h = 1<<5;
	const uint32_t bpcontrol_m = 1<<6;
	const uint32_t bpcontrol_bpmatch = 0xf << 7;
	const uint32_t bpcontrol_bpaction = 0xff << 11;

	if (tdata1 & (bpcontrol_r | bpcontrol_w | bpcontrol_x)) {
		// Trigger is already in use, presumably by user code.
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	tdata1 = set_field(tdata1, bpcontrol_r, trigger->read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = set_field(tdata1, bpcontrol_u, !!(r->misa & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_s, !!(r->misa & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_h, !!(r->misa & (1 << ('H' - 'A'))));
	tdata1 |= bpcontrol_m;
	tdata1 = set_field(tdata1, bpcontrol_bpmatch, 0); // exact match
	tdata1 = set_field(tdata1, bpcontrol_bpaction, 0); // cause bp exception

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	riscv_reg_t tdata1_rb = riscv_get_register_on_hart(target, hartid,
			GDB_REGNO_TDATA1);
	LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

static int maybe_add_trigger_t2(struct target *target, unsigned hartid,
		struct trigger *trigger, uint64_t tdata1)
{
	RISCV_INFO(r);

	// tselect is already set
	if (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD)) {
		// Trigger is already in use, presumably by user code.
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	// address/data match trigger
	tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
	tdata1 = set_field(tdata1, MCONTROL_ACTION,
			MCONTROL_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
	tdata1 |= MCONTROL_M;
	if (r->misa & (1 << ('H' - 'A')))
		tdata1 |= MCONTROL_H;
	if (r->misa & (1 << ('S' - 'A')))
		tdata1 |= MCONTROL_S;
	if (r->misa & (1 << ('U' - 'A')))
		tdata1 |= MCONTROL_U;

	if (trigger->execute)
		tdata1 |= MCONTROL_EXECUTE;
	if (trigger->read)
		tdata1 |= MCONTROL_LOAD;
	if (trigger->write)
		tdata1 |= MCONTROL_STORE;

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	uint64_t tdata1_rb = riscv_get_register_on_hart(target, hartid, GDB_REGNO_TDATA1);
	LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

static int add_trigger(struct target *target, struct trigger *trigger)
{
	RISCV_INFO(r);

	riscv_reg_t tselect[RISCV_MAX_HARTS];

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		tselect[hartid] = riscv_get_register_on_hart(target, hartid,
				GDB_REGNO_TSELECT);
	}

	unsigned int i;
	for (i = 0; i < r->trigger_count[0]; i++) {
		if (r->trigger_unique_id[i] != -1) {
			continue;
		}

		riscv_set_register_on_hart(target, 0, GDB_REGNO_TSELECT, i);

		uint64_t tdata1 = riscv_get_register_on_hart(target, 0, GDB_REGNO_TDATA1);
		int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

		int result = ERROR_OK;
		for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
			if (!riscv_hart_enabled(target, hartid))
				continue;
			if (hartid > 0) {
				riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
			}
			switch (type) {
				case 1:
					result = maybe_add_trigger_t1(target, hartid, trigger, tdata1);
					break;
				case 2:
					result = maybe_add_trigger_t2(target, hartid, trigger, tdata1);
					break;
				default:
					LOG_DEBUG("trigger %d has unknown type %d", i, type);
					continue;
			}

			if (result != ERROR_OK) {
				continue;
			}
		}

		if (result != ERROR_OK) {
			continue;
		}

		LOG_DEBUG("Using trigger %d (type %d) for bp %d", i, type,
				trigger->unique_id);
		r->trigger_unique_id[i] = trigger->unique_id;
		break;
	}

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT,
				tselect[hartid]);
	}

	if (i >= r->trigger_count[0]) {
		LOG_ERROR("Couldn't find an available hardware trigger.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		if (target_read_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to read original instruction at 0x%" TARGET_PRIxADDR,
					breakpoint->address);
			return ERROR_FAIL;
		}

		int retval;
		if (breakpoint->length == 4) {
			retval = target_write_u32(target, breakpoint->address, ebreak());
		} else {
			retval = target_write_u16(target, breakpoint->address, ebreak_c());
		}
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write %d-byte breakpoint instruction at 0x%"
					TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = add_trigger(target, &trigger);
		if (result != ERROR_OK) {
			return result;
		}

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = true;

	return ERROR_OK;
}

static int remove_trigger(struct target *target, struct trigger *trigger)
{
	RISCV_INFO(r);

	unsigned int i;
	for (i = 0; i < r->trigger_count[0]; i++) {
		if (r->trigger_unique_id[i] == trigger->unique_id) {
			break;
		}
	}
	if (i >= r->trigger_count[0]) {
		LOG_ERROR("Couldn't find the hardware resources used by hardware "
				"trigger.");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stop using resource %d for bp %d", i, trigger->unique_id);
	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_reg_t tselect = riscv_get_register_on_hart(target, hartid, GDB_REGNO_TSELECT);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);
	}
	r->trigger_unique_id[i] = -1;

	return ERROR_OK;
}

int riscv_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		if (target_write_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to restore instruction for %d-byte breakpoint at "
					"0x%" TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = remove_trigger(target, &trigger);
		if (result != ERROR_OK) {
			return result;
		}

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = false;

	return ERROR_OK;
}

static void trigger_from_watchpoint(struct trigger *trigger,
		const struct watchpoint *watchpoint)
{
	trigger->address = watchpoint->address;
	trigger->length = watchpoint->length;
	trigger->mask = watchpoint->mask;
	trigger->value = watchpoint->value;
	trigger->read = (watchpoint->rw == WPT_READ || watchpoint->rw == WPT_ACCESS);
	trigger->write = (watchpoint->rw == WPT_WRITE || watchpoint->rw == WPT_ACCESS);
	trigger->execute = false;
	// unique_id is unique across both breakpoints and watchpoints.
	trigger->unique_id = watchpoint->unique_id;
}

int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = add_trigger(target, &trigger);
	if (result != ERROR_OK) {
		return result;
	}
	watchpoint->set = true;

	return ERROR_OK;
}

int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = remove_trigger(target, &trigger);
	if (result != ERROR_OK) {
		return result;
	}
	watchpoint->set = false;

	return ERROR_OK;
}

static int oldriscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	struct target_type *tt = get_target_type(target);
	return tt->step(target, current, address, handle_breakpoints);
}

static int old_or_new_riscv_step(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints
){
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else
		return riscv_openocd_step(target, current, address, handle_breakpoints);
}


static int riscv_examine(struct target *target)
{
	LOG_DEBUG("riscv_examine()");
	if (target_was_examined(target)) {
		LOG_DEBUG("Target was already examined.\n");
		return ERROR_OK;
	}

	// Don't need to select dbus, since the first thing we do is read dtmcontrol.

	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("  version=0x%x", info->dtm_version);

	struct target_type *tt = get_target_type(target);
	if (tt == NULL)
		return ERROR_FAIL;

	int result = tt->init_target(info->cmd_ctx, target);
	if (result != ERROR_OK)
		return result;

	return tt->examine(target);
}

static int oldriscv_poll(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->poll(target);
}

static int old_or_new_riscv_poll(struct target *target)
{
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_poll(target);
	else
		return riscv_openocd_poll(target);
}

static int old_or_new_riscv_halt(struct target *target)
{
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_halt(target);
	else
		return riscv_openocd_halt(target);
}

static int oldriscv_assert_reset(struct target *target)
{
	LOG_DEBUG("RISCV ASSERT RESET");
	struct target_type *tt = get_target_type(target);
	return tt->assert_reset(target);
}

static int oldriscv_deassert_reset(struct target *target)
{
	LOG_DEBUG("RISCV DEASSERT RESET");
	struct target_type *tt = get_target_type(target);
	return tt->deassert_reset(target);
}


static int old_or_new_riscv_assert_reset(struct target *target)
{
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_assert_reset(target);
	else
		return riscv_openocd_assert_reset(target);
}

static int old_or_new_riscv_deassert_reset(struct target *target)
{
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_deassert_reset(target);
	else
		return riscv_openocd_deassert_reset(target);
}

static int oldriscv_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	struct target_type *tt = get_target_type(target);
	return tt->resume(target, current, address, handle_breakpoints,
			debug_execution);
}

static int old_or_new_riscv_resume(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution
){
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_resume(target, current, address, handle_breakpoints, debug_execution);
	else
		return riscv_openocd_resume(target, current, address, handle_breakpoints, debug_execution);
}

static int riscv_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct target_type *tt = get_target_type(target);
	return tt->read_memory(target, address, size, count, buffer);
}

static int riscv_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct target_type *tt = get_target_type(target);
	return tt->write_memory(target, address, size, count, buffer);
}

static int riscv_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	RISCV_INFO(r);
	LOG_DEBUG("reg_class=%d", reg_class);
	LOG_DEBUG("rtos_hartid=%d current_hartid=%d", r->rtos_hartid, r->current_hartid);

	if (r->rtos_hartid != -1 && riscv_rtos_enabled(target))
		riscv_set_current_hartid(target, r->rtos_hartid);
	else
		riscv_set_current_hartid(target, target->coreid);

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = 32;
			break;
		case REG_CLASS_ALL:
			*reg_list_size = REG_COUNT;
			break;
		default:
			LOG_ERROR("Unsupported reg_class: %d", reg_class);
			return ERROR_FAIL;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list) {
		return ERROR_FAIL;
	}
	
	if (!target->reg_cache) {
		LOG_ERROR("Target not initialized. Return ERROR_FAIL.");
		return ERROR_FAIL;
	}
	
	for (int i = 0; i < *reg_list_size; i++) {
		assert(target->reg_cache->reg_list[i].size > 0);
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
	}

	return ERROR_OK;
}

static int riscv_arch_state(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->arch_state(target);
}

// Algorithm must end with a software breakpoint instruction.
static int riscv_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_params,
		struct reg_param *reg_params, target_addr_t entry_point,
		target_addr_t exit_point, int timeout_ms, void *arch_info)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	if (num_mem_params > 0) {
		LOG_ERROR("Memory parameters are not supported for RISC-V algorithms.");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/// Save registers
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK) {
		return ERROR_FAIL;
	}
	uint64_t saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	uint64_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		if (!r) {
			LOG_ERROR("Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("Register %s is %d bits instead of %d bits.",
					reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > REG_XPR31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK) {
			return ERROR_FAIL;
		}
		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);
		if (r->type->set(r, reg_params[i].value) != ERROR_OK) {
			return ERROR_FAIL;
		}
	}


	// Disable Interrupts before attempting to run the algorithm.
	uint64_t current_mstatus;
	uint8_t mstatus_bytes[8];

	LOG_DEBUG("Disabling Interrupts");
	char mstatus_name[20];
	sprintf(mstatus_name, "csr%d", CSR_MSTATUS);
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			mstatus_name, 1);
	reg_mstatus->type->get(reg_mstatus);
	current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	uint64_t ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], set_field(current_mstatus,
				ie_mask, 0));

	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/// Run algorithm
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (oldriscv_resume(target, 0, entry_point, 0, 0) != ERROR_OK) {
		return ERROR_FAIL;
	}

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %d ms.", timeout_ms);
			LOG_ERROR("  now   = 0x%08x", (uint32_t) now);
			LOG_ERROR("  start = 0x%08x", (uint32_t) start);
			oldriscv_halt(target);
			old_or_new_riscv_poll(target);
			return ERROR_TARGET_TIMEOUT;
		}

		int result = old_or_new_riscv_poll(target);
		if (result != ERROR_OK) {
			return result;
		}
	}

	if (reg_pc->type->get(reg_pc) != ERROR_OK) {
		return ERROR_FAIL;
	}
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	// Restore Interrupts
	LOG_DEBUG("Restoring Interrupts");
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], current_mstatus);
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/// Restore registers
	uint8_t buf[8];
	buf_set_u64(buf, 0, info->xlen[0], saved_pc);
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK) {
		return ERROR_FAIL;
	}

	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		buf_set_u64(buf, 0, info->xlen[0], saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK) {
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/* Should run code on the target to perform CRC of 
memory. Not yet implemented.
*/

static int riscv_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count,
		uint32_t* checksum)
{
	*checksum = 0xFFFFFFFF;
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

/* Should run code on the target to check whether a memory
block holds all-ones (because this is generally called on
NOR flash which is 1 when "blank")
Not yet implemented.
*/
int riscv_blank_check_memory(struct target * target,
				target_addr_t address,
				uint32_t count,
				uint32_t * blank,
				uint8_t erased_value)
{
	*blank = 0;

	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

/*** OpenOCD Helper Functions ***/

/* 0 means nothing happened, 1 means the hart's state changed (and thus the
 * poll should terminate), and -1 means there was an error. */
static int riscv_poll_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	riscv_set_current_hartid(target, hartid);

	LOG_DEBUG("polling hart %d, target->state=%d (TARGET_HALTED=%d)", hartid, target->state, TARGET_HALTED);

	/* If OpenOCD this we're running but this hart is halted then it's time
	 * to raise an event. */
	if (target->state != TARGET_HALTED && riscv_is_halted(target)) {
		LOG_DEBUG("  triggered a halt");
		r->on_halt(target);
		return 1;
	}

	return 0;
}

/*** OpenOCD Interface ***/
int riscv_openocd_poll(struct target *target)
{
	LOG_DEBUG("polling all harts");
	int triggered_hart = -1;
	if (riscv_rtos_enabled(target)) {
		/* Check every hart for an event. */
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			int out = riscv_poll_hart(target, i);
			switch (out) {
			case 0:
				continue;
			case 1:
				triggered_hart = i;
				break;
			case -1:
				return ERROR_FAIL;
			}
		}
		if (triggered_hart == -1) {
			LOG_DEBUG("  no harts just halted, target->state=%d", target->state);
			return ERROR_OK;
		}
		LOG_DEBUG("  hart %d halted", triggered_hart);

		/* If we're here then at least one hart triggered.  That means
		 * we want to go and halt _every_ hart in the system, as that's
		 * the invariant we hold here.  Some harts might have already
		 * halted (as we're either in single-step mode or they also
		 * triggered a breakpoint), so don't attempt to halt those
		 * harts. */
		for (int i = 0; i < riscv_count_harts(target); ++i)
			riscv_halt_one_hart(target, i);
	} else {
		if (riscv_poll_hart(target, riscv_current_hartid(target)) == 0)
			return ERROR_OK;

		triggered_hart = riscv_current_hartid(target);
		LOG_DEBUG("  hart %d halted", triggered_hart);
	}

	target->state = TARGET_HALTED;
	switch (riscv_halt_reason(target, triggered_hart)) {
	case RISCV_HALT_BREAKPOINT:
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	case RISCV_HALT_INTERRUPT:
		target->debug_reason = DBG_REASON_DBGRQ;
		break;
	case RISCV_HALT_SINGLESTEP:
		target->debug_reason = DBG_REASON_SINGLESTEP;
		break;
	}
	
	if (riscv_rtos_enabled(target)) {
		target->rtos->current_threadid = triggered_hart + 1;
		target->rtos->current_thread = triggered_hart + 1;
	}

	target->state = TARGET_HALTED;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

int riscv_openocd_halt(struct target *target)
{
	RISCV_INFO(r);

	LOG_DEBUG("halting all harts");

	int out = riscv_halt_all_harts(target);
	if (out != ERROR_OK) {
		LOG_ERROR("Unable to halt all harts");
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	if (riscv_rtos_enabled(target)) {
		target->rtos->current_threadid = r->rtos_hartid + 1;
		target->rtos->current_thread = r->rtos_hartid + 1;
	}

	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return out;
}

int riscv_openocd_resume(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution
) {
	LOG_DEBUG("resuming all harts");

	if (!current) {
		riscv_set_register(target, GDB_REGNO_PC, address);
	}

	int out = riscv_resume_all_harts(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to resume all harts");
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return out;
}

int riscv_openocd_step(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints
) {
	LOG_DEBUG("stepping rtos hart");

	if (!current) {
		riscv_set_register(target, GDB_REGNO_PC, address);
	}

	int out = riscv_step_rtos_hart(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to step rtos hart");
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_SINGLESTEP;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return out;
}

int riscv_openocd_assert_reset(struct target *target)
{
	LOG_DEBUG("asserting reset for all harts");
	int out = riscv_reset_all_harts(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to reset all harts");
		return out;
	}

	return out;
}

int riscv_openocd_deassert_reset(struct target *target)
{
	LOG_DEBUG("deasserting reset for all harts");
	if (target->reset_halt)
		riscv_halt_all_harts(target);
	else
		riscv_resume_all_harts(target);
	return ERROR_OK;
}

struct target_type riscv_target =
{
	.name = "riscv",

	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = riscv_examine,

	/* poll current target status */
	.poll = old_or_new_riscv_poll,

	.halt = old_or_new_riscv_halt,
	.resume = old_or_new_riscv_resume,
	.step = old_or_new_riscv_step,

	.assert_reset = old_or_new_riscv_assert_reset,
	.deassert_reset = old_or_new_riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,

	.blank_check_memory = riscv_blank_check_memory,
	.checksum_memory = riscv_checksum_memory,

	.get_gdb_reg_list = riscv_get_gdb_reg_list,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = riscv_run_algorithm,
};

/*** RISC-V Interface ***/

void riscv_info_init(struct target *target, riscv_info_t *r)
{
	memset(r, 0, sizeof(*r));
	r->dtm_version = 1;
	r->registers_initialized = false;
	r->current_hartid = target->coreid;

	memset(r->trigger_unique_id, 0xff, sizeof(r->trigger_unique_id));

	for (size_t h = 0; h < RISCV_MAX_HARTS; ++h) {
		r->xlen[h] = -1;
		r->debug_buffer_addr[h] = -1;

		for (size_t e = 0; e < RISCV_MAX_REGISTERS; ++e)
			r->valid_saved_registers[h][e] = false;
	}
}

int riscv_halt_all_harts(struct target *target)
{
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		riscv_halt_one_hart(target, i);
	}

	return ERROR_OK;
}

int riscv_halt_one_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	LOG_DEBUG("halting hart %d", hartid);
	riscv_set_current_hartid(target, hartid);
	if (riscv_is_halted(target)) {
		LOG_DEBUG("  hart %d requested halt, but was already halted", hartid);
		return ERROR_OK;
	}

	r->halt_current_hart(target);
	return ERROR_OK;
}

int riscv_resume_all_harts(struct target *target)
{
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		riscv_resume_one_hart(target, i);
	}

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

int riscv_resume_one_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	LOG_DEBUG("resuming hart %d", hartid);
	riscv_set_current_hartid(target, hartid);
	if (!riscv_is_halted(target)) {
		LOG_DEBUG("  hart %d requested resume, but was already resumed", hartid);
		return ERROR_OK;
	}

	r->on_resume(target);
	r->resume_current_hart(target);
	return ERROR_OK;
}

int riscv_reset_all_harts(struct target *target)
{
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		riscv_reset_one_hart(target, i);
	}

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

int riscv_reset_one_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	LOG_DEBUG("resetting hart %d", hartid);
	riscv_halt_one_hart(target, hartid);
	riscv_set_current_hartid(target, hartid);
	r->reset_current_hart(target);
	/* At this point the hart must be halted.  On platforms that support
	 * "reset halt" exactly we expect the hart to have been halted before
	 * executing any instructions, while on older cores it'll just have
	 * halted quickly. */
	return ERROR_OK;
}

int riscv_step_rtos_hart(struct target *target)
{
	RISCV_INFO(r);
	int hartid = r->current_hartid;
	if (riscv_rtos_enabled(target)) {
		hartid = r->rtos_hartid;
		if (hartid == -1) {
			LOG_USER("GDB has asked me to step \"any\" thread, so I'm stepping hart 0.");
			hartid = 0;
		}
	}
	riscv_set_current_hartid(target, hartid);
	LOG_DEBUG("stepping hart %d", hartid);

	assert(riscv_is_halted(target));
	riscv_invalidate_register_cache(target);
	r->on_step(target);
	r->step_current_hart(target);
	riscv_invalidate_register_cache(target);
	r->on_halt(target);
	assert(riscv_is_halted(target));
	return ERROR_OK;
}

int riscv_xlen(const struct target *target)
{
	return riscv_xlen_of_hart(target, riscv_current_hartid(target));
}

int riscv_xlen_of_hart(const struct target *target, int hartid)
{
	RISCV_INFO(r);
	assert(r->xlen[hartid] != -1);
	return r->xlen[hartid];
}

bool riscv_rtos_enabled(const struct target *target)
{
	return target->rtos != NULL;
}

void riscv_set_current_hartid(struct target *target, int hartid)
{
	RISCV_INFO(r);
	if (!r->select_current_hart)
		return;

	int previous_hartid = riscv_current_hartid(target);
	r->current_hartid = hartid;
	assert(riscv_hart_enabled(target, hartid));
	LOG_DEBUG("setting hartid to %d, was %d", hartid, previous_hartid);
	r->select_current_hart(target);

	/* This might get called during init, in which case we shouldn't be
	 * setting up the register cache. */
	if (!target_was_examined(target))
		return;

	/* Avoid invalidating the register cache all the time. */
	if (r->registers_initialized
			&& (!riscv_rtos_enabled(target) || (previous_hartid == hartid))
			&& target->reg_cache->reg_list[GDB_REGNO_XPR0].size == (unsigned)riscv_xlen(target)
			&& (!riscv_rtos_enabled(target) || (r->rtos_hartid != -1))) {
		return;
	} else
		LOG_DEBUG("Initializing registers: xlen=%d", riscv_xlen(target));

	riscv_invalidate_register_cache(target);
}

void riscv_invalidate_register_cache(struct target *target)
{
	RISCV_INFO(r);

	/* Update the register list's widths. */
	register_cache_invalidate(target->reg_cache);
	for (size_t i = 0; i < GDB_REGNO_COUNT; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];

		reg->value = &r->reg_cache_values[i];
		reg->valid = false;

		switch (i) {
		case GDB_REGNO_PRIV:
			reg->size = 8;
			break;
		default:
			reg->size = riscv_xlen(target);
			break;
		}
	}

	r->registers_initialized = true;
}

int riscv_current_hartid(const struct target *target)
{
	RISCV_INFO(r);
	return r->current_hartid;
}

void riscv_set_all_rtos_harts(struct target *target)
{
	RISCV_INFO(r);
	r->rtos_hartid = -1;
}

void riscv_set_rtos_hartid(struct target *target, int hartid)
{
	LOG_DEBUG("setting RTOS hartid %d", hartid);
	RISCV_INFO(r);
	r->rtos_hartid = hartid;
}

int riscv_count_harts(struct target *target)
{
	if (target == NULL) return 1;
	RISCV_INFO(r);
	if (r == NULL) return 1;
	return r->hart_count;
}

bool riscv_has_register(struct target *target, int hartid, int regid)
{
	return 1;
}

void riscv_set_register(struct target *target, enum gdb_regno r, riscv_reg_t v)
{
	return riscv_set_register_on_hart(target, riscv_current_hartid(target), r, v);
}

void riscv_set_register_on_hart(struct target *target, int hartid,
		enum gdb_regno regid, uint64_t value)
{
	RISCV_INFO(r);
	LOG_DEBUG("[%d] %s <- %" PRIx64, hartid, gdb_regno_name(regid), value);
	assert(r->set_register);
	return r->set_register(target, hartid, regid, value);
}

riscv_reg_t riscv_get_register(struct target *target, enum gdb_regno r)
{
	return riscv_get_register_on_hart(target, riscv_current_hartid(target), r);
}

uint64_t riscv_get_register_on_hart(struct target *target, int hartid, enum gdb_regno regid)
{
	RISCV_INFO(r);
	uint64_t value = r->get_register(target, hartid, regid);
	LOG_DEBUG("[%d] %s: %" PRIx64, hartid, gdb_regno_name(regid), value);
	return value;
}

bool riscv_is_halted(struct target *target)
{
	RISCV_INFO(r);
	assert(r->is_halted);
	return r->is_halted(target);
}

enum riscv_halt_reason riscv_halt_reason(struct target *target, int hartid)
{
	RISCV_INFO(r);
	riscv_set_current_hartid(target, hartid);
	assert(riscv_is_halted(target));
	return r->halt_reason(target);
}

int riscv_count_triggers(struct target *target)
{
	return riscv_count_triggers_of_hart(target, riscv_current_hartid(target));
}

int riscv_count_triggers_of_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	assert(hartid < riscv_count_harts(target));
	return r->trigger_count[hartid];
}

size_t riscv_debug_buffer_size(struct target *target)
{
	RISCV_INFO(r);
	return r->debug_buffer_size[riscv_current_hartid(target)];
}

riscv_addr_t riscv_debug_buffer_addr(struct target *target)
{
	RISCV_INFO(r);
	riscv_addr_t out = r->debug_buffer_addr[riscv_current_hartid(target)];
	assert((out & 3) == 0);
	return out;
}

int riscv_debug_buffer_enter(struct target *target, struct riscv_program *program)
{
	RISCV_INFO(r);
	r->debug_buffer_enter(target, program);
	return ERROR_OK;
}

int riscv_debug_buffer_leave(struct target *target, struct riscv_program *program)
{
	RISCV_INFO(r);
	r->debug_buffer_leave(target, program);
	return ERROR_OK;
}

int riscv_write_debug_buffer(struct target *target, int index, riscv_insn_t insn)
{
	RISCV_INFO(r);
	r->write_debug_buffer(target, index, insn);
	return ERROR_OK;
}

riscv_insn_t riscv_read_debug_buffer(struct target *target, int index)
{
	RISCV_INFO(r);
	return r->read_debug_buffer(target, index);
}

riscv_addr_t riscv_read_debug_buffer_x(struct target *target, int index)
{
	riscv_addr_t out = 0;
	switch (riscv_xlen(target)) {
	case 64:
		out |= (uint64_t)riscv_read_debug_buffer(target, index + 1) << 32;
	case 32:
		out |= riscv_read_debug_buffer(target, index + 0) <<  0;
		break;
	default:
		LOG_ERROR("unsupported XLEN %d", riscv_xlen(target));
		abort();
	}
	return out;
}

int riscv_execute_debug_buffer(struct target *target)
{
	RISCV_INFO(r);
	return r->execute_debug_buffer(target);
}

void riscv_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d)
{
	RISCV_INFO(r);
	r->fill_dmi_write_u64(target, buf, a, d);
}

void riscv_fill_dmi_read_u64(struct target *target, char *buf, int a)
{
	RISCV_INFO(r);
	r->fill_dmi_read_u64(target, buf, a);
}

void riscv_fill_dmi_nop_u64(struct target *target, char *buf)
{
	RISCV_INFO(r);
	r->fill_dmi_nop_u64(target, buf);
}

int riscv_dmi_write_u64_bits(struct target *target)
{
	RISCV_INFO(r);
	return r->dmi_write_u64_bits(target);
}

bool riscv_hart_enabled(struct target *target, int hartid)
{
	/* FIXME: Add a hart mask to the RTOS. */
	if (riscv_rtos_enabled(target))
		return hartid < riscv_count_harts(target);

	return hartid == target->coreid;
}

/**
 * Count triggers, and initialize trigger_count for each hart.
 * trigger_count is initialized even if this function fails to discover
 * something.
 * Disable any hardware triggers that have dmode set. We can't have set them
 * ourselves. Maybe they're left over from some killed debug session.
 * */
int riscv_enumerate_triggers(struct target *target)
{
	RISCV_INFO(r);

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect = riscv_get_register_on_hart(target, hartid,
				GDB_REGNO_TSELECT);

		for (unsigned t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
			r->trigger_count[hartid] = t;

			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, t);
			uint64_t tselect_rb = riscv_get_register_on_hart(target, hartid,
					GDB_REGNO_TSELECT);
			// Mask off the top bit, which is used as tdrmode in old
			// implementations.
			tselect_rb &= ~(1ULL << (riscv_xlen(target)-1));
			if (tselect_rb != t)
				break;

			uint64_t tdata1 = riscv_get_register_on_hart(target, hartid,
					GDB_REGNO_TDATA1);
			int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));
			switch (type) {
				case 1:
					// On these older cores we don't support software using
					// triggers.
					riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
					break;
				case 2:
					if (tdata1 & MCONTROL_DMODE(riscv_xlen(target))) {
						riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
					}
					break;
			}
		}

		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);

		LOG_INFO("[%d] Found %d triggers", hartid, r->trigger_count[hartid]);
	}

	return ERROR_OK;
}

const char *gdb_regno_name(enum gdb_regno regno)
{
	static char buf[32];

	switch (regno) {
		case GDB_REGNO_ZERO:
			return "zero";
		case GDB_REGNO_S0:
			return "s0";
		case GDB_REGNO_S1:
			return "s1";
		case GDB_REGNO_PC:
			return "pc";
		case GDB_REGNO_FPR0:
			return "fpr0";
		case GDB_REGNO_FPR31:
			return "fpr31";
		case GDB_REGNO_CSR0:
			return "csr0";
		case GDB_REGNO_TSELECT:
			return "tselect";
		case GDB_REGNO_TDATA1:
			return "tdata1";
		case GDB_REGNO_TDATA2:
			return "tdata2";
		case GDB_REGNO_MISA:
			return "misa";
		case GDB_REGNO_DPC:
			return "dpc";
		case GDB_REGNO_DCSR:
			return "dcsr";
		case GDB_REGNO_DSCRATCH:
			return "dscratch";
		case GDB_REGNO_MSTATUS:
			return "mstatus";
		case GDB_REGNO_PRIV:
			return "priv";
		default:
			if (regno <= GDB_REGNO_XPR31) {
				sprintf(buf, "x%d", regno - GDB_REGNO_XPR0);
			} else if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095) {
				sprintf(buf, "csr%d", regno - GDB_REGNO_CSR0);
			} else {
				sprintf(buf, "gdb_regno_%d", regno);
			}
			return buf;
	}
}

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
 *		jtag_add_[id]r_scan
 *		jtag_execute_query
 *		jtag_add_runtest
 *
 * There are a few functions to just instantly shift a register and get its
 * value:
 *		dtmcontrol_scan
 *		idcode_scan
 *		dbus_scan
 *
 * Because doing one scan and waiting for the result is slow, most functions
 * batch up a bunch of dbus writes and then execute them all at once. They use
 * the scans "class" for this:
 *		scans_new
 *		scans_delete
 *		scans_execute
 *		scans_add_...
 * Usually you new(), call a bunch of add functions, then execute() and look
 * at the results by calling scans_get...()
 *
 * Optimized functions will directly use the scans class above, but slightly
 * lazier code will use the cache functions that in turn use the scans
 * functions:
 *		cache_get...
 *		cache_set...
 *		cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

/* Constants for legacy SiFive hardware breakpoints. */
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
#define DEBUG_ROM_RESUME	(DEBUG_ROM_START + 4)
#define DEBUG_ROM_EXCEPTION	(DEBUG_ROM_START + 8)
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

/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
int riscv_command_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;

/* Wall-clock timeout after reset. Settable via RISC-V Target commands.*/
int riscv_reset_timeout_sec = DEFAULT_RESET_TIMEOUT_SEC;

bool riscv_prefer_sba;

/* In addition to the ones in the standard spec, we'll also expose additional
 * CSRs in this list.
 * The list is either NULL, or a series of ranges (inclusive), terminated with
 * 1,0. */
struct {
	uint16_t low, high;
} *expose_csr;

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

	if (!info) {
		LOG_ERROR("Target has not been initialized");
		return NULL;
	}

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

	riscv_semihosting_init(target);

	return ERROR_OK;
}

static void riscv_deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	struct target_type *tt = get_target_type(target);
	if (tt) {
		tt->deinit_target(target);
		riscv_info_t *info = (riscv_info_t *) target->arch_info;
		free(info);
	}
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
	/* unique_id is unique across both breakpoints and watchpoints. */
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
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	tdata1 = set_field(tdata1, bpcontrol_r, trigger->read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = set_field(tdata1, bpcontrol_u,
			!!(r->misa[hartid] & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_s,
			!!(r->misa[hartid] & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_h,
			!!(r->misa[hartid] & (1 << ('H' - 'A'))));
	tdata1 |= bpcontrol_m;
	tdata1 = set_field(tdata1, bpcontrol_bpmatch, 0); /* exact match */
	tdata1 = set_field(tdata1, bpcontrol_bpaction, 0); /* cause bp exception */

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	riscv_reg_t tdata1_rb;
	if (riscv_get_register_on_hart(target, &tdata1_rb, hartid,
				GDB_REGNO_TDATA1) != ERROR_OK)
		return ERROR_FAIL;
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

	/* tselect is already set */
	if (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD)) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* address/data match trigger */
	tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
	tdata1 = set_field(tdata1, MCONTROL_ACTION,
			MCONTROL_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
	tdata1 |= MCONTROL_M;
	if (r->misa[hartid] & (1 << ('H' - 'A')))
		tdata1 |= MCONTROL_H;
	if (r->misa[hartid] & (1 << ('S' - 'A')))
		tdata1 |= MCONTROL_S;
	if (r->misa[hartid] & (1 << ('U' - 'A')))
		tdata1 |= MCONTROL_U;

	if (trigger->execute)
		tdata1 |= MCONTROL_EXECUTE;
	if (trigger->read)
		tdata1 |= MCONTROL_LOAD;
	if (trigger->write)
		tdata1 |= MCONTROL_STORE;

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	uint64_t tdata1_rb;
	int result = riscv_get_register_on_hart(target, &tdata1_rb, hartid, GDB_REGNO_TDATA1);
	if (result != ERROR_OK)
		return result;
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

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	/* In RTOS mode, we need to set the same trigger in the same slot on every
	 * hart, to keep up the illusion that each hart is a thread running on the
	 * same core. */

	/* Otherwise, we just set the trigger on the one hart this target deals
	 * with. */

	riscv_reg_t tselect[RISCV_MAX_HARTS];

	int first_hart = -1;
	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		if (first_hart < 0)
			first_hart = hartid;
		int result = riscv_get_register_on_hart(target, &tselect[hartid],
				hartid, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
	}
	assert(first_hart >= 0);

	unsigned int i;
	for (i = 0; i < r->trigger_count[first_hart]; i++) {
		if (r->trigger_unique_id[i] != -1)
			continue;

		riscv_set_register_on_hart(target, first_hart, GDB_REGNO_TSELECT, i);

		uint64_t tdata1;
		int result = riscv_get_register_on_hart(target, &tdata1, first_hart,
				GDB_REGNO_TDATA1);
		if (result != ERROR_OK)
			return result;
		int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

		result = ERROR_OK;
		for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
			if (!riscv_hart_enabled(target, hartid))
				continue;
			if (hartid > first_hart)
				riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
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

			if (result != ERROR_OK)
				continue;
		}

		if (result != ERROR_OK)
			continue;

		LOG_DEBUG("Using trigger %d (type %d) for bp %d", i, type,
				trigger->unique_id);
		r->trigger_unique_id[i] = trigger->unique_id;
		break;
	}

	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT,
				tselect[hartid]);
	}

	if (i >= r->trigger_count[first_hart]) {
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
		if (breakpoint->length == 4)
			retval = target_write_u32(target, breakpoint->address, ebreak());
		else
			retval = target_write_u16(target, breakpoint->address, ebreak_c());
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write %d-byte breakpoint instruction at 0x%"
					TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = add_trigger(target, &trigger);
		if (result != ERROR_OK)
			return result;

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

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	int first_hart = -1;
	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		if (first_hart < 0) {
			first_hart = hartid;
			break;
		}
	}
	assert(first_hart >= 0);

	unsigned int i;
	for (i = 0; i < r->trigger_count[first_hart]; i++) {
		if (r->trigger_unique_id[i] == trigger->unique_id)
			break;
	}
	if (i >= r->trigger_count[first_hart]) {
		LOG_ERROR("Couldn't find the hardware resources used by hardware "
				"trigger.");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stop using resource %d for bp %d", i, trigger->unique_id);
	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_reg_t tselect;
		int result = riscv_get_register_on_hart(target, &tselect, hartid, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
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
		if (result != ERROR_OK)
			return result;

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
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = watchpoint->unique_id;
}

int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = add_trigger(target, &trigger);
	if (result != ERROR_OK)
		return result;
	watchpoint->set = true;

	return ERROR_OK;
}

int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = remove_trigger(target, &trigger);
	if (result != ERROR_OK)
		return result;
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
	LOG_DEBUG("handle_breakpoints=%d", handle_breakpoints);
	if (r->is_halted == NULL)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else
		return riscv_openocd_step(target, current, address, handle_breakpoints);
}


static int riscv_examine(struct target *target)
{
	LOG_DEBUG("riscv_examine()");
	if (target_was_examined(target)) {
		LOG_DEBUG("Target was already examined.");
		return ERROR_OK;
	}

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

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

static int riscv_assert_reset(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->assert_reset(target);
}

static int riscv_deassert_reset(struct target *target)
{
	LOG_DEBUG("RISCV DEASSERT RESET");
	struct target_type *tt = get_target_type(target);
	return tt->deassert_reset(target);
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
	LOG_DEBUG("handle_breakpoints=%d", handle_breakpoints);
	if (r->is_halted == NULL)
		return oldriscv_resume(target, current, address, handle_breakpoints, debug_execution);
	else
		return riscv_openocd_resume(target, current, address, handle_breakpoints, debug_execution);
}

static int riscv_select_current_hart(struct target *target)
{
	RISCV_INFO(r);
	if (r->rtos_hartid != -1 && riscv_rtos_enabled(target))
		return riscv_set_current_hartid(target, r->rtos_hartid);
	else
		return riscv_set_current_hartid(target, target->coreid);
}

static int riscv_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	struct target_type *tt = get_target_type(target);
	return tt->read_memory(target, address, size, count, buffer);
}

static int riscv_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
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

	if (!target->reg_cache) {
		LOG_ERROR("Target not initialized. Return ERROR_FAIL.");
		return ERROR_FAIL;
	}

	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = 32;
			break;
		case REG_CLASS_ALL:
			*reg_list_size = GDB_REGNO_COUNT;
			break;
		default:
			LOG_ERROR("Unsupported reg_class: %d", reg_class);
			return ERROR_FAIL;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	for (int i = 0; i < *reg_list_size; i++) {
		assert(!target->reg_cache->reg_list[i].valid ||
				target->reg_cache->reg_list[i].size > 0);
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
	}

	return ERROR_OK;
}

static int riscv_arch_state(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->arch_state(target);
}

/* Algorithm must end with a software breakpoint instruction. */
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

	/* Save registers */
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	uint64_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;

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

		if (r->number > GDB_REGNO_XPR31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK)
			return ERROR_FAIL;
		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);
		if (r->type->set(r, reg_params[i].value) != ERROR_OK)
			return ERROR_FAIL;
	}


	/* Disable Interrupts before attempting to run the algorithm. */
	uint64_t current_mstatus;
	uint8_t mstatus_bytes[8];

	LOG_DEBUG("Disabling Interrupts");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", 1);
	if (!reg_mstatus) {
		LOG_ERROR("Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	reg_mstatus->type->get(reg_mstatus);
	current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	uint64_t ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], set_field(current_mstatus,
				ie_mask, 0));

	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Run algorithm */
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (oldriscv_resume(target, 0, entry_point, 0, 0) != ERROR_OK)
		return ERROR_FAIL;

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
		if (result != ERROR_OK)
			return result;
	}

	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	/* Restore Interrupts */
	LOG_DEBUG("Restoring Interrupts");
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], current_mstatus);
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Restore registers */
	uint8_t buf[8];
	buf_set_u64(buf, 0, info->xlen[0], saved_pc);
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		buf_set_u64(buf, 0, info->xlen[0], saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* Should run code on the target to perform CRC of
memory. Not yet implemented.
*/

static int riscv_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count,
		uint32_t *checksum)
{
	*checksum = 0xFFFFFFFF;
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

/*** OpenOCD Helper Functions ***/

enum riscv_poll_hart {
	RPH_NO_CHANGE,
	RPH_DISCOVERED_HALTED,
	RPH_DISCOVERED_RUNNING,
	RPH_ERROR
};
static enum riscv_poll_hart riscv_poll_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return RPH_ERROR;

	LOG_DEBUG("polling hart %d, target->state=%d", hartid, target->state);

	/* If OpenOCD thinks we're running but this hart is halted then it's time
	 * to raise an event. */
	bool halted = riscv_is_halted(target);
	if (target->state != TARGET_HALTED && halted) {
		LOG_DEBUG("  triggered a halt");
		r->on_halt(target);
		return RPH_DISCOVERED_HALTED;
	} else if (target->state != TARGET_RUNNING && !halted) {
		LOG_DEBUG("  triggered running");
		target->state = TARGET_RUNNING;
		return RPH_DISCOVERED_RUNNING;
	}

	return RPH_NO_CHANGE;
}

/*** OpenOCD Interface ***/
int riscv_openocd_poll(struct target *target)
{
	LOG_DEBUG("polling all harts");
	int halted_hart = -1;
	if (riscv_rtos_enabled(target)) {
		/* Check every hart for an event. */
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			enum riscv_poll_hart out = riscv_poll_hart(target, i);
			switch (out) {
			case RPH_NO_CHANGE:
			case RPH_DISCOVERED_RUNNING:
				continue;
			case RPH_DISCOVERED_HALTED:
				halted_hart = i;
				break;
			case RPH_ERROR:
				return ERROR_FAIL;
			}
		}
		if (halted_hart == -1) {
			LOG_DEBUG("  no harts just halted, target->state=%d", target->state);
			return ERROR_OK;
		}
		LOG_DEBUG("  hart %d halted", halted_hart);

		/* If we're here then at least one hart triggered.  That means
		 * we want to go and halt _every_ hart in the system, as that's
		 * the invariant we hold here.	Some harts might have already
		 * halted (as we're either in single-step mode or they also
		 * triggered a breakpoint), so don't attempt to halt those
		 * harts. */
		for (int i = 0; i < riscv_count_harts(target); ++i)
			riscv_halt_one_hart(target, i);
	} else {
		enum riscv_poll_hart out = riscv_poll_hart(target,
				riscv_current_hartid(target));
		if (out == RPH_NO_CHANGE || out == RPH_DISCOVERED_RUNNING)
			return ERROR_OK;
		else if (out == RPH_ERROR)
			return ERROR_FAIL;

		halted_hart = riscv_current_hartid(target);
		LOG_DEBUG("  hart %d halted", halted_hart);
	}

	target->state = TARGET_HALTED;
	switch (riscv_halt_reason(target, halted_hart)) {
	case RISCV_HALT_BREAKPOINT:
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	case RISCV_HALT_TRIGGER:
		target->debug_reason = DBG_REASON_WATCHPOINT;
		break;
	case RISCV_HALT_INTERRUPT:
		target->debug_reason = DBG_REASON_DBGRQ;
		break;
	case RISCV_HALT_SINGLESTEP:
		target->debug_reason = DBG_REASON_SINGLESTEP;
		break;
	case RISCV_HALT_UNKNOWN:
		target->debug_reason = DBG_REASON_UNDEFINED;
		break;
	case RISCV_HALT_ERROR:
		return ERROR_FAIL;
	}

	if (riscv_rtos_enabled(target)) {
		target->rtos->current_threadid = halted_hart + 1;
		target->rtos->current_thread = halted_hart + 1;
	}

	target->state = TARGET_HALTED;

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		int retval;
		if (riscv_semihosting(target, &retval) != 0)
			return retval;
	}

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
		int debug_execution)
{
	LOG_DEBUG("debug_reason=%d", target->debug_reason);

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		/* To be able to run off a trigger, disable all the triggers, step, and
		 * then resume as usual. */
		struct watchpoint *watchpoint = target->watchpoints;
		bool trigger_temporarily_cleared[RISCV_MAX_HWBPS] = {0};

		int i = 0;
		int result = ERROR_OK;
		while (watchpoint && result == ERROR_OK) {
			LOG_DEBUG("watchpoint %d: set=%d", i, watchpoint->set);
			trigger_temporarily_cleared[i] = watchpoint->set;
			if (watchpoint->set)
				result = riscv_remove_watchpoint(target, watchpoint);
			watchpoint = watchpoint->next;
			i++;
		}

		if (result == ERROR_OK)
			result = riscv_step_rtos_hart(target);

		watchpoint = target->watchpoints;
		i = 0;
		while (watchpoint) {
			LOG_DEBUG("watchpoint %d: cleared=%d", i, trigger_temporarily_cleared[i]);
			if (trigger_temporarily_cleared[i]) {
				if (result == ERROR_OK)
					result = riscv_add_watchpoint(target, watchpoint);
				else
					riscv_add_watchpoint(target, watchpoint);
			}
			watchpoint = watchpoint->next;
			i++;
		}

		if (result != ERROR_OK)
			return result;
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

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

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

/* Command Handlers */
COMMAND_HANDLER(riscv_set_command_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int timeout = atoi(CMD_ARGV[0]);
	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	riscv_command_timeout_sec = timeout;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_reset_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int timeout = atoi(CMD_ARGV[0]);
	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	riscv_reset_timeout_sec = timeout;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_prefer_sba)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_prefer_sba);
	return ERROR_OK;
}

void parse_error(const char *string, char c, unsigned position)
{
	char buf[position+2];
	for (unsigned i = 0; i < position; i++)
		buf[i] = ' ';
	buf[position] = '^';
	buf[position + 1] = 0;

	LOG_ERROR("Parse error at character %c in:", c);
	LOG_ERROR("%s", string);
	LOG_ERROR("%s", buf);
}

COMMAND_HANDLER(riscv_set_expose_csrs)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (unsigned pass = 0; pass < 2; pass++) {
		unsigned range = 0;
		unsigned low = 0;
		bool parse_low = true;
		unsigned high = 0;
		for (unsigned i = 0; i == 0 || CMD_ARGV[0][i-1]; i++) {
			char c = CMD_ARGV[0][i];
			if (isspace(c)) {
				/* Ignore whitespace. */
				continue;
			}

			if (parse_low) {
				if (isdigit(c)) {
					low *= 10;
					low += c - '0';
				} else if (c == '-') {
					parse_low = false;
				} else if (c == ',' || c == 0) {
					if (pass == 1) {
						expose_csr[range].low = low;
						expose_csr[range].high = low;
					}
					low = 0;
					range++;
				} else {
					parse_error(CMD_ARGV[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

			} else {
				if (isdigit(c)) {
					high *= 10;
					high += c - '0';
				} else if (c == ',' || c == 0) {
					parse_low = true;
					if (pass == 1) {
						expose_csr[range].low = low;
						expose_csr[range].high = high;
					}
					low = 0;
					high = 0;
					range++;
				} else {
					parse_error(CMD_ARGV[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}
		}

		if (pass == 0) {
			if (expose_csr)
				free(expose_csr);
			expose_csr = calloc(range + 2, sizeof(*expose_csr));
		} else {
			expose_csr[range].low = 1;
			expose_csr[range].high = 0;
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_authdata_read)
{
	if (CMD_ARGC != 0) {
		LOG_ERROR("Command takes no parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}

	RISCV_INFO(r);
	if (!r) {
		LOG_ERROR("riscv_info is NULL!");
		return ERROR_FAIL;
	}

	if (r->authdata_read) {
		uint32_t value;
		if (r->authdata_read(target, &value) != ERROR_OK)
			return ERROR_FAIL;
		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("authdata_read is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_authdata_write)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 argument");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], value);

	if (r->authdata_write) {
		return r->authdata_write(target, value);
	} else {
		LOG_ERROR("authdata_write is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_dmi_read)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}

	RISCV_INFO(r);
	if (!r) {
		LOG_ERROR("riscv_info is NULL!");
		return ERROR_FAIL;
	}

	if (r->dmi_read) {
		uint32_t address, value;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
		if (r->dmi_read(target, &value, address) != ERROR_OK)
			return ERROR_FAIL;
		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("dmi_read is not implemented for this target.");
		return ERROR_FAIL;
	}
}


COMMAND_HANDLER(riscv_dmi_write)
{
	if (CMD_ARGC != 2) {
		LOG_ERROR("Command takes exactly 2 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	uint32_t address, value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	if (r->dmi_write) {
		return r->dmi_write(target, address, value);
	} else {
		LOG_ERROR("dmi_write is not implemented for this target.");
		return ERROR_FAIL;
	}
}

static const struct command_registration riscv_exec_command_handlers[] = {
	{
		.name = "set_command_timeout_sec",
		.handler = riscv_set_command_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "riscv set_command_timeout_sec [sec]",
		.help = "Set the wall-clock timeout (in seconds) for individual commands"
	},
	{
		.name = "set_reset_timeout_sec",
		.handler = riscv_set_reset_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "riscv set_reset_timeout_sec [sec]",
		.help = "Set the wall-clock timeout (in seconds) after reset is deasserted"
	},
	{
		.name = "set_prefer_sba",
		.handler = riscv_set_prefer_sba,
		.mode = COMMAND_ANY,
		.usage = "riscv set_prefer_sba on|off",
		.help = "When on, prefer to use System Bus Access to access memory. "
			"When off, prefer to use the Program Buffer to access memory."
	},
	{
		.name = "expose_csrs",
		.handler = riscv_set_expose_csrs,
		.mode = COMMAND_ANY,
		.usage = "riscv expose_csrs n0[-m0][,n1[-m1]]...",
		.help = "Configure a list of inclusive ranges for CSRs to expose in "
				"addition to the standard ones. This must be executed before "
				"`init`."
	},
	{
		.name = "authdata_read",
		.handler = riscv_authdata_read,
		.mode = COMMAND_ANY,
		.usage = "riscv authdata_read",
		.help = "Return the 32-bit value read from authdata."
	},
	{
		.name = "authdata_write",
		.handler = riscv_authdata_write,
		.mode = COMMAND_ANY,
		.usage = "riscv authdata_write value",
		.help = "Write the 32-bit value to authdata."
	},
	{
		.name = "dmi_read",
		.handler = riscv_dmi_read,
		.mode = COMMAND_ANY,
		.usage = "riscv dmi_read address",
		.help = "Perform a 32-bit DMI read at address, returning the value."
	},
	{
		.name = "dmi_write",
		.handler = riscv_dmi_write,
		.mode = COMMAND_ANY,
		.usage = "riscv dmi_write address value",
		.help = "Perform a 32-bit DMI write of value at address."
	},
	COMMAND_REGISTRATION_DONE
};

extern __COMMAND_HANDLER(handle_common_semihosting_command);
extern __COMMAND_HANDLER(handle_common_semihosting_fileio_command);
extern __COMMAND_HANDLER(handle_common_semihosting_resumable_exit_command);
extern __COMMAND_HANDLER(handle_common_semihosting_cmdline);

/*
 * To be noted that RISC-V targets use the same semihosting commands as
 * ARM targets.
 *
 * The main reason is compatibility with existing tools. For example the
 * Eclipse OpenOCD/SEGGER J-Link/QEMU plug-ins have several widgets to
 * configure semihosting, which generate commands like `arm semihosting
 * enable`.
 * A secondary reason is the fact that the protocol used is exactly the
 * one specified by ARM. If RISC-V will ever define its own semihosting
 * protocol, then a command like `riscv semihosting enable` will make
 * sense, but for now all semihosting commands are prefixed with `arm`.
 */
static const struct command_registration arm_exec_command_handlers[] = {
	{
		"semihosting",
		.handler = handle_common_semihosting_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting operations",
	},
	{
		"semihosting_cmdline",
		.handler = handle_common_semihosting_cmdline,
		.mode = COMMAND_EXEC,
		.usage = "arguments",
		.help = "command line arguments to be passed to program",
	},
	{
		"semihosting_fileio",
		.handler = handle_common_semihosting_fileio_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting fileio operations",
	},
	{
		"semihosting_resexit",
		.handler = handle_common_semihosting_resumable_exit_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting resumable exit",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration riscv_command_handlers[] = {
	{
		.name = "riscv",
		.mode = COMMAND_ANY,
		.help = "RISC-V Command Group",
		.usage = "",
		.chain = riscv_exec_command_handlers
	},
	{
		.name = "arm",
		.mode = COMMAND_ANY,
		.help = "ARM Command Group",
		.usage = "",
		.chain = arm_exec_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type riscv_target = {
	.name = "riscv",

	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = riscv_examine,

	/* poll current target status */
	.poll = old_or_new_riscv_poll,

	.halt = old_or_new_riscv_halt,
	.resume = old_or_new_riscv_resume,
	.step = old_or_new_riscv_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_reg_list = riscv_get_gdb_reg_list,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = riscv_run_algorithm,

	.commands = riscv_command_handlers
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
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_is_halted(target)) {
		LOG_DEBUG("  hart %d requested halt, but was already halted", hartid);
		return ERROR_OK;
	}

	return r->halt_current_hart(target);
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
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return ERROR_FAIL;
	if (!riscv_is_halted(target)) {
		LOG_DEBUG("  hart %d requested resume, but was already resumed", hartid);
		return ERROR_OK;
	}

	r->on_resume(target);
	return r->resume_current_hart(target);
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
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("stepping hart %d", hartid);

	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart isn't halted before single step!");
		return ERROR_FAIL;
	}
	riscv_invalidate_register_cache(target);
	r->on_step(target);
	if (r->step_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	riscv_invalidate_register_cache(target);
	r->on_halt(target);
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart was not halted after single step!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

bool riscv_supports_extension(struct target *target, int hartid, char letter)
{
	RISCV_INFO(r);
	unsigned num;
	if (letter >= 'a' && letter <= 'z')
		num = letter - 'a';
	else if (letter >= 'A' && letter <= 'Z')
		num = letter - 'A';
	else
		return false;
	return r->misa[hartid] & (1 << num);
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

int riscv_set_current_hartid(struct target *target, int hartid)
{
	RISCV_INFO(r);
	if (!r->select_current_hart)
		return ERROR_OK;

	int previous_hartid = riscv_current_hartid(target);
	r->current_hartid = hartid;
	assert(riscv_hart_enabled(target, hartid));
	LOG_DEBUG("setting hartid to %d, was %d", hartid, previous_hartid);
	if (r->select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	/* This might get called during init, in which case we shouldn't be
	 * setting up the register cache. */
	if (!target_was_examined(target))
		return ERROR_OK;

	/* Avoid invalidating the register cache all the time. */
	if (r->registers_initialized
			&& (!riscv_rtos_enabled(target) || (previous_hartid == hartid))
			&& target->reg_cache->reg_list[GDB_REGNO_ZERO].size == (unsigned)riscv_xlen(target)
			&& (!riscv_rtos_enabled(target) || (r->rtos_hartid != -1))) {
		return ERROR_OK;
	} else
		LOG_DEBUG("Initializing registers: xlen=%d", riscv_xlen(target));

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

void riscv_invalidate_register_cache(struct target *target)
{
	RISCV_INFO(r);

	register_cache_invalidate(target->reg_cache);
	for (size_t i = 0; i < GDB_REGNO_COUNT; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];
		reg->valid = false;
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
	if (target == NULL)
		return 1;
	RISCV_INFO(r);
	if (r == NULL)
		return 1;
	return r->hart_count;
}

bool riscv_has_register(struct target *target, int hartid, int regid)
{
	return 1;
}

/**
 * This function is called when the debug user wants to change the value of a
 * register. The new value may be cached, and may not be written until the hart
 * is resumed. */
int riscv_set_register(struct target *target, enum gdb_regno r, riscv_reg_t v)
{
	return riscv_set_register_on_hart(target, riscv_current_hartid(target), r, v);
}

int riscv_set_register_on_hart(struct target *target, int hartid,
		enum gdb_regno regid, uint64_t value)
{
	RISCV_INFO(r);
	LOG_DEBUG("[%d] %s <- %" PRIx64, hartid, gdb_regno_name(regid), value);
	assert(r->set_register);
	return r->set_register(target, hartid, regid, value);
}

int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno r)
{
	return riscv_get_register_on_hart(target, value,
			riscv_current_hartid(target), r);
}

int riscv_get_register_on_hart(struct target *target, riscv_reg_t *value,
		int hartid, enum gdb_regno regid)
{
	RISCV_INFO(r);
	int result = r->get_register(target, value, hartid, regid);
	LOG_DEBUG("[%d] %s: %" PRIx64, hartid, gdb_regno_name(regid), *value);
	return result;
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
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return RISCV_HALT_ERROR;
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart is not halted!");
		return RISCV_HALT_UNKNOWN;
	}
	return r->halt_reason(target);
}

size_t riscv_debug_buffer_size(struct target *target)
{
	RISCV_INFO(r);
	return r->debug_buffer_size[riscv_current_hartid(target)];
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

	if (r->triggers_enumerated)
		return ERROR_OK;

	r->triggers_enumerated = true;	/* At the very least we tried. */

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect;
		int result = riscv_get_register_on_hart(target, &tselect, hartid,
				GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;

		for (unsigned t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
			r->trigger_count[hartid] = t;

			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, t);
			uint64_t tselect_rb;
			result = riscv_get_register_on_hart(target, &tselect_rb, hartid,
					GDB_REGNO_TSELECT);
			if (result != ERROR_OK)
				return result;
			/* Mask off the top bit, which is used as tdrmode in old
			 * implementations. */
			tselect_rb &= ~(1ULL << (riscv_xlen(target)-1));
			if (tselect_rb != t)
				break;
			uint64_t tdata1;
			result = riscv_get_register_on_hart(target, &tdata1, hartid,
					GDB_REGNO_TDATA1);
			if (result != ERROR_OK)
				return result;

			int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));
			switch (type) {
				case 1:
					/* On these older cores we don't support software using
					 * triggers. */
					riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
					break;
				case 2:
					if (tdata1 & MCONTROL_DMODE(riscv_xlen(target)))
						riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
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
			if (regno <= GDB_REGNO_XPR31)
				sprintf(buf, "x%d", regno - GDB_REGNO_ZERO);
			else if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095)
				sprintf(buf, "csr%d", regno - GDB_REGNO_CSR0);
			else if (regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31)
				sprintf(buf, "f%d", regno - GDB_REGNO_FPR0);
			else
				sprintf(buf, "gdb_regno_%d", regno);
			return buf;
	}
}

static int register_get(struct reg *reg)
{
	struct target *target = (struct target *) reg->arch_info;
	uint64_t value;
	int result = riscv_get_register(target, &value, reg->number);
	if (result != ERROR_OK)
		return result;
	buf_set_u64(reg->value, 0, reg->size, value);
	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = (struct target *) reg->arch_info;

	uint64_t value = buf_get_u64(buf, 0, reg->size);

	LOG_DEBUG("write 0x%" PRIx64 " to %s", value, reg->name);
	struct reg *r = &target->reg_cache->reg_list[reg->number];
	r->valid = true;
	memcpy(r->value, buf, (r->size + 7) / 8);

	riscv_set_register(target, reg->number, value);
	return ERROR_OK;
}

static struct reg_arch_type riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};

struct csr_info {
	unsigned number;
	const char *name;
};

static int cmp_csr_info(const void *p1, const void *p2)
{
	return (int) (((struct csr_info *)p1)->number) - (int) (((struct csr_info *)p2)->number);
}

int riscv_init_registers(struct target *target)
{
	RISCV_INFO(info);

	if (target->reg_cache) {
		if (target->reg_cache->reg_list)
			free(target->reg_cache->reg_list);
		free(target->reg_cache);
	}

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	target->reg_cache->name = "RISC-V Registers";
	target->reg_cache->num_regs = GDB_REGNO_COUNT;

	target->reg_cache->reg_list = calloc(GDB_REGNO_COUNT, sizeof(struct reg));

	const unsigned int max_reg_name_len = 12;
	if (info->reg_names)
		free(info->reg_names);
	info->reg_names = calloc(1, GDB_REGNO_COUNT * max_reg_name_len);
	char *reg_name = info->reg_names;

	static struct reg_feature feature_cpu = {
		.name = "org.gnu.gdb.riscv.cpu"
	};
	static struct reg_feature feature_fpu = {
		.name = "org.gnu.gdb.riscv.fpu"
	};
	static struct reg_feature feature_csr = {
		.name = "org.gnu.gdb.riscv.csr"
	};
	static struct reg_feature feature_virtual = {
		.name = "org.gnu.gdb.riscv.virtual"
	};

	static struct reg_data_type type_ieee_single = {
		.type = REG_TYPE_IEEE_SINGLE,
		.id = "ieee_single"
	};
	static struct reg_data_type type_ieee_double = {
		.type = REG_TYPE_IEEE_DOUBLE,
		.id = "ieee_double"
	};
	struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "encoding.h"
#undef DECLARE_CSR
	};
	/* encoding.h does not contain the registers in sorted order. */
	qsort(csr_info, DIM(csr_info), sizeof(*csr_info), cmp_csr_info);
	unsigned csr_info_index = 0;

	/* When gdb request register N, gdb_get_register_packet() assumes that this
	 * is register at index N in reg_list. So if there are certain registers
	 * that don't exist, we need to leave holes in the list (or renumber, but
	 * it would be nice not to have yet another set of numbers to translate
	 * between). */
	for (uint32_t number = 0; number < GDB_REGNO_COUNT; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];
		r->dirty = false;
		r->valid = false;
		r->exist = true;
		r->type = &riscv_reg_arch_type;
		r->arch_info = target;
		r->number = number;
		r->size = riscv_xlen(target);
		/* r->size is set in riscv_invalidate_register_cache, maybe because the
		 * target is in theory allowed to change XLEN on us. But I expect a lot
		 * of other things to break in that case as well. */
		if (number <= GDB_REGNO_XPR31) {
			r->caller_save = true;
			switch (number) {
				case GDB_REGNO_ZERO:
					r->name = "zero";
					break;
				case GDB_REGNO_RA:
					r->name = "ra";
					break;
				case GDB_REGNO_SP:
					r->name = "sp";
					break;
				case GDB_REGNO_GP:
					r->name = "gp";
					break;
				case GDB_REGNO_TP:
					r->name = "tp";
					break;
				case GDB_REGNO_T0:
					r->name = "t0";
					break;
				case GDB_REGNO_T1:
					r->name = "t1";
					break;
				case GDB_REGNO_T2:
					r->name = "t2";
					break;
				case GDB_REGNO_FP:
					r->name = "fp";
					break;
				case GDB_REGNO_S1:
					r->name = "s1";
					break;
				case GDB_REGNO_A0:
					r->name = "a0";
					break;
				case GDB_REGNO_A1:
					r->name = "a1";
					break;
				case GDB_REGNO_A2:
					r->name = "a2";
					break;
				case GDB_REGNO_A3:
					r->name = "a3";
					break;
				case GDB_REGNO_A4:
					r->name = "a4";
					break;
				case GDB_REGNO_A5:
					r->name = "a5";
					break;
				case GDB_REGNO_A6:
					r->name = "a6";
					break;
				case GDB_REGNO_A7:
					r->name = "a7";
					break;
				case GDB_REGNO_S2:
					r->name = "s2";
					break;
				case GDB_REGNO_S3:
					r->name = "s3";
					break;
				case GDB_REGNO_S4:
					r->name = "s4";
					break;
				case GDB_REGNO_S5:
					r->name = "s5";
					break;
				case GDB_REGNO_S6:
					r->name = "s6";
					break;
				case GDB_REGNO_S7:
					r->name = "s7";
					break;
				case GDB_REGNO_S8:
					r->name = "s8";
					break;
				case GDB_REGNO_S9:
					r->name = "s9";
					break;
				case GDB_REGNO_S10:
					r->name = "s10";
					break;
				case GDB_REGNO_S11:
					r->name = "s11";
					break;
				case GDB_REGNO_T3:
					r->name = "t3";
					break;
				case GDB_REGNO_T4:
					r->name = "t4";
					break;
				case GDB_REGNO_T5:
					r->name = "t5";
					break;
				case GDB_REGNO_T6:
					r->name = "t6";
					break;
			}
			r->group = "general";
			r->feature = &feature_cpu;
		} else if (number == GDB_REGNO_PC) {
			r->caller_save = true;
			sprintf(reg_name, "pc");
			r->group = "general";
			r->feature = &feature_cpu;
		} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			r->caller_save = true;
			if (riscv_supports_extension(target, riscv_current_hartid(target),
						'D')) {
				r->reg_data_type = &type_ieee_double;
				r->size = 64;
			} else if (riscv_supports_extension(target,
						riscv_current_hartid(target), 'F')) {
				r->reg_data_type = &type_ieee_single;
				r->size = 32;
			} else {
				r->exist = false;
			}
			switch (number) {
				case GDB_REGNO_FT0:
					r->name = "ft0";
					break;
				case GDB_REGNO_FT1:
					r->name = "ft1";
					break;
				case GDB_REGNO_FT2:
					r->name = "ft2";
					break;
				case GDB_REGNO_FT3:
					r->name = "ft3";
					break;
				case GDB_REGNO_FT4:
					r->name = "ft4";
					break;
				case GDB_REGNO_FT5:
					r->name = "ft5";
					break;
				case GDB_REGNO_FT6:
					r->name = "ft6";
					break;
				case GDB_REGNO_FT7:
					r->name = "ft7";
					break;
				case GDB_REGNO_FS0:
					r->name = "fs0";
					break;
				case GDB_REGNO_FS1:
					r->name = "fs1";
					break;
				case GDB_REGNO_FA0:
					r->name = "fa0";
					break;
				case GDB_REGNO_FA1:
					r->name = "fa1";
					break;
				case GDB_REGNO_FA2:
					r->name = "fa2";
					break;
				case GDB_REGNO_FA3:
					r->name = "fa3";
					break;
				case GDB_REGNO_FA4:
					r->name = "fa4";
					break;
				case GDB_REGNO_FA5:
					r->name = "fa5";
					break;
				case GDB_REGNO_FA6:
					r->name = "fa6";
					break;
				case GDB_REGNO_FA7:
					r->name = "fa7";
					break;
				case GDB_REGNO_FS2:
					r->name = "fs2";
					break;
				case GDB_REGNO_FS3:
					r->name = "fs3";
					break;
				case GDB_REGNO_FS4:
					r->name = "fs4";
					break;
				case GDB_REGNO_FS5:
					r->name = "fs5";
					break;
				case GDB_REGNO_FS6:
					r->name = "fs6";
					break;
				case GDB_REGNO_FS7:
					r->name = "fs7";
					break;
				case GDB_REGNO_FS8:
					r->name = "fs8";
					break;
				case GDB_REGNO_FS9:
					r->name = "fs9";
					break;
				case GDB_REGNO_FS10:
					r->name = "fs10";
					break;
				case GDB_REGNO_FS11:
					r->name = "fs11";
					break;
				case GDB_REGNO_FT8:
					r->name = "ft8";
					break;
				case GDB_REGNO_FT9:
					r->name = "ft9";
					break;
				case GDB_REGNO_FT10:
					r->name = "ft10";
					break;
				case GDB_REGNO_FT11:
					r->name = "ft11";
					break;
			}
			r->group = "float";
			r->feature = &feature_fpu;
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			r->group = "csr";
			r->feature = &feature_csr;
			unsigned csr_number = number - GDB_REGNO_CSR0;

			while (csr_info[csr_info_index].number < csr_number &&
					csr_info_index < DIM(csr_info) - 1) {
				csr_info_index++;
			}
			if (csr_info[csr_info_index].number == csr_number) {
				r->name = csr_info[csr_info_index].name;
			} else {
				sprintf(reg_name, "csr%d", csr_number);
				/* Assume unnamed registers don't exist, unless we have some
				 * configuration that tells us otherwise. That's important
				 * because eg. Eclipse crashes if a target has too many
				 * registers, and apparently has no way of only showing a
				 * subset of registers in any case. */
				r->exist = false;
			}

			switch (csr_number) {
				case CSR_FFLAGS:
				case CSR_FRM:
				case CSR_FCSR:
					r->exist = riscv_supports_extension(target,
							riscv_current_hartid(target), 'F');
					r->group = "float";
					r->feature = &feature_fpu;
					break;
				case CSR_SSTATUS:
				case CSR_STVEC:
				case CSR_SIP:
				case CSR_SIE:
				case CSR_SCOUNTEREN:
				case CSR_SSCRATCH:
				case CSR_SEPC:
				case CSR_SCAUSE:
				case CSR_STVAL:
				case CSR_SATP:
					r->exist = riscv_supports_extension(target,
							riscv_current_hartid(target), 'S');
					break;
				case CSR_MEDELEG:
				case CSR_MIDELEG:
					/* "In systems with only M-mode, or with both M-mode and
					 * U-mode but without U-mode trap support, the medeleg and
					 * mideleg registers should not exist." */
					r->exist = riscv_supports_extension(target, riscv_current_hartid(target), 'S') ||
						riscv_supports_extension(target, riscv_current_hartid(target), 'N');
					break;

				case CSR_CYCLEH:
				case CSR_TIMEH:
				case CSR_INSTRETH:
				case CSR_HPMCOUNTER3H:
				case CSR_HPMCOUNTER4H:
				case CSR_HPMCOUNTER5H:
				case CSR_HPMCOUNTER6H:
				case CSR_HPMCOUNTER7H:
				case CSR_HPMCOUNTER8H:
				case CSR_HPMCOUNTER9H:
				case CSR_HPMCOUNTER10H:
				case CSR_HPMCOUNTER11H:
				case CSR_HPMCOUNTER12H:
				case CSR_HPMCOUNTER13H:
				case CSR_HPMCOUNTER14H:
				case CSR_HPMCOUNTER15H:
				case CSR_HPMCOUNTER16H:
				case CSR_HPMCOUNTER17H:
				case CSR_HPMCOUNTER18H:
				case CSR_HPMCOUNTER19H:
				case CSR_HPMCOUNTER20H:
				case CSR_HPMCOUNTER21H:
				case CSR_HPMCOUNTER22H:
				case CSR_HPMCOUNTER23H:
				case CSR_HPMCOUNTER24H:
				case CSR_HPMCOUNTER25H:
				case CSR_HPMCOUNTER26H:
				case CSR_HPMCOUNTER27H:
				case CSR_HPMCOUNTER28H:
				case CSR_HPMCOUNTER29H:
				case CSR_HPMCOUNTER30H:
				case CSR_HPMCOUNTER31H:
				case CSR_MCYCLEH:
				case CSR_MINSTRETH:
				case CSR_MHPMCOUNTER3H:
				case CSR_MHPMCOUNTER4H:
				case CSR_MHPMCOUNTER5H:
				case CSR_MHPMCOUNTER6H:
				case CSR_MHPMCOUNTER7H:
				case CSR_MHPMCOUNTER8H:
				case CSR_MHPMCOUNTER9H:
				case CSR_MHPMCOUNTER10H:
				case CSR_MHPMCOUNTER11H:
				case CSR_MHPMCOUNTER12H:
				case CSR_MHPMCOUNTER13H:
				case CSR_MHPMCOUNTER14H:
				case CSR_MHPMCOUNTER15H:
				case CSR_MHPMCOUNTER16H:
				case CSR_MHPMCOUNTER17H:
				case CSR_MHPMCOUNTER18H:
				case CSR_MHPMCOUNTER19H:
				case CSR_MHPMCOUNTER20H:
				case CSR_MHPMCOUNTER21H:
				case CSR_MHPMCOUNTER22H:
				case CSR_MHPMCOUNTER23H:
				case CSR_MHPMCOUNTER24H:
				case CSR_MHPMCOUNTER25H:
				case CSR_MHPMCOUNTER26H:
				case CSR_MHPMCOUNTER27H:
				case CSR_MHPMCOUNTER28H:
				case CSR_MHPMCOUNTER29H:
				case CSR_MHPMCOUNTER30H:
				case CSR_MHPMCOUNTER31H:
					r->exist = riscv_xlen(target) == 32;
					break;
			}

			if (!r->exist && expose_csr) {
				for (unsigned i = 0; expose_csr[i].low <= expose_csr[i].high; i++) {
					if (csr_number >= expose_csr[i].low && csr_number <= expose_csr[i].high) {
						LOG_INFO("Exposing additional CSR %d", csr_number);
						r->exist = true;
						break;
					}
				}
			}

		} else if (number == GDB_REGNO_PRIV) {
			sprintf(reg_name, "priv");
			r->group = "general";
			r->feature = &feature_virtual;
			r->size = 8;
		}
		if (reg_name[0])
			r->name = reg_name;
		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + GDB_REGNO_COUNT * max_reg_name_len);
		r->value = &info->reg_cache_values[number];
	}

	return ERROR_OK;
}

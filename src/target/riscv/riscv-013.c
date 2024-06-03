// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Support for RISC-V, debug version 0.13, which is currently (2/4/17) the
 * latest draft.
 */

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include <helper/log.h>
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/time_support.h"
#include "helper/list.h"
#include "riscv.h"
#include "riscv-013.h"
#include "riscv_reg.h"
#include "riscv-013_reg.h"
#include "debug_defines.h"
#include "rtos/rtos.h"
#include "program.h"
#include "asm.h"
#include "batch.h"
#include "debug_reg_printer.h"
#include "field_helpers.h"

static int riscv013_on_step_or_resume(struct target *target, bool step);
static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step);
static int riscv013_clear_abstract_error(struct target *target);

/* Implementations of the functions in struct riscv_info. */
static int dm013_select_hart(struct target *target, int hart_index);
static int riscv013_halt_prep(struct target *target);
static int riscv013_halt_go(struct target *target);
static int riscv013_resume_go(struct target *target);
static int riscv013_step_current_hart(struct target *target);
static int riscv013_on_step(struct target *target);
static int riscv013_resume_prep(struct target *target);
static enum riscv_halt_reason riscv013_halt_reason(struct target *target);
static int riscv013_write_progbuf(struct target *target, unsigned int index,
		riscv_insn_t d);
static riscv_insn_t riscv013_read_progbuf(struct target *target, unsigned int
		index);
static int riscv013_invalidate_cached_progbuf(struct target *target);
static int riscv013_execute_progbuf(struct target *target, uint32_t *cmderr);
static void riscv013_fill_dmi_write(struct target *target, char *buf, uint64_t a, uint32_t d);
static void riscv013_fill_dmi_read(struct target *target, char *buf, uint64_t a);
static int riscv013_get_dmi_scan_length(struct target *target);
static void riscv013_fill_dm_nop(struct target *target, char *buf);
static unsigned int register_size(struct target *target, enum gdb_regno number);
static int register_read_direct(struct target *target, riscv_reg_t *value,
		enum gdb_regno number);
static int register_write_direct(struct target *target, enum gdb_regno number,
		riscv_reg_t value);
static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment);
static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);

typedef enum {
	HALT_GROUP,
	RESUME_GROUP
} grouptype_t;
static int set_group(struct target *target, bool *supported, unsigned int group,
		grouptype_t grouptype);

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

#define RISCV013_INFO(r) riscv013_info_t *r = get_info(target)

/*** JTAG registers. ***/

typedef enum {
	DMI_OP_NOP = DTM_DMI_OP_NOP,
	DMI_OP_READ = DTM_DMI_OP_READ,
	DMI_OP_WRITE = DTM_DMI_OP_WRITE
} dmi_op_t;
typedef enum {
	DMI_STATUS_SUCCESS = DTM_DMI_OP_SUCCESS,
	DMI_STATUS_FAILED = DTM_DMI_OP_FAILED,
	DMI_STATUS_BUSY = DTM_DMI_OP_BUSY
} dmi_status_t;

/*** Debug Bus registers. ***/

/* TODO: CMDERR_* defines can removed */
#define CMDERR_NONE			DM_ABSTRACTCS_CMDERR_NONE
#define CMDERR_BUSY			DM_ABSTRACTCS_CMDERR_BUSY
#define CMDERR_NOT_SUPPORTED		DM_ABSTRACTCS_CMDERR_NOT_SUPPORTED
#define CMDERR_EXCEPTION		DM_ABSTRACTCS_CMDERR_EXCEPTION
#define CMDERR_HALT_RESUME		DM_ABSTRACTCS_CMDERR_HALT_RESUME
#define CMDERR_OTHER			DM_ABSTRACTCS_CMDERR_OTHER

#define HART_INDEX_MULTIPLE	-1
#define HART_INDEX_UNKNOWN	-2

typedef struct {
	struct list_head list;
	int abs_chain_position;
	/* The base address to access this DM on DMI */
	uint32_t base;
	/* The number of harts connected to this DM. */
	int hart_count;
	/* Indicates we already examined this DM, so don't need to do it again. */
	bool was_examined;
	/* Indicates we already reset this DM, so don't need to do it again. */
	bool was_reset;
	/* Targets that are connected to this DM. */
	struct list_head target_list;
	/* Contains the ID of the hart that is currently selected by this DM.
	 * If multiple harts are selected this is HART_INDEX_MULTIPLE. */
	int current_hartid;

	bool hasel_supported;

	/* The program buffer stores executable code. 0 is an illegal instruction,
	 * so we use 0 to mean the cached value is invalid. */
	uint32_t progbuf_cache[16];

	/* Some operations are illegal when an abstract command is running.
	 * The field is used to track whether the last command timed out, and
	 * abstractcs.busy may have remained set. In that case we may need to
	 * re-check the busy state before executing these operations. */
	bool abstract_cmd_maybe_busy;
} dm013_info_t;

typedef struct {
	struct list_head list;
	struct target *target;
} target_list_t;

typedef struct {
	/* The indexed used to address this hart in its DM. */
	unsigned index;
	/* Number of address bits in the dbus register. */
	unsigned abits;
	/* Number of abstract command data registers. */
	unsigned datacount;
	/* Number of words in the Program Buffer. */
	unsigned progbufsize;

	/* We cache the read-only bits of sbcs here. */
	uint32_t sbcs;

	yes_no_maybe_t progbuf_writable;
	/* We only need the address so that we know the alignment of the buffer. */
	riscv_addr_t progbuf_address;

	/* Number of run-test/idle cycles the target requests we do after each dbus
	 * access. */
	unsigned int dtmcs_idle;

	/* This structure is used to determine how many run-test/idle to use after
	 * an access of corresponding "riscv_scan_delay_class".
	 * Values are incremented every time an access results in a busy
	 * response.
	 */
	struct riscv_scan_delays learned_delays;

	bool abstract_read_csr_supported;
	bool abstract_write_csr_supported;
	bool abstract_read_fpr_supported;
	bool abstract_write_fpr_supported;

	yes_no_maybe_t has_aampostincrement;

	/* Some fields from hartinfo. */
	uint8_t datasize;
	uint8_t dataaccess;
	int16_t dataaddr;

	/* DM that provides access to this target. */
	dm013_info_t *dm;

	/* This target was selected using hasel. */
	bool selected;

	/* When false, we need to set dcsr.ebreak*, halting the target if that's
	 * necessary. */
	bool dcsr_ebreak_is_set;

	/* This hart was placed into a halt group in examine(). */
	bool haltgroup_supported;
} riscv013_info_t;

static LIST_HEAD(dm_list);

static riscv013_info_t *get_info(const struct target *target)
{
	struct riscv_info *info = target->arch_info;
	assert(info);
	assert(info->version_specific);
	return info->version_specific;
}

/**
 * Return the DM structure for this target. If there isn't one, find it in the
 * global list of DMs. If it's not in there, then create one and initialize it
 * to 0.
 */
static dm013_info_t *get_dm(struct target *target)
{
	RISCV013_INFO(info);
	if (info->dm)
		return info->dm;

	int abs_chain_position = target->tap->abs_chain_position;

	dm013_info_t *entry;
	dm013_info_t *dm = NULL;
	list_for_each_entry(entry, &dm_list, list) {
		if (entry->abs_chain_position == abs_chain_position
				&& entry->base == target->dbgbase) {
			dm = entry;
			break;
		}
	}

	if (!dm) {
		LOG_TARGET_DEBUG(target, "Coreid [%d] Allocating new DM", target->coreid);
		dm = calloc(1, sizeof(dm013_info_t));
		if (!dm)
			return NULL;
		dm->abs_chain_position = abs_chain_position;

		/* Safety check for dbgbase */
		assert(target->dbgbase_set || target->dbgbase == 0);

		dm->base = target->dbgbase;
		dm->current_hartid = 0;
		dm->hart_count = -1;
		INIT_LIST_HEAD(&dm->target_list);
		list_add(&dm->list, &dm_list);
	}

	info->dm = dm;
	target_list_t *target_entry;
	list_for_each_entry(target_entry, &dm->target_list, list) {
		if (target_entry->target == target)
			return dm;
	}
	target_entry = calloc(1, sizeof(*target_entry));
	if (!target_entry) {
		info->dm = NULL;
		return NULL;
	}
	target_entry->target = target;
	list_add(&target_entry->list, &dm->target_list);

	return dm;
}

static void riscv013_dm_free(struct target *target)
{
	RISCV013_INFO(info);
	dm013_info_t *dm = info->dm;
	if (!dm)
		return;

	target_list_t *target_entry;
	list_for_each_entry(target_entry, &dm->target_list, list) {
		if (target_entry->target == target) {
			list_del(&target_entry->list);
			free(target_entry);
			break;
		}
	}

	if (list_empty(&dm->target_list)) {
		list_del(&dm->list);
		free(dm);
	}
	info->dm = NULL;
}

static riscv_debug_reg_ctx_t get_riscv_debug_reg_ctx(const struct target *target)
{
	if (!target_was_examined(target)) {
		const riscv_debug_reg_ctx_t default_context = {0};
		return default_context;
	}

	RISCV013_INFO(info);
	const riscv_debug_reg_ctx_t context = {
		.XLEN = { .value = riscv_xlen(target), .is_set = true },
		.DXLEN = { .value = riscv_xlen(target), .is_set = true },
		.abits = { .value = info->abits, .is_set = true },
	};
	return context;
}

static void log_debug_reg(struct target *target, enum riscv_debug_reg_ordinal reg,
		riscv_reg_t value, const char *file, unsigned int line, const char *func)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;
	const riscv_debug_reg_ctx_t context = get_riscv_debug_reg_ctx(target);
	char * const buf = malloc(riscv_debug_reg_to_s(NULL, reg, context, value, RISCV_DEBUG_REG_HIDE_UNNAMED_0) + 1);
	if (!buf) {
		LOG_ERROR("Unable to allocate memory.");
		return;
	}
	riscv_debug_reg_to_s(buf, reg, context, value, RISCV_DEBUG_REG_HIDE_UNNAMED_0);
	log_printf_lf(LOG_LVL_DEBUG, file, line, func, "[%s] %s", target_name(target), buf);
	free(buf);
}

#define LOG_DEBUG_REG(t, r, v) log_debug_reg(t, r##_ORDINAL, v, __FILE__, __LINE__, __func__)

static uint32_t set_dmcontrol_hartsel(uint32_t initial, int hart_index)
{
	assert(hart_index != HART_INDEX_UNKNOWN);

	if (hart_index >= 0) {
		initial = set_field(initial, DM_DMCONTROL_HASEL, DM_DMCONTROL_HASEL_SINGLE);
		uint32_t index_lo = hart_index & ((1 << DM_DMCONTROL_HARTSELLO_LENGTH) - 1);
		initial = set_field(initial, DM_DMCONTROL_HARTSELLO, index_lo);
		uint32_t index_hi = hart_index >> DM_DMCONTROL_HARTSELLO_LENGTH;
		assert(index_hi < (1 << DM_DMCONTROL_HARTSELHI_LENGTH));
		initial = set_field(initial, DM_DMCONTROL_HARTSELHI, index_hi);
	} else if (hart_index == HART_INDEX_MULTIPLE) {
		initial = set_field(initial, DM_DMCONTROL_HASEL, DM_DMCONTROL_HASEL_MULTIPLE);
		/* TODO: https://github.com/riscv/riscv-openocd/issues/748 */
		initial = set_field(initial, DM_DMCONTROL_HARTSELLO, 0);
		initial = set_field(initial, DM_DMCONTROL_HARTSELHI, 0);
	}

	return initial;
}

static unsigned int decode_dmi(const struct target *target, char *text, uint32_t address, uint32_t data)
{
	static const struct {
		uint32_t address;
		enum riscv_debug_reg_ordinal ordinal;
	} description[] = {
		{DM_DMCONTROL, DM_DMCONTROL_ORDINAL},
		{DM_DMSTATUS, DM_DMSTATUS_ORDINAL},
		{DM_ABSTRACTCS, DM_ABSTRACTCS_ORDINAL},
		{DM_COMMAND, DM_COMMAND_ORDINAL},
		{DM_SBCS, DM_SBCS_ORDINAL}
	};

	for (unsigned i = 0; i < ARRAY_SIZE(description); i++) {
		if (riscv_get_dmi_address(target, description[i].address) == address) {
			const riscv_debug_reg_ctx_t context = {
				.XLEN = { .value = 0, .is_set = false },
				.DXLEN = { .value = 0, .is_set = false },
				.abits = { .value = 0, .is_set = false },
			};
			return riscv_debug_reg_to_s(text, description[i].ordinal,
					context, data, RISCV_DEBUG_REG_HIDE_ALL_0);
		}
	}
	if (text)
		text[0] = '\0';
	return 0;
}

/* TODO: Move this function to "batch.c" and make it static. */
void riscv_log_dmi_scan(const struct target *target, int idle,
		const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field->out_value);
	const uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	const unsigned int out_op = get_field(out, DTM_DMI_OP);
	const uint32_t out_data = get_field(out, DTM_DMI_DATA);
	const uint32_t out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	if (field->in_value) {
		const uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
		const unsigned int in_op = get_field(in, DTM_DMI_OP);
		const uint32_t in_data = get_field(in, DTM_DMI_DATA);
		const uint32_t in_address = in >> DTM_DMI_ADDRESS_OFFSET;

		LOG_DEBUG("%db %s %08" PRIx32 " @%02" PRIx32 " -> %s %08" PRIx32 " @%02" PRIx32 "; %di",
				field->num_bits, op_string[out_op], out_data, out_address,
				status_string[in_op], in_data, in_address, idle);

		if (in_op == DTM_DMI_OP_SUCCESS) {
			char in_decoded[decode_dmi(target, NULL, in_address, in_data) + 1];
			decode_dmi(target, in_decoded, in_address, in_data);
			/* FIXME: The current code assumes that the hardware
			 * provides the read address in the dmi.address field
			 * when returning the dmi.data. That is however not
			 * required by the spec, and therefore not guaranteed.
			 * See https://github.com/riscv-collab/riscv-openocd/issues/1043
			 */
			LOG_DEBUG("read: %s", in_decoded);
		}
	} else {
		LOG_DEBUG("%db %s %08" PRIx32 " @%02" PRIx32 " -> ?; %di",
				field->num_bits, op_string[out_op], out_data, out_address,
				idle);
	}
	if (out_op == DTM_DMI_OP_WRITE) {
		char out_decoded[decode_dmi(target, NULL, out_address, out_data) + 1];
		decode_dmi(target, out_decoded, out_address, out_data);
		LOG_DEBUG("write: %s", out_decoded);
	}
}

/*** Utility functions. ***/

static void select_dmi(struct target *target)
{
	if (bscan_tunnel_ir_width != 0) {
		select_dmi_via_bscan(target);
		return;
	}
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
}

static int dtmcontrol_scan(struct target *target, uint32_t out, uint32_t *in_ptr)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4] = { 0 };

	if (bscan_tunnel_ir_width != 0)
		return dtmcontrol_scan_via_bscan(target, out, in_ptr);

	buf_set_u32(out_value, 0, 32, out);

	jtag_add_ir_scan(target->tap, &select_dtmcontrol, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = out_value;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dmi. */
	select_dmi(target);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCS: 0x%x -> 0x%x", out, in);

	if (in_ptr)
		*in_ptr = in;
	return ERROR_OK;
}

static int increase_dmi_busy_delay(struct target *target)
{
	RISCV013_INFO(info);

	int res = dtmcontrol_scan(target, DTM_DTMCS_DMIRESET,
			NULL /* discard result */);
	if (res != ERROR_OK)
		return res;

	res = riscv_scan_increase_delay(&info->learned_delays,
			RISCV_DELAY_BASE);
	return res;
}

static void reset_learned_delays(struct target *target)
{
	RISCV013_INFO(info);
	assert(info);
	memset(&info->learned_delays, 0, sizeof(info->learned_delays));
}

static void decrement_reset_delays_counter(struct target *target, size_t finished_scans)
{
	RISCV_INFO(r);
	if (r->reset_delays_wait < 0) {
		assert(r->reset_delays_wait == -1);
		return;
	}
	if ((size_t)r->reset_delays_wait >= finished_scans) {
		r->reset_delays_wait -= finished_scans;
		return;
	}
	r->reset_delays_wait = -1;
	LOG_TARGET_DEBUG(target,
			"resetting learned delays (reset_delays_wait counter expired)");
	reset_learned_delays(target);
}

static uint32_t riscv013_get_dmi_address(const struct target *target, uint32_t address)
{
	assert(target);
	uint32_t base = 0;
	RISCV013_INFO(info);
	if (info && info->dm)
		base = info->dm->base;
	return address + base;
}

static int batch_run_timeout(struct target *target, struct riscv_batch *batch);

static int dmi_read(struct target *target, uint32_t *value, uint32_t address)
{
	struct riscv_batch *batch = riscv_batch_alloc(target, 1);
	riscv_batch_add_dmi_read(batch, address, RISCV_DELAY_BASE);
	int res = batch_run_timeout(target, batch);
	if (res == ERROR_OK && value)
		*value = riscv_batch_get_dmi_read_data(batch, 0);
	riscv_batch_free(batch);
	return res;
}

static int dm_read(struct target *target, uint32_t *value, uint32_t address)
{
	return dmi_read(target, value, riscv013_get_dmi_address(target, address));
}

static int dm_read_exec(struct target *target, uint32_t *value, uint32_t address)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	struct riscv_batch *batch = riscv_batch_alloc(target, 1);
	riscv_batch_add_dm_read(batch, address, RISCV_DELAY_ABSTRACT_COMMAND);
	dm->abstract_cmd_maybe_busy = true;
	int res = batch_run_timeout(target, batch);
	if (res == ERROR_OK && value)
		*value = riscv_batch_get_dmi_read_data(batch, 0);
	riscv_batch_free(batch);
	return res;
}

static int dmi_write(struct target *target, uint32_t address, uint32_t value)
{
	struct riscv_batch *batch = riscv_batch_alloc(target, 1);
	riscv_batch_add_dmi_write(batch, address, value, /*read_back*/ true,
			RISCV_DELAY_BASE);
	int res = batch_run_timeout(target, batch);
	riscv_batch_free(batch);
	return res;
}

static int dm_write(struct target *target, uint32_t address, uint32_t value)
{
	return dmi_write(target, riscv013_get_dmi_address(target, address), value);
}

static bool check_dbgbase_exists(struct target *target)
{
	uint32_t next_dm = 0;
	unsigned int count = 1;

	LOG_TARGET_DEBUG(target, "Searching for DM with DMI base address (dbgbase) = 0x%x", target->dbgbase);
	while (1) {
		uint32_t current_dm = next_dm;
		if (current_dm == target->dbgbase)
			return true;
		if (dmi_read(target, &next_dm, DM_NEXTDM + current_dm) != ERROR_OK)
			break;
		LOG_TARGET_DEBUG(target, "dm @ 0x%x --> nextdm=0x%x", current_dm, next_dm);
		/* Check if it's last one in the chain. */
		if (next_dm == 0) {
			LOG_TARGET_ERROR(target, "Reached the end of DM chain (detected %u DMs in total).", count);
			break;
		}
		/* Safety: Avoid looping forever in case of buggy nextdm values in the hardware. */
		if (count++ > RISCV_MAX_DMS) {
			LOG_TARGET_ERROR(target, "Supporting no more than %d DMs on a DMI bus. Aborting", RISCV_MAX_DMS);
			break;
		}
	}
	return false;
}

static int dmstatus_read(struct target *target, uint32_t *dmstatus,
		bool authenticated)
{
	int result = dm_read(target, dmstatus, DM_DMSTATUS);
	if (result != ERROR_OK)
		return result;
	int dmstatus_version = get_field(*dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2 (0.13) and 3 (1.0), not "
				"%" PRId32 " (dmstatus=0x%" PRIx32 "). This error might be caused by a JTAG "
				"signal issue. Try reducing the JTAG clock speed.",
				get_field32(*dmstatus, DM_DMSTATUS_VERSION), *dmstatus);
	} else if (authenticated && !get_field(*dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", *dmstatus);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int increase_ac_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	return riscv_scan_increase_delay(&info->learned_delays,
			RISCV_DELAY_ABSTRACT_COMMAND);
}

static uint32_t __attribute__((unused)) abstract_register_size(unsigned width)
{
	switch (width) {
		case 32:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 2);
		case 64:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 3);
		case 128:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 4);
		default:
			LOG_ERROR("Unsupported register width: %d", width);
			return 0;
	}
}

static int wait_for_idle(struct target *target, uint32_t *abstractcs)
{
	assert(target);
	assert(abstractcs);

	dm013_info_t *dm = get_dm(target);
	if (!dm) {
		LOG_ERROR("BUG: Target %s is not assigned to any RISC-V debug module",
				target_name(target));
		*abstractcs = 0;
		return ERROR_FAIL;
	}

	time_t start = time(NULL);
	do {
		if (dm_read(target, abstractcs, DM_ABSTRACTCS) != ERROR_OK) {
			/* We couldn't read abstractcs. For safety, overwrite the output value to
			 * prevent the caller working with a stale value of abstractcs. */
			*abstractcs = 0;
			LOG_TARGET_ERROR(target,
				"potentially unrecoverable error detected - could not read abstractcs");
			return ERROR_FAIL;
		}

		if (get_field(*abstractcs, DM_ABSTRACTCS_BUSY) == 0) {
			dm->abstract_cmd_maybe_busy = false;
			return ERROR_OK;
		}
	} while ((time(NULL) - start) < riscv_get_command_timeout_sec());

	LOG_TARGET_ERROR(target,
		"Timed out after %ds waiting for busy to go low (abstractcs=0x%" PRIx32 "). "
		"Increase the timeout with riscv set_command_timeout_sec.",
		riscv_get_command_timeout_sec(),
		*abstractcs);

	if (!dm->abstract_cmd_maybe_busy)
		LOG_TARGET_ERROR(target,
				"BUG: dm->abstract_cmd_maybe_busy had not been set when starting an abstract command.");
	dm->abstract_cmd_maybe_busy = true;

	return ERROR_TIMEOUT_REACHED;
}

static int dm013_select_target(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	return dm013_select_hart(target, info->index);
}

#define ABSTRACT_COMMAND_BATCH_SIZE 2

static size_t abstract_cmd_fill_batch(struct riscv_batch *batch,
		uint32_t command)
{
	assert(riscv_batch_available_scans(batch)
			>= ABSTRACT_COMMAND_BATCH_SIZE);
	riscv_batch_add_dm_write(batch, DM_COMMAND, command, /* read_back */ true,
			RISCV_DELAY_ABSTRACT_COMMAND);
	return riscv_batch_add_dm_read(batch, DM_ABSTRACTCS, RISCV_DELAY_BASE);
}

static int abstract_cmd_batch_check_and_clear_cmderr(struct target *target,
		const struct riscv_batch *batch, size_t abstractcs_read_key,
		uint32_t *cmderr)
{
	uint32_t abstractcs = riscv_batch_get_dmi_read_data(batch,
			abstractcs_read_key);
	int res;
	LOG_DEBUG_REG(target, DM_ABSTRACTCS, abstractcs);
	if (get_field32(abstractcs, DM_ABSTRACTCS_BUSY) != 0) {
		res = wait_for_idle(target, &abstractcs);
		if (res != ERROR_OK)
			goto clear_cmderr;
		res = increase_ac_busy_delay(target);
		if (res != ERROR_OK)
			goto clear_cmderr;
	}
	*cmderr = get_field32(abstractcs, DM_ABSTRACTCS_CMDERR);
	if (*cmderr == CMDERR_NONE)
		return ERROR_OK;
	res = ERROR_FAIL;
	LOG_TARGET_DEBUG(target,
			"Abstract Command execution failed (abstractcs.cmderr = %" PRIx32 ").",
			*cmderr);
clear_cmderr:
	/* Attempt to clear the error. */
	/* TODO: can we add a more substantial recovery if the clear operation fails? */
	if (dm_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR) != ERROR_OK)
		LOG_TARGET_ERROR(target, "could not clear abstractcs error");
	return res;
}

static int execute_abstract_command(struct target *target, uint32_t command,
		uint32_t *cmderr)
{
	assert(cmderr);
	*cmderr = CMDERR_NONE;
	if (debug_level >= LOG_LVL_DEBUG) {
		switch (get_field(command, DM_COMMAND_CMDTYPE)) {
			case 0:
				LOG_DEBUG_REG(target, AC_ACCESS_REGISTER, command);
				break;
			default:
				LOG_TARGET_DEBUG(target, "command=0x%x", command);
				break;
		}
	}

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	struct riscv_batch *batch = riscv_batch_alloc(target,
			ABSTRACT_COMMAND_BATCH_SIZE);
	const size_t abstractcs_read_key = abstract_cmd_fill_batch(batch, command);

	/* Abstract commands are executed while running the batch. */
	dm->abstract_cmd_maybe_busy = true;

	int res = batch_run_timeout(target, batch);
	if (res != ERROR_OK)
		goto cleanup;

	res = abstract_cmd_batch_check_and_clear_cmderr(target, batch,
			abstractcs_read_key, cmderr);
cleanup:
	riscv_batch_free(batch);
	return res;
}

/**
 * Queue scans into a batch that read the value from abstract data registers:
 * data[index] (and data[index+1] in case of 64-bit value).
 *
 * No extra DTM delay is added after the write to data[N]. It is assumed that
 * this is a one-shot abstract command, that means no auto-execution is set up
 * (abstractauto.autoexecdata bits are zero).
 */
static void abstract_data_read_fill_batch(struct riscv_batch *batch, unsigned int index,
		unsigned int size_bits)
{
	assert(size_bits >= 32);
	assert(size_bits % 32 == 0);
	const unsigned int size_in_words = size_bits / 32;
	const unsigned int offset = index * size_in_words;
	for (unsigned int i = 0; i < size_in_words; ++i) {
		const unsigned int reg_address = DM_DATA0 + offset + i;
		riscv_batch_add_dm_read(batch, reg_address, RISCV_DELAY_BASE);
	}
}

static riscv_reg_t abstract_data_get_from_batch(struct riscv_batch *batch,
		unsigned int index, unsigned int size_bits)
{
	assert(size_bits >= 32);
	assert(size_bits % 32 == 0);
	const unsigned int size_in_words = size_bits / 32;
	assert(size_in_words * sizeof(uint32_t) <= sizeof(riscv_reg_t));
	riscv_reg_t value = 0;
	for (unsigned int i = 0; i < size_in_words; ++i) {
		const uint32_t v = riscv_batch_get_dmi_read_data(batch, i);
		value |= ((riscv_reg_t)v) << (i * 32);
	}
	return value;
}

static int read_abstract_arg(struct target *target, riscv_reg_t *value,
		unsigned int index, unsigned int size_bits)
{
	assert(value);
	assert(size_bits >= 32);
	assert(size_bits % 32 == 0);
	const unsigned char size_in_words = size_bits / 32;
	struct riscv_batch * const batch = riscv_batch_alloc(target, size_in_words);
	abstract_data_read_fill_batch(batch, index, size_bits);
	int result = batch_run_timeout(target, batch);
	if (result == ERROR_OK)
		*value = abstract_data_get_from_batch(batch, index, size_bits);
	riscv_batch_free(batch);
	return result;
}

/**
 * Queue scans into a batch that write the value to abstract data registers:
 * data[index] (and data[index+1] in case of 64-bit value).
 *
 * No extra DTM delay is added after the write to data[N]. It is assumed that
 * this is a one-shot abstract command, that means no auto-execution is set up
 * (abstractauto.autoexecdata bits are zero).
 */
static void abstract_data_write_fill_batch(struct riscv_batch *batch,
		riscv_reg_t value, unsigned int index, unsigned int size_bits)
{
	assert(size_bits % 32 == 0);
	const unsigned int size_in_words = size_bits / 32;
	assert(value <= UINT32_MAX || size_in_words > 1);
	const unsigned int offset = index * size_in_words;

	for (unsigned int i = 0; i < size_in_words; ++i) {
		const unsigned int reg_address = DM_DATA0 + offset + i;

		riscv_batch_add_dm_write(batch, reg_address, (uint32_t)value,
				/* read_back */ true, RISCV_DELAY_BASE);
		value >>= 32;
	}
}

/* TODO: reuse "abstract_data_write_fill_batch()" here*/
static int write_abstract_arg(struct target *target, unsigned index,
		riscv_reg_t value, unsigned size_bits)
{
	unsigned offset = index * size_bits / 32;
	switch (size_bits) {
		default:
			LOG_TARGET_ERROR(target, "Unsupported size: %d bits", size_bits);
			return ERROR_FAIL;
		case 64:
			dm_write(target, DM_DATA0 + offset + 1, (uint32_t)(value >> 32));
			/* falls through */
		case 32:
			dm_write(target, DM_DATA0 + offset, (uint32_t)value);
	}
	return ERROR_OK;
}

/**
 * @par size in bits
 */
static uint32_t access_register_command(struct target *target, uint32_t number,
		unsigned size, uint32_t flags)
{
	uint32_t command = set_field(0, DM_COMMAND_CMDTYPE, 0);
	switch (size) {
		case 32:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 2);
			break;
		case 64:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 3);
			break;
		default:
			LOG_TARGET_ERROR(target, "%d-bit register %s not supported.",
					size, riscv_reg_gdb_regno_name(target, number));
			assert(0);
	}

	if (number <= GDB_REGNO_XPR31) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0x1000 + number - GDB_REGNO_ZERO);
	} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0x1020 + number - GDB_REGNO_FPR0);
	} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				number - GDB_REGNO_CSR0);
	} else if (number >= GDB_REGNO_COUNT) {
		/* Custom register. */
		assert(target->reg_cache->reg_list[number].arch_info);
		riscv_reg_info_t *reg_info = target->reg_cache->reg_list[number].arch_info;
		assert(reg_info);
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0xc000 + reg_info->custom_number);
	} else {
		assert(0);
	}

	command |= flags;

	return command;
}

static int register_read_abstract_with_size(struct target *target,
		riscv_reg_t *value, enum gdb_regno number, unsigned int size)
{
	RISCV013_INFO(info);

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_read_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_read_csr_supported)
		return ERROR_FAIL;
	/* The spec doesn't define abstract register numbers for vector registers. */
	if (number >= GDB_REGNO_V0 && number <= GDB_REGNO_V31)
		return ERROR_FAIL;

	uint32_t command = access_register_command(target, number, size,
			AC_ACCESS_REGISTER_TRANSFER);

	uint32_t cmderr;
	int result = execute_abstract_command(target, command, &cmderr);
	if (result != ERROR_OK) {
		if (cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_read_fpr_supported = false;
				LOG_TARGET_INFO(target, "Disabling abstract command reads from FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_read_csr_supported = false;
				LOG_TARGET_INFO(target, "Disabling abstract command reads from CSRs.");
			}
		}
		return result;
	}

	if (value)
		return read_abstract_arg(target, value, 0, size);

	return ERROR_OK;
}

static int register_read_abstract(struct target *target, riscv_reg_t *value,
		enum gdb_regno number)
{
	const unsigned int size = register_size(target, number);

	return register_read_abstract_with_size(target, value, number, size);
}

static int register_write_abstract(struct target *target, enum gdb_regno number,
		riscv_reg_t value)
{
	RISCV013_INFO(info);

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_write_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_write_csr_supported)
		return ERROR_FAIL;

	const unsigned int size_bits = register_size(target, number);
	const uint32_t command = access_register_command(target, number, size_bits,
			AC_ACCESS_REGISTER_TRANSFER |
			AC_ACCESS_REGISTER_WRITE);
	LOG_DEBUG_REG(target, AC_ACCESS_REGISTER, command);
	assert(size_bits % 32 == 0);
	const unsigned int size_in_words = size_bits / 32;
	const unsigned int batch_size = size_in_words
		+ ABSTRACT_COMMAND_BATCH_SIZE;
	struct riscv_batch * const batch = riscv_batch_alloc(target, batch_size);

	abstract_data_write_fill_batch(batch, value, /*index*/ 0, size_bits);
	const size_t abstractcs_read_key = abstract_cmd_fill_batch(batch, command);
	/* Abstract commands are executed while running the batch. */
	dm->abstract_cmd_maybe_busy = true;

	int res = batch_run_timeout(target, batch);
	if (res != ERROR_OK)
		goto cleanup;

	uint32_t cmderr;
	res = abstract_cmd_batch_check_and_clear_cmderr(target, batch,
			abstractcs_read_key, &cmderr);

	if (res != ERROR_OK) {
		if (cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_write_fpr_supported = false;
				LOG_TARGET_INFO(target, "Disabling abstract command writes to FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_write_csr_supported = false;
				LOG_TARGET_INFO(target, "Disabling abstract command writes to CSRs.");
			}
		}
	}
cleanup:
	riscv_batch_free(batch);
	return res;
}

/*
 * Sets the AAMSIZE field of a memory access abstract command based on
 * the width (bits).
 */
static uint32_t abstract_memory_size(unsigned width)
{
	switch (width) {
		case 8:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 0);
		case 16:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 1);
		case 32:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 2);
		case 64:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 3);
		case 128:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 4);
		default:
			LOG_ERROR("Unsupported memory width: %d", width);
			return 0;
	}
}

/*
 * Creates a memory access abstract command.
 */
static uint32_t access_memory_command(struct target *target, bool virtual,
		unsigned int width, bool postincrement, bool is_write)
{
	uint32_t command = set_field(0, AC_ACCESS_MEMORY_CMDTYPE, 2);
	command = set_field(command, AC_ACCESS_MEMORY_AAMVIRTUAL, virtual);
	command |= abstract_memory_size(width);
	command = set_field(command, AC_ACCESS_MEMORY_AAMPOSTINCREMENT,
						postincrement);
	command = set_field(command, AC_ACCESS_MEMORY_WRITE, is_write);

	return command;
}

static int examine_progbuf(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	if (info->progbuf_writable != YNM_MAYBE)
		return ERROR_OK;

	/* Figure out if progbuf is writable. */

	if (info->progbufsize < 1) {
		info->progbuf_writable = YNM_NO;
		LOG_TARGET_INFO(target, "No program buffer present.");
		return ERROR_OK;
	}

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, auipc(S0));
	if (riscv_program_exec(&program, target) != ERROR_OK)
		return ERROR_FAIL;

	if (register_read_direct(target, &info->progbuf_address, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	riscv_program_init(&program, target);
	riscv_program_insert(&program, sw(S0, S0, 0));
	int result = riscv_program_exec(&program, target);

	if (result != ERROR_OK) {
		/* This program might have failed if the program buffer is not
		 * writable. */
		info->progbuf_writable = YNM_NO;
		return ERROR_OK;
	}

	uint32_t written;
	if (dm_read(target, &written, DM_PROGBUF0) != ERROR_OK)
		return ERROR_FAIL;
	if (written == (uint32_t) info->progbuf_address) {
		LOG_TARGET_INFO(target, "progbuf is writable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_YES;

	} else {
		LOG_TARGET_INFO(target, "progbuf is not writeable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_NO;
	}

	return ERROR_OK;
}

static int is_fpu_reg(enum gdb_regno gdb_regno)
{
	return (gdb_regno >= GDB_REGNO_FPR0 && gdb_regno <= GDB_REGNO_FPR31) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FFLAGS) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FRM) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FCSR);
}

static int is_vector_reg(enum gdb_regno gdb_regno)
{
	return (gdb_regno >= GDB_REGNO_V0 && gdb_regno <= GDB_REGNO_V31) ||
		gdb_regno == GDB_REGNO_VSTART ||
		gdb_regno == GDB_REGNO_VXSAT ||
		gdb_regno == GDB_REGNO_VXRM ||
		gdb_regno == GDB_REGNO_VCSR ||
		gdb_regno == GDB_REGNO_VL ||
		gdb_regno == GDB_REGNO_VTYPE ||
		gdb_regno == GDB_REGNO_VLENB;
}

static int prep_for_register_access(struct target *target,
		riscv_reg_t *orig_mstatus, enum gdb_regno regno)
{
	assert(orig_mstatus);

	if (!is_fpu_reg(regno) && !is_vector_reg(regno)) {
		/* If we don't assign orig_mstatus, clang static analysis
		 * complains when this value is passed to
		 * cleanup_after_register_access(). */
		*orig_mstatus = 0;
		/* No special preparation needed */
		return ERROR_OK;
	}

	LOG_TARGET_DEBUG(target, "Preparing mstatus to access %s",
			riscv_reg_gdb_regno_name(target, regno));

	assert(target->state == TARGET_HALTED &&
			"The target must be halted to modify and then restore mstatus");

	if (riscv_reg_get(target, orig_mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
		return ERROR_FAIL;

	riscv_reg_t new_mstatus = *orig_mstatus;
	riscv_reg_t field_mask = is_fpu_reg(regno) ? MSTATUS_FS : MSTATUS_VS;

	if ((new_mstatus & field_mask) != 0)
		return ERROR_OK;

	new_mstatus = set_field(new_mstatus, field_mask, 1);

	if (riscv_reg_write(target, GDB_REGNO_MSTATUS, new_mstatus) != ERROR_OK)
		return ERROR_FAIL;

	LOG_TARGET_DEBUG(target, "Prepared to access %s (mstatus=0x%" PRIx64 ")",
			riscv_reg_gdb_regno_name(target, regno), new_mstatus);
	return ERROR_OK;
}

static int cleanup_after_register_access(struct target *target,
		riscv_reg_t mstatus, enum gdb_regno regno)
{
	if (!is_fpu_reg(regno) && !is_vector_reg(regno))
		/* Mstatus was not changed for this register access. No need to restore it. */
		return ERROR_OK;

	LOG_TARGET_DEBUG(target, "Restoring mstatus to 0x%" PRIx64, mstatus);
	return riscv_reg_write(target, GDB_REGNO_MSTATUS, mstatus);
}

typedef enum {
	SPACE_DM_DATA,
	SPACE_DMI_PROGBUF,
	SPACE_DMI_RAM
} memory_space_t;

typedef struct {
	/* How can the debugger access this memory? */
	memory_space_t memory_space;
	/* Memory address to access the scratch memory from the hart. */
	riscv_addr_t hart_address;
	/* Memory address to access the scratch memory from the debugger. */
	riscv_addr_t debug_address;
	struct working_area *area;
} scratch_mem_t;

/**
 * Find some scratch memory to be used with the given program.
 */
static int scratch_reserve(struct target *target,
		scratch_mem_t *scratch,
		struct riscv_program *program,
		unsigned size_bytes)
{
	riscv_addr_t alignment = 1;
	while (alignment < size_bytes)
		alignment *= 2;

	scratch->area = NULL;

	riscv013_info_t *info = get_info(target);

	/* Option 1: See if data# registers can be used as the scratch memory */
	if (info->dataaccess == 1) {
		/* Sign extend dataaddr. */
		scratch->hart_address = info->dataaddr;
		if (info->dataaddr & (1<<11))
			scratch->hart_address |= 0xfffffffffffff000ULL;
		/* Align. */
		scratch->hart_address = (scratch->hart_address + alignment - 1) & ~(alignment - 1);

		if ((size_bytes + scratch->hart_address - info->dataaddr + 3) / 4 >=
				info->datasize) {
			scratch->memory_space = SPACE_DM_DATA;
			scratch->debug_address = (scratch->hart_address - info->dataaddr) / 4;
			return ERROR_OK;
		}
	}

	/* Option 2: See if progbuf can be used as the scratch memory */
	if (examine_progbuf(target) != ERROR_OK)
		return ERROR_FAIL;

	/* Allow for ebreak at the end of the program. */
	unsigned program_size = (program->instruction_count + 1) * 4;
	scratch->hart_address = (info->progbuf_address + program_size + alignment - 1) &
		~(alignment - 1);
	if ((info->progbuf_writable == YNM_YES) &&
			((size_bytes + scratch->hart_address - info->progbuf_address + 3) / 4 >=
			info->progbufsize)) {
		scratch->memory_space = SPACE_DMI_PROGBUF;
		scratch->debug_address = (scratch->hart_address - info->progbuf_address) / 4;
		return ERROR_OK;
	}

	/* Option 3: User-configured memory area as scratch RAM */
	if (target_alloc_working_area(target, size_bytes + alignment - 1,
				&scratch->area) == ERROR_OK) {
		scratch->hart_address = (scratch->area->address + alignment - 1) &
			~(alignment - 1);
		scratch->memory_space = SPACE_DMI_RAM;
		scratch->debug_address = scratch->hart_address;
		return ERROR_OK;
	}

	LOG_TARGET_ERROR(target, "Couldn't find %d bytes of scratch RAM to use. Please configure "
			"a work area with 'configure -work-area-phys'.", size_bytes);
	return ERROR_FAIL;
}

static int scratch_release(struct target *target,
		scratch_mem_t *scratch)
{
	return target_free_working_area(target, scratch->area);
}

static int scratch_read64(struct target *target, scratch_mem_t *scratch,
		uint64_t *value)
{
	uint32_t v;
	switch (scratch->memory_space) {
		case SPACE_DM_DATA:
			if (dm_read(target, &v, DM_DATA0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dm_read(target, &v, DM_DATA1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_PROGBUF:
			if (dm_read(target, &v, DM_PROGBUF0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dm_read(target, &v, DM_PROGBUF1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_RAM:
			{
				uint8_t buffer[8] = {0};
				if (read_memory(target, scratch->debug_address, 4, 2, buffer, 4) != ERROR_OK)
					return ERROR_FAIL;
				*value = buffer[0] |
					(((uint64_t) buffer[1]) << 8) |
					(((uint64_t) buffer[2]) << 16) |
					(((uint64_t) buffer[3]) << 24) |
					(((uint64_t) buffer[4]) << 32) |
					(((uint64_t) buffer[5]) << 40) |
					(((uint64_t) buffer[6]) << 48) |
					(((uint64_t) buffer[7]) << 56);
			}
			break;
	}
	return ERROR_OK;
}

static int scratch_write64(struct target *target, scratch_mem_t *scratch,
		uint64_t value)
{
	switch (scratch->memory_space) {
		case SPACE_DM_DATA:
			dm_write(target, DM_DATA0 + scratch->debug_address, (uint32_t)value);
			dm_write(target, DM_DATA1 + scratch->debug_address, (uint32_t)(value >> 32));
			break;
		case SPACE_DMI_PROGBUF:
			dm_write(target, DM_PROGBUF0 + scratch->debug_address, (uint32_t)value);
			dm_write(target, DM_PROGBUF1 + scratch->debug_address, (uint32_t)(value >> 32));
			riscv013_invalidate_cached_progbuf(target);
			break;
		case SPACE_DMI_RAM:
			{
				uint8_t buffer[8] = {
					value,
					value >> 8,
					value >> 16,
					value >> 24,
					value >> 32,
					value >> 40,
					value >> 48,
					value >> 56
				};
				if (write_memory(target, scratch->debug_address, 4, 2, buffer) != ERROR_OK)
					return ERROR_FAIL;
			}
			break;
	}
	return ERROR_OK;
}

/** Return register size in bits. */
static unsigned int register_size(struct target *target, enum gdb_regno number)
{
	/* If reg_cache hasn't been initialized yet, make a guess. We need this for
	 * when this function is called during examine(). */
	if (target->reg_cache)
		return target->reg_cache->reg_list[number].size;
	else
		return riscv_xlen(target);
}

static bool has_sufficient_progbuf(struct target *target, unsigned size)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);

	return info->progbufsize + r->impebreak >= size;
}

/**
 * This function is used to read a 64-bit value from a register by executing a
 * program.
 * The program stores a register to address located in S0.
 * The caller should save S0.
 */
static int internal_register_read64_progbuf_scratch(struct target *target,
		struct riscv_program *program, riscv_reg_t *value)
{
	scratch_mem_t scratch;

	if (scratch_reserve(target, &scratch, program, 8) != ERROR_OK)
		return ERROR_FAIL;

	if (register_write_abstract(target, GDB_REGNO_S0, scratch.hart_address)
			!= ERROR_OK) {
		scratch_release(target, &scratch);
		return ERROR_FAIL;
	}
	if (riscv_program_exec(program, target) != ERROR_OK) {
		scratch_release(target, &scratch);
		return ERROR_FAIL;
	}

	int result = scratch_read64(target, &scratch, value);

	scratch_release(target, &scratch);
	return result;
}

static int fpr_read_progbuf(struct target *target, uint64_t *value,
		enum gdb_regno number)
{
	assert(target->state == TARGET_HALTED);
	assert(number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31);

	const unsigned int freg = number - GDB_REGNO_FPR0;

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_supports_extension(target, 'D') && riscv_xlen(target) < 64) {
		/* There are no instructions to move all the bits from a
		 * register, so we need to use some scratch RAM.
		 */
		if (riscv_program_insert(&program, fsd(freg, S0, 0)) != ERROR_OK)
			return ERROR_FAIL;
		return internal_register_read64_progbuf_scratch(target, &program, value);
	}
	if (riscv_program_insert(&program,
				riscv_supports_extension(target, 'D') ?
				fmv_x_d(S0, freg) : fmv_x_w(S0, freg)) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_program_exec(&program, target) != ERROR_OK)
		return ERROR_FAIL;

	return register_read_abstract(target, value, GDB_REGNO_S0) != ERROR_OK;
}

static int csr_read_progbuf(struct target *target, uint64_t *value,
		enum gdb_regno number)
{
	assert(target->state == TARGET_HALTED);
	assert(number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095);

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_program_csrr(&program, S0, number) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_exec(&program, target) != ERROR_OK)
		return ERROR_FAIL;

	return register_read_abstract(target, value, GDB_REGNO_S0) != ERROR_OK;
}

/**
 * This function reads a register by writing a program to program buffer and
 * executing it.
 */
static int register_read_progbuf(struct target *target, uint64_t *value,
		enum gdb_regno number)
{
	assert(target->state == TARGET_HALTED);

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31)
		return fpr_read_progbuf(target, value, number);
	else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095)
		return csr_read_progbuf(target, value, number);

	LOG_TARGET_ERROR(target, "Unexpected read of %s via program buffer.",
			riscv_reg_gdb_regno_name(target, number));
	return ERROR_FAIL;
}

/**
 * This function is used to write a 64-bit value to a register by executing a
 * program.
 * The program loads a value from address located in S0 to a register.
 * The caller should save S0.
 */
static int internal_register_write64_progbuf_scratch(struct target *target,
		struct riscv_program *program, riscv_reg_t value)
{
	scratch_mem_t scratch;

	if (scratch_reserve(target, &scratch, program, 8) != ERROR_OK)
		return ERROR_FAIL;

	if (register_write_abstract(target, GDB_REGNO_S0, scratch.hart_address)
			!= ERROR_OK) {
		scratch_release(target, &scratch);
		return ERROR_FAIL;
	}
	if (scratch_write64(target, &scratch, value) != ERROR_OK) {
		scratch_release(target, &scratch);
		return ERROR_FAIL;
	}
	int result = riscv_program_exec(program, target);

	scratch_release(target, &scratch);
	return result;
}

static int fpr_write_progbuf(struct target *target, enum gdb_regno number,
		riscv_reg_t value)
{
	assert(target->state == TARGET_HALTED);
	assert(number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31);
	const unsigned int freg = number - GDB_REGNO_FPR0;

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);

	if (riscv_supports_extension(target, 'D') && riscv_xlen(target) < 64) {
		/* There are no instructions to move all the bits from a register,
		 * so we need to use some scratch RAM.
		 */
		if (riscv_program_insert(&program, fld(freg, S0, 0)) != ERROR_OK)
			return ERROR_FAIL;
		return internal_register_write64_progbuf_scratch(target, &program, value);
	}

	if (register_write_abstract(target, GDB_REGNO_S0, value) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_program_insert(&program,
			riscv_supports_extension(target, 'D') ?
			fmv_d_x(freg, S0) : fmv_w_x(freg, S0)) != ERROR_OK)
		return ERROR_FAIL;

	return riscv_program_exec(&program, target);
}

static int vtype_write_progbuf(struct target *target, riscv_reg_t value)
{
	assert(target->state == TARGET_HALTED);

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_abstract(target, GDB_REGNO_S0, value) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv013_reg_save(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_program_insert(&program, csrr(S1, CSR_VL)) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(&program, vsetvl(ZERO, S1, S0)) != ERROR_OK)
		return ERROR_FAIL;

	return riscv_program_exec(&program, target);
}

static int vl_write_progbuf(struct target *target, riscv_reg_t value)
{
	assert(target->state == TARGET_HALTED);

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_abstract(target, GDB_REGNO_S0, value) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv013_reg_save(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_program_insert(&program, csrr(S1, CSR_VTYPE)) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(&program, vsetvl(ZERO, S0, S1)) != ERROR_OK)
		return ERROR_FAIL;

	return riscv_program_exec(&program, target);
}

static int csr_write_progbuf(struct target *target, enum gdb_regno number,
		riscv_reg_t value)
{
	assert(target->state == TARGET_HALTED);
	assert(number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095);

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_abstract(target, GDB_REGNO_S0, value) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_program_csrw(&program, S0, number) != ERROR_OK)
		return ERROR_FAIL;

	return riscv_program_exec(&program, target);
}

/**
 * This function writes a register by writing a program to program buffer and
 * executing it.
 */
static int register_write_progbuf(struct target *target, enum gdb_regno number,
		riscv_reg_t value)
{
	assert(target->state == TARGET_HALTED);

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31)
		return fpr_write_progbuf(target, number, value);
	else if (number == GDB_REGNO_VTYPE)
		return vtype_write_progbuf(target, value);
	else if (number == GDB_REGNO_VL)
		return vl_write_progbuf(target, value);
	else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095)
		return csr_write_progbuf(target, number, value);

	LOG_TARGET_ERROR(target, "Unexpected write to %s via program buffer.",
			riscv_reg_gdb_regno_name(target, number));
	return ERROR_FAIL;
}

/**
 * Immediately write the new value to the requested register. This mechanism
 * bypasses any caches.
 */
static int register_write_direct(struct target *target, enum gdb_regno number,
		riscv_reg_t value)
{
	LOG_TARGET_DEBUG(target, "Writing 0x%" PRIx64 " to %s", value,
			riscv_reg_gdb_regno_name(target, number));

	if (target->state != TARGET_HALTED)
		return register_write_abstract(target, number, value);

	riscv_reg_t mstatus;
	if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	int result = register_write_abstract(target, number, value);

	if (result != ERROR_OK && target->state == TARGET_HALTED)
		result = register_write_progbuf(target, number, value);

	if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	if (result == ERROR_OK)
		LOG_TARGET_DEBUG(target, "%s <- 0x%" PRIx64, riscv_reg_gdb_regno_name(target, number),
				value);

	return result;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, riscv_reg_t *value,
		enum gdb_regno number)
{
	LOG_TARGET_DEBUG(target, "Reading %s", riscv_reg_gdb_regno_name(target, number));

	if (target->state != TARGET_HALTED)
		return register_read_abstract(target, value, number);

	riscv_reg_t mstatus;

	if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	int result = register_read_abstract(target, value, number);

	if (result != ERROR_OK && target->state == TARGET_HALTED)
		result = register_read_progbuf(target, value, number);

	if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	if (result == ERROR_OK)
		LOG_TARGET_DEBUG(target, "%s = 0x%" PRIx64, riscv_reg_gdb_regno_name(target, number),
				*value);

	return result;
}

static int wait_for_authbusy(struct target *target, uint32_t *dmstatus)
{
	time_t start = time(NULL);
	while (1) {
		uint32_t value;
		if (dmstatus_read(target, &value, false) != ERROR_OK)
			return ERROR_FAIL;
		if (dmstatus)
			*dmstatus = value;
		if (!get_field(value, DM_DMSTATUS_AUTHBUSY))
			break;
		if (time(NULL) - start > riscv_get_command_timeout_sec()) {
			LOG_TARGET_ERROR(target, "Timed out after %ds waiting for authbusy to go low (dmstatus=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_get_command_timeout_sec(),
					value);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int set_dcsr_ebreak(struct target *target, bool step)
{
	LOG_TARGET_DEBUG(target, "Set dcsr.ebreak*");

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	RISCV_INFO(r);
	RISCV013_INFO(info);
	riscv_reg_t original_dcsr, dcsr;
	/* We want to twiddle some bits in the debug CSR so debugging works. */
	if (riscv_reg_get(target, &dcsr, GDB_REGNO_DCSR) != ERROR_OK)
		return ERROR_FAIL;
	original_dcsr = dcsr;
	dcsr = set_field(dcsr, CSR_DCSR_STEP, step);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, r->riscv_ebreakm);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, r->riscv_ebreaks && riscv_supports_extension(target, 'S'));
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, r->riscv_ebreaku && riscv_supports_extension(target, 'U'));
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKVS, r->riscv_ebreaku && riscv_supports_extension(target, 'H'));
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKVU, r->riscv_ebreaku && riscv_supports_extension(target, 'H'));
	if (dcsr != original_dcsr &&
			riscv_reg_set(target, GDB_REGNO_DCSR, dcsr) != ERROR_OK)
		return ERROR_FAIL;
	info->dcsr_ebreak_is_set = true;
	return ERROR_OK;
}

static int halt_set_dcsr_ebreak(struct target *target)
{
	RISCV_INFO(r);
	RISCV013_INFO(info);
	LOG_TARGET_DEBUG(target, "Halt to set DCSR.ebreak*");

	/* Remove this hart from the halt group.  This won't work on all targets
	 * because the debug spec allows halt groups to be hard-coded, but I
	 * haven't actually encountered those in the wild yet.
	 *
	 * There is a possible race condition when another hart halts, and
	 * this one is expected to also halt because it's supposed to be in the
	 * same halt group. Or when this hart is halted when that happens.
	 *
	 * A better solution might be to leave the halt groups alone, and track
	 * why we're halting when a halt occurs. When there are halt groups,
	 * that leads to extra halting if not all harts need to set dcsr.ebreak
	 * at the same time.  It also makes for more complicated code.
	 *
	 * The perfect solution would be Quick Access, but I'm not aware of any
	 * hardware that implements it.
	 *
	 * We don't need a perfect solution, because we only get here when a
	 * hart spontaneously resets, or when it powers down and back up again.
	 * Those are both relatively rare. (At least I hope so. Maybe some
	 * design just powers each hart down for 90ms out of every 100ms)
	 */


	if (info->haltgroup_supported) {
		bool supported;
		if (set_group(target, &supported, 0, HALT_GROUP) != ERROR_OK)
			return ERROR_FAIL;
		if (!supported)
			LOG_TARGET_ERROR(target, "Couldn't place hart in halt group 0. "
						 "Some harts may be unexpectedly halted.");
	}

	int result = ERROR_OK;

	r->prepped = true;
	if (riscv013_halt_go(target) != ERROR_OK ||
			set_dcsr_ebreak(target, false) != ERROR_OK ||
			riscv013_step_or_resume_current_hart(target, false) != ERROR_OK) {
		result = ERROR_FAIL;
	} else {
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
	}

	/* Add it back to the halt group. */
	if (info->haltgroup_supported) {
		bool supported;
		if (set_group(target, &supported, target->smp, HALT_GROUP) != ERROR_OK)
			return ERROR_FAIL;
		if (!supported)
			LOG_TARGET_ERROR(target, "Couldn't place hart back in halt group %d. "
						 "Some harts may be unexpectedly halted.", target->smp);
	}

	return result;
}

/*** OpenOCD target functions. ***/

static void deinit_target(struct target *target)
{
	LOG_TARGET_DEBUG(target, "Deinitializing target.");
	struct riscv_info *info = target->arch_info;
	if (!info)
		return;

	riscv013_dm_free(target);

	free(info->version_specific);
	/* TODO: free register arch_info */
	info->version_specific = NULL;
}

static int set_group(struct target *target, bool *supported, unsigned int group,
		grouptype_t grouptype)
{
	uint32_t write_val = DM_DMCS2_HGWRITE;
	assert(group <= 31);
	write_val = set_field(write_val, DM_DMCS2_GROUP, group);
	write_val = set_field(write_val, DM_DMCS2_GROUPTYPE, (grouptype == HALT_GROUP) ? 0 : 1);
	if (dm_write(target, DM_DMCS2, write_val) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t read_val;
	if (dm_read(target, &read_val, DM_DMCS2) != ERROR_OK)
		return ERROR_FAIL;
	if (supported)
		*supported = (get_field(read_val, DM_DMCS2_GROUP) == group);
	return ERROR_OK;
}

static int wait_for_idle_if_needed(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (!dm->abstract_cmd_maybe_busy)
		/* The previous abstract command ended correctly
		 * and busy was cleared. No need to do anything. */
		return ERROR_OK;

	/* The previous abstract command timed out and abstractcs.busy
	 * may have remained set. Wait for it to get cleared. */
	uint32_t abstractcs;
	int result = wait_for_idle(target, &abstractcs);
	if (result != ERROR_OK)
		return result;
	LOG_DEBUG_REG(target, DM_ABSTRACTCS, abstractcs);
	return ERROR_OK;
}

static int reset_dm(struct target *target)
{
	/* TODO: This function returns an error when a DMI operation fails.
	 * However, [3.14.2. Debug Module Control] states:
	 * > 0 (inactive): ... Any accesses to the module may fail.
	 *
	 * Ignoring failures may introduce incompatibility with 0.13.
	 * See https://github.com/riscv/riscv-debug-spec/issues/1021
	 */
	dm013_info_t *dm = get_dm(target);
	assert(dm && "DM is expected to be already allocated.");
	assert(!dm->was_reset && "Attempt to reset an already-reset debug module.");
	/* `dmcontrol.hartsel` should be read first, in order not to
	 * change it when requesting the reset, since changing it
	 * without checking that `abstractcs.busy` is low is
	 * prohibited.
	 */
	uint32_t dmcontrol;
	int result = dm_read(target, &dmcontrol, DM_DMCONTROL);
	if (result != ERROR_OK)
		return result;

	if (get_field32(dmcontrol, DM_DMCONTROL_DMACTIVE)) {
		/* `dmcontrol.hartsel` is not changed. */
		dmcontrol = (dmcontrol & DM_DMCONTROL_HARTSELLO) |
			(dmcontrol & DM_DMCONTROL_HARTSELHI);
		LOG_TARGET_DEBUG(target, "Initiating DM reset.");
		result = dm_write(target, DM_DMCONTROL, dmcontrol);
		if (result != ERROR_OK)
			return result;

		const time_t start = time(NULL);
		LOG_TARGET_DEBUG(target, "Waiting for the DM to acknowledge reset.");
		do {
			result = dm_read(target, &dmcontrol, DM_DMCONTROL);
			if (result != ERROR_OK)
				return result;

			if (time(NULL) - start > riscv_get_command_timeout_sec()) {
				LOG_TARGET_ERROR(target, "DM didn't acknowledge reset in %d s. "
						"Increase the timeout with 'riscv set_command_timeout_sec'.",
						riscv_get_command_timeout_sec());
				return ERROR_TIMEOUT_REACHED;
			}
		} while (get_field32(dmcontrol, DM_DMCONTROL_DMACTIVE));
		LOG_TARGET_DEBUG(target, "DM reset initiated.");
	}

	LOG_TARGET_DEBUG(target, "Activating the DM.");
	result = dm_write(target, DM_DMCONTROL, DM_DMCONTROL_DMACTIVE);
	if (result != ERROR_OK)
		return result;

	const time_t start = time(NULL);
	LOG_TARGET_DEBUG(target, "Waiting for the DM to come out of reset.");
	do {
		result = dm_read(target, &dmcontrol, DM_DMCONTROL);
		if (result != ERROR_OK)
			return result;

		if (time(NULL) - start > riscv_get_command_timeout_sec()) {
			LOG_TARGET_ERROR(target, "Debug Module did not become active in %d s. "
					"Increase the timeout with 'riscv set_command_timeout_sec'.",
					riscv_get_command_timeout_sec());
			return ERROR_TIMEOUT_REACHED;
		}
	} while (!get_field32(dmcontrol, DM_DMCONTROL_DMACTIVE));

	LOG_TARGET_DEBUG(target, "DM successfully reset.");
	dm->was_reset = true;
	return ERROR_OK;
}

static int examine_dm(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (dm->was_examined)
		return ERROR_OK;

	int result = ERROR_FAIL;

	if (dm->was_reset) {
		/* The DM was already reset when examining a different hart.
		 * No need to reset it again. But for safety, assume that an abstract
		 * command might be in progress at the moment.
		 */
		dm->abstract_cmd_maybe_busy = true;
	} else {
		result = reset_dm(target);
		if (result != ERROR_OK)
			return result;
	}

	dm->current_hartid = HART_INDEX_UNKNOWN;

	result = dm_write(target, DM_DMCONTROL, DM_DMCONTROL_HARTSELLO |
			DM_DMCONTROL_HARTSELHI | DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_HASEL);
	if (result != ERROR_OK)
		return result;

	uint32_t dmcontrol;
	result = dm_read(target, &dmcontrol, DM_DMCONTROL);
	if (result != ERROR_OK)
		return result;

	dm->hasel_supported = get_field(dmcontrol, DM_DMCONTROL_HASEL);

	uint32_t hartsel =
		(get_field(dmcontrol, DM_DMCONTROL_HARTSELHI) <<
		 DM_DMCONTROL_HARTSELLO_LENGTH) |
		get_field(dmcontrol, DM_DMCONTROL_HARTSELLO);

	/* Before doing anything else we must first enumerate the harts. */
	const int max_hart_count = MIN(RISCV_MAX_HARTS, hartsel + 1);
	if (dm->hart_count < 0) {
		for (int i = 0; i < max_hart_count; ++i) {
			/* TODO: This is extremely similar to
			 * riscv013_get_hart_state().
			 * It would be best to reuse the code.
			 */
			result = dm013_select_hart(target, i);
			if (result != ERROR_OK)
				return result;

			uint32_t s;
			result = dmstatus_read(target, &s, /*authenticated*/ true);
			if (result != ERROR_OK)
				return result;

			if (get_field(s, DM_DMSTATUS_ANYNONEXISTENT))
				break;

			dm->hart_count = i + 1;

			if (get_field(s, DM_DMSTATUS_ANYHAVERESET)) {
				dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_ACKHAVERESET;
				/* If `abstractcs.busy` is set, debugger should not
				 * change `hartsel`.
				 */
				result = wait_for_idle_if_needed(target);
				if (result != ERROR_OK)
					return result;
				dmcontrol = set_dmcontrol_hartsel(dmcontrol, i);
				result = dm_write(target, DM_DMCONTROL, dmcontrol);
				if (result != ERROR_OK)
					return result;
			}
		}
		LOG_TARGET_DEBUG(target, "Detected %d harts.", dm->hart_count);
	}

	if (dm->hart_count <= 0) {
		LOG_TARGET_ERROR(target, "No harts found!");
		return ERROR_FAIL;
	}

	dm->was_examined = true;
	return ERROR_OK;
}

static int examine(struct target *target)
{
	/* We reset target state in case if something goes wrong during examine:
	 * DTM/DM scans could fail or hart may fail to halt. */
	target->state = TARGET_UNKNOWN;
	target->debug_reason = DBG_REASON_UNDEFINED;

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */
	LOG_TARGET_DEBUG(target, "dbgbase=0x%x", target->dbgbase);

	uint32_t dtmcontrol;
	if (dtmcontrol_scan(target, 0, &dtmcontrol) != ERROR_OK || dtmcontrol == 0) {
		LOG_TARGET_ERROR(target, "Could not scan dtmcontrol. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "dtmcontrol=0x%x", dtmcontrol);
	LOG_DEBUG_REG(target, DTM_DTMCS, dtmcontrol);

	if (get_field(dtmcontrol, DTM_DTMCS_VERSION) != 1) {
		LOG_TARGET_ERROR(target, "Unsupported DTM version %" PRIu32 ". (dtmcontrol=0x%" PRIx32 ")",
				get_field32(dtmcontrol, DTM_DTMCS_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	riscv013_info_t *info = get_info(target);

	info->index = target->coreid;
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcs_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	if (!check_dbgbase_exists(target)) {
		LOG_TARGET_ERROR(target, "Could not find debug module with DMI base address (dbgbase) = 0x%x", target->dbgbase);
		return ERROR_FAIL;
	}

	int result = examine_dm(target);
	if (result != ERROR_OK)
		return result;

	result = dm013_select_target(target);
	if (result != ERROR_OK)
		return result;

	/* We're here because we're uncertain about the state of the target. That
	 * includes our progbuf cache. */
	riscv013_invalidate_cached_progbuf(target);

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, false) != ERROR_OK)
		return ERROR_FAIL;
	LOG_TARGET_DEBUG(target, "dmstatus:  0x%08x", dmstatus);
	int dmstatus_version = get_field(dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		/* Error was already printed out in dmstatus_read(). */
		return ERROR_FAIL;
	}

	uint32_t hartinfo;
	if (dm_read(target, &hartinfo, DM_HARTINFO) != ERROR_OK)
		return ERROR_FAIL;

	info->datasize = get_field(hartinfo, DM_HARTINFO_DATASIZE);
	info->dataaccess = get_field(hartinfo, DM_HARTINFO_DATAACCESS);
	info->dataaddr = get_field(hartinfo, DM_HARTINFO_DATAADDR);

	if (!get_field(dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_TARGET_ERROR(target, "Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", dmstatus);
		return ERROR_FAIL;
	}

	if (dm_read(target, &info->sbcs, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;

	/* Check that abstract data registers are accessible. */
	uint32_t abstractcs;
	if (dm_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
		return ERROR_FAIL;
	info->datacount = get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT);
	info->progbufsize = get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE);

	LOG_TARGET_INFO(target, "datacount=%d progbufsize=%d",
			info->datacount, info->progbufsize);

	RISCV_INFO(r);
	r->impebreak = get_field(dmstatus, DM_DMSTATUS_IMPEBREAK);

	if (!has_sufficient_progbuf(target, 2)) {
		LOG_TARGET_WARNING(target, "We won't be able to execute fence instructions on this "
				"target. Memory may not always appear consistent. "
				"(progbufsize=%d, impebreak=%d)", info->progbufsize,
				r->impebreak);
	}

	if (info->progbufsize < 4 && riscv_enable_virtual) {
		LOG_TARGET_ERROR(target, "set_enable_virtual is not available on this target. It "
				"requires a program buffer size of at least 4. (progbufsize=%d) "
				"Use `riscv set_enable_virtual off` to continue."
					, info->progbufsize);
	}

	/* Don't call any riscv_* functions until after we've counted the number of
	 * cores and initialized registers. */

	enum riscv_hart_state state_at_examine_start;
	if (riscv_get_hart_state(target, &state_at_examine_start) != ERROR_OK)
		return ERROR_FAIL;
	const bool hart_halted_at_examine_start = state_at_examine_start == RISCV_STATE_HALTED;
	if (!hart_halted_at_examine_start) {
		r->prepped = true;
		if (riscv013_halt_go(target) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Fatal: Hart %d failed to halt during %s",
					info->index, __func__);
			return ERROR_FAIL;
		}
	}

	target->state = TARGET_HALTED;
	target->debug_reason = hart_halted_at_examine_start ? DBG_REASON_UNDEFINED : DBG_REASON_DBGRQ;

	/* Without knowing anything else we can at least mess with the
	 * program buffer. */
	r->progbuf_size = info->progbufsize;

	result = register_read_abstract_with_size(target, NULL, GDB_REGNO_S0, 64);
	if (result == ERROR_OK)
		r->xlen = 64;
	else
		r->xlen = 32;

	/* Save s0 and s1. The register cache hasn't be initialized yet so we
	 * need to take care of this manually. */
	uint64_t s0, s1;
	if (register_read_abstract(target, &s0, GDB_REGNO_S0) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to read s0.");
		return ERROR_FAIL;
	}
	if (register_read_abstract(target, &s1, GDB_REGNO_S1) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to read s1.");
		return ERROR_FAIL;
	}

	if (register_read_direct(target, &r->misa, GDB_REGNO_MISA)) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to read MISA.");
		return ERROR_FAIL;
	}

	uint64_t value;
	if (register_read_direct(target, &value, GDB_REGNO_VLENB) != ERROR_OK) {
		if (riscv_supports_extension(target, 'V'))
			LOG_TARGET_WARNING(target, "Couldn't read vlenb; vector register access won't work.");
		r->vlenb = 0;
	} else {
		r->vlenb = value;
		LOG_TARGET_INFO(target, "Vector support with vlenb=%d", r->vlenb);
	}

	if (register_read_direct(target, &value, GDB_REGNO_MTOPI) == ERROR_OK) {
		r->mtopi_readable = true;

		if (register_read_direct(target, &value, GDB_REGNO_MTOPEI) == ERROR_OK) {
			LOG_TARGET_INFO(target, "S?aia detected with IMSIC");
			r->mtopei_readable = true;
		} else {
			r->mtopei_readable = false;
			LOG_TARGET_INFO(target, "S?aia detected without IMSIC");
		}
	} else {
		r->mtopi_readable = false;
	}

	/* Display this as early as possible to help people who are using
	 * really slow simulators. */
	LOG_TARGET_DEBUG(target, " XLEN=%d, misa=0x%" PRIx64, r->xlen, r->misa);

	/* Restore s0 and s1. */
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to write back s0.");
		return ERROR_FAIL;
	}
	if (register_write_direct(target, GDB_REGNO_S1, s1) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to write back s1.");
		return ERROR_FAIL;
	}

	/* Now init registers based on what we discovered. */
	if (riscv013_reg_init_all(target) != ERROR_OK)
		return ERROR_FAIL;

	if (set_dcsr_ebreak(target, false) != ERROR_OK)
		return ERROR_FAIL;

	if (state_at_examine_start == RISCV_STATE_RUNNING) {
		riscv013_step_or_resume_current_hart(target, false);
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
	} else if (state_at_examine_start == RISCV_STATE_HALTED) {
		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_UNDEFINED;
	}

	if (target->smp) {
		if (set_group(target, &info->haltgroup_supported, target->smp, HALT_GROUP) != ERROR_OK)
			return ERROR_FAIL;
		if (info->haltgroup_supported)
			LOG_TARGET_INFO(target, "Core %d made part of halt group %d.", info->index,
					target->smp);
		else
			LOG_TARGET_INFO(target, "Core %d could not be made part of halt group %d.",
					info->index, target->smp);
	}

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	 * when they can connect with gdb/telnet.
	 * We will need to update those suites if we want to change that text. */
	LOG_TARGET_INFO(target, "Examined RISC-V core");
	LOG_TARGET_INFO(target, " XLEN=%d, misa=0x%" PRIx64, r->xlen, r->misa);
	return ERROR_OK;
}

static int riscv013_authdata_read(struct target *target, uint32_t *value, unsigned int index)
{
	if (index > 0) {
		LOG_TARGET_ERROR(target, "Spec 0.13 only has a single authdata register.");
		return ERROR_FAIL;
	}

	if (wait_for_authbusy(target, NULL) != ERROR_OK)
		return ERROR_FAIL;

	return dm_read(target, value, DM_AUTHDATA);
}

static int riscv013_authdata_write(struct target *target, uint32_t value, unsigned int index)
{
	if (index > 0) {
		LOG_TARGET_ERROR(target, "Spec 0.13 only has a single authdata register.");
		return ERROR_FAIL;
	}

	uint32_t before, after;
	if (wait_for_authbusy(target, &before) != ERROR_OK)
		return ERROR_FAIL;

	dm_write(target, DM_AUTHDATA, value);

	if (wait_for_authbusy(target, &after) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(before, DM_DMSTATUS_AUTHENTICATED) &&
			get_field(after, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_TARGET_INFO(target, "authdata_write resulted in successful authentication");
		int result = ERROR_OK;
		dm013_info_t *dm = get_dm(target);
		if (!dm)
			return ERROR_FAIL;
		target_list_t *entry;
		list_for_each_entry(entry, &dm->target_list, list) {
			if (target_examine_one(entry->target) != ERROR_OK)
				result = ERROR_FAIL;
		}
		return result;
	}

	return ERROR_OK;
}

/* Try to find out the widest memory access size depending on the selected memory access methods. */
static unsigned riscv013_data_bits(struct target *target)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);

	for (unsigned int i = 0; i < RISCV_NUM_MEM_ACCESS_METHODS; i++) {
		int method = r->mem_access_methods[i];

		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			if (has_sufficient_progbuf(target, 3))
				return riscv_xlen(target);
		} else if (method == RISCV_MEM_ACCESS_SYSBUS) {
			if (get_field(info->sbcs, DM_SBCS_SBACCESS128))
				return 128;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS64))
				return 64;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS32))
				return 32;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS16))
				return 16;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS8))
				return 8;
		} else if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			/* TODO: Once there is a spec for discovering abstract commands, we can
			 * take those into account as well.  For now we assume abstract commands
			 * support XLEN-wide accesses. */
			return riscv_xlen(target);
		} else if (method == RISCV_MEM_ACCESS_UNSPECIFIED)
			/* No further mem access method to try. */
			break;
	}
	LOG_TARGET_ERROR(target, "Unable to determine supported data bits on this target. Assuming 32 bits.");
	return 32;
}

static COMMAND_HELPER(riscv013_print_info, struct target *target)
{
	RISCV013_INFO(info);

	/* Abstract description. */
	riscv_print_info_line(CMD, "target", "memory.read_while_running8", get_field(info->sbcs, DM_SBCS_SBACCESS8));
	riscv_print_info_line(CMD, "target", "memory.write_while_running8", get_field(info->sbcs, DM_SBCS_SBACCESS8));
	riscv_print_info_line(CMD, "target", "memory.read_while_running16", get_field(info->sbcs, DM_SBCS_SBACCESS16));
	riscv_print_info_line(CMD, "target", "memory.write_while_running16", get_field(info->sbcs, DM_SBCS_SBACCESS16));
	riscv_print_info_line(CMD, "target", "memory.read_while_running32", get_field(info->sbcs, DM_SBCS_SBACCESS32));
	riscv_print_info_line(CMD, "target", "memory.write_while_running32", get_field(info->sbcs, DM_SBCS_SBACCESS32));
	riscv_print_info_line(CMD, "target", "memory.read_while_running64", get_field(info->sbcs, DM_SBCS_SBACCESS64));
	riscv_print_info_line(CMD, "target", "memory.write_while_running64", get_field(info->sbcs, DM_SBCS_SBACCESS64));
	riscv_print_info_line(CMD, "target", "memory.read_while_running128", get_field(info->sbcs, DM_SBCS_SBACCESS128));
	riscv_print_info_line(CMD, "target", "memory.write_while_running128", get_field(info->sbcs, DM_SBCS_SBACCESS128));

	/* Lower level description. */
	riscv_print_info_line(CMD, "dm", "abits", info->abits);
	riscv_print_info_line(CMD, "dm", "progbufsize", info->progbufsize);
	riscv_print_info_line(CMD, "dm", "sbversion", get_field(info->sbcs, DM_SBCS_SBVERSION));
	riscv_print_info_line(CMD, "dm", "sbasize", get_field(info->sbcs, DM_SBCS_SBASIZE));
	riscv_print_info_line(CMD, "dm", "sbaccess128", get_field(info->sbcs, DM_SBCS_SBACCESS128));
	riscv_print_info_line(CMD, "dm", "sbaccess64", get_field(info->sbcs, DM_SBCS_SBACCESS64));
	riscv_print_info_line(CMD, "dm", "sbaccess32", get_field(info->sbcs, DM_SBCS_SBACCESS32));
	riscv_print_info_line(CMD, "dm", "sbaccess16", get_field(info->sbcs, DM_SBCS_SBACCESS16));
	riscv_print_info_line(CMD, "dm", "sbaccess8", get_field(info->sbcs, DM_SBCS_SBACCESS8));

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, false) == ERROR_OK)
		riscv_print_info_line(CMD, "dm", "authenticated", get_field(dmstatus, DM_DMSTATUS_AUTHENTICATED));

	return 0;
}

static int try_set_vsew(struct target *target, unsigned int *debug_vsew)
{
	RISCV_INFO(r);
	unsigned int encoded_vsew =
		(riscv_xlen(target) == 64 && r->vsew64_supported != YNM_NO) ? 3 : 2;

	/* Set standard element width to match XLEN, for vmv instruction to move
	 * the least significant bits into a GPR.
	 */
	if (riscv_reg_write(target, GDB_REGNO_VTYPE, encoded_vsew << 3) != ERROR_OK)
		return ERROR_FAIL;

	if (encoded_vsew == 3 && r->vsew64_supported == YNM_MAYBE) {
		/* Check that it's supported. */
		riscv_reg_t vtype;

		if (riscv_reg_get(target, &vtype, GDB_REGNO_VTYPE) != ERROR_OK)
			return ERROR_FAIL;
		if (vtype >> (riscv_xlen(target) - 1)) {
			r->vsew64_supported = YNM_NO;
			/* Try again. */
			return try_set_vsew(target, debug_vsew);
		}
		r->vsew64_supported = YNM_YES;
	}
	*debug_vsew = encoded_vsew == 3 ? 64 : 32;
	return ERROR_OK;
}

static int prep_for_vector_access(struct target *target,
		riscv_reg_t *orig_mstatus, riscv_reg_t *orig_vtype, riscv_reg_t *orig_vl,
		unsigned int *debug_vl, unsigned int *debug_vsew)
{
	assert(orig_mstatus);
	assert(orig_vtype);
	assert(orig_vl);
	assert(debug_vl);
	assert(debug_vsew);

	RISCV_INFO(r);
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target,
				"Unable to access vector register: target not halted");
		return ERROR_FAIL;
	}
	if (prep_for_register_access(target, orig_mstatus, GDB_REGNO_VL) != ERROR_OK)
		return ERROR_FAIL;

	/* Save vtype and vl. */
	if (riscv_reg_get(target, orig_vtype, GDB_REGNO_VTYPE) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_reg_get(target, orig_vl, GDB_REGNO_VL) != ERROR_OK)
		return ERROR_FAIL;

	if (try_set_vsew(target, debug_vsew) != ERROR_OK)
		return ERROR_FAIL;
	/* Set the number of elements to be updated with results from a vector
	 * instruction, for the vslide1down instruction.
	 * Set it so the entire V register is updated. */
	*debug_vl = DIV_ROUND_UP(r->vlenb * 8, *debug_vsew);
	return riscv_reg_write(target, GDB_REGNO_VL, *debug_vl);
}

static int cleanup_after_vector_access(struct target *target,
		riscv_reg_t mstatus, riscv_reg_t vtype, riscv_reg_t vl)
{
	/* Restore vtype and vl. */
	if (riscv_reg_write(target, GDB_REGNO_VTYPE, vtype) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_reg_write(target, GDB_REGNO_VL, vl) != ERROR_OK)
		return ERROR_FAIL;
	return cleanup_after_register_access(target, mstatus, GDB_REGNO_VL);
}

int riscv013_get_register_buf(struct target *target, uint8_t *value,
		enum gdb_regno regno)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_reg_t mstatus, vtype, vl;
	unsigned int debug_vl, debug_vsew;

	if (prep_for_vector_access(target, &mstatus, &vtype, &vl,
				&debug_vl, &debug_vsew) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	unsigned int vnum = regno - GDB_REGNO_V0;

	int result = ERROR_OK;
	for (unsigned int i = 0; i < debug_vl; i++) {
		/* Can't reuse the same program because riscv_program_exec() adds
		 * ebreak to the end every time. */
		struct riscv_program program;
		riscv_program_init(&program, target);
		riscv_program_insert(&program, vmv_x_s(S0, vnum));
		riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));

		/* Executing the program might result in an exception if there is some
		 * issue with the vector implementation/instructions we're using. If that
		 * happens, attempt to restore as usual. We may have clobbered the
		 * vector register we tried to read already.
		 * For other failures, we just return error because things are probably
		 * so messed up that attempting to restore isn't going to help. */
		result = riscv_program_exec(&program, target);
		if (result == ERROR_OK) {
			riscv_reg_t v;
			if (register_read_direct(target, &v, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u64(value, debug_vsew * i, debug_vsew, v);
		} else {
			LOG_TARGET_ERROR(target,
					"Failed to execute vmv/vslide1down while reading %s",
					riscv_reg_gdb_regno_name(target, regno));
			break;
		}
	}

	if (cleanup_after_vector_access(target, mstatus, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

int riscv013_set_register_buf(struct target *target, enum gdb_regno regno,
		const uint8_t *value)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_reg_t mstatus, vtype, vl;
	unsigned int debug_vl, debug_vsew;

	if (prep_for_vector_access(target, &mstatus, &vtype, &vl,
				&debug_vl, &debug_vsew) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	unsigned int vnum = regno - GDB_REGNO_V0;

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));
	int result = ERROR_OK;
	for (unsigned int i = 0; i < debug_vl; i++) {
		if (register_write_direct(target, GDB_REGNO_S0,
					buf_get_u64(value, debug_vsew * i, debug_vsew)) != ERROR_OK)
			return ERROR_FAIL;
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			break;
	}

	if (cleanup_after_vector_access(target, mstatus, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static uint32_t sb_sbaccess(unsigned int size_bytes)
{
	switch (size_bytes) {
		case 1:
			return set_field(0, DM_SBCS_SBACCESS, 0);
		case 2:
			return set_field(0, DM_SBCS_SBACCESS, 1);
		case 4:
			return set_field(0, DM_SBCS_SBACCESS, 2);
		case 8:
			return set_field(0, DM_SBCS_SBACCESS, 3);
		case 16:
			return set_field(0, DM_SBCS_SBACCESS, 4);
	}
	assert(0);
	return 0;
}

static unsigned int get_sbaadress_reg_count(const struct target *target)
{
	RISCV013_INFO(info);
	const unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	return DIV_ROUND_UP(sbasize, 32);
}

static void batch_fill_sb_write_address(const struct target *target,
		struct riscv_batch *batch, target_addr_t address,
		enum riscv_scan_delay_class sbaddr0_delay)
{
	/* There currently is no support for >64-bit addresses in OpenOCD. */
	assert(sizeof(target_addr_t) == sizeof(uint64_t));
	const uint32_t addresses[] = {DM_SBADDRESS0, DM_SBADDRESS1, DM_SBADDRESS2, DM_SBADDRESS3};
	const uint32_t values[] = {(uint32_t)address, (uint32_t)(address >> 32), 0, 0};
	const unsigned int reg_count = get_sbaadress_reg_count(target);
	assert(reg_count > 0);
	assert(reg_count <= ARRAY_SIZE(addresses));
	assert(ARRAY_SIZE(addresses) == ARRAY_SIZE(values));

	for (unsigned int i = reg_count - 1; i > 0; --i)
		riscv_batch_add_dm_write(batch, addresses[i], values[i], /* read back */ true,
				RISCV_DELAY_BASE);
	riscv_batch_add_dm_write(batch, addresses[0], values[0], /* read back */ true,
			sbaddr0_delay);
}

static int sb_write_address(struct target *target, target_addr_t address,
		enum riscv_scan_delay_class sbaddr0_delay)
{
	struct riscv_batch *batch = riscv_batch_alloc(target,
			get_sbaadress_reg_count(target));
	batch_fill_sb_write_address(target, batch, address, sbaddr0_delay);
	const int res = batch_run_timeout(target, batch);
	riscv_batch_free(batch);
	return res;
}

static int batch_run(struct target *target, struct riscv_batch *batch)
{
	RISCV_INFO(r);
	RISCV013_INFO(info);
	select_dmi(target);
	riscv_batch_add_nop(batch);
	const int result = riscv_batch_run_from(batch, 0, &info->learned_delays,
			/*resets_delays*/  r->reset_delays_wait >= 0,
			r->reset_delays_wait);
	if (result != ERROR_OK)
		return result;
	/* TODO: To use `riscv_batch_finished_scans()` here, it is needed for
	 * all scans to not discard input, meaning
	 * "riscv_batch_add_dm_write(..., false)" should not be used. */
	const size_t finished_scans = batch->used_scans;
	decrement_reset_delays_counter(target, finished_scans);
	if (riscv_batch_was_batch_busy(batch))
		return increase_dmi_busy_delay(target);
	return ERROR_OK;
}

/* It is expected that during creation of the batch
 * "riscv_batch_add_dm_write(..., false)" was not used.
 */
static int batch_run_timeout(struct target *target, struct riscv_batch *batch)
{
	RISCV013_INFO(info);
	select_dmi(target);
	riscv_batch_add_nop(batch);

	size_t finished_scans = 0;
	const time_t start = time(NULL);
	const unsigned int old_base_delay = riscv_scan_get_delay(&info->learned_delays,
			RISCV_DELAY_BASE);
	int result;
	do {
		RISCV_INFO(r);
		result = riscv_batch_run_from(batch, finished_scans,
				&info->learned_delays,
				/*resets_delays*/  r->reset_delays_wait >= 0,
				r->reset_delays_wait);
		if (result != ERROR_OK)
			return result;
		const size_t new_finished_scans = riscv_batch_finished_scans(batch);
		assert(new_finished_scans >= finished_scans);
		decrement_reset_delays_counter(target, new_finished_scans - finished_scans);
		finished_scans = new_finished_scans;
		if (!riscv_batch_was_batch_busy(batch)) {
			assert(finished_scans == batch->used_scans);
			return ERROR_OK;
		}
		result = increase_dmi_busy_delay(target);
		if (result != ERROR_OK)
			return result;
	} while (time(NULL) - start < riscv_get_command_timeout_sec());

	assert(result == ERROR_OK);
	assert(riscv_batch_was_batch_busy(batch));

	/* Reset dmi_busy_delay, so the value doesn't get too big. */
	LOG_TARGET_DEBUG(target, "%s delay is restored to %u.",
			riscv_scan_delay_class_name(RISCV_DELAY_BASE),
			old_base_delay);
	riscv_scan_set_delay(&info->learned_delays, RISCV_DELAY_BASE,
			old_base_delay);

	LOG_TARGET_ERROR(target, "DMI operation didn't complete in %d seconds. "
			"The target is either really slow or broken. You could increase "
			"the timeout with riscv set_command_timeout_sec.",
			riscv_get_command_timeout_sec());
	return ERROR_TIMEOUT_REACHED;
}

static int sba_supports_access(struct target *target, unsigned int size_bytes)
{
	RISCV013_INFO(info);
	switch (size_bytes) {
		case 1:
			return get_field(info->sbcs, DM_SBCS_SBACCESS8);
		case 2:
			return get_field(info->sbcs, DM_SBCS_SBACCESS16);
		case 4:
			return get_field(info->sbcs, DM_SBCS_SBACCESS32);
		case 8:
			return get_field(info->sbcs, DM_SBCS_SBACCESS64);
		case 16:
			return get_field(info->sbcs, DM_SBCS_SBACCESS128);
		default:
			return 0;
	}
}

static int sample_memory_bus_v1(struct target *target,
								struct riscv_sample_buf *buf,
								const riscv_sample_config_t *config,
								int64_t until_ms)
{
	RISCV013_INFO(info);
	unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	if (sbasize > 64) {
		LOG_TARGET_ERROR(target, "Memory sampling is only implemented for sbasize <= 64.");
		return ERROR_NOT_IMPLEMENTED;
	}

	if (get_field(info->sbcs, DM_SBCS_SBVERSION) != 1) {
		LOG_TARGET_ERROR(target, "Memory sampling is only implemented for SBA version 1.");
		return ERROR_NOT_IMPLEMENTED;
	}

	uint32_t sbcs = 0;
	uint32_t sbcs_valid = false;

	uint32_t sbaddress0 = 0;
	bool sbaddress0_valid = false;
	uint32_t sbaddress1 = 0;
	bool sbaddress1_valid = false;

	/* How often to read each value in a batch. */
	const unsigned int repeat = 5;

	unsigned int enabled_count = 0;
	for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
		if (config->bucket[i].enabled)
			enabled_count++;
	}

	while (timeval_ms() < until_ms) {
		/*
		 * batch_run() adds to the batch, so we can't simply reuse the same
		 * batch over and over. So we create a new one every time through the
		 * loop.
		 */
		struct riscv_batch *batch = riscv_batch_alloc(
			target, 1 + enabled_count * 5 * repeat);
		if (!batch)
			return ERROR_FAIL;

		unsigned int result_bytes = 0;
		for (unsigned int n = 0; n < repeat; n++) {
			for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
				if (config->bucket[i].enabled) {
					if (!sba_supports_access(target, config->bucket[i].size_bytes)) {
						LOG_TARGET_ERROR(target, "Hardware does not support SBA access for %d-byte memory sampling.",
								config->bucket[i].size_bytes);
						return ERROR_NOT_IMPLEMENTED;
					}

					uint32_t sbcs_write = DM_SBCS_SBREADONADDR;
					if (enabled_count == 1)
						sbcs_write |= DM_SBCS_SBREADONDATA;
					sbcs_write |= sb_sbaccess(config->bucket[i].size_bytes);
					if (!sbcs_valid || sbcs_write != sbcs) {
						riscv_batch_add_dm_write(batch, DM_SBCS, sbcs_write,
								true, RISCV_DELAY_BASE);
						sbcs = sbcs_write;
						sbcs_valid = true;
					}

					if (sbasize > 32 &&
							(!sbaddress1_valid ||
							sbaddress1 != config->bucket[i].address >> 32)) {
						sbaddress1 = config->bucket[i].address >> 32;
						riscv_batch_add_dm_write(batch, DM_SBADDRESS1,
								sbaddress1, true, RISCV_DELAY_BASE);
						sbaddress1_valid = true;
					}
					if (!sbaddress0_valid ||
							sbaddress0 != (config->bucket[i].address & 0xffffffff)) {
						sbaddress0 = config->bucket[i].address;
						riscv_batch_add_dm_write(batch, DM_SBADDRESS0,
								sbaddress0, true,
								RISCV_DELAY_SYSBUS_READ);
						sbaddress0_valid = true;
					}
					if (config->bucket[i].size_bytes > 4)
						riscv_batch_add_dm_read(batch, DM_SBDATA1,
								RISCV_DELAY_SYSBUS_READ);
					riscv_batch_add_dm_read(batch, DM_SBDATA0,
							RISCV_DELAY_SYSBUS_READ);
					result_bytes += 1 + config->bucket[i].size_bytes;
				}
			}
		}

		if (buf->used + result_bytes >= buf->size) {
			riscv_batch_free(batch);
			break;
		}

		size_t sbcs_read_index = riscv_batch_add_dm_read(batch, DM_SBCS,
				RISCV_DELAY_BASE);

		int result = batch_run(target, batch);
		if (result != ERROR_OK) {
			riscv_batch_free(batch);
			return result;
		}

		/* Discard the batch when we encounter a busy state on the DMI level.
		 * It's too much hassle to try to recover partial data. We'll try again
		 * with a larger DMI delay. */
		unsigned int sbcs_read_op = riscv_batch_get_dmi_read_op(batch, sbcs_read_index);
		if (sbcs_read_op == DTM_DMI_OP_BUSY) {
			result = increase_dmi_busy_delay(target);
			if (result != ERROR_OK) {
				riscv_batch_free(batch);
				return result;
			}
			continue;
		}

		uint32_t sbcs_read = riscv_batch_get_dmi_read_data(batch, sbcs_read_index);
		if (get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			/* Discard this batch when we encounter "busy error" state on the System Bus level.
			 * We'll try next time with a larger System Bus read delay. */
			dm_write(target, DM_SBCS, sbcs_read | DM_SBCS_SBBUSYERROR | DM_SBCS_SBERROR);
			int res = riscv_scan_increase_delay(&info->learned_delays,
					RISCV_DELAY_SYSBUS_READ);
			riscv_batch_free(batch);
			if (res != ERROR_OK)
				return res;
			continue;
		}
		if (get_field(sbcs_read, DM_SBCS_SBERROR)) {
			/* The memory we're sampling was unreadable, somehow. Give up. */
			dm_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR | DM_SBCS_SBERROR);
			riscv_batch_free(batch);
			return ERROR_FAIL;
		}

		unsigned int read_count = 0;
		for (unsigned int n = 0; n < repeat; n++) {
			for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
				if (config->bucket[i].enabled) {
					assert(i < RISCV_SAMPLE_BUF_TIMESTAMP_BEFORE);
					uint64_t value = 0;
					if (config->bucket[i].size_bytes > 4)
						value = ((uint64_t)riscv_batch_get_dmi_read_data(batch, read_count++)) << 32;
					value |= riscv_batch_get_dmi_read_data(batch, read_count++);

					buf->buf[buf->used] = i;
					buf_set_u64(buf->buf + buf->used + 1, 0, config->bucket[i].size_bytes * 8, value);
					buf->used += 1 + config->bucket[i].size_bytes;
				}
			}
		}

		riscv_batch_free(batch);
	}

	return ERROR_OK;
}

static int sample_memory(struct target *target,
						 struct riscv_sample_buf *buf,
						 riscv_sample_config_t *config,
						 int64_t until_ms)
{
	if (!config->enabled)
		return ERROR_OK;

	return sample_memory_bus_v1(target, buf, config, until_ms);
}

static int riscv013_get_hart_state(struct target *target, enum riscv_hart_state *state)
{
	RISCV013_INFO(info);
	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return ERROR_FAIL;
	if (get_field(dmstatus, DM_DMSTATUS_ANYHAVERESET)) {
		LOG_TARGET_INFO(target, "Hart unexpectedly reset!");
		info->dcsr_ebreak_is_set = false;
		/* TODO: Can we make this more obvious to eg. a gdb user? */
		uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_ACKHAVERESET;
		dmcontrol = set_dmcontrol_hartsel(dmcontrol, info->index);
		/* If we had been halted when we reset, request another halt. If we
		 * ended up running out of reset, then the user will (hopefully) get a
		 * message that a reset happened, that the target is running, and then
		 * that it is halted again once the request goes through.
		 */
		if (target->state == TARGET_HALTED) {
			dmcontrol |= DM_DMCONTROL_HALTREQ;
			/* `haltreq` should not be issued if `abstractcs.busy`
			 * is set. */
			int result = wait_for_idle_if_needed(target);
			if (result != ERROR_OK)
				return result;
		}
		dm_write(target, DM_DMCONTROL, dmcontrol);
	}
	if (get_field(dmstatus, DM_DMSTATUS_ALLNONEXISTENT)) {
		*state = RISCV_STATE_NON_EXISTENT;
		return ERROR_OK;
	}
	if (get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL)) {
		*state = RISCV_STATE_UNAVAILABLE;
		return ERROR_OK;
	}
	if (get_field(dmstatus, DM_DMSTATUS_ALLHALTED)) {
		*state = RISCV_STATE_HALTED;
		return ERROR_OK;
	}
	if (get_field(dmstatus, DM_DMSTATUS_ALLRUNNING)) {
		*state = RISCV_STATE_RUNNING;
		return ERROR_OK;
	}
	LOG_TARGET_ERROR(target, "Couldn't determine state. dmstatus=0x%x", dmstatus);
	return ERROR_FAIL;
}

static int handle_became_unavailable(struct target *target,
		enum riscv_hart_state previous_riscv_state)
{
	RISCV013_INFO(info);
	info->dcsr_ebreak_is_set = false;
	return ERROR_OK;
}

static int tick(struct target *target)
{
	RISCV013_INFO(info);
	if (!info->dcsr_ebreak_is_set &&
			target->state == TARGET_RUNNING &&
			target_was_examined(target))
		return halt_set_dcsr_ebreak(target);
	return ERROR_OK;
}

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_TARGET_DEBUG(target, "Init.");
	RISCV_INFO(generic_info);

	generic_info->select_target = &dm013_select_target;
	generic_info->get_hart_state = &riscv013_get_hart_state;
	generic_info->resume_go = &riscv013_resume_go;
	generic_info->step_current_hart = &riscv013_step_current_hart;
	generic_info->resume_prep = &riscv013_resume_prep;
	generic_info->halt_prep = &riscv013_halt_prep;
	generic_info->halt_go = &riscv013_halt_go;
	generic_info->on_step = &riscv013_on_step;
	generic_info->halt_reason = &riscv013_halt_reason;
	generic_info->read_progbuf = &riscv013_read_progbuf;
	generic_info->write_progbuf = &riscv013_write_progbuf;
	generic_info->execute_progbuf = &riscv013_execute_progbuf;
	generic_info->invalidate_cached_progbuf = &riscv013_invalidate_cached_progbuf;
	generic_info->fill_dmi_write = &riscv013_fill_dmi_write;
	generic_info->fill_dmi_read = &riscv013_fill_dmi_read;
	generic_info->fill_dm_nop = &riscv013_fill_dm_nop;
	generic_info->get_dmi_scan_length = &riscv013_get_dmi_scan_length;
	generic_info->authdata_read = &riscv013_authdata_read;
	generic_info->authdata_write = &riscv013_authdata_write;
	generic_info->dmi_read = &dmi_read;
	generic_info->dmi_write = &dmi_write;
	generic_info->get_dmi_address = &riscv013_get_dmi_address;
	generic_info->read_memory = read_memory;
	generic_info->data_bits = &riscv013_data_bits;
	generic_info->print_info = &riscv013_print_info;

	generic_info->handle_became_unavailable = &handle_became_unavailable;
	generic_info->tick = &tick;

	if (!generic_info->version_specific) {
		generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
		if (!generic_info->version_specific)
			return ERROR_FAIL;
	}
	generic_info->sample_memory = sample_memory;
	riscv013_info_t *info = get_info(target);

	info->progbufsize = -1;
	reset_learned_delays(target);

	/* Assume all these abstract commands are supported until we learn
	 * otherwise.
	 * TODO: The spec allows eg. one CSR to be able to be accessed abstractly
	 * while another one isn't. We don't track that this closely here, but in
	 * the future we probably should. */
	info->abstract_read_csr_supported = true;
	info->abstract_write_csr_supported = true;
	info->abstract_read_fpr_supported = true;
	info->abstract_write_fpr_supported = true;

	info->has_aampostincrement = YNM_MAYBE;

	return ERROR_OK;
}

static int assert_reset(struct target *target)
{
	RISCV013_INFO(info);
	int result;

	select_dmi(target);

	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		/* Run the user-supplied script if there is one. */
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	} else {
		dm013_info_t *dm = get_dm(target);
		if (!dm)
			return ERROR_FAIL;

		uint32_t control = set_field(0, DM_DMCONTROL_DMACTIVE, 1);
		control = set_dmcontrol_hartsel(control, info->index);
		control = set_field(control, DM_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
		/* If `abstractcs.busy` is set, debugger should not
		 * change `hartsel` or set `haltreq`
		 */
		const bool hartsel_changed = (int)info->index != dm->current_hartid;
		if (hartsel_changed || target->reset_halt) {
			result = wait_for_idle_if_needed(target);
			if (result != ERROR_OK)
				return result;
		}
		result = dm_write(target, DM_DMCONTROL, control);
		if (result != ERROR_OK)
			return result;
	}

	target->state = TARGET_RESET;

	/* The DM might have gotten reset if OpenOCD called us in some reset that
	 * involves SRST being toggled. So clear our cache which may be out of
	 * date. */
	return riscv013_invalidate_cached_progbuf(target);
}

static int deassert_reset(struct target *target)
{
	RISCV013_INFO(info);
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	int result;

	select_dmi(target);
	/* Clear the reset, but make sure haltreq is still set */
	uint32_t control = 0;
	control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);
	control = set_field(control, DM_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
	control = set_dmcontrol_hartsel(control, info->index);
	/* If `abstractcs.busy` is set, debugger should not
	 * change `hartsel`.
	 */
	const bool hartsel_changed = (int)info->index != dm->current_hartid;
	if (hartsel_changed) {
		result = wait_for_idle_if_needed(target);
		if (result != ERROR_OK)
			return result;
	}
	result = dm_write(target, DM_DMCONTROL, control);
	if (result != ERROR_OK)
		return result;

	uint32_t dmstatus;
	const unsigned int orig_base_delay = riscv_scan_get_delay(&info->learned_delays,
			RISCV_DELAY_BASE);
	time_t start = time(NULL);
	LOG_TARGET_DEBUG(target, "Waiting for hart to come out of reset.");
	do {
		result = dmstatus_read(target, &dmstatus, true);
		if (result != ERROR_OK)
			return result;

		if (time(NULL) - start > riscv_get_command_timeout_sec()) {
			LOG_TARGET_ERROR(target, "Hart didn't leave reset in %ds; "
					"dmstatus=0x%x (allunavail=%s, allhavereset=%s); "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_get_command_timeout_sec(), dmstatus,
					get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL) ? "true" : "false",
					get_field(dmstatus, DM_DMSTATUS_ALLHAVERESET) ? "true" : "false");
			return ERROR_TIMEOUT_REACHED;
		}
		/* Certain debug modules, like the one in GD32VF103
		 * MCUs, violate the specification's requirement that
		 * each hart is in "exactly one of four states" and,
		 * during reset, report harts as both unavailable and
		 * halted/running. To work around this, we check for
		 * the absence of the unavailable state rather than
		 * the presence of any other state. */
	} while (get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL) &&
			!get_field(dmstatus, DM_DMSTATUS_ALLHAVERESET));

	riscv_scan_set_delay(&info->learned_delays, RISCV_DELAY_BASE,
			orig_base_delay);

	if (target->reset_halt) {
		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_DBGRQ;
	} else {
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
	}
	info->dcsr_ebreak_is_set = false;

	/* Ack reset and clear DM_DMCONTROL_HALTREQ if previously set */
	control = 0;
	control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);
	control = set_field(control, DM_DMCONTROL_ACKHAVERESET, 1);
	control = set_dmcontrol_hartsel(control, info->index);
	return dm_write(target, DM_DMCONTROL, control);
}

static int execute_fence(struct target *target)
{
	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	/* FIXME: For non-coherent systems we need to flush the caches right
	 * here, but there's no ISA-defined way of doing that. */
	struct riscv_program program;

	/* program.execution_result may indicate RISCV_PROGBUF_EXEC_RESULT_EXCEPTION -
	 * currently, we ignore this error since most likely this is an indication
	 * that target does not support a fence instruction (execution of an
	 * unsupported instruction results in "Illegal instruction" exception on
	 * targets that comply with riscv-privilege spec).
	 * Currently, RISC-V specification does not provide us with a portable and
	 * less invasive way to detect if a fence is supported by the target. We may
	 * revise this code once the spec allows us to do this */
	if (has_sufficient_progbuf(target, 3)) {
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		riscv_program_fence_rw_rw(&program);
		if (riscv_program_exec(&program, target) != ERROR_OK) {
			if (program.execution_result != RISCV_PROGBUF_EXEC_RESULT_EXCEPTION) {
				LOG_TARGET_ERROR(target, "Unexpected error during fence execution");
				return ERROR_FAIL;
			}
			LOG_TARGET_DEBUG(target, "Unable to execute fence");
		}
		return ERROR_OK;
	}

	if (has_sufficient_progbuf(target, 2)) {
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		if (riscv_program_exec(&program, target) != ERROR_OK) {
			if (program.execution_result != RISCV_PROGBUF_EXEC_RESULT_EXCEPTION) {
				LOG_TARGET_ERROR(target, "Unexpected error during fence.i execution");
				return ERROR_FAIL;
			}
			LOG_TARGET_DEBUG(target, "Unable to execute fence.i");
		}

		riscv_program_init(&program, target);
		riscv_program_fence_rw_rw(&program);
		if (riscv_program_exec(&program, target) != ERROR_OK) {
			if (program.execution_result != RISCV_PROGBUF_EXEC_RESULT_EXCEPTION) {
				LOG_TARGET_ERROR(target, "Unexpected error during fence rw, rw execution");
				return ERROR_FAIL;
			}
			LOG_TARGET_DEBUG(target, "Unable to execute fence rw, rw");
		}
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static void log_memory_access128(target_addr_t address, uint64_t value_h,
		uint64_t value_l, bool is_read)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char fmt[80];
	sprintf(fmt, "M[0x%" TARGET_PRIxADDR "] %ss 0x%%016" PRIx64 "%%016" PRIx64,
			address, is_read ? "read" : "write");
	LOG_DEBUG(fmt, value_h, value_l);
}

static void log_memory_access64(target_addr_t address, uint64_t value,
		unsigned int size_bytes, bool is_read)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char fmt[80];
	sprintf(fmt, "M[0x%" TARGET_PRIxADDR "] %ss 0x%%0%d" PRIx64,
			address, is_read ? "read" : "write", size_bytes * 2);
	switch (size_bytes) {
		case 1:
			value &= 0xff;
			break;
		case 2:
			value &= 0xffff;
			break;
		case 4:
			value &= 0xffffffffUL;
			break;
		case 8:
			break;
		default:
			assert(false);
	}
	LOG_DEBUG(fmt, value);
}
static void log_memory_access(target_addr_t address, uint32_t *sbvalue,
		unsigned int size_bytes, bool is_read)
{
	if (size_bytes == 16) {
		uint64_t value_h = ((uint64_t)sbvalue[3] << 32) | sbvalue[2];
		uint64_t value_l = ((uint64_t)sbvalue[1] << 32) | sbvalue[0];
		log_memory_access128(address, value_h, value_l, is_read);
	} else {
		uint64_t value = ((uint64_t)sbvalue[1] << 32) | sbvalue[0];
		log_memory_access64(address, value, size_bytes, is_read);
	}
}

/* Read the relevant sbdata regs depending on size, and put the results into
 * buffer. */
static int read_memory_bus_word(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	int result;
	uint32_t sbvalue[4] = { 0 };
	static int sbdata[4] = { DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3 };
	assert(size <= 16);
	for (int i = (size - 1) / 4; i >= 0; i--) {
		result = dm_read(target, &sbvalue[i], sbdata[i]);
		if (result != ERROR_OK)
			return result;
		buf_set_u32(buffer + i * 4, 0, 8 * MIN(size, 4), sbvalue[i]);
	}
	log_memory_access(address, sbvalue, size, true);
	return ERROR_OK;
}

static target_addr_t sb_read_address(struct target *target)
{
	RISCV013_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	target_addr_t address = 0;
	uint32_t v;
	if (sbasize > 32) {
		if (dm_read(target, &v, DM_SBADDRESS1) == ERROR_OK)
			address |= v;
		address <<= 32;
	}
	if (dm_read(target, &v, DM_SBADDRESS0) == ERROR_OK)
		address |= v;
	return address;
}

static int read_sbcs_nonbusy(struct target *target, uint32_t *sbcs)
{
	time_t start = time(NULL);
	while (1) {
		if (dm_read(target, sbcs, DM_SBCS) != ERROR_OK)
			return ERROR_FAIL;
		if (!get_field(*sbcs, DM_SBCS_SBBUSY))
			return ERROR_OK;
		if (time(NULL) - start > riscv_get_command_timeout_sec()) {
			LOG_TARGET_ERROR(target, "Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_get_command_timeout_sec(), *sbcs);
			return ERROR_FAIL;
		}
	}
}

static int modify_privilege(struct target *target, uint64_t *mstatus, uint64_t *mstatus_old)
{
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5)) {
		/* Read DCSR */
		uint64_t dcsr;
		if (register_read_direct(target, &dcsr, GDB_REGNO_DCSR) != ERROR_OK)
			return ERROR_FAIL;

		/* Read and save MSTATUS */
		if (register_read_direct(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		*mstatus_old = *mstatus;

		/* If we come from m-mode with mprv set, we want to keep mpp */
		if (get_field(dcsr, CSR_DCSR_PRV) < 3) {
			/* MPP = PRIV */
			*mstatus = set_field(*mstatus, MSTATUS_MPP, get_field(dcsr, CSR_DCSR_PRV));

			/* MPRV = 1 */
			*mstatus = set_field(*mstatus, MSTATUS_MPRV, 1);

			/* Write MSTATUS */
			if (*mstatus != *mstatus_old)
				if (register_write_direct(target, GDB_REGNO_MSTATUS, *mstatus) != ERROR_OK)
					return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int read_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (size != increment) {
		LOG_TARGET_ERROR(target, "sba v0 reads only support size==increment");
		return ERROR_NOT_IMPLEMENTED;
	}

	LOG_TARGET_DEBUG(target, "System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	uint8_t *t_buffer = buffer;
	riscv_addr_t cur_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	uint32_t access = 0;

	const int DM_SBCS_SBSINGLEREAD_OFFSET = 20;
	const uint32_t DM_SBCS_SBSINGLEREAD = (0x1U << DM_SBCS_SBSINGLEREAD_OFFSET);

	const int DM_SBCS_SBAUTOREAD_OFFSET = 15;
	const uint32_t DM_SBCS_SBAUTOREAD = (0x1U << DM_SBCS_SBAUTOREAD_OFFSET);

	/* ww favorise one off reading if there is an issue */
	if (count == 1) {
		for (uint32_t i = 0; i < count; i++) {
			if (dm_read(target, &access, DM_SBCS) != ERROR_OK)
				return ERROR_FAIL;
			dm_write(target, DM_SBADDRESS0, cur_addr);
			/* size/2 matching the bit access of the spec 0.13 */
			access = set_field(access, DM_SBCS_SBACCESS, size/2);
			access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
			LOG_TARGET_DEBUG(target, "read_memory: sab: access:  0x%08x", access);
			dm_write(target, DM_SBCS, access);
			/* 3) read */
			uint32_t value;
			if (dm_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			LOG_TARGET_DEBUG(target, "read_memory: sab: value:  0x%08x", value);
			buf_set_u32(t_buffer, 0, 8 * size, value);
			t_buffer += size;
			cur_addr += size;
		}
		return ERROR_OK;
	}

	/* has to be the same size if we want to read a block */
	LOG_TARGET_DEBUG(target, "Reading block until final address 0x%" PRIx64, fin_addr);
	if (dm_read(target, &access, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;
	/* set current address */
	dm_write(target, DM_SBADDRESS0, cur_addr);
	/* 2) write sbaccess=2, sbsingleread,sbautoread,sbautoincrement
	 * size/2 matching the bit access of the spec 0.13 */
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOREAD, 1);
	access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_TARGET_DEBUG(target, "access:  0x%08x", access);
	dm_write(target, DM_SBCS, access);

	while (cur_addr < fin_addr) {
		LOG_TARGET_DEBUG(target, "sab:autoincrement:\r\n\tsize: %d\tcount:%d\taddress: 0x%08"
				PRIx64, size, count, cur_addr);
		/* read */
		uint32_t value;
		if (dm_read(target, &value, DM_SBDATA0) != ERROR_OK)
			return ERROR_FAIL;
		buf_set_u32(t_buffer, 0, 8 * size, value);
		cur_addr += size;
		t_buffer += size;

		/* if we are reaching last address, we must clear autoread */
		if (cur_addr == fin_addr && count != 1) {
			dm_write(target, DM_SBCS, 0);
			if (dm_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u32(t_buffer, 0, 8 * size, value);
		}
	}

	uint32_t sbcs;
	if (dm_read(target, &sbcs, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * Read the requested memory using the system bus interface.
 */
static int read_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (increment != size && increment != 0) {
		LOG_TARGET_ERROR(target, "sba v1 reads only support increment of size or 0");
		return ERROR_NOT_IMPLEMENTED;
	}

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	RISCV013_INFO(info);
	target_addr_t next_address = address;
	target_addr_t end_address = address + (increment ? count : 1) * size;

	/* TODO: Reading all the elements in a single batch will boost the
	 * performance.
	 */
	while (next_address < end_address) {
		uint32_t sbcs_write = set_field(0, DM_SBCS_SBREADONADDR, 1);
		sbcs_write |= sb_sbaccess(size);
		if (increment == size)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBAUTOINCREMENT, 1);
		if (count > 1)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, count > 1);
		if (dm_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
			return ERROR_FAIL;

		/* This address write will trigger the first read. */
		if (sb_write_address(target, next_address, RISCV_DELAY_SYSBUS_READ) != ERROR_OK)
			return ERROR_FAIL;

		/* First read has been started. Optimistically assume that it has
		 * completed. */

		static int sbdata[4] = {DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3};
		/* TODO: The only purpose of "sbvalue" is to be passed to
		 * "log_memory_access()".  If "log_memory_access()" were to
		 * accept "uint8_t *" instead of "uint32_t *", "sbvalue" would
		 * be unnecessary.
		 */
		uint32_t sbvalue[4] = {0};
		assert(size <= 16);
		for (uint32_t i = (next_address - address) / size; i < count - 1; i++) {
			const uint32_t size_in_words = DIV_ROUND_UP(size, 4);
			struct riscv_batch *batch = riscv_batch_alloc(target, size_in_words);
			/* Read of sbdata0 must be performed as last because it
			 * starts the new bus data transfer
			 * (in case "sbcs.sbreadondata" was set above).
			 * We don't want to start the next bus read before we
			 * fetch all the data from the last bus read. */
			for (uint32_t j = size_in_words - 1; j > 0; --j)
				riscv_batch_add_dm_read(batch, sbdata[j], RISCV_DELAY_BASE);
			riscv_batch_add_dm_read(batch, sbdata[0], RISCV_DELAY_SYSBUS_READ);

			int res = batch_run_timeout(target, batch);
			if (res != ERROR_OK) {
				riscv_batch_free(batch);
				return res;
			}

			const size_t last_key = batch->read_keys_used - 1;
			for (size_t k = 0; k <= last_key; ++k) {
				sbvalue[k] = riscv_batch_get_dmi_read_data(batch,
						last_key - k);
				buf_set_u32(buffer + i * size + k * 4, 0, 8 * size, sbvalue[k]);
			}
			riscv_batch_free(batch);
			const target_addr_t read_addr = address + i * increment;
			log_memory_access(read_addr, sbvalue, size, true);
		}

		uint32_t sbcs_read = 0;
		if (count > 1) {
			/* "Writes to sbcs while sbbusy is high result in undefined behavior.
			 * A debugger must not write to sbcs until it reads sbbusy as 0." */
			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;

			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, 0);
			if (dm_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
				return ERROR_FAIL;
		}

		/* Read the last word, after we disabled sbreadondata if necessary. */
		if (!get_field(sbcs_read, DM_SBCS_SBERROR) &&
				!get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			if (read_memory_bus_word(target, address + (count - 1) * increment, size,
						buffer + (count - 1) * size) != ERROR_OK)
				return ERROR_FAIL;

			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again.
			 * Clear sbbusyerror, as well as readondata or readonaddr. */
			if (dm_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR) != ERROR_OK)
				return ERROR_FAIL;

			if (get_field(sbcs_read, DM_SBCS_SBERROR) == DM_SBCS_SBERROR_NONE) {
				/* Read the address whose read was last completed. */
				next_address = sb_read_address(target);

				/* Read the value for the last address. It's
				 * sitting in the register for us, but we read it
				 * too early (sbbusyerror became set). */
				target_addr_t current_address = next_address - (increment ? size : 0);
				if (read_memory_bus_word(target, current_address, size,
							buffer + current_address - address) != ERROR_OK)
					return ERROR_FAIL;
			}

			int res = riscv_scan_increase_delay(&info->learned_delays,
					RISCV_DELAY_SYSBUS_READ);
			if (res != ERROR_OK)
				return res;
			continue;
		}

		unsigned error = get_field(sbcs_read, DM_SBCS_SBERROR);
		if (error == DM_SBCS_SBERROR_NONE) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			if (dm_write(target, DM_SBCS, DM_SBCS_SBERROR) != ERROR_OK)
				return ERROR_FAIL;
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static void log_mem_access_result(struct target *target, bool success, int method, bool is_read)
{
	RISCV_INFO(r);
	bool warn = false;
	char msg[60];

	/* Compose the message */
	snprintf(msg, 60, "%s to %s memory via %s.",
			success ? "Succeeded" : "Failed",
			is_read ? "read" : "write",
			(method == RISCV_MEM_ACCESS_PROGBUF) ? "program buffer" :
			(method == RISCV_MEM_ACCESS_SYSBUS) ? "system bus" : "abstract access");

	/* Determine the log message severity. Show warnings only once. */
	if (!success) {
		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			warn = r->mem_access_progbuf_warn;
			r->mem_access_progbuf_warn = false;
		}
		if (method == RISCV_MEM_ACCESS_SYSBUS) {
			warn = r->mem_access_sysbus_warn;
			r->mem_access_sysbus_warn = false;
		}
		if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			warn = r->mem_access_abstract_warn;
			r->mem_access_abstract_warn = false;
		}
	}

	if (warn)
		LOG_TARGET_WARNING(target, "%s", msg);
	else
		LOG_TARGET_DEBUG(target, "%s", msg);
}

static bool mem_should_skip_progbuf(struct target *target, target_addr_t address,
		uint32_t size, bool is_read, char **skip_reason)
{
	assert(skip_reason);

	if (!has_sufficient_progbuf(target, 3)) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via progbuf - insufficient progbuf size.",
				is_read ? "read" : "write");
		*skip_reason = "skipped (insufficient progbuf)";
		return true;
	}
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via progbuf - target not halted.",
				is_read ? "read" : "write");
		*skip_reason = "skipped (target not halted)";
		return true;
	}
	if (riscv_xlen(target) < size * 8) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via progbuf - XLEN (%d) is too short for %d-bit memory access.",
				is_read ? "read" : "write", riscv_xlen(target), size * 8);
		*skip_reason = "skipped (XLEN too short)";
		return true;
	}
	if (size > 8) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via progbuf - unsupported size.",
				is_read ? "read" : "write");
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	if ((sizeof(address) * 8 > riscv_xlen(target)) && (address >> riscv_xlen(target))) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via progbuf - progbuf only supports %u-bit address.",
				is_read ? "read" : "write", riscv_xlen(target));
		*skip_reason = "skipped (too large address)";
		return true;
	}

	return false;
}

static bool mem_should_skip_sysbus(struct target *target, target_addr_t address,
		uint32_t size, uint32_t increment, bool is_read, char **skip_reason)
{
	assert(skip_reason);

	RISCV013_INFO(info);
	if (!sba_supports_access(target, size)) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via system bus - unsupported size.",
				is_read ? "read" : "write");
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	if ((sizeof(address) * 8 > sbasize) && (address >> sbasize)) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via system bus - sba only supports %u-bit address.",
				is_read ? "read" : "write", sbasize);
		*skip_reason = "skipped (too large address)";
		return true;
	}
	if (is_read && increment != size && (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0 || increment != 0)) {
		LOG_TARGET_DEBUG(target, "Skipping mem read via system bus - "
				"sba reads only support size==increment or also size==0 for sba v1.");
		*skip_reason = "skipped (unsupported increment)";
		return true;
	}

	return false;
}

static bool mem_should_skip_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t increment, bool is_read, char **skip_reason)
{
	assert(skip_reason);

	if (size > 8) {
		/* TODO: Add 128b support if it's ever used. Involves modifying
				 read/write_abstract_arg() to work on two 64b values. */
		LOG_TARGET_DEBUG(target, "Skipping mem %s via abstract access - unsupported size: %d bits",
				is_read ? "read" : "write", size * 8);
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	if ((sizeof(address) * 8 > riscv_xlen(target)) && (address >> riscv_xlen(target))) {
		LOG_TARGET_DEBUG(target, "Skipping mem %s via abstract access - abstract access only supports %u-bit address.",
				is_read ? "read" : "write", riscv_xlen(target));
		*skip_reason = "skipped (too large address)";
		return true;
	}
	if (is_read && size != increment) {
		LOG_TARGET_ERROR(target, "Skipping mem read via abstract access - "
				"abstract command reads only support size==increment.");
		*skip_reason = "skipped (unsupported increment)";
		return true;
	}

	return false;
}

/*
 * Performs a memory read using memory access abstract commands. The read sizes
 * supported are 1, 2, and 4 bytes despite the spec's support of 8 and 16 byte
 * aamsize fields in the memory access abstract command.
 */
static int read_memory_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	RISCV013_INFO(info);

	int result = ERROR_OK;
	bool use_aampostincrement = info->has_aampostincrement != YNM_NO;

	LOG_TARGET_DEBUG(target, "Reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	memset(buffer, 0, count * size);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;

	/* Create the command (physical address, postincrement, read) */
	uint32_t command = access_memory_command(target, false, width, use_aampostincrement, false);

	/* Execute the reads */
	uint8_t *p = buffer;
	bool updateaddr = true;
	unsigned int width32 = (width < 32) ? 32 : width;
	for (uint32_t c = 0; c < count; c++) {
		/* Update the address if it is the first time or aampostincrement is not supported by the target. */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address + c * size, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to write arg1.");
				return result;
			}
		}

		/* Execute the command */
		uint32_t cmderr;
		result = execute_abstract_command(target, command, &cmderr);

		/* TODO: we need to modify error handling here. */
		/* NOTE: in case of timeout cmderr is set to CMDERR_NONE */
		if (info->has_aampostincrement == YNM_MAYBE) {
			if (result == ERROR_OK) {
				/* Safety: double-check that the address was really auto-incremented */
				riscv_reg_t new_address;
				result = read_abstract_arg(target, &new_address, 1, riscv_xlen(target));
				if (result != ERROR_OK)
					return result;

				if (new_address == address + size) {
					LOG_TARGET_DEBUG(target, "aampostincrement is supported on this target.");
					info->has_aampostincrement = YNM_YES;
				} else {
					LOG_TARGET_WARNING(target, "Buggy aampostincrement! Address not incremented correctly.");
					info->has_aampostincrement = YNM_NO;
				}
			} else {
				/* Try the same access but with postincrement disabled. */
				command = access_memory_command(target, false, width, false, false);
				result = execute_abstract_command(target, command, &cmderr);
				if (result == ERROR_OK) {
					LOG_TARGET_DEBUG(target, "aampostincrement is not supported on this target.");
					info->has_aampostincrement = YNM_NO;
				}
			}
		}

		if (result != ERROR_OK)
			return result;

		/* Copy arg0 to buffer (rounded width up to nearest 32) */
		riscv_reg_t value;
		result = read_abstract_arg(target, &value, 0, width32);
		if (result != ERROR_OK)
			return result;
		buf_set_u64(p, 0, 8 * size, value);

		if (info->has_aampostincrement == YNM_YES)
			updateaddr = false;
		p += size;
	}

	return result;
}

/*
 * Performs a memory write using memory access abstract commands. The write
 * sizes supported are 1, 2, and 4 bytes despite the spec's support of 8 and 16
 * byte aamsize fields in the memory access abstract command.
 */
static int write_memory_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);
	int result = ERROR_OK;
	bool use_aampostincrement = info->has_aampostincrement != YNM_NO;

	LOG_TARGET_DEBUG(target, "writing %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;

	/* Create the command (physical address, postincrement, write) */
	uint32_t command = access_memory_command(target, false, width, use_aampostincrement, true);

	/* Execute the writes */
	const uint8_t *p = buffer;
	bool updateaddr = true;
	for (uint32_t c = 0; c < count; c++) {
		/* Move data to arg0 */
		riscv_reg_t value = buf_get_u64(p, 0, 8 * size);
		result = write_abstract_arg(target, 0, value, riscv_xlen(target));
		if (result != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to write arg0.");
			return result;
		}

		/* Update the address if it is the first time or aampostincrement is not supported by the target. */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address + c * size, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to write arg1.");
				return result;
			}
		}

		/* Execute the command */
		uint32_t cmderr;
		result = execute_abstract_command(target, command, &cmderr);

		/* TODO: we need to modify error handling here. */
		/* NOTE: in case of timeout cmderr is set to CMDERR_NONE */
		if (info->has_aampostincrement == YNM_MAYBE) {
			if (result == ERROR_OK) {
				/* Safety: double-check that the address was really auto-incremented */
				riscv_reg_t new_address;
				result = read_abstract_arg(target, &new_address, 1, riscv_xlen(target));
				if (result != ERROR_OK)
					return result;

				if (new_address == address + size) {
					LOG_TARGET_DEBUG(target, "aampostincrement is supported on this target.");
					info->has_aampostincrement = YNM_YES;
				} else {
					LOG_TARGET_WARNING(target, "Buggy aampostincrement! Address not incremented correctly.");
					info->has_aampostincrement = YNM_NO;
				}
			} else {
				/* Try the same access but with postincrement disabled. */
				command = access_memory_command(target, false, width, false, true);
				result = execute_abstract_command(target, command, &cmderr);
				if (result == ERROR_OK) {
					LOG_TARGET_DEBUG(target, "aampostincrement is not supported on this target.");
					info->has_aampostincrement = YNM_NO;
				}
			}
		}

		if (result != ERROR_OK)
			return result;

		if (info->has_aampostincrement == YNM_YES)
			updateaddr = false;
		p += size;
	}

	return result;
}

/**
 * This function is used to start the memory-reading pipeline.
 * The pipeline looks like this:
 * memory -> s1 -> dm_data[0:1] -> debugger
 * Prior to calling it, the program buffer should contain the appropriate
 * program.
 * This function sets DM_ABSTRACTAUTO_AUTOEXECDATA to trigger second stage of the
 * pipeline (s1 -> dm_data[0:1]) whenever dm_data is read.
 */
static int read_memory_progbuf_inner_startup(struct target *target,
		target_addr_t address, uint32_t increment, uint32_t index)
{
	/* s0 holds the next address to read from.
	 * s1 holds the next data value read.
	 * a0 is a counter in case increment is 0.
	 */
	if (register_write_direct(target, GDB_REGNO_S0, address + index * increment)
			!= ERROR_OK)
		return ERROR_FAIL;

	if (/*is_repeated_read*/ increment == 0 &&
			register_write_direct(target, GDB_REGNO_A0, index) != ERROR_OK)
		return ERROR_FAIL;

	/* AC_ACCESS_REGISTER_POSTEXEC is used to trigger first stage of the
	 * pipeline (memory -> s1) whenever this command is executed.
	 */
	const uint32_t startup_command = access_register_command(target,
			GDB_REGNO_S1, riscv_xlen(target),
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	uint32_t cmderr;
	if (execute_abstract_command(target, startup_command, &cmderr) != ERROR_OK)
		return ERROR_FAIL;
	/* TODO: we need to modify error handling here. */
	/* NOTE: in case of timeout cmderr is set to CMDERR_NONE */

	/* First read has just triggered. Result is in s1.
	 * dm_data registers contain the previous value of s1 (garbage).
	 */
	if (dm_write(target, DM_ABSTRACTAUTO,
				set_field(0, DM_ABSTRACTAUTO_AUTOEXECDATA, 1)) != ERROR_OK)
		return ERROR_FAIL;

	/* Read garbage from dm_data0, which triggers another execution of the
	 * program. Now dm_data contains the first good result (from s1),
	 * and s1 the next memory value.
	 */
	if (dm_read_exec(target, NULL, DM_DATA0) != ERROR_OK)
		goto clear_abstractauto_and_fail;

	uint32_t abstractcs;
	if (wait_for_idle(target, &abstractcs) != ERROR_OK)
		goto clear_abstractauto_and_fail;

	cmderr = get_field32(abstractcs, DM_ABSTRACTCS_CMDERR);
	switch (cmderr) {
	case CMDERR_NONE:
		return ERROR_OK;
	case CMDERR_BUSY:
		LOG_TARGET_ERROR(target, "Unexpected busy error. This is probably a hardware bug.");
		/* fall through */
	default:
		LOG_TARGET_DEBUG(target, "error when reading memory, cmderr=0x%" PRIx32, cmderr);
		riscv013_clear_abstract_error(target);
		goto clear_abstractauto_and_fail;
	}
clear_abstractauto_and_fail:
	dm_write(target, DM_ABSTRACTAUTO, 0);
	return ERROR_FAIL;
}

struct memory_access_info {
	uint8_t *buffer_address;
	target_addr_t target_address;
	uint32_t element_size;
	uint32_t increment;
};

/**
 * This function attempts to restore the pipeline after a busy on abstract
 * access.
 * Target's state is as follows:
 * s0 contains address + index_on_target * increment
 * s1 contains mem[address + (index_on_target - 1) * increment]
 * dm_data[0:1] contains mem[address + (index_on_target - 2) * increment]
 */
static int read_memory_progbuf_inner_on_ac_busy(struct target *target,
		uint32_t start_index, uint32_t *elements_read,
		struct memory_access_info access)
{
	int res = riscv013_clear_abstract_error(target);
	if (res != ERROR_OK)
		return res;
	res = increase_ac_busy_delay(target);
	if (res != ERROR_OK)
		return res;

	if (dm_write(target, DM_ABSTRACTAUTO, 0) != ERROR_OK)
		return ERROR_FAIL;

	/* See how far we got by reading s0/a0 */
	uint32_t index_on_target;

	if (/*is_repeated_read*/ access.increment == 0) {
		/* s0 is constant, a0 is incremented by one each execution */
		riscv_reg_t counter;

		if (register_read_direct(target, &counter, GDB_REGNO_A0) != ERROR_OK)
			return ERROR_FAIL;
		index_on_target = counter;
	} else {
		target_addr_t address_on_target;

		if (register_read_direct(target, &address_on_target, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;
		index_on_target = (address_on_target - access.target_address) /
			access.increment;
	}

	/* According to the spec, if an abstract command fails, one can't make any
	 * assumptions about dm_data registers, so all the values in the pipeline
	 * are clobbered now and need to be reread.
	 */
	const uint32_t min_index_on_target = start_index + 2;
	if (index_on_target < min_index_on_target) {
		LOG_TARGET_ERROR(target, "Arithmetic does not work correctly on the target");
		return ERROR_FAIL;
	} else if (index_on_target == min_index_on_target) {
		LOG_TARGET_DEBUG(target, "No forward progress");
	}
	const uint32_t next_index = (index_on_target - 2);
	*elements_read = next_index - start_index;
	LOG_TARGET_WARNING(target, "Re-reading memory from addresses 0x%"
			TARGET_PRIxADDR " and 0x%" TARGET_PRIxADDR ".",
			access.target_address + access.increment * next_index,
			access.target_address + access.increment * (next_index + 1));
	return read_memory_progbuf_inner_startup(target, access.target_address,
			access.increment, next_index);
}

/**
 * This function attempts to restore the pipeline after a dmi busy.
 */
static int read_memory_progbuf_inner_on_dmi_busy(struct target *target,
		uint32_t start_index, uint32_t next_start_index,
		struct memory_access_info access)
{
	LOG_TARGET_DEBUG(target, "DMI_STATUS_BUSY encountered in batch. Memory read [%"
			PRIu32 ", %" PRIu32 ")", start_index, next_start_index);
	if (start_index == next_start_index)
		LOG_TARGET_DEBUG(target, "No forward progress");

	if (dm_write(target, DM_ABSTRACTAUTO, 0) != ERROR_OK)
		return ERROR_FAIL;
	return read_memory_progbuf_inner_startup(target, access.target_address,
			access.increment, next_start_index);
}

/**
 * This function extracts the data from the batch.
 */
static int read_memory_progbuf_inner_extract_batch_data(struct target *target,
		const struct riscv_batch *batch,
		uint32_t start_index, uint32_t elements_to_read, uint32_t *elements_read,
		struct memory_access_info access)
{
	const bool two_reads_per_element = access.element_size > 4;
	const uint32_t reads_per_element = (two_reads_per_element ? 2 : 1);
	assert(!two_reads_per_element || riscv_xlen(target) == 64);
	assert(elements_to_read <= UINT32_MAX / reads_per_element);
	const uint32_t nreads = elements_to_read * reads_per_element;
	for (uint32_t curr_idx = start_index, read = 0; read < nreads; ++read) {
		switch (riscv_batch_get_dmi_read_op(batch, read)) {
		case DMI_STATUS_BUSY:
			*elements_read = curr_idx - start_index;
			return read_memory_progbuf_inner_on_dmi_busy(target, start_index, curr_idx
					, access);
		case DMI_STATUS_FAILED:
			LOG_TARGET_DEBUG(target,
					"Batch memory read encountered DMI_STATUS_FAILED on read %"
					PRIu32, read);
			return ERROR_FAIL;
		case DMI_STATUS_SUCCESS:
			break;
		default:
			assert(0);
		}
		const uint32_t value = riscv_batch_get_dmi_read_data(batch, read);
		uint8_t * const curr_buff = access.buffer_address +
			curr_idx * access.element_size;
		const target_addr_t curr_addr = access.target_address +
			curr_idx * access.increment;
		const uint32_t size = access.element_size;

		assert(size <= 8);
		const bool is_odd_read = read % 2;

		if (two_reads_per_element && !is_odd_read) {
			buf_set_u32(curr_buff + 4, 0, (size * 8) - 32, value);
			continue;
		}
		const bool is_second_read = two_reads_per_element;

		buf_set_u32(curr_buff, 0, is_second_read ? 32 : (size * 8), value);
		log_memory_access64(curr_addr, buf_get_u64(curr_buff, 0, size * 8),
				size, /*is_read*/ true);
		++curr_idx;
	}
	*elements_read = elements_to_read;
	return ERROR_OK;
}

/**
 * This function reads a batch of elements from memory.
 * Prior to calling this function the folowing conditions should be met:
 * - Appropriate program loaded to program buffer.
 * - DM_ABSTRACTAUTO_AUTOEXECDATA is set.
 */
static int read_memory_progbuf_inner_run_and_process_batch(struct target *target,
		struct riscv_batch *batch, struct memory_access_info access,
		uint32_t start_index, uint32_t elements_to_read, uint32_t *elements_read)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	/* Abstract commands are executed while running the batch. */
	dm->abstract_cmd_maybe_busy = true;
	if (batch_run(target, batch) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t abstractcs;
	if (wait_for_idle(target, &abstractcs) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t elements_to_extract_from_batch;

	uint32_t cmderr = get_field32(abstractcs, DM_ABSTRACTCS_CMDERR);
	switch (cmderr) {
	case CMDERR_NONE:
		LOG_TARGET_DEBUG(target, "successful (partial?) memory read [%"
				PRIu32 ", %" PRIu32 ")", start_index, start_index + elements_to_read);
		elements_to_extract_from_batch = elements_to_read;
		break;
	case CMDERR_BUSY:
		LOG_TARGET_DEBUG(target, "memory read resulted in busy response");
		if (read_memory_progbuf_inner_on_ac_busy(target, start_index,
					&elements_to_extract_from_batch, access)
				!= ERROR_OK)
			return ERROR_FAIL;
		break;
	default:
		LOG_TARGET_DEBUG(target, "error when reading memory, cmderr=0x%" PRIx32, cmderr);
		riscv013_clear_abstract_error(target);
		return ERROR_FAIL;
	}

	if (read_memory_progbuf_inner_extract_batch_data(target, batch, start_index,
				elements_to_extract_from_batch, elements_read, access) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static uint32_t read_memory_progbuf_inner_fill_batch(struct riscv_batch *batch,
		uint32_t count, uint32_t size)
{
	assert(size <= 8);
	const uint32_t two_regs_used[] = {DM_DATA1, DM_DATA0};
	const uint32_t one_reg_used[] = {DM_DATA0};
	const uint32_t reads_per_element = size > 4 ? 2 : 1;
	const uint32_t * const used_regs = size > 4 ? two_regs_used : one_reg_used;
	const uint32_t batch_capacity = riscv_batch_available_scans(batch) / reads_per_element;
	const uint32_t end = MIN(batch_capacity, count);

	for (uint32_t j = 0; j < end; ++j) {
		/* TODO: reuse "abstract_data_read_fill_batch()" here.
		 * TODO: Only the read of "DM_DATA0" starts an abstract
		 * command, so the other read can use "RISCV_DELAY_BASE"
		 */
		for (uint32_t i = 0; i < reads_per_element; ++i)
			riscv_batch_add_dm_read(batch, used_regs[i],
					RISCV_DELAY_ABSTRACT_COMMAND);
	}
	return end;
}

static int read_memory_progbuf_inner_try_to_read(struct target *target,
		struct memory_access_info access, uint32_t *elements_read,
		uint32_t index, uint32_t loop_count)
{
	struct riscv_batch *batch = riscv_batch_alloc(target, RISCV_BATCH_ALLOC_SIZE);
	if (!batch)
		return ERROR_FAIL;

	const uint32_t elements_to_read = read_memory_progbuf_inner_fill_batch(batch,
			loop_count - index, access.element_size);

	int result = read_memory_progbuf_inner_run_and_process_batch(target, batch,
			access, index, elements_to_read, elements_read);
	riscv_batch_free(batch);
	return result;
}

/**
 * read_memory_progbuf_inner_startup() must be called before calling this function
 * with the address argument equal to curr_target_address.
 */
static int read_memory_progbuf_inner_ensure_forward_progress(struct target *target,
		struct memory_access_info access, uint32_t start_index)
{
	LOG_TARGET_DEBUG(target,
			"Executing one loop iteration to ensure forward progress (index=%"
			PRIu32 ")", start_index);
	const target_addr_t curr_target_address = access.target_address +
		start_index * access.increment;
	uint8_t * const curr_buffer_address = access.buffer_address +
		start_index * access.element_size;
	const struct memory_access_info curr_access = {
		.buffer_address = curr_buffer_address,
		.target_address = curr_target_address,
		.element_size = access.element_size,
		.increment = access.increment,
	};
	uint32_t elements_read;
	if (read_memory_progbuf_inner_try_to_read(target, curr_access, &elements_read,
			/*index*/ 0, /*loop_count*/ 1) != ERROR_OK)
		return ERROR_FAIL;

	if (elements_read != 1) {
		assert(elements_read == 0);
		LOG_TARGET_DEBUG(target, "Can not ensure forward progress");
		/* FIXME: Here it would be better to retry the read and fail only if the
		 * delay is greater then some threshold.
		 */
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static void set_buffer_and_log_read(struct memory_access_info access,
		uint32_t index, uint64_t value)
{
	uint8_t * const buffer = access.buffer_address;
	const uint32_t size = access.element_size;
	const uint32_t increment = access.increment;
	const target_addr_t address = access.target_address;

	assert(size <= 8);
	buf_set_u64(buffer + index * size, 0, 8 * size, value);
	log_memory_access64(address + index * increment, value, size,
			/*is_read*/ true);
}

static int read_word_from_dm_data_regs(struct target *target,
		struct memory_access_info access, uint32_t index)
{
	assert(access.element_size <= 8);
	uint64_t value;
	int result = read_abstract_arg(target, &value, /*index*/ 0,
			access.element_size > 4 ? 64 : 32);
	if (result == ERROR_OK)
		set_buffer_and_log_read(access, index, value);
	return result;
}

static int read_word_from_s1(struct target *target,
		struct memory_access_info access, uint32_t index)
{
	uint64_t value;

	if (register_read_direct(target, &value, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;
	set_buffer_and_log_read(access, index, value);
	return ERROR_OK;
}

static int riscv_program_load_mprv(struct riscv_program *p, enum gdb_regno d,
		enum gdb_regno b, int offset, unsigned int size, bool mprven)
{
	if (mprven && riscv_program_csrrsi(p, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN,
				GDB_REGNO_DCSR) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_program_load(p, d, b, offset, size) != ERROR_OK)
		return ERROR_FAIL;

	if (mprven && riscv_program_csrrci(p, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN,
				GDB_REGNO_DCSR) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int read_memory_progbuf_inner_fill_progbuf(struct target *target,
		uint32_t increment, uint32_t size, bool mprven)
{
	const bool is_repeated_read = increment == 0;

	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv013_reg_save(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;
	if (is_repeated_read &&	riscv013_reg_save(target, GDB_REGNO_A0) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;

	riscv_program_init(&program, target);
	if (riscv_program_load_mprv(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0, size,
				mprven) != ERROR_OK)
		return ERROR_FAIL;
	if (is_repeated_read) {
		if (riscv_program_addi(&program, GDB_REGNO_A0, GDB_REGNO_A0, 1)
				!= ERROR_OK)
			return ERROR_FAIL;
	} else {
		if (riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0,
					increment)
				!= ERROR_OK)
			return ERROR_FAIL;
	}
	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * Read the requested memory, taking care to minimize the number of reads and
 * re-read the data only if `abstract command busy` or `DMI busy`
 * is encountered in the process.
 */
static int read_memory_progbuf_inner(struct target *target,
		struct memory_access_info access, uint32_t count, bool mprven)
{
	assert(count > 1 && "If count == 1, read_memory_progbuf_inner_one must be called");

	if (read_memory_progbuf_inner_fill_progbuf(target, access.increment,
				access.element_size, mprven) != ERROR_OK)
		return ERROR_FAIL;

	if (read_memory_progbuf_inner_startup(target, access.target_address,
				access.increment, /*index*/ 0)
			!= ERROR_OK)
		return ERROR_FAIL;
	/* The program in program buffer is executed twice during
	 * read_memory_progbuf_inner_startup().
	 * Here:
	 * dm_data[0:1] == M[address]
	 * s1 == M[address + increment]
	 * s0 == address + increment * 2
	 * `count - 2` program executions are performed in this loop.
	 * No need to execute the program any more, since S1 will already contain
	 * M[address + increment * (count - 1)] and we can read it directly.
	 */
	const uint32_t loop_count = count - 2;

	for (uint32_t index = 0; index < loop_count;) {
		uint32_t elements_read;
		if (read_memory_progbuf_inner_try_to_read(target, access, &elements_read,
					index, loop_count) != ERROR_OK) {
			dm_write(target, DM_ABSTRACTAUTO, 0);
			return ERROR_FAIL;
		}
		if (elements_read == 0) {
			if (read_memory_progbuf_inner_ensure_forward_progress(target, access,
						index) != ERROR_OK) {
				dm_write(target, DM_ABSTRACTAUTO, 0);
				return ERROR_FAIL;
			}
			elements_read = 1;
		}
		index += elements_read;
		assert(index <= loop_count);
	}
	if (dm_write(target, DM_ABSTRACTAUTO, 0) != ERROR_OK)
		return ERROR_FAIL;

	/* Read the penultimate word. */
	if (read_word_from_dm_data_regs(target, access, count - 2)
			!= ERROR_OK)
		return ERROR_FAIL;
	/* Read the last word. */
	return read_word_from_s1(target, access, count - 1);
}

/**
 * Only need to save/restore one GPR to read a single word, and the progbuf
 * program doesn't need to increment.
 */
static int read_memory_progbuf_inner_one(struct target *target,
		struct memory_access_info access, bool mprven)
{
	if (riscv013_reg_save(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;

	riscv_program_init(&program, target);
	if (riscv_program_load_mprv(&program, GDB_REGNO_S1, GDB_REGNO_S1, 0,
				access.element_size, mprven) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	/* Write address to S1, and execute buffer. */
	if (write_abstract_arg(target, 0, access.target_address, riscv_xlen(target))
			!= ERROR_OK)
		return ERROR_FAIL;
	uint32_t command = access_register_command(target, GDB_REGNO_S1,
			riscv_xlen(target), AC_ACCESS_REGISTER_WRITE |
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	uint32_t cmderr;
	if (execute_abstract_command(target, command, &cmderr) != ERROR_OK)
		return ERROR_FAIL;

	return read_word_from_s1(target, access, 0);
}

/**
 * Read the requested memory, silently handling memory access errors.
 */
static int read_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (riscv_xlen(target) < size * 8) {
		LOG_TARGET_ERROR(target, "XLEN (%d) is too short for %"
				PRIu32 "-bit memory read.", riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "reading %" PRIu32 " elements of %" PRIu32
			" bytes from 0x%" TARGET_PRIxADDR, count, size, address);

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	select_dmi(target);

	memset(buffer, 0, count*size);

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	const bool mprven = riscv_enable_virtual && get_field(mstatus, MSTATUS_MPRV);
	const struct memory_access_info access = {
		.target_address = address,
		.increment = increment,
		.buffer_address = buffer,
		.element_size = size,
	};
	int result = (count == 1) ?
		read_memory_progbuf_inner_one(target, access, mprven) :
		read_memory_progbuf_inner(target, access, count, mprven);

	if (mstatus != mstatus_old &&
			register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (count == 0)
		return ERROR_OK;

	if (size != 1 && size != 2 && size != 4 && size != 8 && size != 16) {
		LOG_TARGET_ERROR(target, "BUG: Unsupported size for memory read: %d", size);
		return ERROR_FAIL;
	}

	int ret = ERROR_FAIL;
	RISCV_INFO(r);
	RISCV013_INFO(info);

	char *progbuf_result = "disabled";
	char *sysbus_result = "disabled";
	char *abstract_result = "disabled";

	for (unsigned int i = 0; i < RISCV_NUM_MEM_ACCESS_METHODS; i++) {
		int method = r->mem_access_methods[i];

		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			if (mem_should_skip_progbuf(target, address, size, true, &progbuf_result))
				continue;

			ret = read_memory_progbuf(target, address, size, count, buffer, increment);

			if (ret != ERROR_OK)
				progbuf_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_SYSBUS) {
			if (mem_should_skip_sysbus(target, address, size, increment, true, &sysbus_result))
				continue;

			if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
				ret = read_memory_bus_v0(target, address, size, count, buffer, increment);
			else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
				ret = read_memory_bus_v1(target, address, size, count, buffer, increment);

			if (ret != ERROR_OK)
				sysbus_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			if (mem_should_skip_abstract(target, address, size, increment, true, &abstract_result))
				continue;

			ret = read_memory_abstract(target, address, size, count, buffer, increment);

			if (ret != ERROR_OK)
				abstract_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_UNSPECIFIED)
			/* No further mem access method to try. */
			break;

		log_mem_access_result(target, ret == ERROR_OK, method, true);

		if (ret == ERROR_OK)
			return ret;
	}

	LOG_TARGET_ERROR(target, "Failed to read memory (addr=0x%" PRIx64 ")", address);
	LOG_TARGET_ERROR(target, "  progbuf=%s, sysbus=%s, abstract=%s", progbuf_result, sysbus_result, abstract_result);
	return ret;
}

static int write_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/*1) write sbaddress: for singlewrite and autoincrement, we need to write the address once*/
	LOG_TARGET_DEBUG(target, "System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	dm_write(target, DM_SBADDRESS0, address);
	int64_t value = 0;
	int64_t access = 0;
	riscv_addr_t offset = 0;
	riscv_addr_t t_addr = 0;
	const uint8_t *t_buffer = buffer + offset;

	/* B.8 Writing Memory, single write check if we write in one go */
	if (count == 1) { /* count is in bytes here */
		value = buf_get_u64(t_buffer, 0, 8 * size);

		access = 0;
		access = set_field(access, DM_SBCS_SBACCESS, size/2);
		dm_write(target, DM_SBCS, access);
		LOG_TARGET_DEBUG(target, "  access:  0x%08" PRIx64, access);
		LOG_TARGET_DEBUG(target, "  write_memory:SAB: ONE OFF: value 0x%08" PRIx64, value);
		dm_write(target, DM_SBDATA0, value);
		return ERROR_OK;
	}

	/*B.8 Writing Memory, using autoincrement*/

	access = 0;
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_TARGET_DEBUG(target, "  access:  0x%08" PRIx64, access);
	dm_write(target, DM_SBCS, access);

	/*2)set the value according to the size required and write*/
	for (riscv_addr_t i = 0; i < count; ++i) {
		offset = size*i;
		/* for monitoring only */
		t_addr = address + offset;
		t_buffer = buffer + offset;

		value = buf_get_u64(t_buffer, 0, 8 * size);
		LOG_TARGET_DEBUG(target, "SAB:autoincrement: expected address: 0x%08x value: 0x%08x"
				PRIx64, (uint32_t)t_addr, (uint32_t)value);
		dm_write(target, DM_SBDATA0, value);
	}
	/*reset the autoincrement when finished (something weird is happening if this is not done at the end*/
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 0);
	dm_write(target, DM_SBCS, access);

	return ERROR_OK;
}

static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);
	uint32_t sbcs = sb_sbaccess(size);
	sbcs = set_field(sbcs, DM_SBCS_SBAUTOINCREMENT, 1);
	dm_write(target, DM_SBCS, sbcs);

	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	int result = sb_write_address(target, next_address, RISCV_DELAY_BASE);
	if (result != ERROR_OK)
		return result;

	while (next_address < end_address) {
		LOG_TARGET_DEBUG(target, "Transferring burst starting at address 0x%" TARGET_PRIxADDR,
				next_address);

		struct riscv_batch *batch = riscv_batch_alloc(target, RISCV_BATCH_ALLOC_SIZE);
		if (!batch)
			return ERROR_FAIL;

		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			const uint8_t *p = buffer + i * size;

			if (riscv_batch_available_scans(batch) < (size + 3) / 4)
				break;

			uint32_t sbvalue[4] = { 0 };
			if (size > 12) {
				sbvalue[3] = ((uint32_t)p[12]) |
						(((uint32_t)p[13]) << 8) |
						(((uint32_t)p[14]) << 16) |
						(((uint32_t)p[15]) << 24);
				riscv_batch_add_dm_write(batch, DM_SBDATA3, sbvalue[3], false,
						RISCV_DELAY_BASE);
			}

			if (size > 8) {
				sbvalue[2] = ((uint32_t)p[8]) |
						(((uint32_t)p[9]) << 8) |
						(((uint32_t)p[10]) << 16) |
						(((uint32_t)p[11]) << 24);
				riscv_batch_add_dm_write(batch, DM_SBDATA2, sbvalue[2], false,
						RISCV_DELAY_BASE);
			}
			if (size > 4) {
				sbvalue[1] = ((uint32_t)p[4]) |
						(((uint32_t)p[5]) << 8) |
						(((uint32_t)p[6]) << 16) |
						(((uint32_t)p[7]) << 24);
				riscv_batch_add_dm_write(batch, DM_SBDATA1, sbvalue[1], false,
						RISCV_DELAY_BASE);
			}

			sbvalue[0] = p[0];
			if (size > 2) {
				sbvalue[0] |= ((uint32_t)p[2]) << 16;
				sbvalue[0] |= ((uint32_t)p[3]) << 24;
			}
			if (size > 1)
				sbvalue[0] |= ((uint32_t)p[1]) << 8;

			riscv_batch_add_dm_write(batch, DM_SBDATA0, sbvalue[0], false,
						RISCV_DELAY_SYSBUS_WRITE);

			log_memory_access(address + i * size, sbvalue, size, false);

			next_address += size;
		}

		/* Execute the batch of writes */
		result = batch_run(target, batch);
		if (result != ERROR_OK) {
			riscv_batch_free(batch);
			return result;
		}

		bool dmi_busy_encountered = riscv_batch_was_batch_busy(batch);
		riscv_batch_free(batch);
		if (dmi_busy_encountered)
			LOG_TARGET_DEBUG(target, "DMI busy encountered during system bus write.");

		result = read_sbcs_nonbusy(target, &sbcs);
		if (result != ERROR_OK)
			return result;

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. */
			LOG_TARGET_DEBUG(target, "Sbbusyerror encountered during system bus write.");
			/* Clear the sticky error flag. */
			dm_write(target, DM_SBCS, sbcs | DM_SBCS_SBBUSYERROR);
			/* Slow down before trying again.
			 * FIXME: Possible overflow is ignored here.
			 */
			riscv_scan_increase_delay(&info->learned_delays,
					RISCV_DELAY_SYSBUS_WRITE);
		}

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR) || dmi_busy_encountered) {
			/* Recover from the case when the write commands were issued too fast.
			 * Determine the address from which to resume writing. */
			next_address = sb_read_address(target);
			if (next_address < address) {
				/* This should never happen, probably buggy hardware. */
				LOG_TARGET_DEBUG(target, "unexpected sbaddress=0x%" TARGET_PRIxADDR
						  " - buggy sbautoincrement in hw?", next_address);
				/* Fail the whole operation. */
				return ERROR_FAIL;
			}
			/* Try again - resume writing. */
			continue;
		}

		unsigned int sberror = get_field(sbcs, DM_SBCS_SBERROR);
		if (sberror != 0) {
			/* Sberror indicates the bus access failed, but not because we issued the writes
			 * too fast. Cannot recover. Sbaddress holds the address where the error occurred
			 * (unless sbautoincrement in the HW is buggy).
			 */
			target_addr_t sbaddress = sb_read_address(target);
			LOG_TARGET_DEBUG(target, "System bus access failed with sberror=%u (sbaddress=0x%" TARGET_PRIxADDR ")",
					  sberror, sbaddress);
			if (sbaddress < address) {
				/* This should never happen, probably buggy hardware.
				 * Make a note to the user not to trust the sbaddress value. */
				LOG_TARGET_DEBUG(target, "unexpected sbaddress=0x%" TARGET_PRIxADDR
						  " - buggy sbautoincrement in hw?", next_address);
			}
			/* Clear the sticky error flag */
			dm_write(target, DM_SBCS, DM_SBCS_SBERROR);
			/* Fail the whole operation */
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/**
 * This function is used to start the memory-writing pipeline.
 * As part of the process, the function writes the first item and waits for completion,
 * so forward progress is ensured.
 * The pipeline looks like this:
 * debugger -> dm_data[0:1] -> s1 -> memory
 * Prior to calling it, the program buffer should contain the appropriate
 * program.
 * This function sets DM_ABSTRACTAUTO_AUTOEXECDATA to trigger second stage of the
 * pipeline (dm_data[0:1] -> s1) whenever dm_data is written.
 */
static int write_memory_progbuf_startup(struct target *target, target_addr_t *address_p,
		const uint8_t *buffer, uint32_t size)
{
	/* TODO: There is potential to gain some performance if the operations below are
	 * executed inside the first DMI batch (not separately). */
	if (register_write_direct(target, GDB_REGNO_S0, *address_p) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the first item to data0 [, data1] */
	assert(size <= 8);
	const uint64_t value = buf_get_u64(buffer, 0, 8 * size);
	if (write_abstract_arg(target, /*index*/ 0, value, size > 4 ? 64 : 32)
			!= ERROR_OK)
		return ERROR_FAIL;

	/* Write and execute command that moves the value from data0 [, data1]
	 * into S1 and executes program buffer. */
	uint32_t command = access_register_command(target,
			GDB_REGNO_S1, riscv_xlen(target),
			AC_ACCESS_REGISTER_POSTEXEC |
			AC_ACCESS_REGISTER_TRANSFER |
			AC_ACCESS_REGISTER_WRITE);

	uint32_t cmderr;
	if (execute_abstract_command(target, command, &cmderr) != ERROR_OK)
		return ERROR_FAIL;

	log_memory_access64(*address_p, value, size, /*is_read*/ false);

	/* The execution of the command succeeded, which means:
	 * - write of the first item to memory succeeded
	 * - address on the target (S0) was incremented
	 */
	*address_p += size;

	/* TODO: Setting abstractauto.autoexecdata is not necessary for a write
	 * of one element. */
	return dm_write(target, DM_ABSTRACTAUTO,
			1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);
}

/**
 * This function reverts the changes made by `write_memory_progbuf_startup()`
 */
static int write_memory_progbuf_teardown(struct target *target)
{
	return dm_write(target, DM_ABSTRACTAUTO, 0);
}

/**
 * This function attempts to restore the pipeline after a busy on abstract
 * access or a DMI busy by reading the content of s0 -- the address of the
 * failed write.
 */
static int write_memory_progbuf_handle_busy(struct target *target,
		target_addr_t *address_p, uint32_t size, const uint8_t *buffer)
{
	int res = riscv013_clear_abstract_error(target);
	if (res != ERROR_OK)
		return res;
	res = increase_ac_busy_delay(target);
	if (res != ERROR_OK)
		return res;

	if (write_memory_progbuf_teardown(target) != ERROR_OK)
		return ERROR_FAIL;

	target_addr_t address_on_target;
	if (register_read_direct(target, &address_on_target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	const uint8_t * const curr_buff = buffer + (address_on_target - *address_p);
	LOG_TARGET_DEBUG(target, "Restarting from 0x%" TARGET_PRIxADDR, *address_p);
	*address_p = address_on_target;
	/* This restores the pipeline and ensures one item gets reliably written */
	return write_memory_progbuf_startup(target, address_p, curr_buff, size);
}

/**
 * This function fills the batch with DMI writes (but does not execute the batch).
 * It returns the next address -- the address that will be the start of the next batch.
 */
static target_addr_t write_memory_progbuf_fill_batch(struct riscv_batch *batch,
		target_addr_t start_address, target_addr_t end_address, uint32_t size,
		const uint8_t *buffer)
{
	assert(size <= 8);
	const unsigned int writes_per_element = size > 4 ? 2 : 1;
	const size_t batch_capacity = riscv_batch_available_scans(batch) / writes_per_element;
	/* This is safe even for the edge case when writing at the very top of
	 * the 64-bit address space (in which case end_address overflows to 0).
	 */
	const target_addr_t batch_end_address = start_address +
		MIN((target_addr_t)batch_capacity * size,
				end_address - start_address);
	for (target_addr_t address = start_address; address != batch_end_address;
			address += size, buffer += size) {
		assert(size <= 8);
		const uint64_t value = buf_get_u64(buffer, 0, 8 * size);
		log_memory_access64(address, value, size, /*is_read*/ false);
		if (writes_per_element == 2)
			riscv_batch_add_dm_write(batch, DM_DATA1,
					(uint32_t)(value >> 32), false,	RISCV_DELAY_BASE);
		riscv_batch_add_dm_write(batch, DM_DATA0, (uint32_t)value, false,
				RISCV_DELAY_ABSTRACT_COMMAND);
	}
	return batch_end_address;
}

/**
 * This function runs the batch of writes and updates address_p with the
 * address of the next write.
 */
static int write_memory_progbuf_run_batch(struct target *target, struct riscv_batch *batch,
		target_addr_t *address_p, target_addr_t end_address, uint32_t size,
		const uint8_t *buffer)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	/* Abstract commands are executed while running the batch. */
	dm->abstract_cmd_maybe_busy = true;
	if (batch_run(target, batch) != ERROR_OK)
		return ERROR_FAIL;

	/* Note that if the scan resulted in a Busy DMI response, it
	 * is this call to wait_for_idle() that will cause the dmi_busy_delay
	 * to be incremented if necessary. */
	uint32_t abstractcs;

	if (wait_for_idle(target, &abstractcs) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t cmderr = get_field32(abstractcs, DM_ABSTRACTCS_CMDERR);
	const bool dmi_busy_encountered = riscv_batch_was_batch_busy(batch);
	if (cmderr == CMDERR_NONE && !dmi_busy_encountered) {
		LOG_TARGET_DEBUG(target, "Successfully written memory block M[0x%" TARGET_PRIxADDR
				".. 0x%" TARGET_PRIxADDR ")", *address_p, end_address);
		*address_p = end_address;
		return ERROR_OK;
	} else if (cmderr == CMDERR_BUSY || dmi_busy_encountered) {
		if (cmderr == CMDERR_BUSY)
			LOG_TARGET_DEBUG(target, "Encountered abstract command busy response while writing block M[0x%"
					TARGET_PRIxADDR ".. 0x%" TARGET_PRIxADDR ")", *address_p, end_address);
		if (dmi_busy_encountered)
			LOG_TARGET_DEBUG(target, "Encountered DMI busy response while writing block M[0x%"
					TARGET_PRIxADDR ".. 0x%" TARGET_PRIxADDR ")", *address_p, end_address);
		/* TODO: If dmi busy is encountered, the address of the last
		 * successful write can be deduced by analysing the batch.
		 */
		return write_memory_progbuf_handle_busy(target, address_p, size, buffer);
	}
	LOG_TARGET_ERROR(target, "Error when writing memory, abstractcs=0x%" PRIx32,
			abstractcs);
	riscv013_clear_abstract_error(target);
	return ERROR_FAIL;
}

static int write_memory_progbuf_try_to_write(struct target *target,
		target_addr_t *address_p, target_addr_t end_address, uint32_t size,
		const uint8_t *buffer)
{
	struct riscv_batch * const batch = riscv_batch_alloc(target, RISCV_BATCH_ALLOC_SIZE);
	if (!batch)
		return ERROR_FAIL;

	const target_addr_t batch_end_addr = write_memory_progbuf_fill_batch(batch,
			*address_p, end_address, size, buffer);

	int result = write_memory_progbuf_run_batch(target, batch, address_p,
			batch_end_addr, size, buffer);
	riscv_batch_free(batch);
	return result;
}

static int riscv_program_store_mprv(struct riscv_program *p, enum gdb_regno d,
		enum gdb_regno b, int offset, unsigned int size, bool mprven)
{
	if (mprven && riscv_program_csrrsi(p, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN,
				GDB_REGNO_DCSR) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_program_store(p, d, b, offset, size) != ERROR_OK)
		return ERROR_FAIL;

	if (mprven && riscv_program_csrrci(p, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN,
				GDB_REGNO_DCSR) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int write_memory_progbuf_fill_progbuf(struct target *target,
		uint32_t size, bool mprven)
{
	if (riscv013_reg_save(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv013_reg_save(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;

	riscv_program_init(&program, target);
	if (riscv_program_store_mprv(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0, size,
				mprven) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;

	return riscv_program_write(&program);
}

static int write_memory_progbuf_inner(struct target *target, target_addr_t start_addr,
		uint32_t size, uint32_t count, const uint8_t *buffer, bool mprven)
{
	if (write_memory_progbuf_fill_progbuf(target, size,
				mprven) != ERROR_OK)
		return ERROR_FAIL;

	target_addr_t addr_on_target = start_addr;
	if (write_memory_progbuf_startup(target, &addr_on_target, buffer, size) != ERROR_OK)
		return ERROR_FAIL;

	const target_addr_t end_addr = start_addr + (target_addr_t)size * count;

	for (target_addr_t next_addr_on_target = addr_on_target; addr_on_target != end_addr;
			addr_on_target = next_addr_on_target) {
		const uint8_t * const curr_buff = buffer + (addr_on_target - start_addr);
		if (write_memory_progbuf_try_to_write(target, &next_addr_on_target,
					end_addr, size, curr_buff) != ERROR_OK) {
			write_memory_progbuf_teardown(target);
			return ERROR_FAIL;
		}
		/* write_memory_progbuf_try_to_write() ensures that at least one item
		 * gets successfully written even when busy condition is encountered.
		 * These assertions shuld hold when next_address_on_target overflows. */
		assert(next_addr_on_target - addr_on_target > 0);
		assert(next_addr_on_target - start_addr <= (target_addr_t)size * count);
	}

	return write_memory_progbuf_teardown(target);
}

static int write_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (riscv_xlen(target) < size * 8) {
		LOG_TARGET_ERROR(target, "XLEN (%u) is too short for %" PRIu32 "-bit memory write.",
				riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "writing %" PRIu32 " words of %" PRIu32
			" bytes to 0x%" TARGET_PRIxADDR, count, size, address);

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	const bool mprven = riscv_enable_virtual && get_field(mstatus, MSTATUS_MPRV);

	int result = write_memory_progbuf_inner(target, address, size, count, buffer, mprven);

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (size != 1 && size != 2 && size != 4 && size != 8 && size != 16) {
		LOG_TARGET_ERROR(target, "BUG: Unsupported size for memory write: %d", size);
		return ERROR_FAIL;
	}

	int ret = ERROR_FAIL;
	RISCV_INFO(r);
	RISCV013_INFO(info);

	char *progbuf_result = "disabled";
	char *sysbus_result = "disabled";
	char *abstract_result = "disabled";

	for (unsigned int i = 0; i < RISCV_NUM_MEM_ACCESS_METHODS; i++) {
		int method = r->mem_access_methods[i];

		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			if (mem_should_skip_progbuf(target, address, size, false, &progbuf_result))
				continue;

			ret = write_memory_progbuf(target, address, size, count, buffer);

			if (ret != ERROR_OK)
				progbuf_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_SYSBUS) {
			if (mem_should_skip_sysbus(target, address, size, 0, false, &sysbus_result))
				continue;

			if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
				ret = write_memory_bus_v0(target, address, size, count, buffer);
			else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
				ret = write_memory_bus_v1(target, address, size, count, buffer);

			if (ret != ERROR_OK)
				sysbus_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			if (mem_should_skip_abstract(target, address, size, 0, false, &abstract_result))
				continue;

			ret = write_memory_abstract(target, address, size, count, buffer);

			if (ret != ERROR_OK)
				abstract_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_UNSPECIFIED)
			/* No further mem access method to try. */
			break;

		log_mem_access_result(target, ret == ERROR_OK, method, false);

		if (ret == ERROR_OK)
			return ret;
	}

	LOG_TARGET_ERROR(target, "Target %s: Failed to write memory (addr=0x%" PRIx64 ")", target_name(target), address);
	LOG_TARGET_ERROR(target, "  progbuf=%s, sysbus=%s, abstract=%s", progbuf_result, sysbus_result, abstract_result);
	return ret;
}

static int arch_state(struct target *target)
{
	return ERROR_OK;
}

struct target_type riscv013_target = {
	.name = "riscv",

	.init_target = init_target,
	.deinit_target = deinit_target,
	.examine = examine,

	.poll = &riscv_openocd_poll,
	.halt = &riscv_halt,
	.step = &riscv_openocd_step,

	.assert_reset = assert_reset,
	.deassert_reset = deassert_reset,

	.write_memory = write_memory,

	.arch_state = arch_state
};

/*** 0.13-specific implementations of various RISC-V helper functions. ***/
int riscv013_get_register(struct target *target,
		riscv_reg_t *value, enum gdb_regno rid)
{
	/* It would be beneficial to move this redirection to the
	 * version-independent section, but there is a conflict:
	 * `dcsr[5]` is `dcsr.v` in current spec, but it is `dcsr.debugint` in 0.11.
	 */
	if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		if (riscv_reg_get(target, &dcsr, GDB_REGNO_DCSR) != ERROR_OK)
			return ERROR_FAIL;
		*value = set_field(0, VIRT_PRIV_V, get_field(dcsr, CSR_DCSR_V));
		*value = set_field(*value, VIRT_PRIV_PRV, get_field(dcsr, CSR_DCSR_PRV));
		return ERROR_OK;
	}

	LOG_TARGET_DEBUG(target, "reading register %s",	riscv_reg_gdb_regno_name(target, rid));

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	if (register_read_direct(target, value, rid) != ERROR_OK) {
		*value = -1;
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int riscv013_set_register(struct target *target, enum gdb_regno rid,
		riscv_reg_t value)
{
	LOG_TARGET_DEBUG(target, "writing 0x%" PRIx64 " to register %s",
			value, riscv_reg_gdb_regno_name(target, rid));

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	return register_write_direct(target, rid, value);
}

static int dm013_select_hart(struct target *target, int hart_index)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (hart_index == dm->current_hartid)
		return ERROR_OK;

	/* `hartsel` should not be changed if `abstractcs.busy` is set. */
	int result = wait_for_idle_if_needed(target);
	if (result != ERROR_OK)
		return result;

	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE;
	dmcontrol = set_dmcontrol_hartsel(dmcontrol, hart_index);
	if (dm_write(target, DM_DMCONTROL, dmcontrol) != ERROR_OK) {
		/* Who knows what the state is? */
		dm->current_hartid = HART_INDEX_UNKNOWN;
		return ERROR_FAIL;
	}
	dm->current_hartid = hart_index;
	return ERROR_OK;
}

/* Select all harts that were prepped and that are selectable, clearing the
 * prepped flag on the harts that actually were selected. */
static int select_prepped_harts(struct target *target)
{
	RISCV_INFO(r);
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (!dm->hasel_supported) {
		r->prepped = false;
		return dm013_select_target(target);
	}

	assert(dm->hart_count);
	unsigned hawindow_count = (dm->hart_count + 31) / 32;
	uint32_t *hawindow = calloc(hawindow_count, sizeof(uint32_t));
	if (!hawindow)
		return ERROR_FAIL;

	target_list_t *entry;
	unsigned total_selected = 0;
	unsigned int selected_index = 0;
	list_for_each_entry(entry, &dm->target_list, list) {
		struct target *t = entry->target;
		struct riscv_info *info = riscv_info(t);
		riscv013_info_t *info_013 = get_info(t);
		unsigned int index = info_013->index;
		LOG_TARGET_DEBUG(target, "index=%d, prepped=%d", index, info->prepped);
		if (info->prepped) {
			info_013->selected = true;
			hawindow[index / 32] |= 1 << (index % 32);
			info->prepped = false;
			total_selected++;
			selected_index = index;
		}
	}

	if (total_selected == 0) {
		LOG_TARGET_ERROR(target, "No harts were prepped!");
		free(hawindow);
		return ERROR_FAIL;
	} else if (total_selected == 1) {
		/* Don't use hasel if we only need to talk to one hart. */
		free(hawindow);
		return dm013_select_hart(target, selected_index);
	}

	if (dm013_select_hart(target, HART_INDEX_MULTIPLE) != ERROR_OK) {
		free(hawindow);
		return ERROR_FAIL;
	}

	for (unsigned i = 0; i < hawindow_count; i++) {
		if (dm_write(target, DM_HAWINDOWSEL, i) != ERROR_OK) {
			free(hawindow);
			return ERROR_FAIL;
		}
		if (dm_write(target, DM_HAWINDOW, hawindow[i]) != ERROR_OK) {
			free(hawindow);
			return ERROR_FAIL;
		}
	}

	free(hawindow);
	return ERROR_OK;
}

static int riscv013_halt_prep(struct target *target)
{
	return ERROR_OK;
}

static int riscv013_halt_go(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	if (select_prepped_harts(target) != ERROR_OK)
		return ERROR_FAIL;

	LOG_TARGET_DEBUG(target, "halting hart");

	/* `haltreq` should not be issued if `abstractcs.busy` is set. */
	int result = wait_for_idle_if_needed(target);
	if (result != ERROR_OK)
		return result;

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_HALTREQ;
	dmcontrol = set_dmcontrol_hartsel(dmcontrol, dm->current_hartid);
	dm_write(target, DM_DMCONTROL, dmcontrol);
	uint32_t dmstatus;
	for (size_t i = 0; i < 256; ++i) {
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		/* When no harts are running, there's no point in continuing this loop. */
		if (!get_field(dmstatus, DM_DMSTATUS_ANYRUNNING))
			break;
	}

	/* We declare success if no harts are running. One or more of them may be
	 * unavailable, though. */

	if ((get_field(dmstatus, DM_DMSTATUS_ANYRUNNING))) {
		if (dm_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
			return ERROR_FAIL;

		LOG_TARGET_ERROR(target, "Unable to halt. dmcontrol=0x%08x, dmstatus=0x%08x",
				  dmcontrol, dmstatus);
		return ERROR_FAIL;
	}

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HALTREQ, 0);
	dm_write(target, DM_DMCONTROL, dmcontrol);

	if (dm->current_hartid == HART_INDEX_MULTIPLE) {
		target_list_t *entry;
		list_for_each_entry(entry, &dm->target_list, list) {
			struct target *t = entry->target;
			uint32_t t_dmstatus;
			if (get_field(dmstatus, DM_DMSTATUS_ALLHALTED) ||
					get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL)) {
				/* All harts are either halted or unavailable. No
				 * need to read dmstatus for each hart. */
				t_dmstatus = dmstatus;
			} else {
				/* Only some harts were halted/unavailable. Read
				 * dmstatus for this one to see what its status
				 * is. */
				if (dm013_select_target(target) != ERROR_OK)
					return ERROR_FAIL;
				if (dm_read(target, &t_dmstatus, DM_DMSTATUS) != ERROR_OK)
					return ERROR_FAIL;
			}
			/* Set state for the current target based on its dmstatus. */
			if (get_field(t_dmstatus, DM_DMSTATUS_ALLHALTED)) {
				t->state = TARGET_HALTED;
				if (t->debug_reason == DBG_REASON_NOTHALTED)
					t->debug_reason = DBG_REASON_DBGRQ;
			} else if (get_field(t_dmstatus, DM_DMSTATUS_ALLUNAVAIL)) {
				t->state = TARGET_UNAVAILABLE;
			}
		}

	} else {
		/* Set state for the current target based on its dmstatus. */
		if (get_field(dmstatus, DM_DMSTATUS_ALLHALTED)) {
			target->state = TARGET_HALTED;
			if (target->debug_reason == DBG_REASON_NOTHALTED)
				target->debug_reason = DBG_REASON_DBGRQ;
		} else if (get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL)) {
			target->state = TARGET_UNAVAILABLE;
		}
	}

	return ERROR_OK;
}

static int riscv013_resume_go(struct target *target)
{
	if (select_prepped_harts(target) != ERROR_OK)
		return ERROR_FAIL;

	return riscv013_step_or_resume_current_hart(target, false);
}

static int riscv013_step_current_hart(struct target *target)
{
	return riscv013_step_or_resume_current_hart(target, true);
}

static int riscv013_resume_prep(struct target *target)
{
	assert(target->state == TARGET_HALTED);
	return riscv013_on_step_or_resume(target, false);
}

static int riscv013_on_step(struct target *target)
{
	return riscv013_on_step_or_resume(target, true);
}

static enum riscv_halt_reason riscv013_halt_reason(struct target *target)
{
	riscv_reg_t dcsr;
	int result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return RISCV_HALT_UNKNOWN;

	LOG_TARGET_DEBUG(target, "dcsr.cause: 0x%" PRIx64, get_field(dcsr, CSR_DCSR_CAUSE));

	switch (get_field(dcsr, CSR_DCSR_CAUSE)) {
	case CSR_DCSR_CAUSE_EBREAK:
		return RISCV_HALT_EBREAK;
	case CSR_DCSR_CAUSE_TRIGGER:
		/* We could get here before triggers are enumerated if a trigger was
		 * already set when we connected. Force enumeration now, which has the
		 * side effect of clearing any triggers we did not set. */
		riscv_enumerate_triggers(target);
		LOG_TARGET_DEBUG(target, "halted because of trigger");
		return RISCV_HALT_TRIGGER;
	case CSR_DCSR_CAUSE_STEP:
		return RISCV_HALT_SINGLESTEP;
	case CSR_DCSR_CAUSE_HALTREQ:
	case CSR_DCSR_CAUSE_RESETHALTREQ:
		return RISCV_HALT_INTERRUPT;
	case CSR_DCSR_CAUSE_GROUP:
		return RISCV_HALT_GROUP;
	}

	LOG_TARGET_ERROR(target, "Unknown DCSR cause field: 0x%" PRIx64, get_field(dcsr, CSR_DCSR_CAUSE));
	LOG_TARGET_ERROR(target, "  dcsr=0x%" PRIx32, (uint32_t)dcsr);
	return RISCV_HALT_UNKNOWN;
}

static int riscv013_write_progbuf(struct target *target, unsigned int index, riscv_insn_t data)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (dm->progbuf_cache[index] != data) {
		if (dm_write(target, DM_PROGBUF0 + index, data) != ERROR_OK)
			return ERROR_FAIL;
		dm->progbuf_cache[index] = data;
	} else {
		LOG_TARGET_DEBUG(target, "Cache hit for 0x%" PRIx32 " @%d", data, index);
	}
	return ERROR_OK;
}

static riscv_insn_t riscv013_read_progbuf(struct target *target, unsigned int index)
{
	uint32_t value;
	if (dm_read(target, &value, DM_PROGBUF0 + index) == ERROR_OK)
		return value;
	else
		return 0;
}

static int riscv013_invalidate_cached_progbuf(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm) {
		LOG_TARGET_DEBUG(target, "No DM is specified for the target");
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "Invalidating progbuf cache");
	for (unsigned int i = 0; i < 15; i++)
		dm->progbuf_cache[i] = 0;
	return ERROR_OK;
}

static int riscv013_execute_progbuf(struct target *target, uint32_t *cmderr)
{
	uint32_t run_program = 0;
	run_program = set_field(run_program, AC_ACCESS_REGISTER_AARSIZE, 2);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_POSTEXEC, 1);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_TRANSFER, 0);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_REGNO, 0x1000);

	return execute_abstract_command(target, run_program, cmderr);
}

static void riscv013_fill_dmi_write(struct target *target, char *buf, uint64_t a, uint32_t d)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_WRITE);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, d);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

static void riscv013_fill_dmi_read(struct target *target, char *buf, uint64_t a)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_READ);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

static void riscv013_fill_dm_nop(struct target *target, char *buf)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_NOP);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, 0);
}

static int riscv013_get_dmi_scan_length(struct target *target)
{
	RISCV013_INFO(info);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

static int maybe_execute_fence_i(struct target *target)
{
	if (has_sufficient_progbuf(target, 2))
		return execute_fence(target);
	return ERROR_OK;
}

/* Helper Functions. */
static int riscv013_on_step_or_resume(struct target *target, bool step)
{
	if (maybe_execute_fence_i(target) != ERROR_OK)
		return ERROR_FAIL;

	if (set_dcsr_ebreak(target, step) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_reg_flush_all(target) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step)
{
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Hart is not halted!");
		return ERROR_FAIL;
	}
	LOG_TARGET_DEBUG(target, "resuming (for step?=%d)", step);

	if (riscv_reg_flush_all(target) != ERROR_OK)
		return ERROR_FAIL;

	dm013_info_t *dm = get_dm(target);
	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_RESUMEREQ;
	dmcontrol = set_dmcontrol_hartsel(dmcontrol, dm->current_hartid);
	/* `resumereq` should not be issued if `abstractcs.busy` is set. */
	int result = wait_for_idle_if_needed(target);
	if (result != ERROR_OK)
		return result;
	dm_write(target, DM_DMCONTROL, dmcontrol);

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_RESUMEREQ, 0);

	uint32_t dmstatus;
	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		if (get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL))
			return ERROR_FAIL;
		if (get_field(dmstatus, DM_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		if (step && get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0)
			continue;

		dm_write(target, DM_DMCONTROL, dmcontrol);
		return ERROR_OK;
	}

	dm_write(target, DM_DMCONTROL, dmcontrol);

	LOG_TARGET_ERROR(target, "unable to resume");
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return ERROR_FAIL;
	LOG_TARGET_ERROR(target, "  dmstatus=0x%08x", dmstatus);

	if (step) {
		LOG_TARGET_ERROR(target, "  was stepping, halting");
		riscv_halt(target);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int riscv013_clear_abstract_error(struct target *target)
{
	uint32_t abstractcs;
	int result = wait_for_idle(target, &abstractcs);
	/* Clear the error status, even if busy is still set. */
	if (dm_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR) != ERROR_OK)
		result = ERROR_FAIL;
	return result;
}

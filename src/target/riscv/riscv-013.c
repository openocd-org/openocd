/* SPDX-License-Identifier: GPL-2.0-or-later */

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
#include "debug_defines.h"
#include "rtos/rtos.h"
#include "program.h"
#include "asm.h"
#include "batch.h"

static int riscv013_on_step_or_resume(struct target *target, bool step);
static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step);
static void riscv013_clear_abstract_error(struct target *target);

/* Implementations of the functions in riscv_info_t. */
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int rid);
static int riscv013_set_register(struct target *target, int regid, uint64_t value);
static int dm013_select_hart(struct target *target, int hart_index);
static int riscv013_halt_prep(struct target *target);
static int riscv013_halt_go(struct target *target);
static int riscv013_resume_go(struct target *target);
static int riscv013_step_current_hart(struct target *target);
static int riscv013_on_halt(struct target *target);
static int riscv013_on_step(struct target *target);
static int riscv013_resume_prep(struct target *target);
static enum riscv_halt_reason riscv013_halt_reason(struct target *target);
static int riscv013_write_debug_buffer(struct target *target, unsigned index,
		riscv_insn_t d);
static riscv_insn_t riscv013_read_debug_buffer(struct target *target, unsigned
		index);
static int riscv013_invalidate_cached_debug_buffer(struct target *target);
static int riscv013_execute_debug_buffer(struct target *target);
static void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
static void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a);
static int riscv013_dmi_write_u64_bits(struct target *target);
static void riscv013_fill_dmi_nop_u64(struct target *target, char *buf);
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number);
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value);
static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment);
static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define RISCV013_INFO(r) riscv013_info_t *r = get_info(target)

/*** JTAG registers. ***/

typedef enum {
	DMI_OP_NOP = 0,
	DMI_OP_READ = 1,
	DMI_OP_WRITE = 2
} dmi_op_t;
typedef enum {
	DMI_STATUS_SUCCESS = 0,
	DMI_STATUS_FAILED = 2,
	DMI_STATUS_BUSY = 3
} dmi_status_t;

typedef enum slot {
	SLOT0,
	SLOT1,
	SLOT_LAST,
} slot_t;

/*** Debug Bus registers. ***/

#define CMDERR_NONE				0
#define CMDERR_BUSY				1
#define CMDERR_NOT_SUPPORTED	2
#define CMDERR_EXCEPTION		3
#define CMDERR_HALT_RESUME		4
#define CMDERR_OTHER			7

/*** Info about the core being debugged. ***/

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

typedef enum {
	YNM_MAYBE,
	YNM_YES,
	YNM_NO
} yes_no_maybe_t;

#define HART_INDEX_MULTIPLE	-1
#define HART_INDEX_UNKNOWN	-2

typedef struct {
	struct list_head list;
	int abs_chain_position;

	/* The number of harts connected to this DM. */
	int hart_count;
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

	/* This value is incremented every time a dbus access comes back as "busy".
	 * It's used to determine how many run-test/idle cycles to feed the target
	 * in between accesses. */
	unsigned int dmi_busy_delay;

	/* Number of run-test/idle cycles to add between consecutive bus master
	 * reads/writes respectively. */
	unsigned int bus_master_write_delay, bus_master_read_delay;

	/* This value is increased every time we tried to execute two commands
	 * consecutively, and the second one failed because the previous hadn't
	 * completed yet.  It's used to add extra run-test/idle cycles after
	 * starting a command, so we don't have to waste time checking for busy to
	 * go low. */
	unsigned int ac_busy_delay;

	bool abstract_read_csr_supported;
	bool abstract_write_csr_supported;
	bool abstract_read_fpr_supported;
	bool abstract_write_fpr_supported;

	yes_no_maybe_t has_aampostincrement;

	/* When a function returns some error due to a failure indicated by the
	 * target in cmderr, the caller can look here to see what that error was.
	 * (Compare with errno.) */
	uint8_t cmderr;

	/* Some fields from hartinfo. */
	uint8_t datasize;
	uint8_t dataaccess;
	int16_t dataaddr;

	/* The width of the hartsel field. */
	unsigned hartsellen;

	/* DM that provides access to this target. */
	dm013_info_t *dm;

	/* This target was selected using hasel. */
	bool selected;
} riscv013_info_t;

LIST_HEAD(dm_list);

static riscv013_info_t *get_info(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	assert(info);
	assert(info->version_specific);
	return (riscv013_info_t *) info->version_specific;
}

/**
 * Return the DM structure for this target. If there isn't one, find it in the
 * global list of DMs. If it's not in there, then create one and initialize it
 * to 0.
 */
dm013_info_t *get_dm(struct target *target)
{
	RISCV013_INFO(info);
	if (info->dm)
		return info->dm;

	int abs_chain_position = target->tap->abs_chain_position;

	dm013_info_t *entry;
	dm013_info_t *dm = NULL;
	list_for_each_entry(entry, &dm_list, list) {
		if (entry->abs_chain_position == abs_chain_position) {
			dm = entry;
			break;
		}
	}

	if (!dm) {
		LOG_DEBUG("[%d] Allocating new DM", target->coreid);
		dm = calloc(1, sizeof(dm013_info_t));
		if (!dm)
			return NULL;
		dm->abs_chain_position = abs_chain_position;
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

static void decode_dmi(char *text, unsigned address, unsigned data)
{
	static const struct {
		unsigned address;
		uint64_t mask;
		const char *name;
	} description[] = {
		{ DM_DMCONTROL, DM_DMCONTROL_HALTREQ, "haltreq" },
		{ DM_DMCONTROL, DM_DMCONTROL_RESUMEREQ, "resumereq" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTRESET, "hartreset" },
		{ DM_DMCONTROL, DM_DMCONTROL_HASEL, "hasel" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTSELHI, "hartselhi" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTSELLO, "hartsello" },
		{ DM_DMCONTROL, DM_DMCONTROL_NDMRESET, "ndmreset" },
		{ DM_DMCONTROL, DM_DMCONTROL_DMACTIVE, "dmactive" },
		{ DM_DMCONTROL, DM_DMCONTROL_ACKHAVERESET, "ackhavereset" },

		{ DM_DMSTATUS, DM_DMSTATUS_IMPEBREAK, "impebreak" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLHAVERESET, "allhavereset" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYHAVERESET, "anyhavereset" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLRESUMEACK, "allresumeack" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYRESUMEACK, "anyresumeack" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLNONEXISTENT, "allnonexistent" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYNONEXISTENT, "anynonexistent" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLUNAVAIL, "allunavail" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYUNAVAIL, "anyunavail" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLRUNNING, "allrunning" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYRUNNING, "anyrunning" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLHALTED, "allhalted" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYHALTED, "anyhalted" },
		{ DM_DMSTATUS, DM_DMSTATUS_AUTHENTICATED, "authenticated" },
		{ DM_DMSTATUS, DM_DMSTATUS_AUTHBUSY, "authbusy" },
		{ DM_DMSTATUS, DM_DMSTATUS_HASRESETHALTREQ, "hasresethaltreq" },
		{ DM_DMSTATUS, DM_DMSTATUS_CONFSTRPTRVALID, "confstrptrvalid" },
		{ DM_DMSTATUS, DM_DMSTATUS_VERSION, "version" },

		{ DM_ABSTRACTCS, DM_ABSTRACTCS_PROGBUFSIZE, "progbufsize" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_BUSY, "busy" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR, "cmderr" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_DATACOUNT, "datacount" },

		{ DM_COMMAND, DM_COMMAND_CMDTYPE, "cmdtype" },

		{ DM_SBCS, DM_SBCS_SBVERSION, "sbversion" },
		{ DM_SBCS, DM_SBCS_SBBUSYERROR, "sbbusyerror" },
		{ DM_SBCS, DM_SBCS_SBBUSY, "sbbusy" },
		{ DM_SBCS, DM_SBCS_SBREADONADDR, "sbreadonaddr" },
		{ DM_SBCS, DM_SBCS_SBACCESS, "sbaccess" },
		{ DM_SBCS, DM_SBCS_SBAUTOINCREMENT, "sbautoincrement" },
		{ DM_SBCS, DM_SBCS_SBREADONDATA, "sbreadondata" },
		{ DM_SBCS, DM_SBCS_SBERROR, "sberror" },
		{ DM_SBCS, DM_SBCS_SBASIZE, "sbasize" },
		{ DM_SBCS, DM_SBCS_SBACCESS128, "sbaccess128" },
		{ DM_SBCS, DM_SBCS_SBACCESS64, "sbaccess64" },
		{ DM_SBCS, DM_SBCS_SBACCESS32, "sbaccess32" },
		{ DM_SBCS, DM_SBCS_SBACCESS16, "sbaccess16" },
		{ DM_SBCS, DM_SBCS_SBACCESS8, "sbaccess8" },
	};

	text[0] = 0;
	for (unsigned i = 0; i < ARRAY_SIZE(description); i++) {
		if (description[i].address == address) {
			uint64_t mask = description[i].mask;
			unsigned value = get_field(data, mask);
			if (value) {
				if (i > 0)
					*(text++) = ' ';
				if (mask & (mask >> 1)) {
					/* If the field is more than 1 bit wide. */
					sprintf(text, "%s=%d", description[i].name, value);
				} else {
					strcpy(text, description[i].name);
				}
				text += strlen(text);
			}
		}
	}
}

static void dump_field(int idle, const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned int in_op = get_field(in, DTM_DMI_OP);
	unsigned int in_data = get_field(in, DTM_DMI_DATA);
	unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;

	log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, "scan",
			"%db %s %08x @%02x -> %s %08x @%02x; %di",
			field->num_bits, op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address, idle);

	char out_text[500];
	char in_text[500];
	decode_dmi(out_text, out_address, out_data);
	decode_dmi(in_text, in_address, in_data);
	if (in_text[0] || out_text[0]) {
		log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, "scan", "%s -> %s",
				out_text, in_text);
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

static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4] = { 0 };

	if (bscan_tunnel_ir_width != 0)
		return dtmcontrol_scan_via_bscan(target, out);

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

	return in;
}

static void increase_dmi_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->dmi_busy_delay += info->dmi_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcs_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcs_idle, info->dmi_busy_delay,
			info->ac_busy_delay);

	dtmcontrol_scan(target, DTM_DTMCS_DMIRESET);
}

/**
 * exec: If this is set, assume the scan results in an execution, so more
 * run-test/idle cycles may be required.
 */
static dmi_status_t dmi_scan(struct target *target, uint32_t *address_in,
		uint32_t *data_in, dmi_op_t op, uint32_t address_out, uint32_t data_out,
		bool exec)
{
	riscv013_info_t *info = get_info(target);
	RISCV_INFO(r);
	unsigned num_bits = info->abits + DTM_DMI_OP_LENGTH + DTM_DMI_DATA_LENGTH;
	size_t num_bytes = (num_bits + 7) / 8;
	uint8_t in[num_bytes];
	uint8_t out[num_bytes];
	struct scan_field field = {
		.num_bits = num_bits,
		.out_value = out,
		.in_value = in
	};
	riscv_bscan_tunneled_scan_context_t bscan_ctxt;

	if (r->reset_delays_wait >= 0) {
		r->reset_delays_wait--;
		if (r->reset_delays_wait < 0) {
			info->dmi_busy_delay = 0;
			info->ac_busy_delay = 0;
		}
	}

	memset(in, 0, num_bytes);
	memset(out, 0, num_bytes);

	assert(info->abits != 0);

	buf_set_u32(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	buf_set_u32(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	buf_set_u32(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);

	/* I wanted to place this code in a different function, but the way JTAG command
	   queueing works in the jtag handling functions, the scan fields either have to be
	   heap allocated, global/static, or else they need to stay on the stack until
	   the jtag_execute_queue() call.  Heap or static fields in this case doesn't seem
	   the best fit.  Declaring stack based field values in a subsidiary function call wouldn't
	   work. */
	if (bscan_tunnel_ir_width != 0) {
		riscv_add_bscan_tunneled_scan(target, &field, &bscan_ctxt);
	} else {
		/* Assume dbus is already selected. */
		jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	}

	int idle_count = info->dmi_busy_delay;
	if (exec)
		idle_count += info->ac_busy_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		if (data_in)
			*data_in = ~0;
		return DMI_STATUS_FAILED;
	}

	if (bscan_tunnel_ir_width != 0) {
		/* need to right-shift "in" by one bit, because of clock skew between BSCAN TAP and DM TAP */
		buffer_shr(in, num_bytes, 1);
	}

	if (data_in)
		*data_in = buf_get_u32(in, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);

	if (address_in)
		*address_in = buf_get_u32(in, DTM_DMI_ADDRESS_OFFSET, info->abits);
	dump_field(idle_count, &field);
	return buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

/**
 * @param target
 * @param data_in  The data we received from the target.
 * @param dmi_busy_encountered
 *                 If non-NULL, will be updated to reflect whether DMI busy was
 *                 encountered while executing this operation or not.
 * @param dmi_op   The operation to perform (read/write/nop).
 * @param address  The address argument to that operation.
 * @param data_out The data to send to the target.
 * @param timeout_sec
 * @param exec     When true, this scan will execute something, so extra RTI
 *                 cycles may be added.
 * @param ensure_success
 *                 Scan a nop after the requested operation, ensuring the
 *                 DMI operation succeeded.
 */
static int dmi_op_timeout(struct target *target, uint32_t *data_in,
		bool *dmi_busy_encountered, int dmi_op, uint32_t address,
		uint32_t data_out, int timeout_sec, bool exec, bool ensure_success)
{
	select_dmi(target);

	dmi_status_t status;
	uint32_t address_in;

	if (dmi_busy_encountered)
		*dmi_busy_encountered = false;

	const char *op_name;
	switch (dmi_op) {
		case DMI_OP_NOP:
			op_name = "nop";
			break;
		case DMI_OP_READ:
			op_name = "read";
			break;
		case DMI_OP_WRITE:
			op_name = "write";
			break;
		default:
			LOG_ERROR("Invalid DMI operation: %d", dmi_op);
			return ERROR_FAIL;
	}

	keep_alive();

	time_t start = time(NULL);
	/* This first loop performs the request.  Note that if for some reason this
	 * stays busy, it is actually due to the previous access. */
	while (1) {
		status = dmi_scan(target, NULL, NULL, dmi_op, address, data_out,
				exec);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
			if (dmi_busy_encountered)
				*dmi_busy_encountered = true;
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed %s at 0x%x, status=%d", op_name, address, status);
			return ERROR_FAIL;
		}
		if (time(NULL) - start > timeout_sec)
			return ERROR_TIMEOUT_REACHED;
	}

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("Failed %s at 0x%x; status=%d", op_name, address, status);
		return ERROR_FAIL;
	}

	if (ensure_success) {
		/* This second loop ensures the request succeeded, and gets back data.
		 * Note that NOP can result in a 'busy' result as well, but that would be
		 * noticed on the next DMI access we do. */
		while (1) {
			status = dmi_scan(target, &address_in, data_in, DMI_OP_NOP, address, 0,
					false);
			if (status == DMI_STATUS_BUSY) {
				increase_dmi_busy_delay(target);
				if (dmi_busy_encountered)
					*dmi_busy_encountered = true;
			} else if (status == DMI_STATUS_SUCCESS) {
				break;
			} else {
				if (data_in) {
					LOG_ERROR("Failed %s (NOP) at 0x%x; value=0x%x, status=%d",
							op_name, address, *data_in, status);
				} else {
					LOG_ERROR("Failed %s (NOP) at 0x%x; status=%d", op_name, address,
							status);
				}
				return ERROR_FAIL;
			}
			if (time(NULL) - start > timeout_sec)
				return ERROR_TIMEOUT_REACHED;
		}
	}

	return ERROR_OK;
}

static int dmi_op(struct target *target, uint32_t *data_in,
		bool *dmi_busy_encountered, int dmi_op, uint32_t address,
		uint32_t data_out, bool exec, bool ensure_success)
{
	int result = dmi_op_timeout(target, data_in, dmi_busy_encountered, dmi_op,
			address, data_out, riscv_command_timeout_sec, exec, ensure_success);
	if (result == ERROR_TIMEOUT_REACHED) {
		LOG_ERROR("[%s] DMI operation didn't complete in %d seconds. The target is "
				"either really slow or broken. You could increase the "
				"timeout with riscv set_command_timeout_sec.",
				target_name(target), riscv_command_timeout_sec);
		return ERROR_FAIL;
	}
	return result;
}

static int dmi_read(struct target *target, uint32_t *value, uint32_t address)
{
	return dmi_op(target, value, NULL, DMI_OP_READ, address, 0, false, true);
}

static int dmi_read_exec(struct target *target, uint32_t *value, uint32_t address)
{
	return dmi_op(target, value, NULL, DMI_OP_READ, address, 0, true, true);
}

static int dmi_write(struct target *target, uint32_t address, uint32_t value)
{
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, address, value, false, true);
}

static int dmi_write_exec(struct target *target, uint32_t address,
		uint32_t value, bool ensure_success)
{
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, address, value, true, ensure_success);
}

int dmstatus_read_timeout(struct target *target, uint32_t *dmstatus,
		bool authenticated, unsigned timeout_sec)
{
	int result = dmi_op_timeout(target, dmstatus, NULL, DMI_OP_READ,
			DM_DMSTATUS, 0, timeout_sec, false, true);
	if (result != ERROR_OK)
		return result;
	int dmstatus_version = get_field(*dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2 (0.13) and 3 (1.0), not "
				"%d (dmstatus=0x%x). This error might be caused by a JTAG "
				"signal issue. Try reducing the JTAG clock speed.",
				get_field(*dmstatus, DM_DMSTATUS_VERSION), *dmstatus);
	} else if (authenticated && !get_field(*dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", *dmstatus);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int dmstatus_read(struct target *target, uint32_t *dmstatus,
		bool authenticated)
{
	int result = dmstatus_read_timeout(target, dmstatus, authenticated,
			riscv_command_timeout_sec);
	if (result == ERROR_TIMEOUT_REACHED)
		LOG_TARGET_ERROR(target, "DMSTATUS read didn't complete in %d seconds. The target is "
					 "either really slow or broken. You could increase the "
					 "timeout with `riscv set_command_timeout_sec`.",
				 riscv_command_timeout_sec);
	return result;
}

static void increase_ac_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->ac_busy_delay += info->ac_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcs_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcs_idle, info->dmi_busy_delay,
			info->ac_busy_delay);
}

uint32_t abstract_register_size(unsigned width)
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
	RISCV013_INFO(info);
	time_t start = time(NULL);
	while (1) {
		if (dmi_read(target, abstractcs, DM_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(*abstractcs, DM_ABSTRACTCS_BUSY) == 0)
			return ERROR_OK;

		if (time(NULL) - start > riscv_command_timeout_sec) {
			info->cmderr = get_field(*abstractcs, DM_ABSTRACTCS_CMDERR);

			LOG_ERROR("Timed out after %ds waiting for busy to go low (abstractcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					*abstractcs);
			return ERROR_FAIL;
		}
	}
}

static int dm013_select_target(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	return dm013_select_hart(target, info->index);
}

static int execute_abstract_command(struct target *target, uint32_t command)
{
	RISCV013_INFO(info);
	if (debug_level >= LOG_LVL_DEBUG) {
		switch (get_field(command, DM_COMMAND_CMDTYPE)) {
			case 0:
				LOG_DEBUG("command=0x%x; access register, size=%d, postexec=%d, "
						"transfer=%d, write=%d, regno=0x%x",
						command,
						8 << get_field(command, AC_ACCESS_REGISTER_AARSIZE),
						get_field(command, AC_ACCESS_REGISTER_POSTEXEC),
						get_field(command, AC_ACCESS_REGISTER_TRANSFER),
						get_field(command, AC_ACCESS_REGISTER_WRITE),
						get_field(command, AC_ACCESS_REGISTER_REGNO));
				break;
			default:
				LOG_DEBUG("command=0x%x", command);
				break;
		}
	}

	if (dmi_write_exec(target, DM_COMMAND, command, false) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t abstractcs = 0;
	int result = wait_for_idle(target, &abstractcs);

	info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);
	if (info->cmderr != 0 || result != ERROR_OK) {
		LOG_DEBUG("command 0x%x failed; abstractcs=0x%x", command, abstractcs);
		/* Clear the error. */
		dmi_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static riscv_reg_t read_abstract_arg(struct target *target, unsigned index,
		unsigned size_bits)
{
	riscv_reg_t value = 0;
	uint32_t v;
	unsigned offset = index * size_bits / 32;
	switch (size_bits) {
		default:
			LOG_ERROR("Unsupported size: %d bits", size_bits);
			return ~0;
		case 64:
			dmi_read(target, &v, DM_DATA0 + offset + 1);
			value |= ((uint64_t) v) << 32;
			/* falls through */
		case 32:
			dmi_read(target, &v, DM_DATA0 + offset);
			value |= v;
	}
	return value;
}

static int write_abstract_arg(struct target *target, unsigned index,
		riscv_reg_t value, unsigned size_bits)
{
	unsigned offset = index * size_bits / 32;
	switch (size_bits) {
		default:
			LOG_ERROR("Unsupported size: %d bits", size_bits);
			return ERROR_FAIL;
		case 64:
			dmi_write(target, DM_DATA0 + offset + 1, value >> 32);
			/* falls through */
		case 32:
			dmi_write(target, DM_DATA0 + offset, value);
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
			LOG_ERROR("[%s] %d-bit register %s not supported.",
					target_name(target), size, gdb_regno_name(number));
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

static int register_read_abstract(struct target *target, uint64_t *value,
		uint32_t number, unsigned size)
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

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		if (info->cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_read_fpr_supported = false;
				LOG_INFO("Disabling abstract command reads from FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_read_csr_supported = false;
				LOG_INFO("Disabling abstract command reads from CSRs.");
			}
		}
		return result;
	}

	if (value)
		*value = read_abstract_arg(target, 0, size);

	return ERROR_OK;
}

static int register_write_abstract(struct target *target, uint32_t number,
		uint64_t value, unsigned size)
{
	RISCV013_INFO(info);

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_write_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_write_csr_supported)
		return ERROR_FAIL;

	uint32_t command = access_register_command(target, number, size,
			AC_ACCESS_REGISTER_TRANSFER |
			AC_ACCESS_REGISTER_WRITE);

	if (write_abstract_arg(target, 0, value, size) != ERROR_OK)
		return ERROR_FAIL;

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		if (info->cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_write_fpr_supported = false;
				LOG_INFO("Disabling abstract command writes to FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_write_csr_supported = false;
				LOG_INFO("Disabling abstract command writes to CSRs.");
			}
		}
		return result;
	}

	return ERROR_OK;
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
		unsigned width, bool postincrement, bool write)
{
	uint32_t command = set_field(0, AC_ACCESS_MEMORY_CMDTYPE, 2);
	command = set_field(command, AC_ACCESS_MEMORY_AAMVIRTUAL, virtual);
	command |= abstract_memory_size(width);
	command = set_field(command, AC_ACCESS_MEMORY_AAMPOSTINCREMENT,
						postincrement);
	command = set_field(command, AC_ACCESS_MEMORY_WRITE, write);

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
		LOG_INFO("No program buffer present.");
		return ERROR_OK;
	}

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
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
	if (dmi_read(target, &written, DM_PROGBUF0) != ERROR_OK)
		return ERROR_FAIL;
	if (written == (uint32_t) info->progbuf_address) {
		LOG_INFO("progbuf is writable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_YES;

	} else {
		LOG_INFO("progbuf is not writeable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_NO;
	}

	return ERROR_OK;
}

static int is_fpu_reg(uint32_t gdb_regno)
{
	return (gdb_regno >= GDB_REGNO_FPR0 && gdb_regno <= GDB_REGNO_FPR31) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FFLAGS) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FRM) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FCSR);
}

static int is_vector_reg(uint32_t gdb_regno)
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

static int prep_for_register_access(struct target *target, uint64_t *mstatus,
		int regno)
{
	if (is_fpu_reg(regno) || is_vector_reg(regno)) {
		if (register_read_direct(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if (is_fpu_reg(regno) && (*mstatus & MSTATUS_FS) == 0) {
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
						set_field(*mstatus, MSTATUS_FS, 1)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (is_vector_reg(regno) && (*mstatus & MSTATUS_VS) == 0) {
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
						set_field(*mstatus, MSTATUS_VS, 1)) != ERROR_OK)
				return ERROR_FAIL;
		}
	} else {
		*mstatus = 0;
	}
	return ERROR_OK;
}

static int cleanup_after_register_access(struct target *target,
		uint64_t mstatus, int regno)
{
	if ((is_fpu_reg(regno) && (mstatus & MSTATUS_FS) == 0) ||
			(is_vector_reg(regno) && (mstatus & MSTATUS_VS) == 0))
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
			return ERROR_FAIL;
	return ERROR_OK;
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

	LOG_ERROR("Couldn't find %d bytes of scratch RAM to use. Please configure "
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
			if (dmi_read(target, &v, DM_DATA0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DM_DATA1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_PROGBUF:
			if (dmi_read(target, &v, DM_PROGBUF0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DM_PROGBUF1 + scratch->debug_address) != ERROR_OK)
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
			dmi_write(target, DM_DATA0 + scratch->debug_address, value);
			dmi_write(target, DM_DATA1 + scratch->debug_address, value >> 32);
			break;
		case SPACE_DMI_PROGBUF:
			dmi_write(target, DM_PROGBUF0 + scratch->debug_address, value);
			dmi_write(target, DM_PROGBUF1 + scratch->debug_address, value >> 32);
			riscv013_invalidate_cached_debug_buffer(target);
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
static unsigned register_size(struct target *target, unsigned number)
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
 * Immediately write the new value to the requested register. This mechanism
 * bypasses any caches.
 */
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value)
{
	LOG_TARGET_DEBUG(target, "%s <- 0x%" PRIx64, gdb_regno_name(number), value);

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	int result = register_write_abstract(target, number, value,
			register_size(target, number));
	if (result == ERROR_OK || !has_sufficient_progbuf(target, 2))
		return result;

	struct riscv_program program;
	riscv_program_init(&program, target);

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	scratch_mem_t scratch;
	bool use_scratch = false;
	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			riscv_supports_extension(target, 'D') &&
			riscv_xlen(target) < 64) {
		/* There are no instructions to move all the bits from a register, so
		 * we need to use some scratch RAM. */
		use_scratch = true;
		if (riscv_program_insert(&program, fld(number - GDB_REGNO_FPR0, S0, 0))
				!= ERROR_OK)
			return ERROR_FAIL;

		if (scratch_reserve(target, &scratch, &program, 8) != ERROR_OK)
			return ERROR_FAIL;

		if (register_write_direct(target, GDB_REGNO_S0, scratch.hart_address)
				!= ERROR_OK) {
			scratch_release(target, &scratch);
			return ERROR_FAIL;
		}

		if (scratch_write64(target, &scratch, value) != ERROR_OK) {
			scratch_release(target, &scratch);
			return ERROR_FAIL;
		}

	} else {
		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, 'D')) {
				if (riscv_program_insert(&program,
							fmv_d_x(number - GDB_REGNO_FPR0, S0)) != ERROR_OK)
					return ERROR_FAIL;
			} else {
				if (riscv_program_insert(&program,
							fmv_w_x(number - GDB_REGNO_FPR0, S0)) != ERROR_OK)
					return ERROR_FAIL;
			}
		} else if (number == GDB_REGNO_VTYPE) {
			if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, csrr(S1, CSR_VL)) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, vsetvl(ZERO, S1, S0)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (number == GDB_REGNO_VL) {
			/* "The XLEN-bit-wide read-only vl CSR can only be updated by the
			 * vsetvli and vsetvl instructions, and the fault-only-rst vector
			 * load instruction variants." */
			if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, csrr(S1, CSR_VTYPE)) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, vsetvl(ZERO, S0, S1)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			if (riscv_program_csrw(&program, S0, number) != ERROR_OK)
				return ERROR_FAIL;
		} else {
			LOG_ERROR("Unsupported register (enum gdb_regno)(%d)", number);
			return ERROR_FAIL;
		}
		if (register_write_direct(target, GDB_REGNO_S0, value) != ERROR_OK)
			return ERROR_FAIL;
	}

	int exec_out = riscv_program_exec(&program, target);
	/* Don't message on error. Probably the register doesn't exist. */
	if (exec_out == ERROR_OK && target->reg_cache) {
		struct reg *reg = &target->reg_cache->reg_list[number];
		buf_set_u64(reg->value, 0, reg->size, value);
	}

	if (use_scratch)
		scratch_release(target, &scratch);

	if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	return exec_out;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	int result = register_read_abstract(target, value, number,
			register_size(target, number));

	if (result != ERROR_OK &&
			has_sufficient_progbuf(target, 2) &&
			number > GDB_REGNO_XPR31) {
		struct riscv_program program;
		riscv_program_init(&program, target);

		scratch_mem_t scratch;
		bool use_scratch = false;

		if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;

		/* Write program to move data into s0. */

		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, 'D')
					&& riscv_xlen(target) < 64) {
				/* There are no instructions to move all the bits from a
				 * register, so we need to use some scratch RAM. */
				if (riscv_program_insert(&program,
							fsd(number - GDB_REGNO_FPR0, S0, 0)) != ERROR_OK)
					return ERROR_FAIL;
				if (scratch_reserve(target, &scratch, &program, 8) != ERROR_OK)
					return ERROR_FAIL;
				use_scratch = true;

				if (register_write_direct(target, GDB_REGNO_S0,
							scratch.hart_address) != ERROR_OK) {
					scratch_release(target, &scratch);
					return ERROR_FAIL;
				}
			} else if (riscv_supports_extension(target, 'D')) {
				if (riscv_program_insert(&program, fmv_x_d(S0, number - GDB_REGNO_FPR0)) !=
						ERROR_OK)
					return ERROR_FAIL;
			} else {
				if (riscv_program_insert(&program, fmv_x_w(S0, number - GDB_REGNO_FPR0)) !=
						ERROR_OK)
					return ERROR_FAIL;
			}
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			if (riscv_program_csrr(&program, S0, number) != ERROR_OK)
				return ERROR_FAIL;
		} else {
			LOG_ERROR("Unsupported register: %s", gdb_regno_name(number));
			return ERROR_FAIL;
		}

		/* Execute program. */
		result = riscv_program_exec(&program, target);
		/* Don't message on error. Probably the register doesn't exist. */

		if (use_scratch) {
			result = scratch_read64(target, &scratch, value);
			scratch_release(target, &scratch);
			if (result != ERROR_OK)
				return result;
		} else {
			/* Read S0 */
			if (register_read_direct(target, value, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	if (result == ERROR_OK)
		LOG_TARGET_DEBUG(target, "%s = 0x%" PRIx64, gdb_regno_name(number), *value);

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
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out after %ds waiting for authbusy to go low (dmstatus=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					value);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/*** OpenOCD target functions. ***/

static void deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	free(info->version_specific);
	/* TODO: free register arch_info */
	info->version_specific = NULL;
}

typedef enum {
	HALTGROUP,
	RESUMEGROUP
} grouptype_t;
static int set_group(struct target *target, bool *supported, unsigned group, grouptype_t grouptype)
{
	uint32_t write_val = DM_DMCS2_HGWRITE;
	assert(group <= 31);
	write_val = set_field(write_val, DM_DMCS2_GROUP, group);
	write_val = set_field(write_val, DM_DMCS2_GROUPTYPE, (grouptype == HALTGROUP) ? 0 : 1);
	if (dmi_write(target, DM_DMCS2, write_val) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t read_val;
	if (dmi_read(target, &read_val, DM_DMCS2) != ERROR_OK)
		return ERROR_FAIL;
	*supported = get_field(read_val, DM_DMCS2_GROUP) == group;
	return ERROR_OK;
}

static int examine(struct target *target)
{
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	LOG_DEBUG("  dmireset=%d", get_field(dtmcontrol, DTM_DTMCS_DMIRESET));
	LOG_DEBUG("  idle=%d", get_field(dtmcontrol, DTM_DTMCS_IDLE));
	LOG_DEBUG("  dmistat=%d", get_field(dtmcontrol, DTM_DTMCS_DMISTAT));
	LOG_DEBUG("  abits=%d", get_field(dtmcontrol, DTM_DTMCS_ABITS));
	LOG_DEBUG("  version=%d", get_field(dtmcontrol, DTM_DTMCS_VERSION));
	if (dtmcontrol == 0) {
		LOG_ERROR("dtmcontrol is 0. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	if (get_field(dtmcontrol, DTM_DTMCS_VERSION) != 1) {
		LOG_ERROR("[%s] Unsupported DTM version %d. (dtmcontrol=0x%x)",
				target_name(target), get_field(dtmcontrol, DTM_DTMCS_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	riscv013_info_t *info = get_info(target);
	/* TODO: This won't be true if there are multiple DMs. */
	info->index = target->coreid;
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcs_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	/* Reset the Debug Module. */
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (!dm->was_reset) {
		dmi_write(target, DM_DMCONTROL, 0);
		dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_DMACTIVE);
		dm->was_reset = true;

		/* The DM gets reset, so forget any cached progbuf entries. */
		riscv013_invalidate_cached_debug_buffer(target);
	}

	dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_HARTSELLO |
			DM_DMCONTROL_HARTSELHI | DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_HASEL);
	dm->current_hartid = HART_INDEX_UNKNOWN;
	uint32_t dmcontrol;
	if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
		return ERROR_FAIL;
	/* Ensure the HART_INDEX_UNKNOWN is flushed out */
	if (dm013_select_hart(target, 0) != ERROR_OK)
		return ERROR_FAIL;


	if (!get_field(dmcontrol, DM_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("Debug Module did not become active. dmcontrol=0x%x",
				dmcontrol);
		return ERROR_FAIL;
	}

	dm->hasel_supported = get_field(dmcontrol, DM_DMCONTROL_HASEL);

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, false) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("dmstatus:  0x%08x", dmstatus);
	int dmstatus_version = get_field(dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		/* Error was already printed out in dmstatus_read(). */
		return ERROR_FAIL;
	}

	uint32_t hartsel =
		(get_field(dmcontrol, DM_DMCONTROL_HARTSELHI) <<
		 DM_DMCONTROL_HARTSELLO_LENGTH) |
		get_field(dmcontrol, DM_DMCONTROL_HARTSELLO);
	info->hartsellen = 0;
	while (hartsel & 1) {
		info->hartsellen++;
		hartsel >>= 1;
	}
	LOG_DEBUG("hartsellen=%d", info->hartsellen);

	uint32_t hartinfo;
	if (dmi_read(target, &hartinfo, DM_HARTINFO) != ERROR_OK)
		return ERROR_FAIL;

	info->datasize = get_field(hartinfo, DM_HARTINFO_DATASIZE);
	info->dataaccess = get_field(hartinfo, DM_HARTINFO_DATAACCESS);
	info->dataaddr = get_field(hartinfo, DM_HARTINFO_DATAADDR);

	if (!get_field(dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", dmstatus);
		return ERROR_FAIL;
	}

	if (dmi_read(target, &info->sbcs, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;

	/* Check that abstract data registers are accessible. */
	uint32_t abstractcs;
	if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
		return ERROR_FAIL;
	info->datacount = get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT);
	info->progbufsize = get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE);

	LOG_INFO("[%s] datacount=%d progbufsize=%d", target_name(target),
			info->datacount, info->progbufsize);

	RISCV_INFO(r);
	r->impebreak = get_field(dmstatus, DM_DMSTATUS_IMPEBREAK);

	if (!has_sufficient_progbuf(target, 2)) {
		LOG_WARNING("We won't be able to execute fence instructions on this "
				"target. Memory may not always appear consistent. "
				"(progbufsize=%d, impebreak=%d)", info->progbufsize,
				r->impebreak);
	}

	if (info->progbufsize < 4 && riscv_enable_virtual) {
		LOG_ERROR("set_enable_virtual is not available on this target. It "
				"requires a program buffer size of at least 4. (progbufsize=%d) "
				"Use `riscv set_enable_virtual off` to continue."
					, info->progbufsize);
	}

	/* Before doing anything else we must first enumerate the harts. */
	if (dm->hart_count < 0) {
		for (int i = 0; i < MIN(RISCV_MAX_HARTS, 1 << info->hartsellen); ++i) {
			if (dm013_select_hart(target, i) != ERROR_OK)
				return ERROR_FAIL;

			uint32_t s;
			if (dmstatus_read(target, &s, true) != ERROR_OK)
				return ERROR_FAIL;
			if (get_field(s, DM_DMSTATUS_ANYNONEXISTENT))
				break;
			dm->hart_count = i + 1;

			if (get_field(s, DM_DMSTATUS_ANYHAVERESET))
				dmi_write(target, DM_DMCONTROL,
						set_dmcontrol_hartsel(DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_ACKHAVERESET, i));
		}

		LOG_DEBUG("Detected %d harts.", dm->hart_count);
	}

	if (dm->hart_count == 0) {
		LOG_ERROR("No harts found!");
		return ERROR_FAIL;
	}

	/* Don't call any riscv_* functions until after we've counted the number of
	 * cores and initialized registers. */

	if (dm013_select_hart(target, info->index) != ERROR_OK)
		return ERROR_FAIL;

	enum riscv_hart_state state;
	if (riscv_get_hart_state(target, &state) != ERROR_OK)
		return ERROR_FAIL;
	bool halted = (state == RISCV_STATE_HALTED);
	if (!halted) {
		r->prepped = true;
		if (riscv013_halt_go(target) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Fatal: Hart %d failed to halt during examine()",
					info->index);
			return ERROR_FAIL;
		}
	}

	/* Without knowing anything else we can at least mess with the
		* program buffer. */
	r->debug_buffer_size = info->progbufsize;

	int result = register_read_abstract(target, NULL, GDB_REGNO_S0, 64);
	if (result == ERROR_OK)
		r->xlen = 64;
	else
		r->xlen = 32;

	/* Save s0 and s1. The register cache hasn't be initialized yet so we
	 * need to take care of this manually. */
	uint64_t s0, s1;
	if (register_read_direct(target, &s0, GDB_REGNO_S0) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to read s0.");
		return ERROR_FAIL;
	}
	if (register_read_direct(target, &s1, GDB_REGNO_S1) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to read s1.");
		return ERROR_FAIL;
	}

	if (register_read_direct(target, &r->misa, GDB_REGNO_MISA)) {
		LOG_TARGET_ERROR(target, "Fatal: Failed to read MISA.");
		return ERROR_FAIL;
	}

	uint64_t vlenb;
	if (register_read_direct(target, &vlenb, GDB_REGNO_VLENB) != ERROR_OK) {
		if (riscv_supports_extension(target, 'V'))
			LOG_TARGET_WARNING(target, "Couldn't read vlenb; vector register access won't work.");
		r->vlenb = 0;
	} else {
		LOG_TARGET_INFO(target, "Vector support with vlenb=%d", r->vlenb);
		r->vlenb = vlenb;
	}

	/* Now init registers based on what we discovered. */
	if (riscv_init_registers(target) != ERROR_OK)
		return ERROR_FAIL;

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

	if (!halted) {
		riscv013_step_or_resume_current_hart(target, false);
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
	}

	if (target->smp) {
		bool haltgroup_supported;
		if (set_group(target, &haltgroup_supported, target->smp, HALTGROUP) != ERROR_OK)
			return ERROR_FAIL;
		if (haltgroup_supported)
			LOG_INFO("Core %d made part of halt group %d.", target->coreid,
					target->smp);
		else
			LOG_INFO("Core %d could not be made part of halt group %d.",
					target->coreid, target->smp);
	}

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	 * when they can connect with gdb/telnet.
	 * We will need to update those suites if we want to change that text. */
	LOG_TARGET_INFO(target, "Examined RISC-V core; found %d harts",
			riscv_count_harts(target));
	LOG_TARGET_INFO(target, " XLEN=%d, misa=0x%" PRIx64, r->xlen, r->misa);
	return ERROR_OK;
}

static int riscv013_authdata_read(struct target *target, uint32_t *value, unsigned int index)
{
	if (index > 0) {
		LOG_ERROR("Spec 0.13 only has a single authdata register.");
		return ERROR_FAIL;
	}

	if (wait_for_authbusy(target, NULL) != ERROR_OK)
		return ERROR_FAIL;

	return dmi_read(target, value, DM_AUTHDATA);
}

static int riscv013_authdata_write(struct target *target, uint32_t value, unsigned int index)
{
	if (index > 0) {
		LOG_ERROR("Spec 0.13 only has a single authdata register.");
		return ERROR_FAIL;
	}

	uint32_t before, after;
	if (wait_for_authbusy(target, &before) != ERROR_OK)
		return ERROR_FAIL;

	dmi_write(target, DM_AUTHDATA, value);

	if (wait_for_authbusy(target, &after) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(before, DM_DMSTATUS_AUTHENTICATED) &&
			get_field(after, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_INFO("authdata_write resulted in successful authentication");
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

static int riscv013_hart_count(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	assert(dm);
	return dm->hart_count;
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
	LOG_ERROR("Unable to determine supported data bits on this target. Assuming 32 bits.");
	return 32;
}

COMMAND_HELPER(riscv013_print_info, struct target *target)
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

static int prep_for_vector_access(struct target *target, uint64_t *vtype,
		uint64_t *vl, unsigned *debug_vl)
{
	RISCV_INFO(r);
	/* TODO: this continuous save/restore is terrible for performance. */
	/* Write vtype and vl. */
	unsigned encoded_vsew;
	switch (riscv_xlen(target)) {
		case 32:
			encoded_vsew = 2;
			break;
		case 64:
			encoded_vsew = 3;
			break;
		default:
			LOG_ERROR("Unsupported xlen: %d", riscv_xlen(target));
			return ERROR_FAIL;
	}

	/* Save vtype and vl. */
	if (register_read_direct(target, vtype, GDB_REGNO_VTYPE) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read_direct(target, vl, GDB_REGNO_VL) != ERROR_OK)
		return ERROR_FAIL;

	if (register_write_direct(target, GDB_REGNO_VTYPE, encoded_vsew << 3) != ERROR_OK)
		return ERROR_FAIL;
	*debug_vl = DIV_ROUND_UP(r->vlenb * 8, riscv_xlen(target));
	if (register_write_direct(target, GDB_REGNO_VL, *debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int cleanup_after_vector_access(struct target *target, uint64_t vtype,
		uint64_t vl)
{
	/* Restore vtype and vl. */
	if (register_write_direct(target, GDB_REGNO_VTYPE, vtype) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_VL, vl) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int riscv013_get_register_buf(struct target *target,
		uint8_t *value, int regno)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t vtype, vl;
	unsigned debug_vl;
	if (prep_for_vector_access(target, &vtype, &vl, &debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	unsigned vnum = regno - GDB_REGNO_V0;
	unsigned xlen = riscv_xlen(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, vmv_x_s(S0, vnum));
	riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));

	int result = ERROR_OK;
	for (unsigned i = 0; i < debug_vl; i++) {
		/* Executing the program might result in an exception if there is some
		 * issue with the vector implementation/instructions we're using. If that
		 * happens, attempt to restore as usual. We may have clobbered the
		 * vector register we tried to read already.
		 * For other failures, we just return error because things are probably
		 * so messed up that attempting to restore isn't going to help. */
		result = riscv_program_exec(&program, target);
		if (result == ERROR_OK) {
			uint64_t v;
			if (register_read_direct(target, &v, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u64(value, xlen * i, xlen, v);
		} else {
			break;
		}
	}

	if (cleanup_after_vector_access(target, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	if (cleanup_after_register_access(target, mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int riscv013_set_register_buf(struct target *target,
		int regno, const uint8_t *value)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t vtype, vl;
	unsigned debug_vl;
	if (prep_for_vector_access(target, &vtype, &vl, &debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	unsigned vnum = regno - GDB_REGNO_V0;
	unsigned xlen = riscv_xlen(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));
	int result = ERROR_OK;
	for (unsigned i = 0; i < debug_vl; i++) {
		if (register_write_direct(target, GDB_REGNO_S0,
					buf_get_u64(value, xlen * i, xlen)) != ERROR_OK)
			return ERROR_FAIL;
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			break;
	}

	if (cleanup_after_vector_access(target, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	if (cleanup_after_register_access(target, mstatus, regno) != ERROR_OK)
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

static int sb_write_address(struct target *target, target_addr_t address,
							bool ensure_success)
{
	RISCV013_INFO(info);
	unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	/* There currently is no support for >64-bit addresses in OpenOCD. */
	if (sbasize > 96)
		dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS3, 0, false, false);
	if (sbasize > 64)
		dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS2, 0, false, false);
	if (sbasize > 32)
		dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS1, address >> 32, false, false);
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS0, address,
				  false, ensure_success);
}

static int batch_run(const struct target *target, struct riscv_batch *batch)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);
	if (r->reset_delays_wait >= 0) {
		r->reset_delays_wait -= batch->used_scans;
		if (r->reset_delays_wait <= 0) {
			batch->idle_count = 0;
			info->dmi_busy_delay = 0;
			info->ac_busy_delay = 0;
		}
	}
	return riscv_batch_run(batch);
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
		LOG_ERROR("Memory sampling is only implemented for sbasize <= 64.");
		return ERROR_NOT_IMPLEMENTED;
	}

	if (get_field(info->sbcs, DM_SBCS_SBVERSION) != 1) {
		LOG_ERROR("Memory sampling is only implemented for SBA version 1.");
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
			target, 1 + enabled_count * 5 * repeat,
			info->dmi_busy_delay + info->bus_master_read_delay);
		if (!batch)
			return ERROR_FAIL;

		unsigned int result_bytes = 0;
		for (unsigned int n = 0; n < repeat; n++) {
			for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
				if (config->bucket[i].enabled) {
					if (!sba_supports_access(target, config->bucket[i].size_bytes)) {
						LOG_ERROR("Hardware does not support SBA access for %d-byte memory sampling.",
								config->bucket[i].size_bytes);
						return ERROR_NOT_IMPLEMENTED;
					}

					uint32_t sbcs_write = DM_SBCS_SBREADONADDR;
					if (enabled_count == 1)
						sbcs_write |= DM_SBCS_SBREADONDATA;
					sbcs_write |= sb_sbaccess(config->bucket[i].size_bytes);
					if (!sbcs_valid || sbcs_write != sbcs) {
						riscv_batch_add_dmi_write(batch, DM_SBCS, sbcs_write, true);
						sbcs = sbcs_write;
						sbcs_valid = true;
					}

					if (sbasize > 32 &&
							(!sbaddress1_valid ||
							sbaddress1 != config->bucket[i].address >> 32)) {
						sbaddress1 = config->bucket[i].address >> 32;
						riscv_batch_add_dmi_write(batch, DM_SBADDRESS1, sbaddress1, true);
						sbaddress1_valid = true;
					}
					if (!sbaddress0_valid ||
							sbaddress0 != (config->bucket[i].address & 0xffffffff)) {
						sbaddress0 = config->bucket[i].address;
						riscv_batch_add_dmi_write(batch, DM_SBADDRESS0, sbaddress0, true);
						sbaddress0_valid = true;
					}
					if (config->bucket[i].size_bytes > 4)
						riscv_batch_add_dmi_read(batch, DM_SBDATA1);
					riscv_batch_add_dmi_read(batch, DM_SBDATA0);
					result_bytes += 1 + config->bucket[i].size_bytes;
				}
			}
		}

		if (buf->used + result_bytes >= buf->size) {
			riscv_batch_free(batch);
			break;
		}

		size_t sbcs_read_index = riscv_batch_add_dmi_read(batch, DM_SBCS);

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
			increase_dmi_busy_delay(target);
			continue;
		}

		uint32_t sbcs_read = riscv_batch_get_dmi_read_data(batch, sbcs_read_index);
		if (get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			/* Discard this batch when we encounter "busy error" state on the System Bus level.
			 * We'll try next time with a larger System Bus read delay. */
			info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
			dmi_write(target, DM_SBCS, sbcs_read | DM_SBCS_SBBUSYERROR | DM_SBCS_SBERROR);
			riscv_batch_free(batch);
			continue;
		}
		if (get_field(sbcs_read, DM_SBCS_SBERROR)) {
			/* The memory we're sampling was unreadable, somehow. Give up. */
			dmi_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR | DM_SBCS_SBERROR);
			riscv_batch_free(batch);
			return ERROR_FAIL;
		}

		unsigned int read = 0;
		for (unsigned int n = 0; n < repeat; n++) {
			for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
				if (config->bucket[i].enabled) {
					assert(i < RISCV_SAMPLE_BUF_TIMESTAMP_BEFORE);
					uint64_t value = 0;
					if (config->bucket[i].size_bytes > 4)
						value = ((uint64_t)riscv_batch_get_dmi_read_data(batch, read++)) << 32;
					value |= riscv_batch_get_dmi_read_data(batch, read++);

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
		/* TODO: Can we make this more obvious to eg. a gdb user? */
		uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_ACKHAVERESET;
		dmcontrol = set_dmcontrol_hartsel(dmcontrol, info->index);
		/* If we had been halted when we reset, request another halt. If we
		 * ended up running out of reset, then the user will (hopefully) get a
		 * message that a reset happened, that the target is running, and then
		 * that it is halted again once the request goes through.
		 */
		if (target->state == TARGET_HALTED)
			dmcontrol |= DM_DMCONTROL_HALTREQ;
		dmi_write(target, DM_DMCONTROL, dmcontrol);
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

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	RISCV_INFO(generic_info);

	generic_info->get_register = &riscv013_get_register;
	generic_info->set_register = &riscv013_set_register;
	generic_info->get_register_buf = &riscv013_get_register_buf;
	generic_info->set_register_buf = &riscv013_set_register_buf;
	generic_info->select_target = &dm013_select_target;
	generic_info->get_hart_state = &riscv013_get_hart_state;
	generic_info->resume_go = &riscv013_resume_go;
	generic_info->step_current_hart = &riscv013_step_current_hart;
	generic_info->on_halt = &riscv013_on_halt;
	generic_info->resume_prep = &riscv013_resume_prep;
	generic_info->halt_prep = &riscv013_halt_prep;
	generic_info->halt_go = &riscv013_halt_go;
	generic_info->on_step = &riscv013_on_step;
	generic_info->halt_reason = &riscv013_halt_reason;
	generic_info->read_debug_buffer = &riscv013_read_debug_buffer;
	generic_info->write_debug_buffer = &riscv013_write_debug_buffer;
	generic_info->execute_debug_buffer = &riscv013_execute_debug_buffer;
	generic_info->invalidate_cached_debug_buffer = &riscv013_invalidate_cached_debug_buffer;
	generic_info->fill_dmi_write_u64 = &riscv013_fill_dmi_write_u64;
	generic_info->fill_dmi_read_u64 = &riscv013_fill_dmi_read_u64;
	generic_info->fill_dmi_nop_u64 = &riscv013_fill_dmi_nop_u64;
	generic_info->dmi_write_u64_bits = &riscv013_dmi_write_u64_bits;
	generic_info->authdata_read = &riscv013_authdata_read;
	generic_info->authdata_write = &riscv013_authdata_write;
	generic_info->dmi_read = &dmi_read;
	generic_info->dmi_write = &dmi_write;
	generic_info->read_memory = read_memory;
	generic_info->hart_count = &riscv013_hart_count;
	generic_info->data_bits = &riscv013_data_bits;
	generic_info->print_info = &riscv013_print_info;
	if (!generic_info->version_specific) {
		generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
		if (!generic_info->version_specific)
			return ERROR_FAIL;
	}
	generic_info->sample_memory = sample_memory;
	riscv013_info_t *info = get_info(target);

	info->progbufsize = -1;

	info->dmi_busy_delay = 0;
	info->bus_master_read_delay = 0;
	info->bus_master_write_delay = 0;
	info->ac_busy_delay = 0;

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

	select_dmi(target);

	uint32_t control_base = set_field(0, DM_DMCONTROL_DMACTIVE, 1);

	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		/* Run the user-supplied script if there is one. */
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	} else if (target->rtos) {
		/* There's only one target, and OpenOCD thinks each hart is a thread.
		 * We must reset them all. */

		/* TODO: Try to use hasel in dmcontrol */

		/* Set haltreq for each hart. */
		uint32_t control = control_base;

		control = set_dmcontrol_hartsel(control_base, info->index);
		control = set_field(control, DM_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		dmi_write(target, DM_DMCONTROL, control);

		/* Assert ndmreset */
		control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DM_DMCONTROL, control);

	} else {
		/* Reset just this hart. */
		uint32_t control = set_dmcontrol_hartsel(control_base, info->index);
		control = set_field(control, DM_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DM_DMCONTROL, control);
	}

	target->state = TARGET_RESET;

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	/* The DM might have gotten reset if OpenOCD called us in some reset that
	 * involves SRST being toggled. So clear our cache which may be out of
	 * date. */
	riscv013_invalidate_cached_debug_buffer(target);

	return ERROR_OK;
}

static int deassert_reset(struct target *target)
{
	RISCV013_INFO(info);
	select_dmi(target);

	/* Clear the reset, but make sure haltreq is still set */
	uint32_t control = 0;
	control = set_field(control, DM_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
	control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);
	dmi_write(target, DM_DMCONTROL, set_dmcontrol_hartsel(control, info->index));

	uint32_t dmstatus;
	int dmi_busy_delay = info->dmi_busy_delay;
	time_t start = time(NULL);

	for (unsigned int i = 0; i < riscv_count_harts(target); ++i) {
		unsigned int index = i;
		if (target->rtos) {
			if (index != info->index)
				continue;
			dmi_write(target, DM_DMCONTROL,
					set_dmcontrol_hartsel(control, index));
		} else {
			index = info->index;
		}

		LOG_DEBUG("Waiting for hart %d to come out of reset.", index);
		while (1) {
			int result = dmstatus_read_timeout(target, &dmstatus, true,
					riscv_reset_timeout_sec);
			if (result == ERROR_TIMEOUT_REACHED)
				LOG_ERROR("Hart %d didn't complete a DMI read coming out of "
						"reset in %ds; Increase the timeout with riscv "
						"set_reset_timeout_sec.",
						index, riscv_reset_timeout_sec);
			if (result != ERROR_OK)
				return result;
			/* Certain debug modules, like the one in GD32VF103
			 * MCUs, violate the specification's requirement that
			 * each hart is in "exactly one of four states" and,
			 * during reset, report harts as both unavailable and
			 * halted/running. To work around this, we check for
			 * the absence of the unavailable state rather than
			 * the presence of any other state. */
			if (!get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL))
				break;
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart %d didn't leave reset in %ds; "
						"dmstatus=0x%x; "
						"Increase the timeout with riscv set_reset_timeout_sec.",
						index, riscv_reset_timeout_sec, dmstatus);
				return ERROR_FAIL;
			}
		}
		if (target->reset_halt) {
			target->state = TARGET_HALTED;
			target->debug_reason = DBG_REASON_DBGRQ;
		} else {
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}

		if (get_field(dmstatus, DM_DMSTATUS_ALLHAVERESET)) {
			/* Ack reset. */
			dmi_write(target, DM_DMCONTROL,
					set_dmcontrol_hartsel(control, index) |
					DM_DMCONTROL_ACKHAVERESET);
		}

		if (!target->rtos)
			break;
	}
	info->dmi_busy_delay = dmi_busy_delay;
	return ERROR_OK;
}

static int execute_fence(struct target *target)
{
	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	/* FIXME: For non-coherent systems we need to flush the caches right
	 * here, but there's no ISA-defined way of doing that. */
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence_i(&program);
	riscv_program_fence(&program);
	int result = riscv_program_exec(&program, target);
	if (result != ERROR_OK)
		LOG_TARGET_DEBUG(target, "Unable to execute pre-fence");

	return ERROR_OK;
}

static void log_memory_access(target_addr_t address, uint64_t value,
		unsigned size_bytes, bool read)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char fmt[80];
	sprintf(fmt, "M[0x%" TARGET_PRIxADDR "] %ss 0x%%0%d" PRIx64,
			address, read ? "read" : "write", size_bytes * 2);
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

/* Read the relevant sbdata regs depending on size, and put the results into
 * buffer. */
static int read_memory_bus_word(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	uint32_t value;
	int result;
	static int sbdata[4] = { DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3 };
	assert(size <= 16);
	for (int i = (size - 1) / 4; i >= 0; i--) {
		result = dmi_op(target, &value, NULL, DMI_OP_READ, sbdata[i], 0, false, true);
		if (result != ERROR_OK)
			return result;
		buf_set_u32(buffer + i * 4, 0, 8 * MIN(size, 4), value);
		log_memory_access(address + i * 4, value, MIN(size, 4), true);
	}
	return ERROR_OK;
}

static target_addr_t sb_read_address(struct target *target)
{
	RISCV013_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	target_addr_t address = 0;
	uint32_t v;
	if (sbasize > 32) {
		dmi_read(target, &v, DM_SBADDRESS1);
		address |= v;
		address <<= 32;
	}
	dmi_read(target, &v, DM_SBADDRESS0);
	address |= v;
	return address;
}

static int read_sbcs_nonbusy(struct target *target, uint32_t *sbcs)
{
	time_t start = time(NULL);
	while (1) {
		if (dmi_read(target, sbcs, DM_SBCS) != ERROR_OK)
			return ERROR_FAIL;
		if (!get_field(*sbcs, DM_SBCS_SBBUSY))
			return ERROR_OK;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec, *sbcs);
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
		LOG_ERROR("sba v0 reads only support size==increment");
		return ERROR_NOT_IMPLEMENTED;
	}

	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
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
			if (dmi_read(target, &access, DM_SBCS) != ERROR_OK)
				return ERROR_FAIL;
			dmi_write(target, DM_SBADDRESS0, cur_addr);
			/* size/2 matching the bit access of the spec 0.13 */
			access = set_field(access, DM_SBCS_SBACCESS, size/2);
			access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
			LOG_DEBUG("\r\nread_memory: sab: access:  0x%08x", access);
			dmi_write(target, DM_SBCS, access);
			/* 3) read */
			uint32_t value;
			if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			LOG_DEBUG("\r\nread_memory: sab: value:  0x%08x", value);
			buf_set_u32(t_buffer, 0, 8 * size, value);
			t_buffer += size;
			cur_addr += size;
		}
		return ERROR_OK;
	}

	/* has to be the same size if we want to read a block */
	LOG_DEBUG("reading block until final address 0x%" PRIx64, fin_addr);
	if (dmi_read(target, &access, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;
	/* set current address */
	dmi_write(target, DM_SBADDRESS0, cur_addr);
	/* 2) write sbaccess=2, sbsingleread,sbautoread,sbautoincrement
	 * size/2 matching the bit access of the spec 0.13 */
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOREAD, 1);
	access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08x", access);
	dmi_write(target, DM_SBCS, access);

	while (cur_addr < fin_addr) {
		LOG_DEBUG("\r\nsab:autoincrement: \r\n size: %d\tcount:%d\taddress: 0x%08"
				PRIx64, size, count, cur_addr);
		/* read */
		uint32_t value;
		if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
			return ERROR_FAIL;
		buf_set_u32(t_buffer, 0, 8 * size, value);
		cur_addr += size;
		t_buffer += size;

		/* if we are reaching last address, we must clear autoread */
		if (cur_addr == fin_addr && count != 1) {
			dmi_write(target, DM_SBCS, 0);
			if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u32(t_buffer, 0, 8 * size, value);
		}
	}

	uint32_t sbcs;
	if (dmi_read(target, &sbcs, DM_SBCS) != ERROR_OK)
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
		LOG_ERROR("sba v1 reads only support increment of size or 0");
		return ERROR_NOT_IMPLEMENTED;
	}

	RISCV013_INFO(info);
	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	while (next_address < end_address) {
		uint32_t sbcs_write = set_field(0, DM_SBCS_SBREADONADDR, 1);
		sbcs_write |= sb_sbaccess(size);
		if (increment == size)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBAUTOINCREMENT, 1);
		if (count > 1)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, count > 1);
		if (dmi_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
			return ERROR_FAIL;

		/* This address write will trigger the first read. */
		if (sb_write_address(target, next_address, true) != ERROR_OK)
			return ERROR_FAIL;

		if (info->bus_master_read_delay) {
			jtag_add_runtest(info->bus_master_read_delay, TAP_IDLE);
			if (jtag_execute_queue() != ERROR_OK) {
				LOG_ERROR("Failed to scan idle sequence");
				return ERROR_FAIL;
			}
		}

		/* First value has been read, and is waiting for us to issue a DMI read
		 * to get it. */

		static int sbdata[4] = {DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3};
		assert(size <= 16);
		target_addr_t next_read = address - 1;
		for (uint32_t i = (next_address - address) / size; i < count - 1; i++) {
			for (int j = (size - 1) / 4; j >= 0; j--) {
				uint32_t value;
				unsigned attempt = 0;
				while (1) {
					if (attempt++ > 100) {
						LOG_ERROR("DMI keeps being busy in while reading memory just past " TARGET_ADDR_FMT,
								  next_read);
						return ERROR_FAIL;
					}
					keep_alive();
					dmi_status_t status = dmi_scan(target, NULL, &value,
												   DMI_OP_READ, sbdata[j], 0, false);
					if (status == DMI_STATUS_BUSY)
						increase_dmi_busy_delay(target);
					else if (status == DMI_STATUS_SUCCESS)
						break;
					else
						return ERROR_FAIL;
				}
				if (next_read != address - 1) {
					buf_set_u32(buffer + next_read - address, 0, 8 * MIN(size, 4), value);
					log_memory_access(next_read, value, MIN(size, 4), true);
				}
				next_read = address + i * size + j * 4;
			}
		}

		uint32_t sbcs_read = 0;
		if (count > 1) {
			uint32_t value;
			unsigned attempt = 0;
			while (1) {
				if (attempt++ > 100) {
					LOG_ERROR("DMI keeps being busy in while reading memory just past " TARGET_ADDR_FMT,
								next_read);
					return ERROR_FAIL;
				}
				dmi_status_t status = dmi_scan(target, NULL, &value, DMI_OP_NOP, 0, 0, false);
				if (status == DMI_STATUS_BUSY)
					increase_dmi_busy_delay(target);
				else if (status == DMI_STATUS_SUCCESS)
					break;
				else
					return ERROR_FAIL;
			}
			buf_set_u32(buffer + next_read - address, 0, 8 * MIN(size, 4), value);
			log_memory_access(next_read, value, MIN(size, 4), true);

			/* "Writes to sbcs while sbbusy is high result in undefined behavior.
			 * A debugger must not write to sbcs until it reads sbbusy as 0." */
			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;

			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, 0);
			if (dmi_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
				return ERROR_FAIL;
		}

		/* Read the last word, after we disabled sbreadondata if necessary. */
		if (!get_field(sbcs_read, DM_SBCS_SBERROR) &&
				!get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			if (read_memory_bus_word(target, address + (count - 1) * size, size,
						buffer + (count - 1) * size) != ERROR_OK)
				return ERROR_FAIL;

			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again. */
			if (dmi_write(target, DM_SBCS, sbcs_read | DM_SBCS_SBBUSYERROR) != ERROR_OK)
				return ERROR_FAIL;
			next_address = sb_read_address(target);
			info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
			continue;
		}

		unsigned error = get_field(sbcs_read, DM_SBCS_SBERROR);
		if (error == 0) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			if (dmi_write(target, DM_SBCS, DM_SBCS_SBERROR) != ERROR_OK)
				return ERROR_FAIL;
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static void log_mem_access_result(struct target *target, bool success, int method, bool read)
{
	RISCV_INFO(r);
	bool warn = false;
	char msg[60];

	/* Compose the message */
	snprintf(msg, 60, "%s to %s memory via %s.",
			success ? "Succeeded" : "Failed",
			read ? "read" : "write",
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
		LOG_WARNING("%s", msg);
	else
		LOG_DEBUG("%s", msg);
}

static bool mem_should_skip_progbuf(struct target *target, target_addr_t address,
		uint32_t size, bool read, char **skip_reason)
{
	assert(skip_reason);

	if (!has_sufficient_progbuf(target, 3)) {
		LOG_DEBUG("Skipping mem %s via progbuf - insufficient progbuf size.",
				read ? "read" : "write");
		*skip_reason = "skipped (insufficient progbuf)";
		return true;
	}
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("Skipping mem %s via progbuf - target not halted.",
				read ? "read" : "write");
		*skip_reason = "skipped (target not halted)";
		return true;
	}
	if (riscv_xlen(target) < size * 8) {
		LOG_DEBUG("Skipping mem %s via progbuf - XLEN (%d) is too short for %d-bit memory access.",
				read ? "read" : "write", riscv_xlen(target), size * 8);
		*skip_reason = "skipped (XLEN too short)";
		return true;
	}
	if (size > 8) {
		LOG_DEBUG("Skipping mem %s via progbuf - unsupported size.",
				read ? "read" : "write");
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	if ((sizeof(address) * 8 > riscv_xlen(target)) && (address >> riscv_xlen(target))) {
		LOG_DEBUG("Skipping mem %s via progbuf - progbuf only supports %u-bit address.",
				read ? "read" : "write", riscv_xlen(target));
		*skip_reason = "skipped (too large address)";
		return true;
	}

	return false;
}

static bool mem_should_skip_sysbus(struct target *target, target_addr_t address,
		uint32_t size, uint32_t increment, bool read, char **skip_reason)
{
	assert(skip_reason);

	RISCV013_INFO(info);
	if (!sba_supports_access(target, size)) {
		LOG_DEBUG("Skipping mem %s via system bus - unsupported size.",
				read ? "read" : "write");
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	if ((sizeof(address) * 8 > sbasize) && (address >> sbasize)) {
		LOG_DEBUG("Skipping mem %s via system bus - sba only supports %u-bit address.",
				read ? "read" : "write", sbasize);
		*skip_reason = "skipped (too large address)";
		return true;
	}
	if (read && increment != size && (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0 || increment != 0)) {
		LOG_DEBUG("Skipping mem read via system bus - "
				"sba reads only support size==increment or also size==0 for sba v1.");
		*skip_reason = "skipped (unsupported increment)";
		return true;
	}

	return false;
}

static bool mem_should_skip_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t increment, bool read, char **skip_reason)
{
	assert(skip_reason);

	if (size > 8) {
		/* TODO: Add 128b support if it's ever used. Involves modifying
				 read/write_abstract_arg() to work on two 64b values. */
		LOG_DEBUG("Skipping mem %s via abstract access - unsupported size: %d bits",
				read ? "read" : "write", size * 8);
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	if ((sizeof(address) * 8 > riscv_xlen(target)) && (address >> riscv_xlen(target))) {
		LOG_DEBUG("Skipping mem %s via abstract access - abstract access only supports %u-bit address.",
				read ? "read" : "write", riscv_xlen(target));
		*skip_reason = "skipped (too large address)";
		return true;
	}
	if (read && size != increment) {
		LOG_ERROR("Skipping mem read via abstract access - "
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

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
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
				LOG_ERROR("Failed to write arg1 during read_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);

		if (info->has_aampostincrement == YNM_MAYBE) {
			if (result == ERROR_OK) {
				/* Safety: double-check that the address was really auto-incremented */
				riscv_reg_t new_address = read_abstract_arg(target, 1, riscv_xlen(target));
				if (new_address == address + size) {
					LOG_DEBUG("aampostincrement is supported on this target.");
					info->has_aampostincrement = YNM_YES;
				} else {
					LOG_WARNING("Buggy aampostincrement! Address not incremented correctly.");
					info->has_aampostincrement = YNM_NO;
				}
			} else {
				/* Try the same access but with postincrement disabled. */
				command = access_memory_command(target, false, width, false, false);
				result = execute_abstract_command(target, command);
				if (result == ERROR_OK) {
					LOG_DEBUG("aampostincrement is not supported on this target.");
					info->has_aampostincrement = YNM_NO;
				}
			}
		}

		if (result != ERROR_OK)
			return result;

		/* Copy arg0 to buffer (rounded width up to nearest 32) */
		riscv_reg_t value = read_abstract_arg(target, 0, width32);
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

	LOG_DEBUG("writing %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
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
			LOG_ERROR("Failed to write arg0 during write_memory_abstract().");
			return result;
		}

		/* Update the address if it is the first time or aampostincrement is not supported by the target. */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address + c * size, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_ERROR("Failed to write arg1 during write_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);

		if (info->has_aampostincrement == YNM_MAYBE) {
			if (result == ERROR_OK) {
				/* Safety: double-check that the address was really auto-incremented */
				riscv_reg_t new_address = read_abstract_arg(target, 1, riscv_xlen(target));
				if (new_address == address + size) {
					LOG_DEBUG("aampostincrement is supported on this target.");
					info->has_aampostincrement = YNM_YES;
				} else {
					LOG_WARNING("Buggy aampostincrement! Address not incremented correctly.");
					info->has_aampostincrement = YNM_NO;
				}
			} else {
				/* Try the same access but with postincrement disabled. */
				command = access_memory_command(target, false, width, false, true);
				result = execute_abstract_command(target, command);
				if (result == ERROR_OK) {
					LOG_DEBUG("aampostincrement is not supported on this target.");
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
 * Read the requested memory, taking care to execute every read exactly once,
 * even if cmderr=busy is encountered.
 */
static int read_memory_progbuf_inner(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	RISCV013_INFO(info);

	int result = ERROR_OK;

	/* Write address to S0. */
	result = register_write_direct(target, GDB_REGNO_S0, address);
	if (result != ERROR_OK)
		return result;

	if (increment == 0 &&
			register_write_direct(target, GDB_REGNO_S2, 0) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t command = access_register_command(target, GDB_REGNO_S1,
			riscv_xlen(target),
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	if (execute_abstract_command(target, command) != ERROR_OK)
		return ERROR_FAIL;

	/* First read has just triggered. Result is in s1. */
	if (count == 1) {
		uint64_t value;
		if (register_read_direct(target, &value, GDB_REGNO_S1) != ERROR_OK)
			return ERROR_FAIL;
		buf_set_u64(buffer, 0, 8 * size, value);
		log_memory_access(address, value, size, true);
		return ERROR_OK;
	}

	if (dmi_write(target, DM_ABSTRACTAUTO,
			1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET) != ERROR_OK)
		goto error;
	/* Read garbage from dmi_data0, which triggers another execution of the
	 * program. Now dmi_data0 contains the first good result, and s1 the next
	 * memory value. */
	if (dmi_read_exec(target, NULL, DM_DATA0) != ERROR_OK)
		goto error;

	/* read_addr is the next address that the hart will read from, which is the
	 * value in s0. */
	unsigned index = 2;
	while (index < count) {
		riscv_addr_t read_addr = address + index * increment;
		LOG_DEBUG("i=%d, count=%d, read_addr=0x%" PRIx64, index, count, read_addr);
		/* The pipeline looks like this:
		 * memory -> s1 -> dm_data0 -> debugger
		 * Right now:
		 * s0 contains read_addr
		 * s1 contains mem[read_addr-size]
		 * dm_data0 contains[read_addr-size*2]
		 */

		struct riscv_batch *batch = riscv_batch_alloc(target, RISCV_BATCH_ALLOC_SIZE,
				info->dmi_busy_delay + info->ac_busy_delay);
		if (!batch)
			return ERROR_FAIL;

		unsigned reads = 0;
		for (unsigned j = index; j < count; j++) {
			if (size > 4)
				riscv_batch_add_dmi_read(batch, DM_DATA1);
			riscv_batch_add_dmi_read(batch, DM_DATA0);

			reads++;
			if (riscv_batch_full(batch))
				break;
		}

		batch_run(target, batch);

		/* Wait for the target to finish performing the last abstract command,
		 * and update our copy of cmderr. If we see that DMI is busy here,
		 * dmi_busy_delay will be incremented. */
		uint32_t abstractcs;
		if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;
		while (get_field(abstractcs, DM_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);

		unsigned next_index;
		unsigned ignore_last = 0;
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory read");
				next_index = index + reads;
				break;
			case CMDERR_BUSY:
				LOG_DEBUG("memory read resulted in busy response");

				increase_ac_busy_delay(target);
				riscv013_clear_abstract_error(target);

				dmi_write(target, DM_ABSTRACTAUTO, 0);

				uint32_t dmi_data0, dmi_data1 = 0;
				/* This is definitely a good version of the value that we
				 * attempted to read when we discovered that the target was
				 * busy. */
				if (dmi_read(target, &dmi_data0, DM_DATA0) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}
				if (size > 4 && dmi_read(target, &dmi_data1, DM_DATA1) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* See how far we got, clobbering dmi_data0. */
				if (increment == 0) {
					uint64_t counter;
					result = register_read_direct(target, &counter, GDB_REGNO_S2);
					next_index = counter;
				} else {
					uint64_t next_read_addr;
					result = register_read_direct(target, &next_read_addr,
												  GDB_REGNO_S0);
					next_index = (next_read_addr - address) / increment;
				}
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				uint64_t value64 = (((uint64_t)dmi_data1) << 32) | dmi_data0;
				buf_set_u64(buffer + (next_index - 2) * size, 0, 8 * size, value64);
				log_memory_access(address + (next_index - 2) * size, value64, size, true);

				/* Restore the command, and execute it.
				 * Now DM_DATA0 contains the next value just as it would if no
				 * error had occurred. */
				dmi_write_exec(target, DM_COMMAND, command, true);
				next_index++;

				dmi_write(target, DM_ABSTRACTAUTO,
						1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				ignore_last = 1;

				break;
			default:
				LOG_DEBUG("error when reading memory, abstractcs=0x%08lx", (long)abstractcs);
				riscv013_clear_abstract_error(target);
				riscv_batch_free(batch);
				result = ERROR_FAIL;
				goto error;
		}

		/* Now read whatever we got out of the batch. */
		dmi_status_t status = DMI_STATUS_SUCCESS;
		unsigned read = 0;
		assert(index >= 2);
		for (unsigned j = index - 2; j < index + reads; j++) {
			assert(j < count);
			LOG_DEBUG("index=%d, reads=%d, next_index=%d, ignore_last=%d, j=%d",
				index, reads, next_index, ignore_last, j);
			if (j + 3 + ignore_last > next_index)
				break;

			status = riscv_batch_get_dmi_read_op(batch, read);
			uint64_t value = riscv_batch_get_dmi_read_data(batch, read);
			read++;
			if (status != DMI_STATUS_SUCCESS) {
				/* If we're here because of busy count, dmi_busy_delay will
				 * already have been increased and busy state will have been
				 * cleared in dmi_read(). */
				/* In at least some implementations, we issue a read, and then
				 * can get busy back when we try to scan out the read result,
				 * and the actual read value is lost forever. Since this is
				 * rare in any case, we return error here and rely on our
				 * caller to reread the entire block. */
				LOG_WARNING("Batch memory read encountered DMI error %d. "
						"Falling back on slower reads.", status);
				riscv_batch_free(batch);
				result = ERROR_FAIL;
				goto error;
			}
			if (size > 4) {
				status = riscv_batch_get_dmi_read_op(batch, read);
				if (status != DMI_STATUS_SUCCESS) {
					LOG_WARNING("Batch memory read encountered DMI error %d. "
							"Falling back on slower reads.", status);
					riscv_batch_free(batch);
					result = ERROR_FAIL;
					goto error;
				}
				value <<= 32;
				value |= riscv_batch_get_dmi_read_data(batch, read);
				read++;
			}
			riscv_addr_t offset = j * size;
			buf_set_u64(buffer + offset, 0, 8 * size, value);
			log_memory_access(address + j * increment, value, size, true);
		}

		index = next_index;

		riscv_batch_free(batch);
	}

	dmi_write(target, DM_ABSTRACTAUTO, 0);

	if (count > 1) {
		/* Read the penultimate word. */
		uint32_t dmi_data0, dmi_data1 = 0;
		if (dmi_read(target, &dmi_data0, DM_DATA0) != ERROR_OK)
			return ERROR_FAIL;
		if (size > 4 && dmi_read(target, &dmi_data1, DM_DATA1) != ERROR_OK)
			return ERROR_FAIL;
		uint64_t value64 = (((uint64_t)dmi_data1) << 32) | dmi_data0;
		buf_set_u64(buffer + size * (count - 2), 0, 8 * size, value64);
		log_memory_access(address + size * (count - 2), value64, size, true);
	}

	/* Read the last word. */
	uint64_t value;
	result = register_read_direct(target, &value, GDB_REGNO_S1);
	if (result != ERROR_OK)
		goto error;
	buf_set_u64(buffer + size * (count-1), 0, 8 * size, value);
	log_memory_access(address + size * (count-1), value, size, true);

	return ERROR_OK;

error:
	dmi_write(target, DM_ABSTRACTAUTO, 0);

	return result;
}

/* Only need to save/restore one GPR to read a single word, and the progbuf
 * program doesn't need to increment. */
static int read_memory_progbuf_one(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	int result = ERROR_FAIL;

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		goto restore_mstatus;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_ldr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			goto restore_mstatus;
	}
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	if (riscv_program_ebreak(&program) != ERROR_OK)
		goto restore_mstatus;
	if (riscv_program_write(&program) != ERROR_OK)
		goto restore_mstatus;

	/* Write address to S0, and execute buffer. */
	if (write_abstract_arg(target, 0, address, riscv_xlen(target)) != ERROR_OK)
		goto restore_mstatus;
	uint32_t command = access_register_command(target, GDB_REGNO_S0,
			riscv_xlen(target), AC_ACCESS_REGISTER_WRITE |
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	if (execute_abstract_command(target, command) != ERROR_OK)
		goto restore_mstatus;

	uint64_t value;
	if (register_read_direct(target, &value, GDB_REGNO_S0) != ERROR_OK)
		goto restore_mstatus;
	buf_set_u64(buffer, 0, 8 * size, value);
	log_memory_access(address, value, size, true);
	result = ERROR_OK;

restore_mstatus:
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			result = ERROR_FAIL;

	return result;
}

/**
 * Read the requested memory, silently handling memory access errors.
 */
static int read_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (riscv_xlen(target) < size * 8) {
		LOG_ERROR("XLEN (%d) is too short for %d-bit memory read.",
				riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	int result = ERROR_OK;

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			size, address);

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	select_dmi(target);

	memset(buffer, 0, count*size);

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	if (count == 1)
		return read_memory_progbuf_one(target, address, size, buffer);

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	/* s0 holds the next address to read from
	 * s1 holds the next data value read
	 * s2 is a counter in case increment is 0
	 */
	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;
	if (increment == 0 && riscv_save_register(target, GDB_REGNO_S2) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_ldr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}

	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
	if (increment == 0)
		riscv_program_addi(&program, GDB_REGNO_S2, GDB_REGNO_S2, 1);
	else
		riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, increment);

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	result = read_memory_progbuf_inner(target, address, size, count, buffer, increment);

	if (result != ERROR_OK) {
		/* The full read did not succeed, so we will try to read each word individually. */
		/* This will not be fast, but reading outside actual memory is a special case anyway. */
		/* It will make the toolchain happier, especially Eclipse Memory View as it reads ahead. */
		target_addr_t address_i = address;
		uint32_t count_i = 1;
		uint8_t *buffer_i = buffer;

		for (uint32_t i = 0; i < count; i++, address_i += increment, buffer_i += size) {
			/* TODO: This is much slower than it needs to be because we end up
			 * writing the address to read for every word we read. */
			result = read_memory_progbuf_inner(target, address_i, size, count_i, buffer_i, increment);

			/* The read of a single word failed, so we will just return 0 for that instead */
			if (result != ERROR_OK) {
				LOG_DEBUG("error reading single word of %d bytes from 0x%" TARGET_PRIxADDR,
						size, address_i);

				buf_set_u64(buffer_i, 0, 8 * size, 0);
			}
		}
		result = ERROR_OK;
	}

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	return result;
}

static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (count == 0)
		return ERROR_OK;

	if (size != 1 && size != 2 && size != 4 && size != 8 && size != 16) {
		LOG_ERROR("BUG: Unsupported size for memory read: %d", size);
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

	LOG_ERROR("Target %s: Failed to read memory (addr=0x%" PRIx64 ")", target_name(target), address);
	LOG_ERROR("  progbuf=%s, sysbus=%s, abstract=%s", progbuf_result, sysbus_result, abstract_result);
	return ret;
}

static int write_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/*1) write sbaddress: for singlewrite and autoincrement, we need to write the address once*/
	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	dmi_write(target, DM_SBADDRESS0, address);
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
		dmi_write(target, DM_SBCS, access);
		LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
		LOG_DEBUG("\r\nwrite_memory:SAB: ONE OFF: value 0x%08" PRIx64, value);
		dmi_write(target, DM_SBDATA0, value);
		return ERROR_OK;
	}

	/*B.8 Writing Memory, using autoincrement*/

	access = 0;
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
	dmi_write(target, DM_SBCS, access);

	/*2)set the value according to the size required and write*/
	for (riscv_addr_t i = 0; i < count; ++i) {
		offset = size*i;
		/* for monitoring only */
		t_addr = address + offset;
		t_buffer = buffer + offset;

		value = buf_get_u64(t_buffer, 0, 8 * size);
		LOG_DEBUG("SAB:autoincrement: expected address: 0x%08x value: 0x%08x"
				PRIx64, (uint32_t)t_addr, (uint32_t)value);
		dmi_write(target, DM_SBDATA0, value);
	}
	/*reset the autoincrement when finished (something weird is happening if this is not done at the end*/
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 0);
	dmi_write(target, DM_SBCS, access);

	return ERROR_OK;
}

static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);
	uint32_t sbcs = sb_sbaccess(size);
	sbcs = set_field(sbcs, DM_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DM_SBCS, sbcs);

	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	int result;

	sb_write_address(target, next_address, true);
	while (next_address < end_address) {
		LOG_DEBUG("transferring burst starting at address 0x%" TARGET_PRIxADDR,
				next_address);

		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				RISCV_BATCH_ALLOC_SIZE,
				info->dmi_busy_delay + info->bus_master_write_delay);
		if (!batch)
			return ERROR_FAIL;

		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			const uint8_t *p = buffer + i * size;

			if (riscv_batch_available_scans(batch) < (size + 3) / 4)
				break;

			if (size > 12)
				riscv_batch_add_dmi_write(batch, DM_SBDATA3,
						((uint32_t) p[12]) |
						(((uint32_t) p[13]) << 8) |
						(((uint32_t) p[14]) << 16) |
						(((uint32_t) p[15]) << 24), false);

			if (size > 8)
				riscv_batch_add_dmi_write(batch, DM_SBDATA2,
						((uint32_t) p[8]) |
						(((uint32_t) p[9]) << 8) |
						(((uint32_t) p[10]) << 16) |
						(((uint32_t) p[11]) << 24), false);
			if (size > 4)
				riscv_batch_add_dmi_write(batch, DM_SBDATA1,
						((uint32_t) p[4]) |
						(((uint32_t) p[5]) << 8) |
						(((uint32_t) p[6]) << 16) |
						(((uint32_t) p[7]) << 24), false);
			uint32_t value = p[0];
			if (size > 2) {
				value |= ((uint32_t) p[2]) << 16;
				value |= ((uint32_t) p[3]) << 24;
			}
			if (size > 1)
				value |= ((uint32_t) p[1]) << 8;
			riscv_batch_add_dmi_write(batch, DM_SBDATA0, value, false);

			log_memory_access(address + i * size, value, size, false);
			next_address += size;
		}

		/* Execute the batch of writes */
		result = batch_run(target, batch);
		riscv_batch_free(batch);
		if (result != ERROR_OK)
			return result;

		/* Read sbcs value.
		 * At the same time, detect if DMI busy has occurred during the batch write. */
		bool dmi_busy_encountered;
		if (dmi_op(target, &sbcs, &dmi_busy_encountered, DMI_OP_READ,
				DM_SBCS, 0, false, true) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_busy_encountered)
			LOG_DEBUG("DMI busy encountered during system bus write.");

		/* Wait until sbbusy goes low */
		time_t start = time(NULL);
		while (get_field(sbcs, DM_SBCS_SBBUSY)) {
			if (time(NULL) - start > riscv_command_timeout_sec) {
				LOG_ERROR("Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
						  "Increase the timeout with riscv set_command_timeout_sec.",
						  riscv_command_timeout_sec, sbcs);
				return ERROR_FAIL;
			}
			if (dmi_read(target, &sbcs, DM_SBCS) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. */
			LOG_DEBUG("Sbbusyerror encountered during system bus write.");
			/* Clear the sticky error flag. */
			dmi_write(target, DM_SBCS, sbcs | DM_SBCS_SBBUSYERROR);
			/* Slow down before trying again. */
			info->bus_master_write_delay += info->bus_master_write_delay / 10 + 1;
		}

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR) || dmi_busy_encountered) {
			/* Recover from the case when the write commands were issued too fast.
			 * Determine the address from which to resume writing. */
			next_address = sb_read_address(target);
			if (next_address < address) {
				/* This should never happen, probably buggy hardware. */
				LOG_DEBUG("unexpected sbaddress=0x%" TARGET_PRIxADDR
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
			LOG_DEBUG("System bus access failed with sberror=%u (sbaddress=0x%" TARGET_PRIxADDR ")",
					  sberror, sbaddress);
			if (sbaddress < address) {
				/* This should never happen, probably buggy hardware.
				 * Make a note to the user not to trust the sbaddress value. */
				LOG_DEBUG("unexpected sbaddress=0x%" TARGET_PRIxADDR
						  " - buggy sbautoincrement in hw?", next_address);
			}
			/* Clear the sticky error flag */
			dmi_write(target, DM_SBCS, DM_SBCS_SBERROR);
			/* Fail the whole operation */
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int write_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);

	if (riscv_xlen(target) < size * 8) {
		LOG_ERROR("XLEN (%d) is too short for %d-bit memory write.",
				riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	LOG_DEBUG("writing %d words of %d bytes to 0x%08lx", count, size, (long)address);

	select_dmi(target);

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	/* s0 holds the next address to write to
	 * s1 holds the next data value to write
	 */

	int result = ERROR_OK;
	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (store, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	switch (size) {
		case 1:
			riscv_program_sbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_shr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_swr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_sdr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("write_memory_progbuf(): Unsupported size: %d", size);
			result = ERROR_FAIL;
			goto error;
	}

	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);

	result = riscv_program_ebreak(&program);
	if (result != ERROR_OK)
		goto error;
	riscv_program_write(&program);

	riscv_addr_t cur_addr = address;
	riscv_addr_t distance = (riscv_addr_t)count * size;
	bool setup_needed = true;
	LOG_DEBUG("writing until final address 0x%016" PRIx64, cur_addr + distance);
	while (cur_addr - address < distance) {
		LOG_DEBUG("transferring burst starting at address 0x%016" PRIx64,
				cur_addr);

		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				RISCV_BATCH_ALLOC_SIZE,
				info->dmi_busy_delay + info->ac_busy_delay);
		if (!batch)
			goto error;

		/* To write another word, we put it in S1 and execute the program. */
		for (riscv_addr_t offset = cur_addr - address; offset < distance; offset += size) {
			const uint8_t *t_buffer = buffer + offset;

			uint64_t value = buf_get_u64(t_buffer, 0, 8 * size);

			log_memory_access(cur_addr, value, size, false);
			cur_addr += size;

			if (setup_needed) {
				result = register_write_direct(target, GDB_REGNO_S0,
						address + offset);
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Write value. */
				if (size > 4)
					dmi_write(target, DM_DATA1, value >> 32);
				dmi_write(target, DM_DATA0, value);

				/* Write and execute command that moves value into S1 and
				 * executes program buffer. */
				uint32_t command = access_register_command(target,
						GDB_REGNO_S1, riscv_xlen(target),
						AC_ACCESS_REGISTER_POSTEXEC |
						AC_ACCESS_REGISTER_TRANSFER |
						AC_ACCESS_REGISTER_WRITE);
				result = execute_abstract_command(target, command);
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Turn on autoexec */
				dmi_write(target, DM_ABSTRACTAUTO,
						1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				setup_needed = false;
			} else {
				if (size > 4)
					riscv_batch_add_dmi_write(batch, DM_DATA1, value >> 32, false);
				riscv_batch_add_dmi_write(batch, DM_DATA0, value, false);
				if (riscv_batch_full(batch))
					break;
			}
		}

		result = batch_run(target, batch);
		riscv_batch_free(batch);
		if (result != ERROR_OK)
			goto error;

		/* Note that if the scan resulted in a Busy DMI response, it
		 * is this read to abstractcs that will cause the dmi_busy_delay
		 * to be incremented if necessary. */

		uint32_t abstractcs;
		bool dmi_busy_encountered;
		result = dmi_op(target, &abstractcs, &dmi_busy_encountered,
				DMI_OP_READ, DM_ABSTRACTCS, 0, false, true);
		if (result != ERROR_OK)
			goto error;
		while (get_field(abstractcs, DM_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);
		if (info->cmderr == CMDERR_NONE && !dmi_busy_encountered) {
			LOG_DEBUG("successful (partial?) memory write");
		} else if (info->cmderr == CMDERR_BUSY || dmi_busy_encountered) {
			if (info->cmderr == CMDERR_BUSY)
				LOG_DEBUG("Memory write resulted in abstract command busy response.");
			else if (dmi_busy_encountered)
				LOG_DEBUG("Memory write resulted in DMI busy response.");
			riscv013_clear_abstract_error(target);
			increase_ac_busy_delay(target);

			dmi_write(target, DM_ABSTRACTAUTO, 0);
			result = register_read_direct(target, &cur_addr, GDB_REGNO_S0);
			if (result != ERROR_OK)
				goto error;
			setup_needed = true;
		} else {
			LOG_ERROR("error when writing memory, abstractcs=0x%08lx", (long)abstractcs);
			riscv013_clear_abstract_error(target);
			result = ERROR_FAIL;
			goto error;
		}
	}

error:
	dmi_write(target, DM_ABSTRACTAUTO, 0);

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
		LOG_ERROR("BUG: Unsupported size for memory write: %d", size);
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

	LOG_ERROR("Target %s: Failed to write memory (addr=0x%" PRIx64 ")", target_name(target), address);
	LOG_ERROR("  progbuf=%s, sysbus=%s, abstract=%s", progbuf_result, sysbus_result, abstract_result);
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
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int rid)
{
	LOG_DEBUG("[%s] reading register %s", target_name(target),
			gdb_regno_name(rid));

	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;

	int result = ERROR_OK;
	if (rid == GDB_REGNO_PC) {
		/* TODO: move this into riscv.c. */
		result = register_read_direct(target, value, GDB_REGNO_DPC);
		LOG_TARGET_DEBUG(target, "read PC from DPC: 0x%" PRIx64, *value);
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		/* TODO: move this into riscv.c. */
		result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
		*value = set_field(0, VIRT_PRIV_V, get_field(dcsr, CSR_DCSR_V));
		*value = set_field(*value, VIRT_PRIV_PRV, get_field(dcsr, CSR_DCSR_PRV));
	} else {
		result = register_read_direct(target, value, rid);
		if (result != ERROR_OK)
			*value = -1;
	}

	return result;
}

static int riscv013_set_register(struct target *target, int rid, uint64_t value)
{
	if (dm013_select_target(target) != ERROR_OK)
		return ERROR_FAIL;
	LOG_TARGET_DEBUG(target, "writing 0x%" PRIx64 " to register %s",
			value, gdb_regno_name(rid));

	if (rid <= GDB_REGNO_XPR31) {
		return register_write_direct(target, rid, value);
	} else if (rid == GDB_REGNO_PC) {
		LOG_TARGET_DEBUG(target, "writing PC to DPC: 0x%" PRIx64, value);
		register_write_direct(target, GDB_REGNO_DPC, value);
		uint64_t actual_value;
		register_read_direct(target, &actual_value, GDB_REGNO_DPC);
		LOG_TARGET_DEBUG(target, "  actual DPC written: 0x%016" PRIx64, actual_value);
		if (value != actual_value) {
			LOG_ERROR("Written PC (0x%" PRIx64 ") does not match read back "
					"value (0x%" PRIx64 ")", value, actual_value);
			return ERROR_FAIL;
		}
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
		dcsr = set_field(dcsr, CSR_DCSR_PRV, get_field(value, VIRT_PRIV_PRV));
		dcsr = set_field(dcsr, CSR_DCSR_V, get_field(value, VIRT_PRIV_V));
		return register_write_direct(target, GDB_REGNO_DCSR, dcsr);
	} else {
		return register_write_direct(target, rid, value);
	}

	return ERROR_OK;
}

static int dm013_select_hart(struct target *target, int hart_index)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (hart_index == dm->current_hartid)
		return ERROR_OK;

	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE;
	dmcontrol = set_dmcontrol_hartsel(dmcontrol, hart_index);
	if (dmi_write(target, DM_DMCONTROL, dmcontrol) != ERROR_OK) {
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
		dm013_select_target(target);
		return ERROR_OK;
	}

	assert(dm->hart_count);
	unsigned hawindow_count = (dm->hart_count + 31) / 32;
	uint32_t hawindow[hawindow_count];

	memset(hawindow, 0, sizeof(uint32_t) * hawindow_count);

	target_list_t *entry;
	unsigned total_selected = 0;
	unsigned int selected_index = 0;
	list_for_each_entry(entry, &dm->target_list, list) {
		struct target *t = entry->target;
		riscv_info_t *info = riscv_info(t);
		riscv013_info_t *info_013 = get_info(t);
		unsigned index = info_013->index;
		LOG_DEBUG("index=%d, coreid=%d, prepped=%d", index, t->coreid, info->prepped);
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
		return ERROR_FAIL;
	} else if (total_selected == 1) {
		/* Don't use hasel if we only need to talk to one hart. */
		return dm013_select_hart(target, selected_index);
	}

	if (dm013_select_hart(target, HART_INDEX_MULTIPLE) != ERROR_OK)
		return ERROR_FAIL;

	for (unsigned i = 0; i < hawindow_count; i++) {
		if (dmi_write(target, DM_HAWINDOWSEL, i) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_write(target, DM_HAWINDOW, hawindow[i]) != ERROR_OK)
			return ERROR_FAIL;
	}

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

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_HALTREQ;
	dmcontrol = set_dmcontrol_hartsel(dmcontrol, dm->current_hartid);
	dmi_write(target, DM_DMCONTROL, dmcontrol);
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
		if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
			return ERROR_FAIL;

		LOG_TARGET_ERROR(target, "Unable to halt. dmcontrol=0x%08x, dmstatus=0x%08x",
				  dmcontrol, dmstatus);
		return ERROR_FAIL;
	}

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HALTREQ, 0);
	dmi_write(target, DM_DMCONTROL, dmcontrol);

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
				riscv013_info_t *info = get_info(t);
				dmcontrol = set_dmcontrol_hartsel(dmcontrol, info->index);
				if (dmi_write(target, DM_DMCONTROL, dmcontrol) != ERROR_OK)
					return ERROR_FAIL;
				dm->current_hartid = info->index;
				if (dmi_read(target, &t_dmstatus, DM_DMSTATUS) != ERROR_OK)
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
	return riscv013_on_step_or_resume(target, false);
}

static int riscv013_on_step(struct target *target)
{
	return riscv013_on_step_or_resume(target, true);
}

static int riscv013_on_halt(struct target *target)
{
	return ERROR_OK;
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
		LOG_DEBUG("{%d} halted because of trigger", target->coreid);
		return RISCV_HALT_TRIGGER;
	case CSR_DCSR_CAUSE_STEP:
		return RISCV_HALT_SINGLESTEP;
	case CSR_DCSR_CAUSE_HALTREQ:
	case CSR_DCSR_CAUSE_RESETHALTREQ:
		return RISCV_HALT_INTERRUPT;
	case CSR_DCSR_CAUSE_GROUP:
		return RISCV_HALT_GROUP;
	}

	LOG_ERROR("Unknown DCSR cause field: 0x%" PRIx64, get_field(dcsr, CSR_DCSR_CAUSE));
	LOG_ERROR("  dcsr=0x%" PRIx32, (uint32_t) dcsr);
	return RISCV_HALT_UNKNOWN;
}

int riscv013_write_debug_buffer(struct target *target, unsigned index, riscv_insn_t data)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (dm->progbuf_cache[index] != data) {
		if (dmi_write(target, DM_PROGBUF0 + index, data) != ERROR_OK)
			return ERROR_FAIL;
		dm->progbuf_cache[index] = data;
	} else {
		LOG_DEBUG("cache hit for 0x%" PRIx32 " @%d", data, index);
	}
	return ERROR_OK;
}

riscv_insn_t riscv013_read_debug_buffer(struct target *target, unsigned index)
{
	uint32_t value;
	dmi_read(target, &value, DM_PROGBUF0 + index);
	return value;
}

int riscv013_invalidate_cached_debug_buffer(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	LOG_TARGET_DEBUG(target, "Invalidating progbuf cache");
	for (unsigned int i = 0; i < 15; i++)
		dm->progbuf_cache[i] = 0;
	return ERROR_OK;
}

int riscv013_execute_debug_buffer(struct target *target)
{
	uint32_t run_program = 0;
	run_program = set_field(run_program, AC_ACCESS_REGISTER_AARSIZE, 2);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_POSTEXEC, 1);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_TRANSFER, 0);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_REGNO, 0x1000);

	return execute_abstract_command(target, run_program);
}

void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_WRITE);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, d);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_READ);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_nop_u64(struct target *target, char *buf)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_NOP);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, 0);
}

int riscv013_dmi_write_u64_bits(struct target *target)
{
	RISCV013_INFO(info);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

static int maybe_execute_fence_i(struct target *target)
{
	if (has_sufficient_progbuf(target, 3))
		return execute_fence(target);
	return ERROR_OK;
}

/* Helper Functions. */
static int riscv013_on_step_or_resume(struct target *target, bool step)
{
	if (maybe_execute_fence_i(target) != ERROR_OK)
		return ERROR_FAIL;

	/* We want to twiddle some bits in the debug CSR so debugging works. */
	riscv_reg_t dcsr;
	int result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return result;
	dcsr = set_field(dcsr, CSR_DCSR_STEP, step);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, riscv_ebreakm);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, riscv_ebreaks);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, riscv_ebreaku);
	if (riscv_set_register(target, GDB_REGNO_DCSR, dcsr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_flush_registers(target) != ERROR_OK)
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

	if (riscv_flush_registers(target) != ERROR_OK)
		return ERROR_FAIL;

	dm013_info_t *dm = get_dm(target);
	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_RESUMEREQ;
	dmcontrol = set_dmcontrol_hartsel(dmcontrol, dm->current_hartid);
	dmi_write(target, DM_DMCONTROL, dmcontrol);

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

		dmi_write(target, DM_DMCONTROL, dmcontrol);
		return ERROR_OK;
	}

	dmi_write(target, DM_DMCONTROL, dmcontrol);

	LOG_TARGET_ERROR(target, "unable to resume");
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return ERROR_FAIL;
	LOG_TARGET_ERROR(target, "  dmstatus=0x%08x", dmstatus);

	if (step) {
		LOG_ERROR("  was stepping, halting");
		riscv_halt(target);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

void riscv013_clear_abstract_error(struct target *target)
{
	/* Wait for busy to go away. */
	time_t start = time(NULL);
	uint32_t abstractcs;
	dmi_read(target, &abstractcs, DM_ABSTRACTCS);
	while (get_field(abstractcs, DM_ABSTRACTCS_BUSY)) {
		dmi_read(target, &abstractcs, DM_ABSTRACTCS);

		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("abstractcs.busy is not going low after %d seconds "
					"(abstractcs=0x%x). The target is either really slow or "
					"broken. You could increase the timeout with riscv "
					"set_command_timeout_sec.",
					riscv_command_timeout_sec, abstractcs);
			break;
		}
	}
	/* Clear the error status. */
	dmi_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
}

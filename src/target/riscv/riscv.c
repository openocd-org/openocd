/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/time_support.h>
#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include <target/smp.h>
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/base64.h"
#include "helper/time_support.h"
#include "riscv.h"
#include "gdb_regs.h"
#include "rtos/rtos.h"
#include "debug_defines.h"
#include <helper/bits.h>

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

/*** JTAG registers. ***/

#define DTMCONTROL					0x10
#define DTMCONTROL_VERSION			(0xf)

#define DBUS						0x11

uint8_t ir_dtmcontrol[4] = {DTMCONTROL};
struct scan_field select_dtmcontrol = {
	.in_value = NULL,
	.out_value = ir_dtmcontrol
};
uint8_t ir_dbus[4] = {DBUS};
struct scan_field select_dbus = {
	.in_value = NULL,
	.out_value = ir_dbus
};
uint8_t ir_idcode[4] = {0x1};
struct scan_field select_idcode = {
	.in_value = NULL,
	.out_value = ir_idcode
};

bscan_tunnel_type_t bscan_tunnel_type;
int bscan_tunnel_ir_width; /* if zero, then tunneling is not present/active */
static int bscan_tunnel_ir_id; /* IR ID of the JTAG TAP to access the tunnel. Valid when not 0 */

static const uint8_t bscan_zero[4] = {0};
static const uint8_t bscan_one[4] = {1};

uint8_t ir_user4[4];
struct scan_field select_user4 = {
	.in_value = NULL,
	.out_value = ir_user4
};


uint8_t bscan_tunneled_ir_width[4] = {5};  /* overridden by assignment in riscv_init_target */
struct scan_field _bscan_tunnel_data_register_select_dmi[] = {
		{
			.num_bits = 3,
			.out_value = bscan_zero,
			.in_value = NULL,
		},
		{
			.num_bits = 5, /* initialized in riscv_init_target to ir width of DM */
			.out_value = ir_dbus,
			.in_value = NULL,
		},
		{
			.num_bits = 7,
			.out_value = bscan_tunneled_ir_width,
			.in_value = NULL,
		},
		{
			.num_bits = 1,
			.out_value = bscan_zero,
			.in_value = NULL,
		}
};

struct scan_field _bscan_tunnel_nested_tap_select_dmi[] = {
		{
			.num_bits = 1,
			.out_value = bscan_zero,
			.in_value = NULL,
		},
		{
			.num_bits = 7,
			.out_value = bscan_tunneled_ir_width,
			.in_value = NULL,
		},
		{
			.num_bits = 0, /* initialized in riscv_init_target to ir width of DM */
			.out_value = ir_dbus,
			.in_value = NULL,
		},
		{
			.num_bits = 3,
			.out_value = bscan_zero,
			.in_value = NULL,
		}
};
struct scan_field *bscan_tunnel_nested_tap_select_dmi = _bscan_tunnel_nested_tap_select_dmi;
uint32_t bscan_tunnel_nested_tap_select_dmi_num_fields = ARRAY_SIZE(_bscan_tunnel_nested_tap_select_dmi);

struct scan_field *bscan_tunnel_data_register_select_dmi = _bscan_tunnel_data_register_select_dmi;
uint32_t bscan_tunnel_data_register_select_dmi_num_fields = ARRAY_SIZE(_bscan_tunnel_data_register_select_dmi);

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

bool riscv_enable_virt2phys = true;
bool riscv_ebreakm = true;
bool riscv_ebreaks = true;
bool riscv_ebreaku = true;

bool riscv_enable_virtual;

static enum {
	RO_NORMAL,
	RO_REVERSED
} resume_order;

const virt2phys_info_t sv32 = {
	.name = "Sv32",
	.va_bits = 32,
	.level = 2,
	.pte_shift = 2,
	.vpn_shift = {12, 22},
	.vpn_mask = {0x3ff, 0x3ff},
	.pte_ppn_shift = {10, 20},
	.pte_ppn_mask = {0x3ff, 0xfff},
	.pa_ppn_shift = {12, 22},
	.pa_ppn_mask = {0x3ff, 0xfff},
};

const virt2phys_info_t sv39 = {
	.name = "Sv39",
	.va_bits = 39,
	.level = 3,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30},
	.vpn_mask = {0x1ff, 0x1ff, 0x1ff},
	.pte_ppn_shift = {10, 19, 28},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x3ffffff},
	.pa_ppn_shift = {12, 21, 30},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x3ffffff},
};

const virt2phys_info_t sv48 = {
	.name = "Sv48",
	.va_bits = 48,
	.level = 4,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30, 39},
	.vpn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff},
	.pte_ppn_shift = {10, 19, 28, 37},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ffff},
	.pa_ppn_shift = {12, 21, 30, 39},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ffff},
};

void riscv_sample_buf_maybe_add_timestamp(struct target *target, bool before)
{
	RISCV_INFO(r);
	uint32_t now = timeval_ms() & 0xffffffff;
	if (r->sample_buf.used + 5 < r->sample_buf.size) {
		if (before)
			r->sample_buf.buf[r->sample_buf.used++] = RISCV_SAMPLE_BUF_TIMESTAMP_BEFORE;
		else
			r->sample_buf.buf[r->sample_buf.used++] = RISCV_SAMPLE_BUF_TIMESTAMP_AFTER;
		r->sample_buf.buf[r->sample_buf.used++] = now & 0xff;
		r->sample_buf.buf[r->sample_buf.used++] = (now >> 8) & 0xff;
		r->sample_buf.buf[r->sample_buf.used++] = (now >> 16) & 0xff;
		r->sample_buf.buf[r->sample_buf.used++] = (now >> 24) & 0xff;
	}
}

static int riscv_resume_go_all_harts(struct target *target);

void select_dmi_via_bscan(struct target *target)
{
	jtag_add_ir_scan(target->tap, &select_user4, TAP_IDLE);
	if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER)
		jtag_add_dr_scan(target->tap, bscan_tunnel_data_register_select_dmi_num_fields,
										bscan_tunnel_data_register_select_dmi, TAP_IDLE);
	else /* BSCAN_TUNNEL_NESTED_TAP */
		jtag_add_dr_scan(target->tap, bscan_tunnel_nested_tap_select_dmi_num_fields,
										bscan_tunnel_nested_tap_select_dmi, TAP_IDLE);
}

uint32_t dtmcontrol_scan_via_bscan(struct target *target, uint32_t out)
{
	/* On BSCAN TAP: Select IR=USER4, issue tunneled IR scan via BSCAN TAP's DR */
	uint8_t tunneled_ir_width[4] = {bscan_tunnel_ir_width};
	uint8_t tunneled_dr_width[4] = {32};
	uint8_t out_value[5] = {0};
	uint8_t in_value[5] = {0};

	buf_set_u32(out_value, 0, 32, out);
	struct scan_field tunneled_ir[4] = {};
	struct scan_field tunneled_dr[4] = {};

	if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER) {
		tunneled_ir[0].num_bits = 3;
		tunneled_ir[0].out_value = bscan_zero;
		tunneled_ir[0].in_value = NULL;
		tunneled_ir[1].num_bits = bscan_tunnel_ir_width;
		tunneled_ir[1].out_value = ir_dtmcontrol;
		tunneled_ir[1].in_value = NULL;
		tunneled_ir[2].num_bits = 7;
		tunneled_ir[2].out_value = tunneled_ir_width;
		tunneled_ir[2].in_value = NULL;
		tunneled_ir[3].num_bits = 1;
		tunneled_ir[3].out_value = bscan_zero;
		tunneled_ir[3].in_value = NULL;

		tunneled_dr[0].num_bits = 3;
		tunneled_dr[0].out_value = bscan_zero;
		tunneled_dr[0].in_value = NULL;
		tunneled_dr[1].num_bits = 32 + 1;
		tunneled_dr[1].out_value = out_value;
		tunneled_dr[1].in_value = in_value;
		tunneled_dr[2].num_bits = 7;
		tunneled_dr[2].out_value = tunneled_dr_width;
		tunneled_dr[2].in_value = NULL;
		tunneled_dr[3].num_bits = 1;
		tunneled_dr[3].out_value = bscan_one;
		tunneled_dr[3].in_value = NULL;
	} else {
		/* BSCAN_TUNNEL_NESTED_TAP */
		tunneled_ir[3].num_bits = 3;
		tunneled_ir[3].out_value = bscan_zero;
		tunneled_ir[3].in_value = NULL;
		tunneled_ir[2].num_bits = bscan_tunnel_ir_width;
		tunneled_ir[2].out_value = ir_dtmcontrol;
		tunneled_ir[1].in_value = NULL;
		tunneled_ir[1].num_bits = 7;
		tunneled_ir[1].out_value = tunneled_ir_width;
		tunneled_ir[2].in_value = NULL;
		tunneled_ir[0].num_bits = 1;
		tunneled_ir[0].out_value = bscan_zero;
		tunneled_ir[0].in_value = NULL;

		tunneled_dr[3].num_bits = 3;
		tunneled_dr[3].out_value = bscan_zero;
		tunneled_dr[3].in_value = NULL;
		tunneled_dr[2].num_bits = 32 + 1;
		tunneled_dr[2].out_value = out_value;
		tunneled_dr[2].in_value = in_value;
		tunneled_dr[1].num_bits = 7;
		tunneled_dr[1].out_value = tunneled_dr_width;
		tunneled_dr[1].in_value = NULL;
		tunneled_dr[0].num_bits = 1;
		tunneled_dr[0].out_value = bscan_one;
		tunneled_dr[0].in_value = NULL;
	}
	jtag_add_ir_scan(target->tap, &select_user4, TAP_IDLE);
	jtag_add_dr_scan(target->tap, ARRAY_SIZE(tunneled_ir), tunneled_ir, TAP_IDLE);
	jtag_add_dr_scan(target->tap, ARRAY_SIZE(tunneled_dr), tunneled_dr, TAP_IDLE);
	select_dmi_via_bscan(target);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	/* Note the starting offset is bit 1, not bit 0.  In BSCAN tunnel, there is a one-bit TCK skew between
	   output and input */
	uint32_t in = buf_get_u32(in_value, 1, 32);
	LOG_DEBUG("DTMCS: 0x%x -> 0x%x", out, in);

	return in;
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
			LOG_ERROR("[%s] Unsupported DTM version: %d",
					target_name(target), info->dtm_version);
			return NULL;
	}
}

static int riscv_create_target(struct target *target, Jim_Interp *interp)
{
	LOG_DEBUG("riscv_create_target()");
	target->arch_info = calloc(1, sizeof(riscv_info_t));
	if (!target->arch_info) {
		LOG_ERROR("Failed to allocate RISC-V target structure.");
		return ERROR_FAIL;
	}
	riscv_info_init(target, target->arch_info);
	return ERROR_OK;
}

static int riscv_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("riscv_init_target()");
	RISCV_INFO(info);
	info->cmd_ctx = cmd_ctx;

	select_dtmcontrol.num_bits = target->tap->ir_length;
	select_dbus.num_bits = target->tap->ir_length;
	select_idcode.num_bits = target->tap->ir_length;

	if (bscan_tunnel_ir_width != 0) {
		uint32_t ir_user4_raw = bscan_tunnel_ir_id;
		/* Provide a default value which target some Xilinx FPGA USER4 IR */
		if (ir_user4_raw == 0) {
			assert(target->tap->ir_length >= 6);
			ir_user4_raw = 0x23 << (target->tap->ir_length - 6);
		}
		ir_user4[0] = (uint8_t)ir_user4_raw;
		ir_user4[1] = (uint8_t)(ir_user4_raw >>= 8);
		ir_user4[2] = (uint8_t)(ir_user4_raw >>= 8);
		ir_user4[3] = (uint8_t)(ir_user4_raw >>= 8);
		select_user4.num_bits = target->tap->ir_length;
		bscan_tunneled_ir_width[0] = bscan_tunnel_ir_width;
		if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER)
			bscan_tunnel_data_register_select_dmi[1].num_bits = bscan_tunnel_ir_width;
		else /* BSCAN_TUNNEL_NESTED_TAP */
			bscan_tunnel_nested_tap_select_dmi[2].num_bits = bscan_tunnel_ir_width;
	}

	riscv_semihosting_init(target);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static void riscv_free_registers(struct target *target)
{
	/* Free the shared structure use for most registers. */
	if (target->reg_cache) {
		if (target->reg_cache->reg_list) {
			free(target->reg_cache->reg_list[0].arch_info);
			/* Free the ones we allocated separately. */
			for (unsigned i = GDB_REGNO_COUNT; i < target->reg_cache->num_regs; i++)
				free(target->reg_cache->reg_list[i].arch_info);
			for (unsigned int i = 0; i < target->reg_cache->num_regs; i++)
				free(target->reg_cache->reg_list[i].value);
			free(target->reg_cache->reg_list);
		}
		free(target->reg_cache);
	}
}

static void riscv_deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");

	riscv_info_t *info = target->arch_info;
	struct target_type *tt = get_target_type(target);

	if (riscv_flush_registers(target) != ERROR_OK)
		LOG_ERROR("[%s] Failed to flush registers. Ignoring this error.", target_name(target));

	if (tt && info->version_specific)
		tt->deinit_target(target);

	riscv_free_registers(target);

	range_list_t *entry, *tmp;
	list_for_each_entry_safe(entry, tmp, &info->expose_csr, list) {
		free(entry->name);
		free(entry);
	}

	list_for_each_entry_safe(entry, tmp, &info->expose_custom, list) {
		free(entry->name);
		free(entry);
	}

	free(info->reg_names);
	free(target->arch_info);

	target->arch_info = NULL;
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

static bool can_use_napot_match(struct trigger *trigger, riscv_reg_t *tdata2)
{
	riscv_reg_t addr = trigger->address;
	riscv_reg_t size = trigger->length;
	bool sizePowerOf2 = (size & (size - 1)) == 0;
	bool addrAligned = (addr & (size - 1)) == 0;
	if (size > 1 && sizePowerOf2 && addrAligned) {
		if (tdata2)
			*tdata2 = addr | ((size - 1) >> 1);
		return true;
	}
	return false;
}

static int find_trigger(struct target *target, int type, bool chained, unsigned int *idx)
{
	RISCV_INFO(r);

	unsigned int num_found = 0;
	unsigned int num_required = chained ? 2 : 1;

	for (unsigned i = 0; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == -1) {
			if (r->trigger_tinfo[i] & (1 << type)) {
				num_found++;
				bool done = (num_required == num_found);
				if (done) {
					/* Found num_required consecutive free triggers - success, done. */
					if (idx)
						*idx = i - (num_required - 1);
					return ERROR_OK;
				}
				/* Found a trigger but need more consecutive ones */
				continue;
			}
		}
		/* Trigger already occupied or incompatible type.
		 * Reset the counter of found consecutive triggers */
		num_found = 0;
	}

	return ERROR_FAIL;
}

static int find_first_trigger_by_id(struct target *target, int unique_id)
{
	RISCV_INFO(r);

	for (unsigned i = 0; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == unique_id)
			return i;
	}
	return -1;
}

static int set_trigger(struct target *target, unsigned int idx, riscv_reg_t tdata1, riscv_reg_t tdata2,
	riscv_reg_t tdata1_ignore_mask)
{
	riscv_reg_t tdata1_rb, tdata2_rb;
	if (riscv_set_register(target, GDB_REGNO_TSELECT, idx) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_set_register(target, GDB_REGNO_TDATA1, tdata1) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_get_register(target, &tdata1_rb, GDB_REGNO_TDATA1) != ERROR_OK)
		return ERROR_FAIL;
	if ((tdata1 & ~tdata1_ignore_mask) != (tdata1_rb & ~tdata1_ignore_mask)) {
		LOG_TARGET_DEBUG(target,
			"Trigger %u doesn't support what we need; After writing 0x%"
			PRIx64 " to tdata1 it contains 0x%" PRIx64
			"; tdata1_ignore_mask=0x%" PRIx64,
			idx, tdata1, tdata1_rb, tdata1_ignore_mask);
		riscv_set_register(target, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	if (riscv_set_register(target, GDB_REGNO_TDATA2, tdata2) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_get_register(target, &tdata2_rb, GDB_REGNO_TDATA2) != ERROR_OK)
		return ERROR_FAIL;
	if (tdata2 != tdata2_rb) {
		LOG_TARGET_DEBUG(target,
			"Trigger %u doesn't support what we need; wrote 0x%"
			PRIx64 " to tdata2 but read back 0x%" PRIx64,
			idx, tdata2, tdata2_rb);
		riscv_set_register(target, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return ERROR_OK;
}

static int maybe_add_trigger_t1(struct target *target, struct trigger *trigger)
{
	int ret;
	riscv_reg_t tdata1, tdata2;

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

	unsigned int idx;
	ret = find_trigger(target, CSR_TDATA1_TYPE_LEGACY, false, &idx);
	if (ret != ERROR_OK)
		return ret;

	if (riscv_get_register(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
		return ERROR_FAIL;
	if (tdata1 & (bpcontrol_r | bpcontrol_w | bpcontrol_x)) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	tdata1 = 0;
	tdata1 = set_field(tdata1, bpcontrol_r, trigger->read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = set_field(tdata1, bpcontrol_u, !!(r->misa & BIT('U' - 'A')));
	tdata1 = set_field(tdata1, bpcontrol_s, !!(r->misa & BIT('S' - 'A')));
	tdata1 = set_field(tdata1, bpcontrol_h, !!(r->misa & BIT('H' - 'A')));
	tdata1 = set_field(tdata1, bpcontrol_m, 1);
	tdata1 = set_field(tdata1, bpcontrol_bpaction, 0); /* cause bp exception */
	tdata1 = set_field(tdata1, bpcontrol_bpmatch, 0); /* exact match */
	tdata2 = trigger->address;
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK)
		return ret;
	r->trigger_unique_id[idx] = trigger->unique_id;
	return ERROR_OK;
}

static int maybe_add_trigger_t2(struct target *target, struct trigger *trigger)
{
	int ret;
	riscv_reg_t tdata1, tdata2;

	RISCV_INFO(r);

	tdata1 = 0;
	tdata1 = set_field(tdata1, CSR_MCONTROL_TYPE(riscv_xlen(target)), 2);
	tdata1 = set_field(tdata1, CSR_MCONTROL_DMODE(riscv_xlen(target)), 1);
	tdata1 = set_field(tdata1, CSR_MCONTROL_ACTION, CSR_MCONTROL_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, CSR_MCONTROL_M, 1);
	tdata1 = set_field(tdata1, CSR_MCONTROL_S, !!(r->misa & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, CSR_MCONTROL_U, !!(r->misa & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, CSR_MCONTROL_EXECUTE, trigger->execute);
	tdata1 = set_field(tdata1, CSR_MCONTROL_LOAD, trigger->read);
	tdata1 = set_field(tdata1, CSR_MCONTROL_STORE, trigger->write);
	tdata1 = set_field(tdata1, CSR_MCONTROL_SIZELO, CSR_MCONTROL_SIZELO_ANY & 3);
	tdata1 = set_field(tdata1, CSR_MCONTROL_SIZEHI, (CSR_MCONTROL_SIZELO_ANY >> 2) & 3);

	if (trigger->execute || trigger->length == 1)
		goto MATCH_EQUAL;
	if (!can_use_napot_match(trigger, &tdata2))
		goto MATCH_GE_LT;

	unsigned int idx;
	ret = find_trigger(target, CSR_TDATA1_TYPE_MCONTROL, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	tdata1 = set_field(tdata1, CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_NAPOT);
	tdata1 = set_field(tdata1, CSR_MCONTROL_CHAIN, CSR_MCONTROL_CHAIN_DISABLED);
	ret = set_trigger(target, idx, tdata1, tdata2, CSR_MCONTROL_MASKMAX(riscv_xlen(target)));
	if (ret != ERROR_OK) {
		if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			goto MATCH_GE_LT; /* Fallback to chained MATCH using GT and LE */
		return ret;
	}
	r->trigger_unique_id[idx] = trigger->unique_id;
	return ERROR_OK;

MATCH_GE_LT:
	ret = find_trigger(target, CSR_TDATA1_TYPE_MCONTROL, true, &idx);
	if (ret != ERROR_OK)
		return ret;
	tdata1 = set_field(tdata1, CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_GE);
	tdata1 = set_field(tdata1, CSR_MCONTROL_CHAIN, CSR_MCONTROL_CHAIN_ENABLED);
	tdata2 = trigger->address;
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK) {
		if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			goto MATCH_EQUAL; /* Fallback to EQUAL MATCH */
		return ret;
	}
	tdata1 = set_field(tdata1, CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_LT);
	tdata1 = set_field(tdata1, CSR_MCONTROL_CHAIN, CSR_MCONTROL_CHAIN_DISABLED);
	tdata2 = trigger->address + trigger->length;
	ret = set_trigger(target, idx + 1, tdata1, tdata2, 0);
	if (ret != ERROR_OK) {
		/* Undo the setting of the previous trigger*/
		set_trigger(target, idx, 0, 0, CSR_MCONTROL_MASKMAX(riscv_xlen(target)));
		if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			goto MATCH_EQUAL; /* Fallback to EQUAL MATCH */
		return ret;
	}
	r->trigger_unique_id[idx] = trigger->unique_id;
	r->trigger_unique_id[idx + 1] = trigger->unique_id;
	return ERROR_OK;

MATCH_EQUAL:
	ret = find_trigger(target, CSR_TDATA1_TYPE_MCONTROL, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	tdata1 = set_field(tdata1, CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_EQUAL);
	if (trigger->length == 1) {
		tdata1 = set_field(tdata1, CSR_MCONTROL_SIZELO, CSR_MCONTROL_SIZELO_8BIT & 3);
		tdata1 = set_field(tdata1, CSR_MCONTROL_SIZEHI, (CSR_MCONTROL_SIZELO_8BIT >> 2) & 3);
	}
	tdata1 = set_field(tdata1, CSR_MCONTROL_CHAIN, CSR_MCONTROL_CHAIN_DISABLED);
	tdata2 = trigger->address;
	ret = set_trigger(target, idx, tdata1, tdata2, CSR_MCONTROL_MASKMAX(riscv_xlen(target)));
	if (ret != ERROR_OK)
		return ret;
	r->trigger_unique_id[idx] = trigger->unique_id;
	return ERROR_OK;
}

static int maybe_add_trigger_t4(struct target *target, bool vs, bool vu,
				bool nmi, bool m, bool s, bool u, riscv_reg_t interrupts,
				int unique_id)
{
	int ret;
	riscv_reg_t tdata1, tdata2;

	RISCV_INFO(r);

	tdata1 = 0;
	tdata1 = set_field(tdata1, CSR_ITRIGGER_TYPE(riscv_xlen(target)), CSR_TDATA1_TYPE_ITRIGGER);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_DMODE(riscv_xlen(target)), 1);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_ACTION, CSR_ITRIGGER_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_VS, vs);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_VU, vu);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_NMI, nmi);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_M, m);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_S, s);
	tdata1 = set_field(tdata1, CSR_ITRIGGER_U, u);

	tdata2 = interrupts;

	unsigned int idx;
	ret = find_trigger(target, CSR_TDATA1_TYPE_ITRIGGER, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK)
		return ret;
	r->trigger_unique_id[idx] = unique_id;
	return ERROR_OK;
}

static int maybe_add_trigger_t5(struct target *target, bool vs, bool vu,
				bool m, bool s, bool u, riscv_reg_t exception_codes,
				int unique_id)
{
	int ret;
	riscv_reg_t tdata1, tdata2;

	RISCV_INFO(r);

	tdata1 = 0;
	tdata1 = set_field(tdata1, CSR_ETRIGGER_TYPE(riscv_xlen(target)), CSR_TDATA1_TYPE_ETRIGGER);
	tdata1 = set_field(tdata1, CSR_ETRIGGER_DMODE(riscv_xlen(target)), 1);
	tdata1 = set_field(tdata1, CSR_ETRIGGER_ACTION, CSR_ETRIGGER_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, CSR_ETRIGGER_VS, vs);
	tdata1 = set_field(tdata1, CSR_ETRIGGER_VU, vu);
	tdata1 = set_field(tdata1, CSR_ETRIGGER_M, m);
	tdata1 = set_field(tdata1, CSR_ETRIGGER_S, s);
	tdata1 = set_field(tdata1, CSR_ETRIGGER_U, u);

	tdata2 = exception_codes;

	unsigned int idx;
	ret = find_trigger(target, CSR_TDATA1_TYPE_ETRIGGER, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK)
		return ret;
	r->trigger_unique_id[idx] = unique_id;
	return ERROR_OK;
}

static int maybe_add_trigger_t6(struct target *target, struct trigger *trigger)
{
	int ret;
	riscv_reg_t tdata1, tdata2;

	RISCV_INFO(r);

	tdata1 = 0;
	tdata1 = set_field(tdata1, CSR_MCONTROL6_TYPE(riscv_xlen(target)), 6);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_DMODE(riscv_xlen(target)), 1);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_ACTION, CSR_MCONTROL6_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_M, 1);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_S, !!(r->misa & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, CSR_MCONTROL6_U, !!(r->misa & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, CSR_MCONTROL6_EXECUTE, trigger->execute);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_LOAD, trigger->read);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_STORE, trigger->write);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_SIZE, CSR_MCONTROL6_SIZE_ANY);

	if (trigger->execute || trigger->length == 1)
		goto MATCH_EQUAL;
	if (!can_use_napot_match(trigger, &tdata2))
		goto MATCH_GE_LT;

	unsigned int idx;
	ret = find_trigger(target, CSR_TDATA1_TYPE_MCONTROL6, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	tdata1 = set_field(tdata1, CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_NAPOT);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_CHAIN, CSR_MCONTROL6_CHAIN_DISABLED);
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK) {
		if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			goto MATCH_GE_LT; /* Fallback to chained MATCH using GT and LE */
		return ret;
	}
	r->trigger_unique_id[idx] = trigger->unique_id;
	return ERROR_OK;

MATCH_GE_LT:
	ret = find_trigger(target, CSR_TDATA1_TYPE_MCONTROL6, true, &idx);
	if (ret != ERROR_OK)
		return ret;
	tdata1 = set_field(tdata1, CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_GE);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_CHAIN, CSR_MCONTROL6_CHAIN_ENABLED);
	tdata2 = trigger->address;
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK) {
		if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			goto MATCH_EQUAL; /* Fallback to EQUAL MATCH */
		return ret;
	}
	tdata1 = set_field(tdata1, CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_LT);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_CHAIN, CSR_MCONTROL6_CHAIN_DISABLED);
	tdata2 = trigger->address + trigger->length;
	ret = set_trigger(target, idx + 1, tdata1, tdata2, 0);
	if (ret != ERROR_OK) {
		set_trigger(target, idx, 0, 0, 0); /* Undo the setting of the previous trigger*/
		if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			goto MATCH_EQUAL; /* Fallback to EQUAL MATCH */
		return ret;
	}
	r->trigger_unique_id[idx] = trigger->unique_id;
	r->trigger_unique_id[idx + 1] = trigger->unique_id;
	return ERROR_OK;

MATCH_EQUAL:
	ret = find_trigger(target, CSR_TDATA1_TYPE_MCONTROL6, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	tdata1 = set_field(tdata1, CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_EQUAL);
	if (trigger->length == 1)
		tdata1 = set_field(tdata1, CSR_MCONTROL6_SIZE, CSR_MCONTROL6_SIZE_8BIT);
	tdata1 = set_field(tdata1, CSR_MCONTROL6_CHAIN, CSR_MCONTROL6_CHAIN_DISABLED);
	tdata2 = trigger->address;
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK)
		return ret;
	r->trigger_unique_id[idx] = trigger->unique_id;
	return ERROR_OK;
}

static int add_trigger(struct target *target, struct trigger *trigger)
{
	int ret;
	riscv_reg_t tselect;

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	int result = riscv_get_register(target, &tselect, GDB_REGNO_TSELECT);
	if (result != ERROR_OK)
		return result;

	do {
		ret = maybe_add_trigger_t1(target, trigger);
		if (ret == ERROR_OK)
			break;
		ret = maybe_add_trigger_t2(target, trigger);
		if (ret == ERROR_OK)
			break;
		ret = maybe_add_trigger_t6(target, trigger);
		if (ret == ERROR_OK)
			break;
	} while (0);

	riscv_set_register(target, GDB_REGNO_TSELECT, tselect);

	return ret;
}

/**
 * Write one memory item of given "size". Use memory access of given "access_size".
 * Utilize read-modify-write, if needed.
 * */
static int write_by_given_size(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer, uint32_t access_size)
{
	assert(size == 1 || size == 2 || size == 4 || size == 8);
	assert(access_size == 1 || access_size == 2 || access_size == 4 || access_size == 8);

	if (access_size <= size && address % access_size == 0)
		/* Can do the memory access directly without a helper buffer. */
		return target_write_memory(target, address, access_size, size / access_size, buffer);

	unsigned int offset_head = address % access_size;
	unsigned int n_blocks = ((size + offset_head) <= access_size) ? 1 : 2;
	uint8_t helper_buf[n_blocks * access_size];

	/* Read from memory */
	if (target_read_memory(target, address - offset_head, access_size, n_blocks, helper_buf) != ERROR_OK)
		return ERROR_FAIL;

	/* Modify and write back */
	memcpy(helper_buf + offset_head, buffer, size);
	return target_write_memory(target, address - offset_head, access_size, n_blocks, helper_buf);
}

/**
 * Read one memory item of given "size". Use memory access of given "access_size".
 * Read larger section of memory and pick out the required portion, if needed.
 * */
static int read_by_given_size(struct target *target, target_addr_t address,
	uint32_t size, uint8_t *buffer, uint32_t access_size)
{
	assert(size == 1 || size == 2 || size == 4 || size == 8);
	assert(access_size == 1 || access_size == 2 || access_size == 4 || access_size == 8);

	if (access_size <= size && address % access_size == 0)
		/* Can do the memory access directly without a helper buffer. */
		return target_read_memory(target, address, access_size, size / access_size, buffer);

	unsigned int offset_head = address % access_size;
	unsigned int n_blocks = ((size + offset_head) <= access_size) ? 1 : 2;
	uint8_t helper_buf[n_blocks * access_size];

	/* Read from memory */
	if (target_read_memory(target, address - offset_head, access_size, n_blocks, helper_buf) != ERROR_OK)
		return ERROR_FAIL;

	/* Pick the requested portion from the buffer */
	memcpy(buffer, helper_buf + offset_head, size);
	return ERROR_OK;
}

/**
 * Write one memory item using any memory access size that will work.
 * Utilize read-modify-write, if needed.
 * */
int riscv_write_by_any_size(struct target *target, target_addr_t address, uint32_t size, uint8_t *buffer)
{
	assert(size == 1 || size == 2 ||  size == 4 || size == 8);

	/* Find access size that correspond to data size and the alignment. */
	unsigned int preferred_size = size;
	while (address % preferred_size != 0)
		preferred_size /= 2;

	/* First try the preferred (most natural) access size. */
	if (write_by_given_size(target, address, size, buffer, preferred_size) == ERROR_OK)
		return ERROR_OK;

	/* On failure, try other access sizes.
	   Minimize the number of accesses by trying first the largest size. */
	for (unsigned int access_size = 8; access_size > 0; access_size /= 2) {
		if (access_size == preferred_size)
			/* Already tried this size. */
			continue;

		if (write_by_given_size(target, address, size, buffer, access_size) == ERROR_OK)
			return ERROR_OK;
	}

	/* No access attempt succeeded. */
	return ERROR_FAIL;
}

/**
 * Read one memory item using any memory access size that will work.
 * Read larger section of memory and pick out the required portion, if needed.
 * */
int riscv_read_by_any_size(struct target *target, target_addr_t address, uint32_t size, uint8_t *buffer)
{
	assert(size == 1 || size == 2 ||  size == 4 || size == 8);

	/* Find access size that correspond to data size and the alignment. */
	unsigned int preferred_size = size;
	while (address % preferred_size != 0)
		preferred_size /= 2;

	/* First try the preferred (most natural) access size. */
	if (read_by_given_size(target, address, size, buffer, preferred_size) == ERROR_OK)
		return ERROR_OK;

	/* On failure, try other access sizes.
	   Minimize the number of accesses by trying first the largest size. */
	for (unsigned int access_size = 8; access_size > 0; access_size /= 2) {
		if (access_size == preferred_size)
			/* Already tried this size. */
			continue;

		if (read_by_given_size(target, address, size, buffer, access_size) == ERROR_OK)
			return ERROR_OK;
	}

	/* No access attempt succeeded. */
	return ERROR_FAIL;
}

int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	LOG_TARGET_DEBUG(target, "@0x%" TARGET_PRIxADDR, breakpoint->address);
	assert(breakpoint);
	if (breakpoint->type == BKPT_SOFT) {
		/** @todo check RVC for size/alignment */
		if (!(breakpoint->length == 4 || breakpoint->length == 2)) {
			LOG_ERROR("Invalid breakpoint length %d", breakpoint->length);
			return ERROR_FAIL;
		}

		if (0 != (breakpoint->address % 2)) {
			LOG_ERROR("Invalid breakpoint alignment for address 0x%" TARGET_PRIxADDR, breakpoint->address);
			return ERROR_FAIL;
		}

		/* Read the original instruction. */
		if (riscv_read_by_any_size(
				target, breakpoint->address, breakpoint->length, breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to read original instruction at 0x%" TARGET_PRIxADDR,
					breakpoint->address);
			return ERROR_FAIL;
		}

		uint8_t buff[4] = { 0 };
		buf_set_u32(buff, 0, breakpoint->length * CHAR_BIT, breakpoint->length == 4 ? ebreak() : ebreak_c());
		/* Write the ebreak instruction. */
		if (riscv_write_by_any_size(target, breakpoint->address, breakpoint->length, buff) != ERROR_OK) {
			LOG_ERROR("Failed to write %d-byte breakpoint instruction at 0x%"
					TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int const result = add_trigger(target, &trigger);
		if (result != ERROR_OK)
			return result;
	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->is_set = true;
	return ERROR_OK;
}

static int remove_trigger(struct target *target, int unique_id)
{
	RISCV_INFO(r);

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_reg_t tselect;
	int result = riscv_get_register(target, &tselect, GDB_REGNO_TSELECT);
	if (result != ERROR_OK)
		return result;

	bool done = false;
	for (unsigned int i = 0; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == unique_id) {
			riscv_set_register(target, GDB_REGNO_TSELECT, i);
			riscv_set_register(target, GDB_REGNO_TDATA1, 0);
			r->trigger_unique_id[i] = -1;
			LOG_TARGET_DEBUG(target, "Stop using resource %d for bp %d",
				i, unique_id);
			done = true;
		}
	}
	if (!done) {
		LOG_ERROR("Couldn't find the hardware resources used by hardware "
				"trigger.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register(target, GDB_REGNO_TSELECT, tselect);

	return ERROR_OK;
}

int riscv_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		/* Write the original instruction. */
		if (riscv_write_by_any_size(
				target, breakpoint->address, breakpoint->length, breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to restore instruction for %d-byte breakpoint at "
					"0x%" TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = remove_trigger(target, trigger.unique_id);
		if (result != ERROR_OK)
			return result;

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->is_set = false;

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
	watchpoint->is_set = true;

	return ERROR_OK;
}

int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	LOG_DEBUG("[%d] @0x%" TARGET_PRIxADDR, target->coreid, watchpoint->address);

	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = remove_trigger(target, trigger.unique_id);
	if (result != ERROR_OK)
		return result;
	watchpoint->is_set = false;

	return ERROR_OK;
}

/**
 * Look at the trigger hit bits to find out which trigger is the reason we're
 * halted.  Sets *unique_id to the unique ID of that trigger. If *unique_id is
 * ~0, no match was found.
 */
static int riscv_hit_trigger_hit_bit(struct target *target, uint32_t *unique_id)
{
	RISCV_INFO(r);

	riscv_reg_t tselect;
	if (riscv_get_register(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
		return ERROR_FAIL;

	*unique_id = ~0;
	for (unsigned int i = 0; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == -1)
			continue;

		if (riscv_set_register(target, GDB_REGNO_TSELECT, i) != ERROR_OK)
			return ERROR_FAIL;

		uint64_t tdata1;
		if (riscv_get_register(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
			return ERROR_FAIL;
		int type = get_field(tdata1, CSR_TDATA1_TYPE(riscv_xlen(target)));

		uint64_t hit_mask = 0;
		switch (type) {
			case CSR_TDATA1_TYPE_LEGACY:
				/* Doesn't support hit bit. */
				break;
			case CSR_TDATA1_TYPE_MCONTROL:
				hit_mask = CSR_MCONTROL_HIT;
				break;
			case CSR_TDATA1_TYPE_MCONTROL6:
				hit_mask = CSR_MCONTROL6_HIT;
				break;
			case CSR_TDATA1_TYPE_ITRIGGER:
				hit_mask = CSR_ITRIGGER_HIT(riscv_xlen(target));
				break;
			case CSR_TDATA1_TYPE_ETRIGGER:
				hit_mask = CSR_ETRIGGER_HIT(riscv_xlen(target));
				break;
			default:
				LOG_DEBUG("trigger %d has unknown type %d", i, type);
				continue;
		}

		/* Note: If we ever use chained triggers, then this logic needs
		 * to be changed to ignore triggers that are not the last one in
		 * the chain. */
		if (tdata1 & hit_mask) {
			LOG_DEBUG("Trigger %d (unique_id=%d) has hit bit set.", i, r->trigger_unique_id[i]);
			if (riscv_set_register(target, GDB_REGNO_TDATA1, tdata1 & ~hit_mask) != ERROR_OK)
				return ERROR_FAIL;

			*unique_id = r->trigger_unique_id[i];
			break;
		}
	}

	if (riscv_set_register(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/* Sets *hit_watchpoint to the first watchpoint identified as causing the
 * current halt.
 *
 * The GDB server uses this information to tell GDB what data address has
 * been hit, which enables GDB to print the hit variable along with its old
 * and new value. */
int riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	RISCV_INFO(r);

	LOG_TARGET_DEBUG(target, "Hit Watchpoint");

	/* If we identified which trigger caused the halt earlier, then just use
	 * that. */
	for (struct watchpoint *wp = target->watchpoints; wp; wp = wp->next) {
		if (wp->unique_id == r->trigger_hit) {
			*hit_watchpoint = wp;
			return ERROR_OK;
		}
	}

	riscv_reg_t dpc;
	riscv_get_register(target, &dpc, GDB_REGNO_DPC);
	const uint8_t length = 4;
	LOG_DEBUG("dpc is 0x%" PRIx64, dpc);

	/* fetch the instruction at dpc */
	uint8_t buffer[length];
	if (target_read_buffer(target, dpc, length, buffer) != ERROR_OK) {
		LOG_ERROR("Failed to read instruction at dpc 0x%" PRIx64, dpc);
		return ERROR_FAIL;
	}

	uint32_t instruction = 0;

	for (int i = 0; i < length; i++) {
		LOG_DEBUG("Next byte is %x", buffer[i]);
		instruction += (buffer[i] << 8 * i);
	}
	LOG_DEBUG("Full instruction is %x", instruction);

	/* find out which memory address is accessed by the instruction at dpc */
	/* opcode is first 7 bits of the instruction */
	uint8_t opcode = instruction & 0x7F;
	uint32_t rs1;
	int16_t imm;
	riscv_reg_t mem_addr;

	if (opcode == MATCH_LB || opcode == MATCH_SB) {
		rs1 = (instruction & 0xf8000) >> 15;
		riscv_get_register(target, &mem_addr, rs1);

		if (opcode == MATCH_SB) {
			LOG_DEBUG("%x is store instruction", instruction);
			imm = ((instruction & 0xf80) >> 7) | ((instruction & 0xfe000000) >> 20);
		} else {
			LOG_DEBUG("%x is load instruction", instruction);
			imm = (instruction & 0xfff00000) >> 20;
		}
		/* sign extend 12-bit imm to 16-bits */
		if (imm & (1 << 11))
			imm |= 0xf000;
		mem_addr += imm;
		LOG_DEBUG("memory address=0x%" PRIx64, mem_addr);
	} else {
		LOG_DEBUG("%x is not a RV32I load or store", instruction);
		return ERROR_FAIL;
	}

	struct watchpoint *wp = target->watchpoints;
	while (wp) {
		/*TODO support length/mask */
		if (wp->address == mem_addr) {
			*hit_watchpoint = wp;
			LOG_DEBUG("Hit address=%" TARGET_PRIxADDR, wp->address);
			return ERROR_OK;
		}
		wp = wp->next;
	}

	/* No match found - either we hit a watchpoint caused by an instruction that
	 * this function does not yet disassemble, or we hit a breakpoint.
	 *
	 * OpenOCD will behave as if this function had never been implemented i.e.
	 * report the halt to GDB with no address information. */
	return ERROR_FAIL;
}

static int oldriscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	struct target_type *tt = get_target_type(target);
	return tt->step(target, current, address, handle_breakpoints);
}

static int old_or_new_riscv_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	RISCV_INFO(r);
	LOG_DEBUG("handle_breakpoints=%d", handle_breakpoints);
	if (!r->get_hart_state)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else
		return riscv_openocd_step(target, current, address, handle_breakpoints);
}

static int riscv_examine(struct target *target)
{
	LOG_DEBUG("[%s]", target_name(target));
	if (target_was_examined(target)) {
		LOG_DEBUG("Target was already examined.");
		return ERROR_OK;
	}

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	RISCV_INFO(info);
	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("  version=0x%x", info->dtm_version);

	struct target_type *tt = get_target_type(target);
	if (!tt)
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
	if (!r->get_hart_state)
		return oldriscv_poll(target);
	else
		return riscv_openocd_poll(target);
}

int riscv_flush_registers(struct target *target)
{
	RISCV_INFO(r);

	if (!target->reg_cache)
		return ERROR_OK;

	LOG_DEBUG("[%s]", target_name(target));

	for (uint32_t number = 0; number < target->reg_cache->num_regs; number++) {
		struct reg *reg = &target->reg_cache->reg_list[number];
		if (reg->valid && reg->dirty) {
			uint64_t value = buf_get_u64(reg->value, 0, reg->size);
			LOG_DEBUG("[%s] %s is dirty; write back 0x%" PRIx64,
				  target_name(target), reg->name, value);
			int result = r->set_register(target, number, value);
			if (result != ERROR_OK)
				return ERROR_FAIL;
			reg->dirty = false;
		}
	}

	return ERROR_OK;
}

/* Convert: RISC-V hart's halt reason --> OpenOCD's generic debug reason */
int set_debug_reason(struct target *target, enum riscv_halt_reason halt_reason)
{
	RISCV_INFO(r);
	r->trigger_hit = -1;
	switch (halt_reason) {
		case RISCV_HALT_EBREAK:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case RISCV_HALT_TRIGGER:
			if (riscv_hit_trigger_hit_bit(target, &r->trigger_hit) != ERROR_OK)
				return ERROR_FAIL;
			target->debug_reason = DBG_REASON_WATCHPOINT;
			/* Check if we hit a hardware breakpoint. */
			for (struct breakpoint *bp = target->breakpoints; bp; bp = bp->next) {
				if (bp->unique_id == r->trigger_hit)
					target->debug_reason = DBG_REASON_BREAKPOINT;
			}
			break;
		case RISCV_HALT_INTERRUPT:
		case RISCV_HALT_GROUP:
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
	LOG_DEBUG("[%s] debug_reason=%d", target_name(target), target->debug_reason);

	return ERROR_OK;
}

int halt_prep(struct target *target)
{
	RISCV_INFO(r);

	LOG_DEBUG("[%s] prep hart, debug_reason=%d", target_name(target),
				target->debug_reason);
	r->prepped = false;
	if (target->state == TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "Hart is already halted.");
	} else if (target->state == TARGET_UNAVAILABLE) {
		LOG_TARGET_DEBUG(target, "Hart is unavailable.");
	} else {
		if (r->halt_prep(target) != ERROR_OK)
			return ERROR_FAIL;
		r->prepped = true;
	}

	return ERROR_OK;
}

int riscv_halt_go_all_harts(struct target *target)
{
	RISCV_INFO(r);

	enum riscv_hart_state state;
	if (riscv_get_hart_state(target, &state) != ERROR_OK)
		return ERROR_FAIL;
	if (state == RISCV_STATE_HALTED) {
		LOG_DEBUG("[%s] Hart is already halted.", target_name(target));
	} else {
		if (r->halt_go(target) != ERROR_OK)
			return ERROR_FAIL;

		riscv_invalidate_register_cache(target);
	}

	return ERROR_OK;
}

int halt_go(struct target *target)
{
	riscv_info_t *r = riscv_info(target);
	int result;
	if (!r->get_hart_state) {
		struct target_type *tt = get_target_type(target);
		result = tt->halt(target);
	} else {
		result = riscv_halt_go_all_harts(target);
	}
	if (target->debug_reason == DBG_REASON_NOTHALTED)
		target->debug_reason = DBG_REASON_DBGRQ;

	return result;
}

static int halt_finish(struct target *target)
{
	return target_call_event_callbacks(target, TARGET_EVENT_HALTED);
}

int riscv_halt(struct target *target)
{
	RISCV_INFO(r);

	if (!r->get_hart_state) {
		struct target_type *tt = get_target_type(target);
		return tt->halt(target);
	}

	LOG_TARGET_DEBUG(target, "halting all harts");

	int result = ERROR_OK;
	if (target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			if (halt_prep(t) != ERROR_OK)
				result = ERROR_FAIL;
		}

		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			riscv_info_t *i = riscv_info(t);
			if (i->prepped) {
				if (halt_go(t) != ERROR_OK)
					result = ERROR_FAIL;
			}
		}

		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			if (halt_finish(t) != ERROR_OK)
				return ERROR_FAIL;
		}

	} else {
		if (halt_prep(target) != ERROR_OK)
			result = ERROR_FAIL;
		if (halt_go(target) != ERROR_OK)
			result = ERROR_FAIL;
		if (halt_finish(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	return result;
}

static int riscv_assert_reset(struct target *target)
{
	LOG_DEBUG("[%d]", target->coreid);
	struct target_type *tt = get_target_type(target);
	riscv_invalidate_register_cache(target);
	return tt->assert_reset(target);
}

static int riscv_deassert_reset(struct target *target)
{
	LOG_DEBUG("[%d]", target->coreid);
	struct target_type *tt = get_target_type(target);
	return tt->deassert_reset(target);
}

/* state must be riscv_reg_t state[RISCV_MAX_HWBPS] = {0}; */
static int disable_triggers(struct target *target, riscv_reg_t *state)
{
	RISCV_INFO(r);

	LOG_DEBUG("deal with triggers");

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	if (r->manual_hwbp_set) {
		/* Look at every trigger that may have been set. */
		riscv_reg_t tselect;
		if (riscv_get_register(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
			return ERROR_FAIL;
		for (unsigned int t = 0; t < r->trigger_count; t++) {
			if (riscv_set_register(target, GDB_REGNO_TSELECT, t) != ERROR_OK)
				return ERROR_FAIL;
			riscv_reg_t tdata1;
			if (riscv_get_register(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
				return ERROR_FAIL;
			if (tdata1 & CSR_TDATA1_DMODE(riscv_xlen(target))) {
				state[t] = tdata1;
				if (riscv_set_register(target, GDB_REGNO_TDATA1, 0) != ERROR_OK)
					return ERROR_FAIL;
			}
		}
		if (riscv_set_register(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
			return ERROR_FAIL;

	} else {
		/* Just go through the triggers we manage. */
		struct watchpoint *watchpoint = target->watchpoints;
		int i = 0;
		while (watchpoint) {
			LOG_DEBUG("watchpoint %d: set=%d", i, watchpoint->is_set);
			state[i] = watchpoint->is_set;
			if (watchpoint->is_set) {
				if (riscv_remove_watchpoint(target, watchpoint) != ERROR_OK)
					return ERROR_FAIL;
			}
			watchpoint = watchpoint->next;
			i++;
		}
	}

	return ERROR_OK;
}

static int enable_triggers(struct target *target, riscv_reg_t *state)
{
	RISCV_INFO(r);

	if (r->manual_hwbp_set) {
		/* Look at every trigger that may have been set. */
		riscv_reg_t tselect;
		if (riscv_get_register(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
			return ERROR_FAIL;
		for (unsigned int t = 0; t < r->trigger_count; t++) {
			if (state[t] != 0) {
				if (riscv_set_register(target, GDB_REGNO_TSELECT, t) != ERROR_OK)
					return ERROR_FAIL;
				if (riscv_set_register(target, GDB_REGNO_TDATA1, state[t]) != ERROR_OK)
					return ERROR_FAIL;
			}
		}
		if (riscv_set_register(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
			return ERROR_FAIL;

	} else {
		struct watchpoint *watchpoint = target->watchpoints;
		int i = 0;
		while (watchpoint) {
			LOG_DEBUG("watchpoint %d: cleared=%" PRId64, i, state[i]);
			if (state[i]) {
				if (riscv_add_watchpoint(target, watchpoint) != ERROR_OK)
					return ERROR_FAIL;
			}
			watchpoint = watchpoint->next;
			i++;
		}
	}

	return ERROR_OK;
}

/**
 * Get everything ready to resume.
 */
static int resume_prep(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	RISCV_INFO(r);
	LOG_TARGET_DEBUG(target, "target->state=%d", target->state);

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		/* To be able to run off a trigger, disable all the triggers, step, and
		 * then resume as usual. */
		riscv_reg_t trigger_state[RISCV_MAX_HWBPS] = {0};

		if (disable_triggers(target, trigger_state) != ERROR_OK)
			return ERROR_FAIL;

		if (old_or_new_riscv_step(target, true, 0, false) != ERROR_OK)
			return ERROR_FAIL;

		if (enable_triggers(target, trigger_state) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (r->get_hart_state) {
		if (r->resume_prep(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	LOG_DEBUG("[%d] mark as prepped", target->coreid);
	r->prepped = true;

	return ERROR_OK;
}

/**
 * Resume all the harts that have been prepped, as close to instantaneous as
 * possible.
 */
static int resume_go(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	riscv_info_t *r = riscv_info(target);
	int result;
	if (!r->get_hart_state) {
		struct target_type *tt = get_target_type(target);
		result = tt->resume(target, current, address, handle_breakpoints,
				debug_execution);
	} else {
		result = riscv_resume_go_all_harts(target);
	}

	return result;
}

static int resume_finish(struct target *target, int debug_execution)
{
	register_cache_invalidate(target->reg_cache);

	target->state = debug_execution ? TARGET_DEBUG_RUNNING : TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return target_call_event_callbacks(target,
		debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);
}

/**
 * @par single_hart When true, only resume a single hart even if SMP is
 * configured.  This is used to run algorithms on just one hart.
 */
int riscv_resume(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution,
		bool single_hart)
{
	LOG_DEBUG("handle_breakpoints=%d", handle_breakpoints);
	int result = ERROR_OK;

	struct list_head *targets;

	LIST_HEAD(single_target_list);
	struct target_list single_target_entry = {
		.lh = {NULL, NULL},
		.target = target
	};

	if (target->smp && !single_hart) {
		targets = target->smp_targets;
	} else {
		/* Make a list that just contains a single target, so we can
		 * share code below. */
		list_add(&single_target_entry.lh, &single_target_list);
		targets = &single_target_list;
	}

	struct target_list *tlist;
	foreach_smp_target_direction(resume_order == RO_NORMAL, tlist, targets) {
		struct target *t = tlist->target;
		if (t->state != TARGET_UNAVAILABLE) {
			if (resume_prep(t, current, address, handle_breakpoints,
						debug_execution) != ERROR_OK)
				result = ERROR_FAIL;
		}
	}

	foreach_smp_target_direction(resume_order == RO_NORMAL, tlist, targets) {
		struct target *t = tlist->target;
		riscv_info_t *i = riscv_info(t);
		if (i->prepped) {
			if (resume_go(t, current, address, handle_breakpoints,
						debug_execution) != ERROR_OK)
				result = ERROR_FAIL;
		}
	}

	foreach_smp_target_direction(resume_order == RO_NORMAL, tlist, targets) {
		struct target *t = tlist->target;
		if (t->state != TARGET_UNAVAILABLE) {
			if (resume_finish(t, debug_execution) != ERROR_OK)
				result = ERROR_FAIL;
		}
	}

	return result;
}

static int riscv_target_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	return riscv_resume(target, current, address, handle_breakpoints,
			debug_execution, false);
}

static int riscv_mmu(struct target *target, int *enabled)
{
	if (!riscv_enable_virt2phys) {
		*enabled = 0;
		return ERROR_OK;
	}

	/* Don't use MMU in explicit or effective M (machine) mode */
	riscv_reg_t priv;
	if (riscv_get_register(target, &priv, GDB_REGNO_PRIV) != ERROR_OK) {
		LOG_ERROR("Failed to read priv register.");
		return ERROR_FAIL;
	}

	riscv_reg_t mstatus;
	if (riscv_get_register(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK) {
		LOG_ERROR("Failed to read mstatus register.");
		return ERROR_FAIL;
	}

	if ((get_field(mstatus, MSTATUS_MPRV) ? get_field(mstatus, MSTATUS_MPP) : priv) == PRV_M) {
		LOG_DEBUG("SATP/MMU ignored in Machine mode (mstatus=0x%" PRIx64 ").", mstatus);
		*enabled = 0;
		return ERROR_OK;
	}

	riscv_reg_t satp;
	if (riscv_get_register(target, &satp, GDB_REGNO_SATP) != ERROR_OK) {
		LOG_DEBUG("Couldn't read SATP.");
		/* If we can't read SATP, then there must not be an MMU. */
		*enabled = 0;
		return ERROR_OK;
	}

	if (get_field(satp, RISCV_SATP_MODE(riscv_xlen(target))) == SATP_MODE_OFF) {
		LOG_DEBUG("MMU is disabled.");
		*enabled = 0;
	} else {
		LOG_DEBUG("MMU is enabled.");
		*enabled = 1;
	}

	return ERROR_OK;
}

static int riscv_address_translate(struct target *target,
		target_addr_t virtual, target_addr_t *physical)
{
	RISCV_INFO(r);
	riscv_reg_t satp_value;
	int mode;
	uint64_t ppn_value;
	target_addr_t table_address;
	const virt2phys_info_t *info;
	uint64_t pte = 0;
	int i;

	int result = riscv_get_register(target, &satp_value, GDB_REGNO_SATP);
	if (result != ERROR_OK)
		return result;

	unsigned xlen = riscv_xlen(target);
	mode = get_field(satp_value, RISCV_SATP_MODE(xlen));
	switch (mode) {
		case SATP_MODE_SV32:
			info = &sv32;
			break;
		case SATP_MODE_SV39:
			info = &sv39;
			break;
		case SATP_MODE_SV48:
			info = &sv48;
			break;
		case SATP_MODE_OFF:
			LOG_ERROR("No translation or protection." \
				      " (satp: 0x%" PRIx64 ")", satp_value);
			return ERROR_FAIL;
		default:
			LOG_ERROR("The translation mode is not supported." \
				      " (satp: 0x%" PRIx64 ")", satp_value);
			return ERROR_FAIL;
	}
	LOG_DEBUG("virtual=0x%" TARGET_PRIxADDR "; mode=%s", virtual, info->name);

	/* verify bits xlen-1:va_bits-1 are all equal */
	target_addr_t mask = ((target_addr_t)1 << (xlen - (info->va_bits - 1))) - 1;
	target_addr_t masked_msbs = (virtual >> (info->va_bits - 1)) & mask;
	if (masked_msbs != 0 && masked_msbs != mask) {
		LOG_ERROR("Virtual address 0x%" TARGET_PRIxADDR " is not sign-extended "
				"for %s mode.", virtual, info->name);
		return ERROR_FAIL;
	}

	ppn_value = get_field(satp_value, RISCV_SATP_PPN(xlen));
	table_address = ppn_value << RISCV_PGSHIFT;
	i = info->level - 1;
	while (i >= 0) {
		uint64_t vpn = virtual >> info->vpn_shift[i];
		vpn &= info->vpn_mask[i];
		target_addr_t pte_address = table_address +
									(vpn << info->pte_shift);
		uint8_t buffer[8];
		assert(info->pte_shift <= 3);
		int retval = r->read_memory(target, pte_address,
				4, (1 << info->pte_shift) / 4, buffer, 4);
		if (retval != ERROR_OK)
			return ERROR_FAIL;

		if (info->pte_shift == 2)
			pte = buf_get_u32(buffer, 0, 32);
		else
			pte = buf_get_u64(buffer, 0, 64);

		LOG_DEBUG("i=%d; PTE @0x%" TARGET_PRIxADDR " = 0x%" PRIx64, i,
				pte_address, pte);

		if (!(pte & PTE_V) || (!(pte & PTE_R) && (pte & PTE_W)))
			return ERROR_FAIL;

		if ((pte & PTE_R) || (pte & PTE_X)) /* Found leaf PTE. */
			break;

		i--;
		if (i < 0)
			break;
		ppn_value = pte >> PTE_PPN_SHIFT;
		table_address = ppn_value << RISCV_PGSHIFT;
	}

	if (i < 0) {
		LOG_ERROR("Couldn't find the PTE.");
		return ERROR_FAIL;
	}

	/* Make sure to clear out the high bits that may be set. */
	*physical = virtual & (((target_addr_t)1 << info->va_bits) - 1);

	while (i < info->level) {
		ppn_value = pte >> info->pte_ppn_shift[i];
		ppn_value &= info->pte_ppn_mask[i];
		*physical &= ~(((target_addr_t)info->pa_ppn_mask[i]) <<
				info->pa_ppn_shift[i]);
		*physical |= (ppn_value << info->pa_ppn_shift[i]);
		i++;
	}
	LOG_DEBUG("0x%" TARGET_PRIxADDR " -> 0x%" TARGET_PRIxADDR, virtual,
			*physical);

	return ERROR_OK;
}

static int riscv_virt2phys(struct target *target, target_addr_t virtual, target_addr_t *physical)
{
	int enabled;
	if (riscv_mmu(target, &enabled) == ERROR_OK) {
		if (!enabled)
			return ERROR_FAIL;

		if (riscv_address_translate(target, virtual, physical) == ERROR_OK)
			return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int riscv_read_phys_memory(struct target *target, target_addr_t phys_address,
			uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV_INFO(r);
	return r->read_memory(target, phys_address, size, count, buffer, size);
}

static int riscv_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (count == 0) {
		LOG_WARNING("0-length read from 0x%" TARGET_PRIxADDR, address);
		return ERROR_OK;
	}

	target_addr_t physical_addr;
	if (target->type->virt2phys(target, address, &physical_addr) == ERROR_OK)
		address = physical_addr;

	RISCV_INFO(r);
	return r->read_memory(target, address, size, count, buffer, size);
}

static int riscv_write_phys_memory(struct target *target, target_addr_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct target_type *tt = get_target_type(target);
	return tt->write_memory(target, phys_address, size, count, buffer);
}

static int riscv_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (count == 0) {
		LOG_WARNING("0-length write to 0x%" TARGET_PRIxADDR, address);
		return ERROR_OK;
	}

	target_addr_t physical_addr;
	if (target->type->virt2phys(target, address, &physical_addr) == ERROR_OK)
		address = physical_addr;

	struct target_type *tt = get_target_type(target);
	return tt->write_memory(target, address, size, count, buffer);
}

const char *riscv_get_gdb_arch(struct target *target)
{
	switch (riscv_xlen(target)) {
		case 32:
			return "riscv:rv32";
		case 64:
			return "riscv:rv64";
	}
	LOG_ERROR("Unsupported xlen: %d", riscv_xlen(target));
	return NULL;
}

static int riscv_get_gdb_reg_list_internal(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class, bool read)
{
	LOG_TARGET_DEBUG(target, "reg_class=%d, read=%d", reg_class, read);

	if (!target->reg_cache) {
		LOG_ERROR("Target not initialized. Return ERROR_FAIL.");
		return ERROR_FAIL;
	}

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = 33;
			break;
		case REG_CLASS_ALL:
			*reg_list_size = target->reg_cache->num_regs;
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
		if (read &&
				target->reg_cache->reg_list[i].exist &&
				!target->reg_cache->reg_list[i].valid) {
			if (target->reg_cache->reg_list[i].type->get(
						&target->reg_cache->reg_list[i]) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int riscv_get_gdb_reg_list_noread(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	return riscv_get_gdb_reg_list_internal(target, reg_list, reg_list_size,
			reg_class, false);
}

static int riscv_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	return riscv_get_gdb_reg_list_internal(target, reg_list, reg_list_size,
			reg_class, true);
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
	RISCV_INFO(info);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Write memory parameters to the target memory */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_OUT ||
				mem_params[i].direction == PARAM_IN_OUT) {
			int retval = target_write_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
			if (retval != ERROR_OK) {
				LOG_ERROR("Couldn't write input mem param into the memory, addr=0x%" TARGET_PRIxADDR " size=0x%" PRIx32,
						mem_params[i].address, mem_params[i].size);
				return retval;
			}
		}
	}

	/* Save registers */
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", true);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("saved_pc=0x%" PRIx64, saved_pc);

	uint64_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, false);
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

		if (reg_params[i].direction == PARAM_OUT || reg_params[i].direction == PARAM_IN_OUT) {
			if (r->type->set(r, reg_params[i].value) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	/* Disable Interrupts before attempting to run the algorithm. */
	uint64_t current_mstatus;
	uint64_t irq_disabled_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
	if (riscv_interrupts_disable(target, irq_disabled_mask, &current_mstatus) != ERROR_OK)
		return ERROR_FAIL;

	/* Run algorithm */
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (riscv_resume(target, 0, entry_point, 0, 1, true) != ERROR_OK)
		return ERROR_FAIL;

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %" PRId64 " ms.", now - start);
			riscv_halt(target);
			old_or_new_riscv_poll(target);
			enum gdb_regno regnums[] = {
				GDB_REGNO_RA, GDB_REGNO_SP, GDB_REGNO_GP, GDB_REGNO_TP,
				GDB_REGNO_T0, GDB_REGNO_T1, GDB_REGNO_T2, GDB_REGNO_FP,
				GDB_REGNO_S1, GDB_REGNO_A0, GDB_REGNO_A1, GDB_REGNO_A2,
				GDB_REGNO_A3, GDB_REGNO_A4, GDB_REGNO_A5, GDB_REGNO_A6,
				GDB_REGNO_A7, GDB_REGNO_S2, GDB_REGNO_S3, GDB_REGNO_S4,
				GDB_REGNO_S5, GDB_REGNO_S6, GDB_REGNO_S7, GDB_REGNO_S8,
				GDB_REGNO_S9, GDB_REGNO_S10, GDB_REGNO_S11, GDB_REGNO_T3,
				GDB_REGNO_T4, GDB_REGNO_T5, GDB_REGNO_T6,
				GDB_REGNO_PC,
				GDB_REGNO_MSTATUS, GDB_REGNO_MEPC, GDB_REGNO_MCAUSE,
			};
			for (unsigned i = 0; i < ARRAY_SIZE(regnums); i++) {
				enum gdb_regno regno = regnums[i];
				riscv_reg_t reg_value;
				if (riscv_get_register(target, &reg_value, regno) != ERROR_OK)
					break;
				LOG_ERROR("%s = 0x%" PRIx64, gdb_regno_name(regno), reg_value);
			}
			return ERROR_TARGET_TIMEOUT;
		}

		int result = old_or_new_riscv_poll(target);
		if (result != ERROR_OK)
			return result;
	}

	/* TODO: The current hart id might have been changed in poll(). */
	/* if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL; */

	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (exit_point && final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	/* Restore Interrupts */
	if (riscv_interrupts_restore(target, current_mstatus) != ERROR_OK)
		return ERROR_FAIL;

	/* Restore registers */
	uint8_t buf[8] = { 0 };
	buf_set_u64(buf, 0, info->xlen, saved_pc);
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN ||
				reg_params[i].direction == PARAM_IN_OUT) {
			struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, false);
			if (r->type->get(r) != ERROR_OK) {
				LOG_ERROR("get(%s) failed", r->name);
				return ERROR_FAIL;
			}
			buf_cpy(r->value, reg_params[i].value, reg_params[i].size);
		}
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, false);
		buf_set_u64(buf, 0, info->xlen, saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK) {
			LOG_ERROR("set(%s) failed", r->name);
			return ERROR_FAIL;
		}
	}

	/* Read memory parameters from the target memory */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN ||
				mem_params[i].direction == PARAM_IN_OUT) {
			int retval = target_read_buffer(target, mem_params[i].address, mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK) {
				LOG_ERROR("Couldn't read output mem param from the memory, addr=0x%" TARGET_PRIxADDR " size=0x%" PRIx32,
						mem_params[i].address, mem_params[i].size);
				return retval;
			}
		}
	}

	return ERROR_OK;
}

static int riscv_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count,
		uint32_t *checksum)
{
	struct working_area *crc_algorithm;
	struct reg_param reg_params[2];
	int retval;

	LOG_DEBUG("address=0x%" TARGET_PRIxADDR "; count=0x%" PRIx32, address, count);

	static const uint8_t riscv32_crc_code[] = {
#include "../../../contrib/loaders/checksum/riscv32_crc.inc"
	};
	static const uint8_t riscv64_crc_code[] = {
#include "../../../contrib/loaders/checksum/riscv64_crc.inc"
	};

	static const uint8_t *crc_code;

	unsigned xlen = riscv_xlen(target);
	unsigned crc_code_size;
	if (xlen == 32) {
		crc_code = riscv32_crc_code;
		crc_code_size = sizeof(riscv32_crc_code);
	} else {
		crc_code = riscv64_crc_code;
		crc_code_size = sizeof(riscv64_crc_code);
	}

	if (count < crc_code_size * 4) {
		/* Don't use the algorithm for relatively small buffers. It's faster
		 * just to read the memory.  target_checksum_memory() will take care of
		 * that if we fail. */
		return ERROR_FAIL;
	}

	retval = target_alloc_working_area(target, crc_code_size, &crc_algorithm);
	if (retval != ERROR_OK)
		return retval;

	if (crc_algorithm->address + crc_algorithm->size > address &&
			crc_algorithm->address < address + count) {
		/* Region to checksum overlaps with the work area we've been assigned.
		 * Bail. (Would be better to manually checksum what we read there, and
		 * use the algorithm for the rest.) */
		target_free_working_area(target, crc_algorithm);
		return ERROR_FAIL;
	}

	retval = target_write_buffer(target, crc_algorithm->address, crc_code_size,
			crc_code);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
				crc_algorithm->address, retval);
		target_free_working_area(target, crc_algorithm);
		return retval;
	}

	init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, address);
	buf_set_u64(reg_params[1].value, 0, xlen, count);

	/* 20 second timeout/megabyte */
	int timeout = 20000 * (1 + (count / (1024 * 1024)));

	retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
			crc_algorithm->address,
			0,	/* Leave exit point unspecified because we don't know. */
			timeout, NULL);

	if (retval == ERROR_OK)
		*checksum = buf_get_u32(reg_params[0].value, 0, 32);
	else
		LOG_ERROR("error executing RISC-V CRC algorithm");

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	target_free_working_area(target, crc_algorithm);

	LOG_DEBUG("checksum=0x%" PRIx32 ", result=%d", *checksum, retval);

	return retval;
}

/*** OpenOCD Helper Functions ***/

enum riscv_next_action {
	RPH_NONE,
	RPH_RESUME,
	RPH_REMAIN_HALTED
};
static int riscv_poll_hart(struct target *target, enum riscv_next_action *next_action)
{
	RISCV_INFO(r);

	LOG_TARGET_DEBUG(target, "polling, target->state=%d", target->state);

	*next_action = RPH_NONE;

	enum riscv_hart_state previous_riscv_state = 0;
	enum target_state previous_target_state = target->state;
	switch (target->state) {
		case TARGET_UNKNOWN:
			/* Special case, handled further down. */
			previous_riscv_state = RISCV_STATE_UNAVAILABLE;	/* Need to assign something. */
			break;
		case TARGET_RUNNING:
			previous_riscv_state = RISCV_STATE_RUNNING;
			break;
		case TARGET_HALTED:
			previous_riscv_state = RISCV_STATE_HALTED;
			break;
		case TARGET_RESET:
			previous_riscv_state = RISCV_STATE_HALTED;
			break;
		case TARGET_DEBUG_RUNNING:
			previous_riscv_state = RISCV_STATE_RUNNING;
			break;
		case TARGET_UNAVAILABLE:
			previous_riscv_state = RISCV_STATE_UNAVAILABLE;
			break;
	}

	/* If OpenOCD thinks we're running but this hart is halted then it's time
	 * to raise an event. */
	enum riscv_hart_state state;
	if (riscv_get_hart_state(target, &state) != ERROR_OK)
		return ERROR_FAIL;

	if (state == RISCV_STATE_NON_EXISTENT) {
		LOG_TARGET_ERROR(target, "Hart is non-existent!");
		return ERROR_FAIL;
	}

	if (state == RISCV_STATE_HALTED && timeval_ms() - r->last_activity > 100) {
		/* If we've been idle for a while, flush the register cache. Just in case
		 * OpenOCD is going to be disconnected without shutting down cleanly. */
		if (riscv_flush_registers(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (target->state == TARGET_UNKNOWN || state != previous_riscv_state) {
		switch (state) {
			case RISCV_STATE_HALTED:
				LOG_TARGET_DEBUG(target, "  triggered a halt; previous_target_state=%d",
					previous_target_state);
				target->state = TARGET_HALTED;
				enum riscv_halt_reason halt_reason = riscv_halt_reason(target);
				if (set_debug_reason(target, halt_reason) != ERROR_OK)
					return ERROR_FAIL;

				if (halt_reason == RISCV_HALT_EBREAK) {
					int retval;
					/* Detect if this EBREAK is a semihosting request. If so, handle it. */
					switch (riscv_semihosting(target, &retval)) {
						case SEMI_NONE:
							break;
						case SEMI_WAITING:
							/* This hart should remain halted. */
							*next_action = RPH_REMAIN_HALTED;
							break;
						case SEMI_HANDLED:
							/* This hart should be resumed, along with any other
							* harts that halted due to haltgroups. */
							*next_action = RPH_RESUME;
							return ERROR_OK;
						case SEMI_ERROR:
							return retval;
					}
				}

				r->on_halt(target);

				/* We shouldn't do the callbacks yet. What if
				 * there are multiple harts that halted at the
				 * same time? We need to set debug reason on each
				 * of them before calling a callback, which is
				 * going to figure out the "current thread". */

				r->halted_needs_event_callback = true;
				if (previous_target_state == TARGET_DEBUG_RUNNING)
					r->halted_callback_event = TARGET_EVENT_DEBUG_HALTED;
				else
					r->halted_callback_event = TARGET_EVENT_HALTED;
				break;

			case RISCV_STATE_RUNNING:
				LOG_TARGET_DEBUG(target, "  triggered running");
				target->state = TARGET_RUNNING;
				target->debug_reason = DBG_REASON_NOTHALTED;
				break;

			case RISCV_STATE_UNAVAILABLE:
				LOG_TARGET_DEBUG(target, "  became unavailable");
				LOG_TARGET_INFO(target, "became unavailable.");
				target->state = TARGET_UNAVAILABLE;
				break;

			case RISCV_STATE_NON_EXISTENT:
				LOG_TARGET_ERROR(target, "Hart is non-existent!");
				target->state = TARGET_UNAVAILABLE;
				break;
		}
	}

	return ERROR_OK;
}

int sample_memory(struct target *target)
{
	RISCV_INFO(r);

	if (!r->sample_buf.buf || !r->sample_config.enabled)
		return ERROR_OK;

	LOG_DEBUG("buf used/size: %d/%d", r->sample_buf.used, r->sample_buf.size);

	uint64_t start = timeval_ms();
	riscv_sample_buf_maybe_add_timestamp(target, true);
	int result = ERROR_OK;
	if (r->sample_memory) {
		result = r->sample_memory(target, &r->sample_buf, &r->sample_config,
									  start + TARGET_DEFAULT_POLLING_INTERVAL);
		if (result != ERROR_NOT_IMPLEMENTED)
			goto exit;
	}

	/* Default slow path. */
	while (timeval_ms() - start < TARGET_DEFAULT_POLLING_INTERVAL) {
		for (unsigned int i = 0; i < ARRAY_SIZE(r->sample_config.bucket); i++) {
			if (r->sample_config.bucket[i].enabled &&
					r->sample_buf.used + 1 + r->sample_config.bucket[i].size_bytes < r->sample_buf.size) {
				assert(i < RISCV_SAMPLE_BUF_TIMESTAMP_BEFORE);
				r->sample_buf.buf[r->sample_buf.used] = i;
				result = riscv_read_phys_memory(
					target, r->sample_config.bucket[i].address,
					r->sample_config.bucket[i].size_bytes, 1,
					r->sample_buf.buf + r->sample_buf.used + 1);
				if (result == ERROR_OK)
					r->sample_buf.used += 1 + r->sample_config.bucket[i].size_bytes;
				else
					goto exit;
			}
		}
	}

exit:
	riscv_sample_buf_maybe_add_timestamp(target, false);
	if (result != ERROR_OK) {
		LOG_INFO("Turning off memory sampling because it failed.");
		r->sample_config.enabled = false;
	}
	return result;
}

/*** OpenOCD Interface ***/
int riscv_openocd_poll(struct target *target)
{
	LOG_DEBUG("polling all harts");

	struct list_head *targets;

	LIST_HEAD(single_target_list);
	struct target_list single_target_entry = {
		.lh = {NULL, NULL},
		.target = target
	};

	if (target->smp) {
		targets = target->smp_targets;
	} else {
		/* Make a list that just contains a single target, so we can
		 * share code below. */
		list_add(&single_target_entry.lh, &single_target_list);
		targets = &single_target_list;
	}

	unsigned should_remain_halted = 0;
	unsigned should_resume = 0;
	unsigned halted = 0;
	unsigned running = 0;
	struct target_list *entry;
	foreach_smp_target(entry, targets) {
		struct target *t = entry->target;
		riscv_info_t *info = riscv_info(t);

		/* Clear here just in case there were errors and we never got to
		 * check this flag further down. */
		info->halted_needs_event_callback = false;

		if (!target_was_examined(t))
			continue;

		enum riscv_next_action next_action;
		if (riscv_poll_hart(t, &next_action) != ERROR_OK)
			return ERROR_FAIL;

		switch (next_action) {
			case RPH_NONE:
				if (t->state == TARGET_HALTED)
					halted++;
				if (t->state == TARGET_RUNNING ||
					t->state == TARGET_DEBUG_RUNNING)
					running++;
				break;
			case RPH_REMAIN_HALTED:
				should_remain_halted++;
				break;
			case RPH_RESUME:
				should_resume++;
				break;
		}
	}

	LOG_DEBUG("should_remain_halted=%d, should_resume=%d",
				should_remain_halted, should_resume);
	if (should_remain_halted && should_resume) {
		LOG_WARNING("%d harts should remain halted, and %d should resume.",
					should_remain_halted, should_resume);
	}
	if (should_remain_halted) {
		LOG_TARGET_DEBUG(target, "halt all; should_remain_halted=%d",
			should_remain_halted);
		riscv_halt(target);
	} else if (should_resume) {
		LOG_DEBUG("resume all");
		riscv_resume(target, true, 0, 0, 0, false);
	} else if (halted && running) {
		LOG_TARGET_DEBUG(target, "halt all; halted=%d",
			halted);
		riscv_halt(target);
	} else {
		/* For targets that were discovered to be halted, call the
		 * appropriate callback. */
		foreach_smp_target(entry, targets)
		{
			struct target *t = entry->target;
			riscv_info_t *info = riscv_info(t);
			if (info->halted_needs_event_callback) {
				target_call_event_callbacks(t, info->halted_callback_event);
				info->halted_needs_event_callback = false;
			}
		}
	}

	/* Sample memory if any target is running. */
	foreach_smp_target(entry, targets) {
		struct target *t = entry->target;
		if (t->state == TARGET_RUNNING) {
			sample_memory(target);
			break;
		}
	}

	return ERROR_OK;
}

int riscv_openocd_step(struct target *target, int current,
	target_addr_t address, int handle_breakpoints)
{
	LOG_TARGET_DEBUG(target, "stepping hart");

	if (!current) {
		if (riscv_set_register(target, GDB_REGNO_PC, address) != ERROR_OK)
			return ERROR_FAIL;
	}

	struct breakpoint *breakpoint = NULL;
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		if (current) {
			if (riscv_get_register(target, &address, GDB_REGNO_PC) != ERROR_OK)
				return ERROR_FAIL;
		}
		breakpoint = breakpoint_find(target, address);
		if (breakpoint && (riscv_remove_breakpoint(target, breakpoint) != ERROR_OK))
			return ERROR_FAIL;
	}

	riscv_reg_t trigger_state[RISCV_MAX_HWBPS] = {0};
	if (disable_triggers(target, trigger_state) != ERROR_OK)
		return ERROR_FAIL;

	bool success = true;
	uint64_t current_mstatus;
	RISCV_INFO(info);

	if (info->isrmask_mode == RISCV_ISRMASK_STEPONLY) {
		/* Disable Interrupts before stepping. */
		uint64_t irq_disabled_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
		if (riscv_interrupts_disable(target, irq_disabled_mask,
				&current_mstatus) != ERROR_OK) {
			success = false;
			LOG_ERROR("unable to disable interrupts");
			goto _exit;
		}
	}

	if (riscv_step_rtos_hart(target) != ERROR_OK) {
		success = false;
		LOG_ERROR("unable to step rtos hart");
	}

	register_cache_invalidate(target->reg_cache);

	if (info->isrmask_mode == RISCV_ISRMASK_STEPONLY)
		if (riscv_interrupts_restore(target, current_mstatus) != ERROR_OK) {
			success = false;
			LOG_ERROR("unable to restore interrupts");
		}

_exit:
	if (enable_triggers(target, trigger_state) != ERROR_OK) {
		success = false;
		LOG_ERROR("unable to enable triggers");
	}

	if (breakpoint && (riscv_add_breakpoint(target, breakpoint) != ERROR_OK)) {
		success = false;
		LOG_TARGET_ERROR(target, "unable to restore the disabled breakpoint");
	}

	if (success) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_SINGLESTEP;
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}
	return success ? ERROR_OK : ERROR_FAIL;
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
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);
	bool prefer_sba;
	LOG_WARNING("`riscv set_prefer_sba` is deprecated. Please use `riscv set_mem_access` instead.");
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], prefer_sba);
	if (prefer_sba) {
		/* Use system bus with highest priority */
		r->mem_access_methods[0] = RISCV_MEM_ACCESS_SYSBUS;
		r->mem_access_methods[1] = RISCV_MEM_ACCESS_PROGBUF;
		r->mem_access_methods[2] = RISCV_MEM_ACCESS_ABSTRACT;
	} else {
		/* Use progbuf with highest priority */
		r->mem_access_methods[0] = RISCV_MEM_ACCESS_PROGBUF;
		r->mem_access_methods[1] = RISCV_MEM_ACCESS_SYSBUS;
		r->mem_access_methods[2] = RISCV_MEM_ACCESS_ABSTRACT;
	}

	/* Reset warning flags */
	r->mem_access_progbuf_warn = true;
	r->mem_access_sysbus_warn = true;
	r->mem_access_abstract_warn = true;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_mem_access)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);
	int progbuf_cnt = 0;
	int sysbus_cnt = 0;
	int abstract_cnt = 0;

	if (CMD_ARGC < 1 || CMD_ARGC > RISCV_NUM_MEM_ACCESS_METHODS) {
		LOG_ERROR("Command takes 1 to %d parameters", RISCV_NUM_MEM_ACCESS_METHODS);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* Check argument validity */
	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		if (strcmp("progbuf", CMD_ARGV[i]) == 0) {
			progbuf_cnt++;
		} else if (strcmp("sysbus", CMD_ARGV[i]) == 0) {
			sysbus_cnt++;
		} else if (strcmp("abstract", CMD_ARGV[i]) == 0) {
			abstract_cnt++;
		} else {
			LOG_ERROR("Unknown argument '%s'. "
				"Must be one of: 'progbuf', 'sysbus' or 'abstract'.", CMD_ARGV[i]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}
	if (progbuf_cnt > 1 || sysbus_cnt > 1 || abstract_cnt > 1) {
		LOG_ERROR("Syntax error - duplicate arguments to `riscv set_mem_access`.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* Args are valid, store them */
	for (unsigned int i = 0; i < RISCV_NUM_MEM_ACCESS_METHODS; i++)
		r->mem_access_methods[i] = RISCV_MEM_ACCESS_UNSPECIFIED;
	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		if (strcmp("progbuf", CMD_ARGV[i]) == 0)
			r->mem_access_methods[i] = RISCV_MEM_ACCESS_PROGBUF;
		else if (strcmp("sysbus", CMD_ARGV[i]) == 0)
			r->mem_access_methods[i] = RISCV_MEM_ACCESS_SYSBUS;
		else if (strcmp("abstract", CMD_ARGV[i]) == 0)
			r->mem_access_methods[i] = RISCV_MEM_ACCESS_ABSTRACT;
	}

	/* Reset warning flags */
	r->mem_access_progbuf_warn = true;
	r->mem_access_sysbus_warn = true;
	r->mem_access_abstract_warn = true;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_enable_virtual)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_enable_virtual);
	return ERROR_OK;
}

int parse_ranges(struct list_head *ranges, const char *tcl_arg, const char *reg_type, unsigned int max_val)
{
	char *args = strdup(tcl_arg);
	if (!args)
		return ERROR_FAIL;

	/* For backward compatibility, allow multiple parameters within one TCL argument, separated by ',' */
	char *arg = strtok(args, ",");
	while (arg) {
		unsigned low = 0;
		unsigned high = 0;
		char *name = NULL;

		char *dash = strchr(arg, '-');
		char *equals = strchr(arg, '=');
		unsigned int pos;

		if (!dash && !equals) {
			/* Expecting single register number. */
			if (sscanf(arg, "%u%n", &low, &pos) != 1 || pos != strlen(arg)) {
				LOG_ERROR("Failed to parse single register number from '%s'.", arg);
				free(args);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		} else if (dash && !equals) {
			/* Expecting register range - two numbers separated by a dash: ##-## */
			*dash = 0;
			dash++;
			if (sscanf(arg, "%u%n", &low, &pos) != 1 || pos != strlen(arg)) {
				LOG_ERROR("Failed to parse single register number from '%s'.", arg);
				free(args);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			if (sscanf(dash, "%u%n", &high, &pos) != 1 || pos != strlen(dash)) {
				LOG_ERROR("Failed to parse single register number from '%s'.", dash);
				free(args);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			if (high < low) {
				LOG_ERROR("Incorrect range encountered [%u, %u].", low, high);
				free(args);
				return ERROR_FAIL;
			}
		} else if (!dash && equals) {
			/* Expecting single register number with textual name specified: ##=name */
			*equals = 0;
			equals++;
			if (sscanf(arg, "%u%n", &low, &pos) != 1 || pos != strlen(arg)) {
				LOG_ERROR("Failed to parse single register number from '%s'.", arg);
				free(args);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			name = calloc(1, strlen(equals) + strlen(reg_type) + 2);
			if (!name) {
				LOG_ERROR("Failed to allocate register name.");
				free(args);
				return ERROR_FAIL;
			}

			/* Register prefix: "csr_" or "custom_" */
			strcpy(name, reg_type);
			name[strlen(reg_type)] = '_';

			if (sscanf(equals, "%[_a-zA-Z0-9]%n", name + strlen(reg_type) + 1, &pos) != 1 || pos != strlen(equals)) {
				LOG_ERROR("Failed to parse register name from '%s'.", equals);
				free(args);
				free(name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		} else {
			LOG_ERROR("Invalid argument '%s'.", arg);
			free(args);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		high = high > low ? high : low;

		if (high > max_val) {
			LOG_ERROR("Cannot expose %s register number %u, maximum allowed value is %u.", reg_type, high, max_val);
			free(name);
			free(args);
			return ERROR_FAIL;
		}

		/* Check for overlap, name uniqueness. */
		range_list_t *entry;
		list_for_each_entry(entry, ranges, list) {
			if ((entry->low <= high) && (low <= entry->high)) {
				if (low == high)
					LOG_WARNING("Duplicate %s register number - "
							"Register %u has already been exposed previously", reg_type, low);
				else
					LOG_WARNING("Overlapping register ranges - Register range starting from %u overlaps "
							"with already exposed register/range at %u.", low, entry->low);
			}

			if (entry->name && name && (strcasecmp(entry->name, name) == 0)) {
				LOG_ERROR("Duplicate register name \"%s\" found.", name);
				free(name);
				free(args);
				return ERROR_FAIL;
			}
		}

		range_list_t *range = calloc(1, sizeof(range_list_t));
		if (!range) {
			LOG_ERROR("Failed to allocate range list.");
			free(name);
			free(args);
			return ERROR_FAIL;
		}

		range->low = low;
		range->high = high;
		range->name = name;
		list_add(&range->list, ranges);

		arg = strtok(NULL, ",");
	}

	free(args);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_expose_csrs)
{
	if (CMD_ARGC == 0) {
		LOG_ERROR("Command expects parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(info);
	int ret = ERROR_OK;

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		ret = parse_ranges(&info->expose_csr, CMD_ARGV[i], "csr", 0xfff);
		if (ret != ERROR_OK)
			break;
	}

	return ret;
}

COMMAND_HANDLER(riscv_set_expose_custom)
{
	if (CMD_ARGC == 0) {
		LOG_ERROR("Command expects parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(info);
	int ret = ERROR_OK;

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		ret = parse_ranges(&info->expose_custom, CMD_ARGV[i], "custom", 0x3fff);
		if (ret != ERROR_OK)
			break;
	}

	return ret;
}

COMMAND_HANDLER(riscv_authdata_read)
{
	unsigned int index = 0;
	if (CMD_ARGC == 0) {
		/* nop */
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], index);
	} else {
		LOG_ERROR("Command takes at most one parameter");
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
		if (r->authdata_read(target, &value, index) != ERROR_OK)
			return ERROR_FAIL;
		command_print_sameline(CMD, "0x%08" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("authdata_read is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_authdata_write)
{
	uint32_t value;
	unsigned int index = 0;

	if (CMD_ARGC == 0) {
		/* nop */
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], value);
	} else if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], index);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
	} else {
		LOG_ERROR("Command takes at most 2 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (r->authdata_write) {
		return r->authdata_write(target, value, index);
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
		command_print(CMD, "0x%" PRIx32, value);
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
		/* Perform the DMI write */
		int retval = r->dmi_write(target, address, value);

		/* Invalidate our cached progbuf copy:
		   - if the user tinkered directly with a progbuf register
		   - if debug module was reset, in which case progbuf registers
		     may not retain their value.
		*/
		bool progbufTouched = (address >= DM_PROGBUF0 && address <= DM_PROGBUF15);
		bool dmDeactivated = (address == DM_DMCONTROL && (value & DM_DMCONTROL_DMACTIVE) == 0);
		if (progbufTouched || dmDeactivated) {
			if (r->invalidate_cached_debug_buffer)
				r->invalidate_cached_debug_buffer(target);
		}

		return retval;
	}

	LOG_ERROR("dmi_write is not implemented for this target.");
	return ERROR_FAIL;
}

COMMAND_HANDLER(riscv_reset_delays)
{
	int wait = 0;

	if (CMD_ARGC > 1) {
		LOG_ERROR("Command takes at most one argument");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], wait);

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);
	r->reset_delays_wait = wait;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ir)
{
	if (CMD_ARGC != 2) {
		LOG_ERROR("Command takes exactly 2 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	if (!strcmp(CMD_ARGV[0], "idcode"))
		buf_set_u32(ir_idcode, 0, 32, value);
	else if (!strcmp(CMD_ARGV[0], "dtmcs"))
		buf_set_u32(ir_dtmcontrol, 0, 32, value);
	else if (!strcmp(CMD_ARGV[0], "dmi"))
		buf_set_u32(ir_dbus, 0, 32, value);
	else
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_resume_order)
{
	if (CMD_ARGC > 1) {
		LOG_ERROR("Command takes at most one argument");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!strcmp(CMD_ARGV[0], "normal")) {
		resume_order = RO_NORMAL;
	} else if (!strcmp(CMD_ARGV[0], "reversed")) {
		resume_order = RO_REVERSED;
	} else {
		LOG_ERROR("Unsupported resume order: %s", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_use_bscan_tunnel)
{
	int irwidth = 0;
	int tunnel_type = BSCAN_TUNNEL_NESTED_TAP;

	if (CMD_ARGC > 2) {
		LOG_ERROR("Command takes at most two arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], irwidth);
	} else if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], irwidth);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tunnel_type);
	}
	if (tunnel_type == BSCAN_TUNNEL_NESTED_TAP)
		LOG_INFO("Nested Tap based Bscan Tunnel Selected");
	else if (tunnel_type == BSCAN_TUNNEL_DATA_REGISTER)
		LOG_INFO("Simple Register based Bscan Tunnel Selected");
	else
		LOG_INFO("Invalid Tunnel type selected ! : selecting default Nested Tap Type");

	bscan_tunnel_type = tunnel_type;
	bscan_tunnel_ir_width = irwidth;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_bscan_tunnel_ir)
{
	int ir_id = 0;

	if (CMD_ARGC > 1) {
		LOG_ERROR("Command takes at most one arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], ir_id);
	}

	LOG_INFO("Bscan tunnel IR 0x%x selected", ir_id);

	bscan_tunnel_ir_id = ir_id;
	return ERROR_OK;
}


COMMAND_HANDLER(riscv_set_maskisr)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(info);

	static const struct jim_nvp nvp_maskisr_modes[] = {
		{ .name = "off", .value = RISCV_ISRMASK_OFF },
		{ .name = "steponly", .value = RISCV_ISRMASK_STEPONLY },
		{ .name = NULL, .value = -1 },
	};
	const struct jim_nvp *n;

	if (CMD_ARGC > 0) {
		n = jim_nvp_name2value_simple(nvp_maskisr_modes, CMD_ARGV[0]);
		if (!n->name)
			return ERROR_COMMAND_SYNTAX_ERROR;
		info->isrmask_mode = n->value;
	} else {
		n = jim_nvp_value2name_simple(nvp_maskisr_modes, info->isrmask_mode);
		command_print(CMD, "riscv interrupt mask %s", n->name);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_enable_virt2phys)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_enable_virt2phys);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ebreakm)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_ebreakm);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ebreaks)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_ebreaks);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ebreaku)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_ebreaku);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_itrigger)
{
	if (CMD_ARGC < 1) {
		LOG_ERROR("Command takes at least 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	const int ITRIGGER_UNIQUE_ID = -CSR_TDATA1_TYPE_ITRIGGER;

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	if (!strcmp(CMD_ARGV[0], "set")) {
		if (find_first_trigger_by_id(target, ITRIGGER_UNIQUE_ID) >= 0) {
			LOG_TARGET_ERROR(target, "An itrigger is already set, and OpenOCD "
				  "doesn't support setting more than one at a time.");
			return ERROR_FAIL;
		}
		bool vs = false;
		bool vu = false;
		bool nmi = false;
		bool m = false;
		bool s = false;
		bool u = false;
		riscv_reg_t interrupts = 0;

		for (unsigned int i = 1; i < CMD_ARGC; i++) {
			if (!strcmp(CMD_ARGV[i], "vs"))
				vs = true;
			else if (!strcmp(CMD_ARGV[i], "vu"))
				vu = true;
			else if (!strcmp(CMD_ARGV[i], "nmi"))
				nmi = true;
			else if (!strcmp(CMD_ARGV[i], "m"))
				m = true;
			else if (!strcmp(CMD_ARGV[i], "s"))
				s = true;
			else if (!strcmp(CMD_ARGV[i], "u"))
				u = true;
			else
				COMMAND_PARSE_NUMBER(u64, CMD_ARGV[i], interrupts);
		}
		if (!nmi && interrupts == 0) {
			LOG_ERROR("Doesn't make sense to set itrigger with "
				  "mie_bits=0 and without nmi.");
			return ERROR_FAIL;
		} else if (!vs && !vu && !m && !s && !u) {
			LOG_ERROR("Doesn't make sense to set itrigger without at "
				  "least one of vs, vu, m, s, or u.");
			return ERROR_FAIL;
		}
		int result = maybe_add_trigger_t4(target, vs, vu, nmi, m, s, u, interrupts, ITRIGGER_UNIQUE_ID);
		if (result != ERROR_OK)
			LOG_TARGET_ERROR(target, "Failed to set requested itrigger.");
		return result;

	} else if (!strcmp(CMD_ARGV[0], "clear")) {
		if (CMD_ARGC != 1) {
			LOG_ERROR("clear command takes no extra arguments.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (find_first_trigger_by_id(target, ITRIGGER_UNIQUE_ID) < 0) {
			LOG_TARGET_ERROR(target, "No itrigger is set. Nothing to clear.");
			return ERROR_FAIL;
		}
		return remove_trigger(target, ITRIGGER_UNIQUE_ID);

	} else {
		LOG_ERROR("First argument must be either 'set' or 'clear'.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_etrigger)
{
	if (CMD_ARGC < 1) {
		LOG_ERROR("Command takes at least 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	const int ETRIGGER_UNIQUE_ID = -CSR_TDATA1_TYPE_ETRIGGER;

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	if (!strcmp(CMD_ARGV[0], "set")) {
		if (find_first_trigger_by_id(target, ETRIGGER_UNIQUE_ID) >= 0) {
			LOG_TARGET_ERROR(target, "An etrigger is already set, and OpenOCD "
				  "doesn't support setting more than one at a time.");
			return ERROR_FAIL;
		}
		bool vs = false;
		bool vu = false;
		bool m = false;
		bool s = false;
		bool u = false;
		riscv_reg_t exception_codes = 0;

		for (unsigned int i = 1; i < CMD_ARGC; i++) {
			if (!strcmp(CMD_ARGV[i], "vs"))
				vs = true;
			else if (!strcmp(CMD_ARGV[i], "vu"))
				vu = true;
			else if (!strcmp(CMD_ARGV[i], "m"))
				m = true;
			else if (!strcmp(CMD_ARGV[i], "s"))
				s = true;
			else if (!strcmp(CMD_ARGV[i], "u"))
				u = true;
			else
				COMMAND_PARSE_NUMBER(u64, CMD_ARGV[i], exception_codes);
		}
		if (exception_codes == 0) {
			LOG_ERROR("Doesn't make sense to set etrigger with "
				  "exception_codes=0.");
			return ERROR_FAIL;
		} else if (!vs && !vu && !m && !s && !u) {
			LOG_ERROR("Doesn't make sense to set etrigger without at "
				  "least one of vs, vu, m, s, or u.");
			return ERROR_FAIL;
		}
		int result = maybe_add_trigger_t5(target, vs, vu, m, s, u, exception_codes, ETRIGGER_UNIQUE_ID);
		if (result != ERROR_OK)
			LOG_TARGET_ERROR(target, "Failed to set requested etrigger.");
		return result;

	} else if (!strcmp(CMD_ARGV[0], "clear")) {
		if (CMD_ARGC != 1) {
			LOG_ERROR("clear command takes no extra arguments.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (find_first_trigger_by_id(target, ETRIGGER_UNIQUE_ID) < 0) {
			LOG_TARGET_ERROR(target, "No etrigger is set. Nothing to clear.");
			return ERROR_FAIL;
		}
		return remove_trigger(target, ETRIGGER_UNIQUE_ID);

	} else {
		LOG_ERROR("First argument must be either 'set' or 'clear'.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_repeat_read)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (CMD_ARGC < 2) {
		LOG_ERROR("Command requires at least count and address arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (CMD_ARGC > 3) {
		LOG_ERROR("Command takes at most 3 arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t count;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], count);
	target_addr_t address;
	COMMAND_PARSE_ADDRESS(CMD_ARGV[1], address);
	uint32_t size = 4;
	if (CMD_ARGC > 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], size);

	if (count == 0)
		return ERROR_OK;

	uint8_t *buffer = malloc(size * count);
	if (!buffer) {
		LOG_ERROR("malloc failed");
		return ERROR_FAIL;
	}
	int result = r->read_memory(target, address, size, count, buffer, 0);
	if (result == ERROR_OK) {
		target_handle_md_output(cmd, target, address, size, count, buffer,
			false);
	}
	free(buffer);
	return result;
}

COMMAND_HANDLER(handle_memory_sample_command)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (CMD_ARGC == 0) {
		command_print(CMD, "Memory sample configuration for %s:", target_name(target));
		for (unsigned i = 0; i < ARRAY_SIZE(r->sample_config.bucket); i++) {
			if (r->sample_config.bucket[i].enabled) {
				command_print(CMD, "bucket %d; address=0x%" TARGET_PRIxADDR "; size=%d", i,
							  r->sample_config.bucket[i].address,
							  r->sample_config.bucket[i].size_bytes);
			} else {
				command_print(CMD, "bucket %d; disabled", i);
			}
		}
		return ERROR_OK;
	}

	if (CMD_ARGC < 2) {
		LOG_ERROR("Command requires at least bucket and address arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t bucket;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], bucket);
	if (bucket > ARRAY_SIZE(r->sample_config.bucket)) {
		LOG_ERROR("Max bucket number is %d.", (unsigned) ARRAY_SIZE(r->sample_config.bucket));
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (!strcmp(CMD_ARGV[1], "clear")) {
		r->sample_config.bucket[bucket].enabled = false;
	} else {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[1], r->sample_config.bucket[bucket].address);

		if (CMD_ARGC > 2) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], r->sample_config.bucket[bucket].size_bytes);
			if (r->sample_config.bucket[bucket].size_bytes != 4 &&
					r->sample_config.bucket[bucket].size_bytes != 8) {
				LOG_ERROR("Only 4-byte and 8-byte sizes are supported.");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
		} else {
			r->sample_config.bucket[bucket].size_bytes = 4;
		}

		r->sample_config.bucket[bucket].enabled = true;
	}

	if (!r->sample_buf.buf) {
		r->sample_buf.size = 1024 * 1024;
		r->sample_buf.buf = malloc(r->sample_buf.size);
	}

	/* Clear the buffer when the configuration is changed. */
	r->sample_buf.used = 0;

	r->sample_config.enabled = true;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_dump_sample_buf_command)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (CMD_ARGC > 1) {
		LOG_ERROR("Command takes at most 1 arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	bool base64 = false;
	if (CMD_ARGC > 0) {
		if (!strcmp(CMD_ARGV[0], "base64")) {
			base64 = true;
		} else {
			LOG_ERROR("Unknown argument: %s", CMD_ARGV[0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	int result = ERROR_OK;
	if (base64) {
		unsigned char *encoded = base64_encode(r->sample_buf.buf,
									  r->sample_buf.used, NULL);
		if (!encoded) {
			LOG_ERROR("Failed base64 encode!");
			result = ERROR_FAIL;
			goto error;
		}
		command_print(CMD, "%s", encoded);
		free(encoded);
	} else {
		unsigned i = 0;
		while (i < r->sample_buf.used) {
			uint8_t command = r->sample_buf.buf[i++];
			if (command == RISCV_SAMPLE_BUF_TIMESTAMP_BEFORE) {
				uint32_t timestamp = buf_get_u32(r->sample_buf.buf + i, 0, 32);
				i += 4;
				command_print(CMD, "timestamp before: %u", timestamp);
			} else if (command == RISCV_SAMPLE_BUF_TIMESTAMP_AFTER) {
				uint32_t timestamp = buf_get_u32(r->sample_buf.buf + i, 0, 32);
				i += 4;
				command_print(CMD, "timestamp after: %u", timestamp);
			} else if (command < ARRAY_SIZE(r->sample_config.bucket)) {
				command_print_sameline(CMD, "0x%" TARGET_PRIxADDR ": ",
									   r->sample_config.bucket[command].address);
				if (r->sample_config.bucket[command].size_bytes == 4) {
					uint32_t value = buf_get_u32(r->sample_buf.buf + i, 0, 32);
					i += 4;
					command_print(CMD, "0x%08" PRIx32, value);
				} else if (r->sample_config.bucket[command].size_bytes == 8) {
					uint64_t value = buf_get_u64(r->sample_buf.buf + i, 0, 64);
					i += 8;
					command_print(CMD, "0x%016" PRIx64, value);
				} else {
					LOG_ERROR("Found invalid size in bucket %d: %d", command,
							  r->sample_config.bucket[command].size_bytes);
					result = ERROR_FAIL;
					goto error;
				}
			} else {
				LOG_ERROR("Found invalid command byte in sample buf: 0x%2x at offset 0x%x",
					command, i - 1);
				result = ERROR_FAIL;
				goto error;
			}
		}
	}

error:
	/* Clear the sample buffer even when there was an error. */
	r->sample_buf.used = 0;
	return result;
}

COMMAND_HELPER(riscv_print_info_line, const char *section, const char *key,
			   unsigned value)
{
	char full_key[80];
	snprintf(full_key, sizeof(full_key), "%s.%s", section, key);
	command_print(CMD, "%-21s %3d", full_key, value);
	return 0;
}

COMMAND_HANDLER(handle_info)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	/* This output format can be fed directly into TCL's "array set". */

	riscv_print_info_line(CMD, "hart", "xlen", riscv_xlen(target));
	riscv_enumerate_triggers(target);
	riscv_print_info_line(CMD, "hart", "trigger_count",
						  r->trigger_count);

	if (r->print_info)
		return CALL_COMMAND_HANDLER(r->print_info, target);

	return 0;
}

static const struct command_registration riscv_exec_command_handlers[] = {
	{
		.name = "dump_sample_buf",
		.handler = handle_dump_sample_buf_command,
		.mode = COMMAND_ANY,
		.usage = "[base64]",
		.help = "Print the contents of the sample buffer, and clear the buffer."
	},
	{
		.name = "info",
		.handler = handle_info,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Displays some information OpenOCD detected about the target."
	},
	{
		.name = "memory_sample",
		.handler = handle_memory_sample_command,
		.mode = COMMAND_ANY,
		.usage = "bucket address|clear [size=4]",
		.help = "Causes OpenOCD to frequently read size bytes at the given address."
	},
	{
		.name = "repeat_read",
		.handler = handle_repeat_read,
		.mode = COMMAND_ANY,
		.usage = "count address [size=4]",
		.help = "Repeatedly read the value at address."
	},
	{
		.name = "set_command_timeout_sec",
		.handler = riscv_set_command_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "[sec]",
		.help = "Set the wall-clock timeout (in seconds) for individual commands"
	},
	{
		.name = "set_reset_timeout_sec",
		.handler = riscv_set_reset_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "[sec]",
		.help = "Set the wall-clock timeout (in seconds) after reset is deasserted"
	},
	{
		.name = "set_prefer_sba",
		.handler = riscv_set_prefer_sba,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "When on, prefer to use System Bus Access to access memory. "
			"When off (default), prefer to use the Program Buffer to access memory."
	},
	{
		.name = "set_mem_access",
		.handler = riscv_set_mem_access,
		.mode = COMMAND_ANY,
		.usage = "method1 [method2] [method3]",
		.help = "Set which memory access methods shall be used and in which order "
			"of priority. Method can be one of: 'progbuf', 'sysbus' or 'abstract'."
	},
	{
		.name = "set_enable_virtual",
		.handler = riscv_set_enable_virtual,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "When on, memory accesses are performed on physical or virtual "
				"memory depending on the current system configuration. "
				"When off (default), all memory accessses are performed on physical memory."
	},
	{
		.name = "expose_csrs",
		.handler = riscv_set_expose_csrs,
		.mode = COMMAND_CONFIG,
		.usage = "n0[-m0|=name0][,n1[-m1|=name1]]...",
		.help = "Configure a list of inclusive ranges for CSRs to expose in "
				"addition to the standard ones. This must be executed before "
				"`init`."
	},
	{
		.name = "expose_custom",
		.handler = riscv_set_expose_custom,
		.mode = COMMAND_CONFIG,
		.usage = "n0[-m0|=name0][,n1[-m1|=name1]]...",
		.help = "Configure a list of inclusive ranges for custom registers to "
			"expose. custom0 is accessed as abstract register number 0xc000, "
			"etc. This must be executed before `init`."
	},
	{
		.name = "authdata_read",
		.handler = riscv_authdata_read,
		.usage = "[index]",
		.mode = COMMAND_ANY,
		.help = "Return the 32-bit value read from authdata or authdata0 "
				"(index=0), or authdata1 (index=1)."
	},
	{
		.name = "authdata_write",
		.handler = riscv_authdata_write,
		.mode = COMMAND_ANY,
		.usage = "[index] value",
		.help = "Write the 32-bit value to authdata or authdata0 (index=0), "
				"or authdata1 (index=1)."
	},
	{
		.name = "dmi_read",
		.handler = riscv_dmi_read,
		.mode = COMMAND_ANY,
		.usage = "address",
		.help = "Perform a 32-bit DMI read at address, returning the value."
	},
	{
		.name = "dmi_write",
		.handler = riscv_dmi_write,
		.mode = COMMAND_ANY,
		.usage = "address value",
		.help = "Perform a 32-bit DMI write of value at address."
	},
	{
		.name = "reset_delays",
		.handler = riscv_reset_delays,
		.mode = COMMAND_ANY,
		.usage = "[wait]",
		.help = "OpenOCD learns how many Run-Test/Idle cycles are required "
			"between scans to avoid encountering the target being busy. This "
			"command resets those learned values after `wait` scans. It's only "
			"useful for testing OpenOCD itself."
	},
	{
		.name = "resume_order",
		.handler = riscv_resume_order,
		.mode = COMMAND_ANY,
		.usage = "normal|reversed",
		.help = "Choose the order that harts are resumed in when `hasel` is not "
			"supported. Normal order is from lowest hart index to highest. "
			"Reversed order is from highest hart index to lowest."
	},
	{
		.name = "set_ir",
		.handler = riscv_set_ir,
		.mode = COMMAND_ANY,
		.usage = "[idcode|dtmcs|dmi] value",
		.help = "Set IR value for specified JTAG register."
	},
	{
		.name = "use_bscan_tunnel",
		.handler = riscv_use_bscan_tunnel,
		.mode = COMMAND_ANY,
		.usage = "value [type]",
		.help = "Enable or disable use of a BSCAN tunnel to reach DM.  Supply "
			"the width of the DM transport TAP's instruction register to "
			"enable.  Supply a value of 0 to disable. Pass A second argument "
			"(optional) to indicate Bscan Tunnel Type {0:(default) NESTED_TAP , "
			"1: DATA_REGISTER}"
	},
	{
		.name = "set_bscan_tunnel_ir",
		.handler = riscv_set_bscan_tunnel_ir,
		.mode = COMMAND_ANY,
		.usage = "value",
		.help = "Specify the JTAG TAP IR used to access the bscan tunnel. "
			"By default it is 0x23 << (ir_length - 6), which map some "
			"Xilinx FPGA (IR USER4)"
	},
	{
		.name = "set_maskisr",
		.handler = riscv_set_maskisr,
		.mode = COMMAND_EXEC,
		.help = "mask riscv interrupts",
		.usage = "['off'|'steponly']",
	},
	{
		.name = "set_enable_virt2phys",
		.handler = riscv_set_enable_virt2phys,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "When on (default), enable translation from virtual address to "
			"physical address."
	},
	{
		.name = "set_ebreakm",
		.handler = riscv_set_ebreakm,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "Control dcsr.ebreakm. When off, M-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	{
		.name = "set_ebreaks",
		.handler = riscv_set_ebreaks,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "Control dcsr.ebreaks. When off, S-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	{
		.name = "set_ebreaku",
		.handler = riscv_set_ebreaku,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "Control dcsr.ebreaku. When off, U-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	{
		.name = "etrigger",
		.handler = riscv_etrigger,
		.mode = COMMAND_EXEC,
		.usage = "set [vs] [vu] [m] [s] [u] <exception_codes>|clear",
		.help = "Set or clear a single exception trigger."
	},
	{
		.name = "itrigger",
		.handler = riscv_itrigger,
		.mode = COMMAND_EXEC,
		.usage = "set [vs] [vu] [nmi] [m] [s] [u] <mie_bits>|clear",
		.help = "Set or clear a single interrupt trigger."
	},
	COMMAND_REGISTRATION_DONE
};

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
extern const struct command_registration semihosting_common_handlers[];

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
		.chain = semihosting_common_handlers
	},
	COMMAND_REGISTRATION_DONE
};

static unsigned riscv_xlen_nonconst(struct target *target)
{
	return riscv_xlen(target);
}

static unsigned int riscv_data_bits(struct target *target)
{
	RISCV_INFO(r);
	if (r->data_bits)
		return r->data_bits(target);
	return riscv_xlen(target);
}

struct target_type riscv_target = {
	.name = "riscv",

	.target_create = riscv_create_target,
	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = riscv_examine,

	/* poll current target status */
	.poll = old_or_new_riscv_poll,

	.halt = riscv_halt,
	.resume = riscv_target_resume,
	.step = old_or_new_riscv_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,
	.read_phys_memory = riscv_read_phys_memory,
	.write_phys_memory = riscv_write_phys_memory,

	.checksum_memory = riscv_checksum_memory,

	.mmu = riscv_mmu,
	.virt2phys = riscv_virt2phys,

	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = riscv_get_gdb_reg_list_noread,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = riscv_run_algorithm,

	.commands = riscv_command_handlers,

	.address_bits = riscv_xlen_nonconst,
	.data_bits = riscv_data_bits
};

/*** RISC-V Interface ***/

void riscv_info_init(struct target *target, riscv_info_t *r)
{
	memset(r, 0, sizeof(*r));
	r->dtm_version = 1;
	r->version_specific = NULL;

	memset(r->trigger_unique_id, 0xff, sizeof(r->trigger_unique_id));

	r->xlen = -1;

	r->isrmask_mode = RISCV_ISRMASK_OFF;

	r->mem_access_methods[0] = RISCV_MEM_ACCESS_PROGBUF;
	r->mem_access_methods[1] = RISCV_MEM_ACCESS_SYSBUS;
	r->mem_access_methods[2] = RISCV_MEM_ACCESS_ABSTRACT;

	r->mem_access_progbuf_warn = true;
	r->mem_access_sysbus_warn = true;
	r->mem_access_abstract_warn = true;

	INIT_LIST_HEAD(&r->expose_csr);
	INIT_LIST_HEAD(&r->expose_custom);
}

static int riscv_resume_go_all_harts(struct target *target)
{
	RISCV_INFO(r);

	LOG_TARGET_DEBUG(target, "resuming hart, state=%d", target->state);
	if (target->state == TARGET_HALTED) {
		if (r->resume_go(target) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		LOG_DEBUG("[%s] hart requested resume, but was already resumed",
				target_name(target));
	}

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

int riscv_interrupts_disable(struct target *target, uint64_t irq_mask, uint64_t *old_mstatus)
{
	LOG_DEBUG("Disabling Interrupts");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", true);
	if (!reg_mstatus) {
		LOG_ERROR("Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	int retval = reg_mstatus->type->get(reg_mstatus);
	if (retval != ERROR_OK)
		return retval;

	RISCV_INFO(info);
	uint8_t mstatus_bytes[8] = { 0 };
	uint64_t current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	buf_set_u64(mstatus_bytes, 0, info->xlen, set_field(current_mstatus,
				irq_mask, 0));

	retval = reg_mstatus->type->set(reg_mstatus, mstatus_bytes);
	if (retval != ERROR_OK)
		return retval;

	if (old_mstatus)
		*old_mstatus = current_mstatus;

	return ERROR_OK;
}

int riscv_interrupts_restore(struct target *target, uint64_t old_mstatus)
{
	LOG_DEBUG("Restore Interrupts");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", true);
	if (!reg_mstatus) {
		LOG_ERROR("Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	RISCV_INFO(info);
	uint8_t mstatus_bytes[8];
	buf_set_u64(mstatus_bytes, 0, info->xlen, old_mstatus);
	return reg_mstatus->type->set(reg_mstatus, mstatus_bytes);
}

int riscv_step_rtos_hart(struct target *target)
{
	RISCV_INFO(r);
	LOG_DEBUG("[%s] stepping", target_name(target));

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Hart isn't halted before single step!");
		return ERROR_FAIL;
	}
	r->on_step(target);
	if (r->step_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	r->on_halt(target);
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Hart was not halted after single step!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

bool riscv_supports_extension(struct target *target, char letter)
{
	RISCV_INFO(r);
	unsigned num;
	if (letter >= 'a' && letter <= 'z')
		num = letter - 'a';
	else if (letter >= 'A' && letter <= 'Z')
		num = letter - 'A';
	else
		return false;
	return r->misa & BIT(num);
}

unsigned riscv_xlen(const struct target *target)
{
	RISCV_INFO(r);
	return r->xlen;
}

void riscv_invalidate_register_cache(struct target *target)
{
	/* Do not invalidate the register cache if it is not yet set up
	 * (e.g. when the target failed to get examined). */
	if (!target->reg_cache)
		return;

	LOG_DEBUG("[%d]", target->coreid);
	register_cache_invalidate(target->reg_cache);
	for (size_t i = 0; i < target->reg_cache->num_regs; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];
		reg->valid = false;
	}
}


unsigned int riscv_count_harts(struct target *target)
{
	if (!target)
		return 1;
	RISCV_INFO(r);
	if (!r || !r->hart_count)
		return 1;
	return r->hart_count(target);
}

/**
 * If write is true:
 *   return true iff we are guaranteed that the register will contain exactly
 *       the value we just wrote when it's read.
 * If write is false:
 *   return true iff we are guaranteed that the register will read the same
 *       value in the future as the value we just read.
 */
static bool gdb_regno_cacheable(enum gdb_regno regno, bool write)
{
	/* GPRs, FPRs, vector registers are just normal data stores. */
	if (regno <= GDB_REGNO_XPR31 ||
			(regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31) ||
			(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31))
		return true;

	/* Most CSRs won't change value on us, but we can't assume it about arbitrary
	 * CSRs. */
	switch (regno) {
		case GDB_REGNO_DPC:
			return true;

		case GDB_REGNO_VSTART:
		case GDB_REGNO_VXSAT:
		case GDB_REGNO_VXRM:
		case GDB_REGNO_VLENB:
		case GDB_REGNO_VL:
		case GDB_REGNO_VTYPE:
		case GDB_REGNO_MISA:
		case GDB_REGNO_DCSR:
		case GDB_REGNO_DSCRATCH0:
		case GDB_REGNO_MSTATUS:
		case GDB_REGNO_MEPC:
		case GDB_REGNO_MCAUSE:
		case GDB_REGNO_SATP:
			/*
			 * WARL registers might not contain the value we just wrote, but
			 * these ones won't spontaneously change their value either. *
			 */
			return !write;

		case GDB_REGNO_TSELECT:	/* I think this should be above, but then it doesn't work. */
		case GDB_REGNO_TDATA1:	/* Changes value when tselect is changed. */
		case GDB_REGNO_TDATA2:  /* Changse value when tselect is changed. */
		default:
			return false;
	}
}

/**
 * This function is called when the debug user wants to change the value of a
 * register. The new value may be cached, and may not be written until the hart
 * is resumed. */
int riscv_set_register(struct target *target, enum gdb_regno regid, riscv_reg_t value)
{
	RISCV_INFO(r);
	LOG_DEBUG("[%s] %s <- %" PRIx64, target_name(target), gdb_regno_name(regid), value);
	assert(r->set_register);

	keep_alive();

	/* TODO: Hack to deal with gdb that thinks these registers still exist. */
	if (regid > GDB_REGNO_XPR15 && regid <= GDB_REGNO_XPR31 && value == 0 &&
			riscv_supports_extension(target, 'E'))
		return ERROR_OK;

	struct reg *reg = &target->reg_cache->reg_list[regid];
	buf_set_u64(reg->value, 0, reg->size, value);

	if (gdb_regno_cacheable(regid, true)) {
		reg->valid = true;
		reg->dirty = true;
	} else {
		if (r->set_register(target, regid, value) != ERROR_OK)
			return ERROR_FAIL;
	}

	LOG_DEBUG("[%s] wrote 0x%" PRIx64 " to %s valid=%d",
			  target_name(target), value, reg->name, reg->valid);
	return ERROR_OK;
}

int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno regid)
{
	RISCV_INFO(r);

	keep_alive();

	struct reg *reg = &target->reg_cache->reg_list[regid];
	if (!reg->exist) {
		LOG_DEBUG("[%s] %s does not exist.",
				  target_name(target), gdb_regno_name(regid));
		return ERROR_FAIL;
	}

	if (reg && reg->valid) {
		*value = buf_get_u64(reg->value, 0, reg->size);
		LOG_DEBUG("[%s] %s: %" PRIx64 " (cached)", target_name(target),
				  gdb_regno_name(regid), *value);
		return ERROR_OK;
	}

	/* TODO: Hack to deal with gdb that thinks these registers still exist. */
	if (regid > GDB_REGNO_XPR15 && regid <= GDB_REGNO_XPR31 &&
			riscv_supports_extension(target, 'E')) {
		*value = 0;
		return ERROR_OK;
	}

	int result = r->get_register(target, value, regid);

	if (result == ERROR_OK) {
		/* Update the cache in case we're called from
		 * riscv_save_register(). */
		buf_set_u64(reg->value, 0, reg->size, *value);
		reg->valid = gdb_regno_cacheable(regid, false);
	}

	LOG_DEBUG("[%s] %s: %" PRIx64, target_name(target),
			gdb_regno_name(regid), *value);
	return result;
}

int riscv_save_register(struct target *target, enum gdb_regno regid)
{
	RISCV_INFO(r);
	riscv_reg_t value;
	if (!target->reg_cache) {
		assert(!target_was_examined(target));
		return ERROR_OK;
	}

	struct reg *reg = &target->reg_cache->reg_list[regid];
	LOG_DEBUG("[%s] save %s", target_name(target), reg->name);
	if (riscv_get_register(target, &value, regid) != ERROR_OK)
		return ERROR_FAIL;

	if (!reg->valid)
		return ERROR_FAIL;
	/* Mark the register dirty. We assume that this function is called
	 * because the caller is about to mess with the underlying value of the
	 * register. */
	reg->dirty = true;

	r->last_activity = timeval_ms();

	return ERROR_OK;
}

int riscv_get_hart_state(struct target *target, enum riscv_hart_state *state)
{
	RISCV_INFO(r);
	assert(r->get_hart_state);
	return r->get_hart_state(target, state);
}

enum riscv_halt_reason riscv_halt_reason(struct target *target)
{
	RISCV_INFO(r);
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Hart is not halted!");
		return RISCV_HALT_UNKNOWN;
	}
	return r->halt_reason(target);
}

size_t riscv_debug_buffer_size(struct target *target)
{
	RISCV_INFO(r);
	return r->debug_buffer_size;
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

	riscv_reg_t tselect;
	int result = riscv_get_register(target, &tselect, GDB_REGNO_TSELECT);
	/* If tselect is not readable, the trigger module is likely not
		* implemented. There are no triggers to enumerate then and no error
		* should be thrown. */
	if (result != ERROR_OK) {
		LOG_DEBUG("[%s] Cannot access tselect register. "
				"Assuming that triggers are not implemented.", target_name(target));
		r->trigger_count = 0;
		return ERROR_OK;
	}

	for (unsigned int t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
		r->trigger_count = t;

		/* If we can't write tselect, then this hart does not support triggers. */
		if (riscv_set_register(target, GDB_REGNO_TSELECT, t) != ERROR_OK)
			break;
		uint64_t tselect_rb;
		result = riscv_get_register(target, &tselect_rb, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
		/* Mask off the top bit, which is used as tdrmode in old
			* implementations. */
		tselect_rb &= ~(1ULL << (riscv_xlen(target) - 1));
		if (tselect_rb != t)
			break;

		uint64_t tinfo;
		result = riscv_get_register(target, &tinfo, GDB_REGNO_TINFO);
		if (result == ERROR_OK) {
			/* tinfo == 0 invalid tinfo
			 * tinfo == 1 trigger doesn’t exist */
			if (tinfo == 0 || tinfo == 1)
				break;
			r->trigger_tinfo[t] = tinfo;
		} else {
			uint64_t tdata1;
			result = riscv_get_register(target, &tdata1, GDB_REGNO_TDATA1);
			if (result != ERROR_OK)
				return result;

			int type = get_field(tdata1, CSR_TDATA1_TYPE(riscv_xlen(target)));
			if (type == 0)
				break;
			switch (type) {
				case CSR_TDATA1_TYPE_LEGACY:
					/* On these older cores we don't support software using
						* triggers. */
					riscv_set_register(target, GDB_REGNO_TDATA1, 0);
					break;
				case CSR_TDATA1_TYPE_MCONTROL:
					if (tdata1 & CSR_MCONTROL_DMODE(riscv_xlen(target)))
						riscv_set_register(target, GDB_REGNO_TDATA1, 0);
					break;
				case CSR_TDATA1_TYPE_MCONTROL6:
					if (tdata1 & CSR_MCONTROL6_DMODE(riscv_xlen(target)))
						riscv_set_register(target, GDB_REGNO_TDATA1, 0);
					break;
				case CSR_TDATA1_TYPE_ITRIGGER:
					if (tdata1 & CSR_ITRIGGER_DMODE(riscv_xlen(target)))
						riscv_set_register(target, GDB_REGNO_TDATA1, 0);
					break;
				case CSR_TDATA1_TYPE_ETRIGGER:
					if (tdata1 & CSR_ETRIGGER_DMODE(riscv_xlen(target)))
						riscv_set_register(target, GDB_REGNO_TDATA1, 0);
					break;
			}
			r->trigger_tinfo[t] = 1 << type;
		}
		LOG_TARGET_DEBUG(target, "Trigger %u: supported types (mask) = 0x%08x", t, r->trigger_tinfo[t]);
	}

	riscv_set_register(target, GDB_REGNO_TSELECT, tselect);

	LOG_INFO("[%s] Found %d triggers", target_name(target), r->trigger_count);

	return ERROR_OK;
}

const char *gdb_regno_name(enum gdb_regno regno)
{
	static char buf[32];

	switch (regno) {
		case GDB_REGNO_ZERO:
			return "zero";
		case GDB_REGNO_RA:
			return "ra";
		case GDB_REGNO_SP:
			return "sp";
		case GDB_REGNO_GP:
			return "gp";
		case GDB_REGNO_TP:
			return "tp";
		case GDB_REGNO_T0:
			return "t0";
		case GDB_REGNO_T1:
			return "t1";
		case GDB_REGNO_T2:
			return "t2";
		case GDB_REGNO_S0:
			return "s0";
		case GDB_REGNO_S1:
			return "s1";
		case GDB_REGNO_A0:
			return "a0";
		case GDB_REGNO_A1:
			return "a1";
		case GDB_REGNO_A2:
			return "a2";
		case GDB_REGNO_A3:
			return "a3";
		case GDB_REGNO_A4:
			return "a4";
		case GDB_REGNO_A5:
			return "a5";
		case GDB_REGNO_A6:
			return "a6";
		case GDB_REGNO_A7:
			return "a7";
		case GDB_REGNO_S2:
			return "s2";
		case GDB_REGNO_S3:
			return "s3";
		case GDB_REGNO_S4:
			return "s4";
		case GDB_REGNO_S5:
			return "s5";
		case GDB_REGNO_S6:
			return "s6";
		case GDB_REGNO_S7:
			return "s7";
		case GDB_REGNO_S8:
			return "s8";
		case GDB_REGNO_S9:
			return "s9";
		case GDB_REGNO_S10:
			return "s10";
		case GDB_REGNO_S11:
			return "s11";
		case GDB_REGNO_T3:
			return "t3";
		case GDB_REGNO_T4:
			return "t4";
		case GDB_REGNO_T5:
			return "t5";
		case GDB_REGNO_T6:
			return "t6";
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
		case GDB_REGNO_DSCRATCH0:
			return "dscratch0";
		case GDB_REGNO_MSTATUS:
			return "mstatus";
		case GDB_REGNO_MEPC:
			return "mepc";
		case GDB_REGNO_MCAUSE:
			return "mcause";
		case GDB_REGNO_PRIV:
			return "priv";
		case GDB_REGNO_SATP:
			return "satp";
		case GDB_REGNO_VTYPE:
			return "vtype";
		case GDB_REGNO_VL:
			return "vl";
		case GDB_REGNO_V0:
			return "v0";
		case GDB_REGNO_V1:
			return "v1";
		case GDB_REGNO_V2:
			return "v2";
		case GDB_REGNO_V3:
			return "v3";
		case GDB_REGNO_V4:
			return "v4";
		case GDB_REGNO_V5:
			return "v5";
		case GDB_REGNO_V6:
			return "v6";
		case GDB_REGNO_V7:
			return "v7";
		case GDB_REGNO_V8:
			return "v8";
		case GDB_REGNO_V9:
			return "v9";
		case GDB_REGNO_V10:
			return "v10";
		case GDB_REGNO_V11:
			return "v11";
		case GDB_REGNO_V12:
			return "v12";
		case GDB_REGNO_V13:
			return "v13";
		case GDB_REGNO_V14:
			return "v14";
		case GDB_REGNO_V15:
			return "v15";
		case GDB_REGNO_V16:
			return "v16";
		case GDB_REGNO_V17:
			return "v17";
		case GDB_REGNO_V18:
			return "v18";
		case GDB_REGNO_V19:
			return "v19";
		case GDB_REGNO_V20:
			return "v20";
		case GDB_REGNO_V21:
			return "v21";
		case GDB_REGNO_V22:
			return "v22";
		case GDB_REGNO_V23:
			return "v23";
		case GDB_REGNO_V24:
			return "v24";
		case GDB_REGNO_V25:
			return "v25";
		case GDB_REGNO_V26:
			return "v26";
		case GDB_REGNO_V27:
			return "v27";
		case GDB_REGNO_V28:
			return "v28";
		case GDB_REGNO_V29:
			return "v29";
		case GDB_REGNO_V30:
			return "v30";
		case GDB_REGNO_V31:
			return "v31";
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
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	RISCV_INFO(r);

	if (reg->number >= GDB_REGNO_V0 && reg->number <= GDB_REGNO_V31) {
		if (!r->get_register_buf) {
			LOG_ERROR("Reading register %s not supported on this RISC-V target.",
					gdb_regno_name(reg->number));
			return ERROR_FAIL;
		}

		if (r->get_register_buf(target, reg->value, reg->number) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		uint64_t value;
		int result = riscv_get_register(target, &value, reg->number);
		if (result != ERROR_OK)
			return result;
		buf_set_u64(reg->value, 0, reg->size, value);
	}
	reg->valid = gdb_regno_cacheable(reg->number, false);
	char *str = buf_to_hex_str(reg->value, reg->size);
	LOG_DEBUG("[%s] read 0x%s from %s (valid=%d)", target_name(target),
			str, reg->name, reg->valid);
	free(str);
	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	RISCV_INFO(r);

	char *str = buf_to_hex_str(buf, reg->size);
	LOG_DEBUG("[%s] write 0x%s to %s (valid=%d)", target_name(target),
			str, reg->name, reg->valid);
	free(str);

	/* Exit early for writing x0, which on the hardware would be ignored, and we
	 * don't want to update our cache. */
	if (reg->number == GDB_REGNO_ZERO)
		return ERROR_OK;

	memcpy(reg->value, buf, DIV_ROUND_UP(reg->size, 8));
	reg->valid = gdb_regno_cacheable(reg->number, true);

	if (reg->number == GDB_REGNO_TDATA1 ||
			reg->number == GDB_REGNO_TDATA2) {
		r->manual_hwbp_set = true;
		/* When enumerating triggers, we clear any triggers with DMODE set,
		 * assuming they were left over from a previous debug session. So make
		 * sure that is done before a user might be setting their own triggers.
		 */
		if (riscv_enumerate_triggers(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (reg->number >= GDB_REGNO_V0 && reg->number <= GDB_REGNO_V31) {
		if (!r->set_register_buf) {
			LOG_ERROR("Writing register %s not supported on this RISC-V target.",
					gdb_regno_name(reg->number));
			return ERROR_FAIL;
		}

		if (r->set_register_buf(target, reg->number, reg->value) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		uint64_t value = buf_get_u64(buf, 0, reg->size);
		if (riscv_set_register(target, reg->number, value) != ERROR_OK)
			return ERROR_FAIL;
	}

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

	riscv_free_registers(target);

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	if (!target->reg_cache)
		return ERROR_FAIL;
	target->reg_cache->name = "RISC-V Registers";
	target->reg_cache->num_regs = GDB_REGNO_COUNT;

	if (!list_empty(&info->expose_custom)) {
		range_list_t *entry;
		list_for_each_entry(entry, &info->expose_custom, list)
			target->reg_cache->num_regs += entry->high - entry->low + 1;
	}

	LOG_DEBUG("[%s] create register cache for %d registers",
			target_name(target), target->reg_cache->num_regs);

	target->reg_cache->reg_list =
		calloc(target->reg_cache->num_regs, sizeof(struct reg));
	if (!target->reg_cache->reg_list)
		return ERROR_FAIL;

	const unsigned int max_reg_name_len = 12;
	free(info->reg_names);
	info->reg_names =
		calloc(target->reg_cache->num_regs, max_reg_name_len);
	if (!info->reg_names)
		return ERROR_FAIL;
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
	static struct reg_feature feature_vector = {
		.name = "org.gnu.gdb.riscv.vector"
	};
	static struct reg_feature feature_virtual = {
		.name = "org.gnu.gdb.riscv.virtual"
	};
	static struct reg_feature feature_custom = {
		.name = "org.gnu.gdb.riscv.custom"
	};

	/* These types are built into gdb. */
	static struct reg_data_type type_ieee_single = { .type = REG_TYPE_IEEE_SINGLE, .id = "ieee_single" };
	static struct reg_data_type type_ieee_double = { .type = REG_TYPE_IEEE_DOUBLE, .id = "ieee_double" };
	static struct reg_data_type_union_field single_double_fields[] = {
		{"float", &type_ieee_single, single_double_fields + 1},
		{"double", &type_ieee_double, NULL},
	};
	static struct reg_data_type_union single_double_union = {
		.fields = single_double_fields
	};
	static struct reg_data_type type_ieee_single_double = {
		.type = REG_TYPE_ARCH_DEFINED,
		.id = "FPU_FD",
		.type_class = REG_TYPE_CLASS_UNION,
		.reg_type_union = &single_double_union
	};
	static struct reg_data_type type_uint8 = { .type = REG_TYPE_UINT8, .id = "uint8" };
	static struct reg_data_type type_uint16 = { .type = REG_TYPE_UINT16, .id = "uint16" };
	static struct reg_data_type type_uint32 = { .type = REG_TYPE_UINT32, .id = "uint32" };
	static struct reg_data_type type_uint64 = { .type = REG_TYPE_UINT64, .id = "uint64" };
	static struct reg_data_type type_uint128 = { .type = REG_TYPE_UINT128, .id = "uint128" };

	/* This is roughly the XML we want:
	 * <vector id="bytes" type="uint8" count="16"/>
	 * <vector id="shorts" type="uint16" count="8"/>
	 * <vector id="words" type="uint32" count="4"/>
	 * <vector id="longs" type="uint64" count="2"/>
	 * <vector id="quads" type="uint128" count="1"/>
	 * <union id="riscv_vector_type">
	 *   <field name="b" type="bytes"/>
	 *   <field name="s" type="shorts"/>
	 *   <field name="w" type="words"/>
	 *   <field name="l" type="longs"/>
	 *   <field name="q" type="quads"/>
	 * </union>
	 */

	info->vector_uint8.type = &type_uint8;
	info->vector_uint8.count = info->vlenb;
	info->type_uint8_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint8_vector.id = "bytes";
	info->type_uint8_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint8_vector.reg_type_vector = &info->vector_uint8;

	info->vector_uint16.type = &type_uint16;
	info->vector_uint16.count = info->vlenb / 2;
	info->type_uint16_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint16_vector.id = "shorts";
	info->type_uint16_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint16_vector.reg_type_vector = &info->vector_uint16;

	info->vector_uint32.type = &type_uint32;
	info->vector_uint32.count = info->vlenb / 4;
	info->type_uint32_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint32_vector.id = "words";
	info->type_uint32_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint32_vector.reg_type_vector = &info->vector_uint32;

	info->vector_uint64.type = &type_uint64;
	info->vector_uint64.count = info->vlenb / 8;
	info->type_uint64_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint64_vector.id = "longs";
	info->type_uint64_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint64_vector.reg_type_vector = &info->vector_uint64;

	info->vector_uint128.type = &type_uint128;
	info->vector_uint128.count = info->vlenb / 16;
	info->type_uint128_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint128_vector.id = "quads";
	info->type_uint128_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint128_vector.reg_type_vector = &info->vector_uint128;

	info->vector_fields[0].name = "b";
	info->vector_fields[0].type = &info->type_uint8_vector;
	if (info->vlenb >= 2) {
		info->vector_fields[0].next = info->vector_fields + 1;
		info->vector_fields[1].name = "s";
		info->vector_fields[1].type = &info->type_uint16_vector;
	} else {
		info->vector_fields[0].next = NULL;
	}
	if (info->vlenb >= 4) {
		info->vector_fields[1].next = info->vector_fields + 2;
		info->vector_fields[2].name = "w";
		info->vector_fields[2].type = &info->type_uint32_vector;
	} else {
		info->vector_fields[1].next = NULL;
	}
	if (info->vlenb >= 8) {
		info->vector_fields[2].next = info->vector_fields + 3;
		info->vector_fields[3].name = "l";
		info->vector_fields[3].type = &info->type_uint64_vector;
	} else {
		info->vector_fields[2].next = NULL;
	}
	if (info->vlenb >= 16) {
		info->vector_fields[3].next = info->vector_fields + 4;
		info->vector_fields[4].name = "q";
		info->vector_fields[4].type = &info->type_uint128_vector;
	} else {
		info->vector_fields[3].next = NULL;
	}
	info->vector_fields[4].next = NULL;

	info->vector_union.fields = info->vector_fields;

	info->type_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_vector.id = "riscv_vector";
	info->type_vector.type_class = REG_TYPE_CLASS_UNION;
	info->type_vector.reg_type_union = &info->vector_union;

	struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "encoding.h"
#undef DECLARE_CSR
	};
	/* encoding.h does not contain the registers in sorted order. */
	qsort(csr_info, ARRAY_SIZE(csr_info), sizeof(*csr_info), cmp_csr_info);
	unsigned csr_info_index = 0;

	int custom_within_range = 0;

	riscv_reg_info_t *shared_reg_info = calloc(1, sizeof(riscv_reg_info_t));
	if (!shared_reg_info)
		return ERROR_FAIL;
	shared_reg_info->target = target;

	/* When gdb requests register N, gdb_get_register_packet() assumes that this
	 * is register at index N in reg_list. So if there are certain registers
	 * that don't exist, we need to leave holes in the list (or renumber, but
	 * it would be nice not to have yet another set of numbers to translate
	 * between). */
	for (uint32_t number = 0; number < target->reg_cache->num_regs; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];
		r->dirty = false;
		r->valid = false;
		r->exist = true;
		r->type = &riscv_reg_arch_type;
		r->arch_info = shared_reg_info;
		r->number = number;
		r->size = riscv_xlen(target);
		/* r->size is set in riscv_invalidate_register_cache, maybe because the
		 * target is in theory allowed to change XLEN on us. But I expect a lot
		 * of other things to break in that case as well. */
		if (number <= GDB_REGNO_XPR31) {
			r->exist = number <= GDB_REGNO_XPR15 ||
				!riscv_supports_extension(target, 'E');
			/* TODO: For now we fake that all GPRs exist because otherwise gdb
			 * doesn't work. */
			r->exist = true;
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
			if (riscv_supports_extension(target, 'D')) {
				r->size = 64;
				if (riscv_supports_extension(target, 'F'))
					r->reg_data_type = &type_ieee_single_double;
				else
					r->reg_data_type = &type_ieee_double;
			} else if (riscv_supports_extension(target, 'F')) {
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
					csr_info_index < ARRAY_SIZE(csr_info) - 1) {
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
					r->exist = riscv_supports_extension(target, 'F');
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
					r->exist = riscv_supports_extension(target, 'S');
					break;
				case CSR_MEDELEG:
				case CSR_MIDELEG:
					/* "In systems with only M-mode, or with both M-mode and
					 * U-mode but without U-mode trap support, the medeleg and
					 * mideleg registers should not exist." */
					r->exist = riscv_supports_extension(target, 'S') ||
						riscv_supports_extension(target, 'N');
					break;

				case CSR_PMPCFG1:
				case CSR_PMPCFG3:
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

				case CSR_VSTART:
				case CSR_VXSAT:
				case CSR_VXRM:
				case CSR_VL:
				case CSR_VCSR:
				case CSR_VTYPE:
				case CSR_VLENB:
					r->exist = (info->vlenb > 0);
					break;
			}

			if (!r->exist && !list_empty(&info->expose_csr)) {
				range_list_t *entry;
				list_for_each_entry(entry, &info->expose_csr, list)
					if ((entry->low <= csr_number) && (csr_number <= entry->high)) {
						if (entry->name) {
							*reg_name = 0;
							r->name = entry->name;
						}

						LOG_DEBUG("Exposing additional CSR %d (name=%s)",
								csr_number, entry->name ? entry->name : reg_name);

						r->exist = true;
						break;
					}
			}

		} else if (number == GDB_REGNO_PRIV) {
			sprintf(reg_name, "priv");
			r->group = "general";
			r->feature = &feature_virtual;
			r->size = 8;

		} else if (number >= GDB_REGNO_V0 && number <= GDB_REGNO_V31) {
			r->caller_save = false;
			r->exist = (info->vlenb > 0);
			r->size = info->vlenb * 8;
			sprintf(reg_name, "v%d", number - GDB_REGNO_V0);
			r->group = "vector";
			r->feature = &feature_vector;
			r->reg_data_type = &info->type_vector;

		} else if (number >= GDB_REGNO_COUNT) {
			/* Custom registers. */
			assert(!list_empty(&info->expose_custom));

			range_list_t *range = list_first_entry(&info->expose_custom, range_list_t, list);

			unsigned custom_number = range->low + custom_within_range;

			r->group = "custom";
			r->feature = &feature_custom;
			r->arch_info = calloc(1, sizeof(riscv_reg_info_t));
			if (!r->arch_info)
				return ERROR_FAIL;
			((riscv_reg_info_t *) r->arch_info)->target = target;
			((riscv_reg_info_t *) r->arch_info)->custom_number = custom_number;
			sprintf(reg_name, "custom%d", custom_number);

			if (range->name) {
				*reg_name = 0;
				r->name = range->name;
			}

			LOG_DEBUG("Exposing additional custom register %d (name=%s)",
					number, range->name ? range->name : reg_name);

			custom_within_range++;
			if (custom_within_range > range->high - range->low) {
				custom_within_range = 0;
				list_rotate_left(&info->expose_custom);
			}
		}

		if (reg_name[0]) {
			r->name = reg_name;
			reg_name += strlen(reg_name) + 1;
			assert(reg_name < info->reg_names + target->reg_cache->num_regs *
					max_reg_name_len);
		}
		r->value = calloc(1, DIV_ROUND_UP(r->size, 8));
	}

	return ERROR_OK;
}


void riscv_add_bscan_tunneled_scan(struct target *target, struct scan_field *field,
					riscv_bscan_tunneled_scan_context_t *ctxt)
{
	jtag_add_ir_scan(target->tap, &select_user4, TAP_IDLE);

	memset(ctxt->tunneled_dr, 0, sizeof(ctxt->tunneled_dr));
	if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER) {
		ctxt->tunneled_dr[3].num_bits = 1;
		ctxt->tunneled_dr[3].out_value = bscan_one;
		ctxt->tunneled_dr[2].num_bits = 7;
		ctxt->tunneled_dr_width = field->num_bits;
		ctxt->tunneled_dr[2].out_value = &ctxt->tunneled_dr_width;
		/* for BSCAN tunnel, there is a one-TCK skew between shift in and shift out, so
		   scanning num_bits + 1, and then will right shift the input field after executing the queues */

		ctxt->tunneled_dr[1].num_bits = field->num_bits + 1;
		ctxt->tunneled_dr[1].out_value = field->out_value;
		ctxt->tunneled_dr[1].in_value = field->in_value;

		ctxt->tunneled_dr[0].num_bits = 3;
		ctxt->tunneled_dr[0].out_value = bscan_zero;
	} else {
		/* BSCAN_TUNNEL_NESTED_TAP */
		ctxt->tunneled_dr[0].num_bits = 1;
		ctxt->tunneled_dr[0].out_value = bscan_one;
		ctxt->tunneled_dr[1].num_bits = 7;
		ctxt->tunneled_dr_width = field->num_bits;
		ctxt->tunneled_dr[1].out_value = &ctxt->tunneled_dr_width;
		/* for BSCAN tunnel, there is a one-TCK skew between shift in and shift out, so
		   scanning num_bits + 1, and then will right shift the input field after executing the queues */
		ctxt->tunneled_dr[2].num_bits = field->num_bits + 1;
		ctxt->tunneled_dr[2].out_value = field->out_value;
		ctxt->tunneled_dr[2].in_value = field->in_value;
		ctxt->tunneled_dr[3].num_bits = 3;
		ctxt->tunneled_dr[3].out_value = bscan_zero;
	}
	jtag_add_dr_scan(target->tap, ARRAY_SIZE(ctxt->tunneled_dr), ctxt->tunneled_dr, TAP_IDLE);
}

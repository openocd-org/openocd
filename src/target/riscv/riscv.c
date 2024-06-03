// SPDX-License-Identifier: GPL-2.0-or-later

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
#include "riscv_reg.h"
#include "program.h"
#include "gdb_regs.h"
#include "rtos/rtos.h"
#include "debug_defines.h"
#include <helper/bits.h>
#include "field_helpers.h"

/*** JTAG registers. ***/

#define DTMCONTROL					0x10
#define DTMCONTROL_VERSION			(0xf)

#define DBUS						0x11

#define RISCV_TRIGGER_HIT_NOT_FOUND ((int64_t)-1)

static uint8_t ir_dtmcontrol[4] = {DTMCONTROL};
struct scan_field select_dtmcontrol = {
	.in_value = NULL,
	.out_value = ir_dtmcontrol
};
static uint8_t ir_dbus[4] = {DBUS};
struct scan_field select_dbus = {
	.in_value = NULL,
	.out_value = ir_dbus
};
static uint8_t ir_idcode[4] = {0x1};
struct scan_field select_idcode = {
	.in_value = NULL,
	.out_value = ir_idcode
};

static bscan_tunnel_type_t bscan_tunnel_type;
int bscan_tunnel_ir_width; /* if zero, then tunneling is not present/active */
static int bscan_tunnel_ir_id; /* IR ID of the JTAG TAP to access the tunnel. Valid when not 0 */

static const uint8_t bscan_zero[4] = {0};
static const uint8_t bscan_one[4] = {1};

static uint8_t ir_user4[4];
static struct scan_field select_user4 = {
	.in_value = NULL,
	.out_value = ir_user4
};


static uint8_t bscan_tunneled_ir_width[4] = {5};  /* overridden by assignment in riscv_init_target */
static struct scan_field _bscan_tunnel_data_register_select_dmi[] = {
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

static struct scan_field _bscan_tunnel_nested_tap_select_dmi[] = {
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
static struct scan_field *bscan_tunnel_nested_tap_select_dmi = _bscan_tunnel_nested_tap_select_dmi;
static uint32_t bscan_tunnel_nested_tap_select_dmi_num_fields = ARRAY_SIZE(_bscan_tunnel_nested_tap_select_dmi);

static struct scan_field *bscan_tunnel_data_register_select_dmi = _bscan_tunnel_data_register_select_dmi;
static uint32_t bscan_tunnel_data_register_select_dmi_num_fields = ARRAY_SIZE(_bscan_tunnel_data_register_select_dmi);

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool is_read, is_write, is_execute;
	int unique_id;
};

struct tdata2_cache {
	struct list_head elem_tdata2;
	riscv_reg_t tdata2;
};

struct tdata1_cache {
	riscv_reg_t tdata1;
	struct list_head tdata2_cache_head;
	struct list_head elem_tdata1;
};

/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
static int riscv_command_timeout_sec_value = DEFAULT_COMMAND_TIMEOUT_SEC;

/* DEPRECATED Wall-clock timeout after reset. Settable via RISC-V Target commands.*/
static int riscv_reset_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;

int riscv_get_command_timeout_sec(void)
{
	return MAX(riscv_command_timeout_sec_value, riscv_reset_timeout_sec);
}

static bool riscv_enable_virt2phys = true;

bool riscv_enable_virtual;

static enum {
	RO_NORMAL,
	RO_REVERSED
} resume_order;

static const virt2phys_info_t sv32 = {
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

static const virt2phys_info_t sv32x4 = {
	.name = "Sv32x4",
	.va_bits = 34,
	.level = 2,
	.pte_shift = 2,
	.vpn_shift = {12, 22},
	.vpn_mask = {0x3ff, 0xfff},
	.pte_ppn_shift = {10, 20},
	.pte_ppn_mask = {0x3ff, 0xfff},
	.pa_ppn_shift = {12, 22},
	.pa_ppn_mask = {0x3ff, 0xfff},
};

static const virt2phys_info_t sv39 = {
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

static const virt2phys_info_t sv39x4 = {
	.name = "Sv39x4",
	.va_bits = 41,
	.level = 3,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30},
	.vpn_mask = {0x1ff, 0x1ff, 0x7ff},
	.pte_ppn_shift = {10, 19, 28},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x3ffffff},
	.pa_ppn_shift = {12, 21, 30},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x3ffffff},
};

static const virt2phys_info_t sv48 = {
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

static const virt2phys_info_t sv48x4 = {
	.name = "Sv48x4",
	.va_bits = 50,
	.level = 4,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30, 39},
	.vpn_mask = {0x1ff, 0x1ff, 0x1ff, 0x7ff},
	.pte_ppn_shift = {10, 19, 28, 37},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ffff},
	.pa_ppn_shift = {12, 21, 30, 39},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ffff},
};

static const virt2phys_info_t sv57 = {
	.name = "Sv57",
	.va_bits = 57,
	.level = 5,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30, 39, 48},
	.vpn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff, 0x1ff},
	.pte_ppn_shift = {10, 19, 28, 37, 46},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff, 0xff},
	.pa_ppn_shift = {12, 21, 30, 39, 48},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff, 0xff},
};

static const virt2phys_info_t sv57x4 = {
	.name = "Sv57x4",
	.va_bits = 59,
	.level = 5,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30, 39, 48},
	.vpn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff, 0x7ff},
	.pte_ppn_shift = {10, 19, 28, 37, 46},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff, 0xff},
	.pa_ppn_shift = {12, 21, 30, 39, 48},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff, 0xff},
};

static enum riscv_halt_reason riscv_halt_reason(struct target *target);
static void riscv_info_init(struct target *target, struct riscv_info *r);
static void riscv_invalidate_register_cache(struct target *target);
static int riscv_step_rtos_hart(struct target *target);

static void riscv_sample_buf_maybe_add_timestamp(struct target *target, bool before)
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

int dtmcontrol_scan_via_bscan(struct target *target, uint32_t out, uint32_t *in_ptr)
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

	if (in_ptr)
		*in_ptr = in;
	return ERROR_OK;
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

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "dtmcontrol scan failed, error code = %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCONTROL: 0x%x -> 0x%x", out, in);

	if (in_ptr)
		*in_ptr = in;
	return ERROR_OK;
}

static struct target_type *get_target_type(struct target *target)
{
	if (!target->arch_info) {
		LOG_TARGET_ERROR(target, "Target has not been initialized.");
		return NULL;
	}

	RISCV_INFO(info);
	switch (info->dtm_version) {
		case DTM_DTMCS_VERSION_0_11:
			return &riscv011_target;
		case DTM_DTMCS_VERSION_1_0:
			return &riscv013_target;
		default:
			/* TODO: once we have proper support for non-examined targets
			 * we should have an assert here */
			LOG_TARGET_ERROR(target, "Unsupported DTM version: %d",
					info->dtm_version);
			return NULL;
	}
}

static int riscv_create_target(struct target *target, Jim_Interp *interp)
{
	LOG_TARGET_DEBUG(target, "riscv_create_target()");
	target->arch_info = calloc(1, sizeof(struct riscv_info));
	if (!target->arch_info) {
		LOG_TARGET_ERROR(target, "Failed to allocate RISC-V target structure.");
		return ERROR_FAIL;
	}
	riscv_info_init(target, target->arch_info);
	return ERROR_OK;
}

static int riscv_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_TARGET_DEBUG(target, "riscv_init_target()");
	RISCV_INFO(info);
	info->cmd_ctx = cmd_ctx;
	info->reset_delays_wait = -1;

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
		h_u32_to_le(ir_user4, ir_user4_raw);
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

static void free_wp_triggers_cache(struct target *target)
{
	RISCV_INFO(r);

	for (unsigned int i = 0; i < r->trigger_count; ++i) {
		struct tdata1_cache *elem_1, *tmp_1;
		list_for_each_entry_safe(elem_1, tmp_1, &r->wp_triggers_negative_cache[i], elem_tdata1) {
			struct tdata2_cache *elem_2, *tmp_2;
			list_for_each_entry_safe(elem_2, tmp_2, &elem_1->tdata2_cache_head, elem_tdata2) {
				list_del(&elem_2->elem_tdata2);
				free(elem_2);
			}
			list_del(&elem_1->elem_tdata1);
			free(elem_1);
		}
	}
	free(r->wp_triggers_negative_cache);
}

static void riscv_deinit_target(struct target *target)
{
	LOG_TARGET_DEBUG(target, "riscv_deinit_target()");

	struct riscv_info *info = target->arch_info;
	struct target_type *tt = get_target_type(target);
	if (!tt)
		LOG_TARGET_ERROR(target, "Could not identify target type.");

	if (riscv_reg_flush_all(target) != ERROR_OK)
		LOG_TARGET_ERROR(target, "Failed to flush registers. Ignoring this error.");

	if (tt && info && info->version_specific)
		tt->deinit_target(target);

	riscv_reg_free_all(target);
	free_wp_triggers_cache(target);

	if (!info)
		return;

	range_list_t *entry, *tmp;
	list_for_each_entry_safe(entry, tmp, &info->hide_csr, list) {
		free(entry->name);
		free(entry);
	}

	list_for_each_entry_safe(entry, tmp, &info->expose_csr, list) {
		free(entry->name);
		free(entry);
	}

	list_for_each_entry_safe(entry, tmp, &info->expose_custom, list) {
		free(entry->name);
		free(entry);
	}

	free(target->arch_info);

	target->arch_info = NULL;
}

static void trigger_from_breakpoint(struct trigger *trigger,
		const struct breakpoint *breakpoint)
{
	trigger->address = breakpoint->address;
	trigger->length = breakpoint->length;
	trigger->mask = ~0LL;
	trigger->is_read = false;
	trigger->is_write = false;
	trigger->is_execute = true;
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = breakpoint->unique_id;
}

static bool can_use_napot_match(struct trigger *trigger)
{
	riscv_reg_t addr = trigger->address;
	riscv_reg_t size = trigger->length;
	bool size_power_of_2 = (size & (size - 1)) == 0;
	bool addr_aligned = (addr & (size - 1)) == 0;
	return size > 1 && size_power_of_2 && addr_aligned;
}

/* Find the next free trigger of the given type, without talking to the target. */
static int find_next_free_trigger(struct target *target, int type, bool chained,
		unsigned int *idx)
{
	assert(idx);
	RISCV_INFO(r);

	unsigned int num_found = 0;
	unsigned int num_required = chained ? 2 : 1;

	for (unsigned int i = *idx; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == -1) {
			if (r->trigger_tinfo[i] & (1 << type)) {
				num_found++;
				if (num_required == num_found) {
					/* Found num_required consecutive free triggers - success, done. */
					*idx = i - (num_required - 1);
					LOG_TARGET_DEBUG(target,
							"%d trigger(s) of type %d found on index %u, "
							"chained == %s",
							num_required, type, *idx,
							chained ? "true" : "false");
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

	for (unsigned int i = 0; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == unique_id)
			return i;
	}
	return -1;
}

static int set_trigger(struct target *target, unsigned int idx, riscv_reg_t tdata1, riscv_reg_t tdata2,
	riscv_reg_t tdata1_ignore_mask)
{
	riscv_reg_t tdata1_rb, tdata2_rb;
	// Select which trigger to use
	if (riscv_reg_set(target, GDB_REGNO_TSELECT, idx) != ERROR_OK)
		return ERROR_FAIL;

	// Disable the trigger by writing 0 to it
	if (riscv_reg_set(target, GDB_REGNO_TDATA1, 0) != ERROR_OK)
		return ERROR_FAIL;

	// Set trigger data for tdata2 (and tdata3 if it was supported)
	if (riscv_reg_set(target, GDB_REGNO_TDATA2, tdata2) != ERROR_OK)
		return ERROR_FAIL;

	// Set trigger data for tdata1
	if (riscv_reg_set(target, GDB_REGNO_TDATA1, tdata1) != ERROR_OK)
		return ERROR_FAIL;

	// Read back tdata1, tdata2, (tdata3), and check if the configuration is supported
	if (riscv_reg_get(target, &tdata1_rb, GDB_REGNO_TDATA1) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_reg_get(target, &tdata2_rb, GDB_REGNO_TDATA2) != ERROR_OK)
		return ERROR_FAIL;
	bool tdata1_config_denied = (tdata1 & ~tdata1_ignore_mask) != (tdata1_rb & ~tdata1_ignore_mask);
	bool tdata2_config_denied = tdata2 != tdata2_rb;
	if (tdata1_config_denied || tdata2_config_denied) {
		LOG_TARGET_DEBUG(target, "Trigger %u doesn't support what we need.", idx);

		if (tdata1_config_denied)
			LOG_TARGET_DEBUG(target,
				"After writing 0x%" PRIx64 " to tdata1 it contains 0x%" PRIx64 "; tdata1_ignore_mask=0x%" PRIx64,
				tdata1, tdata1_rb, tdata1_ignore_mask);

		if (tdata2_config_denied)
			LOG_TARGET_DEBUG(target,
				"wrote 0x%" PRIx64 " to tdata2 but read back 0x%" PRIx64,
				tdata2, tdata2_rb);
		if (riscv_reg_set(target, GDB_REGNO_TDATA1, 0) != ERROR_OK)
			return ERROR_FAIL;
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

	unsigned int idx = 0;
	ret = find_next_free_trigger(target, CSR_TDATA1_TYPE_LEGACY, false, &idx);
	if (ret != ERROR_OK)
		return ret;

	if (riscv_reg_get(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
		return ERROR_FAIL;
	if (tdata1 & (bpcontrol_r | bpcontrol_w | bpcontrol_x)) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	tdata1 = 0;
	tdata1 = set_field(tdata1, bpcontrol_r, trigger->is_read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->is_write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->is_execute);
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

struct trigger_request_info {
	riscv_reg_t tdata1;
	riscv_reg_t tdata2;
	riscv_reg_t tdata1_ignore_mask;
};

static void log_trigger_request_info(struct trigger_request_info trig_info)
{
	LOG_DEBUG("tdata1=%" PRIx64 ", tdata2=%" PRIx64 ", tdata1_ignore_mask=%" PRIx64,
			trig_info.tdata1, trig_info.tdata2, trig_info.tdata1_ignore_mask);
};

static struct tdata1_cache *tdata1_cache_alloc(struct list_head *tdata1_cache_head, riscv_reg_t tdata1)
{
	struct tdata1_cache *elem = (struct tdata1_cache *)calloc(1, sizeof(struct tdata1_cache));
	elem->tdata1 = tdata1;
	INIT_LIST_HEAD(&elem->tdata2_cache_head);
	list_add_tail(&elem->elem_tdata1, tdata1_cache_head);
	return elem;
}

static void tdata2_cache_alloc(struct list_head *tdata2_cache_head, riscv_reg_t tdata2)
{
	struct tdata2_cache * const elem = calloc(1, sizeof(struct tdata2_cache));
	elem->tdata2 = tdata2;
	list_add(&elem->elem_tdata2, tdata2_cache_head);
}

struct tdata2_cache *tdata2_cache_search(struct list_head *tdata2_cache_head, riscv_reg_t find_tdata2)
{
	struct tdata2_cache *elem_2;
	list_for_each_entry(elem_2, tdata2_cache_head, elem_tdata2) {
		if (elem_2->tdata2 == find_tdata2)
			return elem_2;
	}
	return NULL;
}

struct tdata1_cache *tdata1_cache_search(struct list_head *tdata1_cache_head, riscv_reg_t find_tdata1)
{
	struct tdata1_cache *elem_1;
	list_for_each_entry(elem_1, tdata1_cache_head, elem_tdata1) {
		if (elem_1->tdata1 == find_tdata1)
			return elem_1;
	}
	return NULL;
}

static void create_wp_trigger_cache(struct target *target)
{
	RISCV_INFO(r);

	r->wp_triggers_negative_cache = (struct list_head *)calloc(r->trigger_count,
		sizeof(struct list_head));
	for (unsigned int i = 0; i < r->trigger_count; ++i)
		INIT_LIST_HEAD(&r->wp_triggers_negative_cache[i]);
}

static void wp_triggers_cache_add(struct target *target, unsigned int idx, riscv_reg_t tdata1,
	riscv_reg_t tdata2, int error_code)
{
	RISCV_INFO(r);

	struct tdata1_cache *tdata1_cache = tdata1_cache_search(&r->wp_triggers_negative_cache[idx], tdata1);
	if (!tdata1_cache) {
		tdata1_cache = tdata1_cache_alloc(&r->wp_triggers_negative_cache[idx], tdata1);
	} else {
		struct tdata2_cache *tdata2_cache = tdata2_cache_search(&tdata1_cache->tdata2_cache_head, tdata2);
		if (tdata2_cache) {
			list_move(&tdata2_cache->elem_tdata2, &tdata1_cache->tdata2_cache_head);
			return;
		}
	}
	tdata2_cache_alloc(&tdata1_cache->tdata2_cache_head, tdata2);
}

static bool wp_triggers_cache_search(struct target *target, unsigned int idx,
	riscv_reg_t tdata1, riscv_reg_t tdata2)
{
	RISCV_INFO(r);

	struct tdata1_cache *tdata1_cache = tdata1_cache_search(&r->wp_triggers_negative_cache[idx], tdata1);
	if (!tdata1_cache)
		return false;
	struct tdata2_cache *tdata2_cache = tdata2_cache_search(&tdata1_cache->tdata2_cache_head, tdata2);
	if (!tdata2_cache)
		return false;
	assert(tdata1_cache->tdata1 == tdata1 && tdata2_cache->tdata2 == tdata2);
	return true;
}

static int try_use_trigger_and_cache_result(struct target *target, unsigned int idx, riscv_reg_t tdata1,
	riscv_reg_t tdata2, riscv_reg_t tdata1_ignore_mask)
{
	if (wp_triggers_cache_search(target, idx, tdata1, tdata2))
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	int ret = set_trigger(target, idx, tdata1, tdata2, tdata1_ignore_mask);

	/* Add these values to the cache to remember that they are not supported. */
	if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		wp_triggers_cache_add(target, idx, tdata1, tdata2, ret);
	return ret;
}

static int try_setup_single_match_trigger(struct target *target,
		struct trigger *trigger, struct trigger_request_info trig_info)
{
	LOG_TARGET_DEBUG(target, "trying to set up a match trigger");
	log_trigger_request_info(trig_info);

	int trigger_type =
		get_field(trig_info.tdata1, CSR_MCONTROL_TYPE(riscv_xlen(target)));
	int ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	RISCV_INFO(r);

	/* Find the first trigger, supporting required tdata1 value */
	for (unsigned int idx = 0;
			find_next_free_trigger(target, trigger_type, false, &idx) == ERROR_OK;
			++idx) {
		ret = try_use_trigger_and_cache_result(target, idx, trig_info.tdata1, trig_info.tdata2,
			trig_info.tdata1_ignore_mask);

		if (ret == ERROR_OK) {
			r->trigger_unique_id[idx] = trigger->unique_id;
			return ERROR_OK;
		}
		if (ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			return ret;
	}
	return ret;
}

static int try_setup_chained_match_triggers(struct target *target,
		struct trigger *trigger, struct trigger_request_info t1,
		struct trigger_request_info t2)
{
	LOG_TARGET_DEBUG(target, "trying to set up a chain of match triggers");
	log_trigger_request_info(t1);
	log_trigger_request_info(t2);
	int trigger_type =
		get_field(t1.tdata1, CSR_MCONTROL_TYPE(riscv_xlen(target)));
	int ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	RISCV_INFO(r);

	/* Find the first 2 consecutive triggers, supporting required tdata1 values */
	for (unsigned int idx = 0;
			find_next_free_trigger(target, trigger_type, true, &idx) == ERROR_OK;
			++idx) {
		ret = try_use_trigger_and_cache_result(target, idx, t1.tdata1, t1.tdata2,
			t1.tdata1_ignore_mask);

		if (ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			continue;
		else if (ret != ERROR_OK)
			return ret;

		ret = try_use_trigger_and_cache_result(target, idx + 1, t2.tdata1, t2.tdata2,
			t2.tdata1_ignore_mask);

		if (ret == ERROR_OK) {
			r->trigger_unique_id[idx] = trigger->unique_id;
			r->trigger_unique_id[idx + 1] = trigger->unique_id;
			return ERROR_OK;
		}
		/* Undo the setting of the previous trigger */
		int ret_undo = set_trigger(target, idx, 0, 0, 0);
		if (ret_undo != ERROR_OK)
			return ret_undo;

		if (ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			return ret;
	}
	return ret;
}

struct match_triggers_tdata1_fields {
	riscv_reg_t common;
	struct {
		/* Other values are available for this field,
		 * but currently only `any` is needed.
		 */
		riscv_reg_t any;
	} size;
	struct {
		riscv_reg_t enable;
		riscv_reg_t disable;
	} chain;
	struct {
		riscv_reg_t napot;
		riscv_reg_t lt;
		riscv_reg_t ge;
		riscv_reg_t eq;
	} match;
	riscv_reg_t tdata1_ignore_mask;
};

static struct match_triggers_tdata1_fields fill_match_triggers_tdata1_fields_t2(struct target *target,
	struct trigger *trigger)
{
	RISCV_INFO(r);

	struct match_triggers_tdata1_fields result = {
		.common =
			field_value(CSR_MCONTROL_TYPE(riscv_xlen(target)), CSR_TDATA1_TYPE_MCONTROL) |
			field_value(CSR_MCONTROL_DMODE(riscv_xlen(target)), 1) |
			field_value(CSR_MCONTROL_ACTION, CSR_MCONTROL_ACTION_DEBUG_MODE) |
			field_value(CSR_MCONTROL_M, 1) |
			field_value(CSR_MCONTROL_S, !!(r->misa & BIT('S' - 'A'))) |
			field_value(CSR_MCONTROL_U, !!(r->misa & BIT('U' - 'A'))) |
			field_value(CSR_MCONTROL_EXECUTE, trigger->is_execute) |
			field_value(CSR_MCONTROL_LOAD, trigger->is_read) |
			field_value(CSR_MCONTROL_STORE, trigger->is_write),
		.size = {
			.any =
				field_value(CSR_MCONTROL_SIZELO, CSR_MCONTROL_SIZELO_ANY & 3) |
				field_value(CSR_MCONTROL_SIZEHI, (CSR_MCONTROL_SIZELO_ANY >> 2) & 3)
		},
		.chain = {
			.enable = field_value(CSR_MCONTROL_CHAIN, CSR_MCONTROL_CHAIN_ENABLED),
			.disable = field_value(CSR_MCONTROL_CHAIN, CSR_MCONTROL_CHAIN_DISABLED)
		},
		.match = {
			.napot = field_value(CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_NAPOT),
			.lt = field_value(CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_LT),
			.ge = field_value(CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_GE),
			.eq = field_value(CSR_MCONTROL_MATCH, CSR_MCONTROL_MATCH_EQUAL)
		},
		.tdata1_ignore_mask = CSR_MCONTROL_MASKMAX(riscv_xlen(target))
	};
	return result;
}

static struct match_triggers_tdata1_fields fill_match_triggers_tdata1_fields_t6(struct target *target,
	struct trigger *trigger)
{
	bool misa_s = riscv_supports_extension(target, 'S');
	bool misa_u = riscv_supports_extension(target, 'U');
	bool misa_h = riscv_supports_extension(target, 'H');

	struct match_triggers_tdata1_fields result = {
		.common =
			field_value(CSR_MCONTROL6_TYPE(riscv_xlen(target)), CSR_TDATA1_TYPE_MCONTROL6) |
			field_value(CSR_MCONTROL6_DMODE(riscv_xlen(target)), 1) |
			field_value(CSR_MCONTROL6_ACTION, CSR_MCONTROL_ACTION_DEBUG_MODE) |
			field_value(CSR_MCONTROL6_M, 1) |
			field_value(CSR_MCONTROL6_S, misa_s) |
			field_value(CSR_MCONTROL6_U, misa_u) |
			field_value(CSR_MCONTROL6_VS, misa_h && misa_s) |
			field_value(CSR_MCONTROL6_VU, misa_h && misa_u) |
			field_value(CSR_MCONTROL6_EXECUTE, trigger->is_execute) |
			field_value(CSR_MCONTROL6_LOAD, trigger->is_read) |
			field_value(CSR_MCONTROL6_STORE, trigger->is_write),
		.size = {
			.any = field_value(CSR_MCONTROL6_SIZE, CSR_MCONTROL6_SIZE_ANY)
		},
		.chain = {
			.enable = field_value(CSR_MCONTROL6_CHAIN, CSR_MCONTROL6_CHAIN_ENABLED),
			.disable = field_value(CSR_MCONTROL6_CHAIN, CSR_MCONTROL6_CHAIN_DISABLED)
		},
		.match = {
			.napot = field_value(CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_NAPOT),
			.lt = field_value(CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_LT),
			.ge = field_value(CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_GE),
			.eq = field_value(CSR_MCONTROL6_MATCH, CSR_MCONTROL6_MATCH_EQUAL)
		},
		.tdata1_ignore_mask = 0
	};
	return result;
}

static int maybe_add_trigger_t2_t6_for_wp(struct target *target,
		struct trigger *trigger, struct match_triggers_tdata1_fields fields)
{
	RISCV_INFO(r);
	int ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (trigger->length > 0) {
		/* Setting a load/store trigger ("watchpoint") on a range of addresses */
		if (can_use_napot_match(trigger)) {
			if (r->wp_allow_napot_trigger) {
				LOG_TARGET_DEBUG(target, "trying to setup NAPOT match trigger");
				struct trigger_request_info napot = {
					.tdata1 = fields.common | fields.size.any |
						fields.chain.disable | fields.match.napot,
					.tdata2 = trigger->address | ((trigger->length - 1) >> 1),
					.tdata1_ignore_mask = fields.tdata1_ignore_mask
				};
				ret = try_setup_single_match_trigger(target, trigger, napot);
				if (ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
					return ret;
			} else {
				LOG_TARGET_DEBUG(target, "NAPOT match triggers are disabled for watchpoints. "
							 "Use 'riscv set_enable_trigger_feature napot wp' to enable it.");
			}
		}

		if (r->wp_allow_ge_lt_trigger) {
			LOG_TARGET_DEBUG(target, "trying to setup GE+LT chained match trigger pair");
			struct trigger_request_info ge_1 = {
				.tdata1 = fields.common | fields.size.any | fields.chain.enable |
					fields.match.ge,
				.tdata2 = trigger->address,
				.tdata1_ignore_mask = fields.tdata1_ignore_mask
			};
			struct trigger_request_info lt_2 = {
				.tdata1 = fields.common | fields.size.any | fields.chain.disable |
					fields.match.lt,
				.tdata2 = trigger->address + trigger->length,
				.tdata1_ignore_mask = fields.tdata1_ignore_mask
			};
			ret = try_setup_chained_match_triggers(target, trigger, ge_1, lt_2);
			if (ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
				return ret;

			LOG_TARGET_DEBUG(target, "trying to setup LT+GE chained match trigger pair");
			struct trigger_request_info lt_1 = {
				.tdata1 = fields.common | fields.size.any | fields.chain.enable |
					fields.match.lt,
				.tdata2 = trigger->address + trigger->length,
				.tdata1_ignore_mask = fields.tdata1_ignore_mask
			};
			struct trigger_request_info ge_2 = {
				.tdata1 = fields.common | fields.size.any | fields.chain.disable |
					fields.match.ge,
				.tdata2 = trigger->address,
				.tdata1_ignore_mask = fields.tdata1_ignore_mask
			};
			ret = try_setup_chained_match_triggers(target, trigger, lt_1, ge_2);
			if (ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
				return ret;
		} else {
			LOG_TARGET_DEBUG(target, "LT+GE chained match triggers are disabled for watchpoints. "
						 "Use 'riscv set_enable_trigger_feature ge_lt wp' to enable it.");
		}
	}

	if (r->wp_allow_equality_match_trigger) {
		LOG_TARGET_DEBUG(target, "trying to setup equality match trigger");
		struct trigger_request_info eq = {
			.tdata1 = fields.common | fields.size.any | fields.chain.disable |
				fields.match.eq,
			.tdata2 = trigger->address,
			.tdata1_ignore_mask = fields.tdata1_ignore_mask
		};
		ret = try_setup_single_match_trigger(target, trigger, eq);
		if (ret != ERROR_OK)
			return ret;
	} else {
		LOG_TARGET_DEBUG(target, "equality match triggers are disabled for watchpoints. "
					 "Use 'riscv set_enable_trigger_feature eq wp' to enable it.");
	}

	if (ret == ERROR_OK && trigger->length > 1) {
		LOG_TARGET_DEBUG(target, "Trigger will match accesses at address 0x%" TARGET_PRIxADDR
				", but may not match accesses at addresses in the inclusive range from 0x%"
				TARGET_PRIxADDR " to 0x%" TARGET_PRIxADDR ".", trigger->address,
				trigger->address + 1, trigger->address + trigger->length - 1);
		RISCV_INFO(info);
		if (!info->range_trigger_fallback_encountered)
			/* This message is displayed only once per target to avoid
			 * overwhelming the user with such messages on resume.
			 */
			LOG_TARGET_WARNING(target,
					"Could not set a trigger that will match a whole address range. "
					"As a fallback, this trigger (and maybe others) will only match "
					"against the first address of the range.");
		info->range_trigger_fallback_encountered = true;
	}

	return ret;
}

static int maybe_add_trigger_t2_t6_for_bp(struct target *target,
		struct trigger *trigger, struct match_triggers_tdata1_fields fields)
{
	LOG_TARGET_DEBUG(target, "trying to setup equality match trigger");
	struct trigger_request_info eq = {
		.tdata1 = fields.common | fields.size.any | fields.chain.disable |
			fields.match.eq,
		.tdata2 = trigger->address,
		.tdata1_ignore_mask = fields.tdata1_ignore_mask
	};

	return try_setup_single_match_trigger(target, trigger, eq);
}

static int maybe_add_trigger_t2_t6(struct target *target,
		struct trigger *trigger, struct match_triggers_tdata1_fields fields)
{
	if (trigger->is_execute) {
		assert(!trigger->is_read && !trigger->is_write);
		return maybe_add_trigger_t2_t6_for_bp(target, trigger, fields);
	}

	assert(trigger->is_read || trigger->is_write);
	return maybe_add_trigger_t2_t6_for_wp(target, trigger, fields);
}

static int maybe_add_trigger_t3(struct target *target, bool vs, bool vu,
				bool m, bool s, bool u, bool pending, unsigned int count,
				int unique_id)
{
	int ret;
	riscv_reg_t tdata1;

	RISCV_INFO(r);

	tdata1 = 0;
	tdata1 = set_field(tdata1, CSR_ICOUNT_TYPE(riscv_xlen(target)), CSR_TDATA1_TYPE_ICOUNT);
	tdata1 = set_field(tdata1, CSR_ICOUNT_DMODE(riscv_xlen(target)), 1);
	tdata1 = set_field(tdata1, CSR_ICOUNT_ACTION, CSR_ICOUNT_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, CSR_ICOUNT_VS, vs);
	tdata1 = set_field(tdata1, CSR_ICOUNT_VU, vu);
	tdata1 = set_field(tdata1, CSR_ICOUNT_PENDING, pending);
	tdata1 = set_field(tdata1, CSR_ICOUNT_M, m);
	tdata1 = set_field(tdata1, CSR_ICOUNT_S, s);
	tdata1 = set_field(tdata1, CSR_ICOUNT_U, u);
	tdata1 = set_field(tdata1, CSR_ICOUNT_COUNT, count);

	unsigned int idx = 0;
	ret = find_next_free_trigger(target, CSR_TDATA1_TYPE_ICOUNT, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	ret = set_trigger(target, idx, tdata1, 0, 0);
	if (ret != ERROR_OK)
		return ret;
	r->trigger_unique_id[idx] = unique_id;
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

	unsigned int idx = 0;
	ret = find_next_free_trigger(target, CSR_TDATA1_TYPE_ITRIGGER, false, &idx);
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

	unsigned int idx = 0;
	ret = find_next_free_trigger(target, CSR_TDATA1_TYPE_ETRIGGER, false, &idx);
	if (ret != ERROR_OK)
		return ret;
	ret = set_trigger(target, idx, tdata1, tdata2, 0);
	if (ret != ERROR_OK)
		return ret;
	r->trigger_unique_id[idx] = unique_id;
	return ERROR_OK;
}

static int add_trigger(struct target *target, struct trigger *trigger)
{
	int ret;
	riscv_reg_t tselect;

	ret = riscv_enumerate_triggers(target);
	if (ret != ERROR_OK)
		return ret;

	ret = riscv_reg_get(target, &tselect, GDB_REGNO_TSELECT);
	if (ret != ERROR_OK)
		return ret;

	do {
		ret = maybe_add_trigger_t1(target, trigger);
		if (ret == ERROR_OK)
			break;
		ret = maybe_add_trigger_t2_t6(target, trigger,
				fill_match_triggers_tdata1_fields_t2(target, trigger));
		if (ret == ERROR_OK)
			break;
		ret = maybe_add_trigger_t2_t6(target, trigger,
				fill_match_triggers_tdata1_fields_t6(target, trigger));
		if (ret == ERROR_OK)
			break;
	} while (0);

	if (riscv_reg_set(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK &&
			ret == ERROR_OK)
		return ERROR_FAIL;

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

static int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	LOG_TARGET_DEBUG(target, "@0x%" TARGET_PRIxADDR, breakpoint->address);
	assert(breakpoint);
	if (breakpoint->type == BKPT_SOFT) {
		/** @todo check RVC for size/alignment */
		if (!(breakpoint->length == 4 || breakpoint->length == 2)) {
			LOG_TARGET_ERROR(target, "Invalid breakpoint length %d", breakpoint->length);
			return ERROR_FAIL;
		}

		if (0 != (breakpoint->address % 2)) {
			LOG_TARGET_ERROR(target, "Invalid breakpoint alignment for address 0x%" TARGET_PRIxADDR,
				breakpoint->address);
			return ERROR_FAIL;
		}

		/* Read the original instruction. */
		if (riscv_read_by_any_size(
				target, breakpoint->address, breakpoint->length, breakpoint->orig_instr) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to read original instruction at 0x%" TARGET_PRIxADDR,
					breakpoint->address);
			return ERROR_FAIL;
		}

		uint8_t buff[4] = { 0 };
		buf_set_u32(buff, 0, breakpoint->length * CHAR_BIT, breakpoint->length == 4 ? ebreak() : ebreak_c());
		/* Write the ebreak instruction. */
		if (riscv_write_by_any_size(target, breakpoint->address, breakpoint->length, buff) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to write %d-byte breakpoint instruction at 0x%"
					TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}
		breakpoint->is_set = true;

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int const result = add_trigger(target, &trigger);
		if (result != ERROR_OK)
			return result;

		int trigger_idx = find_first_trigger_by_id(target, breakpoint->unique_id);
		breakpoint_hw_set(breakpoint, trigger_idx);
	} else {
		LOG_TARGET_INFO(target, "OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return ERROR_OK;
}

static int remove_trigger(struct target *target, int unique_id)
{
	RISCV_INFO(r);

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_reg_t tselect;
	int result = riscv_reg_get(target, &tselect, GDB_REGNO_TSELECT);
	if (result != ERROR_OK)
		return result;

	bool done = false;
	for (unsigned int i = 0; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == unique_id) {
			riscv_reg_set(target, GDB_REGNO_TSELECT, i);
			riscv_reg_set(target, GDB_REGNO_TDATA1, 0);
			r->trigger_unique_id[i] = -1;
			LOG_TARGET_DEBUG(target, "Stop using resource %d for bp %d",
				i, unique_id);
			done = true;
		}
	}
	if (!done) {
		LOG_TARGET_ERROR(target,
			"Couldn't find the hardware resources used by hardware trigger.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_reg_set(target, GDB_REGNO_TSELECT, tselect);

	return ERROR_OK;
}

static int riscv_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		/* Write the original instruction. */
		if (riscv_write_by_any_size(
				target, breakpoint->address, breakpoint->length, breakpoint->orig_instr) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to restore instruction for %d-byte breakpoint at "
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
		LOG_TARGET_INFO(target, "OpenOCD only supports hardware and software breakpoints.");
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
	trigger->is_read = (watchpoint->rw == WPT_READ || watchpoint->rw == WPT_ACCESS);
	trigger->is_write = (watchpoint->rw == WPT_WRITE || watchpoint->rw == WPT_ACCESS);
	trigger->is_execute = false;
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = watchpoint->unique_id;
}

int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	if (watchpoint->mask != WATCHPOINT_IGNORE_DATA_VALUE_MASK) {
		LOG_TARGET_ERROR(target, "Watchpoints on data values are not implemented");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = add_trigger(target, &trigger);
	if (result != ERROR_OK)
		return result;

	int trigger_idx = find_first_trigger_by_id(target, watchpoint->unique_id);
	watchpoint_set(watchpoint, trigger_idx);

	return ERROR_OK;
}

int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	LOG_TARGET_DEBUG(target, "Removing watchpoint @0x%" TARGET_PRIxADDR, watchpoint->address);

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
 * RISCV_TRIGGER_HIT_NOT_FOUND, no match was found.
 */

static int riscv_trigger_detect_hit_bits(struct target *target, int64_t *unique_id)
{
	/* FIXME: this function assumes that we have only one trigger that can
	 * have hit bit set. Debug spec allows hit bit to bit set if a trigger has
	 * matched but did not fire. Such targets will receive erroneous results.
	 */

	// FIXME: Add hit bits support detection and caching
	RISCV_INFO(r);

	riscv_reg_t tselect;
	if (riscv_reg_get(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
		return ERROR_FAIL;

	*unique_id = RISCV_TRIGGER_HIT_NOT_FOUND;
	for (unsigned int i = 0; i < r->trigger_count; i++) {
		if (r->trigger_unique_id[i] == -1)
			continue;

		if (riscv_reg_set(target, GDB_REGNO_TSELECT, i) != ERROR_OK)
			return ERROR_FAIL;

		uint64_t tdata1;
		if (riscv_reg_get(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
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
				hit_mask = CSR_MCONTROL6_HIT0 | CSR_MCONTROL6_HIT1;
				break;
			case CSR_TDATA1_TYPE_ICOUNT:
				hit_mask = CSR_ICOUNT_HIT;
				break;
			case CSR_TDATA1_TYPE_ITRIGGER:
				hit_mask = CSR_ITRIGGER_HIT(riscv_xlen(target));
				break;
			case CSR_TDATA1_TYPE_ETRIGGER:
				hit_mask = CSR_ETRIGGER_HIT(riscv_xlen(target));
				break;
			default:
				LOG_TARGET_DEBUG(target, "Trigger %u has unknown type %d", i, type);
				continue;
		}

		/* FIXME: this logic needs to be changed to ignore triggers that are not
		 * the last one in the chain. */
		if (tdata1 & hit_mask) {
			LOG_TARGET_DEBUG(target, "Trigger %u (unique_id=%" PRIi64 ") has hit bit set.",
				i, r->trigger_unique_id[i]);
			if (riscv_reg_set(target, GDB_REGNO_TDATA1, tdata1 & ~hit_mask) != ERROR_OK)
				return ERROR_FAIL;

			*unique_id = r->trigger_unique_id[i];
			break;
		}
	}

	if (riscv_reg_set(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * These functions are needed to extract individual bits (for offset)
 * from the instruction
 */
// c.lwsp rd_n0 c_uimm8sphi c_uimm8splo - offset[5] offset[4:2|7:6]
static uint16_t get_offset_clwsp(riscv_insn_t instruction)
{
	uint16_t offset_4to2and7to6_bits =
		get_field32(instruction, INSN_FIELD_C_UIMM8SPLO);
	uint16_t offset_4to2_bits = offset_4to2and7to6_bits >> 2;
	uint16_t offset_7to6_bits = offset_4to2and7to6_bits & 0x3;
	uint16_t offset_5_bit = get_field32(instruction, INSN_FIELD_C_UIMM8SPHI);
	return (offset_4to2_bits << 2) + (offset_5_bit << 5)
		   + (offset_7to6_bits << 6);
}

// c.ldsp rd_n0 c_uimm9sphi c_uimm9splo - offset[5] offset[4:3|8:6]
static uint16_t get_offset_cldsp(riscv_insn_t instruction)
{
	uint16_t offset_4to3and8to6_bits =
		get_field32(instruction, INSN_FIELD_C_UIMM9SPLO);
	uint16_t offset_4to3_bits = offset_4to3and8to6_bits >> 3;
	uint16_t offset_8to6_bits = offset_4to3and8to6_bits & 0x7;
	uint16_t offset_5_bit = get_field32(instruction, INSN_FIELD_C_UIMM9SPHI);
	return (offset_4to3_bits << 3) + (offset_5_bit << 5)
		   + (offset_8to6_bits << 6);
}

// c.swsp c_rs2 c_uimm8sp_s - offset[5:2|7:6]
static uint16_t get_offset_cswsp(riscv_insn_t instruction)
{
	uint16_t offset_5to2and7to6_bits =
		get_field32(instruction, INSN_FIELD_C_UIMM8SP_S);
	uint16_t offset_5to2_bits = offset_5to2and7to6_bits >> 2;
	uint16_t offset_7to6_bits = offset_5to2and7to6_bits & 0x3;
	return (offset_5to2_bits << 2) + (offset_7to6_bits << 6);
}

// c.sdsp c_rs2 c_uimm9sp_s - offset[5:3|8:6]
static uint16_t get_offset_csdsp(riscv_insn_t instruction)
{
	uint16_t offset_5to3and8to6_bits =
		get_field32(instruction, INSN_FIELD_C_UIMM9SP_S);
	uint16_t offset_5to3_bits = offset_5to3and8to6_bits >> 3;
	uint16_t offset_8to6_bits = offset_5to3and8to6_bits & 0x7;
	return (offset_5to3_bits << 3) + (offset_8to6_bits << 6);
}

// c.lw rd_p rs1_p c_uimm7lo c_uimm7hi - offset[2|6] offset[5:3]
static uint16_t get_offset_clw(riscv_insn_t instruction)
{
	uint16_t offset_2and6_bits = get_field32(instruction, INSN_FIELD_C_UIMM7LO);
	uint16_t offset_2_bit = offset_2and6_bits >> 1;
	uint16_t offset_6_bit = offset_2and6_bits & 0x1;
	uint16_t offset_5to3_bits = get_field32(instruction, INSN_FIELD_C_UIMM7HI);
	return (offset_2_bit << 2) + (offset_5to3_bits << 3) + (offset_6_bit << 6);
}

// c.ld rd_p rs1_p c_uimm8lo c_uimm8hi - offset[7:6] offset[5:3]
static uint16_t get_offset_cld(riscv_insn_t instruction)
{
	uint16_t offset_7to6_bits = get_field32(instruction, INSN_FIELD_C_UIMM8LO);
	uint16_t offset_5to3_bits = get_field32(instruction, INSN_FIELD_C_UIMM8HI);
	return (offset_5to3_bits << 3) + (offset_7to6_bits << 6);
}

// c.lq rd_p rs1_p c_uimm9lo c_uimm9hi - offset[7:6] offset[5|4|8]
static uint16_t get_offset_clq(riscv_insn_t instruction)
{
	uint16_t offset_7to6_bits = get_field32(instruction, INSN_FIELD_C_UIMM9LO);
	uint16_t offset_5to4and8_bits =
		get_field32(instruction, INSN_FIELD_C_UIMM9HI);
	uint16_t offset_5to4_bits = offset_5to4and8_bits >> 1;
	uint16_t offset_8_bit = offset_5to4and8_bits & 0x1;
	return (offset_5to4_bits << 4) + (offset_7to6_bits << 6)
		   + (offset_8_bit << 8);
}

// c.lqsp rd_n0 c_uimm10sphi c_uimm10splo - offset[5] offset[4|9:6]
static uint16_t get_offset_clqsp(riscv_insn_t instruction)
{
	uint16_t offset_4and9to6_bits =
		get_field32(instruction, INSN_FIELD_C_UIMM10SPLO);
	uint16_t offset_4_bit = offset_4and9to6_bits >> 4;
	uint16_t offset_9to6_bits = offset_4and9to6_bits & 0xf;
	uint16_t offset_5_bit = get_field32(instruction, INSN_FIELD_C_UIMM10SPHI);
	return (offset_4_bit << 4) + (offset_5_bit << 5) + (offset_9to6_bits << 6);
}

// c.sqsp c_rs2 c_uimm10sp_s - offset[5:4|9:6]
static uint16_t get_offset_csqsp(riscv_insn_t instruction)
{
	uint16_t offset_5to4and9to6_bits =
		get_field32(instruction, INSN_FIELD_C_UIMM10SP_S);
	uint16_t offset_5to4_biits = offset_5to4and9to6_bits >> 4;
	uint16_t offset_9to6_bits = offset_5to4and9to6_bits & 0xf;
	return (offset_5to4_biits << 4) + (offset_9to6_bits << 6);
}

/**
 * Decode rs1' register num for RVC.
 * See "Table: Registers specified by the three-bit rs1′, rs2′, and rd′ fields
 * of the CIW, CL, CS, CA, and CB formats" in "The RISC-V Instruction Set Manual
 * Volume I: Unprivileged ISA".
 * */
static uint32_t get_rs1_c(riscv_insn_t instruction)
{
	return GDB_REGNO_S0 + get_field32(instruction, INSN_FIELD_C_SREG1);
}

static uint32_t get_opcode(const riscv_insn_t instruction)
{
	// opcode is first 7 bits of the instruction
	uint32_t opcode = instruction & INSN_FIELD_OPCODE;
	if ((instruction & 0x03) < 0x03) { // opcode size RVC
		// RVC MASK_C = 0xe003 for load/store instructions
		opcode = instruction & MASK_C_LD;
	}
	return opcode;
}

static int get_loadstore_membase_regno(struct target *target,
		const riscv_insn_t instruction, int *regid)
{
	uint32_t opcode = get_opcode(instruction);
	int rs;

	switch (opcode) {
	case MATCH_LB:
	case MATCH_FLH & ~INSN_FIELD_FUNCT3:
	case MATCH_SB:
	case MATCH_FSH & ~INSN_FIELD_FUNCT3:
		rs = get_field32(instruction, INSN_FIELD_RS1);
		break;

	case MATCH_C_LWSP:
	case MATCH_C_LDSP: // if xlen >= 64 or MATCH_C_FLWSP:
	case MATCH_C_FLDSP: // or MATCH_C_LQSP if xlen == 128
	case MATCH_C_SWSP:
	case MATCH_C_SDSP: // if xlen >= 64 or MATCH_C_FSWSP:
	case MATCH_C_FSDSP: // or MATCH_C_SQSP if xlen == 128
		rs = GDB_REGNO_SP;
		break;

	case MATCH_C_LW:
	case MATCH_C_FLW: // or MATCH_C_LD if xlen >= 64
	case MATCH_C_FLD: // or MATCH_C_LQ if xlen == 128
	case MATCH_C_SW:
	case MATCH_C_FSW: // or MATCH_C_SD if xlen >= 64
	case MATCH_C_FSD: // or MATCH_C_SQ if xlen == 128
		rs = get_rs1_c(instruction);
		break;

	default:
		LOG_TARGET_DEBUG(target, "0x%" PRIx32 " is not a RV32I or \"C\" load or"
						 " store", instruction);
		return ERROR_FAIL;
	}
	*regid = rs;
	return ERROR_OK;
}

static int get_loadstore_memoffset(struct target *target,
		const riscv_insn_t instruction, int16_t *memoffset)
{
	uint32_t opcode = get_opcode(instruction);
	int16_t offset;

	switch (opcode) {
	case MATCH_LB:
	case MATCH_FLH & ~INSN_FIELD_FUNCT3:
	case MATCH_SB:
	case MATCH_FSH & ~INSN_FIELD_FUNCT3:
		if (opcode == MATCH_SB || opcode == (MATCH_FSH & ~INSN_FIELD_FUNCT3)) {
			offset = get_field32(instruction, INSN_FIELD_IMM12LO) |
				  (get_field32(instruction, INSN_FIELD_IMM12HI) << 5);
		} else if (opcode == MATCH_LB ||
				   opcode == (MATCH_FLH & ~INSN_FIELD_FUNCT3)) {
			offset = get_field32(instruction, INSN_FIELD_IMM12);
		} else {
			assert(false);
		}
		/* sign extend 12-bit imm to 16-bits */
		if (offset & (1 << 11))
			offset |= 0xf000;
		break;

	case MATCH_C_LWSP:
		offset = get_offset_clwsp(instruction);
		break;

	case MATCH_C_LDSP: // if xlen >= 64 or MATCH_C_FLWSP:
		if (riscv_xlen(target) > 32) { // MATCH_C_LDSP
			offset = get_offset_cldsp(instruction);
		} else { // MATCH_C_FLWSP
			offset = get_offset_clwsp(instruction);
		}
		break;

	case MATCH_C_FLDSP: // or MATCH_C_LQSP if xlen == 128
		if (riscv_xlen(target) == 128) { // MATCH_C_LQSP
			offset = get_offset_clqsp(instruction);
		} else { // MATCH_C_FLDSP
			offset = get_offset_cldsp(instruction);
		}
		break;

	case MATCH_C_SWSP:
		offset = get_offset_cswsp(instruction);
		break;

	case MATCH_C_SDSP: // if xlen >= 64 or MATCH_C_FSWSP:
		if (riscv_xlen(target) > 32) { // MATCH_C_SDSP
			offset = get_offset_csdsp(instruction);
		} else { // MATCH_C_FSWSP
			offset = get_offset_cswsp(instruction);
		}
		break;

	case MATCH_C_FSDSP: // or MATCH_C_SQSP if xlen == 128
		if (riscv_xlen(target) == 128) { // MATCH_C_SQSP
			offset = get_offset_csqsp(instruction);
		} else { // MATCH_C_FSDSP
			offset = get_offset_csdsp(instruction); // same as C.SDSP
		}
		break;

	case MATCH_C_LW:
		offset = get_offset_clw(instruction);
		break;

	case MATCH_C_FLW: // or MATCH_C_LD if xlen >= 64
		if (riscv_xlen(target) > 32) { // MATCH_C_LD
			offset = get_offset_cld(instruction);
		} else { // MATCH_C_FLW
			offset = get_offset_clw(instruction); // same as C.FLW
		}
		break;

	case MATCH_C_FLD: // or MATCH_C_LQ if xlen == 128
		if (riscv_xlen(target) == 128) { // MATCH_C_LQ
			offset = get_offset_clq(instruction);
		} else { // MATCH_C_FLD
			offset = get_offset_cld(instruction); // same as C.LD
		}
		break;

	case MATCH_C_SW:
		offset = get_offset_clw(instruction); // same as C.LW
		break;

	case MATCH_C_FSW: // or MATCH_C_SD if xlen >= 64
		if (riscv_xlen(target) > 32) { // MATCH_C_SD
			offset = get_offset_cld(instruction); // same as C.LD
		} else { // MATCH_C_FSW
			offset = get_offset_clw(instruction); // same as C.LW
		}
		break;

	case MATCH_C_FSD: // or MATCH_C_SQ if xlen == 128
		if (riscv_xlen(target) == 128) { // MATCH_C_SQ
			offset = get_offset_clq(instruction); // same as C.LQ
		} else { // MATCH_C_FSD
			offset = get_offset_cld(instruction); // same as C.LD
		}
		break;

	default:
		LOG_TARGET_DEBUG(target, "0x%" PRIx32 " is not a RV32I or \"C\" load or"
						 " store", instruction);
		return ERROR_FAIL;
	}
	*memoffset = offset;
	return ERROR_OK;
}

static int verify_loadstore(struct target *target,
		const riscv_insn_t instruction, bool *is_read)
{
	uint32_t opcode = get_opcode(instruction);
	bool misa_f = riscv_supports_extension(target, 'F');
	bool misa_d = riscv_supports_extension(target, 'D');
	enum watchpoint_rw rw;

	switch (opcode) {
	case MATCH_LB:
	case MATCH_FLH & ~INSN_FIELD_FUNCT3:
		rw = WPT_READ;
		break;

	case MATCH_SB:
	case MATCH_FSH & ~INSN_FIELD_FUNCT3:
		rw = WPT_WRITE;
		break;

	case MATCH_C_LWSP:
		if (get_field32(instruction, INSN_FIELD_RD) == 0) {
			LOG_TARGET_DEBUG(target,
				"The code points with rd = x0 are reserved for C.LWSP");
			return ERROR_FAIL;
		}
		rw = WPT_READ;
		break;

	case MATCH_C_LDSP: // if xlen >= 64 or MATCH_C_FLWSP:
		if (riscv_xlen(target) > 32) { // MATCH_C_LDSP
			if (get_field32(instruction, INSN_FIELD_RD) == 0) {
				LOG_TARGET_DEBUG(target,
					"The code points with rd = x0 are reserved for C.LDSP");
				return ERROR_FAIL;
			}
		} else { // MATCH_C_FLWSP
			if (!misa_f) {
				LOG_TARGET_DEBUG(target, "Matched C.FLWSP but target doesn\'t "
								 "have the \"F\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_READ;
		break;

	case MATCH_C_FLDSP: // or MATCH_C_LQSP if xlen == 128
		if (riscv_xlen(target) == 128) { // MATCH_C_LQSP
			if (get_field32(instruction, INSN_FIELD_RD) == 0) {
				LOG_TARGET_DEBUG(target,
					"The code points with rd = x0 are reserved for C.LQSP");
				return ERROR_FAIL;
			}
		} else { // MATCH_C_FLDSP
			if (!misa_d) {
				LOG_TARGET_DEBUG(target, "Matched C.FLDSP but target doesn\'t "
								 "have the \"D\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_READ;
		break;

	case MATCH_C_SWSP:
		rw = WPT_WRITE;
		break;

	case MATCH_C_SDSP: // if xlen >= 64 or MATCH_C_FSWSP:
		if (riscv_xlen(target) == 32) { // MATCH_C_FSWSP
			if (!misa_f) {
				LOG_TARGET_DEBUG(target, "Matched C.FSWSP but target doesn\'t "
								 "have the \"F\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_WRITE;
		break;

	case MATCH_C_FSDSP: // or MATCH_C_SQSP if xlen == 128
		if (riscv_xlen(target) != 128) { // MATCH_C_SQSP
			if (!misa_d) {
				LOG_TARGET_DEBUG(target, "Matched C.FSDSP but target doesn\'t "
								 "have the \"D\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_WRITE;
		break;

	case MATCH_C_LW:
		rw = WPT_READ;
		break;

	case MATCH_C_FLW: // or MATCH_C_LD if xlen >= 64
		if (riscv_xlen(target) == 32) { // MATCH_C_FLW
			if (!misa_f) {
				LOG_TARGET_DEBUG(target, "Matched C.FLW but target doesn\'t "
								 "have the \"F\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_READ;
		break;

	case MATCH_C_FLD: // or MATCH_C_LQ if xlen == 128
		if (riscv_xlen(target) != 128) { // MATCH_C_FLD
			if (!misa_d) {
				LOG_TARGET_DEBUG(target, "Matched C.FLD but target doesn\'t "
								 "have the \"D\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_READ;
		break;

	case MATCH_C_SW:
		rw = WPT_WRITE;
		break;

	case MATCH_C_FSW: // or MATCH_C_SD if xlen >= 64
		if (riscv_xlen(target) == 32) { // MATCH_C_FSW
			if (!misa_f) {
				LOG_TARGET_DEBUG(target, "Matched C.FSW but target doesn\'t "
								 "have the \"F\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_WRITE;
		break;

	case MATCH_C_FSD: // or MATCH_C_SQ if xlen == 128
		if (riscv_xlen(target) != 128) { // MATCH_C_FSD
			if (!misa_d) {
				LOG_TARGET_DEBUG(target, "Matched C.FSD but target doesn\'t "
								 "have the \"D\" extension");
				return ERROR_FAIL;
			}
		}
		rw = WPT_WRITE;
		break;

	default:
		LOG_TARGET_DEBUG(target, "0x%" PRIx32 " is not a RV32I or \"C\" load or"
						 " store", instruction);
		return ERROR_FAIL;
	}

	if (rw == WPT_WRITE) {
		*is_read = false;
		LOG_TARGET_DEBUG(target, "0x%" PRIx32 " is store instruction",
						 instruction);
	} else {
		*is_read = true;
		LOG_TARGET_DEBUG(target, "0x%" PRIx32 " is load instruction",
						 instruction);
	}
	return ERROR_OK;
}

/* Sets *hit_watchpoint to the first watchpoint identified as causing the
 * current halt.
 *
 * The GDB server uses this information to tell GDB what data address has
 * been hit, which enables GDB to print the hit variable along with its old
 * and new value. */
static int riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
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
	if (riscv_reg_get(target, &dpc, GDB_REGNO_DPC) != ERROR_OK)
		return ERROR_FAIL;
	const uint8_t length = 4;
	LOG_TARGET_DEBUG(target, "dpc is 0x%" PRIx64, dpc);

	/* fetch the instruction at dpc */
	uint8_t buffer[length];
	if (target_read_buffer(target, dpc, length, buffer) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read instruction at dpc 0x%" PRIx64,
						 dpc);
		return ERROR_FAIL;
	}

	riscv_insn_t instruction = 0;

	for (int i = 0; i < length; i++) {
		LOG_TARGET_DEBUG(target, "Next byte is %x", buffer[i]);
		instruction += (buffer[i] << 8 * i);
	}
	LOG_TARGET_DEBUG(target, "Full instruction is %x", instruction);

	int rs;
	target_addr_t mem_addr;
	int16_t memoffset;

	if (get_loadstore_membase_regno(target, instruction, &rs) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_reg_get(target, &mem_addr, rs) != ERROR_OK)
		return ERROR_FAIL;
	if (get_loadstore_memoffset(target, instruction, &memoffset) != ERROR_OK)
		return ERROR_FAIL;

	mem_addr += memoffset;
	bool is_load;

	if (verify_loadstore(target, instruction, &is_load) != ERROR_OK)
		return ERROR_FAIL;

	struct watchpoint *wp = target->watchpoints;
	while (wp) {
		/* TODO support mask and check read/write/access */
		/* TODO check for intersection of the access range and watchpoint range
				Recommended matching:
				if (intersects(mem_addr, mem_addr + ref_size, wp->address, wp->address + wp->length))
		*/
		if (mem_addr >= wp->address &&
			mem_addr < (wp->address + wp->length)) {
			*hit_watchpoint = wp;
			LOG_TARGET_DEBUG(target, "WP hit found: %s 0x%" TARGET_PRIxADDR
				" covered by %s wp at address 0x%" TARGET_PRIxADDR,
				is_load ? "Load from" : "Store to", mem_addr,
				(wp->rw == WPT_READ ?
					"read" : (wp->rw == WPT_WRITE ? "write" : "access")),
				wp->address);
			return ERROR_OK;
		}
		wp = wp->next;
	}

	/* No match found - either we hit a watchpoint caused by an instruction that
	 * this function does not yet disassemble, or we hit a breakpoint.
	 *
	 * OpenOCD will behave as if this function had never been implemented i.e.
	 * report the halt to GDB with no address information. */
	LOG_TARGET_DEBUG(target, "No watchpoint found that would cover %s 0x%"
		TARGET_PRIxADDR, is_load ? "load from" : "store to", mem_addr);
	return ERROR_FAIL;
}

static int oldriscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	struct target_type *tt = get_target_type(target);
	if (!tt)
		return ERROR_FAIL;
	return tt->step(target, current, address, handle_breakpoints);
}

static int riscv_openocd_step_impl(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int handle_callbacks);

static int old_or_new_riscv_step_impl(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int handle_callbacks)
{
	RISCV_INFO(r);
	LOG_TARGET_DEBUG(target, "handle_breakpoints=%d", handle_breakpoints);
	if (!r->get_hart_state)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else
		return riscv_openocd_step_impl(target, current, address, handle_breakpoints,
			handle_callbacks);
}

static int old_or_new_riscv_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	return old_or_new_riscv_step_impl(target, current, address,
		handle_breakpoints, true /* handle callbacks*/);
}

static int riscv_examine(struct target *target)
{
	LOG_TARGET_DEBUG(target, "Starting examination");
	if (target_was_examined(target)) {
		LOG_TARGET_DEBUG(target, "Target was already examined.");
		return ERROR_OK;
	}

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	RISCV_INFO(info);
	uint32_t dtmcontrol;
	if (dtmcontrol_scan(target, 0, &dtmcontrol) != ERROR_OK || dtmcontrol == 0) {
		LOG_TARGET_ERROR(target, "Could not read dtmcontrol. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	LOG_TARGET_DEBUG(target, "dtmcontrol=0x%x", dtmcontrol);
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_TARGET_DEBUG(target, "version=0x%x", info->dtm_version);

	int examine_status = ERROR_FAIL;
	struct target_type *tt = get_target_type(target);
	if (!tt)
		goto examine_fail;

	examine_status = tt->init_target(info->cmd_ctx, target);
	if (examine_status != ERROR_OK)
		goto examine_fail;

	examine_status = tt->examine(target);
	if (examine_status != ERROR_OK)
		goto examine_fail;

	return ERROR_OK;

examine_fail:
	info->dtm_version = DTM_DTMCS_VERSION_UNKNOWN;
	return examine_status;
}

static int oldriscv_poll(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	if (!tt)
		return ERROR_FAIL;
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

static enum target_debug_reason
derive_debug_reason_without_hitbit(const struct target *target, riscv_reg_t dpc)
{
	/* TODO: if we detect that etrigger/itrigger/icount is set, we should
	 * just report DBG_REASON_UNKNOWN, since we can't disctiguish these
	 * triggers from BP/WP or from other triggers of such type. However,
	 * currently this renders existing testsuite as failing. We need to
	 * fix the testsuite first
	 */
	// TODO: the code below does not handle context-aware trigger types
	for (const struct breakpoint *bp = target->breakpoints; bp; bp = bp->next) {
		// TODO: investigate if we need to handle bp length
		if (bp->type == BKPT_HARD && bp->is_set && bp->address == dpc) {
			// FIXME: bp->linked_brp is uninitialized
			if (bp->asid) {
				LOG_TARGET_ERROR(target,
					"can't derive debug reason for context-aware breakpoint: "
					"unique_id = %" PRIu32 ", address = %" TARGET_PRIxADDR
					", asid = %" PRIx32 ", linked = %d",
					bp->unique_id, bp->address, bp->asid, bp->linked_brp);
				return DBG_REASON_UNDEFINED;
			}
			return DBG_REASON_BREAKPOINT;
		}
	}
	return DBG_REASON_WATCHPOINT;
}
/**
 * Set OpenOCD's generic debug reason from the RISC-V halt reason.
 */
static int set_debug_reason(struct target *target, enum riscv_halt_reason halt_reason)
{
	RISCV_INFO(r);
	r->trigger_hit = -1;
	switch (halt_reason) {
		case RISCV_HALT_EBREAK:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case RISCV_HALT_TRIGGER:
			target->debug_reason = DBG_REASON_UNDEFINED;
			if (riscv_trigger_detect_hit_bits(target, &r->trigger_hit) != ERROR_OK)
				return ERROR_FAIL;
			// FIXME: handle multiple hit bits
			if (r->trigger_hit != RISCV_TRIGGER_HIT_NOT_FOUND) {
				/* We scan for breakpoints first. If no breakpoints are found we still
				 * assume that debug reason is DBG_REASON_BREAKPOINT, unless
				 * there is a watchpoint match - This is to take
				 * ETrigger/ITrigger/ICount into account
				 */
				LOG_TARGET_DEBUG(target,
					"Active hit bit is detected, trying to find trigger owner.");
				for (struct breakpoint *bp = target->breakpoints; bp; bp = bp->next) {
					if (bp->unique_id == r->trigger_hit) {
						target->debug_reason = DBG_REASON_BREAKPOINT;
						LOG_TARGET_DEBUG(target,
							"Breakpoint with unique_id = %" PRIu32 " owns the trigger.",
							bp->unique_id);
					}
				}
				if (target->debug_reason == DBG_REASON_UNDEFINED) {
					// by default we report all triggers as breakpoints
					target->debug_reason = DBG_REASON_BREAKPOINT;
					for (struct watchpoint *wp = target->watchpoints; wp; wp = wp->next) {
						if (wp->unique_id == r->trigger_hit) {
							target->debug_reason = DBG_REASON_WATCHPOINT;
							LOG_TARGET_DEBUG(target,
								"Watchpoint with unique_id = %" PRIu32 " owns the trigger.",
								wp->unique_id);
						}
					}
				}
			} else {
				LOG_TARGET_DEBUG(target,
					"No trigger hit found, deriving debug reason without it.");
				riscv_reg_t dpc;
				if (riscv_reg_get(target, &dpc, GDB_REGNO_DPC) != ERROR_OK)
					return ERROR_FAIL;
				/* Here we don't have the hit bit set (likely, HW does not support it).
				 * We are trying to guess the state. But here comes the problem:
				 * if we have etrigger/itrigger/icount raised - we can't really
				 * distinguish it from the breakpoint or watchpoint. There is not
				 * much we can do here, except for checking current PC against pending
				 * breakpoints and hope for the best)
				 */
				target->debug_reason = derive_debug_reason_without_hitbit(target, dpc);
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
	LOG_TARGET_DEBUG(target, "debug_reason=%d", target->debug_reason);

	return ERROR_OK;
}

static int halt_prep(struct target *target)
{
	RISCV_INFO(r);

	LOG_TARGET_DEBUG(target, "prep hart, debug_reason=%d", target->debug_reason);
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

static int riscv_halt_go_all_harts(struct target *target)
{
	RISCV_INFO(r);

	enum riscv_hart_state state;
	if (riscv_get_hart_state(target, &state) != ERROR_OK)
		return ERROR_FAIL;
	if (state == RISCV_STATE_HALTED) {
		LOG_TARGET_DEBUG(target, "Hart is already halted.");
		if (target->state != TARGET_HALTED) {
			target->state = TARGET_HALTED;
			enum riscv_halt_reason halt_reason = riscv_halt_reason(target);
			if (set_debug_reason(target, halt_reason) != ERROR_OK)
				return ERROR_FAIL;
		}
	} else {
		if (r->halt_go(target) != ERROR_OK)
			return ERROR_FAIL;

		riscv_invalidate_register_cache(target);
	}

	return ERROR_OK;
}

static int halt_go(struct target *target)
{
	RISCV_INFO(r);
	int result;
	if (!r->get_hart_state) {
		struct target_type *tt = get_target_type(target);
		if (!tt)
			return ERROR_FAIL;
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
		if (!tt)
			return ERROR_FAIL;
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
			struct riscv_info *i = riscv_info(t);
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
	LOG_TARGET_DEBUG(target, "");
	struct target_type *tt = get_target_type(target);
	if (!tt)
		return ERROR_FAIL;
	riscv_invalidate_register_cache(target);
	return tt->assert_reset(target);
}

static int riscv_deassert_reset(struct target *target)
{
	LOG_TARGET_DEBUG(target, "");
	struct target_type *tt = get_target_type(target);
	if (!tt)
		return ERROR_FAIL;
	return tt->deassert_reset(target);
}

/* state must be riscv_reg_t state[RISCV_MAX_HWBPS] = {0}; */
static int disable_triggers(struct target *target, riscv_reg_t *state)
{
	RISCV_INFO(r);

	LOG_TARGET_DEBUG(target, "Disabling triggers.");

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	if (r->manual_hwbp_set) {
		/* Look at every trigger that may have been set. */
		riscv_reg_t tselect;
		if (riscv_reg_get(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
			return ERROR_FAIL;
		for (unsigned int t = 0; t < r->trigger_count; t++) {
			if (riscv_reg_set(target, GDB_REGNO_TSELECT, t) != ERROR_OK)
				return ERROR_FAIL;
			riscv_reg_t tdata1;
			if (riscv_reg_get(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
				return ERROR_FAIL;
			if (tdata1 & CSR_TDATA1_DMODE(riscv_xlen(target))) {
				state[t] = tdata1;
				if (riscv_reg_set(target, GDB_REGNO_TDATA1, 0) != ERROR_OK)
					return ERROR_FAIL;
			}
		}
		if (riscv_reg_set(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
			return ERROR_FAIL;

	} else {
		/* Just go through the triggers we manage. */
		struct watchpoint *watchpoint = target->watchpoints;
		int i = 0;
		while (watchpoint) {
			LOG_TARGET_DEBUG(target, "Watchpoint %d: set=%d", i, watchpoint->is_set);
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
		if (riscv_reg_get(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
			return ERROR_FAIL;
		for (unsigned int t = 0; t < r->trigger_count; t++) {
			if (state[t] != 0) {
				if (riscv_reg_set(target, GDB_REGNO_TSELECT, t) != ERROR_OK)
					return ERROR_FAIL;
				if (riscv_reg_set(target, GDB_REGNO_TDATA1, state[t]) != ERROR_OK)
					return ERROR_FAIL;
			}
		}
		if (riscv_reg_set(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
			return ERROR_FAIL;

	} else {
		struct watchpoint *watchpoint = target->watchpoints;
		int i = 0;
		while (watchpoint) {
			LOG_TARGET_DEBUG(target, "Watchpoint %d: cleared=%" PRId64, i, state[i]);
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
	assert(target->state == TARGET_HALTED);
	RISCV_INFO(r);

	if (!current && riscv_reg_set(target, GDB_REGNO_PC, address) != ERROR_OK)
		return ERROR_FAIL;

	if (handle_breakpoints) {
		/* To be able to run off a trigger, we perform a step operation and then
		 * resume. If handle_breakpoints is true then step temporarily disables
		 * pending breakpoints so we can safely perform the step. */
		if (old_or_new_riscv_step_impl(target, current, address, handle_breakpoints,
				false /* callbacks are not called */) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (r->get_hart_state) {
		if (r->resume_prep(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "Mark as prepped.");
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
	assert(target->state == TARGET_HALTED);
	RISCV_INFO(r);
	int result;
	if (!r->get_hart_state) {
		struct target_type *tt = get_target_type(target);
		if (!tt)
			return ERROR_FAIL;
		result = tt->resume(target, current, address, handle_breakpoints,
				debug_execution);
	} else {
		result = riscv_resume_go_all_harts(target);
	}

	return result;
}

static int resume_finish(struct target *target, int debug_execution)
{
	assert(target->state == TARGET_HALTED);
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
static int riscv_resume(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution,
		bool single_hart)
{
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

	LOG_TARGET_DEBUG(target, "current=%s, address=0x%"
				TARGET_PRIxADDR ", handle_breakpoints=%s, debug_exec=%s",
				current ? "true" : "false",
				address,
				handle_breakpoints ? "true" : "false",
				debug_execution ? "true" : "false");

	struct target_list *tlist;
	foreach_smp_target_direction(resume_order == RO_NORMAL, tlist, targets) {
		struct target *t = tlist->target;
		LOG_TARGET_DEBUG(t, "target->state=%s", target_state_name(t));
		if (t->state != TARGET_HALTED)
			LOG_TARGET_DEBUG(t, "skipping this target: target not halted");
		else if (resume_prep(t, current, address, handle_breakpoints,
					debug_execution) != ERROR_OK)
			result = ERROR_FAIL;
	}

	foreach_smp_target_direction(resume_order == RO_NORMAL, tlist, targets) {
		struct target *t = tlist->target;
		struct riscv_info *i = riscv_info(t);
		if (i->prepped) {
			if (resume_go(t, current, address, handle_breakpoints,
						debug_execution) != ERROR_OK)
				result = ERROR_FAIL;
		}
	}

	foreach_smp_target_direction(resume_order == RO_NORMAL, tlist, targets) {
		struct target *t = tlist->target;
		if (t->state == TARGET_HALTED) {
			if (resume_finish(t, debug_execution) != ERROR_OK)
				result = ERROR_FAIL;
		}
	}

	return result;
}

static int riscv_target_resume(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}
	return riscv_resume(target, current, address, handle_breakpoints,
			debug_execution, false);
}

static int riscv_effective_privilege_mode(struct target *target, int *v_mode, int *effective_mode)
{
	riscv_reg_t priv;
	if (riscv_reg_get(target, &priv, GDB_REGNO_PRIV) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read priv register.");
		return ERROR_FAIL;
	}
	*v_mode = get_field(priv, VIRT_PRIV_V);

	riscv_reg_t mstatus;
	if (riscv_reg_get(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read mstatus register.");
		return ERROR_FAIL;
	}

	if (get_field(mstatus, MSTATUS_MPRV))
		*effective_mode = get_field(mstatus, MSTATUS_MPP);
	else
		*effective_mode = get_field(priv, VIRT_PRIV_PRV);

	LOG_TARGET_DEBUG(target, "Effective mode=%d; v=%d", *effective_mode, *v_mode);

	return ERROR_OK;
}

static int riscv_mmu(struct target *target, int *enabled)
{
	*enabled = 0;

	if (!riscv_enable_virt2phys)
		return ERROR_OK;

	/* Don't use MMU in explicit or effective M (machine) mode */
	riscv_reg_t priv;
	if (riscv_reg_get(target, &priv, GDB_REGNO_PRIV) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read priv register.");
		return ERROR_FAIL;
	}

	int effective_mode;
	int v_mode;
	if (riscv_effective_privilege_mode(target, &v_mode, &effective_mode) != ERROR_OK)
		return ERROR_FAIL;

	unsigned int xlen = riscv_xlen(target);

	if (v_mode) {
		/* vsatp and hgatp registers are considered active for the
		 * purposes of the address-translation algorithm unless the
		 * effective privilege mode is U and hstatus.HU=0. */
		if (effective_mode == PRV_U) {
			riscv_reg_t hstatus;
			if (riscv_reg_get(target, &hstatus, GDB_REGNO_HSTATUS) != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to read hstatus register.");
				return ERROR_FAIL;
			}

			if (get_field(hstatus, HSTATUS_HU) == 0)
				/* In hypervisor mode regular satp translation
				 * doesn't happen. */
				return ERROR_OK;

		}

		riscv_reg_t vsatp;
		if (riscv_reg_get(target, &vsatp, GDB_REGNO_VSATP) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to read vsatp register; priv=0x%" PRIx64,
					priv);
			return ERROR_FAIL;
		}
		/* vsatp is identical to satp, so we can use the satp macros. */
		if (RISCV_SATP_MODE(xlen) != SATP_MODE_OFF) {
			LOG_TARGET_DEBUG(target, "VS-stage translation is enabled.");
			*enabled = 1;
			return ERROR_OK;
		}

		riscv_reg_t hgatp;
		if (riscv_reg_get(target, &hgatp, GDB_REGNO_HGATP) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to read hgatp register; priv=0x%" PRIx64,
					priv);
			return ERROR_FAIL;
		}
		if (RISCV_HGATP_MODE(xlen) != HGATP_MODE_OFF) {
			LOG_TARGET_DEBUG(target, "G-stage address translation is enabled.");
			*enabled = 1;
		} else {
			LOG_TARGET_DEBUG(target, "No V-mode address translation enabled.");
		}

		return ERROR_OK;
	}

	/* Don't use MMU in explicit or effective M (machine) mode */
	if (effective_mode == PRV_M) {
		LOG_TARGET_DEBUG(target, "SATP/MMU ignored in Machine mode.");
		return ERROR_OK;
	}

	riscv_reg_t satp;
	if (riscv_reg_get(target, &satp, GDB_REGNO_SATP) != ERROR_OK) {
		LOG_TARGET_DEBUG(target, "Couldn't read SATP.");
		/* If we can't read SATP, then there must not be an MMU. */
		return ERROR_OK;
	}

	if (get_field(satp, RISCV_SATP_MODE(xlen)) == SATP_MODE_OFF) {
		LOG_TARGET_DEBUG(target, "MMU is disabled.");
	} else {
		LOG_TARGET_DEBUG(target, "MMU is enabled.");
		*enabled = 1;
	}

	return ERROR_OK;
}

/* Translate address from virtual to physical, using info and ppn.
 * If extra_info is non-NULL, then translate page table accesses for the primary
 * translation using extra_info and extra_ppn. */
static int riscv_address_translate(struct target *target,
		const virt2phys_info_t *info, target_addr_t ppn,
		const virt2phys_info_t *extra_info, target_addr_t extra_ppn,
		target_addr_t virtual, target_addr_t *physical)
{
	RISCV_INFO(r);
	unsigned int xlen = riscv_xlen(target);

	LOG_TARGET_DEBUG(target, "mode=%s; ppn=0x%" TARGET_PRIxADDR "; virtual=0x%" TARGET_PRIxADDR,
		info->name, ppn, virtual);

	/* verify bits xlen-1:va_bits-1 are all equal */
	assert(xlen >= info->va_bits);
	target_addr_t mask = ((target_addr_t)1 << (xlen - (info->va_bits - 1))) - 1;
	target_addr_t masked_msbs = (virtual >> (info->va_bits - 1)) & mask;
	if (masked_msbs != 0 && masked_msbs != mask) {
		LOG_TARGET_ERROR(target, "Virtual address 0x%" TARGET_PRIxADDR " is not sign-extended "
				"for %s mode.", virtual, info->name);
		return ERROR_FAIL;
	}

	uint64_t pte = 0;
	target_addr_t table_address = ppn << RISCV_PGSHIFT;
	int i = info->level - 1;
	while (i >= 0) {
		uint64_t vpn = virtual >> info->vpn_shift[i];
		vpn &= info->vpn_mask[i];
		target_addr_t pte_address = table_address + (vpn << info->pte_shift);

		if (extra_info) {
			/* Perform extra stage translation. */
			if (riscv_address_translate(target, extra_info, extra_ppn,
						    NULL, 0, pte_address, &pte_address) != ERROR_OK)
				return ERROR_FAIL;
		}

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

		LOG_TARGET_DEBUG(target, "i=%d; PTE @0x%" TARGET_PRIxADDR " = 0x%" PRIx64, i,
				pte_address, pte);

		if (!(pte & PTE_V) || (!(pte & PTE_R) && (pte & PTE_W))) {
			LOG_TARGET_ERROR(target, "invalid PTE @0x%" TARGET_PRIxADDR ": 0x%" PRIx64
					"; mode=%s; i=%d", pte_address, pte, info->name, i);
			return ERROR_FAIL;
		}

		if ((pte & PTE_R) || (pte & PTE_W) || (pte & PTE_X)) /* Found leaf PTE. */
			break;

		i--;
		if (i < 0)
			break;
		ppn = pte >> PTE_PPN_SHIFT;
		table_address = ppn << RISCV_PGSHIFT;
	}

	if (i < 0) {
		LOG_TARGET_ERROR(target, "Couldn't find the PTE.");
		return ERROR_FAIL;
	}

	/* Make sure to clear out the high bits that may be set. */
	*physical = virtual & (((target_addr_t)1 << info->va_bits) - 1);

	while (i < info->level) {
		ppn = pte >> info->pte_ppn_shift[i];
		ppn &= info->pte_ppn_mask[i];
		*physical &= ~(((target_addr_t)info->pa_ppn_mask[i]) <<
				info->pa_ppn_shift[i]);
		*physical |= (ppn << info->pa_ppn_shift[i]);
		i++;
	}
	LOG_TARGET_DEBUG(target, "mode=%s; 0x%" TARGET_PRIxADDR " -> 0x%" TARGET_PRIxADDR,
			 info->name, virtual, *physical);
	return ERROR_OK;
}

/* Virtual to physical translation for hypervisor mode. */
static int riscv_virt2phys_v(struct target *target, target_addr_t virtual, target_addr_t *physical)
{
	riscv_reg_t vsatp;
	if (riscv_reg_get(target, &vsatp, GDB_REGNO_VSATP) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read vsatp register.");
		return ERROR_FAIL;
	}
	/* vsatp is identical to satp, so we can use the satp macros. */
	unsigned int xlen = riscv_xlen(target);
	int vsatp_mode = get_field(vsatp, RISCV_SATP_MODE(xlen));
	LOG_TARGET_DEBUG(target, "VS-stage translation mode: %d", vsatp_mode);
	riscv_reg_t hgatp;
	if (riscv_reg_get(target, &hgatp, GDB_REGNO_HGATP) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read hgatp register.");
		return ERROR_FAIL;
	}
	int hgatp_mode = get_field(hgatp, RISCV_HGATP_MODE(xlen));
	LOG_TARGET_DEBUG(target, "G-stage translation mode: %d", hgatp_mode);

	const virt2phys_info_t *vsatp_info;
	/* VS-stage address translation. */
	switch (vsatp_mode) {
		case SATP_MODE_SV32:
			vsatp_info = &sv32;
			break;
		case SATP_MODE_SV39:
			vsatp_info = &sv39;
			break;
		case SATP_MODE_SV48:
			vsatp_info = &sv48;
			break;
		case SATP_MODE_SV57:
			vsatp_info = &sv57;
			break;
		case SATP_MODE_OFF:
			vsatp_info = NULL;
			LOG_TARGET_DEBUG(target, "vsatp mode is %d. No VS-stage translation. (vsatp: 0x%" PRIx64 ")",
				vsatp_mode, vsatp);
			break;
		default:
			LOG_TARGET_ERROR(target,
				"vsatp mode %d is not supported. (vsatp: 0x%" PRIx64 ")",
				vsatp_mode, vsatp);
			return ERROR_FAIL;
	}

	const virt2phys_info_t *hgatp_info;
	/* G-stage address translation. */
	switch (hgatp_mode) {
		case HGATP_MODE_SV32X4:
			hgatp_info = &sv32x4;
			break;
		case HGATP_MODE_SV39X4:
			hgatp_info = &sv39x4;
			break;
		case HGATP_MODE_SV48X4:
			hgatp_info = &sv48x4;
			break;
		case HGATP_MODE_SV57X4:
			hgatp_info = &sv57x4;
			break;
		case HGATP_MODE_OFF:
			hgatp_info = NULL;
			LOG_TARGET_DEBUG(target, "hgatp mode is %d. No G-stage translation. (hgatp: 0x%" PRIx64 ")",
				hgatp_mode, hgatp);
			break;
		default:
			LOG_TARGET_ERROR(target,
				"hgatp mode %d is not supported. (hgatp: 0x%" PRIx64 ")",
				hgatp_mode, hgatp);
			return ERROR_FAIL;
	}

	/* For any virtual memory access, the original virtual address is
		* converted in the first stage by VS-level address translation,
		* as controlled by the vsatp register, into a guest physical
		* address. */
	target_addr_t guest_physical;
	if (vsatp_info) {
		/* When V=1, memory accesses that would normally bypass
			* address translation are subject to G- stage address
			* translation alone.  This includes memory accesses made
			* in support of VS-stage address translation, such as
			* reads and writes of VS-level page tables. */

		if (riscv_address_translate(target,
				vsatp_info, get_field(vsatp, RISCV_SATP_PPN(xlen)),
				hgatp_info, get_field(hgatp, RISCV_SATP_PPN(xlen)),
				virtual, &guest_physical) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		guest_physical = virtual;
	}

	/* The guest physical address is then converted in the second
		* stage by guest physical address translation, as controlled by
		* the hgatp register, into a supervisor physical address. */
	if (hgatp_info) {
		if (riscv_address_translate(target,
				hgatp_info, get_field(hgatp, RISCV_HGATP_PPN(xlen)),
				NULL, 0,
				guest_physical, physical) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		*physical = guest_physical;
	}

	return ERROR_OK;
}

static int riscv_virt2phys(struct target *target, target_addr_t virtual, target_addr_t *physical)
{
	int enabled;
	if (riscv_mmu(target, &enabled) != ERROR_OK)
		return ERROR_FAIL;
	if (!enabled) {
		*physical = virtual;
		LOG_TARGET_DEBUG(target, "MMU is disabled. 0x%" TARGET_PRIxADDR " -> 0x%" TARGET_PRIxADDR, virtual, *physical);
		return ERROR_OK;
	}

	riscv_reg_t priv;
	if (riscv_reg_get(target, &priv, GDB_REGNO_PRIV) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read priv register.");
		return ERROR_FAIL;
	}

	if (priv & VIRT_PRIV_V)
		return riscv_virt2phys_v(target, virtual, physical);

	riscv_reg_t satp_value;
	if (riscv_reg_get(target, &satp_value, GDB_REGNO_SATP) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read SATP register.");
		return ERROR_FAIL;
	}

	unsigned int xlen = riscv_xlen(target);
	int satp_mode = get_field(satp_value, RISCV_SATP_MODE(xlen));
	const virt2phys_info_t *satp_info;
	switch (satp_mode) {
		case SATP_MODE_SV32:
			satp_info = &sv32;
			break;
		case SATP_MODE_SV39:
			satp_info = &sv39;
			break;
		case SATP_MODE_SV48:
			satp_info = &sv48;
			break;
		case SATP_MODE_SV57:
			satp_info = &sv57;
			break;
		case SATP_MODE_OFF:
			LOG_TARGET_ERROR(target, "No translation or protection."
				      " (satp: 0x%" PRIx64 ")", satp_value);
			return ERROR_FAIL;
		default:
			LOG_TARGET_ERROR(target, "The translation mode is not supported."
				      " (satp: 0x%" PRIx64 ")", satp_value);
			return ERROR_FAIL;
	}

	return riscv_address_translate(target,
			satp_info, get_field(satp_value, RISCV_SATP_PPN(xlen)),
			NULL, 0,
			virtual, physical);
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
		LOG_TARGET_WARNING(target, "0-length read from 0x%" TARGET_PRIxADDR, address);
		return ERROR_OK;
	}

	target_addr_t physical_addr;
	int result = target->type->virt2phys(target, address, &physical_addr);
	if (result != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Address translation failed.");
		return result;
	}

	RISCV_INFO(r);
	return r->read_memory(target, physical_addr, size, count, buffer, size);
}

static int riscv_write_phys_memory(struct target *target, target_addr_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct target_type *tt = get_target_type(target);
	if (!tt)
		return ERROR_FAIL;
	return tt->write_memory(target, phys_address, size, count, buffer);
}

static int riscv_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (count == 0) {
		LOG_TARGET_WARNING(target, "0-length write to 0x%" TARGET_PRIxADDR, address);
		return ERROR_OK;
	}

	target_addr_t physical_addr;
	int result = target->type->virt2phys(target, address, &physical_addr);
	if (result != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Address translation failed.");
		return result;
	}

	struct target_type *tt = get_target_type(target);
	if (!tt)
		return ERROR_FAIL;
	return tt->write_memory(target, physical_addr, size, count, buffer);
}

static const char *riscv_get_gdb_arch(const struct target *target)
{
	switch (riscv_xlen(target)) {
		case 32:
			return "riscv:rv32";
		case 64:
			return "riscv:rv64";
	}
	LOG_TARGET_ERROR(target, "Unsupported xlen: %d", riscv_xlen(target));
	return NULL;
}

static int riscv_get_gdb_reg_list_internal(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class, bool is_read)
{
	LOG_TARGET_DEBUG(target, "reg_class=%d, read=%d", reg_class, is_read);

	if (!target->reg_cache) {
		LOG_TARGET_ERROR(target, "Target not initialized. Return ERROR_FAIL.");
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
			LOG_TARGET_ERROR(target, "Unsupported reg_class: %d", reg_class);
			return ERROR_FAIL;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	for (int i = 0; i < *reg_list_size; i++) {
		assert(!target->reg_cache->reg_list[i].valid ||
				target->reg_cache->reg_list[i].size > 0);
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
		if (is_read &&
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
	assert(target->state == TARGET_HALTED);
	const bool semihosting_active = target->semihosting &&
		target->semihosting->is_active;
	LOG_USER("%s halted due to %s.%s",
			target_name(target),
			debug_reason_name(target),
			semihosting_active ? " Semihosting is active." : "");
	struct target_type *tt = get_target_type(target);
	if (!tt)
		return ERROR_FAIL;
	assert(tt->arch_state);
	return tt->arch_state(target);
}

/* Algorithm must end with a software breakpoint instruction. */
static int riscv_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_params,
		struct reg_param *reg_params, target_addr_t entry_point,
		target_addr_t exit_point, unsigned int timeout_ms, void *arch_info)
{
	RISCV_INFO(info);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted (run target algo)");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Write memory parameters to the target memory */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_OUT ||
				mem_params[i].direction == PARAM_IN_OUT) {
			int retval = target_write_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
			if (retval != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Couldn't write input mem param into the memory, addr=0x%" TARGET_PRIxADDR
					" size=0x%" PRIx32, mem_params[i].address, mem_params[i].size);
				return retval;
			}
		}
	}

	/* Save registers */
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", true);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_TARGET_DEBUG(target, "saved_pc=0x%" PRIx64, saved_pc);

	uint64_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		LOG_TARGET_DEBUG(target, "save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, false);
		if (!r) {
			LOG_TARGET_ERROR(target, "Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_TARGET_ERROR(target, "Register %s is %d bits instead of %d bits.",
					reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_TARGET_ERROR(target, "Only GPRs can be use as argument registers.");
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
	LOG_TARGET_DEBUG(target, "Resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (riscv_resume(target, 0, entry_point, 0, 1, true) != ERROR_OK)
		return ERROR_FAIL;

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_TARGET_ERROR(target, "Algorithm timed out after %" PRId64 " ms.", now - start);
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
				if (riscv_reg_get(target, &reg_value, regno) != ERROR_OK)
					break;

				LOG_TARGET_ERROR(target, "%s = 0x%" PRIx64, riscv_reg_gdb_regno_name(target, regno), reg_value);
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
		LOG_TARGET_ERROR(target, "PC ended up at 0x%" PRIx64 " instead of 0x%"
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
				LOG_TARGET_ERROR(target, "get(%s) failed", r->name);
				return ERROR_FAIL;
			}
			buf_cpy(r->value, reg_params[i].value, reg_params[i].size);
		}
		LOG_TARGET_DEBUG(target, "restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, false);
		buf_set_u64(buf, 0, info->xlen, saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "set(%s) failed", r->name);
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
				LOG_TARGET_ERROR(target, "Couldn't read output mem param from the memory, "
					"addr=0x%" TARGET_PRIxADDR " size=0x%" PRIx32,
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

	LOG_TARGET_DEBUG(target, "address=0x%" TARGET_PRIxADDR "; count=0x%" PRIx32, address, count);

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
		LOG_TARGET_ERROR(target, "Failed to write code to " TARGET_ADDR_FMT ": %d",
				crc_algorithm->address, retval);
		target_free_working_area(target, crc_algorithm);
		return retval;
	}

	init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, address);
	buf_set_u64(reg_params[1].value, 0, xlen, count);

	/* 20 second timeout/megabyte */
	unsigned int timeout = 20000 * (1 + (count / (1024 * 1024)));

	retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
			crc_algorithm->address,
			0,	/* Leave exit point unspecified because we don't know. */
			timeout, NULL);

	if (retval == ERROR_OK)
		*checksum = buf_get_u32(reg_params[0].value, 0, 32);
	else
		LOG_TARGET_ERROR(target, "Error executing RISC-V CRC algorithm.");

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	target_free_working_area(target, crc_algorithm);

	LOG_TARGET_DEBUG(target, "checksum=0x%" PRIx32 ", result=%d", *checksum, retval);

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
		if (riscv_reg_flush_all(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (target->state == TARGET_UNKNOWN || state != previous_riscv_state) {
		switch (state) {
			case RISCV_STATE_HALTED:
				if (previous_riscv_state == RISCV_STATE_UNAVAILABLE)
					LOG_TARGET_INFO(target, "became available (halted)");

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
						case SEMIHOSTING_NONE:
							break;
						case SEMIHOSTING_WAITING:
							/* This hart should remain halted. */
							*next_action = RPH_REMAIN_HALTED;
							break;
						case SEMIHOSTING_HANDLED:
							/* This hart should be resumed, along with any other
							* harts that halted due to haltgroups. */
							*next_action = RPH_RESUME;
							return ERROR_OK;
						case SEMIHOSTING_ERROR:
							return retval;
					}
				}

				if (r->handle_became_halted &&
						r->handle_became_halted(target, previous_riscv_state) != ERROR_OK)
					return ERROR_FAIL;

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
				if (previous_riscv_state == RISCV_STATE_UNAVAILABLE)
					LOG_TARGET_INFO(target, "became available (running)");

				LOG_TARGET_DEBUG(target, "  triggered running");
				target->state = TARGET_RUNNING;
				target->debug_reason = DBG_REASON_NOTHALTED;
				if (r->handle_became_running &&
						r->handle_became_running(target, previous_riscv_state) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case RISCV_STATE_UNAVAILABLE:
				LOG_TARGET_DEBUG(target, "  became unavailable");
				LOG_TARGET_INFO(target, "became unavailable.");
				target->state = TARGET_UNAVAILABLE;
				if (r->handle_became_unavailable &&
						r->handle_became_unavailable(target, previous_riscv_state) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case RISCV_STATE_NON_EXISTENT:
				LOG_TARGET_ERROR(target, "Hart is non-existent!");
				target->state = TARGET_UNAVAILABLE;
				break;
		}
	}

	return ERROR_OK;
}

static int sample_memory(struct target *target)
{
	RISCV_INFO(r);

	if (!r->sample_buf.buf || !r->sample_config.enabled)
		return ERROR_OK;

	LOG_TARGET_DEBUG(target, "buf used/size: %d/%d", r->sample_buf.used, r->sample_buf.size);

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
		LOG_TARGET_INFO(target, "Turning off memory sampling because it failed.");
		r->sample_config.enabled = false;
	}
	return result;
}

/*** OpenOCD Interface ***/
int riscv_openocd_poll(struct target *target)
{
	LOG_TARGET_DEBUG(target, "Polling all harts.");

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

	unsigned int should_remain_halted = 0;
	unsigned int should_resume = 0;
	unsigned int halted = 0;
	unsigned int running = 0;
	struct target_list *entry;
	foreach_smp_target(entry, targets) {
		struct target *t = entry->target;
		struct riscv_info *info = riscv_info(t);

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

	LOG_TARGET_DEBUG(target, "should_remain_halted=%d, should_resume=%d",
				should_remain_halted, should_resume);
	if (should_remain_halted && should_resume) {
		LOG_TARGET_WARNING(target, "%d harts should remain halted, and %d should resume.",
					should_remain_halted, should_resume);
	}
	if (should_remain_halted) {
		LOG_TARGET_DEBUG(target, "halt all; should_remain_halted=%d",
			should_remain_halted);
		riscv_halt(target);
	} else if (should_resume) {
		LOG_TARGET_DEBUG(target, "resume all");
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
			struct riscv_info *info = riscv_info(t);
			if (info->halted_needs_event_callback) {
				target_call_event_callbacks(t, info->halted_callback_event);
				info->halted_needs_event_callback = false;
			}
		}
	}

	/* Call tick() for every hart. What happens in tick() is opaque to this
	 * layer. The reason it's outside the previous loop is that at this point
	 * the state of every hart has settled, so any side effects happening in
	 * tick() won't affect the delicate poll() code. */
	foreach_smp_target(entry, targets) {
		struct target *t = entry->target;
		struct riscv_info *info = riscv_info(t);
		if (info->tick && info->tick(t) != ERROR_OK)
			return ERROR_FAIL;
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

static int riscv_openocd_step_impl(struct target *target, int current,
	target_addr_t address, int handle_breakpoints, int handle_callbacks)
{
	LOG_TARGET_DEBUG(target, "stepping hart");

	if (!current) {
		if (riscv_reg_set(target, GDB_REGNO_PC, address) != ERROR_OK)
			return ERROR_FAIL;
	}

	struct breakpoint *breakpoint = NULL;
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		if (current) {
			if (riscv_reg_get(target, &address, GDB_REGNO_PC) != ERROR_OK)
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
			LOG_TARGET_ERROR(target, "Unable to disable interrupts.");
			goto _exit;
		}
	}

	if (riscv_step_rtos_hart(target) != ERROR_OK) {
		success = false;
		LOG_TARGET_ERROR(target, "Unable to step rtos hart.");
	}

	register_cache_invalidate(target->reg_cache);

	if (info->isrmask_mode == RISCV_ISRMASK_STEPONLY)
		if (riscv_interrupts_restore(target, current_mstatus) != ERROR_OK) {
			success = false;
			LOG_TARGET_ERROR(target, "Unable to restore interrupts.");
		}

_exit:
	if (enable_triggers(target, trigger_state) != ERROR_OK) {
		success = false;
		LOG_TARGET_ERROR(target, "Unable to enable triggers.");
	}

	if (breakpoint && (riscv_add_breakpoint(target, breakpoint) != ERROR_OK)) {
		success = false;
		LOG_TARGET_ERROR(target, "Unable to restore the disabled breakpoint.");
	}

	if (success) {
		target->state = TARGET_RUNNING;
		if (handle_callbacks)
			target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_SINGLESTEP;
		if (handle_callbacks)
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}

	return success ? ERROR_OK : ERROR_FAIL;
}

int riscv_openocd_step(struct target *target, int current,
	target_addr_t address, int handle_breakpoints)
{
	return riscv_openocd_step_impl(target, current, address, handle_breakpoints,
		true /* handle_callbacks */);
}

/* Command Handlers */
COMMAND_HANDLER(riscv_set_command_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int timeout = atoi(CMD_ARGV[0]);
	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	riscv_command_timeout_sec_value = timeout;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_reset_timeout_sec)
{
	LOG_WARNING("The command 'riscv set_reset_timeout_sec' is deprecated! Please, use 'riscv set_command_timeout_sec'.");
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter.");
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

static int parse_ranges(struct list_head *ranges, const char *tcl_arg, const char *reg_type, unsigned int max_val)
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
		LOG_ERROR("Command expects parameters.");
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
		LOG_ERROR("Command expects parameters.");
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

COMMAND_HANDLER(riscv_hide_csrs)
{
	if (CMD_ARGC == 0) {
		LOG_ERROR("Command expects parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(info);
	int ret = ERROR_OK;

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		ret = parse_ranges(&info->hide_csr, CMD_ARGV[i], "csr", 0xfff);
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
		LOG_ERROR("Command takes at most one parameter.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}

	RISCV_INFO(r);
	if (!r) {
		LOG_TARGET_ERROR(target, "riscv_info is NULL!");
		return ERROR_FAIL;
	}

	if (r->authdata_read) {
		uint32_t value;
		if (r->authdata_read(target, &value, index) != ERROR_OK)
			return ERROR_FAIL;
		command_print_sameline(CMD, "0x%08" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_TARGET_ERROR(target, "authdata_read is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_authdata_write)
{
	uint32_t value;
	unsigned int index = 0;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], value);
	} else {
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], index);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (!r->authdata_write) {
		LOG_TARGET_ERROR(target, "authdata_write is not implemented for this target.");
		return ERROR_FAIL;
	}

	return r->authdata_write(target, value, index);
}

uint32_t riscv_get_dmi_address(const struct target *target, uint32_t dm_address)
{
	assert(target);
	RISCV_INFO(r);
	if (!r || !r->get_dmi_address)
		return dm_address;
	return r->get_dmi_address(target, dm_address);
}

static int riscv_dmi_read(struct target *target, uint32_t *value, uint32_t address)
{
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}
	RISCV_INFO(r);
	if (!r) {
		LOG_TARGET_ERROR(target, "riscv_info is NULL!");
		return ERROR_FAIL;
	}
	if (!r->dmi_read) {
		LOG_TARGET_ERROR(target, "dmi_read is not implemented.");
		return ERROR_FAIL;
	}
	return r->dmi_read(target, value, address);
}

static int riscv_dmi_write(struct target *target, uint32_t dmi_address, uint32_t value)
{
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}
	RISCV_INFO(r);
	if (!r) {
		LOG_TARGET_ERROR(target, "riscv_info is NULL!");
		return ERROR_FAIL;
	}
	if (!r->dmi_write) {
		LOG_TARGET_ERROR(target, "dmi_write is not implemented.");
		return ERROR_FAIL;
	}
	const int result = r->dmi_write(target, dmi_address, value);
	/* Invalidate our cached progbuf copy:
	 * - if the user tinkered directly with a progbuf register
	 * - if debug module was reset, in which case progbuf registers
	 * may not retain their value.
	 * FIXME: If there are multiple DMs on a single TAP, it is possible to
	 * clobber progbuf or reset the DM of another target.
	 */
	const bool progbuf_touched =
			(dmi_address >= riscv_get_dmi_address(target, DM_PROGBUF0) &&
			dmi_address <= riscv_get_dmi_address(target, DM_PROGBUF15));
	const bool dm_deactivated =
			(dmi_address == riscv_get_dmi_address(target, DM_DMCONTROL) &&
			(value & DM_DMCONTROL_DMACTIVE) == 0);
	if (progbuf_touched || dm_deactivated) {
		if (r->invalidate_cached_progbuf) {
			/* Here the return value of invalidate_cached_progbuf()
			 * is ignored. It is okay to do so for now, since the
			 * only case an error is returned is a failure to
			 * assign a DM to the target, which would have already
			 * caused an error during dmi_write().
			 * FIXME: invalidate_cached_progbuf() should be void.
			 */
			r->invalidate_cached_progbuf(target);
		} else {
			LOG_TARGET_DEBUG(target,
					"invalidate_cached_progbuf() is not implemented.");
		}
	}
	return result;
}

COMMAND_HANDLER(handle_riscv_dmi_read)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t dmi_address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], dmi_address);

	struct target * const target = get_current_target(CMD_CTX);
	uint32_t value;
	const int result = riscv_dmi_read(target, &value, dmi_address);
	if (result == ERROR_OK)
		command_print(CMD, "0x%" PRIx32, value);
	return result;
}

COMMAND_HANDLER(handle_riscv_dmi_write)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t dmi_address, value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], dmi_address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct target * const target = get_current_target(CMD_CTX);
	return riscv_dmi_write(target, dmi_address, value);
}

COMMAND_HANDLER(handle_riscv_dm_read)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t dm_address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], dm_address);

	struct target * const target = get_current_target(CMD_CTX);
	uint32_t value;
	const int result = riscv_dmi_read(target, &value,
			riscv_get_dmi_address(target, dm_address));
	if (result == ERROR_OK)
		command_print(CMD, "0x%" PRIx32, value);
	return result;
}

COMMAND_HANDLER(handle_riscv_dm_write)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t dm_address, value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], dm_address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct target * const target = get_current_target(CMD_CTX);
	return riscv_dmi_write(target, riscv_get_dmi_address(target, dm_address),
					value);
}

COMMAND_HANDLER(riscv_reset_delays)
{
	int wait = 0;

	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

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
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (CMD_ARGC == 0) {
		command_print(CMD, "riscv_ebreakm enabled: %s", r->riscv_ebreakm ? "on" : "off");
		return ERROR_OK;
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_ON_OFF(CMD_ARGV[0], r->riscv_ebreakm);
		return ERROR_OK;
	}

	LOG_ERROR("Command takes 0 or 1 parameters");
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(riscv_set_ebreaks)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (CMD_ARGC == 0) {
		command_print(CMD, "riscv_ebreaks enabled: %s", r->riscv_ebreaks ? "on" : "off");
		return ERROR_OK;
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_ON_OFF(CMD_ARGV[0], r->riscv_ebreaks);
		return ERROR_OK;
	}

	LOG_ERROR("Command takes 0 or 1 parameters");
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(riscv_set_ebreaku)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (CMD_ARGC == 0) {
		command_print(CMD, "riscv_ebreaku enabled: %s", r->riscv_ebreaku ? "on" : "off");
		return ERROR_OK;
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_ON_OFF(CMD_ARGV[0], r->riscv_ebreaku);
		return ERROR_OK;
	}

	LOG_ERROR("Command takes 0 or 1 parameters");
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HELPER(riscv_clear_trigger, int trigger_id, const char *name)
{
	struct target *target = get_current_target(CMD_CTX);
	if (CMD_ARGC != 1) {
		LOG_ERROR("clear command takes no extra arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (find_first_trigger_by_id(target, trigger_id) < 0) {
		LOG_TARGET_ERROR(target, "No %s is set. Nothing to clear.", name);
		return ERROR_FAIL;
	}
	return remove_trigger(target, trigger_id);
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
		return riscv_clear_trigger(CMD, ITRIGGER_UNIQUE_ID, "itrigger");

	} else {
		LOG_ERROR("First argument must be either 'set' or 'clear'.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_icount)
{
	if (CMD_ARGC < 1) {
		LOG_ERROR("Command takes at least 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	const int ICOUNT_UNIQUE_ID = -CSR_TDATA1_TYPE_ICOUNT;

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	if (!strcmp(CMD_ARGV[0], "set")) {
		if (find_first_trigger_by_id(target, ICOUNT_UNIQUE_ID) >= 0) {
			LOG_TARGET_ERROR(target, "An icount trigger is already set, and OpenOCD "
				  "doesn't support setting more than one at a time.");
			return ERROR_FAIL;
		}
		bool vs = false;
		bool vu = false;
		bool m = false;
		bool s = false;
		bool u = false;
		bool pending = false;
		unsigned int count = 0;

		for (unsigned int i = 1; i < CMD_ARGC; i++) {
			if (!strcmp(CMD_ARGV[i], "vs"))
				vs = true;
			else if (!strcmp(CMD_ARGV[i], "vu"))
				vu = true;
			else if (!strcmp(CMD_ARGV[i], "pending"))
				pending = true;
			else if (!strcmp(CMD_ARGV[i], "m"))
				m = true;
			else if (!strcmp(CMD_ARGV[i], "s"))
				s = true;
			else if (!strcmp(CMD_ARGV[i], "u"))
				u = true;
			else
				COMMAND_PARSE_NUMBER(uint, CMD_ARGV[i], count);
		}
		if (count == 0) {
			LOG_ERROR("Doesn't make sense to set icount trigger with "
				  "count=0.");
			return ERROR_FAIL;
		} else if (!vs && !vu && !m && !s && !u) {
			LOG_ERROR("Doesn't make sense to set itrigger without at "
				  "least one of vs, vu, m, s, or u.");
			return ERROR_FAIL;
		}
		int result = maybe_add_trigger_t3(target, vs, vu, m, s, u, pending, count, ICOUNT_UNIQUE_ID);
		if (result != ERROR_OK)
			LOG_TARGET_ERROR(target, "Failed to set requested icount trigger.");
		return result;

	} else if (!strcmp(CMD_ARGV[0], "clear")) {
		return riscv_clear_trigger(CMD, ICOUNT_UNIQUE_ID, "icount trigger");

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
		return riscv_clear_trigger(CMD, ETRIGGER_UNIQUE_ID, "etrigger");

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
		for (unsigned int i = 0; i < ARRAY_SIZE(r->sample_config.bucket); i++) {
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
		LOG_TARGET_ERROR(target, "Max bucket number is %zd.", ARRAY_SIZE(r->sample_config.bucket));
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
				LOG_TARGET_ERROR(target, "Only 4-byte and 8-byte sizes are supported.");
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
			LOG_TARGET_ERROR(target, "Failed base64 encode!");
			result = ERROR_FAIL;
			goto error;
		}
		command_print(CMD, "%s", encoded);
		free(encoded);
	} else {
		unsigned int i = 0;
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
					LOG_TARGET_ERROR(target, "Found invalid size in bucket %d: %d", command,
							  r->sample_config.bucket[command].size_bytes);
					result = ERROR_FAIL;
					goto error;
				}
			} else {
				LOG_TARGET_ERROR(target, "Found invalid command byte in sample buf: 0x%2x at offset 0x%x",
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

static COMMAND_HELPER(riscv_print_info_line_if_available, const char *section,
		const char *key, unsigned int value, bool is_available)
{
	char full_key[80];
	snprintf(full_key, sizeof(full_key), "%s.%s", section, key);
	if (is_available)
		command_print(CMD, "%-21s %3d", full_key, value);
	else
		command_print(CMD, "%-21s unavailable", full_key);
	return 0;
}

COMMAND_HELPER(riscv_print_info_line, const char *section, const char *key,
			   unsigned int value)
{
	return CALL_COMMAND_HANDLER(riscv_print_info_line_if_available, section,
			key, value, /*is_available*/ true);
}

COMMAND_HANDLER(handle_info)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	/* This output format can be fed directly into TCL's "array set". */

	riscv_print_info_line(CMD, "hart", "xlen", riscv_xlen(target));

	const bool trigger_count_available =
		riscv_enumerate_triggers(target) == ERROR_OK;
	riscv_print_info_line_if_available(CMD, "hart", "trigger_count",
				r->trigger_count, trigger_count_available);
	if (r->print_info)
		return CALL_COMMAND_HANDLER(r->print_info, target);

	return 0;
}

COMMAND_HANDLER(riscv_exec_progbuf)
{
	if (CMD_ARGC < 1 || CMD_ARGC > 16) {
		LOG_ERROR("Command 'exec_progbuf' takes 1 to 16 arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);

	RISCV_INFO(r);
	if (r->dtm_version != DTM_DTMCS_VERSION_1_0) {
		LOG_TARGET_ERROR(target, "exec_progbuf: Program buffer is "
				"only supported on v0.13 or v1.0 targets.");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "exec_progbuf: Can't execute "
				"program buffer, target not halted.");
		return ERROR_FAIL;
	}

	if (riscv_progbuf_size(target) == 0) {
		LOG_TARGET_ERROR(target, "exec_progbuf: Program buffer not implemented "
				"in the target.");
		return ERROR_FAIL;
	}

	struct riscv_program prog;
	riscv_program_init(&prog, target);

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		riscv_insn_t instr;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i], instr);
		if (riscv_program_insert(&prog, instr) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (riscv_reg_flush_all(target) != ERROR_OK)
		return ERROR_FAIL;
	int error = riscv_program_exec(&prog, target);
	riscv_invalidate_register_cache(target);

	if (error != ERROR_OK) {
		LOG_TARGET_ERROR(target, "exec_progbuf: Program buffer execution failed.");
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "exec_progbuf: Program buffer execution successful.");

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_enable_trigger_feature)
{
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	if (CMD_ARGC == 2) {
		bool enable_for_wp = true;

		if (!strcmp(CMD_ARGV[1], "wp"))
			enable_for_wp = true;
		else if (!strcmp(CMD_ARGV[1], "none"))
			enable_for_wp = false;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;

		if (!strcmp(CMD_ARGV[0], "all")) {
			r->wp_allow_equality_match_trigger = enable_for_wp;
			r->wp_allow_napot_trigger = enable_for_wp;
			r->wp_allow_ge_lt_trigger = enable_for_wp;
		} else if (!strcmp(CMD_ARGV[0], "eq")) {
			r->wp_allow_equality_match_trigger = enable_for_wp;
		} else if (!strcmp(CMD_ARGV[0], "napot")) {
			r->wp_allow_napot_trigger = enable_for_wp;
		} else if (!strcmp(CMD_ARGV[0], "ge_lt")) {
			r->wp_allow_ge_lt_trigger = enable_for_wp;
		} else {
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "Triggers feature configuration:\n"
			   "Equality match trigger: for wp (%s)\n"
			   "NAPOT trigger: for wp (%s)\n"
			   "ge-lt chained triggers: for wp (%s)",
			   r->wp_allow_equality_match_trigger ? "enabled" : "disabled",
			   r->wp_allow_napot_trigger ? "enabled" : "disabled",
			   r->wp_allow_ge_lt_trigger ? "enabled" : "disabled");

	return ERROR_OK;
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
		.help = "DEPRECATED. Use 'riscv set_command_timeout_sec' instead."
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
		.name = "hide_csrs",
		.handler = riscv_hide_csrs,
		.mode = COMMAND_CONFIG,
		.usage = "{n0|n-m0}[,n1|n-m1]......",
		.help = "Configure a list of inclusive ranges for CSRs to hide from gdb. "
			"Hidden registers are still available, but are not listed in "
			"gdb target description and `reg` command output. "
			"This must be executed before `init`."
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
		.handler = handle_riscv_dmi_read,
		.mode = COMMAND_ANY,
		.usage = "address",
		.help = "Read and return 32-bit value from the given address on the "
				"RISC-V DMI bus."
	},
	{
		.name = "dmi_write",
		.handler = handle_riscv_dmi_write,
		.mode = COMMAND_ANY,
		.usage = "address value",
		.help = "Write a 32-bit value to the given address on the RISC-V DMI bus."
	},
	{
		.name = "dm_read",
		.handler = handle_riscv_dm_read,
		.mode = COMMAND_ANY,
		.usage = "reg_address",
		.help = "Read and return 32-bit value from a debug module's register "
				"at reg_address."
	},
	{
		.name = "dm_write",
		.handler = handle_riscv_dm_write,
		.mode = COMMAND_ANY,
		.usage = "reg_address value",
		.help = "Write a 32-bit value to the debug module's register at "
				"reg_address."
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
		.usage = "[on|off]",
		.help = "Control dcsr.ebreakm. When off, M-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	{
		.name = "set_ebreaks",
		.handler = riscv_set_ebreaks,
		.mode = COMMAND_ANY,
		.usage = "[on|off]",
		.help = "Control dcsr.ebreaks. When off, S-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	{
		.name = "set_ebreaku",
		.handler = riscv_set_ebreaku,
		.mode = COMMAND_ANY,
		.usage = "[on|off]",
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
		.name = "icount",
		.handler = riscv_icount,
		.mode = COMMAND_EXEC,
		.usage = "set [vs] [vu] [m] [s] [u] [pending] <count>|clear",
		.help = "Set or clear a single instruction count trigger."
	},
	{
		.name = "itrigger",
		.handler = riscv_itrigger,
		.mode = COMMAND_EXEC,
		.usage = "set [vs] [vu] [nmi] [m] [s] [u] <mie_bits>|clear",
		.help = "Set or clear a single interrupt trigger."
	},
	{
		.name = "exec_progbuf",
		.handler = riscv_exec_progbuf,
		.mode = COMMAND_EXEC,
		.usage = "instr1 [instr2 [... instr16]]",
		.help = "Execute a sequence of 32-bit instructions using the program buffer. "
			"The final ebreak instruction is added automatically, if needed."
	},
	{
		.name = "set_enable_trigger_feature",
		.handler = riscv_set_enable_trigger_feature,
		.mode = COMMAND_ANY,
		.usage = "[('eq'|'napot'|'ge_lt'|'all') ('wp'|'none')]",
		.help = "Control whether OpenOCD is allowed to use certain RISC-V trigger features for watchpoints."
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

static const struct command_registration riscv_command_handlers[] = {
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
	{
		.chain = smp_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

static unsigned int riscv_xlen_nonconst(struct target *target)
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

/* Initializes the shared RISC-V structure. */
static void riscv_info_init(struct target *target, struct riscv_info *r)
{
	memset(r, 0, sizeof(*r));

	r->common_magic = RISCV_COMMON_MAGIC;

	r->dtm_version = DTM_DTMCS_VERSION_UNKNOWN;
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
	INIT_LIST_HEAD(&r->hide_csr);

	r->vsew64_supported = YNM_MAYBE;

	r->riscv_ebreakm = true;
	r->riscv_ebreaks = true;
	r->riscv_ebreaku = true;

	r->wp_allow_equality_match_trigger = true;
	r->wp_allow_ge_lt_trigger = true;
	r->wp_allow_napot_trigger = true;
}

static int riscv_resume_go_all_harts(struct target *target)
{
	RISCV_INFO(r);

	LOG_TARGET_DEBUG(target, "Resuming hart, state=%d.", target->state);
	if (target->state == TARGET_HALTED) {
		if (r->resume_go(target) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		LOG_TARGET_DEBUG(target, "Hart requested resume, but was already resumed.");
	}

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

int riscv_interrupts_disable(struct target *target, uint64_t irq_mask, uint64_t *old_mstatus)
{
	LOG_TARGET_DEBUG(target, "Disabling interrupts.");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", true);
	if (!reg_mstatus) {
		LOG_TARGET_ERROR(target, "Couldn't find mstatus!");
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
	LOG_TARGET_DEBUG(target, "Restoring interrupts.");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", true);
	if (!reg_mstatus) {
		LOG_TARGET_ERROR(target, "Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	RISCV_INFO(info);
	uint8_t mstatus_bytes[8];
	buf_set_u64(mstatus_bytes, 0, info->xlen, old_mstatus);
	return reg_mstatus->type->set(reg_mstatus, mstatus_bytes);
}

static int riscv_step_rtos_hart(struct target *target)
{
	RISCV_INFO(r);
	LOG_TARGET_DEBUG(target, "Stepping.");

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Hart isn't halted before single step!");
		return ERROR_FAIL;
	}
	r->on_step(target);
	if (r->step_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Hart was not halted after single step!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

bool riscv_supports_extension(const struct target *target, char letter)
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

unsigned int riscv_vlenb(const struct target *target)
{
	RISCV_INFO(r);
	return r->vlenb;
}

static void riscv_invalidate_register_cache(struct target *target)
{
	/* Do not invalidate the register cache if it is not yet set up
	 * (e.g. when the target failed to get examined). */
	if (!target->reg_cache)
		return;

	LOG_TARGET_DEBUG(target, "Invalidating register cache.");
	register_cache_invalidate(target->reg_cache);
}

int riscv_get_hart_state(struct target *target, enum riscv_hart_state *state)
{
	RISCV_INFO(r);
	assert(r->get_hart_state);
	return r->get_hart_state(target, state);
}

static enum riscv_halt_reason riscv_halt_reason(struct target *target)
{
	RISCV_INFO(r);
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Hart is not halted!");
		return RISCV_HALT_UNKNOWN;
	}
	return r->halt_reason(target);
}

size_t riscv_progbuf_size(struct target *target)
{
	RISCV_INFO(r);
	return r->progbuf_size;
}

int riscv_write_progbuf(struct target *target, int index, riscv_insn_t insn)
{
	RISCV_INFO(r);
	r->write_progbuf(target, index, insn);
	return ERROR_OK;
}

riscv_insn_t riscv_read_progbuf(struct target *target, int index)
{
	RISCV_INFO(r);
	return r->read_progbuf(target, index);
}

int riscv_execute_progbuf(struct target *target, uint32_t *cmderr)
{
	RISCV_INFO(r);
	return r->execute_progbuf(target, cmderr);
}

void riscv_fill_dmi_write(struct target *target, char *buf, uint64_t a, uint32_t d)
{
	RISCV_INFO(r);
	r->fill_dmi_write(target, buf, a, d);
}

void riscv_fill_dmi_read(struct target *target, char *buf, uint64_t a)
{
	RISCV_INFO(r);
	r->fill_dmi_read(target, buf, a);
}

void riscv_fill_dm_nop(struct target *target, char *buf)
{
	RISCV_INFO(r);
	r->fill_dm_nop(target, buf);
}

int riscv_get_dmi_scan_length(struct target *target)
{
	RISCV_INFO(r);
	return r->get_dmi_scan_length(target);
}

static int check_if_trigger_exists(struct target *target, unsigned int index)
{
	/* If we can't write tselect, then this hart does not support triggers. */
	if (riscv_reg_set(target, GDB_REGNO_TSELECT, index) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	riscv_reg_t tselect_rb;
	if (riscv_reg_get(target, &tselect_rb, GDB_REGNO_TSELECT) != ERROR_OK)
		return ERROR_FAIL;
	/* Mask off the top bit, which is used as tdrmode in legacy RISC-V Debug Spec
	 * (old revisions of v0.11 spec). */
	tselect_rb &= ~(1ULL << (riscv_xlen(target) - 1));
	if (tselect_rb != index)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	return ERROR_OK;
}

/**
 * This function reads `tinfo` or `tdata1`, when reading `tinfo` fails,
 * to determine trigger types supported by a trigger.
 * It is assumed that the trigger is already selected via writing `tselect`.
 */
static int get_trigger_types(struct target *target, unsigned int *trigger_tinfo,
		riscv_reg_t tdata1)
{
	assert(trigger_tinfo);
	riscv_reg_t tinfo;
	if (riscv_reg_get(target, &tinfo, GDB_REGNO_TINFO) == ERROR_OK) {
		/* tinfo.INFO == 1: trigger doesn’t exist
		 * tinfo == 0 or tinfo.INFO != 1 and tinfo LSB is set: invalid tinfo */
		if (tinfo == 0 || tinfo & 0x1)
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		*trigger_tinfo = tinfo;
		return ERROR_OK;
	}
	const unsigned int type = get_field(tdata1, CSR_TDATA1_TYPE(riscv_xlen(target)));
	if (type == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	*trigger_tinfo = 1 << type;
	return ERROR_OK;
}

static int disable_trigger_if_dmode(struct target *target, riscv_reg_t tdata1)
{
	bool dmode_is_set = false;
	switch (get_field(tdata1, CSR_TDATA1_TYPE(riscv_xlen(target)))) {
		case CSR_TDATA1_TYPE_LEGACY:
			/* On these older cores we don't support software using
			 * triggers. */
			dmode_is_set = true;
			break;
		case CSR_TDATA1_TYPE_MCONTROL:
			dmode_is_set = tdata1 & CSR_MCONTROL_DMODE(riscv_xlen(target));
			break;
		case CSR_TDATA1_TYPE_MCONTROL6:
			dmode_is_set = tdata1 & CSR_MCONTROL6_DMODE(riscv_xlen(target));
			break;
		case CSR_TDATA1_TYPE_ICOUNT:
			dmode_is_set = tdata1 & CSR_ICOUNT_DMODE(riscv_xlen(target));
			break;
		case CSR_TDATA1_TYPE_ITRIGGER:
			dmode_is_set = tdata1 & CSR_ITRIGGER_DMODE(riscv_xlen(target));
			break;
		case CSR_TDATA1_TYPE_ETRIGGER:
			dmode_is_set = tdata1 & CSR_ETRIGGER_DMODE(riscv_xlen(target));
			break;
	}
	if (!dmode_is_set)
		/* Nothing to do */
		return ERROR_OK;
	return riscv_reg_set(target, GDB_REGNO_TDATA1, 0);
}

/**
 * Count triggers, and initialize trigger_count for each hart.
 * trigger_count is initialized even if this function fails to discover
 * something.
 * Disable any hardware triggers that have dmode set. We can't have set them
 * ourselves. Maybe they're left over from some killed debug session.
 */
int riscv_enumerate_triggers(struct target *target)
{
	RISCV_INFO(r);

	if (r->triggers_enumerated)
		return ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Unable to enumerate triggers: target not halted.");
		return ERROR_FAIL;
	}

	riscv_reg_t orig_tselect;
	int result = riscv_reg_get(target, &orig_tselect, GDB_REGNO_TSELECT);
	/* If tselect is not readable, the trigger module is likely not
	 * implemented. */
	if (result != ERROR_OK) {
		LOG_TARGET_INFO(target, "Cannot access tselect register. "
				"Assuming that triggers are not implemented.");
		r->triggers_enumerated = true;
		r->trigger_count = 0;
		return ERROR_OK;
	}

	unsigned int t = 0;
	for (; t < ARRAY_SIZE(r->trigger_tinfo); ++t) {
		result = check_if_trigger_exists(target, t);
		if (result == ERROR_FAIL)
			return ERROR_FAIL;
		if (result == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			break;

		riscv_reg_t tdata1;
		if (riscv_reg_get(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
			return ERROR_FAIL;

		result = get_trigger_types(target, &r->trigger_tinfo[t], tdata1);
		if (result == ERROR_FAIL)
			return ERROR_FAIL;
		if (result == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			break;

		LOG_TARGET_DEBUG(target, "Trigger %u: supported types (mask) = 0x%08x",
				t, r->trigger_tinfo[t]);

		if (disable_trigger_if_dmode(target, tdata1) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (riscv_reg_set(target, GDB_REGNO_TSELECT, orig_tselect) != ERROR_OK)
		return ERROR_FAIL;

	r->triggers_enumerated = true;
	r->trigger_count = t;
	LOG_TARGET_INFO(target, "Found %d triggers", r->trigger_count);
	create_wp_trigger_cache(target);
	return ERROR_OK;
}

void riscv_add_bscan_tunneled_scan(struct target *target, const struct scan_field *field,
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

/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Generic Xtensa debug module API for OpenOCD                           *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "xtensa_debug_module.h"

#define TAPINS_PWRCTL           0x08
#define TAPINS_PWRSTAT          0x09
#define TAPINS_NARSEL           0x1C
#define TAPINS_IDCODE           0x1E
#define TAPINS_BYPASS           0x1F

#define TAPINS_PWRCTL_LEN       8
#define TAPINS_PWRSTAT_LEN      8
#define TAPINS_NARSEL_ADRLEN    8
#define TAPINS_NARSEL_DATALEN   32
#define TAPINS_IDCODE_LEN       32
#define TAPINS_BYPASS_LEN       1


static void xtensa_dm_add_set_ir(struct xtensa_debug_module *dm, uint8_t value)
{
	struct scan_field field;
	uint8_t t[4] = { 0 };

	memset(&field, 0, sizeof(field));
	field.num_bits = dm->tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, value);
	jtag_add_ir_scan(dm->tap, &field, TAP_IDLE);
}

static void xtensa_dm_add_dr_scan(struct xtensa_debug_module *dm,
	int len,
	const uint8_t *src,
	uint8_t *dest,
	tap_state_t endstate)
{
	struct scan_field field;

	memset(&field, 0, sizeof(field));
	field.num_bits = len;
	field.out_value = src;
	field.in_value = dest;
	jtag_add_dr_scan(dm->tap, 1, &field, endstate);
}

int xtensa_dm_init(struct xtensa_debug_module *dm, const struct xtensa_debug_module_config *cfg)
{
	if (!dm || !cfg)
		return ERROR_FAIL;

	dm->pwr_ops = cfg->pwr_ops;
	dm->dbg_ops = cfg->dbg_ops;
	dm->tap = cfg->tap;
	dm->queue_tdi_idle = cfg->queue_tdi_idle;
	dm->queue_tdi_idle_arg = cfg->queue_tdi_idle_arg;
	return ERROR_OK;
}

int xtensa_dm_queue_enable(struct xtensa_debug_module *dm)
{
	return dm->dbg_ops->queue_reg_write(dm, NARADR_DCRSET, OCDDCR_ENABLEOCD);
}

int xtensa_dm_queue_reg_read(struct xtensa_debug_module *dm, unsigned int reg, uint8_t *value)
{
	uint8_t regdata = (reg << 1) | 0;
	uint8_t dummy[4] = { 0, 0, 0, 0 };

	if (reg > NARADR_MAX) {
		LOG_ERROR("Invalid DBG reg ID %d!", reg);
		return ERROR_FAIL;
	}
	xtensa_dm_add_set_ir(dm, TAPINS_NARSEL);
	xtensa_dm_add_dr_scan(dm, TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	xtensa_dm_add_dr_scan(dm, TAPINS_NARSEL_DATALEN, dummy, value, TAP_IDLE);
	return ERROR_OK;
}

int xtensa_dm_queue_reg_write(struct xtensa_debug_module *dm, unsigned int reg, uint32_t value)
{
	uint8_t regdata = (reg << 1) | 1;
	uint8_t valdata[] = { value, value >> 8, value >> 16, value >> 24 };

	if (reg > NARADR_MAX) {
		LOG_ERROR("Invalid DBG reg ID %d!", reg);
		return ERROR_FAIL;
	}
	xtensa_dm_add_set_ir(dm, TAPINS_NARSEL);
	xtensa_dm_add_dr_scan(dm, TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	xtensa_dm_add_dr_scan(dm, TAPINS_NARSEL_DATALEN, valdata, NULL, TAP_IDLE);
	return ERROR_OK;
}

int xtensa_dm_queue_pwr_reg_read(struct xtensa_debug_module *dm, unsigned int reg, uint8_t *data, uint8_t clear)
{
	uint8_t value_clr = clear;
	uint8_t tap_insn;
	int tap_insn_sz;

	if (reg == DMREG_PWRCTL) {
		tap_insn = TAPINS_PWRCTL;
		tap_insn_sz = TAPINS_PWRCTL_LEN;
	} else if (reg == DMREG_PWRSTAT) {
		tap_insn = TAPINS_PWRSTAT;
		tap_insn_sz = TAPINS_PWRSTAT_LEN;
	} else {
		LOG_ERROR("Invalid PWR reg ID %d!", reg);
		return ERROR_FAIL;
	}
	xtensa_dm_add_set_ir(dm, tap_insn);
	xtensa_dm_add_dr_scan(dm, tap_insn_sz, &value_clr, data, TAP_IDLE);
	return ERROR_OK;
}

int xtensa_dm_queue_pwr_reg_write(struct xtensa_debug_module *dm, unsigned int reg, uint8_t data)
{
	uint8_t value = data;
	uint8_t tap_insn;
	int tap_insn_sz;

	if (reg == DMREG_PWRCTL) {
		tap_insn = TAPINS_PWRCTL;
		tap_insn_sz = TAPINS_PWRCTL_LEN;
	} else if (reg == DMREG_PWRSTAT) {
		tap_insn = TAPINS_PWRSTAT;
		tap_insn_sz = TAPINS_PWRSTAT_LEN;
	} else {
		LOG_ERROR("Invalid PWR reg ID %d!", reg);
		return ERROR_FAIL;
	}
	xtensa_dm_add_set_ir(dm, tap_insn);
	xtensa_dm_add_dr_scan(dm, tap_insn_sz, &value, NULL, TAP_IDLE);
	return ERROR_OK;
}

int xtensa_dm_device_id_read(struct xtensa_debug_module *dm)
{
	uint8_t id_buf[sizeof(uint32_t)];

	dm->dbg_ops->queue_reg_read(dm, NARADR_OCDID, id_buf);
	xtensa_dm_queue_tdi_idle(dm);
	int res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	dm->device_id = buf_get_u32(id_buf, 0, 32);
	return ERROR_OK;
}

int xtensa_dm_power_status_read(struct xtensa_debug_module *dm, uint32_t clear)
{
	/* uint8_t id_buf[sizeof(uint32_t)]; */

	/* TODO: JTAG does not work when PWRCTL_JTAGDEBUGUSE is not set.
	 * It is set in xtensa_examine(), need to move reading of NARADR_OCDID out of this function */
	/* dm->dbg_ops->queue_reg_read(dm, NARADR_OCDID, id_buf);
	 *Read reset state */
	dm->pwr_ops->queue_reg_read(dm, DMREG_PWRSTAT, &dm->power_status.stat, clear);
	dm->pwr_ops->queue_reg_read(dm, DMREG_PWRSTAT, &dm->power_status.stath, clear);
	xtensa_dm_queue_tdi_idle(dm);
	return jtag_execute_queue();
}

int xtensa_dm_core_status_read(struct xtensa_debug_module *dm)
{
	uint8_t dsr_buf[sizeof(uint32_t)];

	xtensa_dm_queue_enable(dm);
	dm->dbg_ops->queue_reg_read(dm, NARADR_DSR, dsr_buf);
	xtensa_dm_queue_tdi_idle(dm);
	int res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	dm->core_status.dsr = buf_get_u32(dsr_buf, 0, 32);
	return res;
}

int xtensa_dm_core_status_clear(struct xtensa_debug_module *dm, xtensa_dsr_t bits)
{
	dm->dbg_ops->queue_reg_write(dm, NARADR_DSR, bits);
	xtensa_dm_queue_tdi_idle(dm);
	return jtag_execute_queue();
}

int xtensa_dm_trace_start(struct xtensa_debug_module *dm, struct xtensa_trace_start_config *cfg)
{
	/*Turn off trace unit so we can start a new trace. */
	dm->dbg_ops->queue_reg_write(dm, NARADR_TRAXCTRL, 0);
	xtensa_dm_queue_tdi_idle(dm);
	int res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;

	/*Set up parameters */
	dm->dbg_ops->queue_reg_write(dm, NARADR_TRAXADDR, 0);
	if (cfg->stopmask != XTENSA_STOPMASK_DISABLED) {
		dm->dbg_ops->queue_reg_write(dm, NARADR_PCMATCHCTRL,
			(cfg->stopmask << PCMATCHCTRL_PCML_SHIFT));
		dm->dbg_ops->queue_reg_write(dm, NARADR_TRIGGERPC, cfg->stoppc);
	}
	dm->dbg_ops->queue_reg_write(dm, NARADR_DELAYCNT, cfg->after);
	/*Options are mostly hardcoded for now. ToDo: make this more configurable. */
	dm->dbg_ops->queue_reg_write(
		dm,
		NARADR_TRAXCTRL,
		TRAXCTRL_TREN |
		((cfg->stopmask != XTENSA_STOPMASK_DISABLED) ? TRAXCTRL_PCMEN : 0) | TRAXCTRL_TMEN |
		(cfg->after_is_words ? 0 : TRAXCTRL_CNTU) | (0 << TRAXCTRL_SMPER_SHIFT) | TRAXCTRL_PTOWS);
	xtensa_dm_queue_tdi_idle(dm);
	return jtag_execute_queue();
}

int xtensa_dm_trace_stop(struct xtensa_debug_module *dm, bool pto_enable)
{
	uint8_t traxctl_buf[sizeof(uint32_t)];
	uint32_t traxctl;
	struct xtensa_trace_status trace_status;

	dm->dbg_ops->queue_reg_read(dm, NARADR_TRAXCTRL, traxctl_buf);
	xtensa_dm_queue_tdi_idle(dm);
	int res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	traxctl = buf_get_u32(traxctl_buf, 0, 32);

	if (!pto_enable)
		traxctl &= ~(TRAXCTRL_PTOWS | TRAXCTRL_PTOWT);

	dm->dbg_ops->queue_reg_write(dm, NARADR_TRAXCTRL, traxctl | TRAXCTRL_TRSTP);
	xtensa_dm_queue_tdi_idle(dm);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;

	/*Check current status of trace hardware */
	res = xtensa_dm_trace_status_read(dm, &trace_status);
	if (res != ERROR_OK)
		return res;

	if (trace_status.stat & TRAXSTAT_TRACT) {
		LOG_ERROR("Failed to stop tracing (0x%x)!", trace_status.stat);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int xtensa_dm_trace_status_read(struct xtensa_debug_module *dm, struct xtensa_trace_status *status)
{
	uint8_t traxstat_buf[sizeof(uint32_t)];

	dm->dbg_ops->queue_reg_read(dm, NARADR_TRAXSTAT, traxstat_buf);
	xtensa_dm_queue_tdi_idle(dm);
	int res = jtag_execute_queue();
	if (res == ERROR_OK && status)
		status->stat = buf_get_u32(traxstat_buf, 0, 32);
	return res;
}

int xtensa_dm_trace_config_read(struct xtensa_debug_module *dm, struct xtensa_trace_config *config)
{
	uint8_t traxctl_buf[sizeof(uint32_t)];
	uint8_t memadrstart_buf[sizeof(uint32_t)];
	uint8_t memadrend_buf[sizeof(uint32_t)];
	uint8_t adr_buf[sizeof(uint32_t)];

	if (!config)
		return ERROR_FAIL;

	dm->dbg_ops->queue_reg_read(dm, NARADR_TRAXCTRL, traxctl_buf);
	dm->dbg_ops->queue_reg_read(dm, NARADR_MEMADDRSTART, memadrstart_buf);
	dm->dbg_ops->queue_reg_read(dm, NARADR_MEMADDREND, memadrend_buf);
	dm->dbg_ops->queue_reg_read(dm, NARADR_TRAXADDR, adr_buf);
	xtensa_dm_queue_tdi_idle(dm);
	int res = jtag_execute_queue();
	if (res == ERROR_OK) {
		config->ctrl = buf_get_u32(traxctl_buf, 0, 32);
		config->memaddr_start = buf_get_u32(memadrstart_buf, 0, 32);
		config->memaddr_end = buf_get_u32(memadrend_buf, 0, 32);
		config->addr = buf_get_u32(adr_buf, 0, 32);
	}
	return res;
}

int xtensa_dm_trace_data_read(struct xtensa_debug_module *dm, uint8_t *dest, uint32_t size)
{
	if (!dest)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < size / 4; i++)
		dm->dbg_ops->queue_reg_read(dm, NARADR_TRAXDATA, &dest[i * 4]);
	xtensa_dm_queue_tdi_idle(dm);
	return jtag_execute_queue();
}

int xtensa_dm_perfmon_enable(struct xtensa_debug_module *dm, int counter_id,
	const struct xtensa_perfmon_config *config)
{
	if (!config)
		return ERROR_FAIL;

	uint8_t pmstat_buf[4];
	uint32_t pmctrl = ((config->tracelevel) << 4) +
		(config->select << 8) +
		(config->mask << 16) +
		(config->kernelcnt << 3);

	/* enable performance monitor */
	dm->dbg_ops->queue_reg_write(dm, NARADR_PMG, 0x1);
	/* reset counter */
	dm->dbg_ops->queue_reg_write(dm, NARADR_PM0 + counter_id, 0);
	dm->dbg_ops->queue_reg_write(dm, NARADR_PMCTRL0 + counter_id, pmctrl);
	dm->dbg_ops->queue_reg_read(dm, NARADR_PMSTAT0 + counter_id, pmstat_buf);
	xtensa_dm_queue_tdi_idle(dm);
	return jtag_execute_queue();
}

int xtensa_dm_perfmon_dump(struct xtensa_debug_module *dm, int counter_id,
	struct xtensa_perfmon_result *out_result)
{
	uint8_t pmstat_buf[4];
	uint8_t pmcount_buf[4];

	dm->dbg_ops->queue_reg_read(dm, NARADR_PMSTAT0 + counter_id, pmstat_buf);
	dm->dbg_ops->queue_reg_read(dm, NARADR_PM0 + counter_id, pmcount_buf);
	xtensa_dm_queue_tdi_idle(dm);
	int res = jtag_execute_queue();
	if (res == ERROR_OK) {
		uint32_t stat = buf_get_u32(pmstat_buf, 0, 32);
		uint64_t result = buf_get_u32(pmcount_buf, 0, 32);

		/* TODO: if counter # counter_id+1 has 'select' set to 1, use its value as the
		* high 32 bits of the counter. */
		if (out_result) {
			out_result->overflow = ((stat & 1) != 0);
			out_result->value = result;
		}
	}

	return res;
}

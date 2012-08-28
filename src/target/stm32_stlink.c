/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2011 by Spencer Oliver                                  *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/jtag.h"
#include "jtag/stlink/stlink_transport.h"
#include "jtag/stlink/stlink_interface.h"
#include "jtag/stlink/stlink_layout.h"
#include "register.h"
#include "algorithm.h"
#include "target.h"
#include "breakpoints.h"
#include "target_type.h"
#include "armv7m.h"
#include "cortex_m.h"
#include "arm_semihosting.h"

#define ARMV7M_SCS_DCRSR	0xe000edf4
#define ARMV7M_SCS_DCRDR	0xe000edf8

static inline struct stlink_interface_s *target_to_stlink(struct target *target)
{
	return target->tap->priv;
}

static int stm32_stlink_load_core_reg_u32(struct target *target,
		enum armv7m_regtype type,
		uint32_t num, uint32_t *value)
{
	int retval;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);

	LOG_DEBUG("%s", __func__);

	/* NOTE:  we "know" here that the register identifiers used
	 * in the v7m header match the Cortex-M3 Debug Core Register
	 * Selector values for R0..R15, xPSR, MSP, and PSP.
	 */
	switch (num) {
	case 0 ... 18:
		/* read a normal core register */
		retval = stlink_if->layout->api->read_reg(stlink_if->fd, num, value);

		if (retval != ERROR_OK) {
			LOG_ERROR("JTAG failure %i", retval);
			return ERROR_JTAG_DEVICE_ERROR;
		}
		LOG_DEBUG("load from core reg %i  value 0x%" PRIx32 "", (int)num, *value);
		break;

	case ARMV7M_FPSID:
	case ARMV7M_FPEXC:
		*value = 0;
		break;

	case ARMV7M_FPSCR:
		/* Floating-point Status and Registers */
		retval = target_write_u32(target, ARMV7M_SCS_DCRSR, 33);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, ARMV7M_SCS_DCRDR, value);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("load from core reg %i  value 0x%" PRIx32 "", (int)num, *value);
		break;

	case ARMV7M_S0 ... ARMV7M_S31:
		/* Floating-point Status and Registers */
		retval = target_write_u32(target, ARMV7M_SCS_DCRSR, num-ARMV7M_S0+64);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, ARMV7M_SCS_DCRDR, value);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("load from core reg %i  value 0x%" PRIx32 "", (int)num, *value);
		break;

	case ARMV7M_D0 ... ARMV7M_D15:
		value = 0;
		break;

	case ARMV7M_PRIMASK:
	case ARMV7M_BASEPRI:
	case ARMV7M_FAULTMASK:
	case ARMV7M_CONTROL:
		/* Cortex-M3 packages these four registers as bitfields
		 * in one Debug Core register.  So say r0 and r2 docs;
		 * it was removed from r1 docs, but still works.
		 */
		retval = stlink_if->layout->api->read_reg(stlink_if->fd, 20, value);
		if (retval != ERROR_OK)
			return retval;

		switch (num) {
		case ARMV7M_PRIMASK:
			*value = buf_get_u32((uint8_t *) value, 0, 1);
			break;

		case ARMV7M_BASEPRI:
			*value = buf_get_u32((uint8_t *) value, 8, 8);
			break;

		case ARMV7M_FAULTMASK:
			*value = buf_get_u32((uint8_t *) value, 16, 1);
			break;

		case ARMV7M_CONTROL:
			*value = buf_get_u32((uint8_t *) value, 24, 2);
			break;
		}

		LOG_DEBUG("load from special reg %i value 0x%" PRIx32 "",
			  (int)num, *value);
		break;

	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static int stm32_stlink_store_core_reg_u32(struct target *target,
		enum armv7m_regtype type,
		uint32_t num, uint32_t value)
{
	int retval;
	uint32_t reg;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct stlink_interface_s *stlink_if = target_to_stlink(target);

	LOG_DEBUG("%s", __func__);

#ifdef ARMV7_GDB_HACKS
	/* If the LR register is being modified, make sure it will put us
	 * in "thumb" mode, or an INVSTATE exception will occur. This is a
	 * hack to deal with the fact that gdb will sometimes "forge"
	 * return addresses, and doesn't set the LSB correctly (i.e., when
	 * printing expressions containing function calls, it sets LR = 0.)
	 * Valid exception return codes have bit 0 set too.
	 */
	if (num == ARMV7M_R14)
		value |= 0x01;
#endif

	/* NOTE:  we "know" here that the register identifiers used
	 * in the v7m header match the Cortex-M3 Debug Core Register
	 * Selector values for R0..R15, xPSR, MSP, and PSP.
	 */
	switch (num) {
	case 0 ... 18:
		retval = stlink_if->layout->api->write_reg(stlink_if->fd, num, value);

		if (retval != ERROR_OK) {
			struct reg *r;

			LOG_ERROR("JTAG failure");
			r = armv7m->core_cache->reg_list + num;
			r->dirty = r->valid;
			return ERROR_JTAG_DEVICE_ERROR;
		}
		LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", (int)num, value);
		break;

	case ARMV7M_FPSID:
	case ARMV7M_FPEXC:
		break;

	case ARMV7M_FPSCR:
		/* Floating-point Status and Registers */
		retval = target_write_u32(target, ARMV7M_SCS_DCRDR, value);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, ARMV7M_SCS_DCRSR, 33 | (1<<16));
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", (int)num, value);
		break;

	case ARMV7M_S0 ... ARMV7M_S31:
		/* Floating-point Status and Registers */
		retval = target_write_u32(target, ARMV7M_SCS_DCRDR, value);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, ARMV7M_SCS_DCRSR, (num-ARMV7M_S0+64) | (1<<16));
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", (int)num, value);
		break;

	case ARMV7M_D0 ... ARMV7M_D15:
		break;

	case ARMV7M_PRIMASK:
	case ARMV7M_BASEPRI:
	case ARMV7M_FAULTMASK:
	case ARMV7M_CONTROL:
		/* Cortex-M3 packages these four registers as bitfields
		 * in one Debug Core register.  So say r0 and r2 docs;
		 * it was removed from r1 docs, but still works.
		 */

		stlink_if->layout->api->read_reg(stlink_if->fd, 20, &reg);

		switch (num) {
		case ARMV7M_PRIMASK:
			buf_set_u32((uint8_t *) &reg, 0, 1, value);
			break;

		case ARMV7M_BASEPRI:
			buf_set_u32((uint8_t *) &reg, 8, 8, value);
			break;

		case ARMV7M_FAULTMASK:
			buf_set_u32((uint8_t *) &reg, 16, 1, value);
			break;

		case ARMV7M_CONTROL:
			buf_set_u32((uint8_t *) &reg, 24, 2, value);
			break;
		}

		stlink_if->layout->api->write_reg(stlink_if->fd, 20, reg);

		LOG_DEBUG("write special reg %i value 0x%" PRIx32 " ", (int)num, value);
		break;

	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static int stm32_stlink_examine_debug_reason(struct target *target)
{
	if ((target->debug_reason != DBG_REASON_DBGRQ)
			&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {
		target->debug_reason = DBG_REASON_BREAKPOINT;
	}

	return ERROR_OK;
}

static int stm32_stlink_init_arch_info(struct target *target,
				       struct cortex_m3_common *cortex_m3,
				       struct jtag_tap *tap)
{
	struct armv7m_common *armv7m;

	LOG_DEBUG("%s", __func__);

	armv7m = &cortex_m3->armv7m;
	armv7m_init_arch_info(target, armv7m);

	armv7m->load_core_reg_u32 = stm32_stlink_load_core_reg_u32;
	armv7m->store_core_reg_u32 = stm32_stlink_store_core_reg_u32;

	armv7m->examine_debug_reason = stm32_stlink_examine_debug_reason;
	armv7m->stlink = true;

	return ERROR_OK;
}

static int stm32_stlink_init_target(struct command_context *cmd_ctx,
				    struct target *target)
{
	LOG_DEBUG("%s", __func__);

	armv7m_build_reg_cache(target);

	return ERROR_OK;
}

static int stm32_stlink_target_create(struct target *target,
		Jim_Interp *interp)
{
	LOG_DEBUG("%s", __func__);

	struct cortex_m3_common *cortex_m3 = calloc(1, sizeof(struct cortex_m3_common));

	if (!cortex_m3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32_stlink_init_arch_info(target, cortex_m3, target->tap);

	return ERROR_OK;
}

static int stm32_stlink_load_context(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int num_regs = armv7m->core_cache->num_regs;

	for (int i = 0; i < num_regs; i++) {
		if (!armv7m->core_cache->reg_list[i].valid)
			armv7m->read_core_reg(target, i);
	}

	return ERROR_OK;
}

static int stlink_debug_entry(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct arm *arm = &armv7m->arm;
	struct reg *r;
	uint32_t xPSR;
	int retval;

	retval = armv7m->examine_debug_reason(target);
	if (retval != ERROR_OK)
		return retval;

	stm32_stlink_load_context(target);

	r = armv7m->core_cache->reg_list + ARMV7M_xPSR;
	xPSR = buf_get_u32(r->value, 0, 32);

	/* Are we in an exception handler */
	if (xPSR & 0x1FF) {
		armv7m->core_mode = ARMV7M_MODE_HANDLER;
		armv7m->exception_number = (xPSR & 0x1FF);

		arm->core_mode = ARM_MODE_HANDLER;
		arm->map = armv7m_msp_reg_map;
	} else {
		unsigned control = buf_get_u32(armv7m->core_cache
				->reg_list[ARMV7M_CONTROL].value, 0, 2);

		/* is this thread privileged? */
		armv7m->core_mode = control & 1;
		arm->core_mode = armv7m->core_mode
				? ARM_MODE_USER_THREAD
				: ARM_MODE_THREAD;

		/* which stack is it using? */
		if (control & 2)
			arm->map = armv7m_psp_reg_map;
		else
			arm->map = armv7m_msp_reg_map;

		armv7m->exception_number = 0;
	}

	LOG_DEBUG("entered debug state in core mode: %s at PC 0x%08" PRIx32 ", target->state: %s",
		armv7m_mode_strings[armv7m->core_mode],
		*(uint32_t *)(arm->pc->value),
		target_state_name(target));

	return retval;
}

static int stm32_stlink_poll(struct target *target)
{
	enum target_state state;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);

	state = stlink_if->layout->api->state(stlink_if->fd);

	if (state == TARGET_UNKNOWN) {
		LOG_ERROR("jtag status contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	if (target->state == state)
		return ERROR_OK;

	if (state == TARGET_HALTED) {
		target->state = state;

		int retval = stlink_debug_entry(target);
		if (retval != ERROR_OK)
			return retval;

		if (arm_semihosting(target, &retval) != 0)
			return retval;

		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		LOG_DEBUG("halted: PC: 0x%08x", buf_get_u32(armv7m->arm.pc->value, 0, 32));
	}

	return ERROR_OK;
}

static int stm32_stlink_assert_reset(struct target *target)
{
	int res = ERROR_OK;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	bool use_srst_fallback = true;

	LOG_DEBUG("%s", __func__);

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	bool srst_asserted = false;

	if (jtag_reset_config & RESET_SRST_NO_GATING) {
		jtag_add_reset(0, 1);
		res = stlink_if->layout->api->assert_srst(stlink_if->fd, 0);
		srst_asserted = true;
	}

	stlink_if->layout->api->write_debug_reg(stlink_if->fd, DCB_DHCSR, DBGKEY|C_DEBUGEN);
	stlink_if->layout->api->write_debug_reg(stlink_if->fd, DCB_DEMCR, VC_CORERESET);

	if (jtag_reset_config & RESET_HAS_SRST) {
		if (!srst_asserted) {
			jtag_add_reset(0, 1);
			res = stlink_if->layout->api->assert_srst(stlink_if->fd, 0);
		}
		if (res == ERROR_COMMAND_NOTFOUND)
			LOG_ERROR("Hardware srst not supported, falling back to software reset");
		else if (res == ERROR_OK) {
			/* hardware srst supported */
			use_srst_fallback = false;
		}
	}

	if (use_srst_fallback) {
		/* stlink v1 api does not support hardware srst, so we use a software reset fallback */
		stlink_if->layout->api->write_debug_reg(stlink_if->fd, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);
	}

	res = stlink_if->layout->api->reset(stlink_if->fd);

	if (res != ERROR_OK)
		return res;

	/* registers are now invalid */
	register_cache_invalidate(armv7m->core_cache);

	if (target->reset_halt) {
		target->state = TARGET_RESET;
		target->debug_reason = DBG_REASON_DBGRQ;
	} else {
		target->state = TARGET_HALTED;
	}

	return ERROR_OK;
}

static int stm32_stlink_deassert_reset(struct target *target)
{
	int res;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	LOG_DEBUG("%s", __func__);

	if (jtag_reset_config & RESET_HAS_SRST)
		stlink_if->layout->api->assert_srst(stlink_if->fd, 1);

	/* virtual deassert reset, we need it for the internal
	 * jtag state machine
	 */
	jtag_add_reset(0, 0);

	if (!target->reset_halt) {
		res = target_resume(target, 1, 0, 0, 0);

		if (res != ERROR_OK)
			return res;
	}

	return ERROR_OK;
}

static int stm32_stlink_soft_reset_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int stm32_stlink_halt(struct target *target)
{
	int res;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);

	LOG_DEBUG("%s", __func__);

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	res = stlink_if->layout->api->halt(stlink_if->fd);

	if (res != ERROR_OK)
		return res;

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int stm32_stlink_resume(struct target *target, int current,
		uint32_t address, int handle_breakpoints,
		int debug_execution)
{
	int res;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint32_t resume_pc;
	struct breakpoint *breakpoint = NULL;
	struct reg *pc;

	LOG_DEBUG("%s %d 0x%08x %d %d", __func__, current, address,
			handle_breakpoints, debug_execution);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	pc = armv7m->arm.pc;
	if (!current) {
		buf_set_u32(pc->value, 0, 32, address);
		pc->dirty = true;
		pc->valid = true;
	}

	if (!breakpoint_find(target, buf_get_u32(pc->value, 0, 32))
			&& !debug_execution) {
		armv7m_maybe_skip_bkpt_inst(target, NULL);
	}

	resume_pc = buf_get_u32(pc->value, 0, 32);

	armv7m_restore_context(target);

	/* registers are now invalid */
	register_cache_invalidate(armv7m->core_cache);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at 0x%8.8" PRIx32 " (ID: %d)",
					breakpoint->address,
					breakpoint->unique_id);
			cortex_m3_unset_breakpoint(target, breakpoint);

			res = stlink_if->layout->api->step(stlink_if->fd);

			if (res != ERROR_OK)
				return res;

			cortex_m3_set_breakpoint(target, breakpoint);
		}
	}

	res = stlink_if->layout->api->run(stlink_if->fd);

	if (res != ERROR_OK)
		return res;

	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	return ERROR_OK;
}

static int stm32_stlink_step(struct target *target, int current,
		uint32_t address, int handle_breakpoints)
{
	int res;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct breakpoint *breakpoint = NULL;
	struct reg *pc = armv7m->arm.pc;
	bool bkpt_inst_found = false;

	LOG_DEBUG("%s", __func__);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!current) {
		buf_set_u32(pc->value, 0, 32, address);
		pc->dirty = true;
		pc->valid = true;
	}

	uint32_t pc_value = buf_get_u32(pc->value, 0, 32);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, pc_value);
		if (breakpoint)
			cortex_m3_unset_breakpoint(target, breakpoint);
	}

	armv7m_maybe_skip_bkpt_inst(target, &bkpt_inst_found);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	armv7m_restore_context(target);

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	res = stlink_if->layout->api->step(stlink_if->fd);

	if (res != ERROR_OK)
		return res;

	/* registers are now invalid */
	register_cache_invalidate(armv7m->core_cache);

	if (breakpoint)
		cortex_m3_set_breakpoint(target, breakpoint);

	stlink_debug_entry(target);
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	LOG_INFO("halted: PC: 0x%08x", buf_get_u32(armv7m->arm.pc->value, 0, 32));

	return ERROR_OK;
}

static int stm32_stlink_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count,
		uint8_t *buffer)
{
	int res;
	uint32_t buffer_threshold = 128;
	uint32_t addr_increment = 4;
	uint32_t c;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);

	if (!count || !buffer)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_DEBUG("%s 0x%08x %d %d", __func__, address, size, count);

	/* prepare byte count, buffer threshold
	 * and address increment for none 32bit access
	 */
	if (size != 4) {
		count *= size;
		buffer_threshold = 64;
		addr_increment = 1;
	}

	while (count) {
		if (count > buffer_threshold)
			c = buffer_threshold;
		else
			c = count;

		if (size != 4)
			res = stlink_if->layout->api->read_mem8(stlink_if->fd,
					address, c, buffer);
		else
			res = stlink_if->layout->api->read_mem32(stlink_if->fd,
					address, c, buffer);

		if (res != ERROR_OK)
			return res;

		address += (c * addr_increment);
		buffer += (c * addr_increment);
		count -= c;
	}

	return ERROR_OK;
}

static int stm32_stlink_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count,
		const uint8_t *buffer)
{
	int res;
	uint32_t buffer_threshold = 128;
	uint32_t addr_increment = 4;
	uint32_t c;
	struct stlink_interface_s *stlink_if = target_to_stlink(target);

	if (!count || !buffer)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_DEBUG("%s 0x%08x %d %d", __func__, address, size, count);

	/* prepare byte count, buffer threshold
	 * and address increment for none 32bit access
	 */
	if (size != 4) {
		count *= size;
		buffer_threshold = 64;
		addr_increment = 1;
	}

	while (count) {
		if (count > buffer_threshold)
			c = buffer_threshold;
		else
			c = count;

		if (size != 4)
			res = stlink_if->layout->api->write_mem8(stlink_if->fd,
					address, c, buffer);
		else
			res = stlink_if->layout->api->write_mem32(stlink_if->fd,
					address, c, buffer);

		if (res != ERROR_OK)
			return res;

		address += (c * addr_increment);
		buffer += (c * addr_increment);
		count -= c;
	}

	return ERROR_OK;
}

static int stm32_stlink_bulk_write_memory(struct target *target,
		uint32_t address, uint32_t count,
		const uint8_t *buffer)
{
	return stm32_stlink_write_memory(target, address, 4, count, buffer);
}

static const struct command_registration stm32_stlink_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type stm32_stlink_target = {
	.name = "stm32_stlink",

	.init_target = stm32_stlink_init_target,
	.target_create = stm32_stlink_target_create,
	.examine = cortex_m3_examine,
	.commands = stm32_stlink_command_handlers,

	.poll = stm32_stlink_poll,
	.arch_state = armv7m_arch_state,

	.assert_reset = stm32_stlink_assert_reset,
	.deassert_reset = stm32_stlink_deassert_reset,
	.soft_reset_halt = stm32_stlink_soft_reset_halt,

	.halt = stm32_stlink_halt,
	.resume = stm32_stlink_resume,
	.step = stm32_stlink_step,

	.get_gdb_reg_list = armv7m_get_gdb_reg_list,

	.read_memory = stm32_stlink_read_memory,
	.write_memory = stm32_stlink_write_memory,
	.bulk_write_memory = stm32_stlink_bulk_write_memory,
	.checksum_memory = armv7m_checksum_memory,
	.blank_check_memory = armv7m_blank_check_memory,

	.run_algorithm = armv7m_run_algorithm,
	.start_algorithm = armv7m_start_algorithm,
	.wait_algorithm = armv7m_wait_algorithm,

	.add_breakpoint = cortex_m3_add_breakpoint,
	.remove_breakpoint = cortex_m3_remove_breakpoint,
	.add_watchpoint = cortex_m3_add_watchpoint,
	.remove_watchpoint = cortex_m3_remove_watchpoint,
};

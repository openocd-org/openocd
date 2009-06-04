/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
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

#include "avrt.h"
#include "target.h"
#include "target_type.h"


#define AVR_JTAG_INS_LEN	4

/* cli handling */
int avr_register_commands(struct command_context_s *cmd_ctx);

/* forward declarations */
int avr_target_create(struct target_s *target, Jim_Interp *interp);
int avr_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int avr_quit(void);

int avr_arch_state(struct target_s *target);
int avr_poll(target_t *target);
int avr_halt(target_t *target);
int avr_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution);
int avr_step(struct target_s *target, int current, u32 address, int handle_breakpoints);

int avr_assert_reset(target_t *target);
int avr_deassert_reset(target_t *target);
int avr_soft_reset_halt(struct target_s *target);

/* IR and DR functions */
int avr_jtag_sendinstr(jtag_tap_t *tap, u8 *ir_in, u8 ir_out);
int avr_jtag_senddat(jtag_tap_t *tap, u32 *dr_in, u32 dr_out, int len);

int mcu_write_ir(jtag_tap_t *tap, u8 *ir_in, u8 *ir_out, int ir_len, int rti);
int mcu_write_dr(jtag_tap_t *tap, u8 *dr_in, u8 *dr_out, int dr_len, int rti);
int mcu_write_ir_u8(jtag_tap_t *tap, u8 *ir_in, u8 ir_out, int ir_len, int rti);
int mcu_write_dr_u8(jtag_tap_t *tap, u8 *ir_in, u8 ir_out, int dr_len, int rti);
int mcu_write_ir_u16(jtag_tap_t *tap, u16 *ir_in, u16 ir_out, int ir_len, int rti);
int mcu_write_dr_u16(jtag_tap_t *tap, u16 *ir_in, u16 ir_out, int dr_len, int rti);
int mcu_write_ir_u32(jtag_tap_t *tap, u32 *ir_in, u32 ir_out, int ir_len, int rti);
int mcu_write_dr_u32(jtag_tap_t *tap, u32 *ir_in, u32 ir_out, int dr_len, int rti);
int mcu_execute_queue(void);

target_type_t avr_target =
{
	.name = "avr",

	.poll = avr_poll,
	.arch_state = avr_arch_state,

	.target_request_data = NULL,

	.halt = avr_halt,
	.resume = avr_resume,
	.step = avr_step,

	.assert_reset = avr_assert_reset,
	.deassert_reset = avr_deassert_reset,
	.soft_reset_halt = avr_soft_reset_halt,
/*
	.get_gdb_reg_list = avr_get_gdb_reg_list,

	.read_memory = avr_read_memory,
	.write_memory = avr_write_memory,
	.bulk_write_memory = avr_bulk_write_memory,
	.checksum_memory = avr_checksum_memory,
	.blank_check_memory = avr_blank_check_memory,

	.run_algorithm = avr_run_algorithm,

	.add_breakpoint = avr_add_breakpoint,
	.remove_breakpoint = avr_remove_breakpoint,
	.add_watchpoint = avr_add_watchpoint,
	.remove_watchpoint = avr_remove_watchpoint,
*/
	.register_commands = avr_register_commands,
	.target_create = avr_target_create,
	.init_target = avr_init_target,
	.quit = avr_quit,
/*
	.virt2phys = avr_virt2phys,
	.mmu = avr_mmu
*/
};

int avr_register_commands(struct command_context_s *cmd_ctx)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_target_create(struct target_s *target, Jim_Interp *interp)
{
	avr_common_t *avr = calloc(1, sizeof(avr_common_t));

	avr->jtag_info.tap = target->tap;
	target->arch_info = avr;

	return ERROR_OK;
}

int avr_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_quit(void)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_arch_state(struct target_s *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_poll(target_t *target)
{
	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_DEBUG_RUNNING))
	{
		target->state = TARGET_HALTED;
	}

	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_halt(target_t *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_step(struct target_s *target, int current, u32 address, int handle_breakpoints)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_assert_reset(target_t *target)
{
	target->state = TARGET_RESET;

	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_deassert_reset(target_t *target)
{
	target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_soft_reset_halt(struct target_s *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int avr_jtag_senddat(jtag_tap_t *tap, u32* dr_in, u32 dr_out, int len)
{
	return mcu_write_dr_u32(tap, dr_in, dr_out, len, 1);
}

int avr_jtag_sendinstr(jtag_tap_t *tap, u8 *ir_in, u8 ir_out)
{
	return mcu_write_ir_u8(tap, ir_in, ir_out, AVR_JTAG_INS_LEN, 1);
}

/* IR and DR functions */
int mcu_write_ir(jtag_tap_t *tap, u8 *ir_in, u8 *ir_out, int ir_len, int rti)
{
	if (NULL == tap)
	{
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}
	if (ir_len != tap->ir_length)
	{
		LOG_ERROR("invalid ir_len");
		return ERROR_FAIL;
	}

	{
		scan_field_t field[1];

		field[0].tap = tap;
		field[0].num_bits = tap->ir_length;
		field[0].out_value = ir_out;
		field[0].in_value = ir_in;
		jtag_add_plain_ir_scan(sizeof(field) / sizeof(field[0]), field, jtag_set_end_state(TAP_IDLE));
	}

	return ERROR_OK;
}

int mcu_write_dr(jtag_tap_t *tap, u8 *dr_in, u8 *dr_out, int dr_len, int rti)
{
	if (NULL == tap)
	{
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}

	{
		scan_field_t field[1];

		field[0].tap = tap;
		field[0].num_bits = dr_len;
		field[0].out_value = dr_out;
		field[0].in_value = dr_in;
		jtag_add_plain_dr_scan(sizeof(field) / sizeof(field[0]), field, jtag_set_end_state(TAP_IDLE));
	}

	return ERROR_OK;
}

int mcu_write_ir_u8(jtag_tap_t *tap, u8 *ir_in, u8 ir_out, int ir_len, int rti)
{
	if (ir_len > 8)
	{
		LOG_ERROR("ir_len overflow, maxium is 8");
		return ERROR_FAIL;
	}

	mcu_write_ir(tap, ir_in, &ir_out, ir_len, rti);

	return ERROR_OK;
}

int mcu_write_dr_u8(jtag_tap_t *tap, u8 *dr_in, u8 dr_out, int dr_len, int rti)
{
	if (dr_len > 8)
	{
		LOG_ERROR("dr_len overflow, maxium is 8");
		return ERROR_FAIL;
	}

	mcu_write_dr(tap, dr_in, &dr_out, dr_len, rti);

	return ERROR_OK;
}

int mcu_write_ir_u16(jtag_tap_t *tap, u16 *ir_in, u16 ir_out, int ir_len, int rti)
{
	if (ir_len > 16)
	{
		LOG_ERROR("ir_len overflow, maxium is 16");
		return ERROR_FAIL;
	}

	mcu_write_ir(tap, (u8*)ir_in, (u8*)&ir_out, ir_len, rti);

	return ERROR_OK;
}

int mcu_write_dr_u16(jtag_tap_t *tap, u16 *dr_in, u16 dr_out, int dr_len, int rti)
{
	if (dr_len > 16)
	{
		LOG_ERROR("dr_len overflow, maxium is 16");
		return ERROR_FAIL;
	}

	mcu_write_dr(tap, (u8*)dr_in, (u8*)&dr_out, dr_len, rti);

	return ERROR_OK;
}

int mcu_write_ir_u32(jtag_tap_t *tap, u32 *ir_in, u32 ir_out, int ir_len, int rti)
{
	if (ir_len > 32)
	{
		LOG_ERROR("ir_len overflow, maxium is 32");
		return ERROR_FAIL;
	}

	mcu_write_ir(tap, (u8*)ir_in, (u8*)&ir_out, ir_len, rti);

	return ERROR_OK;
}

int mcu_write_dr_u32(jtag_tap_t *tap, u32 *dr_in, u32 dr_out, int dr_len, int rti)
{
	if (dr_len > 32)
	{
		LOG_ERROR("dr_len overflow, maxium is 32");
		return ERROR_FAIL;
	}

	mcu_write_dr(tap, (u8*)dr_in, (u8*)&dr_out, dr_len, rti);

	return ERROR_OK;
}

int mcu_execute_queue(void)
{
	return jtag_execute_queue();
}

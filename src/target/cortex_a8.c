/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009 by Dirk Behme                                      *
 *   dirk.behme@gmail.com - copy from cortex_m3                            *
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
 *                                                                         *
 *   Cortex-A8(tm) TRM, ARM DDI 0344H                                      *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "cortex_a8.h"
#include "armv7m.h"

#include "register.h"
#include "target.h"
#include "target_request.h"
#include "log.h"
#include "jtag.h"
#include "arm_jtag.h"

#include <stdlib.h>
#include <string.h>

/* cli handling */
int cortex_a8_register_commands(struct command_context_s *cmd_ctx);

/* forward declarations */
int cortex_a8_target_create(struct target_s *target, Jim_Interp *interp);

target_type_t cortexa8_target =
{
	.name = "cortex_a8",

	.poll = NULL,
	.arch_state = armv7m_arch_state,

	.target_request_data = NULL,

	.halt = NULL,
	.resume = NULL,
	.step = NULL,

	.assert_reset = NULL,
	.deassert_reset = NULL,
	.soft_reset_halt = NULL,

	.get_gdb_reg_list = armv7m_get_gdb_reg_list,

	.read_memory = cortex_a8_read_memory,
	.write_memory = cortex_a8_write_memory,
	.bulk_write_memory = NULL,
	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.run_algorithm = armv7m_run_algorithm,

	.add_breakpoint = NULL,
	.remove_breakpoint = NULL,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.register_commands = cortex_a8_register_commands,
	.target_create = cortex_a8_target_create,
	.init_target = NULL,
	.examine = NULL,
	.quit = NULL
};

int cortex_a8_dcc_read(swjdp_common_t *swjdp, u8 *value, u8 *ctrl)
{
	u16 dcrdr;

	mem_ap_read_buf_u16( swjdp, (u8*)&dcrdr, 1, DCB_DCRDR);
	*ctrl = (u8)dcrdr;
	*value = (u8)(dcrdr >> 8);

	LOG_DEBUG("data 0x%x ctrl 0x%x", *value, *ctrl);

	/* write ack back to software dcc register
	 * signify we have read data */
	if (dcrdr & (1 << 0))
	{
		dcrdr = 0;
		mem_ap_write_buf_u16( swjdp, (u8*)&dcrdr, 1, DCB_DCRDR);
	}

	return ERROR_OK;
}

int cortex_a8_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	int retval;

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	/* cortex_a8 handles unaligned memory access */

	switch (size)
	{
		case 4:
			retval = mem_ap_read_buf_u32(swjdp, buffer, 4 * count, address);
			break;
		case 2:
			retval = mem_ap_read_buf_u16(swjdp, buffer, 2 * count, address);
			break;
		case 1:
			retval = mem_ap_read_buf_u8(swjdp, buffer, count, address);
			break;
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
	}

	return retval;
}

int cortex_a8_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	int retval;

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	switch (size)
	{
		case 4:
			retval = mem_ap_write_buf_u32(swjdp, buffer, 4 * count, address);
			break;
		case 2:
			retval = mem_ap_write_buf_u16(swjdp, buffer, 2 * count, address);
			break;
		case 1:
			retval = mem_ap_write_buf_u8(swjdp, buffer, count, address);
			break;
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
	}

	return retval;
}

int cortex_a8_handle_target_request(void *priv)
{
	target_t *target = priv;
	if (!target->type->examined)
		return ERROR_OK;
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING)
	{
		u8 data;
		u8 ctrl;

		cortex_a8_dcc_read(swjdp, &data, &ctrl);

		/* check if we have data */
		if (ctrl & (1 << 0))
		{
			u32 request;

			/* we assume target is quick enough */
			request = data;
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 8);
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 16);
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 24);
			target_request(target, request);
		}
	}

	return ERROR_OK;
}

int cortex_a8_init_arch_info(target_t *target, cortex_a8_common_t *cortex_a8, jtag_tap_t *tap)
{
	armv7m_common_t *armv7m;
	armv7m = &cortex_a8->armv7m;

	/* prepare JTAG information for the new target */
	cortex_a8->jtag_info.tap = tap;
	cortex_a8->jtag_info.scann_size = 4;

	armv7m->swjdp_info.dp_select_value = -1;
	armv7m->swjdp_info.ap_csw_value = -1;
	armv7m->swjdp_info.ap_tar_value = -1;
	armv7m->swjdp_info.jtag_info = &cortex_a8->jtag_info;

	/* initialize arch-specific breakpoint handling */

	cortex_a8->common_magic = CORTEX_A8_COMMON_MAGIC;
	cortex_a8->arch_info = NULL;

	/* register arch-specific functions */
	armv7m->examine_debug_reason = NULL;

	armv7m->pre_debug_entry = NULL;
	armv7m->post_debug_entry = NULL;

	armv7m->pre_restore_context = NULL;
	armv7m->post_restore_context = NULL;

	armv7m_init_arch_info(target, armv7m);
	armv7m->arch_info = cortex_a8;
	armv7m->load_core_reg_u32 = NULL;
	armv7m->store_core_reg_u32 = NULL;

	target_register_timer_callback(cortex_a8_handle_target_request, 1, 1, target);

	return ERROR_OK;
}

int cortex_a8_target_create(struct target_s *target, Jim_Interp *interp)
{
	cortex_a8_common_t *cortex_a8 = calloc(1,sizeof(cortex_a8_common_t));

	cortex_a8_init_arch_info(target, cortex_a8, target->tap);

	return ERROR_OK;
}

int cortex_a8_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;

	retval = armv7m_register_commands(cmd_ctx);

	register_command(cmd_ctx, NULL, "cortex_a8", NULL, COMMAND_ANY, "cortex_a8 specific commands");

	return retval;
}

/***************************************************************************
 *   Copyright (C) 2009 by Marvell Semiconductors, Inc.                    *
 *   Written by Nicolas Pitre <nico at marvell.com>                           *
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

/*
 * NAND controller interface for Marvell Orion/Kirkwood SoCs.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

#include "nand.h"
#include "target.h"
#include "armv4_5.h"
#include "binarybuffer.h"

typedef struct orion_nand_controller_s
{
	struct target_s	*target;
	working_area_t *copy_area;

	u32		cmd;
	u32		addr;
	u32		data;
} orion_nand_controller_t;

#define CHECK_HALTED \
	do { \
		if (target->state != TARGET_HALTED) { \
			LOG_ERROR("NAND flash access requires halted target"); \
			return ERROR_NAND_OPERATION_FAILED; \
		} \
	} while (0)

int orion_nand_command(struct nand_device_s *device, u8 command)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_write_u8(target, hw->cmd, command);
	return ERROR_OK;
}

int orion_nand_address(struct nand_device_s *device, u8 address)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_write_u8(target, hw->addr, address);
	return ERROR_OK;
}

int orion_nand_read(struct nand_device_s *device, void *data)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_read_u8(target, hw->data, data);
	return ERROR_OK;
}

int orion_nand_write(struct nand_device_s *device, u16 data)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_write_u8(target, hw->data, data);
	return ERROR_OK;
}

int orion_nand_slow_block_write(struct nand_device_s *device, u8 *data, int size)
{
	while (size--)
		orion_nand_write(device, *data++);
	return ERROR_OK;
}

int orion_nand_fast_block_write(struct nand_device_s *device, u8 *data, int size)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;
	armv4_5_algorithm_t algo;
	reg_param_t reg_params[3];
	u32 target_buf;
	int retval;

	static const u32 code[] = {
		0xe4d13001,	/* ldrb	r3, [r1], #1	*/
		0xe5c03000,	/* strb	r3, [r0]	*/
		0xe2522001,	/* subs	r2, r2, #1	*/
		0x1afffffb,	/* bne	0		*/
		0xeafffffe,	/* b	.		*/
	};
	int code_size = sizeof(code);

	if (!hw->copy_area) {
		u8 code_buf[code_size];
		int i;

		/* make sure we have a working area */
		if (target_alloc_working_area(target,
					      code_size + device->page_size,
					      &hw->copy_area) != ERROR_OK)
		{
			return orion_nand_slow_block_write(device, data, size);
		}

		/* copy target instructions to target endianness */
		for (i = 0; i < code_size/4; i++)
			target_buffer_set_u32(target, code_buf + i*4, code[i]);

		/* write code to working area */
                retval = target->type->write_memory(target,
					hw->copy_area->address,
					4, code_size/4, code_buf);
		if (retval != ERROR_OK)
			return retval;
	}

	/* copy data to target's memory */
	target_buf = hw->copy_area->address + code_size;
	retval = target->type->bulk_write_memory(target, target_buf,
						 size/4, data);
	if (retval == ERROR_OK && size & 3) {
		retval = target->type->write_memory(target,
					target_buf + (size & ~3),
					1, size & 3, data + (size & ~3));
	}
	if (retval != ERROR_OK)
		return retval;

	algo.common_magic = ARMV4_5_COMMON_MAGIC;
	algo.core_mode = ARMV4_5_MODE_SVC;
	algo.core_state = ARMV4_5_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_IN);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN);

	buf_set_u32(reg_params[0].value, 0, 32, hw->data);
	buf_set_u32(reg_params[1].value, 0, 32, target_buf);
	buf_set_u32(reg_params[2].value, 0, 32, size);

	retval = target->type->run_algorithm(target, 0, NULL, 3, reg_params,
					hw->copy_area->address,
					hw->copy_area->address + code_size - 4,
					1000, &algo);
	if (retval != ERROR_OK)
		LOG_ERROR("error executing hosted NAND write");

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	return retval;
}

int orion_nand_reset(struct nand_device_s *device)
{
	return orion_nand_command(device, NAND_CMD_RESET);
}

int orion_nand_controller_ready(struct nand_device_s *device, int timeout)
{
	return 1;
}

int orion_nand_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

int orion_nand_device_command(struct command_context_s *cmd_ctx, char *cmd,
			      char **args, int argc,
			      struct nand_device_s *device)
{
	orion_nand_controller_t *hw;
	u32 base;
	u8 ale, cle;

	if (argc != 3) {
		LOG_ERROR("arguments must be: <target_number> <NAND_address>\n");
		return ERROR_NAND_DEVICE_INVALID;
	}

	hw = calloc(1, sizeof(*hw));
	if (!hw) {
		LOG_ERROR("no memory for nand controller\n");
		return ERROR_NAND_DEVICE_INVALID;
	}

	device->controller_priv = hw;
	hw->target = get_target_by_num(strtoul(args[1], NULL, 0));
	if (!hw->target) {
		LOG_ERROR("no target '%s' configured", args[1]);
		free(hw);
		return ERROR_NAND_DEVICE_INVALID;
	}

	base = strtoul(args[2], NULL, 0);
	cle = 0;
	ale = 1;

	hw->data = base;
	hw->cmd = base + (1 << cle);
	hw->addr = base + (1 << ale);

	return ERROR_OK;
}

int orion_nand_init(struct nand_device_s *device)
{
	return ERROR_OK;
}

nand_flash_controller_t orion_nand_controller =
{
	.name			= "orion",
	.command		= orion_nand_command,
	.address		= orion_nand_address,
	.read_data		= orion_nand_read,
	.write_data		= orion_nand_write,
	.write_block_data	= orion_nand_fast_block_write,
	.reset			= orion_nand_reset,
	.controller_ready	= orion_nand_controller_ready,
	.nand_device_command	= orion_nand_device_command,
	.register_commands	= orion_nand_register_commands,
	.init			= orion_nand_init,
};


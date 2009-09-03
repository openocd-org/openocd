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

#include "arm_nandio.h"
#include "armv4_5.h"


typedef struct orion_nand_controller_s
{
	struct target_s	*target;

	struct arm_nand_data	io;

	uint32_t		cmd;
	uint32_t		addr;
	uint32_t		data;
} orion_nand_controller_t;

#define CHECK_HALTED \
	do { \
		if (target->state != TARGET_HALTED) { \
			LOG_ERROR("NAND flash access requires halted target"); \
			return ERROR_NAND_OPERATION_FAILED; \
		} \
	} while (0)

static int orion_nand_command(struct nand_device_s *device, uint8_t command)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_write_u8(target, hw->cmd, command);
	return ERROR_OK;
}

static int orion_nand_address(struct nand_device_s *device, uint8_t address)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_write_u8(target, hw->addr, address);
	return ERROR_OK;
}

static int orion_nand_read(struct nand_device_s *device, void *data)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_read_u8(target, hw->data, data);
	return ERROR_OK;
}

static int orion_nand_write(struct nand_device_s *device, uint16_t data)
{
	orion_nand_controller_t *hw = device->controller_priv;
	target_t *target = hw->target;

	CHECK_HALTED;
	target_write_u8(target, hw->data, data);
	return ERROR_OK;
}

static int orion_nand_slow_block_write(struct nand_device_s *device, uint8_t *data, int size)
{
	while (size--)
		orion_nand_write(device, *data++);
	return ERROR_OK;
}

static int orion_nand_fast_block_write(struct nand_device_s *device, uint8_t *data, int size)
{
	orion_nand_controller_t *hw = device->controller_priv;
	int retval;

	hw->io.chunk_size = device->page_size;

	retval = arm_nandwrite(&hw->io, data, size);
	if (retval == ERROR_NAND_NO_BUFFER)
		retval = orion_nand_slow_block_write(device, data, size);

	return retval;
}

static int orion_nand_reset(struct nand_device_s *device)
{
	return orion_nand_command(device, NAND_CMD_RESET);
}

static int orion_nand_controller_ready(struct nand_device_s *device, int timeout)
{
	return 1;
}

static int orion_nand_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

int orion_nand_device_command(struct command_context_s *cmd_ctx, char *cmd,
			      char **args, int argc,
			      struct nand_device_s *device)
{
	orion_nand_controller_t *hw;
	uint32_t base;
	uint8_t ale, cle;

	if (argc != 3) {
		LOG_ERROR("arguments must be: <target_id> <NAND_address>\n");
		return ERROR_NAND_DEVICE_INVALID;
	}

	hw = calloc(1, sizeof(*hw));
	if (!hw) {
		LOG_ERROR("no memory for nand controller\n");
		return ERROR_NAND_DEVICE_INVALID;
	}

	device->controller_priv = hw;
	hw->target = get_target(args[1]);
	if (!hw->target) {
		LOG_ERROR("target '%s' not defined", args[1]);
		free(hw);
		return ERROR_NAND_DEVICE_INVALID;
	}

	base = strtoul(args[2], NULL, 0);
	cle = 0;
	ale = 1;

	hw->data = base;
	hw->cmd = base + (1 << cle);
	hw->addr = base + (1 << ale);

	hw->io.target = hw->target;
	hw->io.data = hw->data;

	return ERROR_OK;
}

static int orion_nand_init(struct nand_device_s *device)
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


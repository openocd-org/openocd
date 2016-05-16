/***************************************************************************
 *   Copyright (C) 2009 by Marvell Semiconductors, Inc.                    *
 *   Written by Nicolas Pitre <nico at marvell.com>                        *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/*
 * NAND controller interface for Marvell Orion/Kirkwood SoCs.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "arm_io.h"
#include <target/arm.h>

struct orion_nand_controller {
	struct arm_nand_data	io;

	uint32_t		cmd;
	uint32_t		addr;
	uint32_t		data;
};

#define CHECK_HALTED \
	do { \
		if (target->state != TARGET_HALTED) { \
			LOG_ERROR("NAND flash access requires halted target"); \
			return ERROR_NAND_OPERATION_FAILED; \
		} \
	} while (0)

static int orion_nand_command(struct nand_device *nand, uint8_t command)
{
	struct orion_nand_controller *hw = nand->controller_priv;
	struct target *target = nand->target;

	CHECK_HALTED;
	target_write_u8(target, hw->cmd, command);
	return ERROR_OK;
}

static int orion_nand_address(struct nand_device *nand, uint8_t address)
{
	struct orion_nand_controller *hw = nand->controller_priv;
	struct target *target = nand->target;

	CHECK_HALTED;
	target_write_u8(target, hw->addr, address);
	return ERROR_OK;
}

static int orion_nand_read(struct nand_device *nand, void *data)
{
	struct orion_nand_controller *hw = nand->controller_priv;
	struct target *target = nand->target;

	CHECK_HALTED;
	target_read_u8(target, hw->data, data);
	return ERROR_OK;
}

static int orion_nand_write(struct nand_device *nand, uint16_t data)
{
	struct orion_nand_controller *hw = nand->controller_priv;
	struct target *target = nand->target;

	CHECK_HALTED;
	target_write_u8(target, hw->data, data);
	return ERROR_OK;
}

static int orion_nand_slow_block_write(struct nand_device *nand, uint8_t *data, int size)
{
	while (size--)
		orion_nand_write(nand, *data++);
	return ERROR_OK;
}

static int orion_nand_fast_block_write(struct nand_device *nand, uint8_t *data, int size)
{
	struct orion_nand_controller *hw = nand->controller_priv;
	int retval;

	hw->io.chunk_size = nand->page_size;

	retval = arm_nandwrite(&hw->io, data, size);
	if (retval == ERROR_NAND_NO_BUFFER)
		retval = orion_nand_slow_block_write(nand, data, size);

	return retval;
}

static int orion_nand_reset(struct nand_device *nand)
{
	return orion_nand_command(nand, NAND_CMD_RESET);
}

NAND_DEVICE_COMMAND_HANDLER(orion_nand_device_command)
{
	struct orion_nand_controller *hw;
	uint32_t base;
	uint8_t ale, cle;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	hw = calloc(1, sizeof(*hw));
	if (!hw) {
		LOG_ERROR("no memory for nand controller");
		return ERROR_NAND_DEVICE_INVALID;
	}

	nand->controller_priv = hw;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], base);
	cle = 0;
	ale = 1;

	hw->data = base;
	hw->cmd = base + (1 << cle);
	hw->addr = base + (1 << ale);

	hw->io.target = nand->target;
	hw->io.data = hw->data;
	hw->io.op = ARM_NAND_NONE;

	return ERROR_OK;
}

static int orion_nand_init(struct nand_device *nand)
{
	return ERROR_OK;
}

struct nand_flash_controller orion_nand_controller = {
	.name = "orion",
	.usage = "<target_id> <NAND_address>",
	.command = orion_nand_command,
	.address = orion_nand_address,
	.read_data = orion_nand_read,
	.write_data = orion_nand_write,
	.write_block_data = orion_nand_fast_block_write,
	.reset = orion_nand_reset,
	.nand_device_command = orion_nand_device_command,
	.init = orion_nand_init,
};

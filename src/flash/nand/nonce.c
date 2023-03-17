// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "hello.h"

static int nonce_nand_command(struct nand_device *nand, uint8_t command)
{
	return ERROR_OK;
}
static int nonce_nand_address(struct nand_device *nand, uint8_t address)
{
	return ERROR_OK;
}
static int nonce_nand_read(struct nand_device *nand, void *data)
{
	return ERROR_OK;
}
static int nonce_nand_write(struct nand_device *nand, uint16_t data)
{
	return ERROR_OK;
}
static int nonce_nand_fast_block_write(struct nand_device *nand,
		uint8_t *data, int size)
{
	return ERROR_OK;
}

static int nonce_nand_reset(struct nand_device *nand)
{
	return nonce_nand_command(nand, NAND_CMD_RESET);
}

NAND_DEVICE_COMMAND_HANDLER(nonce_nand_device_command)
{
	return ERROR_OK;
}

static int nonce_nand_init(struct nand_device *nand)
{
	return ERROR_OK;
}

struct nand_flash_controller nonce_nand_controller = {
	.name = "nonce",
	.commands = hello_command_handlers,
	.nand_device_command = &nonce_nand_device_command,
	.init = &nonce_nand_init,
	.reset = &nonce_nand_reset,
	.command = &nonce_nand_command,
	.address = &nonce_nand_address,
	.read_data = &nonce_nand_read,
	.write_data = &nonce_nand_write,
	.write_block_data = &nonce_nand_fast_block_write,
};

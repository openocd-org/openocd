// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2010 by Spencer Oliver <spen@spen-soft.co.uk>
 *
 * Copyright (C) 2019 by Tomas Vanek <vanekt@fbl.cz>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

static struct flash_bank *virtual_get_master_bank(struct flash_bank *bank)
{
	struct flash_bank *master_bank;

	master_bank = get_flash_bank_by_name_noprobe(bank->driver_priv);
	if (!master_bank)
		LOG_ERROR("master flash bank '%s' does not exist", (char *)bank->driver_priv);

	return master_bank;
}

static void virtual_update_bank_info(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return;

	/* update the info we do not have */
	bank->size = master_bank->size;
	if (bank->read_only)
		return;

	bank->chip_width = master_bank->chip_width;
	bank->bus_width = master_bank->bus_width;
	bank->erased_value = master_bank->erased_value;
	bank->default_padded_value = master_bank->default_padded_value;
	bank->write_start_alignment = master_bank->write_start_alignment;
	bank->write_end_alignment = master_bank->write_end_alignment;
	bank->minimal_write_gap = master_bank->minimal_write_gap;
	bank->num_sectors = master_bank->num_sectors;
	bank->sectors = master_bank->sectors;
	bank->num_prot_blocks = master_bank->num_prot_blocks;
	bank->prot_blocks = master_bank->prot_blocks;
}

FLASH_BANK_COMMAND_HANDLER(virtual_flash_bank_command)
{
	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* get the master flash bank */
	const char *bank_name = CMD_ARGV[6];
	struct flash_bank *master_bank = get_flash_bank_by_name_noprobe(bank_name);

	if (!master_bank) {
		LOG_ERROR("master flash bank '%s' does not exist", bank_name);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* save master bank name - use this to get settings later */
	bank->driver_priv = strdup(bank_name);

	return ERROR_OK;
}

static int virtual_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	return flash_driver_protect(master_bank, set, first, last);
}

static int virtual_protect_check(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	if (!master_bank->driver->protect_check)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	/* call master handler */
	return master_bank->driver->protect_check(master_bank);
}

static int virtual_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	return master_bank->driver->erase(master_bank, first, last);
}

static int virtual_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	return master_bank->driver->write(master_bank, buffer, offset, count);
}

static int virtual_probe(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->probe(master_bank);
	if (retval != ERROR_OK)
		return retval;

	/* update the info we do not have */
	virtual_update_bank_info(bank);

	return ERROR_OK;
}

static int virtual_auto_probe(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->auto_probe(master_bank);
	if (retval != ERROR_OK)
		return retval;

	/* update the info we do not have */
	virtual_update_bank_info(bank);

	return ERROR_OK;
}

static int virtual_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	command_print_sameline(cmd, "%s driver for flash bank %s at " TARGET_ADDR_FMT,
			bank->driver->name, master_bank->name, master_bank->base);

	return ERROR_OK;
}

static int virtual_blank_check(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler or default */
	if (master_bank->driver->erase_check)
		return master_bank->driver->erase_check(master_bank);

	return default_flash_blank_check(master_bank);
}

static int virtual_flash_read(struct flash_bank *bank,
		uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler or default */
	if (master_bank->driver->read)
		return master_bank->driver->read(master_bank, buffer, offset, count);

	return default_flash_read(master_bank, buffer, offset, count);
}

void virtual_flash_free_driver_priv(struct flash_bank *bank)
{
	free(bank->driver_priv);
	bank->driver_priv = NULL;

	/* For 'virtual' flash driver bank->sectors and bank->prot_blocks pointers are copied from
	 * master flash_bank structure. They point to memory locations allocated by master flash driver
	 * so master driver is responsible for releasing them.
	 * Avoid UB caused by double-free memory corruption if flash bank is 'virtual'. */
	bank->sectors = NULL;
	bank->prot_blocks = NULL;
}

const struct flash_driver virtual_flash = {
	.name = "virtual",
	.flash_bank_command = virtual_flash_bank_command,
	.erase = virtual_erase,
	.protect = virtual_protect,
	.write = virtual_write,
	.read = virtual_flash_read,
	.probe = virtual_probe,
	.auto_probe = virtual_auto_probe,
	.erase_check = virtual_blank_check,
	.protect_check = virtual_protect_check,
	.info = virtual_info,
	.free_driver_priv = virtual_flash_free_driver_priv,
};

FLASH_BANK_COMMAND_HANDLER(ro_alias_bank_command)
{
	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	// get the master flash bank
	const char *bank_name = CMD_ARGV[6];
	struct flash_bank *master_bank = get_flash_bank_by_name_noprobe(bank_name);

	if (!master_bank) {
		LOG_ERROR("master flash bank '%s' does not exist", bank_name);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	// save master bank name - use this to get settings later
	bank->driver_priv = strdup(bank_name);

	bank->read_only = true;

	return ERROR_OK;
}

static int ro_alias_erase(struct flash_bank *bank,
	unsigned int first, unsigned int last)
{
	char *bank_name = bank->driver_priv;

	LOG_ERROR("Erase of read-only flash alias refused. Use master flash bank '%s'",
			bank_name);

	return ERROR_FAIL;
}

static int ro_alias_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	char *bank_name = bank->driver_priv;

	LOG_ERROR("Write to read-only flash alias refused. Use master flash bank '%s'",
			bank_name);

	return ERROR_FAIL;
}

const struct flash_driver ro_alias_flash = {
	.name = "ro_alias",
	.flash_bank_command = ro_alias_bank_command,
	.erase = ro_alias_erase,
	.write = ro_alias_write,
	.read = virtual_flash_read,
	.probe = virtual_probe,
	.auto_probe = virtual_auto_probe,
	.erase_check = virtual_blank_check,
	.info = virtual_info,
	.free_driver_priv = virtual_flash_free_driver_priv,
};

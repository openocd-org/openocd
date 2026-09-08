// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2019 by stfnbr <stfnbr@disroot.org>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/bits.h>
#include <helper/time_support.h>

#define PAC55XX_FLASH_BASE		 0x00000000UL

#define PAC55XX_PERIPH_BASE		 0x40000000UL
#define PAC55XX_MEMCTL_BASE		 (PAC55XX_PERIPH_BASE + 0xD0000)

#define PAC55XX_MEMCTL_FLASHSTATUS (PAC55XX_MEMCTL_BASE + 0x04) // status to show if flash is busy writing or erasing
#define PAC55XX_MEMCTL_FLASHLOCK   (PAC55XX_MEMCTL_BASE + 0x08) // write a key to unlock the flash for write or erase
#define PAC55XX_MEMCTL_FLASHPAGE   (PAC55XX_MEMCTL_BASE + 0x0C) // select a flash page
#define PAC55XX_MEMCTL_FLASHERASE  (PAC55XX_MEMCTL_BASE + 0x20) // write a key to erase a flash page

#define PAC55XX_MEMCTL_FLASHSTATUS_ERASEBUSY	BIT(1)	// erase in progress
#define PAC55XX_MEMCTL_FLASHSTATUS_WRITEBUSY	BIT(0)	// write in progress

#define PAC55XX_MEMCTL_FLASHLOCK_ALLOWWRITEERASE	0x43DF140A
#define PAC55XX_MEMCTL_FLASHLOCK_LOCKED				0x0
#define PAC55XX_MEMCTL_FLASHERASE_ERASE				0x8C799CA7

// Flash timeout values in milliseconds.
#define FLASH_ERASE_TIMEOUT_MS	20	// vendor specifies 2ms maximum for erase of a single page -> choose 20 to be safe
#define FLASH_WRITE_TIMEOUT_MS	20	// no upper limit specified - 20 seems to work fine

/** ***********************************************************************************************
 * @brief Wait until the flash controller is not busy anymore.
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int pac55xx_wait_for_operation_to_finish(struct flash_bank *bank, unsigned int timeout_ms)
{
	const int64_t start_time = timeval_ms();

	while (true) {
		uint32_t status;

		// read status register
		int ret = target_read_u32(bank->target, PAC55XX_MEMCTL_FLASHSTATUS, &status);
		if (ret != ERROR_OK)
			return ret;

		// check the busy-bits
		if ((status & PAC55XX_MEMCTL_FLASHSTATUS_ERASEBUSY) == 0 &&
			(status & PAC55XX_MEMCTL_FLASHSTATUS_WRITEBUSY) == 0)
			return ERROR_OK;

		if ((timeval_ms() - start_time) > timeout_ms) {
			LOG_ERROR("Timed out waiting for flash");
			return ERROR_TIMEOUT_REACHED;
		}

		keep_alive();
	}
}

int pac55xx_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	// target must be halted
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target must be halted to erase.");
		return ERROR_TARGET_NOT_HALTED;
	}

	// Unlock flash
	int ret = target_write_u32(bank->target, PAC55XX_MEMCTL_FLASHLOCK, PAC55XX_MEMCTL_FLASHLOCK_ALLOWWRITEERASE);
	if (ret != ERROR_OK)
		goto flash_lock;

	// loop through the pages to erase
	for (unsigned int i = first; i < last; ++i) {
		// define which page to erase
		ret = target_write_u32(bank->target, PAC55XX_MEMCTL_FLASHPAGE, i);
		if (ret != ERROR_OK)
			goto flash_lock;

		// start erase operation
		ret = target_write_u32(bank->target, PAC55XX_MEMCTL_FLASHERASE, PAC55XX_MEMCTL_FLASHERASE_ERASE);
		if (ret != ERROR_OK)
			goto flash_lock;

		// wait until operation is finished
		ret = pac55xx_wait_for_operation_to_finish(bank, FLASH_ERASE_TIMEOUT_MS);
		if (ret != ERROR_OK)
			goto flash_lock;
	}

flash_lock:
	{
		// lock the flash
		int ret_lock_flash = target_write_u32(bank->target, PAC55XX_MEMCTL_FLASHLOCK, PAC55XX_MEMCTL_FLASHLOCK_LOCKED);
		if (ret == ERROR_OK)
			ret = ret_lock_flash;
	}

	return ret;
}

int pac55xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	// target must be halted
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target must be halted to write.");
		return ERROR_TARGET_NOT_HALTED;
	}

	// double check if start address is aligned to page size
	// Writing must start at an address aligned to 16 bytes
	assert(offset % 16 == 0);

	// double check if size is aligned to page size
	// Size of write operation must always be aligned to 16 bytes
	assert(count % 16 == 0);

	// unlock flash
	int ret = target_write_u32(bank->target, PAC55XX_MEMCTL_FLASHLOCK, PAC55XX_MEMCTL_FLASHLOCK_ALLOWWRITEERASE);
	if (ret != ERROR_OK)
		goto flash_lock;

	for (uint32_t flash_address = PAC55XX_FLASH_BASE + offset;
			flash_address < PAC55XX_FLASH_BASE + offset + count;
			flash_address += 16, buffer += 16) {
		// wait until flash is not busy anymore
		ret = pac55xx_wait_for_operation_to_finish(bank, FLASH_WRITE_TIMEOUT_MS);
		if (ret != ERROR_OK)
			goto flash_lock;

		// write 16 bytes to flash
		ret = target_write_memory(bank->target, flash_address, 4, 4, buffer);
		if (ret != ERROR_OK)
			goto flash_lock;
	}

	// wait until flash write operation finished
	ret = pac55xx_wait_for_operation_to_finish(bank, FLASH_WRITE_TIMEOUT_MS);
	if (ret != ERROR_OK)
		goto flash_lock;

flash_lock:
	{
		// lock the flash
		int ret_lock_flash = target_write_u32(bank->target, PAC55XX_MEMCTL_FLASHLOCK, PAC55XX_MEMCTL_FLASHLOCK_LOCKED);
		if (ret == ERROR_OK)
			ret = ret_lock_flash;
	}

	return ret;
}

int pac55xx_probe(struct flash_bank *bank)
{
	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(pac55xx_flash_bank_command)
{
	uint32_t num_pages = 128;
	uint32_t page_size = 0x400; // page size in bytes

	bank->base = PAC55XX_FLASH_BASE;
	bank->size = num_pages * page_size;
	bank->write_start_alignment = 16;
	bank->write_end_alignment = 16;
	bank->num_sectors = num_pages;

	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = 0;

	return ERROR_OK;
}

const struct flash_driver pac55xx_flash = {
	.name = "pac55xx",

	// const struct command_registration *commands;

	.flash_bank_command = pac55xx_flash_bank_command,
	.erase = pac55xx_erase,
	.write = pac55xx_write,
	.probe = pac55xx_probe,
	.read = default_flash_read,
	.erase_check = default_flash_blank_check,
	.auto_probe = pac55xx_probe,
	.free_driver_priv = default_flash_free_driver_priv,
};

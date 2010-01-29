/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
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

#include "imp.h"
#include "pic32mx.h"
#include <target/mips32.h>


static
struct pic32mx_devs_s {
	uint8_t	devid;
	char	*name;
	uint32_t	pfm_size;
} pic32mx_devs[] = {
	{ 0x78, "460F512L USB", 512 },
	{ 0x74, "460F256L USB", 256 },
	{ 0x6D, "440F128L USB", 128 },
	{ 0x56, "440F512H USB", 512 },
	{ 0x52, "440F256H USB", 256 },
	{ 0x4D, "440F128H USB", 128 },
	{ 0x42, "420F032H USB",  32 },
	{ 0x38, "360F512L",     512 },
	{ 0x34, "360F256L",     256 },
	{ 0x2D, "340F128L",     128 },
	{ 0x2A, "320F128L",     128 },
	{ 0x16, "340F512H",     512 },
	{ 0x12, "340F256H",     256 },
	{ 0x0D, "340F128H",     128 },
	{ 0x0A, "320F128H",     128 },
	{ 0x06, "320F064H",      64 },
	{ 0x02, "320F032H",      32 },
	{ 0x00, NULL, 0 }
};

static int pic32mx_write_row(struct flash_bank *bank, uint32_t address, uint32_t srcaddr);
static int pic32mx_write_word(struct flash_bank *bank, uint32_t address, uint32_t word);

/* flash bank pic32mx <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(pic32mx_flash_bank_command)
{
	struct pic32mx_flash_bank *pic32mx_info;

	if (CMD_ARGC < 6)
	{
		LOG_WARNING("incomplete flash_bank pic32mx configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	pic32mx_info = malloc(sizeof(struct pic32mx_flash_bank));
	bank->driver_priv = pic32mx_info;

	pic32mx_info->write_algorithm = NULL;
	pic32mx_info->probed = 0;

	return ERROR_OK;
}

static uint32_t pic32mx_get_flash_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;

	target_read_u32(target, PIC32MX_NVMCON, &status);

	return status;
}

static uint32_t pic32mx_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;

	/* wait for busy to clear */
	while (((status = pic32mx_get_flash_status(bank)) & NVMCON_NVMWR) && (timeout-- > 0))
	{
		LOG_DEBUG("status: 0x%" PRIx32, status);
		alive_sleep(1);
	}
	if (timeout <= 0)
		LOG_DEBUG("timeout: status: 0x%" PRIx32, status);

	return status;
}

static int pic32mx_nvm_exec(struct flash_bank *bank, uint32_t op, uint32_t timeout)
{
	struct target *target = bank->target;
	uint32_t status;

	target_write_u32(target, PIC32MX_NVMCON, NVMCON_NVMWREN | op);

	/* unlock flash registers */
	target_write_u32(target, PIC32MX_NVMKEY, NVMKEY1);
	target_write_u32(target, PIC32MX_NVMKEY, NVMKEY2);

	/* start operation */
	target_write_u32(target, PIC32MX_NVMCONSET, NVMCON_NVMWR);

	status = pic32mx_wait_status_busy(bank, timeout);

	/* lock flash registers */
	target_write_u32(target, PIC32MX_NVMCONCLR, NVMCON_NVMWREN);

	return status;
}

static int pic32mx_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;

	uint32_t devcfg0;
	int s;
	int num_pages;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_read_u32(target, PIC32MX_DEVCFG0, &devcfg0);
	if ((devcfg0 & (1 << 28)) == 0) /* code protect bit */
		num_pages = 0xffff;  /* All pages protected */
	else if (bank->base == PIC32MX_KSEG1_BOOT_FLASH)
	{
		if (devcfg0 & (1 << 24))
			num_pages = 0;       /* All pages unprotected */
		else
			num_pages = 0xffff;  /* All pages protected */
	}
	else /* pgm flash */
		num_pages = (~devcfg0 >> 12) & 0xff;
	for (s = 0; s < bank->num_sectors && s < num_pages; s++)
		bank->sectors[s].is_protected = 1;
	for (; s < bank->num_sectors; s++)
		bank->sectors[s].is_protected = 0;

	return ERROR_OK;
}

static int pic32mx_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;
	uint32_t status;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)) && (bank->base == PIC32MX_KSEG0_PGM_FLASH || bank->base == PIC32MX_KSEG1_PGM_FLASH))
	{
		LOG_DEBUG("Erasing entire program flash");
		status = pic32mx_nvm_exec(bank, NVMCON_OP_PFM_ERASE, 50);
		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		return ERROR_OK;
	}

	for (i = first; i <= last; i++)
	{
		if (bank->base >= PIC32MX_KSEG1_PGM_FLASH)
			target_write_u32(target, PIC32MX_NVMADDR, KS1Virt2Phys(bank->base + bank->sectors[i].offset));
		else
			target_write_u32(target, PIC32MX_NVMADDR, KS0Virt2Phys(bank->base + bank->sectors[i].offset));

		status = pic32mx_nvm_exec(bank, NVMCON_OP_PAGE_ERASE, 10);

		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int pic32mx_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct pic32mx_flash_bank *pic32mx_info = NULL;
	struct target *target = bank->target;
#if 0
	uint16_t prot_reg[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	int i, reg, bit;
	int status;
	uint32_t protection;
#endif

	pic32mx_info = bank->driver_priv;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

#if 0
	if ((first && (first % pic32mx_info->ppage_size)) || ((last + 1) && (last + 1) % pic32mx_info->ppage_size))
	{
		LOG_WARNING("sector start/end incorrect - stm32 has %dK sector protection", pic32mx_info->ppage_size);
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* medium density - each bit refers to a 4bank protection
	 * high density - each bit refers to a 2bank protection */
	target_read_u32(target, PIC32MX_FLASH_WRPR, &protection);

	prot_reg[0] = (uint16_t)protection;
	prot_reg[1] = (uint16_t)(protection >> 8);
	prot_reg[2] = (uint16_t)(protection >> 16);
	prot_reg[3] = (uint16_t)(protection >> 24);

	if (pic32mx_info->ppage_size == 2)
	{
		/* high density flash */

		/* bit 7 controls sector 62 - 255 protection */
		if (last > 61)
		{
			if (set)
				prot_reg[3] &= ~(1 << 7);
			else
				prot_reg[3] |= (1 << 7);
		}

		if (first > 61)
			first = 62;
		if (last > 61)
			last = 61;

		for (i = first; i <= last; i++)
		{
			reg = (i / pic32mx_info->ppage_size) / 8;
			bit = (i / pic32mx_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	}
	else
	{
		/* medium density flash */
		for (i = first; i <= last; i++)
		{
			reg = (i / pic32mx_info->ppage_size) / 8;
			bit = (i / pic32mx_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	}

	if ((status = pic32mx_erase_options(bank)) != ERROR_OK)
		return status;

	pic32mx_info->option_bytes.protection[0] = prot_reg[0];
	pic32mx_info->option_bytes.protection[1] = prot_reg[1];
	pic32mx_info->option_bytes.protection[2] = prot_reg[2];
	pic32mx_info->option_bytes.protection[3] = prot_reg[3];

	return pic32mx_write_options(bank);
#else
	return ERROR_OK;
#endif
}

static int pic32mx_write_block(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 512;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	int retval = ERROR_OK;
#if 0
	struct pic32mx_flash_bank *pic32mx_info = bank->driver_priv;
	struct armv7m_algorithm armv7m_info;

	uint8_t pic32mx_flash_write_code[] = {
									/* write: */
		0xDF, 0xF8, 0x24, 0x40,		/* ldr	r4, PIC32MX_FLASH_CR */
		0x09, 0x4D,					/* ldr	r5, PIC32MX_FLASH_SR */
		0x4F, 0xF0, 0x01, 0x03,		/* mov	r3, #1 */
		0x23, 0x60,					/* str	r3, [r4, #0] */
		0x30, 0xF8, 0x02, 0x3B,		/* ldrh r3, [r0], #2 */
		0x21, 0xF8, 0x02, 0x3B,		/* strh r3, [r1], #2 */
									/* busy: */
		0x2B, 0x68,					/* ldr 	r3, [r5, #0] */
		0x13, 0xF0, 0x01, 0x0F,		/* tst 	r3, #0x01 */
		0xFB, 0xD0,					/* beq 	busy */
		0x13, 0xF0, 0x14, 0x0F,		/* tst	r3, #0x14 */
		0x01, 0xD1,					/* bne	exit */
		0x01, 0x3A,					/* subs	r2, r2, #1 */
		0xED, 0xD1,					/* bne	write */
									/* exit: */
		0xFE, 0xE7,					/* b exit */
		0x10, 0x20, 0x02, 0x40,		/* PIC32MX_FLASH_CR:	.word 0x40022010 */
		0x0C, 0x20, 0x02, 0x40		/* PIC32MX_FLASH_SR:	.word 0x4002200C */
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(pic32mx_flash_write_code), &pic32mx_info->write_algorithm) != ERROR_OK)
	{
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	if ((retval = target_write_buffer(target, pic32mx_info->write_algorithm->address, sizeof(pic32mx_flash_write_code), pic32mx_flash_write_code)) != ERROR_OK)
		return retval;
#endif

	/* memory buffer */
	if (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
#if 0
		/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
		if (pic32mx_info->write_algorithm)
			target_free_working_area(target, pic32mx_info->write_algorithm);
#endif

		LOG_WARNING("no large enough working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	while (count >= buffer_size/4)
	{
		uint32_t status;

		if ((retval = target_write_buffer(target, source->address, buffer_size, buffer)) != ERROR_OK) {
			LOG_ERROR("Failed to write row buffer (%d words) to RAM", (int)(buffer_size/4));
			break;
		}

#if 0
		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, buffer_size/4);

		if ((retval = target_run_algorithm(target, 0, NULL, 4, reg_params, pic32mx_info->write_algorithm->address, \
				pic32mx_info->write_algorithm->address + (sizeof(pic32mx_flash_write_code) - 10), 10000, &armv7m_info)) != ERROR_OK)
		{
			LOG_ERROR("error executing pic32mx flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (buf_get_u32(reg_params[3].value, 0, 32) & 0x14)
		{
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
#endif
		status = pic32mx_write_row(bank, address, source->address);
		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer  += buffer_size;
		address += buffer_size;
		count   -= buffer_size/4;
	}

	target_free_working_area(target, source);

	while (count > 0)
	{
		uint32_t value;
		memcpy(&value, buffer, sizeof(uint32_t));

		uint32_t status = pic32mx_write_word(bank, address, value);
		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer  += 4;
		address += 4;
		count--;
	}

	return retval;
}

static int pic32mx_write_word(struct flash_bank *bank, uint32_t address, uint32_t word)
{
	struct target *target = bank->target;

	if (bank->base >= PIC32MX_KSEG1_PGM_FLASH)
		target_write_u32(target, PIC32MX_NVMADDR, KS1Virt2Phys(address));
	else
		target_write_u32(target, PIC32MX_NVMADDR, KS0Virt2Phys(address));
	target_write_u32(target, PIC32MX_NVMDATA, word);

	return pic32mx_nvm_exec(bank, NVMCON_OP_WORD_PROG, 5);
}

/*
 * Write a 128 word (512 byte) row to flash address from RAM srcaddr.
 */
static int pic32mx_write_row(struct flash_bank *bank, uint32_t address, uint32_t srcaddr)
{
	struct target *target = bank->target;

	LOG_DEBUG("addr: 0x%08" PRIx32 " srcaddr: 0x%08" PRIx32 "", address, srcaddr);

	if (address >= PIC32MX_KSEG1_PGM_FLASH)
		target_write_u32(target, PIC32MX_NVMADDR,    KS1Virt2Phys(address));
	else
		target_write_u32(target, PIC32MX_NVMADDR,    KS0Virt2Phys(address));
	if (srcaddr >= PIC32MX_KSEG1_RAM)
		target_write_u32(target, PIC32MX_NVMSRCADDR, KS1Virt2Phys(srcaddr));
	else
		target_write_u32(target, PIC32MX_NVMSRCADDR, KS0Virt2Phys(srcaddr));

	return pic32mx_nvm_exec(bank, NVMCON_OP_ROW_PROG, 100);
}

static int pic32mx_write(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_remaining = (count & 0x00000003);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	uint32_t status;
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3)
	{
		LOG_WARNING("offset 0x%" PRIx32 "breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* multiple words (4-byte) to be programmed? */
	if (words_remaining > 0)
	{
		/* try using a block write */
		if ((retval = pic32mx_write_block(bank, buffer, offset, words_remaining)) != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
			else if (retval == ERROR_FLASH_OPERATION_FAILED)
			{
				LOG_ERROR("flash writing failed with error code: 0x%x", retval);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
		else
		{
			buffer += words_remaining * 4;
			address += words_remaining * 4;
			words_remaining = 0;
		}
	}

	while (words_remaining > 0)
	{
		uint32_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint32_t));

		status = pic32mx_write_word(bank, address, value);
		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;

		bytes_written += 4;
		words_remaining--;
		address += 4;
	}

	if (bytes_remaining)
	{
		uint32_t value = 0xffffffff;
		memcpy(&value, buffer + bytes_written, bytes_remaining);

		status = pic32mx_write_word(bank, address, value);
		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int pic32mx_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct pic32mx_flash_bank *pic32mx_info = bank->driver_priv;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int i;
	uint16_t num_pages = 0;
	uint32_t device_id;
	int page_size;

	pic32mx_info->probed = 0;

	device_id = ejtag_info->idcode;
	LOG_INFO("device id = 0x%08" PRIx32 " (manuf 0x%03x dev 0x%02x, ver 0x%03x)",
			  device_id,
			  (unsigned)((device_id >> 1)&0x7ff),
			  (unsigned)((device_id >> 12)&0xff),
			  (unsigned)((device_id >> 20)&0xfff));

	if (((device_id >> 1)&0x7ff) != PIC32MX_MANUF_ID) {
		LOG_WARNING("Cannot identify target as a PIC32MX family.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	page_size = 4096;
	if (bank->base == PIC32MX_KSEG1_BOOT_FLASH || bank->base == 1) {
		/* 0xBFC00000: Boot flash size fixed at 12k */
		num_pages = 12;
	} else {
		/* 0xBD000000: Program flash size varies with device */
		for (i = 0; pic32mx_devs[i].name != NULL; i++)
			if (pic32mx_devs[i].devid == ((device_id >> 12) & 0xff)) {
				num_pages = pic32mx_devs[i].pfm_size;
				break;
			}
		if (pic32mx_devs[i].name == NULL) {
			LOG_WARNING("Cannot identify target as a PIC32MX family.");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

#if 0
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* get flash size from target */
	if (target_read_u16(target, 0x1FFFF7E0, &num_pages) != ERROR_OK)
	{
		/* failed reading flash size, default to max target family */
		num_pages = 0xffff;
	}
#endif

	LOG_INFO("flash size = %dkbytes", num_pages);

	/* calculate numbers of pages */
	num_pages /= (page_size / 1024);

	if (bank->base == 0) bank->base = PIC32MX_KSEG1_PGM_FLASH;
	if (bank->base == 1) bank->base = PIC32MX_KSEG1_BOOT_FLASH;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->chip_width = 4;
	bank->bus_width  = 4;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++)
	{
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	pic32mx_info->probed = 1;

	return ERROR_OK;
}

static int pic32mx_auto_probe(struct flash_bank *bank)
{
	struct pic32mx_flash_bank *pic32mx_info = bank->driver_priv;
	if (pic32mx_info->probed)
		return ERROR_OK;
	return pic32mx_probe(bank);
}

#if 0
COMMAND_HANDLER(pic32mx_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int pic32mx_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	uint32_t device_id;
	int printed = 0, i;

	device_id = ejtag_info->idcode;

	if (((device_id >> 1)&0x7ff) != PIC32MX_MANUF_ID) {
		snprintf(buf, buf_size,
				 "Cannot identify target as a PIC32MX family (manufacturer 0x%03d != 0x%03d)\n",
				 (unsigned)((device_id >> 1)&0x7ff),
				 PIC32MX_MANUF_ID);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	for (i = 0; pic32mx_devs[i].name != NULL; i++)
		if (pic32mx_devs[i].devid == ((device_id >> 12) & 0xff)) {
			printed = snprintf(buf, buf_size, "PIC32MX%s", pic32mx_devs[i].name);
			break;
		}
	if (pic32mx_devs[i].name == NULL) {
		snprintf(buf, buf_size, "Cannot identify target as a PIC32MX family\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	buf += printed;
	buf_size -= printed;
	printed = snprintf(buf, buf_size, "  Ver: 0x%03x",
					   (unsigned)((device_id >> 20)&0xfff));

	return ERROR_OK;
}

#if 0
COMMAND_HANDLER(pic32mx_handle_lock_command)
{
	struct target *target = NULL;
	struct pic32mx_flash_bank *pic32mx_info = NULL;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "pic32mx lock <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	pic32mx_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (pic32mx_erase_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "pic32mx failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	pic32mx_info->option_bytes.RDP = 0;

	if (pic32mx_write_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "pic32mx failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "pic32mx locked");

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mx_handle_unlock_command)
{
	struct target *target = NULL;
	struct pic32mx_flash_bank *pic32mx_info = NULL;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "pic32mx unlock <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	pic32mx_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (pic32mx_erase_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "pic32mx failed to unlock device");
		return ERROR_OK;
	}

	if (pic32mx_write_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "pic32mx failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "pic32mx unlocked");

	return ERROR_OK;
}
#endif

#if 0
static int pic32mx_chip_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
#if 0
	uint32_t status;
#endif

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("PIC32MX chip erase called");

#if 0
	/* unlock option flash registers */
	target_write_u32(target, PIC32MX_FLASH_KEYR, KEY1);
	target_write_u32(target, PIC32MX_FLASH_KEYR, KEY2);

	/* chip erase flash memory */
	target_write_u32(target, PIC32MX_FLASH_CR, FLASH_MER);
	target_write_u32(target, PIC32MX_FLASH_CR, FLASH_MER | FLASH_STRT);

	status = pic32mx_wait_status_busy(bank, 10);

	target_write_u32(target, PIC32MX_FLASH_CR, FLASH_LOCK);

	if (status & FLASH_WRPRTERR)
	{
		LOG_ERROR("pic32mx device protected");
		return ERROR_OK;
	}

	if (status & FLASH_PGERR)
	{
		LOG_ERROR("pic32mx device programming failed");
		return ERROR_OK;
	}
#endif

	return ERROR_OK;
}
#endif

COMMAND_HANDLER(pic32mx_handle_chip_erase_command)
{
#if 0
	int i;

	if (CMD_ARGC != 0)
	{
		command_print(CMD_CTX, "pic32mx chip_erase");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (pic32mx_chip_erase(bank) == ERROR_OK)
	{
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
		{
			bank->sectors[i].is_erased = 1;
		}

		command_print(CMD_CTX, "pic32mx chip erase complete");
	}
	else
	{
		command_print(CMD_CTX, "pic32mx chip erase failed");
	}
#endif

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mx_handle_pgm_word_command)
{
	uint32_t address, value;
	int status, res;

	if (CMD_ARGC != 3)
	{
		command_print(CMD_CTX, "pic32mx pgm_word <addr> <value> <bank>");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 2, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (address < bank->base || address >= (bank->base + bank->size))
	{
		command_print(CMD_CTX, "flash address '%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	res = ERROR_OK;
	status = pic32mx_write_word(bank, address, value);
	if (status & NVMCON_NVMERR)
		res = ERROR_FLASH_OPERATION_FAILED;
	if (status & NVMCON_LVDERR)
		res = ERROR_FLASH_OPERATION_FAILED;

	if (res == ERROR_OK)
		command_print(CMD_CTX, "pic32mx pgm word complete");
	else
		command_print(CMD_CTX, "pic32mx pgm word failed (status = 0x%x)", status);

	return ERROR_OK;
}
static const struct command_registration pic32mx_exec_command_handlers[] = {
	{
		.name = "chip_erase",
		.handler = pic32mx_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "erase device",
	},
	{
		.name = "pgm_word",
		.handler = pic32mx_handle_pgm_word_command,
		.mode = COMMAND_EXEC,
		.help = "program a word",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration pic32mx_command_handlers[] = {
	{
		.name = "pic32mx",
		.mode = COMMAND_ANY,
		.help = "pic32mx flash command group",
		.chain = pic32mx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver pic32mx_flash = {
	.name = "pic32mx",
	.commands = pic32mx_command_handlers,
	.flash_bank_command = pic32mx_flash_bank_command,
	.erase = pic32mx_erase,
	.protect = pic32mx_protect,
	.write = pic32mx_write,
	.probe = pic32mx_probe,
	.auto_probe = pic32mx_auto_probe,
	.erase_check = default_flash_mem_blank_check,
	.protect_check = pic32mx_protect_check,
	.info = pic32mx_info,
};

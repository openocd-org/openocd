/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                  *
 *   kesmtp@freenet.de                                                     *
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
#include "helper/binarybuffer.h"

static int kinetis_get_master_bank(struct flash_bank *bank,
				   struct flash_bank **master_bank)
{
	*master_bank = get_flash_bank_by_name_noprobe(bank->name);
	if (*master_bank == NULL) {
		LOG_ERROR("master flash bank '%s' does not exist",
			  (char *)bank->driver_priv);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int kinetis_update_bank_info(struct flash_bank *bank)
{
	int result;
	struct flash_bank *master_bank;

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	/* update the info we do not have */
	bank->size = master_bank->size;
	bank->chip_width = master_bank->chip_width;
	bank->bus_width = master_bank->bus_width;
	bank->num_sectors = master_bank->num_sectors;
	bank->sectors = master_bank->sectors;

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(kinetis_flash_bank_command)
{
	if (CMD_ARGC < 6) {
		LOG_ERROR("incomplete flash_bank kinetis configuration %d",
			  CMD_ARGC);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	LOG_INFO("add flash_bank kinetis %s", bank->name);

	return ERROR_OK;
}

static int kinetis_protect(struct flash_bank *bank, int set, int first,
			   int last)
{
	int result;
	struct flash_bank *master_bank;

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	LOG_WARNING("kinetis_protect not supported yet");

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

static int kinetis_protect_check(struct flash_bank *bank)
{
	int result;
	struct flash_bank *master_bank;
	uint8_t buffer[4];
	uint32_t fprot, psize, psec;
	int i, b;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	/* read protection register FTFL_FPROT */
	result = target_read_memory(bank->target, 0x40020010, 1, 4, buffer);

	if (result != ERROR_OK) {
		return result;
	}

	fprot = target_buffer_get_u32(bank->target, buffer);

	/* every bit protect 1/32 of the full flash */
	psize = bank->size / 32;
	psec = 0;
	b = 0;

	for (i = 0; i < bank->num_sectors; i++) {
		if ((fprot >> b) & 1)
			bank->sectors[i].is_protected = 0;
		else
			bank->sectors[i].is_protected = 1;

		psec += bank->sectors[i].size;

		if (psec >= psize) {
			psec = 0;
			b++;
		}
	}

	return ERROR_OK;
}

static int kinetis_ftfl_command(struct flash_bank *bank, uint32_t w0,
				uint32_t w1, uint32_t w2)
{
	uint8_t buffer[12];
	int result, i;

	/* wait for done */
	for (i = 0; i < 50; i++) {
		result =
		    target_read_memory(bank->target, 0x40020000, 1, 1, buffer);

		if (result != ERROR_OK) {
			return result;
		}

		if (buffer[0] & 0x80)
			break;

		buffer[0] = 0x00;
	}

	if (buffer[0] != 0x80) {
		/* reset error flags */
		buffer[0] = 0x30;
		result =
		    target_write_memory(bank->target, 0x40020000, 1, 1, buffer);
		if (result != ERROR_OK) {
			return result;
		}
	}

	target_buffer_set_u32(bank->target, buffer, w0);
	target_buffer_set_u32(bank->target, buffer + 4, w1);
	target_buffer_set_u32(bank->target, buffer + 8, w2);

	result = target_write_memory(bank->target, 0x40020004, 4, 3, buffer);

	if (result != ERROR_OK) {
		return result;
	}

	/* start command */
	buffer[0] = 0x80;
	result = target_write_memory(bank->target, 0x40020000, 1, 1, buffer);
	if (result != ERROR_OK) {
		return result;
	}

	/* wait for done */
	for (i = 0; i < 50; i++) {
		result =
		    target_read_memory(bank->target, 0x40020000, 1, 1, buffer);

		if (result != ERROR_OK) {
			return result;
		}

		if (buffer[0] & 0x80)
			break;

		buffer[0] = 0x00;
	}

	if (buffer[0] != 0x80) {
		LOG_ERROR
		    ("ftfl command failed FSTAT: %02X W0: %08X W1: %08X W2: %08X",
		     buffer[0], w0, w1, w2);

		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int kinetis_erase(struct flash_bank *bank, int first, int last)
{
	struct flash_bank *master_bank;
	int result, i;
	uint32_t w0 = 0, w1 = 0, w2 = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	if ((first > bank->num_sectors) || (last > bank->num_sectors)) {
		return ERROR_FLASH_OPERATION_FAILED;
	}

	for (i = first; i <= last; i++) {
		/* set command and sector address */
		w0 = (0x09 << 24) | bank->sectors[i].offset;

		result = kinetis_ftfl_command(bank, w0, w1, w2);

		if (result != ERROR_OK) {
			LOG_WARNING("erase sector %d failed", i);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bank->sectors[i].is_erased = 1;
	}

	if (first == 0) {
		LOG_WARNING
		    ("flash configuration field erased, please reset the device");
	}

	return ERROR_OK;
}

static int kinetis_write(struct flash_bank *bank, uint8_t * buffer,
			 uint32_t offset, uint32_t count)
{
	struct flash_bank *master_bank;
	unsigned int i, result, fallback = 0;
	uint8_t buf[8];
	uint32_t wc, w0 = 0, w1 = 0, w2 = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	/* make flex ram available */
	w0 = (0x81 << 24) | 0x00ff0000;

	result = kinetis_ftfl_command(bank, w0, w1, w2);

	if (result != ERROR_OK) {
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* check if ram ready */
	result = target_read_memory(bank->target, 0x40020001, 1, 1, buf);

	if (result != ERROR_OK) {
		return result;
	}

	if (!(buf[0] & (1 << 1))) {
		/* fallback to longword write */
		fallback = 1;

		LOG_WARNING
		    ("ram not ready, fallback to slow longword write (FCNFG: %02X)",
		     buf[0]);
	}

	/* program section command */
	if (fallback == 0) {
		for (i = 0; i < count; i += (2 * 1024)) {
			wc = 512;

			if ((count - i) < (2 * 1024)) {
				wc = count - i;
				wc /= 4;
			}

			LOG_DEBUG("write section @ %08X with length %d",
				  offset + i, wc * 4);

			/* write data to flexram */
			result =
			    target_write_memory(bank->target, 0x14000000, 4, wc,
						buffer + i);

			if (result != ERROR_OK) {
				LOG_ERROR("target_write_memory failed");

				return result;
			}

			/* execute section command */
			w0 = (0x0b << 24) | (offset + i);
			w1 = (256 << 16);

			result = kinetis_ftfl_command(bank, w0, w1, w2);

			if (result != ERROR_OK) {
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
	}
	/* program longword command */
	else {
		for (i = 0; i < count; i += 4) {
			LOG_DEBUG("write longword @ %08X", offset + i);

			w0 = (0x06 << 24) | (offset + i);
			w1 = buf_get_u32(buffer + offset + i, 0, 32);

			result = kinetis_ftfl_command(bank, w0, w1, w2);

			if (result != ERROR_OK) {
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
	}

	return ERROR_OK;
}

static int kinetis_probe(struct flash_bank *bank)
{
	struct flash_bank *master_bank;
	int result, i;
	uint8_t buf[4];
	uint32_t sim_sdid, sim_fcfg1, sim_fcfg2, offset = 0;
	uint32_t nvm_size, pf_size, flash_size, ee_size;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	result = target_read_memory(bank->target, 0x40048024, 1, 4, buf);
	if (result != ERROR_OK) {
		return result;
	}
	sim_sdid = target_buffer_get_u32(bank->target, buf);
	result = target_read_memory(bank->target, 0x4004804c, 1, 4, buf);
	if (result != ERROR_OK) {
		return result;
	}
	sim_fcfg1 = target_buffer_get_u32(bank->target, buf);
	result = target_read_memory(bank->target, 0x40048050, 1, 4, buf);
	if (result != ERROR_OK) {
		return result;
	}
	sim_fcfg2 = target_buffer_get_u32(bank->target, buf);

	LOG_DEBUG("SDID: %08X FCFG1: %08X FCFG2: %08X", sim_sdid, sim_fcfg1,
		  sim_fcfg2);

	switch ((sim_fcfg1 >> 28) & 0x0f) {
	case 0x07:
		nvm_size = 128 * 1024;
		break;
	case 0x09:
	case 0x0f:
		nvm_size = 256 * 1024;
		break;
	default:
		nvm_size = 0;
		break;
	}

	switch ((sim_fcfg1 >> 24) & 0x0f) {
	case 0x07:
		pf_size = 128 * 1024;
		break;
	case 0x09:
		pf_size = 256 * 1024;
		break;
	case 0x0b:
	case 0x0f:
		pf_size = 512 * 1024;
		break;
	default:
		pf_size = 0;
		break;
	}

	/* pf_size is the total size */
	flash_size = pf_size - nvm_size;

	switch ((sim_fcfg1 >> 16) & 0x0f) {
	case 0x02:
		ee_size = 4 * 1024;
		break;
	case 0x03:
		ee_size = 2 * 1024;
		break;
	case 0x04:
		ee_size = 1 * 1024;
		break;
	case 0x05:
		ee_size = 512;
		break;
	case 0x06:
		ee_size = 256;
		break;
	case 0x07:
		ee_size = 128;
		break;
	case 0x08:
		ee_size = 64;
		break;
	case 0x09:
		ee_size = 32;
		break;
	default:
		ee_size = 0;
		break;
	}

	LOG_DEBUG("NVM: %d PF: %d EE: %d BL1: %d", nvm_size, pf_size, ee_size,
		  (sim_fcfg2 >> 23) & 1);

	if (flash_size != bank->size) {
		LOG_WARNING("flash size is different %d != %d", flash_size,
			    bank->size);
	}

	bank->num_sectors = bank->size / (2 * 1024);
	assert(bank->num_sectors > 0);
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = 2 * 1024;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	/* update the info we do not have */
	return kinetis_update_bank_info(bank);
}

static int kinetis_auto_probe(struct flash_bank *bank)
{
	return kinetis_probe(bank);
}

static int kinetis_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int result;
	struct flash_bank *master_bank;

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	snprintf(buf, buf_size,
		 "%s driver for flash bank %s at 0x%8.8" PRIx32 "",
		 bank->driver->name, master_bank->name, master_bank->base);

	return ERROR_OK;
}

static int kinetis_blank_check(struct flash_bank *bank)
{
	int result;
	struct flash_bank *master_bank;

	LOG_WARNING("kinetis_blank_check not supported yet");

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	return ERROR_OK;
}

static int kinetis_flash_read(struct flash_bank *bank,
			      uint8_t * buffer, uint32_t offset, uint32_t count)
{
	int result;
	struct flash_bank *master_bank;

	LOG_WARNING("kinetis_flash_read not supported yet");

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = kinetis_get_master_bank(bank, &master_bank);

	if (result != ERROR_OK) {
		return result;
	}

	return ERROR_OK;
}

struct flash_driver kinetis_flash = {
	.name = "kinetis",
	.flash_bank_command = kinetis_flash_bank_command,
	.erase = kinetis_erase,
	.protect = kinetis_protect,
	.write = kinetis_write,
	.read = kinetis_flash_read,
	.probe = kinetis_probe,
	.auto_probe = kinetis_auto_probe,
	.erase_check = kinetis_blank_check,
	.protect_check = kinetis_protect_check,
	.info = kinetis_info,
};

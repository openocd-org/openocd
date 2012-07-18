/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   kesmtp@freenet.de                                                     *
 *                                                                         *
 *   Copyright (C) 2011 sleep(5) ltd                                       *
 *   tomas@sleepfive.com                                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Christopher D. Kilgour                          *
 *   techie at whiterocker.com                                             *
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

/*
 * Implementation Notes
 *
 * The persistent memories in the Kinetis chip families K10 through
 * K70 are all manipulated with the Flash Memory Module.  Some
 * variants call this module the FTFE, others call it the FTFL.  To
 * indicate that both are considered here, we use FTFX.
 *
 * Within the module, according to the chip variant, the persistent
 * memory is divided into what Freescale terms Program Flash, FlexNVM,
 * and FlexRAM.  All chip variants have Program Flash.  Some chip
 * variants also have FlexNVM and FlexRAM, which always appear
 * together.
 *
 * A given Kinetis chip may have 2 or 4 blocks of flash.  Here we map
 * each block to a separate bank.  Each block size varies by chip and
 * may be determined by the read-only SIM_FCFG1 register.  The sector
 * size within each bank/block varies by the chip granularity as
 * described below.
 *
 * Kinetis offers four different of flash granularities applicable
 * across the chip families.  The granularity is apparently reflected
 * by at least the reference manual suffix.  For example, for chip
 * MK60FN1M0VLQ12, reference manual K60P144M150SF3RM ends in "SF3RM",
 * where the "3" indicates there are four flash blocks with 4kiB
 * sectors.  All possible granularities are indicated below.
 *
 * The first half of the flash (1 or 2 blocks, depending on the
 * granularity) is always Program Flash and always starts at address
 * 0x00000000.  The "PFLSH" flag, bit 23 of the read-only SIM_FCFG2
 * register, determines whether the second half of the flash is also
 * Program Flash or FlexNVM+FlexRAM.  When PFLSH is set, the second
 * half of flash is Program Flash and is contiguous in the memory map
 * from the first half.  When PFLSH is clear, the second half of flash
 * is FlexNVM and always starts at address 0x10000000.  FlexRAM, which
 * is also present when PFLSH is clear, always starts at address
 * 0x14000000.
 *
 * The Flash Memory Module provides a register set where flash
 * commands are loaded to perform flash operations like erase and
 * program.  Different commands are available depending on whether
 * Program Flash or FlexNVM/FlexRAM is being manipulated.  Although
 * the commands used are quite consistent between flash blocks, the
 * parameters they accept differ according to the flash granularity.
 * Some Kinetis chips have different granularity between Program Flash
 * and FlexNVM/FlexRAM, so flash command arguments may differ between
 * blocks in the same chip.
 *
 * Although not documented as such by Freescale, it appears that bits
 * 8:7 of the read-only SIM_SDID register reflect the granularity
 * settings 0..3, so sector sizes and block counts are applicable
 * according to the following table.
 */
const struct {
	unsigned pflash_sector_size_bytes;
	unsigned nvm_sector_size_bytes;
	unsigned num_blocks;
} kinetis_flash_params[4] = {
	{ 1<<10, 1<<10, 2 },
	{ 2<<10, 1<<10, 2 },
	{ 2<<10, 2<<10, 2 },
	{ 4<<10, 4<<10, 4 }
};

struct kinetis_flash_bank {
	unsigned granularity;
	unsigned bank_ordinal;
	uint32_t sector_size;
	uint32_t protection_size;

	uint32_t sim_sdid;
	uint32_t sim_fcfg1;
	uint32_t sim_fcfg2;

	enum {
		FC_AUTO = 0,
		FC_PFLASH,
		FC_FLEX_NVM,
		FC_FLEX_RAM,
	} flash_class;
};

FLASH_BANK_COMMAND_HANDLER(kinetis_flash_bank_command)
{
	struct kinetis_flash_bank *bank_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_INFO("add flash_bank kinetis %s", bank->name);

	bank_info = malloc(sizeof(struct kinetis_flash_bank));

	memset(bank_info, 0, sizeof(struct kinetis_flash_bank));

	bank->driver_priv = bank_info;

	return ERROR_OK;
}

static int kinetis_protect(struct flash_bank *bank, int set, int first,
			   int last)
{
	LOG_WARNING("kinetis_protect not supported yet");
	/* FIXME: TODO */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_FLASH_BANK_INVALID;
}

static int kinetis_protect_check(struct flash_bank *bank)
{
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (kinfo->flash_class == FC_PFLASH) {
		int result;
		uint8_t buffer[4];
		uint32_t fprot, psec;
		int i, b;

		/* read protection register FTFx_FPROT */
		result = target_read_memory(bank->target, 0x40020010, 1, 4, buffer);

		if (result != ERROR_OK)
			return result;

		fprot = target_buffer_get_u32(bank->target, buffer);

		/*
		 * Every bit protects 1/32 of the full flash (not necessarily
		 * just this bank), but we enforce the bank ordinals for
		 * PFlash to start at zero.
		 */
		b = kinfo->bank_ordinal * (bank->size / kinfo->protection_size);
		for (psec = 0, i = 0; i < bank->num_sectors; i++) {
			if ((fprot >> b) & 1)
				bank->sectors[i].is_protected = 0;
			else
				bank->sectors[i].is_protected = 1;

			psec += bank->sectors[i].size;

			if (psec >= kinfo->protection_size) {
				psec = 0;
				b++;
			}
		}
	} else {
		LOG_ERROR("Protection checks for FlexNVM not yet supported");
		return ERROR_FLASH_BANK_INVALID;
	}

	return ERROR_OK;
}

static int kinetis_ftfx_command(struct flash_bank *bank, uint32_t w0,
				uint32_t w1, uint32_t w2, uint8_t *ftfx_fstat)
{
	uint8_t buffer[12];
	int result, i;

	/* wait for done */
	for (i = 0; i < 50; i++) {
		result =
			target_read_memory(bank->target, 0x40020000, 1, 1, buffer);

		if (result != ERROR_OK)
			return result;

		if (buffer[0] & 0x80)
			break;

		buffer[0] = 0x00;
	}

	if (buffer[0] != 0x80) {
		/* reset error flags */
		buffer[0] = 0x30;
		result =
			target_write_memory(bank->target, 0x40020000, 1, 1, buffer);
		if (result != ERROR_OK)
			return result;
	}

	target_buffer_set_u32(bank->target, buffer, w0);
	target_buffer_set_u32(bank->target, buffer + 4, w1);
	target_buffer_set_u32(bank->target, buffer + 8, w2);

	result = target_write_memory(bank->target, 0x40020004, 4, 3, buffer);

	if (result != ERROR_OK)
		return result;

	/* start command */
	buffer[0] = 0x80;
	result = target_write_memory(bank->target, 0x40020000, 1, 1, buffer);
	if (result != ERROR_OK)
		return result;

	/* wait for done */
	for (i = 0; i < 50; i++) {
		result =
			target_read_memory(bank->target, 0x40020000, 1, 1, ftfx_fstat);

		if (result != ERROR_OK)
			return result;

		if (*ftfx_fstat & 0x80)
			break;
	}

	if ((*ftfx_fstat & 0xf0) != 0x80) {
		LOG_ERROR
			("ftfx command failed FSTAT: %02X W0: %08X W1: %08X W2: %08X",
			 *ftfx_fstat, w0, w1, w2);

		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int kinetis_erase(struct flash_bank *bank, int first, int last)
{
	int result, i;
	uint32_t w0 = 0, w1 = 0, w2 = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first > bank->num_sectors) || (last > bank->num_sectors))
		return ERROR_FLASH_OPERATION_FAILED;

	/*
	 * FIXME: TODO: use the 'Erase Flash Block' command if the
	 * requested erase is PFlash or NVM and encompasses the entire
	 * block.  Should be quicker.
	 */
	for (i = first; i <= last; i++) {
		uint8_t ftfx_fstat;
		/* set command and sector address */
		w0 = (0x09 << 24) | (bank->base + bank->sectors[i].offset);

		result = kinetis_ftfx_command(bank, w0, w1, w2, &ftfx_fstat);

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

static int kinetis_write(struct flash_bank *bank, uint8_t *buffer,
			 uint32_t offset, uint32_t count)
{
	unsigned int i, result, fallback = 0;
	uint8_t buf[8];
	uint32_t wc, w0 = 0, w1 = 0, w2 = 0;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (kinfo->flash_class == FC_FLEX_NVM) {
		uint8_t ftfx_fstat;

		LOG_DEBUG("flash write into FlexNVM @%08X", offset);

		/* make flex ram available */
		w0 = (0x81 << 24) | 0x00ff0000;

		result = kinetis_ftfx_command(bank, w0, w1, w2, &ftfx_fstat);

		if (result != ERROR_OK)
			return ERROR_FLASH_OPERATION_FAILED;

		/* check if ram ready */
		result = target_read_memory(bank->target, 0x40020001, 1, 1, buf);

		if (result != ERROR_OK)
			return result;

		if (!(buf[0] & (1 << 1))) {
			/* fallback to longword write */
			fallback = 1;

			LOG_WARNING("ram not ready, fallback to slow longword write (FCNFG: %02X)",
				    buf[0]);
		}
	} else {
		LOG_DEBUG("flash write into PFLASH @08%X", offset);
	}


	/* program section command */
	if (fallback == 0) {
		unsigned prog_section_bytes = kinfo->sector_size >> 8;
		for (i = 0; i < count; i += kinfo->sector_size) {
			/*
			 * The largest possible Kinetis "section" is
			 * 16 bytes.  A full Kinetis sector is always
			 * 256 "section"s.
			 */
			uint8_t residual_buffer[16];
			uint8_t ftfx_fstat;
			uint32_t section_count = 256;
			uint32_t residual_wc = 0;

			/*
			 * Assume the word count covers an entire
			 * sector.
			 */
			wc = kinfo->sector_size / 4;

			/*
			 * If bytes to be programmed are less than the
			 * full sector, then determine the number of
			 * full-words to program, and put together the
			 * residual buffer so that a full "section"
			 * may always be programmed.
			 */
			if ((count - i) < kinfo->sector_size) {
				/* number of bytes to program beyond full section */
				unsigned residual_bc = (count-i) % prog_section_bytes;

				/* number of complete words to copy directly from buffer */
				wc = (count - i) / 4;

				/* number of total sections to write, including residual */
				section_count = DIV_ROUND_UP((count-i), prog_section_bytes);

				/* any residual bytes delivers a whole residual section */
				residual_wc = (residual_bc ? prog_section_bytes : 0)/4;

				/* clear residual buffer then populate residual bytes */
				(void) memset(residual_buffer, 0xff, prog_section_bytes);
				(void) memcpy(residual_buffer, &buffer[i+4*wc], residual_bc);
			}

			LOG_DEBUG("write section @ %08X with length %d bytes",
				  offset + i, (count - i));

			/* write data to flexram as whole-words */
			result = target_write_memory(bank->target, 0x14000000, 4, wc,
						     buffer + i);

			if (result != ERROR_OK) {
				LOG_ERROR("target_write_memory failed");
				return result;
			}

			/* write the residual words to the flexram */
			if (residual_wc) {
				result = target_write_memory(bank->target,
							     0x14000000+4*wc,
							     4, residual_wc,
							     residual_buffer);

				if (result != ERROR_OK) {
					LOG_ERROR("target_write_memory failed");
					return result;
				}
			}

			/* execute section-write command */
			w0 = (0x0b << 24) | (bank->base + offset + i);
			w1 = section_count << 16;

			result = kinetis_ftfx_command(bank, w0, w1, w2, &ftfx_fstat);

			if (result != ERROR_OK)
				return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	/* program longword command, not supported in "SF3" devices */
	else if (kinfo->granularity != 3) {
		for (i = 0; i < count; i += 4) {
			uint8_t ftfx_fstat;

			LOG_DEBUG("write longword @ %08X", offset + i);

			w0 = (0x06 << 24) | (bank->base + offset + i);
			if (count - i < 4) {
				uint32_t padding = 0xffffffff;
				memcpy(&padding, buffer + i, count - i);
				w1 = buf_get_u32(&padding, 0, 32);
			} else {
				w1 = buf_get_u32(buffer + i, 0, 32);
			}

			result = kinetis_ftfx_command(bank, w0, w1, w2, &ftfx_fstat);

			if (result != ERROR_OK)
				return ERROR_FLASH_OPERATION_FAILED;
		}
	} else {
		LOG_ERROR("Flash write strategy not implemented");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int kinetis_read_part_info(struct flash_bank *bank)
{
	int result, i;
	uint8_t buf[4];
	uint32_t offset = 0;
	uint8_t fcfg1_nvmsize, fcfg1_pfsize, fcfg1_eesize, fcfg2_pflsh;
	uint32_t nvm_size = 0, pf_size = 0, ee_size = 0;
	unsigned granularity, num_blocks = 0, num_pflash_blocks = 0, num_nvm_blocks = 0,
		first_nvm_bank = 0, reassign = 0;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	result = target_read_memory(bank->target, 0x40048024, 1, 4, buf);
	if (result != ERROR_OK)
		return result;
	kinfo->sim_sdid = target_buffer_get_u32(bank->target, buf);
	granularity = (kinfo->sim_sdid >> 7) & 0x03;
	result = target_read_memory(bank->target, 0x4004804c, 1, 4, buf);
	if (result != ERROR_OK)
		return result;
	kinfo->sim_fcfg1 = target_buffer_get_u32(bank->target, buf);
	result = target_read_memory(bank->target, 0x40048050, 1, 4, buf);
	if (result != ERROR_OK)
		return result;
	kinfo->sim_fcfg2 = target_buffer_get_u32(bank->target, buf);
	fcfg2_pflsh = (kinfo->sim_fcfg2 >> 23) & 0x01;

	LOG_DEBUG("SDID: %08X FCFG1: %08X FCFG2: %08X", kinfo->sim_sdid,
		  kinfo->sim_fcfg1, kinfo->sim_fcfg2);

	fcfg1_nvmsize = (uint8_t)((kinfo->sim_fcfg1 >> 28) & 0x0f);
	fcfg1_pfsize = (uint8_t)((kinfo->sim_fcfg1 >> 24) & 0x0f);
	fcfg1_eesize = (uint8_t)((kinfo->sim_fcfg1 >> 16) & 0x0f);

	/* when the PFLSH bit is set, there is no FlexNVM/FlexRAM */
	if (!fcfg2_pflsh) {
		switch (fcfg1_nvmsize) {
		case 0x03:
		case 0x07:
		case 0x09:
		case 0x0b:
			nvm_size = 1 << (14 + (fcfg1_nvmsize >> 1));
			break;
		case 0x0f:
			if (granularity == 3)
				nvm_size = 512<<10;
			else
				nvm_size = 256<<10;
			break;
		default:
			nvm_size = 0;
			break;
		}

		switch (fcfg1_eesize) {
		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
		case 0x08:
		case 0x09:
			ee_size = (16 << (10 - fcfg1_eesize));
			break;
		default:
			ee_size = 0;
			break;
		}
	}

	switch (fcfg1_pfsize) {
	case 0x03:
	case 0x05:
	case 0x07:
	case 0x09:
	case 0x0b:
	case 0x0d:
		pf_size = 1 << (14 + (fcfg1_pfsize >> 1));
		break;
	case 0x0f:
		if (granularity == 3)
			pf_size = 1024<<10;
		else if (fcfg2_pflsh)
			pf_size = 512<<10;
		else
			pf_size = 256<<10;
		break;
	default:
		pf_size = 0;
		break;
	}

	LOG_DEBUG("FlexNVM: %d PFlash: %d FlexRAM: %d PFLSH: %d",
		  nvm_size, pf_size, ee_size, fcfg2_pflsh);

	num_blocks = kinetis_flash_params[granularity].num_blocks;
	num_pflash_blocks = num_blocks / (2 - fcfg2_pflsh);
	first_nvm_bank = num_pflash_blocks;
	num_nvm_blocks = num_blocks - num_pflash_blocks;

	LOG_DEBUG("%d blocks total: %d PFlash, %d FlexNVM",
		  num_blocks, num_pflash_blocks, num_nvm_blocks);

	/*
	 * If the flash class is already assigned, verify the
	 * parameters.
	 */
	if (kinfo->flash_class != FC_AUTO) {
		if (kinfo->bank_ordinal != (unsigned) bank->bank_number) {
			LOG_WARNING("Flash ordinal/bank number mismatch");
			reassign = 1;
		} else if (kinfo->granularity != granularity) {
			LOG_WARNING("Flash granularity mismatch");
			reassign = 1;
		} else {
			switch (kinfo->flash_class) {
			case FC_PFLASH:
				if (kinfo->bank_ordinal >= first_nvm_bank) {
					LOG_WARNING("Class mismatch, bank %d is not PFlash",
						    bank->bank_number);
					reassign = 1;
				} else if (bank->size != (pf_size / num_pflash_blocks)) {
					LOG_WARNING("PFlash size mismatch");
					reassign = 1;
				} else if (bank->base !=
					 (0x00000000 + bank->size * kinfo->bank_ordinal)) {
					LOG_WARNING("PFlash address range mismatch");
					reassign = 1;
				} else if (kinfo->sector_size !=
					 kinetis_flash_params[granularity].pflash_sector_size_bytes) {
					LOG_WARNING("PFlash sector size mismatch");
					reassign = 1;
				} else {
					LOG_DEBUG("PFlash bank %d already configured okay",
						  kinfo->bank_ordinal);
				}
				break;
			case FC_FLEX_NVM:
				if ((kinfo->bank_ordinal >= num_blocks) ||
				    (kinfo->bank_ordinal < first_nvm_bank)) {
					LOG_WARNING("Class mismatch, bank %d is not FlexNVM",
						    bank->bank_number);
					reassign = 1;
				} else if (bank->size != (nvm_size / num_nvm_blocks)) {
					LOG_WARNING("FlexNVM size mismatch");
					reassign = 1;
				} else if (bank->base !=
					 (0x10000000 + bank->size * kinfo->bank_ordinal)) {
					LOG_WARNING("FlexNVM address range mismatch");
					reassign = 1;
				} else if (kinfo->sector_size !=
					 kinetis_flash_params[granularity].nvm_sector_size_bytes) {
					LOG_WARNING("FlexNVM sector size mismatch");
					reassign = 1;
				} else {
					LOG_DEBUG("FlexNVM bank %d already configured okay",
						  kinfo->bank_ordinal);
				}
				break;
			case FC_FLEX_RAM:
				if (kinfo->bank_ordinal != num_blocks) {
					LOG_WARNING("Class mismatch, bank %d is not FlexRAM",
						    bank->bank_number);
					reassign = 1;
				} else if (bank->size != ee_size) {
					LOG_WARNING("FlexRAM size mismatch");
					reassign = 1;
				} else if (bank->base != 0x14000000) {
					LOG_WARNING("FlexRAM address mismatch");
					reassign = 1;
				} else if (kinfo->sector_size !=
					 kinetis_flash_params[granularity].nvm_sector_size_bytes) {
					LOG_WARNING("FlexRAM sector size mismatch");
					reassign = 1;
				} else {
					LOG_DEBUG("FlexRAM bank %d already configured okay",
						  kinfo->bank_ordinal);
				}
				break;

			default:
				LOG_WARNING("Unknown or inconsistent flash class");
				reassign = 1;
				break;
			}
		}
	} else {
		LOG_INFO("Probing flash info for bank %d", bank->bank_number);
		reassign = 1;
	}

	if (!reassign)
		return ERROR_OK;

	kinfo->granularity = granularity;

	if ((unsigned)bank->bank_number < num_pflash_blocks) {
		/* pflash, banks start at address zero */
		kinfo->flash_class = FC_PFLASH;
		bank->size = (pf_size / num_pflash_blocks);
		bank->base = 0x00000000 + bank->size * bank->bank_number;
		kinfo->sector_size = kinetis_flash_params[granularity].pflash_sector_size_bytes;
		kinfo->protection_size = pf_size / 32;
	} else if ((unsigned)bank->bank_number < num_blocks) {
		/* nvm, banks start at address 0x10000000 */
		kinfo->flash_class = FC_FLEX_NVM;
		bank->size = (nvm_size / num_nvm_blocks);
		bank->base = 0x10000000 + bank->size * (bank->bank_number - first_nvm_bank);
		kinfo->sector_size = kinetis_flash_params[granularity].nvm_sector_size_bytes;
		kinfo->protection_size = 0; /* FIXME: TODO: depends on DEPART bits, chip */
	} else if ((unsigned)bank->bank_number == num_blocks) {
		LOG_ERROR("FlexRAM support not yet implemented");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	} else {
		LOG_ERROR("Cannot determine parameters for bank %d, only %d banks on device",
			  bank->bank_number, num_blocks);
		return ERROR_FLASH_BANK_INVALID;
	}

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->num_sectors = bank->size / kinfo->sector_size;
	assert(bank->num_sectors > 0);
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = kinfo->sector_size;
		offset += kinfo->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	return ERROR_OK;
}

static int kinetis_probe(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	return kinetis_read_part_info(bank);
}

static int kinetis_auto_probe(struct flash_bank *bank)
{
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	if (kinfo->sim_sdid)
		return ERROR_OK;

	return kinetis_probe(bank);
}

static int kinetis_info(struct flash_bank *bank, char *buf, int buf_size)
{
	const char *bank_class_names[] = {
		"(ANY)", "PFlash", "FlexNVM", "FlexRAM"
	};

	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	(void) snprintf(buf, buf_size,
			"%s driver for %s flash bank %s at 0x%8.8" PRIx32 "",
			bank->driver->name, bank_class_names[kinfo->flash_class],
			bank->name, bank->base);

	return ERROR_OK;
}

static int kinetis_blank_check(struct flash_bank *bank)
{
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (kinfo->flash_class == FC_PFLASH) {
		int result;
		uint32_t w0 = 0, w1 = 0, w2 = 0;
		uint8_t ftfx_fstat;

		/* check if whole bank is blank */
		w0 = (0x00 << 24) | bank->base;
		w1 = 0; /* "normal margin" */

		result = kinetis_ftfx_command(bank, w0, w1, w2, &ftfx_fstat);

		if (result != ERROR_OK)
			return result;

		if (ftfx_fstat & 0x01) {
			/* the whole bank is not erased, check sector-by-sector */
			int i;
			for (i = 0; i < bank->num_sectors; i++) {
				w0 = (0x01 << 24) | (bank->base + bank->sectors[i].offset);
				w1 = (0x100 << 16) | 0; /* normal margin */

				result = kinetis_ftfx_command(bank, w0, w1, w2, &ftfx_fstat);

				if (result == ERROR_OK) {
					bank->sectors[i].is_erased = !(ftfx_fstat & 0x01);
				} else {
					LOG_DEBUG("Ignoring errored PFlash sector blank-check");
					bank->sectors[i].is_erased = -1;
				}
			}
		} else {
			/* the whole bank is erased, update all sectors */
			int i;
			for (i = 0; i < bank->num_sectors; i++)
				bank->sectors[i].is_erased = 1;
		}
	} else {
		LOG_WARNING("kinetis_blank_check not supported yet for FlexNVM");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int kinetis_flash_read(struct flash_bank *bank,
			      uint8_t *buffer, uint32_t offset, uint32_t count)
{
	LOG_WARNING("kinetis_flash_read not supported yet");

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_FLASH_OPERATION_FAILED;
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

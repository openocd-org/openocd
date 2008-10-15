/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "replacements.h"

#include "cfi.h"
#include "non_cfi.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv4_5.h"
#include "algorithm.h"
#include "binarybuffer.h"
#include "types.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int cfi_register_commands(struct command_context_s *cmd_ctx);
int cfi_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int cfi_erase(struct flash_bank_s *bank, int first, int last);
int cfi_protect(struct flash_bank_s *bank, int set, int first, int last);
int cfi_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int cfi_probe(struct flash_bank_s *bank);
int cfi_auto_probe(struct flash_bank_s *bank);
int cfi_protect_check(struct flash_bank_s *bank);
int cfi_info(struct flash_bank_s *bank, char *buf, int buf_size);

int cfi_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

#define CFI_MAX_BUS_WIDTH	4
#define CFI_MAX_CHIP_WIDTH	4

/* defines internal maximum size for code fragment in cfi_intel_write_block() */
#define CFI_MAX_INTEL_CODESIZE 256

flash_driver_t cfi_flash =
{
	.name = "cfi",
	.register_commands = cfi_register_commands,
	.flash_bank_command = cfi_flash_bank_command,
	.erase = cfi_erase,
	.protect = cfi_protect,
	.write = cfi_write,
	.probe = cfi_probe,
	.auto_probe = cfi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = cfi_protect_check,
	.info = cfi_info
};

cfi_unlock_addresses_t cfi_unlock_addresses[] =
{
	[CFI_UNLOCK_555_2AA] = { .unlock1 = 0x555, .unlock2 = 0x2aa },
	[CFI_UNLOCK_5555_2AAA] = { .unlock1 = 0x5555, .unlock2 = 0x2aaa },
};

/* CFI fixups foward declarations */
void cfi_fixup_0002_erase_regions(flash_bank_t *flash, void *param);
void cfi_fixup_0002_unlock_addresses(flash_bank_t *flash, void *param);
void cfi_fixup_atmel_reversed_erase_regions(flash_bank_t *flash, void *param);

/* fixup after identifying JEDEC manufactuer and ID */
cfi_fixup_t cfi_jedec_fixups[] = {
	{CFI_MFR_SST, 0x00D4, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_SST, 0x00D5, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_SST, 0x00D6, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_SST, 0x00D7, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_SST, 0x2780, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_ST, 0x00D5, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_ST, 0x00D6, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_AMD, 0x2223, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_AMD, 0x22ab, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_FUJITSU, 0x226b, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_AMIC, 0xb31a, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_MX, 0x225b, cfi_fixup_non_cfi, NULL},
	{CFI_MFR_AMD, 0x225b, cfi_fixup_non_cfi, NULL},
	{0, 0, NULL, NULL}
};

/* fixup after reading cmdset 0002 primary query table */
cfi_fixup_t cfi_0002_fixups[] = {
	{CFI_MFR_SST, 0x00D4, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x00D5, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x00D6, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x00D7, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x2780, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_ATMEL, 0x00C8, cfi_fixup_atmel_reversed_erase_regions, NULL},
	{CFI_MFR_FUJITSU, 0x226b, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_AMIC, 0xb31a, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_MX, 0x225b, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_AMD, 0x225b, cfi_fixup_0002_unlock_addresses, &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_ANY, CFI_ID_ANY, cfi_fixup_0002_erase_regions, NULL},
	{0, 0, NULL, NULL}
};

/* fixup after reading cmdset 0001 primary query table */
cfi_fixup_t cfi_0001_fixups[] = {
	{0, 0, NULL, NULL}
};

void cfi_fixup(flash_bank_t *bank, cfi_fixup_t *fixups)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_fixup_t *f;

	for (f = fixups; f->fixup; f++)
	{
		if (((f->mfr == CFI_MFR_ANY) || (f->mfr == cfi_info->manufacturer)) &&
			((f->id  == CFI_ID_ANY)  || (f->id  == cfi_info->device_id)))
		{
			f->fixup(bank, f->param);
		}
	}
}

/* inline u32 flash_address(flash_bank_t *bank, int sector, u32 offset) */
__inline__ u32 flash_address(flash_bank_t *bank, int sector, u32 offset)
{
	/* while the sector list isn't built, only accesses to sector 0 work */
	if (sector == 0)
		return bank->base + offset * bank->bus_width;
	else
	{
		if (!bank->sectors)
		{
			LOG_ERROR("BUG: sector list not yet built");
			exit(-1);
		}
		return bank->base + bank->sectors[sector].offset + offset * bank->bus_width;
	}

}

void cfi_command(flash_bank_t *bank, u8 cmd, u8 *cmd_buf)
{
	int i;

	/* clear whole buffer, to ensure bits that exceed the bus_width
	 * are set to zero
	 */
	for (i = 0; i < CFI_MAX_BUS_WIDTH; i++)
		cmd_buf[i] = 0;

	if (bank->target->endianness == TARGET_LITTLE_ENDIAN)
	{
		for (i = bank->bus_width; i > 0; i--)
		{
			*cmd_buf++ = (i & (bank->chip_width - 1)) ? 0x0 : cmd;
		}
	}
	else
	{
		for (i = 1; i <= bank->bus_width; i++)
		{
			*cmd_buf++ = (i & (bank->chip_width - 1)) ? 0x0 : cmd;
		}
	}
}

/* read unsigned 8-bit value from the bank
 * flash banks are expected to be made of similar chips
 * the query result should be the same for all
 */
u8 cfi_query_u8(flash_bank_t *bank, int sector, u32 offset)
{
	target_t *target = bank->target;
	u8 data[CFI_MAX_BUS_WIDTH];

	target->type->read_memory(target, flash_address(bank, sector, offset), bank->bus_width, 1, data);

	if (bank->target->endianness == TARGET_LITTLE_ENDIAN)
		return data[0];
	else
		return data[bank->bus_width - 1];
}

/* read unsigned 8-bit value from the bank
 * in case of a bank made of multiple chips,
 * the individual values are ORed
 */
u8 cfi_get_u8(flash_bank_t *bank, int sector, u32 offset)
{
	target_t *target = bank->target;
	u8 data[CFI_MAX_BUS_WIDTH];
	int i;

	target->type->read_memory(target, flash_address(bank, sector, offset), bank->bus_width, 1, data);

	if (bank->target->endianness == TARGET_LITTLE_ENDIAN)
	{
		for (i = 0; i < bank->bus_width / bank->chip_width; i++)
			data[0] |= data[i];

		return data[0];
	}
	else
	{
		u8 value = 0;
		for (i = 0; i < bank->bus_width / bank->chip_width; i++)
			value |= data[bank->bus_width - 1 - i];

		return value;
	}
}

u16 cfi_query_u16(flash_bank_t *bank, int sector, u32 offset)
{
	target_t *target = bank->target;
	u8 data[CFI_MAX_BUS_WIDTH * 2];

	target->type->read_memory(target, flash_address(bank, sector, offset), bank->bus_width, 2, data);

	if (bank->target->endianness == TARGET_LITTLE_ENDIAN)
		return data[0] | data[bank->bus_width] << 8;
	else
		return data[bank->bus_width - 1] | data[(2 * bank->bus_width) - 1] << 8;
}

u32 cfi_query_u32(flash_bank_t *bank, int sector, u32 offset)
{
	target_t *target = bank->target;
	u8 data[CFI_MAX_BUS_WIDTH * 4];

	target->type->read_memory(target, flash_address(bank, sector, offset), bank->bus_width, 4, data);

	if (bank->target->endianness == TARGET_LITTLE_ENDIAN)
		return data[0] | data[bank->bus_width] << 8 | data[bank->bus_width * 2] << 16 | data[bank->bus_width * 3] << 24;
	else
		return data[bank->bus_width - 1] | data[(2* bank->bus_width) - 1] << 8 |
				data[(3 * bank->bus_width) - 1] << 16 | data[(4 * bank->bus_width) - 1] << 24;
}

void cfi_intel_clear_status_register(flash_bank_t *bank)
{
	target_t *target = bank->target;
	u8 command[8];

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("BUG: attempted to clear status register while target wasn't halted");
		exit(-1);
	}

	cfi_command(bank, 0x50, command);
	target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command);
}

u8 cfi_intel_wait_status_busy(flash_bank_t *bank, int timeout)
{
	u8 status;

	while ((!((status = cfi_get_u8(bank, 0, 0x0)) & 0x80)) && (timeout-- > 0))
	{
		LOG_DEBUG("status: 0x%x", status);
		alive_sleep(1);
	}

	/* mask out bit 0 (reserved) */
	status = status & 0xfe;

	LOG_DEBUG("status: 0x%x", status);

	if ((status & 0x80) != 0x80)
	{
		LOG_ERROR("timeout while waiting for WSM to become ready");
	}
	else if (status != 0x80)
	{
		LOG_ERROR("status register: 0x%x", status);
		if (status & 0x2)
			LOG_ERROR("Block Lock-Bit Detected, Operation Abort");
		if (status & 0x4)
			LOG_ERROR("Program suspended");
		if (status & 0x8)
			LOG_ERROR("Low Programming Voltage Detected, Operation Aborted");
		if (status & 0x10)
			LOG_ERROR("Program Error / Error in Setting Lock-Bit");
		if (status & 0x20)
			LOG_ERROR("Error in Block Erasure or Clear Lock-Bits");
		if (status & 0x40)
			LOG_ERROR("Block Erase Suspended");

		cfi_intel_clear_status_register(bank);
	}

	return status;
}

int cfi_spansion_wait_status_busy(flash_bank_t *bank, int timeout)
{
	u8 status, oldstatus;

	oldstatus = cfi_get_u8(bank, 0, 0x0);

	do {
		status = cfi_get_u8(bank, 0, 0x0);
		if ((status ^ oldstatus) & 0x40) {
			if (status & 0x20) {
				oldstatus = cfi_get_u8(bank, 0, 0x0);
				status = cfi_get_u8(bank, 0, 0x0);
				if ((status ^ oldstatus) & 0x40) {
					LOG_ERROR("dq5 timeout, status: 0x%x", status);
					return(ERROR_FLASH_OPERATION_FAILED);
				} else {
					LOG_DEBUG("status: 0x%x", status);
					return(ERROR_OK);
				}
			}
		} else {
			LOG_DEBUG("status: 0x%x", status);
			return(ERROR_OK);
		}

		oldstatus = status;
		alive_sleep(1);
	} while (timeout-- > 0);

	LOG_ERROR("timeout, status: 0x%x", status);

	return(ERROR_FLASH_BUSY);
}

int cfi_read_intel_pri_ext(flash_bank_t *bank)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_intel_pri_ext_t *pri_ext = malloc(sizeof(cfi_intel_pri_ext_t));
	target_t *target = bank->target;
	u8 command[8];

	cfi_info->pri_ext = pri_ext;

	pri_ext->pri[0] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0);
	pri_ext->pri[1] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 1);
	pri_ext->pri[2] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 2);

	if ((pri_ext->pri[0] != 'P') || (pri_ext->pri[1] != 'R') || (pri_ext->pri[2] != 'I'))
	{
		cfi_command(bank, 0xf0, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}
		cfi_command(bank, 0xff, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}
		LOG_ERROR("Could not read bank flash bank information");
		return ERROR_FLASH_BANK_INVALID;
	}

	pri_ext->major_version = cfi_query_u8(bank, 0, cfi_info->pri_addr + 3);
	pri_ext->minor_version = cfi_query_u8(bank, 0, cfi_info->pri_addr + 4);

	LOG_DEBUG("pri: '%c%c%c', version: %c.%c", pri_ext->pri[0], pri_ext->pri[1], pri_ext->pri[2], pri_ext->major_version, pri_ext->minor_version);

	pri_ext->feature_support = cfi_query_u32(bank, 0, cfi_info->pri_addr + 5);
	pri_ext->suspend_cmd_support = cfi_query_u8(bank, 0, cfi_info->pri_addr + 9);
	pri_ext->blk_status_reg_mask = cfi_query_u16(bank, 0, cfi_info->pri_addr + 0xa);

	LOG_DEBUG("feature_support: 0x%x, suspend_cmd_support: 0x%x, blk_status_reg_mask: 0x%x", pri_ext->feature_support, pri_ext->suspend_cmd_support, pri_ext->blk_status_reg_mask);

	pri_ext->vcc_optimal = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0xc);
	pri_ext->vpp_optimal = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0xd);

	LOG_DEBUG("Vcc opt: %1.1x.%1.1x, Vpp opt: %1.1x.%1.1x",
		  (pri_ext->vcc_optimal & 0xf0) >> 4, pri_ext->vcc_optimal & 0x0f,
		  (pri_ext->vpp_optimal & 0xf0) >> 4, pri_ext->vpp_optimal & 0x0f);

	pri_ext->num_protection_fields = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0xe);
	if (pri_ext->num_protection_fields != 1)
	{
		LOG_WARNING("expected one protection register field, but found %i", pri_ext->num_protection_fields);
	}

	pri_ext->prot_reg_addr = cfi_query_u16(bank, 0, cfi_info->pri_addr + 0xf);
	pri_ext->fact_prot_reg_size = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0x11);
	pri_ext->user_prot_reg_size = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0x12);

	LOG_DEBUG("protection_fields: %i, prot_reg_addr: 0x%x, factory pre-programmed: %i, user programmable: %i", pri_ext->num_protection_fields, pri_ext->prot_reg_addr, 1 << pri_ext->fact_prot_reg_size, 1 << pri_ext->user_prot_reg_size);

	return ERROR_OK;
}

int cfi_read_spansion_pri_ext(flash_bank_t *bank)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = malloc(sizeof(cfi_spansion_pri_ext_t));
	target_t *target = bank->target;
	u8 command[8];

	cfi_info->pri_ext = pri_ext;

	pri_ext->pri[0] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0);
	pri_ext->pri[1] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 1);
	pri_ext->pri[2] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 2);

	if ((pri_ext->pri[0] != 'P') || (pri_ext->pri[1] != 'R') || (pri_ext->pri[2] != 'I'))
	{
		cfi_command(bank, 0xf0, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}
		LOG_ERROR("Could not read spansion bank information");
		return ERROR_FLASH_BANK_INVALID;
	}

	pri_ext->major_version = cfi_query_u8(bank, 0, cfi_info->pri_addr + 3);
	pri_ext->minor_version = cfi_query_u8(bank, 0, cfi_info->pri_addr + 4);

	LOG_DEBUG("pri: '%c%c%c', version: %c.%c", pri_ext->pri[0], pri_ext->pri[1], pri_ext->pri[2], pri_ext->major_version, pri_ext->minor_version);

	pri_ext->SiliconRevision = cfi_query_u8(bank, 0, cfi_info->pri_addr + 5);
	pri_ext->EraseSuspend    = cfi_query_u8(bank, 0, cfi_info->pri_addr + 6);
	pri_ext->BlkProt         = cfi_query_u8(bank, 0, cfi_info->pri_addr + 7);
	pri_ext->TmpBlkUnprotect = cfi_query_u8(bank, 0, cfi_info->pri_addr + 8);
	pri_ext->BlkProtUnprot   = cfi_query_u8(bank, 0, cfi_info->pri_addr + 9);
	pri_ext->SimultaneousOps = cfi_query_u8(bank, 0, cfi_info->pri_addr + 10);
	pri_ext->BurstMode       = cfi_query_u8(bank, 0, cfi_info->pri_addr + 11);
	pri_ext->PageMode        = cfi_query_u8(bank, 0, cfi_info->pri_addr + 12);
	pri_ext->VppMin          = cfi_query_u8(bank, 0, cfi_info->pri_addr + 13);
	pri_ext->VppMax          = cfi_query_u8(bank, 0, cfi_info->pri_addr + 14);
	pri_ext->TopBottom       = cfi_query_u8(bank, 0, cfi_info->pri_addr + 15);

	LOG_DEBUG("Silicon Revision: 0x%x, Erase Suspend: 0x%x, Block protect: 0x%x", pri_ext->SiliconRevision,
	      pri_ext->EraseSuspend, pri_ext->BlkProt);

	LOG_DEBUG("Temporary Unprotect: 0x%x, Block Protect Scheme: 0x%x, Simultaneous Ops: 0x%x", pri_ext->TmpBlkUnprotect,
	      pri_ext->BlkProtUnprot, pri_ext->SimultaneousOps);

	LOG_DEBUG("Burst Mode: 0x%x, Page Mode: 0x%x, ", pri_ext->BurstMode, pri_ext->PageMode);


	LOG_DEBUG("Vpp min: %2.2d.%1.1d, Vpp max: %2.2d.%1.1x",
		  (pri_ext->VppMin & 0xf0) >> 4, pri_ext->VppMin & 0x0f,
		  (pri_ext->VppMax & 0xf0) >> 4, pri_ext->VppMax & 0x0f);

	LOG_DEBUG("WP# protection 0x%x", pri_ext->TopBottom);

	/* default values for implementation specific workarounds */
	pri_ext->_unlock1 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock1;
	pri_ext->_unlock2 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock2;
	pri_ext->_reversed_geometry = 0;

	return ERROR_OK;
}

int cfi_read_atmel_pri_ext(flash_bank_t *bank)
{
	int retval;
	cfi_atmel_pri_ext_t atmel_pri_ext;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = malloc(sizeof(cfi_spansion_pri_ext_t));
	target_t *target = bank->target;
	u8 command[8];

	/* ATMEL devices use the same CFI primary command set (0x2) as AMD/Spansion,
	 * but a different primary extended query table.
	 * We read the atmel table, and prepare a valid AMD/Spansion query table.
	 */

	memset(pri_ext, 0, sizeof(cfi_spansion_pri_ext_t));

	cfi_info->pri_ext = pri_ext;

	atmel_pri_ext.pri[0] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0);
	atmel_pri_ext.pri[1] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 1);
	atmel_pri_ext.pri[2] = cfi_query_u8(bank, 0, cfi_info->pri_addr + 2);

	if ((atmel_pri_ext.pri[0] != 'P') || (atmel_pri_ext.pri[1] != 'R') || (atmel_pri_ext.pri[2] != 'I'))
	{
		cfi_command(bank, 0xf0, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}
		LOG_ERROR("Could not read atmel bank information");
		return ERROR_FLASH_BANK_INVALID;
	}

	pri_ext->pri[0] = atmel_pri_ext.pri[0];
	pri_ext->pri[1] = atmel_pri_ext.pri[1];
	pri_ext->pri[2] = atmel_pri_ext.pri[2];

	atmel_pri_ext.major_version = cfi_query_u8(bank, 0, cfi_info->pri_addr + 3);
	atmel_pri_ext.minor_version = cfi_query_u8(bank, 0, cfi_info->pri_addr + 4);

	LOG_DEBUG("pri: '%c%c%c', version: %c.%c", atmel_pri_ext.pri[0], atmel_pri_ext.pri[1], atmel_pri_ext.pri[2], atmel_pri_ext.major_version, atmel_pri_ext.minor_version);

	pri_ext->major_version = atmel_pri_ext.major_version;
	pri_ext->minor_version = atmel_pri_ext.minor_version;

	atmel_pri_ext.features = cfi_query_u8(bank, 0, cfi_info->pri_addr + 5);
	atmel_pri_ext.bottom_boot = cfi_query_u8(bank, 0, cfi_info->pri_addr + 6);
	atmel_pri_ext.burst_mode = cfi_query_u8(bank, 0, cfi_info->pri_addr + 7);
	atmel_pri_ext.page_mode = cfi_query_u8(bank, 0, cfi_info->pri_addr + 8);

	LOG_DEBUG("features: 0x%2.2x, bottom_boot: 0x%2.2x, burst_mode: 0x%2.2x, page_mode: 0x%2.2x",
		atmel_pri_ext.features, atmel_pri_ext.bottom_boot, atmel_pri_ext.burst_mode, atmel_pri_ext.page_mode);

	if (atmel_pri_ext.features & 0x02)
		pri_ext->EraseSuspend = 2;

	if (atmel_pri_ext.bottom_boot)
		pri_ext->TopBottom = 2;
	else
		pri_ext->TopBottom = 3;

	pri_ext->_unlock1 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock1;
	pri_ext->_unlock2 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock2;

	return ERROR_OK;
}

int cfi_read_0002_pri_ext(flash_bank_t *bank)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;

	if (cfi_info->manufacturer == CFI_MFR_ATMEL)
	{
		return cfi_read_atmel_pri_ext(bank);
	}
	else
	{
		return cfi_read_spansion_pri_ext(bank);
	}
}

int cfi_spansion_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	int printed;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;

	printed = snprintf(buf, buf_size, "\nSpansion primary algorithm extend information:\n");
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "pri: '%c%c%c', version: %c.%c\n", pri_ext->pri[0],
			   pri_ext->pri[1], pri_ext->pri[2],
			   pri_ext->major_version, pri_ext->minor_version);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "Silicon Rev.: 0x%x, Address Sensitive unlock: 0x%x\n",
			   (pri_ext->SiliconRevision) >> 2,
			   (pri_ext->SiliconRevision) & 0x03);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "Erase Suspend: 0x%x, Sector Protect: 0x%x\n",
			   pri_ext->EraseSuspend,
			   pri_ext->BlkProt);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "VppMin: %2.2d.%1.1x, VppMax: %2.2d.%1.1x\n",
		(pri_ext->VppMin & 0xf0) >> 4, pri_ext->VppMin & 0x0f,
		(pri_ext->VppMax & 0xf0) >> 4, pri_ext->VppMax & 0x0f);

	return ERROR_OK;
}

int cfi_intel_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	int printed;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_intel_pri_ext_t *pri_ext = cfi_info->pri_ext;

	printed = snprintf(buf, buf_size, "\nintel primary algorithm extend information:\n");
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "pri: '%c%c%c', version: %c.%c\n", pri_ext->pri[0], pri_ext->pri[1], pri_ext->pri[2], pri_ext->major_version, pri_ext->minor_version);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "feature_support: 0x%x, suspend_cmd_support: 0x%x, blk_status_reg_mask: 0x%x\n", pri_ext->feature_support, pri_ext->suspend_cmd_support, pri_ext->blk_status_reg_mask);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "Vcc opt: %1.1x.%1.1x, Vpp opt: %1.1x.%1.1x\n",
		(pri_ext->vcc_optimal & 0xf0) >> 4, pri_ext->vcc_optimal & 0x0f,
		(pri_ext->vpp_optimal & 0xf0) >> 4, pri_ext->vpp_optimal & 0x0f);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "protection_fields: %i, prot_reg_addr: 0x%x, factory pre-programmed: %i, user programmable: %i\n", pri_ext->num_protection_fields, pri_ext->prot_reg_addr, 1 << pri_ext->fact_prot_reg_size, 1 << pri_ext->user_prot_reg_size);

	return ERROR_OK;
}

int cfi_register_commands(struct command_context_s *cmd_ctx)
{
	/*command_t *cfi_cmd = */
	register_command(cmd_ctx, NULL, "cfi", NULL, COMMAND_ANY, "flash bank cfi <base> <size> <chip_width> <bus_width> <targetNum> [jedec_probe/x16_as_x8]");
	/*
	register_command(cmd_ctx, cfi_cmd, "part_id", cfi_handle_part_id_command, COMMAND_EXEC,
					 "print part id of cfi flash bank <num>");
	*/
	return ERROR_OK;
}

/* flash_bank cfi <base> <size> <chip_width> <bus_width> <target#> [options]
 */
int cfi_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	cfi_flash_bank_t *cfi_info;
	int i;

	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank cfi configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	if ((strtoul(args[4], NULL, 0) > CFI_MAX_CHIP_WIDTH)
		|| (strtoul(args[3], NULL, 0) > CFI_MAX_BUS_WIDTH))
	{
		LOG_ERROR("chip and bus width have to specified in bytes");
		return ERROR_FLASH_BANK_INVALID;
	}

	cfi_info = malloc(sizeof(cfi_flash_bank_t));
	cfi_info->probed = 0;
	bank->driver_priv = cfi_info;

	cfi_info->write_algorithm = NULL;

	cfi_info->x16_as_x8 = 0;
	cfi_info->jedec_probe = 0;
	cfi_info->not_cfi = 0;

	for (i = 6; i < argc; i++)
	{
		if (strcmp(args[i], "x16_as_x8") == 0)
		{
			cfi_info->x16_as_x8 = 1;
		}
		else if (strcmp(args[i], "jedec_probe") == 0)
		{
			cfi_info->jedec_probe = 1;
		}
	}

	cfi_info->write_algorithm = NULL;

	/* bank wasn't probed yet */
	cfi_info->qry[0] = -1;

	return ERROR_OK;
}

int cfi_intel_erase(struct flash_bank_s *bank, int first, int last)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	target_t *target = bank->target;
	u8 command[8];
	int i;

	cfi_intel_clear_status_register(bank);

	for (i = first; i <= last; i++)
	{
		cfi_command(bank, 0x20, command);
		if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		cfi_command(bank, 0xd0, command);
		if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		if (cfi_intel_wait_status_busy(bank, 1000 * (1 << cfi_info->block_erase_timeout_typ)) == 0x80)
			bank->sectors[i].is_erased = 1;
		else
		{
			cfi_command(bank, 0xff, command);
			if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
			{
				return retval;
			}

			LOG_ERROR("couldn't erase block %i of flash bank at base 0x%x", i, bank->base);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	cfi_command(bank, 0xff, command);
	return target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command);

}

int cfi_spansion_erase(struct flash_bank_s *bank, int first, int last)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;
	target_t *target = bank->target;
	u8 command[8];
	int i;

	for (i = first; i <= last; i++)
	{
		cfi_command(bank, 0xaa, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		cfi_command(bank, 0x55, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock2), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		cfi_command(bank, 0x80, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		cfi_command(bank, 0xaa, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		cfi_command(bank, 0x55, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock2), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		cfi_command(bank, 0x30, command);
		if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		if (cfi_spansion_wait_status_busy(bank, 1000 * (1 << cfi_info->block_erase_timeout_typ)) == ERROR_OK)
			bank->sectors[i].is_erased = 1;
		else
		{
			cfi_command(bank, 0xf0, command);
			if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
			{
				return retval;
			}

			LOG_ERROR("couldn't erase block %i of flash bank at base 0x%x", i, bank->base);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	cfi_command(bank, 0xf0, command);
	return target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command);
}

int cfi_erase(struct flash_bank_s *bank, int first, int last)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	switch(cfi_info->pri_id)
	{
		case 1:
		case 3:
			return cfi_intel_erase(bank, first, last);
			break;
		case 2:
			return cfi_spansion_erase(bank, first, last);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_OK;
}

int cfi_intel_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_intel_pri_ext_t *pri_ext = cfi_info->pri_ext;
	target_t *target = bank->target;
	u8 command[8];
	int retry = 0;
	int i;

	/* if the device supports neither legacy lock/unlock (bit 3) nor
	 * instant individual block locking (bit 5).
	 */
	if (!(pri_ext->feature_support & 0x28))
		return ERROR_FLASH_OPERATION_FAILED;

	cfi_intel_clear_status_register(bank);

	for (i = first; i <= last; i++)
	{
		cfi_command(bank, 0x60, command);
		LOG_DEBUG("address: 0x%4.4x, command: 0x%4.4x", flash_address(bank, i, 0x0), target_buffer_get_u32(target, command));
		if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}
		if (set)
		{
			cfi_command(bank, 0x01, command);
			LOG_DEBUG("address: 0x%4.4x, command: 0x%4.4x", flash_address(bank, i, 0x0), target_buffer_get_u32(target, command));
			if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
			{
				return retval;
			}
			bank->sectors[i].is_protected = 1;
		}
		else
		{
			cfi_command(bank, 0xd0, command);
			LOG_DEBUG("address: 0x%4.4x, command: 0x%4.4x", flash_address(bank, i, 0x0), target_buffer_get_u32(target, command));
			if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
			{
				return retval;
			}
			bank->sectors[i].is_protected = 0;
		}

		/* instant individual block locking doesn't require reading of the status register */
		if (!(pri_ext->feature_support & 0x20))
		{
			/* Clear lock bits operation may take up to 1.4s */
			cfi_intel_wait_status_busy(bank, 1400);
		}
		else
		{
			u8 block_status;
			/* read block lock bit, to verify status */
			cfi_command(bank, 0x90, command);
			if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x55), bank->bus_width, 1, command)) != ERROR_OK)
			{
				return retval;
			}
			block_status = cfi_get_u8(bank, i, 0x2);

			if ((block_status & 0x1) != set)
			{
				LOG_ERROR("couldn't change block lock status (set = %i, block_status = 0x%2.2x)", set, block_status);
				cfi_command(bank, 0x70, command);
				if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x55), bank->bus_width, 1, command)) != ERROR_OK)
				{
					return retval;
				}
				cfi_intel_wait_status_busy(bank, 10);

				if (retry > 10)
					return ERROR_FLASH_OPERATION_FAILED;
				else
				{
					i--;
					retry++;
				}
			}
		}
	}

	/* if the device doesn't support individual block lock bits set/clear,
	 * all blocks have been unlocked in parallel, so we set those that should be protected
	 */
	if ((!set) && (!(pri_ext->feature_support & 0x20)))
	{
		for (i = 0; i < bank->num_sectors; i++)
		{
			if (bank->sectors[i].is_protected == 1)
			{
				cfi_intel_clear_status_register(bank);

				cfi_command(bank, 0x60, command);
				if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
				{
					return retval;
				}

				cfi_command(bank, 0x01, command);
				if((retval = target->type->write_memory(target, flash_address(bank, i, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
				{
					return retval;
				}

				cfi_intel_wait_status_busy(bank, 100);
			}
		}
	}

	cfi_command(bank, 0xff, command);
	return target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command);
}

int cfi_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	switch(cfi_info->pri_id)
	{
		case 1:
		case 3:
			cfi_intel_protect(bank, set, first, last);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_OK;
}

/* FIXME Replace this by a simple memcpy() - still unsure about sideeffects */
static void cfi_add_byte(struct flash_bank_s *bank, u8 *word, u8 byte)
{
	/* target_t *target = bank->target; */

	int i;

	/* NOTE:
	 * The data to flash must not be changed in endian! We write a bytestrem in
	 * target byte order already. Only the control and status byte lane of the flash
	 * WSM is interpreted by the CPU in different ways, when read a u16 or u32
	 * word (data seems to be in the upper or lower byte lane for u16 accesses).
	 */

#if 0
	if (target->endianness == TARGET_LITTLE_ENDIAN)
	{
#endif
		/* shift bytes */
		for (i = 0; i < bank->bus_width - 1; i++)
			word[i] = word[i + 1];
		word[bank->bus_width - 1] = byte;
#if 0
	}
	else
	{
		/* shift bytes */
		for (i = bank->bus_width - 1; i > 0; i--)
			word[i] = word[i - 1];
		word[0] = byte;
	}
#endif
}

/* Convert code image to target endian */
/* FIXME create general block conversion fcts in target.c?) */
static void cfi_fix_code_endian(target_t *target, u8 *dest, const u32 *src, u32 count)
{
	u32 i;
	for (i=0; i< count; i++)
	{
		target_buffer_set_u32(target, dest, *src);
		dest+=4;
		src++;
	}
}

u32 cfi_command_val(flash_bank_t *bank, u8 cmd)
{
	target_t *target = bank->target;

	u8 buf[CFI_MAX_BUS_WIDTH];
	cfi_command(bank, cmd, buf);
	switch (bank->bus_width)
	{
	case 1 :
		return buf[0];
		break;
	case 2 :
		return target_buffer_get_u16(target, buf);
		break;
	case 4 :
		return target_buffer_get_u32(target, buf);
		break;
	default :
		LOG_ERROR("Unsupported bank buswidth %d, can't do block memory writes", bank->bus_width);
		return 0;
	}
}

int cfi_intel_write_block(struct flash_bank_s *bank, u8 *buffer, u32 address, u32 count)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	target_t *target = bank->target;
	reg_param_t reg_params[7];
	armv4_5_algorithm_t armv4_5_info;
	working_area_t *source;
	u32 buffer_size = 32768;
	u32 write_command_val, busy_pattern_val, error_pattern_val;

	/* algorithm register usage:
	 * r0: source address (in RAM)
	 * r1: target address (in Flash)
	 * r2: count
	 * r3: flash write command
	 * r4: status byte (returned to host)
	 * r5: busy test pattern
	 * r6: error test pattern
	 */

	static const u32 word_32_code[] = {
		0xe4904004,   /* loop:	ldr r4, [r0], #4 */
		0xe5813000,   /* 		str r3, [r1] */
		0xe5814000,   /* 		str r4, [r1] */
		0xe5914000,   /* busy:  ldr r4, [r1] */
		0xe0047005,   /*		and r7, r4, r5 */
		0xe1570005,   /*		cmp r7, r5 */
		0x1afffffb,   /*		bne busy */
		0xe1140006,   /* 		tst r4, r6 */
		0x1a000003,   /*		bne done */
		0xe2522001,   /*		subs r2, r2, #1 */
		0x0a000001,   /* 		beq done */
		0xe2811004,   /*		add r1, r1 #4 */
		0xeafffff2,   /* 		b loop */
		0xeafffffe    /* done:	b -2 */
	};

	static const u32 word_16_code[] = {
		0xe0d040b2,   /* loop:	ldrh r4, [r0], #2 */
		0xe1c130b0,   /* 		strh r3, [r1] */
		0xe1c140b0,   /* 		strh r4, [r1] */
		0xe1d140b0,   /* busy	ldrh r4, [r1] */
		0xe0047005,   /*		and r7, r4, r5 */
		0xe1570005,   /*		cmp r7, r5 */
		0x1afffffb,   /*		bne busy */
		0xe1140006,   /* 		tst r4, r6 */
		0x1a000003,   /*		bne done */
		0xe2522001,   /*		subs r2, r2, #1 */
		0x0a000001,   /* 		beq done */
		0xe2811002,   /*		add r1, r1 #2 */
		0xeafffff2,   /* 		b loop */
		0xeafffffe    /* done:	b -2 */
	};

	static const u32 word_8_code[] = {
		0xe4d04001,   /* loop:	ldrb r4, [r0], #1 */
		0xe5c13000,   /* 		strb r3, [r1] */
		0xe5c14000,   /* 		strb r4, [r1] */
		0xe5d14000,   /* busy	ldrb r4, [r1] */
		0xe0047005,   /*		and r7, r4, r5 */
		0xe1570005,   /*		cmp r7, r5 */
		0x1afffffb,   /*		bne busy */
		0xe1140006,   /* 		tst r4, r6 */
		0x1a000003,   /*		bne done */
		0xe2522001,   /*		subs r2, r2, #1 */
		0x0a000001,   /* 		beq done */
		0xe2811001,   /*		add r1, r1 #1 */
		0xeafffff2,   /* 		b loop */
		0xeafffffe    /* done:	b -2 */
	};
	u8 target_code[4*CFI_MAX_INTEL_CODESIZE];
	const u32 *target_code_src;
	int target_code_size;
	int retval = ERROR_OK;


	cfi_intel_clear_status_register(bank);

	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;

	/* If we are setting up the write_algorith, we need target_code_src */
	/* if not we only need target_code_size.														*/
	/* 																																	*/
	/* However, we don't want to create multiple code paths, so we			*/
	/* do the unecessary evaluation of target_code_src, which the 			*/
	/* compiler will probably nicely optimize away if not needed				*/

	/* prepare algorithm code for target endian */
	switch (bank->bus_width)
	{
	case 1 :
		target_code_src = word_8_code;
		target_code_size = sizeof(word_8_code);
		break;
	case 2 :
		target_code_src = word_16_code;
		target_code_size = sizeof(word_16_code);
		break;
	case 4 :
		target_code_src = word_32_code;
		target_code_size = sizeof(word_32_code);
		break;
	default:
		LOG_ERROR("Unsupported bank buswidth %d, can't do block memory writes", bank->bus_width);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* flash write code */
	if (!cfi_info->write_algorithm)
	{
		if ( target_code_size > sizeof(target_code) )
		{
			LOG_WARNING("Internal error - target code buffer to small. Increase CFI_MAX_INTEL_CODESIZE and recompile.");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		cfi_fix_code_endian(target, target_code, target_code_src, target_code_size / 4);

		/* Get memory for block write handler */
		retval = target_alloc_working_area(target, target_code_size, &cfi_info->write_algorithm);
		if (retval != ERROR_OK)
		{
			LOG_WARNING("No working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		};

		/* write algorithm code to working area */
		retval = target_write_buffer(target, cfi_info->write_algorithm->address, target_code_size, target_code);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("Unable to write block write code to target");
			goto cleanup;
		}
	}

	/* Get a workspace buffer for the data to flash starting with 32k size.
	   Half size until buffer would be smaller 256 Bytem then fail back */
	/* FIXME Why 256 bytes, why not 32 bytes (smallest flash write page */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto cleanup;
		}
	};

	/* setup algo registers */
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN);
	init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);
	init_reg_param(&reg_params[6], "r6", 32, PARAM_OUT);

	/* prepare command and status register patterns */
	write_command_val = cfi_command_val(bank, 0x40);
	busy_pattern_val  = cfi_command_val(bank, 0x80);
	error_pattern_val = cfi_command_val(bank, 0x7e);

	LOG_INFO("Using target buffer at 0x%08x and of size 0x%04x", source->address, buffer_size );

	/* Programming main loop */
	while (count > 0)
	{
		u32 thisrun_count = (count > buffer_size) ? buffer_size : count;
		u32 wsm_error;

		if((retval = target_write_buffer(target, source->address, thisrun_count, buffer)) != ERROR_OK)
		{
			goto cleanup;
		}

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count / bank->bus_width);

		buf_set_u32(reg_params[3].value, 0, 32, write_command_val);
		buf_set_u32(reg_params[5].value, 0, 32, busy_pattern_val);
		buf_set_u32(reg_params[6].value, 0, 32, error_pattern_val);

		LOG_INFO("Write 0x%04x bytes to flash at 0x%08x", thisrun_count, address );

		/* Execute algorithm, assume breakpoint for last instruction */
		retval = target->type->run_algorithm(target, 0, NULL, 7, reg_params,
			cfi_info->write_algorithm->address,
			cfi_info->write_algorithm->address + target_code_size - sizeof(u32),
			10000, /* 10s should be enough for max. 32k of data */
			&armv4_5_info);

		/* On failure try a fall back to direct word writes */
		if (retval != ERROR_OK)
		{
			cfi_intel_clear_status_register(bank);
			LOG_ERROR("Execution of flash algorythm failed. Can't fall back. Please report.");
			retval = ERROR_FLASH_OPERATION_FAILED;
			/* retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE; */
			/* FIXME To allow fall back or recovery, we must save the actual status
			   somewhere, so that a higher level code can start recovery. */
			goto cleanup;
		}

		/* Check return value from algo code */
		wsm_error = buf_get_u32(reg_params[4].value, 0, 32) & error_pattern_val;
		if (wsm_error)
		{
			/* read status register (outputs debug inforation) */
			cfi_intel_wait_status_busy(bank, 100);
			cfi_intel_clear_status_register(bank);
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto cleanup;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}

	/* free up resources */
cleanup:
	if (source)
		target_free_working_area(target, source);

	if (cfi_info->write_algorithm)
	{
		target_free_working_area(target, cfi_info->write_algorithm);
		cfi_info->write_algorithm = NULL;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);

	return retval;
}

int cfi_spansion_write_block(struct flash_bank_s *bank, u8 *buffer, u32 address, u32 count)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;
	target_t *target = bank->target;
	reg_param_t reg_params[10];
	armv4_5_algorithm_t armv4_5_info;
	working_area_t *source;
	u32 buffer_size = 32768;
	u32 status;
	int retval, retvaltemp;
	int exit_code = ERROR_OK;

	/* input parameters - */
	/*	R0 = source address */
	/*	R1 = destination address */
	/*	R2 = number of writes */
	/*	R3 = flash write command */
	/*	R4 = constant to mask DQ7 bits (also used for Dq5 with shift) */
	/* output parameters - */
	/*	R5 = 0x80 ok 0x00 bad */
	/* temp registers - */
	/*	R6 = value read from flash to test status */
	/*	R7 = holding register */
	/* unlock registers - */
	/*  R8 = unlock1_addr */
	/*  R9 = unlock1_cmd */
	/*  R10 = unlock2_addr */
	/*  R11 = unlock2_cmd */

	static const u32 word_32_code[] = {
						/* 00008100 <sp_32_code>:		*/
		0xe4905004,		/* ldr	r5, [r0], #4			*/
		0xe5889000, 	/* str	r9, [r8]				*/
		0xe58ab000, 	/* str	r11, [r10]				*/
		0xe5883000, 	/* str	r3, [r8]				*/
		0xe5815000, 	/* str	r5, [r1]				*/
		0xe1a00000, 	/* nop							*/
						/*								*/
						/* 00008110 <sp_32_busy>:		*/
		0xe5916000, 	/* ldr	r6, [r1]				*/
		0xe0257006, 	/* eor	r7, r5, r6				*/
		0xe0147007, 	/* ands	r7, r4, r7				*/
		0x0a000007, 	/* beq	8140 <sp_32_cont> ; b if DQ7 == Data7 */
		0xe0166124, 	/* ands	r6, r6, r4, lsr #2		*/
		0x0afffff9, 	/* beq	8110 <sp_32_busy> ;	b if DQ5 low */
		0xe5916000, 	/* ldr	r6, [r1]				*/
		0xe0257006, 	/* eor	r7, r5, r6				*/
		0xe0147007, 	/* ands	r7, r4, r7				*/
		0x0a000001, 	/* beq	8140 <sp_32_cont> ; b if DQ7 == Data7 */
		0xe3a05000, 	/* mov	r5, #0	; 0x0 - return 0x00, error */
		0x1a000004, 	/* bne	8154 <sp_32_done>		*/
						/*								*/
				/* 00008140 <sp_32_cont>:				*/
		0xe2522001, 	/* subs	r2, r2, #1	; 0x1		*/
		0x03a05080, 	/* moveq	r5, #128	; 0x80	*/
		0x0a000001, 	/* beq	8154 <sp_32_done>		*/
		0xe2811004, 	/* add	r1, r1, #4	; 0x4		*/
		0xeaffffe8, 	/* b	8100 <sp_32_code>		*/
						/*								*/
						/* 00008154 <sp_32_done>:		*/
		0xeafffffe		/* b	8154 <sp_32_done>		*/
		};

		static const u32 word_16_code[] = {
				/* 00008158 <sp_16_code>:              */
		0xe0d050b2, 	/* ldrh	r5, [r0], #2		   */
		0xe1c890b0, 	/* strh	r9, [r8]				*/
		0xe1cab0b0, 	/* strh	r11, [r10]				*/
		0xe1c830b0, 	/* strh	r3, [r8]				*/
		0xe1c150b0, 	/* strh	r5, [r1]		       */
		0xe1a00000, 	/* nop			(mov r0,r0)    */
				/* 				       */
				/* 00008168 <sp_16_busy>:	       */
		0xe1d160b0, 	/* ldrh	r6, [r1]		       */
		0xe0257006, 	/* eor	r7, r5, r6		       */
		0xe0147007, 	/* ands	r7, r4, r7		       */
		0x0a000007, 	/* beq	8198 <sp_16_cont>	       */
		0xe0166124, 	/* ands	r6, r6, r4, lsr #2	       */
		0x0afffff9, 	/* beq	8168 <sp_16_busy>	       */
		0xe1d160b0, 	/* ldrh	r6, [r1]		       */
		0xe0257006, 	/* eor	r7, r5, r6		       */
		0xe0147007, 	/* ands	r7, r4, r7		       */
		0x0a000001, 	/* beq	8198 <sp_16_cont>	       */
		0xe3a05000, 	/* mov	r5, #0	; 0x0		       */
		0x1a000004, 	/* bne	81ac <sp_16_done>	       */
				/* 				       */
				/* 00008198 <sp_16_cont>:	       */
		0xe2522001, 	/* subs	r2, r2, #1	; 0x1	       */
		0x03a05080, 	/* moveq	r5, #128	; 0x80 */
		0x0a000001, 	/* beq	81ac <sp_16_done>	       */
		0xe2811002, 	/* add	r1, r1, #2	; 0x2	       */
		0xeaffffe8, 	/* b	8158 <sp_16_code>	       */
				/* 				       */
				/* 000081ac <sp_16_done>:	       */
		0xeafffffe 	/* b	81ac <sp_16_done>              */
		};

		static const u32 word_8_code[] = {
				/* 000081b0 <sp_16_code_end>:          */
		0xe4d05001, 	/* ldrb	r5, [r0], #1		       */
		0xe5c89000, 	/* strb	r9, [r8]				*/
		0xe5cab000, 	/* strb	r11, [r10]				*/
		0xe5c83000, 	/* strb	r3, [r8]				*/
		0xe5c15000, 	/* strb	r5, [r1]		       */
		0xe1a00000, 	/* nop			(mov r0,r0)    */
				/* 				       */
				/* 000081c0 <sp_8_busy>:	       */
		0xe5d16000, 	/* ldrb	r6, [r1]		       */
		0xe0257006, 	/* eor	r7, r5, r6		       */
		0xe0147007, 	/* ands	r7, r4, r7		       */
		0x0a000007, 	/* beq	81f0 <sp_8_cont>	       */
		0xe0166124, 	/* ands	r6, r6, r4, lsr #2	       */
		0x0afffff9, 	/* beq	81c0 <sp_8_busy>	       */
		0xe5d16000, 	/* ldrb	r6, [r1]		       */
		0xe0257006, 	/* eor	r7, r5, r6		       */
		0xe0147007, 	/* ands	r7, r4, r7		       */
		0x0a000001, 	/* beq	81f0 <sp_8_cont>	       */
		0xe3a05000, 	/* mov	r5, #0	; 0x0		       */
		0x1a000004, 	/* bne	8204 <sp_8_done>	       */
				/* 				       */
				/* 000081f0 <sp_8_cont>:	       */
		0xe2522001, 	/* subs	r2, r2, #1	; 0x1	       */
		0x03a05080, 	/* moveq	r5, #128	; 0x80 */
		0x0a000001, 	/* beq	8204 <sp_8_done>	       */
		0xe2811001, 	/* add	r1, r1, #1	; 0x1	       */
		0xeaffffe8, 	/* b	81b0 <sp_16_code_end>	       */
				/* 				       */
				/* 00008204 <sp_8_done>:	       */
		0xeafffffe 	/* b	8204 <sp_8_done>               */
	};

	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;

	/* flash write code */
	if (!cfi_info->write_algorithm)
	{
		u8 *target_code;
		int target_code_size;
		const u32 *src;

		/* convert bus-width dependent algorithm code to correct endiannes */
		switch (bank->bus_width)
		{
		case 1:
			src = word_8_code;
			target_code_size = sizeof(word_8_code);
			break;
		case 2:
			src = word_16_code;
			target_code_size = sizeof(word_16_code);
			break;
		case 4:
			src = word_32_code;
			target_code_size = sizeof(word_32_code);
			break;
		default:
			LOG_ERROR("Unsupported bank buswidth %d, can't do block memory writes", bank->bus_width);
			return ERROR_FLASH_OPERATION_FAILED;
		}
		target_code = malloc(target_code_size);
		cfi_fix_code_endian(target, target_code, src, target_code_size / 4);

		/* allocate working area */
		retval=target_alloc_working_area(target, target_code_size,
				&cfi_info->write_algorithm);
		if (retval != ERROR_OK)
		{
			free(target_code);
			return retval;
		}

		/* write algorithm code to working area */
		if((retval = target_write_buffer(target, cfi_info->write_algorithm->address,
		                    target_code_size, target_code)) != ERROR_OK)
		{
			free(target_code);
			return retval;
		}

		free(target_code);
	}
	/* the following code still assumes target code is fixed 24*4 bytes */

	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
			if (cfi_info->write_algorithm)
				target_free_working_area(target, cfi_info->write_algorithm);

			LOG_WARNING("not enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	};

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "r5", 32, PARAM_IN);
	init_reg_param(&reg_params[6], "r8", 32, PARAM_OUT);
	init_reg_param(&reg_params[7], "r9", 32, PARAM_OUT);
	init_reg_param(&reg_params[8], "r10", 32, PARAM_OUT);
	init_reg_param(&reg_params[9], "r11", 32, PARAM_OUT);

	while (count > 0)
	{
		u32 thisrun_count = (count > buffer_size) ? buffer_size : count;

		retvaltemp = target_write_buffer(target, source->address, thisrun_count, buffer);

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count / bank->bus_width);
		buf_set_u32(reg_params[3].value, 0, 32, cfi_command_val(bank, 0xA0));
		buf_set_u32(reg_params[4].value, 0, 32, cfi_command_val(bank, 0x80));
		buf_set_u32(reg_params[6].value, 0, 32, flash_address(bank, 0, pri_ext->_unlock1));
		buf_set_u32(reg_params[7].value, 0, 32, 0xaaaaaaaa);
		buf_set_u32(reg_params[8].value, 0, 32, flash_address(bank, 0, pri_ext->_unlock2));
		buf_set_u32(reg_params[9].value, 0, 32, 0x55555555);

		retval = target->type->run_algorithm(target, 0, NULL, 10, reg_params,
						     cfi_info->write_algorithm->address,
						     cfi_info->write_algorithm->address + ((24 * 4) - 4),
						     10000, &armv4_5_info);

		status = buf_get_u32(reg_params[5].value, 0, 32);

		if ((retval != ERROR_OK) || (retvaltemp != ERROR_OK) || status != 0x80)
		{
			LOG_DEBUG("status: 0x%x", status);
			exit_code = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}

	target_free_working_area(target, source);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);
	destroy_reg_param(&reg_params[7]);
	destroy_reg_param(&reg_params[8]);
	destroy_reg_param(&reg_params[9]);

	return exit_code;
}

int cfi_intel_write_word(struct flash_bank_s *bank, u8 *word, u32 address)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	target_t *target = bank->target;
	u8 command[8];

	cfi_intel_clear_status_register(bank);
	cfi_command(bank, 0x40, command);
	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, word)) != ERROR_OK)
	{
		return retval;
	}

	if (cfi_intel_wait_status_busy(bank, 1000 * (1 << cfi_info->word_write_timeout_max)) != 0x80)
	{
		cfi_command(bank, 0xff, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		LOG_ERROR("couldn't write word at base 0x%x, address %x", bank->base, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int cfi_intel_write_words(struct flash_bank_s *bank, u8 *word, u32 wordcount, u32 address)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	target_t *target = bank->target;
	u8 command[8];

	/* Calculate buffer size and boundary mask */
	u32 buffersize = 1UL << cfi_info->max_buf_write_size;
	u32 buffermask = buffersize-1;
	u32 bufferwsize;

	/* Check for valid range */
	if (address & buffermask)
	{
		LOG_ERROR("Write address at base 0x%x, address %x not aligned to 2^%d boundary", bank->base, address, cfi_info->max_buf_write_size);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	switch(bank->chip_width)
	{
	case 4 : bufferwsize = buffersize / 4; break;
	case 2 : bufferwsize = buffersize / 2; break;
	case 1 : bufferwsize = buffersize; break;
	default:
		LOG_ERROR("Unsupported chip width %d", bank->chip_width);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check for valid size */
	if (wordcount > bufferwsize)
	{
		LOG_ERROR("Number of data words %d exceeds available buffersize %d", wordcount, buffersize);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Write to flash buffer */
	cfi_intel_clear_status_register(bank);

	/* Initiate buffer operation _*/
	cfi_command(bank, 0xE8, command);
	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}
	if (cfi_intel_wait_status_busy(bank, 1000 * (1 << cfi_info->buf_write_timeout_max)) != 0x80)
	{
		cfi_command(bank, 0xff, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		LOG_ERROR("couldn't start buffer write operation at base 0x%x, address %x", bank->base, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Write buffer wordcount-1 and data words */
	cfi_command(bank, bufferwsize-1, command);
	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	if((retval = target->type->write_memory(target, address, bank->bus_width, bufferwsize, word)) != ERROR_OK)
	{
		return retval;
	}

	/* Commit write operation */
	cfi_command(bank, 0xd0, command);
	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}
	if (cfi_intel_wait_status_busy(bank, 1000 * (1 << cfi_info->buf_write_timeout_max)) != 0x80)
	{
		cfi_command(bank, 0xff, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		LOG_ERROR("Buffer write at base 0x%x, address %x failed.", bank->base, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int cfi_spansion_write_word(struct flash_bank_s *bank, u8 *word, u32 address)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;
	target_t *target = bank->target;
	u8 command[8];

	cfi_command(bank, 0xaa, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	cfi_command(bank, 0x55, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock2), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	cfi_command(bank, 0xa0, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, word)) != ERROR_OK)
	{
		return retval;
	}

	if (cfi_spansion_wait_status_busy(bank, 1000 * (1 << cfi_info->word_write_timeout_max)) != ERROR_OK)
	{
		cfi_command(bank, 0xf0, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		LOG_ERROR("couldn't write word at base 0x%x, address %x", bank->base, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int cfi_spansion_write_words(struct flash_bank_s *bank, u8 *word, u32 wordcount, u32 address)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	target_t *target = bank->target;
	u8 command[8];
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;

	/* Calculate buffer size and boundary mask */
	u32 buffersize = 1UL << cfi_info->max_buf_write_size;
	u32 buffermask = buffersize-1;
	u32 bufferwsize;

	/* Check for valid range */
	if (address & buffermask)
	{
		LOG_ERROR("Write address at base 0x%x, address %x not aligned to 2^%d boundary", bank->base, address, cfi_info->max_buf_write_size);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	switch(bank->chip_width)
	{
	case 4 : bufferwsize = buffersize / 4; break;
	case 2 : bufferwsize = buffersize / 2; break;
	case 1 : bufferwsize = buffersize; break;
	default:
		LOG_ERROR("Unsupported chip width %d", bank->chip_width);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check for valid size */
	if (wordcount > bufferwsize)
	{
		LOG_ERROR("Number of data words %d exceeds available buffersize %d", wordcount, buffersize);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	// Unlock
	cfi_command(bank, 0xaa, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	cfi_command(bank, 0x55, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock2), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	// Buffer load command
	cfi_command(bank, 0x25, command);
	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	/* Write buffer wordcount-1 and data words */
	cfi_command(bank, bufferwsize-1, command);
	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	if((retval = target->type->write_memory(target, address, bank->bus_width, bufferwsize, word)) != ERROR_OK)
	{
		return retval;
	}

	/* Commit write operation */
	cfi_command(bank, 0x29, command);
	if((retval = target->type->write_memory(target, address, bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	if (cfi_spansion_wait_status_busy(bank, 1000 * (1 << cfi_info->word_write_timeout_max)) != ERROR_OK)
	{
		cfi_command(bank, 0xf0, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		LOG_ERROR("couldn't write block at base 0x%x, address %x, size %x", bank->base, address, bufferwsize);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int cfi_write_word(struct flash_bank_s *bank, u8 *word, u32 address)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;

	switch(cfi_info->pri_id)
	{
		case 1:
		case 3:
			return cfi_intel_write_word(bank, word, address);
			break;
		case 2:
			return cfi_spansion_write_word(bank, word, address);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_FLASH_OPERATION_FAILED;
}

int cfi_write_words(struct flash_bank_s *bank, u8 *word, u32 wordcount, u32 address)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;

	switch(cfi_info->pri_id)
	{
		case 1:
		case 3:
			return cfi_intel_write_words(bank, word, wordcount, address);
			break;
		case 2:
			return cfi_spansion_write_words(bank, word, wordcount, address); 
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_FLASH_OPERATION_FAILED;
}

int cfi_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 address = bank->base + offset;	/* address of first byte to be programmed */
	u32 write_p, copy_p;
	int align;	/* number of unaligned bytes */
	int blk_count; /* number of bus_width bytes for block copy */
	u8 current_word[CFI_MAX_BUS_WIDTH * 4];	/* word (bus_width size) currently being programmed */
	int i;
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* start at the first byte of the first word (bus_width size) */
	write_p = address & ~(bank->bus_width - 1);
	if ((align = address - write_p) != 0)
	{
		LOG_INFO("Fixup %d unaligned head bytes", align );

		for (i = 0; i < bank->bus_width; i++)
			current_word[i] = 0;
		copy_p = write_p;

		/* copy bytes before the first write address */
		for (i = 0; i < align; ++i, ++copy_p)
		{
			u8 byte;
			if((retval = target->type->read_memory(target, copy_p, 1, 1, &byte)) != ERROR_OK)
			{
				return retval;
			}
			cfi_add_byte(bank, current_word, byte);
		}

		/* add bytes from the buffer */
		for (; (i < bank->bus_width) && (count > 0); i++)
		{
			cfi_add_byte(bank, current_word, *buffer++);
			count--;
			copy_p++;
		}

		/* if the buffer is already finished, copy bytes after the last write address */
		for (; (count == 0) && (i < bank->bus_width); ++i, ++copy_p)
		{
			u8 byte;
			if((retval = target->type->read_memory(target, copy_p, 1, 1, &byte)) != ERROR_OK)
			{
				return retval;
			}
			cfi_add_byte(bank, current_word, byte);
		}

		retval = cfi_write_word(bank, current_word, write_p);
		if (retval != ERROR_OK)
			return retval;
		write_p = copy_p;
	}

	/* handle blocks of bus_size aligned bytes */
	blk_count = count & ~(bank->bus_width - 1); /* round down, leave tail bytes */
	switch(cfi_info->pri_id)
	{
		/* try block writes (fails without working area) */
		case 1:
		case 3:
			retval = cfi_intel_write_block(bank, buffer, write_p, blk_count);
			break;
		case 2:
			retval = cfi_spansion_write_block(bank, buffer, write_p, blk_count);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
	}
	if (retval == ERROR_OK)
	{
		/* Increment pointers and decrease count on succesful block write */
		buffer += blk_count;
		write_p += blk_count;
		count -= blk_count;
	}
	else
	{
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		{
			u32 buffersize = 1UL << cfi_info->max_buf_write_size;
			u32 buffermask = buffersize-1;
			u32 bufferwsize;

			switch(bank->chip_width)
			{
			case 4 : bufferwsize = buffersize / 4; break;
			case 2 : bufferwsize = buffersize / 2; break;
			case 1 : bufferwsize = buffersize; break;
			default:
				LOG_ERROR("Unsupported chip width %d", bank->chip_width);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			/* fall back to memory writes */
			while (count >= bank->bus_width)
			{
				int fallback;
				if ((write_p & 0xff) == 0)
				{
					LOG_INFO("Programming at %08x, count %08x bytes remaining", write_p, count);
				}
				fallback = 1;
				if ((bufferwsize > 0) && (count >= buffersize) && !(write_p & buffermask))
				{
					retval = cfi_write_words(bank, buffer, bufferwsize, write_p);
					if (retval == ERROR_OK)
					{
						buffer += buffersize;
						write_p += buffersize;
						count -= buffersize;
						fallback=0;
					}
				}
				/* try the slow way? */
				if (fallback)
				{
					for (i = 0; i < bank->bus_width; i++)
						current_word[i] = 0;

					for (i = 0; i < bank->bus_width; i++)
					{
						cfi_add_byte(bank, current_word, *buffer++);
					}

					retval = cfi_write_word(bank, current_word, write_p);
					if (retval != ERROR_OK)
						return retval;

					write_p += bank->bus_width;
					count -= bank->bus_width;
				}
			}
		}
		else
			return retval;
	}

	/* return to read array mode, so we can read from flash again for padding */
	cfi_command(bank, 0xf0, current_word);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, current_word)) != ERROR_OK)
	{
		return retval;
	}
	cfi_command(bank, 0xff, current_word);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, current_word)) != ERROR_OK)
	{
		return retval;
	}

	/* handle unaligned tail bytes */
	if (count > 0)
	{
		LOG_INFO("Fixup %d unaligned tail bytes", count );

		copy_p = write_p;
		for (i = 0; i < bank->bus_width; i++)
			current_word[i] = 0;

		for (i = 0; (i < bank->bus_width) && (count > 0); ++i, ++copy_p)
		{
			cfi_add_byte(bank, current_word, *buffer++);
			count--;
		}
		for (; i < bank->bus_width; ++i, ++copy_p)
		{
			u8 byte;
			if((retval = target->type->read_memory(target, copy_p, 1, 1, &byte)) != ERROR_OK)
			{
				return retval;
			}
			cfi_add_byte(bank, current_word, byte);
		}
		retval = cfi_write_word(bank, current_word, write_p);
		if (retval != ERROR_OK)
			return retval;
	}

	/* return to read array mode */
	cfi_command(bank, 0xf0, current_word);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, current_word)) != ERROR_OK)
	{
		return retval;
	}
	cfi_command(bank, 0xff, current_word);
	return target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, current_word);
}

void cfi_fixup_atmel_reversed_erase_regions(flash_bank_t *bank, void *param)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;

	pri_ext->_reversed_geometry = 1;
}

void cfi_fixup_0002_erase_regions(flash_bank_t *bank, void *param)
{
	int i;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;

	if ((pri_ext->_reversed_geometry) || (pri_ext->TopBottom == 3))
	{
		LOG_DEBUG("swapping reversed erase region information on cmdset 0002 device");

		for (i = 0; i < cfi_info->num_erase_regions / 2; i++)
		{
			int j = (cfi_info->num_erase_regions - 1) - i;
			u32 swap;

			swap = cfi_info->erase_region_info[i];
			cfi_info->erase_region_info[i] = cfi_info->erase_region_info[j];
			cfi_info->erase_region_info[j] = swap;
		}
	}
}

void cfi_fixup_0002_unlock_addresses(flash_bank_t *bank, void *param)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;
	cfi_unlock_addresses_t *unlock_addresses = param;

	pri_ext->_unlock1 = unlock_addresses->unlock1;
	pri_ext->_unlock2 = unlock_addresses->unlock2;
}

int cfi_probe(struct flash_bank_s *bank)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	target_t *target = bank->target;
	u8 command[8];
	int num_sectors = 0;
	int i;
	int sector = 0;
	u32 offset = 0;
	u32 unlock1 = 0x555;
	u32 unlock2 = 0x2aa;
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	cfi_info->probed = 0;

	/* JEDEC standard JESD21C uses 0x5555 and 0x2aaa as unlock addresses,
	 * while CFI compatible AMD/Spansion flashes use 0x555 and 0x2aa
	 */
	if (cfi_info->jedec_probe)
	{
		unlock1 = 0x5555;
		unlock2 = 0x2aaa;
	}

	/* switch to read identifier codes mode ("AUTOSELECT") */
	cfi_command(bank, 0xaa, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, unlock1), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}
	cfi_command(bank, 0x55, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, unlock2), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}
	cfi_command(bank, 0x90, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, unlock1), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	if (bank->chip_width == 1)
	{
		u8 manufacturer, device_id;
		if((retval = target_read_u8(target, bank->base + 0x0, &manufacturer)) != ERROR_OK)
		{
			return retval;
		}
		if((retval = target_read_u8(target, bank->base + 0x1, &device_id)) != ERROR_OK)
		{
			return retval;
		}
		cfi_info->manufacturer = manufacturer;
		cfi_info->device_id = device_id;
	}
	else if (bank->chip_width == 2)
	{
		if((retval = target_read_u16(target, bank->base + 0x0, &cfi_info->manufacturer)) != ERROR_OK)
		{
			return retval;
		}
		if((retval = target_read_u16(target, bank->base + 0x2, &cfi_info->device_id)) != ERROR_OK)
		{
			return retval;
		}
	}

	/* switch back to read array mode */
	cfi_command(bank, 0xf0, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x00), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}
	cfi_command(bank, 0xff, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x00), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	cfi_fixup(bank, cfi_jedec_fixups);

	/* query only if this is a CFI compatible flash,
	 * otherwise the relevant info has already been filled in
	 */
	if (cfi_info->not_cfi == 0)
	{
		/* enter CFI query mode
		 * according to JEDEC Standard No. 68.01,
		 * a single bus sequence with address = 0x55, data = 0x98 should put
		 * the device into CFI query mode.
		 *
		 * SST flashes clearly violate this, and we will consider them incompatbile for now
		 */
		cfi_command(bank, 0x98, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x55), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}

		cfi_info->qry[0] = cfi_query_u8(bank, 0, 0x10);
		cfi_info->qry[1] = cfi_query_u8(bank, 0, 0x11);
		cfi_info->qry[2] = cfi_query_u8(bank, 0, 0x12);

		LOG_DEBUG("CFI qry returned: 0x%2.2x 0x%2.2x 0x%2.2x", cfi_info->qry[0], cfi_info->qry[1], cfi_info->qry[2]);

		if ((cfi_info->qry[0] != 'Q') || (cfi_info->qry[1] != 'R') || (cfi_info->qry[2] != 'Y'))
		{
			cfi_command(bank, 0xf0, command);
			if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
			{
				return retval;
			}
			cfi_command(bank, 0xff, command);
			if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
			{
				return retval;
			}
			LOG_ERROR("Could not probe bank");
			return ERROR_FLASH_BANK_INVALID;
		}

		cfi_info->pri_id = cfi_query_u16(bank, 0, 0x13);
		cfi_info->pri_addr = cfi_query_u16(bank, 0, 0x15);
		cfi_info->alt_id = cfi_query_u16(bank, 0, 0x17);
		cfi_info->alt_addr = cfi_query_u16(bank, 0, 0x19);

		LOG_DEBUG("qry: '%c%c%c', pri_id: 0x%4.4x, pri_addr: 0x%4.4x, alt_id: 0x%4.4x, alt_addr: 0x%4.4x", cfi_info->qry[0], cfi_info->qry[1], cfi_info->qry[2], cfi_info->pri_id, cfi_info->pri_addr, cfi_info->alt_id, cfi_info->alt_addr);

		cfi_info->vcc_min = cfi_query_u8(bank, 0, 0x1b);
		cfi_info->vcc_max = cfi_query_u8(bank, 0, 0x1c);
		cfi_info->vpp_min = cfi_query_u8(bank, 0, 0x1d);
		cfi_info->vpp_max = cfi_query_u8(bank, 0, 0x1e);
		cfi_info->word_write_timeout_typ = cfi_query_u8(bank, 0, 0x1f);
		cfi_info->buf_write_timeout_typ = cfi_query_u8(bank, 0, 0x20);
		cfi_info->block_erase_timeout_typ = cfi_query_u8(bank, 0, 0x21);
		cfi_info->chip_erase_timeout_typ = cfi_query_u8(bank, 0, 0x22);
		cfi_info->word_write_timeout_max = cfi_query_u8(bank, 0, 0x23);
		cfi_info->buf_write_timeout_max = cfi_query_u8(bank, 0, 0x24);
		cfi_info->block_erase_timeout_max = cfi_query_u8(bank, 0, 0x25);
		cfi_info->chip_erase_timeout_max = cfi_query_u8(bank, 0, 0x26);

		LOG_DEBUG("Vcc min: %1.1x.%1.1x, Vcc max: %1.1x.%1.1x, Vpp min: %1.1x.%1.1x, Vpp max: %1.1x.%1.1x",
			(cfi_info->vcc_min & 0xf0) >> 4, cfi_info->vcc_min & 0x0f,
			(cfi_info->vcc_max & 0xf0) >> 4, cfi_info->vcc_max & 0x0f,
			(cfi_info->vpp_min & 0xf0) >> 4, cfi_info->vpp_min & 0x0f,
			(cfi_info->vpp_max & 0xf0) >> 4, cfi_info->vpp_max & 0x0f);
		LOG_DEBUG("typ. word write timeout: %u, typ. buf write timeout: %u, typ. block erase timeout: %u, typ. chip erase timeout: %u", 1 << cfi_info->word_write_timeout_typ, 1 << cfi_info->buf_write_timeout_typ,
			1 << cfi_info->block_erase_timeout_typ, 1 << cfi_info->chip_erase_timeout_typ);
		LOG_DEBUG("max. word write timeout: %u, max. buf write timeout: %u, max. block erase timeout: %u, max. chip erase timeout: %u", (1 << cfi_info->word_write_timeout_max) * (1 << cfi_info->word_write_timeout_typ),
			(1 << cfi_info->buf_write_timeout_max) * (1 << cfi_info->buf_write_timeout_typ),
			(1 << cfi_info->block_erase_timeout_max) * (1 << cfi_info->block_erase_timeout_typ),
			(1 << cfi_info->chip_erase_timeout_max) * (1 << cfi_info->chip_erase_timeout_typ));

		cfi_info->dev_size = cfi_query_u8(bank, 0, 0x27);
		cfi_info->interface_desc = cfi_query_u16(bank, 0, 0x28);
		cfi_info->max_buf_write_size = cfi_query_u16(bank, 0, 0x2a);
		cfi_info->num_erase_regions = cfi_query_u8(bank, 0, 0x2c);

		LOG_DEBUG("size: 0x%x, interface desc: %i, max buffer write size: %x", 1 << cfi_info->dev_size, cfi_info->interface_desc, (1 << cfi_info->max_buf_write_size));

		if (((1 << cfi_info->dev_size) * bank->bus_width / bank->chip_width) != bank->size)
		{
			LOG_WARNING("configuration specifies 0x%x size, but a 0x%x size flash was found", bank->size, 1 << cfi_info->dev_size);
		}

		if (cfi_info->num_erase_regions)
		{
			cfi_info->erase_region_info = malloc(4 * cfi_info->num_erase_regions);
			for (i = 0; i < cfi_info->num_erase_regions; i++)
			{
				cfi_info->erase_region_info[i] = cfi_query_u32(bank, 0, 0x2d + (4 * i));
				LOG_DEBUG("erase region[%i]: %i blocks of size 0x%x", i, (cfi_info->erase_region_info[i] & 0xffff) + 1, (cfi_info->erase_region_info[i] >> 16) * 256);
			}
		}
		else
		{
			cfi_info->erase_region_info = NULL;
		}

		/* We need to read the primary algorithm extended query table before calculating
		 * the sector layout to be able to apply fixups
		 */
		switch(cfi_info->pri_id)
		{
			/* Intel command set (standard and extended) */
			case 0x0001:
			case 0x0003:
				cfi_read_intel_pri_ext(bank);
				break;
			/* AMD/Spansion, Atmel, ... command set */
			case 0x0002:
				cfi_read_0002_pri_ext(bank);
				break;
			default:
				LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
				break;
		}

		/* return to read array mode
		 * we use both reset commands, as some Intel flashes fail to recognize the 0xF0 command
		 */
		cfi_command(bank, 0xf0, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}
		cfi_command(bank, 0xff, command);
		if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command)) != ERROR_OK)
		{
			return retval;
		}
	}

	/* apply fixups depending on the primary command set */
	switch(cfi_info->pri_id)
	{
		/* Intel command set (standard and extended) */
		case 0x0001:
		case 0x0003:
			cfi_fixup(bank, cfi_0001_fixups);
			break;
		/* AMD/Spansion, Atmel, ... command set */
		case 0x0002:
			cfi_fixup(bank, cfi_0002_fixups);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	if (cfi_info->num_erase_regions == 0)
	{
		/* a device might have only one erase block, spanning the whole device */
		bank->num_sectors = 1;
		bank->sectors = malloc(sizeof(flash_sector_t));

		bank->sectors[sector].offset = 0x0;
		bank->sectors[sector].size = bank->size;
		bank->sectors[sector].is_erased = -1;
		bank->sectors[sector].is_protected = -1;
	}
	else
	{
		for (i = 0; i < cfi_info->num_erase_regions; i++)
		{
			num_sectors += (cfi_info->erase_region_info[i] & 0xffff) + 1;
		}

		bank->num_sectors = num_sectors;
		bank->sectors = malloc(sizeof(flash_sector_t) * num_sectors);

		for (i = 0; i < cfi_info->num_erase_regions; i++)
		{
			int j;
			for (j = 0; j < (cfi_info->erase_region_info[i] & 0xffff) + 1; j++)
			{
				bank->sectors[sector].offset = offset;
				bank->sectors[sector].size = ((cfi_info->erase_region_info[i] >> 16) * 256) * bank->bus_width / bank->chip_width;
				offset += bank->sectors[sector].size;
				bank->sectors[sector].is_erased = -1;
				bank->sectors[sector].is_protected = -1;
				sector++;
			}
		}
	}
	
	cfi_info->probed = 1;

	return ERROR_OK;
}

int cfi_auto_probe(struct flash_bank_s *bank)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	if (cfi_info->probed)
		return ERROR_OK;
	return cfi_probe(bank);
}


int cfi_intel_protect_check(struct flash_bank_s *bank)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_intel_pri_ext_t *pri_ext = cfi_info->pri_ext;
	target_t *target = bank->target;
	u8 command[CFI_MAX_BUS_WIDTH];
	int i;

	/* check if block lock bits are supported on this device */
	if (!(pri_ext->blk_status_reg_mask & 0x1))
		return ERROR_FLASH_OPERATION_FAILED;

	cfi_command(bank, 0x90, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, 0x55), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	for (i = 0; i < bank->num_sectors; i++)
	{
		u8 block_status = cfi_get_u8(bank, i, 0x2);

		if (block_status & 1)
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}

	cfi_command(bank, 0xff, command);
	return target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command);
}

int cfi_spansion_protect_check(struct flash_bank_s *bank)
{
	int retval;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;
	cfi_spansion_pri_ext_t *pri_ext = cfi_info->pri_ext;
	target_t *target = bank->target;
	u8 command[8];
	int i;

	cfi_command(bank, 0xaa, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	cfi_command(bank, 0x55, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock2), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	cfi_command(bank, 0x90, command);
	if((retval = target->type->write_memory(target, flash_address(bank, 0, pri_ext->_unlock1), bank->bus_width, 1, command)) != ERROR_OK)
	{
		return retval;
	}

	for (i = 0; i < bank->num_sectors; i++)
	{
		u8 block_status = cfi_get_u8(bank, i, 0x2);

		if (block_status & 1)
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}

	cfi_command(bank, 0xf0, command);
	return target->type->write_memory(target, flash_address(bank, 0, 0x0), bank->bus_width, 1, command);
}

int cfi_protect_check(struct flash_bank_s *bank)
{
	cfi_flash_bank_t *cfi_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	switch(cfi_info->pri_id)
	{
		case 1:
		case 3:
			return cfi_intel_protect_check(bank);
			break;
		case 2:
			return cfi_spansion_protect_check(bank);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_OK;
}

int cfi_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	int printed;
	cfi_flash_bank_t *cfi_info = bank->driver_priv;

	if (cfi_info->qry[0] == (char)-1)
	{
		printed = snprintf(buf, buf_size, "\ncfi flash bank not probed yet\n");
		return ERROR_OK;
	}

	if (cfi_info->not_cfi == 0)
	printed = snprintf(buf, buf_size, "\ncfi information:\n");
	else
		printed = snprintf(buf, buf_size, "\nnon-cfi flash:\n");
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "\nmfr: 0x%4.4x, id:0x%4.4x\n",
		cfi_info->manufacturer, cfi_info->device_id);
	buf += printed;
	buf_size -= printed;

	if (cfi_info->not_cfi == 0)
	{
	printed = snprintf(buf, buf_size, "qry: '%c%c%c', pri_id: 0x%4.4x, pri_addr: 0x%4.4x, alt_id: 0x%4.4x, alt_addr: 0x%4.4x\n", cfi_info->qry[0], cfi_info->qry[1], cfi_info->qry[2], cfi_info->pri_id, cfi_info->pri_addr, cfi_info->alt_id, cfi_info->alt_addr);
	buf += printed;
	buf_size -= printed;

		printed = snprintf(buf, buf_size, "Vcc min: %1.1x.%1.1x, Vcc max: %1.1x.%1.1x, Vpp min: %1.1x.%1.1x, Vpp max: %1.1x.%1.1x\n",
		                   (cfi_info->vcc_min & 0xf0) >> 4, cfi_info->vcc_min & 0x0f,
	(cfi_info->vcc_max & 0xf0) >> 4, cfi_info->vcc_max & 0x0f,
	(cfi_info->vpp_min & 0xf0) >> 4, cfi_info->vpp_min & 0x0f,
	(cfi_info->vpp_max & 0xf0) >> 4, cfi_info->vpp_max & 0x0f);
	buf += printed;
	buf_size -= printed;

		printed = snprintf(buf, buf_size, "typ. word write timeout: %u, typ. buf write timeout: %u, typ. block erase timeout: %u, typ. chip erase timeout: %u\n",
		                   1 << cfi_info->word_write_timeout_typ,
		                   1 << cfi_info->buf_write_timeout_typ,
		                   1 << cfi_info->block_erase_timeout_typ,
		                   1 << cfi_info->chip_erase_timeout_typ);
	buf += printed;
	buf_size -= printed;

		printed = snprintf(buf, buf_size, "max. word write timeout: %u, max. buf write timeout: %u, max. block erase timeout: %u, max. chip erase timeout: %u\n",
		                   (1 << cfi_info->word_write_timeout_max) * (1 << cfi_info->word_write_timeout_typ),
		  (1 << cfi_info->buf_write_timeout_max) * (1 << cfi_info->buf_write_timeout_typ),
		  (1 << cfi_info->block_erase_timeout_max) * (1 << cfi_info->block_erase_timeout_typ),
		  (1 << cfi_info->chip_erase_timeout_max) * (1 << cfi_info->chip_erase_timeout_typ));
	buf += printed;
	buf_size -= printed;

		printed = snprintf(buf, buf_size, "size: 0x%x, interface desc: %i, max buffer write size: %x\n",
		                   1 << cfi_info->dev_size,
		                   cfi_info->interface_desc,
		                   1 << cfi_info->max_buf_write_size);
	buf += printed;
	buf_size -= printed;

	switch(cfi_info->pri_id)
	{
		case 1:
		case 3:
			cfi_intel_info(bank, buf, buf_size);
			break;
		case 2:
			cfi_spansion_info(bank, buf, buf_size);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}
	}

	return ERROR_OK;
}

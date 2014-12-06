/***************************************************************************
 *   Copyright (C) 2013 Cosmin Gorgovan                                    *
 *   cosmin [at] linux-geek [dot] org                                      *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/*
	Flash driver for the Nuvoton NuMicro Mini51 and M051 series microcontrollers

	Part		 |APROM Size |Part ID (at 0x5000_0000)
	----------------------------------------------
	MINI51LAN	 4 KB			0x00205100
	MINI51ZAN	 4 KB			0x00205103
	MINI51TAN	 4 KB			0x00205104
	MINI52LAN	 8 KB			0x00205200
	MINI52ZAN	 8 KB			0x00205203
	MINI52TAN	 8 KB			0x00205204
	MINI54LAN	 16 KB		 0x00205400
	MINI54ZAN	 16 KB		 0x00205403
	MINI54TAN	 16 KB		 0x00205404
	M052LBN		  8 KB		 0x10005200
	M054LBN		 16 KB		 0x10005400
	M058LBN		 32 KB		 0x10005800
	M0516LBN	 64 KB		 0x10005A00
	M052ZBN		  8 KB		 0x10005203
	M054ZBN		 16 KB		 0x10005403
	M058ZBN		 32 KB		 0x10005803
	M0516ZBN	 64 KB		 0x10005A03
	M052LDN		  8 KB		 0x20005200
	M054LDN		 16 KB		 0x20005400
	M058LDN		 32 KB		 0x20005800
	M0516LDN	 64 KB		 0x20005A00
	M052ZDN		  8 KB		 0x20005203
	M054ZDN		 16 KB		 0x20005403
	M058ZDN		 32 KB		 0x20005803
	M0516ZDN	 64 KB		 0x20005A03
	M052LDE		  8 KB		 0x30005200
	M054LDE		 16 KB		 0x30005400
	M058LDE		 32 KB		 0x30005800
	M0516LDE	 64 KB		 0x30005A00
	M052ZDE		  8 KB		 0x30005203
	M054ZDE		 16 KB		 0x30005403
	M058ZDE		 32 KB		 0x30005803
	M0516ZDE	 64 KB		 0x30005A03

	Datasheet & TRM
	---------------

	The ISP flash programming procedure is described on pages 130 and 131 of the (not very verbose) TRM.

	http://www.keil.com/dd/docs/datashts/nuvoton/mini51/da00-mini51_52_54c1.pdf

	M051 ISP datasheet pages 190-206:
	http://www.nuvoton.com/hq/resource-download.jsp?tp_GUID=DA05-M052-54-58-516

	This driver
	-----------

	* chip_erase, erase, read and write operations have been implemented;
	* All operations support APROM, LDROM, FLASH DATA and CONFIG;

	Flash access limitations
	------------------------

	For implementing the read operation, please note that the APROM isn't memory mapped when booted from LDROM.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

#define PART_ID_REG     0x50000000
#define IPRSTC1         0x50000008
#define REGLOCKADDR     0x50000100
#define ISPCON          0x5000C000
#define ISPADR          0x5000C004
#define ISPDAT          0x5000C008
#define ISPCMD          0x5000C00C
#define ISPTRG          0x5000C010
/* Undocumented isp register */
#define ISPUNKNOWN      0x5000C01C

#define IPRSTC_CPU_RST      0x02

#define ISPCON_ISPFF        0x40
#define ISPCON_LDUEN        0x20
#define ISPCON_CFGUEN       0x10
#define ISPCON_APUEN        0x08
#define ISPCON_BS_LDROM     0x02
#define ISPCON_ISPEN        0x01

#define ISPCMD_READ         0x00
#define ISPCMD_PROGRAM      0x21
#define ISPCMD_ERASE        0x22
#define ISPCMD_CHIP_ERASE   0x26

#define ISPTRG_ISPGO        0x01

#define MINI51_APROM_BASE  0x00000000
#define MINI51_DATA_BASE   0x0001F000
#define MINI51_LDROM_BASE  0x00100000
#define MINI51_CONFIG_BASE 0x00300000

#define MINI51_KB          1024
#define MINI51_PAGE_SIZE   512
#define MINI51_TIMEOUT     1000


#define ENSURE_OK(status) if (status != ERROR_OK) return status

#define MINI51_MAX_FLASH_BANKS 4

struct mini51_flash_bank_type {
	uint32_t base;
	uint32_t size;
};

struct mini51_cpu_type {
	char *name;
	uint32_t ppid;
	unsigned n_banks;
	struct mini51_flash_bank_type bank[MINI51_MAX_FLASH_BANKS];
};

#define MINI51_BANKS_MINI51(aprom_size) \
	.n_banks = 3, \
	{ {MINI51_APROM_BASE, (aprom_size)}, {MINI51_LDROM_BASE, 2*1024}, {MINI51_CONFIG_BASE, 512} }

#define MINI51_BANKS_M051(aprom_size) \
	.n_banks = 4, \
	{ {MINI51_APROM_BASE, (aprom_size)}, {MINI51_DATA_BASE, 4*1024}, {MINI51_LDROM_BASE, 4*1024}, \
	{MINI51_CONFIG_BASE, 1024} }

static const struct mini51_cpu_type mini51_cpu[] = {
	{ "MINI51LAN", 0x00205100, MINI51_BANKS_MINI51(4*1024) },
	{ "MINI51ZAN", 0x00205103, MINI51_BANKS_MINI51(4*1024) },
	{ "MINI51TAN", 0x00205104, MINI51_BANKS_MINI51(4*1024) },
	{ "MINI52LAN", 0x00205200, MINI51_BANKS_MINI51(8*1024) },
	{ "MINI52ZAN", 0x00205203, MINI51_BANKS_MINI51(8*1024) },
	{ "MINI52TAN", 0x00205204, MINI51_BANKS_MINI51(8*1024) },
	{ "MINI54LAN", 0x00205400, MINI51_BANKS_MINI51(16*1024) },
	{ "MINI54ZAN", 0x00205403, MINI51_BANKS_MINI51(16*1024) },
	{ "MINI54TAN", 0x00205404, MINI51_BANKS_MINI51(16*1024) },

	{ "M052LBN",   0x10005200, MINI51_BANKS_M051(8*1024) },
	{ "M054LBN",   0x10005400, MINI51_BANKS_M051(16*1024) },
	{ "M058LBN",   0x10005800, MINI51_BANKS_M051(32*1024) },
	{ "M0516LBN",  0x10005A00, MINI51_BANKS_M051(64*1024) },
	{ "M052ZBN",   0x10005203, MINI51_BANKS_M051(8*1024) },
	{ "M054ZBN",   0x10005403, MINI51_BANKS_M051(16*1024) },
	{ "M058ZBN",   0x10005803, MINI51_BANKS_M051(32*1024) },
	{ "M0516ZBN",  0x10005A03, MINI51_BANKS_M051(64*1024) },
	{ "M052LDN",   0x20005200, MINI51_BANKS_M051(8*1024) },
	{ "M054LDN",   0x20005400, MINI51_BANKS_M051(16*1024) },
	{ "M058LDN",   0x20005800, MINI51_BANKS_M051(32*1024) },
	{ "M0516LDN",  0x20005A00, MINI51_BANKS_M051(64*1024) },
	{ "M052ZDN",   0x20005203, MINI51_BANKS_M051(8*1024) },
	{ "M054ZDN",   0x20005403, MINI51_BANKS_M051(16*1024) },
	{ "M058ZDN",   0x20005803, MINI51_BANKS_M051(32*1024) },
	{ "M0516ZDN",  0x20005A03, MINI51_BANKS_M051(64*1024) },
	{ "M052LDE",   0x30005200, MINI51_BANKS_M051(8*1024) },
	{ "M054LDE",   0x30005400, MINI51_BANKS_M051(16*1024) },
	{ "M058LDE",   0x30005800, MINI51_BANKS_M051(32*1024) },
	{ "M0516LDE",  0x30005A00, MINI51_BANKS_M051(64*1024) },
	{ "M052ZDE",   0x30005203, MINI51_BANKS_M051(8*1024) },
	{ "M054ZDE",   0x30005403, MINI51_BANKS_M051(16*1024) },
	{ "M058ZDE",   0x30005803, MINI51_BANKS_M051(32*1024) },
	{ "M0516ZDE",  0x30005A03, MINI51_BANKS_M051(64*1024) },
};

struct mini51_flash_bank {
	bool probed;
	const struct mini51_cpu_type *cpu;
};

/* Private methods */

static int mini51_unlock_reg(struct target *target)
{
	int status;
	status = target_write_u32(target, REGLOCKADDR, 0x59);
	if (status != ERROR_OK)
		return status;
	status = target_write_u32(target, REGLOCKADDR, 0x16);
	if (status != ERROR_OK)
		return status;
	status = target_write_u32(target, REGLOCKADDR, 0x88);
	if (status != ERROR_OK)
		return status;

	return ERROR_OK;
}


static int mini51_get_part_id(struct target *target, uint32_t *part_id)
{
	int retu = target_read_u32(target, PART_ID_REG, part_id);
	LOG_INFO("device id = 0x%08" PRIx32 "", *part_id);
	return retu;
}

static int mini51_get_cpu_type(struct target *target, const struct mini51_cpu_type** cpu)
{
	uint32_t part_id;
	int status;

	status = mini51_get_part_id(target, &part_id);
	ENSURE_OK(status);

	for (size_t i = 0; i < sizeof(mini51_cpu)/sizeof(mini51_cpu[0]); i++) {
		if (part_id == mini51_cpu[i].ppid) {
			*cpu = &mini51_cpu[i];
			LOG_INFO("device name = %s", (*cpu)->name);
			return ERROR_OK;
		}
	}

	return ERROR_FLASH_OPERATION_FAILED;
}

static int mini51_get_flash_size(struct flash_bank *bank, const struct mini51_cpu_type *cpu, uint32_t *flash_size)
{
	for (size_t i = 0; i < cpu->n_banks; i++) {
		if (bank->base == cpu->bank[i].base) {
			*flash_size = cpu->bank[i].size;
			LOG_INFO("bank base = 0x%08" PRIx32 ", size = 0x%08" PRIx32 "", bank->base, *flash_size);
			return ERROR_OK;
		}
	}
	return ERROR_FLASH_OPERATION_FAILED;
}

static int mini51_isp_execute(struct target *target)
{
	int status;
	uint32_t ispcon;
	int timeout;
	uint32_t isptrg;

	/* start ISP operation */
	status = target_write_u32(target, ISPTRG, ISPTRG_ISPGO);
	ENSURE_OK(status);

	/* Wait for for command to finish executing */
	timeout = MINI51_TIMEOUT;
	do {
		target_read_u32(target, ISPTRG, &isptrg);
		timeout--;
	} while ((isptrg & ISPTRG_ISPGO) && (timeout > 0));
	if (timeout == 0) {
		LOG_WARNING("Mini51 flash driver: Timeout executing flash command\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check for errors */
	status = target_read_u32(target, ISPCON, &ispcon);
	ENSURE_OK(status);
	if (ispcon & ISPCON_ISPFF) {
		LOG_WARNING("Mini51 flash driver: operation failed\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return status;
}

static int mini51_isp_execute_cmd(struct target *target, uint32_t cmd, uint32_t address, uint32_t data)
{
	int status;
	status = target_write_u32(target, ISPDAT, data);
	ENSURE_OK(status);
	status = target_write_u32(target, ISPADR, address);
	ENSURE_OK(status);
	status = target_write_u32(target, ISPCMD, cmd);
	ENSURE_OK(status);

	status = mini51_isp_execute(target);
	return status;
}

static int mini51_isp_execute_cmd_read(struct target *target, uint32_t cmd, uint32_t address, uint32_t *data)
{
	int status;
	status = target_write_u32(target, ISPADR, address);
	ENSURE_OK(status);
	status = target_write_u32(target, ISPCMD, cmd);
	ENSURE_OK(status);

	status = mini51_isp_execute(target);
	ENSURE_OK(status);

	status = target_read_u32(target, ISPDAT, data);
	ENSURE_OK(status);

	return status;
}

static int mini51_isp_enable(struct target *target)
{
	int status;
	uint32_t ispcon;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	status = mini51_unlock_reg(target);
	ENSURE_OK(status);
	status = target_read_u32(target, ISPCON, &ispcon);
	ENSURE_OK(status);
	ispcon |= ISPCON_ISPEN | ISPCON_LDUEN | ISPCON_APUEN | ISPCON_CFGUEN;
	status = target_write_u32(target, ISPCON, ispcon);
	return status;
}

/* Public (API) methods */

FLASH_BANK_COMMAND_HANDLER(mini51_flash_bank_command)
{
	struct mini51_flash_bank *mini51_info;
	mini51_info = malloc(sizeof(struct mini51_flash_bank));
	mini51_info->probed = false;
	bank->driver_priv = mini51_info;

	return ERROR_OK;
}

static int mini51_protect_check(struct flash_bank *bank)
{
	LOG_WARNING("Mini51 flash driver: protect_check not implemented yet\n");

	return ERROR_FLASH_OPERATION_FAILED;
}

static int mini51_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int status;
	uint32_t ispdat;
	struct target *target = bank->target;

	if ((offset & 0x3) || (count & 0x3)) {
		LOG_WARNING("Mini51 flash driver: unaligned access not supported\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	status = mini51_isp_enable(target);
	ENSURE_OK(status);

	for (uint32_t i = offset; i < offset + count; i += 4) {
		status = mini51_isp_execute_cmd_read(target, ISPCMD_READ, bank->base + i, &ispdat);
		memcpy(buffer, &ispdat, sizeof(ispdat));
		ENSURE_OK(status);
		buffer += sizeof(ispdat);
	}

	return ERROR_OK;
}


static int mini51_erase(struct flash_bank *bank, int first, int last)
{
	int status;
	struct target *target = bank->target;

	/* Enable ISP */
	status = mini51_isp_enable(target);
	ENSURE_OK(status);

	for (int page_start = first; page_start <= last; page_start++) {
		/* Set up erase command */
		uint32_t address = bank->base + page_start*MINI51_PAGE_SIZE;
		status = mini51_isp_execute_cmd(target, ISPCMD_ERASE, address, 0);
		ENSURE_OK(status);
	}

	return ERROR_OK;
}

static int mini51_protect(struct flash_bank *bank, int set, int first, int last)
{
	LOG_WARNING("Mini51 flash driver: protect operation not implemented yet\n");

	return ERROR_FLASH_OPERATION_FAILED;
}

static int mini51_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int status;
	uint32_t ispdat;
	struct target *target = bank->target;

	if ((offset & 0x3) || (count & 0x3)) {
		LOG_WARNING("Mini51 flash driver: unaligned access not supported\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	status = mini51_isp_enable(target);
	ENSURE_OK(status);

	for (uint32_t i = offset; i < offset + count; i += 4) {
		memcpy(&ispdat, buffer, sizeof(ispdat));
		status = mini51_isp_execute_cmd(target, ISPCMD_PROGRAM, bank->base + i, ispdat);
		ENSURE_OK(status);
		buffer += sizeof(ispdat);
	}

	return ERROR_OK;
}

static int mini51_probe(struct flash_bank *bank)
{
	uint32_t flash_size;
	int status;
	int num_pages;
	uint32_t offset = 0;
	const struct mini51_cpu_type *cpu;
	struct target *target = bank->target;

	status = mini51_get_cpu_type(target, &cpu);
	if (status != ERROR_OK) {
		LOG_WARNING("Mini51 flash driver: Failed to detect a known part\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	status = mini51_get_flash_size(bank, cpu, &flash_size);
	if (status != ERROR_OK) {
		LOG_WARNING("Mini51 flash driver: Failed to detect flash size\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	num_pages = flash_size / MINI51_PAGE_SIZE;

	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	bank->size = flash_size;

	for (int i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = MINI51_PAGE_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
		offset += MINI51_PAGE_SIZE;
	}

	struct mini51_flash_bank *mini51_info = bank->driver_priv;
	mini51_info->probed = true;
	mini51_info->cpu = cpu;

	return ERROR_OK;
}

static int mini51_auto_probe(struct flash_bank *bank)
{
	struct mini51_flash_bank *mini51_info = bank->driver_priv;
	if (mini51_info->probed)
		return ERROR_OK;
	return mini51_probe(bank);
}

COMMAND_HANDLER(mini51_handle_read_isp_command)
{
	uint32_t address;
	uint32_t ispdat;
	int status;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

	struct target *target = get_current_target(CMD_CTX);

	status = mini51_isp_enable(target);
	ENSURE_OK(status);
	status = mini51_isp_execute_cmd_read(target, ISPCMD_READ, address, &ispdat);
	ENSURE_OK(status);
	LOG_INFO("0x%08" PRIx32 ": 0x%08" PRIx32, address, ispdat);
	return ERROR_OK;
}

COMMAND_HANDLER(mini51_handle_write_isp_command)
{
	uint32_t address;
	uint32_t ispdat;
	int status;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], ispdat);

	struct target *target = get_current_target(CMD_CTX);

	status = mini51_isp_enable(target);
	ENSURE_OK(status);
	status = mini51_isp_execute_cmd(target, ISPCMD_PROGRAM, address, ispdat);
	ENSURE_OK(status);
	LOG_INFO("0x%08" PRIx32 ": 0x%08" PRIx32, address, ispdat);
	return ERROR_OK;
}

COMMAND_HANDLER(mini51_handle_chip_erase_command)
{
	int status;
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	status = mini51_isp_enable(target);
	ENSURE_OK(status);
	/* Write one to undocumented flash control register */
	status = target_write_u32(target, ISPUNKNOWN, 1);
	ENSURE_OK(status);

	status = mini51_isp_execute_cmd(target, ISPCMD_CHIP_ERASE, 0, 0);
	ENSURE_OK(status);
	return ERROR_OK;
}

static const struct command_registration mini51_exec_command_handlers[] = {
	{
		.name = "read_isp",
		.handler = mini51_handle_read_isp_command,
		.usage = "address",
		.mode = COMMAND_EXEC,
		.help = "read flash through ISP.",
	},
	{
		.name = "write_isp",
		.handler = mini51_handle_write_isp_command,
		.usage = "address value",
		.mode = COMMAND_EXEC,
		.help = "write flash through ISP.",
	},
	{
		.name = "chip_erase",
		.handler = mini51_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "chip erase.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration mini51_command_handlers[] = {
	{
		.name = "mini51",
		.mode = COMMAND_ANY,
		.help = "mini51 flash command group",
		.usage = "",
		.chain = mini51_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver mini51_flash = {
	.name = "mini51",
	.commands = mini51_command_handlers,
	.flash_bank_command = mini51_flash_bank_command,
	.erase = mini51_erase,
	.protect = mini51_protect,
	.write = mini51_write,
	.read = mini51_read,
	.probe = mini51_probe,
	.auto_probe = mini51_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mini51_protect_check,
};

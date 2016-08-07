/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2014 by Tomas Vanek (PSoC 4 support derived from STM32) *
 *   vanekt@fbl.cz                                                         *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* device documets:

 PSoC(R) 4: PSoC 4200 Family Datasheet
	Document Number: 001-87197 Rev. *B  Revised August 29, 2013

 PSoC 4100/4200 Family PSoC(R) 4 Architecture TRM
	Document No. 001-85634 Rev. *C March 25, 2014

 PSoC(R) 4 Registers TRM Spec.
	Document No. 001-85847 Rev. *A June 25, 2013

 CY8C41xx, CY8C42xx Programming Specifications
	Document No. 001-81799 Rev. *C March 4, 2014
*/

/* register locations */
#define PSOC4_CPUSS_SYSREQ	0x40000004
#define PSOC4_CPUSS_SYSARG	0x40000008
#define PSOC4_TEST_MODE		0x40030014
#define PSOC4_SPCIF_GEOMETRY	0x400E0000

#define PSOC4_SFLASH_MACRO	0x0ffff000

/* constants */
#define PSOC4_SROM_KEY1			0xb6
#define PSOC4_SROM_KEY2			0xd3
#define PSOC4_SROM_SYSREQ_BIT		(1<<31)
#define PSOC4_SROM_HMASTER_BIT		(1<<30)
#define PSOC4_SROM_PRIVILEGED_BIT	(1<<28)
#define PSOC4_SROM_STATUS_SUCCEEDED	0xa0000000
#define PSOC4_SROM_STATUS_FAILED	0xf0000000

#define PSOC4_CMD_GET_SILICON_ID	0
#define PSOC4_CMD_LOAD_LATCH		4
#define PSOC4_CMD_WRITE_ROW		5
#define PSOC4_CMD_PROGRAM_ROW		6
#define PSOC4_CMD_ERASE_ALL		0xa
#define PSOC4_CMD_CHECKSUM		0xb
#define PSOC4_CMD_WRITE_PROTECTION	0xd

#define PSOC4_CHIP_PROT_VIRGIN		0x0
#define PSOC4_CHIP_PROT_OPEN		0x1
#define PSOC4_CHIP_PROT_PROTECTED	0x2
#define PSOC4_CHIP_PROT_KILL		0x4


struct psoc4_chip_details {
	uint16_t id;
	const char *type;
	const char *package;
	uint32_t flash_size_in_kb;
};

/* list of PSoC 4 chips
 * flash_size_in_kb is not necessary as it can be decoded from SPCIF_GEOMETRY
 */
const struct psoc4_chip_details psoc4_devices[] = {
	/* 4200 series */
	{ 0x04A6, "CY8C4245PVI-482", "SSOP-28", .flash_size_in_kb = 32 },
	{ 0x04B6, "CY8C4245LQI-483", "QFN-40",  .flash_size_in_kb = 32 },
	{ 0x04C8, "CY8C4245AXI-483", "TQFP-44", .flash_size_in_kb = 32 },
	{ 0x04FB, "CY8C4245AXI-473", "TQFP-44", .flash_size_in_kb = 32 },
	{ 0x04F0, "CY8C4244PVI-432", "SSOP-28", .flash_size_in_kb = 16 },
	{ 0x04F1, "CY8C4244PVI-442", "SSOP-28", .flash_size_in_kb = 16 },
	{ 0x04F6, "CY8C4244LQI-443", "QFN-40",  .flash_size_in_kb = 16 },
	{ 0x04FA, "CY8C4244AXI-443", "TQFP-44", .flash_size_in_kb = 16 },

	/* 4100 series */
	{ 0x0410, "CY8C4124PVI-432", "SSOP-28", .flash_size_in_kb = 16 },
	{ 0x0411, "CY8C4124PVI-442", "SSOP-28", .flash_size_in_kb = 16 },
	{ 0x041C, "CY8C4124LQI-443", "QFN-40",  .flash_size_in_kb = 16 },
	{ 0x041A, "CY8C4124AXI-443", "TQFP-44", .flash_size_in_kb = 16 },
	{ 0x041B, "CY8C4125AXI-473", "TQFP-44", .flash_size_in_kb = 32 },
	{ 0x0412, "CY8C4125PVI-482", "SSOP-28", .flash_size_in_kb = 32 },
	{ 0x0417, "CY8C4125LQI-483", "QFN-40",  .flash_size_in_kb = 32 },
	{ 0x0416, "CY8C4125AXI-483", "TQFP-44", .flash_size_in_kb = 32 },

	/* CCG1 series */
	{ 0x0490, "CYPD1103-35FNXI", "CSP-35",  .flash_size_in_kb = 32 },
	{ 0x0489, "CYPD1121-40LQXI", "QFN-40",  .flash_size_in_kb = 32 },
	{ 0x048A, "CYPD1122-40LQXI", "QFN-40",  .flash_size_in_kb = 32 },
	{ 0x0491, "CYPD1131-35FNXI", "CSP-35",  .flash_size_in_kb = 32 },
	{ 0x0498, "CYPD1132-16SXI",  "SOIC-16", .flash_size_in_kb = 32 },
	{ 0x0481, "CYPD1134-28PVXI", "SSOP-28", .flash_size_in_kb = 32 },
	{ 0x048B, "CYPD1134-40LQXI", "QFN-40",  .flash_size_in_kb = 32 },
};


struct psoc4_flash_bank {
	uint32_t row_size;
	uint32_t user_bank_size;
	int probed;
	uint32_t silicon_id;
	uint8_t chip_protection;
	uint8_t cmd_program_row;
};


static const struct psoc4_chip_details *psoc4_details_by_id(uint32_t silicon_id)
{
	const struct psoc4_chip_details *p = psoc4_devices;
	unsigned int i;
	uint16_t id = silicon_id >> 16; /* ignore die revision */
	for (i = 0; i < sizeof(psoc4_devices)/sizeof(psoc4_devices[0]); i++, p++) {
		if (p->id == id)
			return p;
	}
	LOG_DEBUG("Unknown PSoC 4 device silicon id 0x%08" PRIx32 ".", silicon_id);
	return NULL;
}

static const char *psoc4_decode_chip_protection(uint8_t protection)
{
	switch (protection) {
	case PSOC4_CHIP_PROT_VIRGIN:
		return "protection VIRGIN";
	case PSOC4_CHIP_PROT_OPEN:
		return "protection open";
	case PSOC4_CHIP_PROT_PROTECTED:
		return "PROTECTED";
	case PSOC4_CHIP_PROT_KILL:
		return "protection KILL";
	default:
		LOG_WARNING("Unknown protection state 0x%02" PRIx8 "", protection);
		return "";
	}
}


/* flash bank <name> psoc <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(psoc4_flash_bank_command)
{
	struct psoc4_flash_bank *psoc4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	psoc4_info = calloc(1, sizeof(struct psoc4_flash_bank));

	bank->driver_priv = psoc4_info;
	psoc4_info->user_bank_size = bank->size;

	return ERROR_OK;
}


/* PSoC 4 system ROM request
 *  Setting SROM_SYSREQ_BIT in CPUSS_SYSREQ register runs NMI service
 *  in sysrem ROM. Algorithm just waits for NMI to finish.
 *  When sysreq_params_size == 0 only one parameter is passed in CPUSS_SYSARG register.
 *  Otherwise address of memory parameter block is set in CPUSS_SYSARG
 *  and the first parameter is written to the first word of parameter block
 */
static int psoc4_sysreq(struct target *target, uint8_t cmd, uint16_t cmd_param,
		uint32_t *sysreq_params, uint32_t sysreq_params_size)
{
	struct working_area *sysreq_wait_algorithm;
	struct working_area *sysreq_mem;

	struct reg_param reg_params[1];
	struct armv7m_algorithm armv7m_info;

	int retval = ERROR_OK;

	uint32_t param1 = PSOC4_SROM_KEY1
			 | ((PSOC4_SROM_KEY2 + cmd) << 8)
			 | (cmd_param << 16);

	static uint8_t psoc4_sysreq_wait_code[] = {
		/* system request NMI is served immediately after algo run
       now we are done: break */
		0x00, 0xbe,		/* bkpt 0 */
	};

	const int code_words = (sizeof(psoc4_sysreq_wait_code) + 3) / 4;
					/* stack must be aligned */
	const int stack_size = 196;
	/* tested stack sizes on PSoC 4:
		ERASE_ALL	144
		PROGRAM_ROW	112
		other sysreq	 68
	*/

	/* allocate area for sysreq wait code and stack */
	if (target_alloc_working_area(target, code_words * 4 + stack_size,
			&sysreq_wait_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for sysreq code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Write the code */
	retval = target_write_buffer(target,
			sysreq_wait_algorithm->address,
			sizeof(psoc4_sysreq_wait_code),
			psoc4_sysreq_wait_code);
	if (retval != ERROR_OK) {
		/* we already allocated the writing code, but failed to get a
		 * buffer, free the algorithm */
		goto cleanup_algo;
	}

	if (sysreq_params_size) {
		/* Allocate memory for sysreq_params */
		retval = target_alloc_working_area(target, sysreq_params_size, &sysreq_mem);
		if (retval != ERROR_OK) {
			LOG_WARNING("no working area for sysreq parameters");

			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto cleanup_algo;
		}

		/* Write sysreq_params */
		sysreq_params[0] = param1;
		retval = target_write_buffer(target, sysreq_mem->address,
				sysreq_params_size, (uint8_t *)sysreq_params);
		if (retval != ERROR_OK)
			goto cleanup_mem;

		/* Set address of sysreq parameters block */
		retval = target_write_u32(target, PSOC4_CPUSS_SYSARG, sysreq_mem->address);
		if (retval != ERROR_OK)
			goto cleanup_mem;

	} else {
		/* Sysreq without memory block of parameters */
		/* Set register parameter */
		retval = target_write_u32(target, PSOC4_CPUSS_SYSARG, param1);
		if (retval != ERROR_OK)
			goto cleanup_mem;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* sysreq stack */
	init_reg_param(&reg_params[0], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32,
		    sysreq_wait_algorithm->address + sysreq_wait_algorithm->size);

	struct armv7m_common *armv7m = target_to_armv7m(target);
	if (armv7m == NULL) {

		/* something is very wrong if armv7m is NULL */
		LOG_ERROR("unable to get armv7m target");
		goto cleanup;
	}

	/* Set SROM request */
	retval = target_write_u32(target, PSOC4_CPUSS_SYSREQ,
				  PSOC4_SROM_SYSREQ_BIT | PSOC4_SROM_HMASTER_BIT | cmd);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Execute wait code */
	retval = target_run_algorithm(target, 0, NULL,
				sizeof(reg_params) / sizeof(*reg_params), reg_params,
				sysreq_wait_algorithm->address, 0, 1000, &armv7m_info);
	if (retval != ERROR_OK)
		LOG_ERROR("sysreq wait code execution failed");

cleanup:
	destroy_reg_param(&reg_params[0]);

cleanup_mem:
	if (sysreq_params_size)
		target_free_working_area(target, sysreq_mem);

cleanup_algo:
	target_free_working_area(target, sysreq_wait_algorithm);

	return retval;
}


/* helper routine to get silicon ID from a PSoC 4 chip */
static int psoc4_get_silicon_id(struct target *target, uint32_t *silicon_id, uint8_t *protection)
{
	uint32_t params = PSOC4_SROM_KEY1
			 | ((PSOC4_SROM_KEY2 + PSOC4_CMD_GET_SILICON_ID) << 8);
	uint32_t part0, part1;

	int retval = psoc4_sysreq(target, PSOC4_CMD_GET_SILICON_ID, 0, NULL, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, PSOC4_CPUSS_SYSARG, &part0);
	if (retval != ERROR_OK)
		return retval;

	if (part0 == params) {
		LOG_ERROR("sysreq silicon id request not served");
		return ERROR_FAIL;
	}

	retval = target_read_u32(target, PSOC4_CPUSS_SYSREQ, &part1);
	if (retval != ERROR_OK)
		return retval;

	uint32_t silicon = ((part0 & 0xffff) << 16)
			| (((part0 >> 16) & 0xff) << 8)
			| (part1 & 0xff);
	uint8_t prot = (part1 >> 12) & 0xff;

	if (silicon_id)
			*silicon_id = silicon;
	if (protection)
			*protection = prot;

	LOG_DEBUG("silicon id: 0x%08" PRIx32 "", silicon);
	LOG_DEBUG("protection: 0x%02" PRIx8 "", prot);
	return retval;
}


static int psoc4_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	uint32_t prot_addr = PSOC4_SFLASH_MACRO;
	uint32_t protection;
	int i, s;
	int num_bits;
	int retval = ERROR_OK;

	num_bits = bank->num_sectors;

	for (i = 0; i < num_bits; i += 32) {
		retval = target_read_u32(target, prot_addr, &protection);
		if (retval != ERROR_OK)
			return retval;

		prot_addr += 4;

		for (s = 0; s < 32; s++) {
			if (i + s >= num_bits)
				break;
			bank->sectors[i + s].is_protected = (protection & (1 << s)) ? 1 : 0;
		}
	}

	retval = psoc4_get_silicon_id(target, NULL, &(psoc4_info->chip_protection));
	return retval;
}


static int psoc4_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int i;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Call "Erase All" system ROM API */
	uint32_t param;
	int retval = psoc4_sysreq(target, PSOC4_CMD_ERASE_ALL,
			0,
			&param, sizeof(param));

	if (retval == ERROR_OK)
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

	return retval;
}


static int psoc4_erase(struct flash_bank *bank, int first, int last)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	if (psoc4_info->cmd_program_row == PSOC4_CMD_WRITE_ROW) {
		LOG_INFO("Autoerase enabled, erase command ignored");
		return ERROR_OK;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return psoc4_mass_erase(bank);

	LOG_ERROR("Only mass erase available");

	return ERROR_FAIL;
}


static int psoc4_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	if (psoc4_info->probed == 0)
		return ERROR_FAIL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t *sysrq_buffer = NULL;
	int retval;
	int num_bits = bank->num_sectors;
	const int param_sz = 8;
	int prot_sz = num_bits / 8;
	int chip_prot = PSOC4_CHIP_PROT_OPEN;
	int flash_macro = 0; /* PSoC 42xx has only macro 0 */
	int i;

	sysrq_buffer = calloc(1, param_sz + prot_sz);
	if (sysrq_buffer == NULL) {
		LOG_ERROR("no memory for row buffer");
		return ERROR_FAIL;
	}

	for (i = first; i < num_bits && i <= last; i++)
		bank->sectors[i].is_protected = set;

	uint32_t *p = sysrq_buffer + 2;
	for (i = 0; i < num_bits; i++) {
		if (bank->sectors[i].is_protected)
			p[i / 32] |= 1 << (i % 32);
	}

	/* Call "Load Latch" system ROM API */
	sysrq_buffer[1] = prot_sz - 1;
	retval = psoc4_sysreq(target, PSOC4_CMD_LOAD_LATCH,
			0,	/* Byte number in latch from what to write */
			sysrq_buffer, param_sz + psoc4_info->row_size);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Call "Write Protection" system ROM API */
	retval = psoc4_sysreq(target, PSOC4_CMD_WRITE_PROTECTION,
			chip_prot | (flash_macro << 8), NULL, 0);
cleanup:
	if (retval != ERROR_OK)
		psoc4_protect_check(bank);

	if (sysrq_buffer)
		free(sysrq_buffer);

	return retval;
}


COMMAND_HANDLER(psoc4_handle_flash_autoerase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	bool enable = psoc4_info->cmd_program_row == PSOC4_CMD_WRITE_ROW;

	if (CMD_ARGC >= 2)
		COMMAND_PARSE_ON_OFF(CMD_ARGV[1], enable);

	if (enable) {
		psoc4_info->cmd_program_row = PSOC4_CMD_WRITE_ROW;
		LOG_INFO("Flash auto-erase enabled, non mass erase commands will be ignored.");
	} else {
		psoc4_info->cmd_program_row = PSOC4_CMD_PROGRAM_ROW;
		LOG_INFO("Flash auto-erase disabled. Use psoc mass_erase before flash programming.");
	}

	return retval;
}


static int psoc4_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t *sysrq_buffer = NULL;
	int retval = ERROR_OK;
	const int param_sz = 8;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_ERROR("offset 0x%08" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	sysrq_buffer = malloc(param_sz + psoc4_info->row_size);
	if (sysrq_buffer == NULL) {
		LOG_ERROR("no memory for row buffer");
		return ERROR_FAIL;
	}

	uint8_t *row_buffer = (uint8_t *)sysrq_buffer + param_sz;
	uint32_t row_num = offset / psoc4_info->row_size;
	uint32_t row_offset = offset - row_num * psoc4_info->row_size;
	if (row_offset)
		memset(row_buffer, 0, row_offset);

	bool save_poll = jtag_poll_get_enabled();
	jtag_poll_set_enabled(false);

	while (count) {
		uint32_t chunk_size = psoc4_info->row_size - row_offset;
		if (chunk_size > count) {
			chunk_size = count;
			memset(row_buffer + chunk_size, 0, psoc4_info->row_size - chunk_size);
		}
		memcpy(row_buffer + row_offset, buffer, chunk_size);
		LOG_DEBUG("offset / row: 0x%08" PRIx32 " / %" PRIu32 ", size %" PRIu32 "",
				offset, row_offset, chunk_size);

		/* Call "Load Latch" system ROM API */
		sysrq_buffer[1] = psoc4_info->row_size - 1;
		retval = psoc4_sysreq(target, PSOC4_CMD_LOAD_LATCH,
				0,	/* Byte number in latch from what to write */
				sysrq_buffer, param_sz + psoc4_info->row_size);
		if (retval != ERROR_OK)
			goto cleanup;

		/* Call "Program Row" or "Write Row" system ROM API */
		uint32_t sysrq_param;
		retval = psoc4_sysreq(target, psoc4_info->cmd_program_row,
				row_num & 0xffff,
				&sysrq_param, sizeof(sysrq_param));
		if (retval != ERROR_OK)
			goto cleanup;

		buffer += chunk_size;
		row_num++;
		row_offset = 0;
		count -= chunk_size;
	}

cleanup:
	jtag_poll_set_enabled(save_poll);

	if (sysrq_buffer)
		free(sysrq_buffer);

	return retval;
}


static int psoc4_probe(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flash_size_in_kb = 0;
	uint32_t max_flash_size_in_kb;
	uint32_t cpu_id;
	uint32_t silicon_id;
	uint32_t row_size;
	uint32_t base_address = 0x00000000;
	uint8_t protection;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	psoc4_info->probed = 0;
	psoc4_info->cmd_program_row = PSOC4_CMD_PROGRAM_ROW;

	/* Get the CPUID from the ARM Core
	 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0432c/DDI0432C_cortex_m0_r0p0_trm.pdf 4.2.1 */
	int retval = target_read_u32(target, 0xE000ED00, &cpu_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("cpu id = 0x%08" PRIx32 "", cpu_id);

	/* set page size, protection granularity and max flash size depending on family */
	switch ((cpu_id >> 4) & 0xFFF) {
	case 0xc20: /* M0 -> PSoC4 */
		row_size = 128;
		max_flash_size_in_kb = 32;
		break;
	default:
		LOG_WARNING("Cannot identify target as a PSoC 4 family.");
		return ERROR_FAIL;
	}

	uint32_t spcif_geometry;
	retval = target_read_u32(target, PSOC4_SPCIF_GEOMETRY, &spcif_geometry);
	if (retval == ERROR_OK) {
		row_size = 128 * ((spcif_geometry >> 22) & 3);
		flash_size_in_kb = (spcif_geometry & 0xffff) * 256 / 1024;
		LOG_INFO("SPCIF geometry: %" PRIu32 " kb flash, row %" PRIu32 " bytes.",
			 flash_size_in_kb, row_size);
	}

	/* Early revisions of ST-Link v2 have some problem reading PSOC4_SPCIF_GEOMETRY
		and an error is reported late. Dummy read gets this error. */
	uint32_t dummy;
	target_read_u32(target, PSOC4_CPUSS_SYSREQ, &dummy);

	/* get silicon ID from target. */
	retval = psoc4_get_silicon_id(target, &silicon_id, &protection);
	if (retval != ERROR_OK)
		return retval;

	const struct psoc4_chip_details *details = psoc4_details_by_id(silicon_id);
	if (details) {
		LOG_INFO("%s device detected.", details->type);
		if (flash_size_in_kb == 0)
			flash_size_in_kb = details->flash_size_in_kb;
		else if (flash_size_in_kb != details->flash_size_in_kb)
			LOG_ERROR("Flash size mismatch");
	}

	psoc4_info->row_size = row_size;
	psoc4_info->silicon_id = silicon_id;
	psoc4_info->chip_protection = protection;

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("PSoC 4 flash size failed, probe inaccurate - assuming %" PRIu32 " k flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (psoc4_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = psoc4_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %" PRIu32 " kbytes", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	uint32_t num_rows = flash_size_in_kb * 1024 / row_size;

	/* check that calculation result makes sense */
	assert(num_rows > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = num_rows * row_size;
	bank->num_sectors = num_rows;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_rows);

	uint32_t i;
	for (i = 0; i < num_rows; i++) {
		bank->sectors[i].offset = i * row_size;
		bank->sectors[i].size = row_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	LOG_INFO("flash bank set %" PRIu32 " rows", num_rows);
	psoc4_info->probed = 1;

	return ERROR_OK;
}

static int psoc4_auto_probe(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	if (psoc4_info->probed)
		return ERROR_OK;
	return psoc4_probe(bank);
}


static int get_psoc4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	int printed = 0;

	if (psoc4_info->probed == 0)
		return ERROR_FAIL;

	const struct psoc4_chip_details *details = psoc4_details_by_id(psoc4_info->silicon_id);

	if (details) {
		uint32_t chip_revision = psoc4_info->silicon_id & 0xffff;
		printed = snprintf(buf, buf_size, "PSoC 4 %s rev 0x%04" PRIx32 " package %s",
				details->type, chip_revision, details->package);
	} else
		printed = snprintf(buf, buf_size, "PSoC 4 silicon id 0x%08" PRIx32 "",
				psoc4_info->silicon_id);

	buf += printed;
	buf_size -= printed;

	const char *prot_txt = psoc4_decode_chip_protection(psoc4_info->chip_protection);
	uint32_t size_in_kb = bank->size / 1024;
	snprintf(buf, buf_size, " flash %" PRIu32 " kb %s", size_in_kb, prot_txt);
	return ERROR_OK;
}


COMMAND_HANDLER(psoc4_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = psoc4_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD_CTX, "psoc mass erase complete");
	else
		command_print(CMD_CTX, "psoc mass erase failed");

	return retval;
}


static const struct command_registration psoc4_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc4_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "flash_autoerase",
		.handler = psoc4_handle_flash_autoerase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id on|off",
		.help = "Set autoerase mode for flash bank.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc4_command_handlers[] = {
	{
		.name = "psoc4",
		.mode = COMMAND_ANY,
		.help = "PSoC 4 flash command group",
		.usage = "",
		.chain = psoc4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver psoc4_flash = {
	.name = "psoc4",
	.commands = psoc4_command_handlers,
	.flash_bank_command = psoc4_flash_bank_command,
	.erase = psoc4_erase,
	.protect = psoc4_protect,
	.write = psoc4_write,
	.read = default_flash_read,
	.probe = psoc4_probe,
	.auto_probe = psoc4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = psoc4_protect_check,
	.info = get_psoc4_info,
};

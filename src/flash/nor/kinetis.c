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
 *   Copyright (C) 2013 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

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
 */

static const struct {
	unsigned pflash_sector_size_bytes;
	unsigned nvm_sector_size_bytes;
	unsigned num_blocks;
} kinetis_flash_params[4] = {
	{ 1<<10, 1<<10, 2 },
	{ 2<<10, 1<<10, 2 },
	{ 2<<10, 2<<10, 2 },
	{ 4<<10, 4<<10, 4 }
};

/* Addressess */
#define FLEXRAM		0x14000000
#define FTFx_FSTAT	0x40020000
#define FTFx_FCNFG	0x40020001
#define FTFx_FCCOB3	0x40020004
#define FTFx_FPROT3	0x40020010
#define SIM_SDID	0x40048024
#define SIM_FCFG1	0x4004804c
#define SIM_FCFG2	0x40048050

/* Commands */
#define FTFx_CMD_BLOCKSTAT  0x00
#define FTFx_CMD_SECTSTAT   0x01
#define FTFx_CMD_LWORDPROG  0x06
#define FTFx_CMD_SECTERASE  0x09
#define FTFx_CMD_SECTWRITE  0x0b
#define FTFx_CMD_SETFLEXRAM 0x81
#define FTFx_CMD_MASSERASE  0x44

/* The Kinetis K series uses the following SDID layout :
 * Bit 31-16 : 0
 * Bit 15-12 : REVID
 * Bit 11-7  : DIEID
 * Bit 6-4   : FAMID
 * Bit 3-0   : PINID
 *
 * The Kinetis KL series uses the following SDID layout :
 * Bit 31-28 : FAMID
 * Bit 27-24 : SUBFAMID
 * Bit 23-20 : SERIESID
 * Bit 19-16 : SRAMSIZE
 * Bit 15-12 : REVID
 * Bit 6-4   : Reserved (0)
 * Bit 3-0   : PINID
 *
 * SERIESID should be 1 for the KL-series so we assume that if
 * bits 31-16 are 0 then it's a K-series MCU.
 */

#define KINETIS_SDID_K_SERIES_MASK  0x0000FFFF

#define KINETIS_SDID_DIEID_MASK 0x00000F80
#define KINETIS_SDID_DIEID_K_A	0x00000100
#define KINETIS_SDID_DIEID_K_B	0x00000200
#define KINETIS_SDID_DIEID_KL	0x00000000

/* We can't rely solely on the FAMID field to determine the MCU
 * type since some FAMID values identify multiple MCUs with
 * different flash sector sizes (K20 and K22 for instance).
 * Therefore we combine it with the DIEID bits which may possibly
 * break if Freescale bumps the DIEID for a particular MCU. */
#define KINETIS_K_SDID_TYPE_MASK 0x00000FF0
#define KINETIS_K_SDID_K10_M50	 0x00000000
#define KINETIS_K_SDID_K10_M72	 0x00000080
#define KINETIS_K_SDID_K10_M100	 0x00000100
#define KINETIS_K_SDID_K10_M120	 0x00000180
#define KINETIS_K_SDID_K11		 0x00000220
#define KINETIS_K_SDID_K12		 0x00000200
#define KINETIS_K_SDID_K20_M50	 0x00000010
#define KINETIS_K_SDID_K20_M72	 0x00000090
#define KINETIS_K_SDID_K20_M100	 0x00000110
#define KINETIS_K_SDID_K20_M120	 0x00000190
#define KINETIS_K_SDID_K21_M50   0x00000230
#define KINETIS_K_SDID_K21_M120	 0x00000330
#define KINETIS_K_SDID_K22_M50   0x00000210
#define KINETIS_K_SDID_K22_M120	 0x00000310
#define KINETIS_K_SDID_K30_M72   0x000000A0
#define KINETIS_K_SDID_K30_M100  0x00000120
#define KINETIS_K_SDID_K40_M72   0x000000B0
#define KINETIS_K_SDID_K40_M100  0x00000130
#define KINETIS_K_SDID_K50_M72   0x000000E0
#define KINETIS_K_SDID_K51_M72	 0x000000F0
#define KINETIS_K_SDID_K53		 0x00000170
#define KINETIS_K_SDID_K60_M100  0x00000140
#define KINETIS_K_SDID_K60_M150  0x000001C0
#define KINETIS_K_SDID_K70_M150  0x000001D0

#define KINETIS_KL_SDID_SERIESID_MASK 0x00F00000
#define KINETIS_KL_SDID_SERIESID_KL   0x00100000

struct kinetis_flash_bank {
	unsigned granularity;
	unsigned bank_ordinal;
	uint32_t sector_size;
	uint32_t protection_size;
	uint32_t klxx;

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

/* Kinetis Program-LongWord Microcodes */
static const uint8_t kinetis_flash_write_code[] = {
	/* Params:
	 * r0 - workarea buffer
	* r1 - target address
	* r2 - wordcount
	* Clobbered:
	* r4 - tmp
	* r5 - tmp
	* r6 - tmp
	* r7 - tmp
	*/

							/* .L1: */
						/* for(register uint32_t i=0;i<wcount;i++){ */
	0x04, 0x1C,					/* mov    r4, r0          */
	0x00, 0x23,					/* mov    r3, #0          */
							/* .L2: */
	0x0E, 0x1A,					/* sub    r6, r1, r0      */
	0xA6, 0x19,					/* add    r6, r4, r6      */
	0x93, 0x42,					/* cmp    r3, r2          */
	0x16, 0xD0,					/* beq    .L9             */
							/* .L5: */
						/* while((FTFx_FSTAT&FTFA_FSTAT_CCIF_MASK) != FTFA_FSTAT_CCIF_MASK){}; */
	0x0B, 0x4D,					/* ldr    r5, .L10        */
	0x2F, 0x78,					/* ldrb   r7, [r5]        */
	0x7F, 0xB2,					/* sxtb   r7, r7          */
	0x00, 0x2F,					/* cmp    r7, #0          */
	0xFA, 0xDA,					/* bge    .L5             */
						/* FTFx_FSTAT = FTFA_FSTAT_ACCERR_MASK|FTFA_FSTAT_FPVIOL_MASK|FTFA_FSTAT_RDCO */
	0x70, 0x27,					/* mov    r7, #112        */
	0x2F, 0x70,					/* strb   r7, [r5]        */
						/* FTFx_FCCOB3 = faddr; */
	0x09, 0x4F,					/* ldr    r7, .L10+4      */
	0x3E, 0x60,					/* str    r6, [r7]        */
	0x06, 0x27,					/* mov    r7, #6          */
						/* FTFx_FCCOB0 = 0x06;  */
	0x08, 0x4E,					/* ldr    r6, .L10+8      */
	0x37, 0x70,					/* strb   r7, [r6]        */
						/* FTFx_FCCOB7 = *pLW;  */
	0x80, 0xCC,					/* ldmia  r4!, {r7}       */
	0x08, 0x4E,					/* ldr    r6, .L10+12     */
	0x37, 0x60,					/* str    r7, [r6]        */
						/* FTFx_FSTAT = FTFA_FSTAT_CCIF_MASK; */
	0x80, 0x27,					/* mov    r7, #128        */
	0x2F, 0x70,					/* strb   r7, [r5]        */
							/* .L4: */
						/* while((FTFx_FSTAT&FTFA_FSTAT_CCIF_MASK) != FTFA_FSTAT_CCIF_MASK){}; */
	0x2E, 0x78,					/* ldrb    r6, [r5]       */
	0x77, 0xB2,					/* sxtb    r7, r6         */
	0x00, 0x2F,					/* cmp     r7, #0         */
	0xFB, 0xDA,					/* bge     .L4            */
	0x01, 0x33,					/* add     r3, r3, #1     */
	0xE4, 0xE7,					/* b       .L2            */
							/* .L9: */
	0x00, 0xBE,					/* bkpt #0                */
							/* .L10: */
	0x00, 0x00, 0x02, 0x40,		/* .word    1073872896    */
	0x04, 0x00, 0x02, 0x40,		/* .word    1073872900    */
	0x07, 0x00, 0x02, 0x40,		/* .word    1073872903    */
	0x08, 0x00, 0x02, 0x40,		/* .word    1073872904    */
};

/* Program LongWord Block Write */
static int kinetis_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t wcount)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 2048;		/* Default minimum value */
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* Params:
	 * r0 - workarea buffer
	 * r1 - target address
	 * r2 - wordcount
	 * Clobbered:
	 * r4 - tmp
	 * r5 - tmp
	 * r6 - tmp
	 * r7 - tmp
	 */

	/* Increase buffer_size if needed */
	if (buffer_size < (target->working_area_size/2))
		buffer_size = (target->working_area_size/2);

	LOG_INFO("Kinetis: FLASH Write ...");

	/* check code alignment */
	if (offset & 0x1) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(kinetis_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
		sizeof(kinetis_flash_write_code), kinetis_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 4;
		if (buffer_size <= 256) {
			/* free working area, write algorithm already allocated */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("No large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* *pLW (*buffer) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* faddr */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* number of words to program */

	/* write code buffer and use Flash programming code within kinetis       */
	/* Set breakpoint to 0 with time-out of 1000 ms                          */
	while (wcount > 0) {
		uint32_t thisrun_count = (wcount > (buffer_size / 4)) ? (buffer_size / 4) : wcount;

		retval = target_write_buffer(target, source->address, thisrun_count * 4, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);

		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
				write_algorithm->address, 0, 100000, &armv7m_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error executing kinetis Flash programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 4;
		address += thisrun_count * 4;
		wcount -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return retval;
}

static int kinetis_protect(struct flash_bank *bank, int set, int first, int last)
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

		/* read protection register */
		result = target_read_memory(bank->target, FTFx_FPROT3, 1, 4, buffer);

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

static int kinetis_ftfx_command(struct flash_bank *bank, uint8_t fcmd, uint32_t faddr,
				uint8_t fccob4, uint8_t fccob5, uint8_t fccob6, uint8_t fccob7,
				uint8_t fccob8, uint8_t fccob9, uint8_t fccoba, uint8_t fccobb,
				uint8_t *ftfx_fstat)
{
	uint8_t command[12] = {faddr & 0xff, (faddr >> 8) & 0xff, (faddr >> 16) & 0xff, fcmd,
			fccob7, fccob6, fccob5, fccob4,
			fccobb, fccoba, fccob9, fccob8};
	int result, i;
	uint8_t buffer;

	/* wait for done */
	for (i = 0; i < 50; i++) {
		result =
			target_read_memory(bank->target, FTFx_FSTAT, 1, 1, &buffer);

		if (result != ERROR_OK)
			return result;

		if (buffer & 0x80)
			break;

		buffer = 0x00;
	}

	if (buffer != 0x80) {
		/* reset error flags */
		buffer = 0x30;
		result =
			target_write_memory(bank->target, FTFx_FSTAT, 1, 1, &buffer);
		if (result != ERROR_OK)
			return result;
	}

	result = target_write_memory(bank->target, FTFx_FCCOB3, 4, 3, command);

	if (result != ERROR_OK)
		return result;

	/* start command */
	buffer = 0x80;
	result = target_write_memory(bank->target, FTFx_FSTAT, 1, 1, &buffer);
	if (result != ERROR_OK)
		return result;

	/* wait for done */
	for (i = 0; i < 240; i++) { /* Need longtime for "Mass Erase" Command Nemui Changed */
		result =
			target_read_memory(bank->target, FTFx_FSTAT, 1, 1, ftfx_fstat);

		if (result != ERROR_OK)
			return result;

		if (*ftfx_fstat & 0x80)
			break;
	}

	if ((*ftfx_fstat & 0xf0) != 0x80) {
		LOG_ERROR
			("ftfx command failed FSTAT: %02X FCCOB: %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
			 *ftfx_fstat, command[3], command[2], command[1], command[0],
			 command[7], command[6], command[5], command[4],
			 command[11], command[10], command[9], command[8]);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int kinetis_mass_erase(struct flash_bank *bank)
{
	int result;
	uint8_t ftfx_fstat;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* check if whole bank is blank */
	LOG_INFO("Execute Erase All Blocks");
	/* set command and sector address */
	result = kinetis_ftfx_command(bank, FTFx_CMD_MASSERASE, 0,
					    0, 0, 0, 0,  0, 0, 0, 0,  &ftfx_fstat);
	/* Anyway Result, write FSEC to unsecure forcely */
	/*	if (result != ERROR_OK)
		return result;*/

	/* Write to MCU security status unsecure in Flash security byte(for Kinetis-L need) */
	LOG_INFO("Write to MCU security status unsecure Anyway!");
	uint8_t padding[4] = {0xFE, 0xFF, 0xFF, 0xFF}; /* Write 0xFFFFFFFE */

	result = kinetis_ftfx_command(bank, FTFx_CMD_LWORDPROG, (bank->base + 0x0000040C),
				padding[3], padding[2], padding[1], padding[0],
				0, 0, 0, 0,  &ftfx_fstat);
	if (result != ERROR_OK)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

static int kinetis_erase(struct flash_bank *bank, int first, int last)
{
	int result, i;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first > bank->num_sectors) || (last > bank->num_sectors))
		return ERROR_FLASH_OPERATION_FAILED;

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return kinetis_mass_erase(bank);

	/*
	 * FIXME: TODO: use the 'Erase Flash Block' command if the
	 * requested erase is PFlash or NVM and encompasses the entire
	 * block.  Should be quicker.
	 */
	for (i = first; i <= last; i++) {
		uint8_t ftfx_fstat;
		/* set command and sector address */
		result = kinetis_ftfx_command(bank, FTFx_CMD_SECTERASE, bank->base + bank->sectors[i].offset,
				0, 0, 0, 0,  0, 0, 0, 0,  &ftfx_fstat);

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

static int kinetis_write(struct flash_bank *bank, const uint8_t *buffer,
			 uint32_t offset, uint32_t count)
{
	unsigned int i, result, fallback = 0;
	uint8_t buf[8];
	uint32_t wc;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;
	uint8_t *new_buffer = NULL;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (kinfo->klxx) {
		/* fallback to longword write */
		fallback = 1;
		LOG_WARNING("Kinetis L Series supports Program Longword execution only.");
		LOG_DEBUG("flash write into PFLASH @08%" PRIX32, offset);

	} else if (kinfo->flash_class == FC_FLEX_NVM) {
		uint8_t ftfx_fstat;

		LOG_DEBUG("flash write into FlexNVM @%08" PRIX32, offset);

		/* make flex ram available */
		result = kinetis_ftfx_command(bank, FTFx_CMD_SETFLEXRAM, 0x00ff0000, 0, 0, 0, 0,  0, 0, 0, 0,  &ftfx_fstat);

		if (result != ERROR_OK)
			return ERROR_FLASH_OPERATION_FAILED;

		/* check if ram ready */
		result = target_read_memory(bank->target, FTFx_FCNFG, 1, 1, buf);

		if (result != ERROR_OK)
			return result;

		if (!(buf[0] & (1 << 1))) {
			/* fallback to longword write */
			fallback = 1;

			LOG_WARNING("ram not ready, fallback to slow longword write (FCNFG: %02X)", buf[0]);
		}
	} else {
		LOG_DEBUG("flash write into PFLASH @08%" PRIX32, offset);
	}


	/* program section command */
	if (fallback == 0) {
		/*
		 * Kinetis uses different terms for the granularity of
		 * sector writes, e.g. "phrase" or "128 bits".  We use
		 * the generic term "chunk". The largest possible
		 * Kinetis "chunk" is 16 bytes (128 bits).
		 */
		unsigned prog_section_chunk_bytes = kinfo->sector_size >> 8;
		/* assume the NVM sector size is half the FlexRAM size */
		unsigned prog_size_bytes = MIN(kinfo->sector_size,
				kinetis_flash_params[kinfo->granularity].nvm_sector_size_bytes);
		for (i = 0; i < count; i += prog_size_bytes) {
			uint8_t residual_buffer[16];
			uint8_t ftfx_fstat;
			uint32_t section_count = prog_size_bytes / prog_section_chunk_bytes;
			uint32_t residual_wc = 0;

			/*
			 * Assume the word count covers an entire
			 * sector.
			 */
			wc = prog_size_bytes / 4;

			/*
			 * If bytes to be programmed are less than the
			 * full sector, then determine the number of
			 * full-words to program, and put together the
			 * residual buffer so that a full "section"
			 * may always be programmed.
			 */
			if ((count - i) < prog_size_bytes) {
				/* number of bytes to program beyond full section */
				unsigned residual_bc = (count-i) % prog_section_chunk_bytes;

				/* number of complete words to copy directly from buffer */
				wc = (count - i) / 4;

				/* number of total sections to write, including residual */
				section_count = DIV_ROUND_UP((count-i), prog_section_chunk_bytes);

				/* any residual bytes delivers a whole residual section */
				residual_wc = (residual_bc ? prog_section_chunk_bytes : 0)/4;

				/* clear residual buffer then populate residual bytes */
				(void) memset(residual_buffer, 0xff, prog_section_chunk_bytes);
				(void) memcpy(residual_buffer, &buffer[i+4*wc], residual_bc);
			}

			LOG_DEBUG("write section @ %08" PRIX32 " with length %" PRIu32 " bytes",
				  offset + i, (uint32_t)wc*4);

			/* write data to flexram as whole-words */
			result = target_write_memory(bank->target, FLEXRAM, 4, wc,
					buffer + i);

			if (result != ERROR_OK) {
				LOG_ERROR("target_write_memory failed");
				return result;
			}

			/* write the residual words to the flexram */
			if (residual_wc) {
				result = target_write_memory(bank->target,
						FLEXRAM+4*wc,
						4, residual_wc,
						residual_buffer);

				if (result != ERROR_OK) {
					LOG_ERROR("target_write_memory failed");
					return result;
				}
			}

			/* execute section-write command */
			result = kinetis_ftfx_command(bank, FTFx_CMD_SECTWRITE, bank->base + offset + i,
					section_count>>8, section_count, 0, 0,
					0, 0, 0, 0,  &ftfx_fstat);

			if (result != ERROR_OK)
				return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	/* program longword command, not supported in "SF3" devices */
	else if ((kinfo->granularity != 3) || (kinfo->klxx)) {

		if (count & 0x3) {
			uint32_t old_count = count;
			count = (old_count | 3) + 1;
			new_buffer = malloc(count);
			if (new_buffer == NULL) {
				LOG_ERROR("odd number of bytes to write and no memory "
					"for padding buffer");
				return ERROR_FAIL;
			}
			LOG_INFO("odd number of bytes to write (%" PRIu32 "), extending to %" PRIu32 " "
				"and padding with 0xff", old_count, count);
			memset(new_buffer, 0xff, count);
			buffer = memcpy(new_buffer, buffer, old_count);
		}

		uint32_t words_remaining = count / 4;

		/* try using a block write */
		int retval = kinetis_write_block(bank, buffer, offset, words_remaining);

		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* if block write failed (no sufficient working area),
			 * we use normal (slow) single word accesses */
			LOG_WARNING("couldn't use block writes, falling back to single "
				"memory accesses");

			for (i = 0; i < count; i += 4) {
				uint8_t ftfx_fstat;

				LOG_DEBUG("write longword @ %08" PRIX32, (uint32_t)(offset + i));

				uint8_t padding[4] = {0xff, 0xff, 0xff, 0xff};
				memcpy(padding, buffer + i, MIN(4, count-i));

				result = kinetis_ftfx_command(bank, FTFx_CMD_LWORDPROG, bank->base + offset + i,
						padding[3], padding[2], padding[1], padding[0],
						0, 0, 0, 0,  &ftfx_fstat);

				if (result != ERROR_OK)
					return ERROR_FLASH_OPERATION_FAILED;
			}
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
	uint32_t offset = 0;
	uint8_t fcfg1_nvmsize, fcfg1_pfsize, fcfg1_eesize, fcfg2_pflsh;
	uint32_t nvm_size = 0, pf_size = 0, ee_size = 0;
	unsigned granularity, num_blocks = 0, num_pflash_blocks = 0, num_nvm_blocks = 0,
		first_nvm_bank = 0, reassign = 0;
	struct target *target = bank->target;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	result = target_read_u32(target, SIM_SDID, &kinfo->sim_sdid);
	if (result != ERROR_OK)
		return result;

	kinfo->klxx = 0;

	/* K-series MCU? */
	if ((kinfo->sim_sdid & (~KINETIS_SDID_K_SERIES_MASK)) == 0) {
		uint32_t mcu_type = kinfo->sim_sdid & KINETIS_K_SDID_TYPE_MASK;

		switch (mcu_type) {
		case KINETIS_K_SDID_K10_M50:
		case KINETIS_K_SDID_K20_M50:
			/* 1kB sectors */
			granularity = 0;
			break;
		case KINETIS_K_SDID_K10_M72:
		case KINETIS_K_SDID_K20_M72:
		case KINETIS_K_SDID_K30_M72:
		case KINETIS_K_SDID_K30_M100:
		case KINETIS_K_SDID_K40_M72:
		case KINETIS_K_SDID_K40_M100:
		case KINETIS_K_SDID_K50_M72:
			/* 2kB sectors, 1kB FlexNVM sectors */
			granularity = 1;
			break;
		case KINETIS_K_SDID_K10_M100:
		case KINETIS_K_SDID_K20_M100:
		case KINETIS_K_SDID_K11:
		case KINETIS_K_SDID_K12:
		case KINETIS_K_SDID_K21_M50:
		case KINETIS_K_SDID_K22_M50:
		case KINETIS_K_SDID_K51_M72:
		case KINETIS_K_SDID_K53:
		case KINETIS_K_SDID_K60_M100:
			/* 2kB sectors */
			granularity = 2;
			break;
		case KINETIS_K_SDID_K10_M120:
		case KINETIS_K_SDID_K20_M120:
		case KINETIS_K_SDID_K21_M120:
		case KINETIS_K_SDID_K22_M120:
		case KINETIS_K_SDID_K60_M150:
		case KINETIS_K_SDID_K70_M150:
			/* 4kB sectors */
			granularity = 3;
			break;
		default:
			LOG_ERROR("Unsupported K-family FAMID");
			return ERROR_FLASH_OPER_UNSUPPORTED;
		}
	}
	/* KL-series? */
	else if ((kinfo->sim_sdid & KINETIS_KL_SDID_SERIESID_MASK) == KINETIS_KL_SDID_SERIESID_KL) {
		kinfo->klxx = 1;
		granularity = 0;
	} else {
		LOG_ERROR("MCU is unsupported");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	result = target_read_u32(target, SIM_FCFG1, &kinfo->sim_fcfg1);
	if (result != ERROR_OK)
		return result;

	result = target_read_u32(target, SIM_FCFG2, &kinfo->sim_fcfg2);
	if (result != ERROR_OK)
		return result;
	fcfg2_pflsh = (kinfo->sim_fcfg2 >> 23) & 0x01;

	LOG_DEBUG("SDID: 0x%08" PRIX32 " FCFG1: 0x%08" PRIX32 " FCFG2: 0x%08" PRIX32, kinfo->sim_sdid,
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

	LOG_DEBUG("FlexNVM: %" PRIu32 " PFlash: %" PRIu32 " FlexRAM: %" PRIu32 " PFLSH: %d",
		  nvm_size, pf_size, ee_size, fcfg2_pflsh);
	if (kinfo->klxx)
		num_blocks = 1;
	else
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
					LOG_WARNING("Class mismatch, bank %d is not PFlash", bank->bank_number);
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
					LOG_WARNING("Class mismatch, bank %d is not FlexNVM", bank->bank_number);
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
					LOG_WARNING("Class mismatch, bank %d is not FlexRAM", bank->bank_number);
					reassign = 1;
				} else if (bank->size != ee_size) {
					LOG_WARNING("FlexRAM size mismatch");
					reassign = 1;
				} else if (bank->base != FLEXRAM) {
					LOG_WARNING("FlexRAM address mismatch");
					reassign = 1;
				} else if (kinfo->sector_size !=
					 kinetis_flash_params[granularity].nvm_sector_size_bytes) {
					LOG_WARNING("FlexRAM sector size mismatch");
					reassign = 1;
				} else {
					LOG_DEBUG("FlexRAM bank %d already configured okay", kinfo->bank_ordinal);
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
		uint8_t ftfx_fstat;

		/* check if whole bank is blank */
		result = kinetis_ftfx_command(bank, FTFx_CMD_BLOCKSTAT, bank->base, 0, 0, 0, 0,  0, 0, 0, 0, &ftfx_fstat);

		if (result != ERROR_OK)
			return result;

		if (ftfx_fstat & 0x01) {
			/* the whole bank is not erased, check sector-by-sector */
			int i;
			for (i = 0; i < bank->num_sectors; i++) {
				/* normal margin */
				result = kinetis_ftfx_command(bank, FTFx_CMD_SECTSTAT, bank->base + bank->sectors[i].offset,
						1, 0, 0, 0,  0, 0, 0, 0, &ftfx_fstat);

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

struct flash_driver kinetis_flash = {
	.name = "kinetis",
	.flash_bank_command = kinetis_flash_bank_command,
	.erase = kinetis_erase,
	.protect = kinetis_protect,
	.write = kinetis_write,
	.read = default_flash_read,
	.probe = kinetis_probe,
	.auto_probe = kinetis_auto_probe,
	.erase_check = kinetis_blank_check,
	.protect_check = kinetis_protect_check,
	.info = kinetis_info,
};

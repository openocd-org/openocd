/***************************************************************************
 *   Copyright (C) 2011 by Marc Willam, Holger Wech                        *
 *   m.willam@gmx.eu                                                       *
 *   Copyright (C) 2011 Ronny Strutz                                       *
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
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLASH_DQ6 0x00000040	/* Data toggle flag bit (TOGG) position */
#define FLASH_DQ5 0x00000020	/* Time limit exceeding flag bit (TLOV) position */

enum fm3_variant
{
	mb9bfxx1,
	mb9bfxx2,
	mb9bfxx3,
	mb9bfxx4,
	mb9bfxx5,
	mb9bfxx6
};

struct fm3_flash_bank
{
	struct working_area *write_algorithm;
	enum fm3_variant variant;
	int probed;
};

FLASH_BANK_COMMAND_HANDLER(fm3_flash_bank_command)
{
	struct fm3_flash_bank *fm3_info;

	if (CMD_ARGC < 6)
	{
		LOG_WARNING("incomplete flash_bank fm3 configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	LOG_INFO("******HWE* FLASH CMD Parameter %s", CMD_ARGV[5]);

	fm3_info = malloc(sizeof(struct fm3_flash_bank));
	bank->driver_priv = fm3_info;

	if (strcmp(CMD_ARGV[5], "mb9bfxx1.cpu") == 0)
	{
		fm3_info->variant = mb9bfxx1;
	}
	else if (strcmp(CMD_ARGV[5], "mb9bfxx2.cpu") == 0)
	{
		fm3_info->variant = mb9bfxx2;
	}
	else if (strcmp(CMD_ARGV[5], "mb9bfxx3.cpu") == 0)
	{
		fm3_info->variant = mb9bfxx3;
	}
	else if (strcmp(CMD_ARGV[5], "mb9bfxx4.cpu") == 0)
	{
		fm3_info->variant = mb9bfxx4;
	}
	else if (strcmp(CMD_ARGV[5], "mb9bfxx5.cpu") == 0)
	{
		fm3_info->variant = mb9bfxx5;
	}
	else if (strcmp(CMD_ARGV[5], "mb9bfxx6.cpu") == 0)
	{
		fm3_info->variant = mb9bfxx6;
		LOG_INFO("******HWE* fm3 Variant set to: mb9bfxx6");
	}
	else
	{
		LOG_ERROR("unknown fm3 variant: %s", CMD_ARGV[5]);
		free(fm3_info);
		return ERROR_FLASH_BANK_INVALID;
	}

	fm3_info->write_algorithm = NULL;
	fm3_info->probed = 0;

	return ERROR_OK;
}

static int fm3_busy_wait(struct target *target, uint32_t offset, int timeout_ms)
{
	int retval = ERROR_OK;
	uint16_t state1, state2;
	int ms = 0;

	while(1) {
		target_read_u16(target, offset, &state1);	/* dummy-read - see flash manual */
		target_read_u16(target, offset, &state1);
		target_read_u16(target, offset, &state2);

		if ( (state1 & FLASH_DQ6) == (state2 & FLASH_DQ6) ) {
			break;
		}
		else if (state1 & FLASH_DQ5) {
			target_read_u16(target, offset, &state1);
			target_read_u16(target, offset, &state2);
			if ( (state1 & FLASH_DQ6) != (state2 & FLASH_DQ6) )
				retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
		usleep(1000);
		++ms;

		if (ms > timeout_ms) {
			LOG_ERROR("toggle bit reading timed out!");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
	}

	if (retval == ERROR_OK)
		LOG_DEBUG("fm3_busy_wait(%x) needs about %d ms", offset, ms);

	return retval;
}

static int fm3_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int retval = ERROR_OK;
	uint32_t u32DummyRead;
	int sector, odd;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Fujitsu MB9Bxxx: Sector Erase ... (%d to %d)", first, last);

	target_write_u32(target, 0x40000000, 0x0001);		/* FASZR = 0x01, Enables CPU Programming Mode */
	target_read_u32(target, 0x40000000, &u32DummyRead); /* dummy read of FASZR */

	for (sector = first ; sector <= last ; sector++) {
		uint32_t offset = bank->sectors[sector].offset;

		for (odd = 0; odd < 2 ; odd++) {

			if (odd)
				offset += 4;

			target_write_u16(target, 0x1550, 0x00AA);
			target_write_u16(target, 0x0AA8, 0x0055);
			target_write_u16(target, 0x1550, 0x0080);
			target_write_u16(target, 0x1550, 0x00AA);
			target_write_u16(target, 0x0AA8, 0x0055);
			target_write_u16(target, offset, 0x0030);

			retval = fm3_busy_wait(target, offset, 500);

			if (retval != ERROR_OK)
				break;
		}
		bank->sectors[sector].is_erased = 1;
	}

	target_write_u32(target, 0x40000000, 0x0002);
	target_read_u32(target, 0x40000000, &u32DummyRead); /* dummy read of FASZR */

	return retval;
}

static int fm3_write_block(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct fm3_flash_bank *fm3_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 8192;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* RAMCODE used for fm3 Flash programming:                 */
	/* R0 keeps source start address         (u32Source)       */
	/* R1 keeps target start address         (u32Target)       */
	/* R2 keeps number of halfwords to write (u32Count)        */
	/* R3 returns result value               (u32FlashResult)  */

	const uint8_t fm3_flash_write_code[] = {
								/*    fm3_FLASH_IF->FASZ &= 0xFFFD;                               */
	0x00, 0xBF,                 /*        NOP                                                     */
	0x5F, 0xF0, 0x80, 0x43,     /*        MOVS.W   R3, #(fm3_FLASH_IF->FASZ)                      */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	0x4F, 0xF6, 0xFD, 0x74,     /*        MOVW     R4, #0xFFFD                                    */
	0x23, 0x40,                 /*        ANDS     R3, R3, R4                                     */
	0x5F, 0xF0, 0x80, 0x44,     /*        MOVS.W   R4, #(fm3_FLASH_IF->FASZ)                      */
	0x23, 0x60,                 /*        STR      R3, [R4]                                       */
								/*    fm3_FLASH_IF->FASZ |= 1;                                    */
	0x5F, 0xF0, 0x80, 0x43,     /*        MOVS.W   R3, #(fm3_FLASH_IF->FASZ)                      */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	0x53, 0xF0, 0x01, 0x03,     /*        ORRS.W   R3, R3, #1                                     */
	0x5F, 0xF0, 0x80, 0x44,     /*        MOVS.W   R4, #(fm3_FLASH_IF->FASZ)                      */
	0x23, 0x60,                 /*        STR      R3, [R4]                                       */
	                            /*    u32DummyRead = fm3_FLASH_IF->FASZ;                          */
	0x2B, 0x4B,                 /*        LDR.N    R3, ??u32DummyRead                             */
	0x5F, 0xF0, 0x80, 0x44,     /*        MOVS.W   R4, #(fm3_FLASH_IF->FASZ)                      */
	0x24, 0x68,                 /*        LDR      R4, [R4]                                       */
	0x1C, 0x60,                 /*        STR      R4, [R3]                                       */
	                            /*    u32FlashResult = FLASH_WRITE_NO_RESULT                      */
	0x2A, 0x4B,                 /*        LDR.N    R3, ??u32FlashResult                           */
	0x00, 0x24,                 /*        MOVS     R4, #0                                         */
	0x1C, 0x60,                 /*        STR      R4, [R3]                                       */
	                            /*    while ((u32Count > 0 ) && (u32FlashResult                   */
	                            /*      == FLASH_WRITE_NO_RESULT))                                */
	0x01, 0x2A,                 /* L0:    CMP      R2, #1                                         */
	0x32, 0xDB,                 /*        BLT.N    L1                                             */
	0x27, 0x4B,                 /*        LDR.N    R3, ??u32FlashResult                           */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	0x00, 0x2B,                 /*        CMP      R3, #0                                         */
	0x2E, 0xD1,                 /*        BNE.N    L1                                             */
	                            /*    *(FLASH_SEQ_1550) = FLASH_WRITE_1;                          */
	0x41, 0xF2, 0x50, 0x53,     /*        MOVW     R3, #0x1550                                    */
	0xAA, 0x24,                 /*        MOVS     R4. #0xAA                                      */
	0x1C, 0x80,                 /*        STRH     R4, [R3]                                       */
	                            /*    *(FLASH_SEQ_0AA8) = FLASH_WRITE_2;                          */
	0x40, 0xF6, 0xA8, 0x23,     /*        MOVW     R3, #0x0AA8                                    */
	0x55, 0x24,                 /*        MOVS     R4. #0x55                                      */
	0x1C, 0x80,                 /*        STRH     R4, [R3]                                       */
	                            /*    *(FLASH_SEQ_1550) = FLASH_WRITE_3;                          */
	0x41, 0xF2, 0x50, 0x53,     /*        MOVW     R3, #0x1550                                    */
	0xA0, 0x24,                 /*        MOVS     R4. #0xA0                                      */
	0x1C, 0x80,                 /*        STRH     R4, [R3]                                       */
	                            /*    *(volatile uint16_t*)u32Target                              */
	                            /*      = *(volatile uint16_t*)u32Source;                         */
	0x03, 0x88,                 /*        LDRH     R3, [R0]                                       */
	0x0B, 0x80,                 /*        STRH     R3, [R1]                                       */
	                            /*    while (u32FlashResult == FLASH_WRITE_NO_RESTULT)            */
	0x1E, 0x4B,                 /* L2:    LDR.N    R3, ??u32FlashResult                           */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	0x00, 0x2B,                 /*        CMP      R3, #0                                         */
	0x11, 0xD1,                 /*        BNE.N    L3                                             */
	                            /*    if ((*(volatile uint16_t*)u32Target & FLASH_DQ5)             */
	                            /*      == FLASH_DQ5)                                             */
	0x0B, 0x88,                 /*        LDRH     R3, [R1]                                       */
	0x9B, 0x06,                 /*        LSLS     R3, R3, #0x1A                                  */
	0x02, 0xD5,                 /*        BPL.N    L4                                             */
	                            /*    u32FlashResult = FLASH_WRITE_TIMEOUT                        */
	0x1B, 0x4B,                 /*        LDR.N    R3, ??u32FlashResult                           */
	0x02, 0x24,                 /*        MOVS     R4, #2                                         */
	0x1C, 0x60,                 /*        STR      R4, [R3]                                       */
	                            /*    if ((*(volatile uint16_t *)u32Target & FLASH_DQ7)            */
	                            /*      == (*(volatile uint16_t*)u32Source & FLASH_DQ7))          */
	0x0B, 0x88,                 /* L4:    LDRH     R3, [R1]                                       */
	0x13, 0xF0, 0x80, 0x03,     /*        ANDS.W   R3, R3, #0x80                                  */
	0x04, 0x88,                 /*        LDRH     R4, [R0]                                       */
	0x14, 0xF0, 0x80, 0x04,     /*        ANDS.W   R4, R4, #0x80                                  */
	0xA3, 0x42,                 /*        CMP      R3, R4                                         */
	0xED, 0xD1,                 /*        BNE.N    L2                                             */
	                            /*    u32FlashResult = FLASH_WRITE_OKAY                           */
	0x15, 0x4B,                 /*        LDR.N    R3, ??u32FlashResult                           */
	0x01, 0x24,                 /*        MOVS     R4, #1                                         */
	0x1C, 0x60,                 /*        STR      R4, [R3]                                       */
	0xE9, 0xE7,                 /*        B.N      L2                                             */
	                            /*    if (u32FlashResult != FLASH_WRITE_TIMEOUT)                   */
	0x13, 0x4B,                 /*        LDR.N    R3, ??u32FlashResult                           */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	0x02, 0x2B,                 /*        CMP      R3, #2                                         */
	0x02, 0xD0,                 /*        BEQ.N    L5                                             */
	                            /*    u32FlashResult = FLASH_WRITE_NO_RESULT                      */
	0x11, 0x4B,                 /*        LDR.N    R3, ??u32FlashResult                           */
	0x00, 0x24,                 /*        MOVS     R4, #0                                         */
	0x1C, 0x60,                 /*        STR      R4, [R3]                                       */
	                            /*    u32Count--;                                                 */
	0x52, 0x1E,                 /* L5:    SUBS     R2, R2, #1                                     */
	                            /*    u32Source += 2;                                             */
	0x80, 0x1C,                 /*        ADDS     R0, R0, #2                                     */
	                            /*    u32Target += 2;                                             */
	0x89, 0x1C,                 /*        ADDS     R1, R1, #2                                     */
	0xCA, 0xE7,                 /*        B.N      L0                                             */
	                            /*    fm3_FLASH_IF->FASZ &= 0xFFFE;                               */
	0x5F, 0xF0, 0x80, 0x43,     /* L1:    MOVS.W   R3, #(fm3_FLASH_IF->FASZ)                      */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	0x4F, 0xF6, 0xFE, 0x74,     /*        MOVW     R4, #0xFFFE                                    */
	0x23, 0x40,                 /*        ANDS     R3, R3, R4                                     */
	0x5F, 0xF0, 0x80, 0x44,     /*        MOVS.W   R4, #(fm3_FLASH_IF->FASZ)                      */
	0x23, 0x60,                 /*        STR      R3, [R4]                                       */
	                            /*    fm3_FLASH_IF->FASZ |= 2;                                    */
	0x5F, 0xF0, 0x80, 0x43,     /*        MOVS.W   R3, #(fm3_FLASH_IF->FASZ)                      */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	0x53, 0xF0, 0x02, 0x03,     /*        ORRS.W   R3, R3, #2                                     */
	0x5F, 0xF0, 0x80, 0x44,     /*        MOVS.W   R4, #(fm3_FLASH_IF->FASZ)                      */
	0x23, 0x60,                 /*        STR      R4, [R3]                                       */
	                            /*    u32DummyRead = fm3_FLASH_IF->FASZ;                          */
	0x04, 0x4B,                 /*        LDR.N    R3, ??u32DummyRead                             */
	0x5F, 0xF0, 0x80, 0x44,     /*        MOVS.W   R4, #(fm3_FLASH_IF->FASZ)                      */
	0x24, 0x68,                 /*        LDR      R4, [R4]                                       */
	0x1C, 0x60,                 /*        STR      R4, [R3]                                       */
	                            /*    copy u32FlashResult to R3 for return value                  */
	0xDF, 0xF8, 0x0C, 0x30,     /*        LDR.W    R3, ??u32FlashResult                           */
	0x1B, 0x68,                 /*        LDR      R3, [R3]                                       */
	                            /*    Breakpoint here                                             */
	0x00, 0xBE,                 /* Breakpoint #0                                                  */
	0x00, 0x00,                 /*    alignment padding bytes                                     */
	0x00, 0x80, 0xFF, 0x1F,     /* u32DummyRead address in RAM (0x1FFF8000)                       */
	0x04, 0x80, 0xFF, 0x1F      /* u32FlashResult address in RAM (0x1FFF8004)                     */
	};

	LOG_INFO("Fujitsu MB9B500: FLASH Write ...");

	/* disable HW watchdog */
	target_write_u32(target, 0x40011C00, 0x1ACCE551);
	target_write_u32(target, 0x40011C00, 0xE5331AAE);
	target_write_u32(target, 0x40011008, 0x00000000);

	count = count / 2;		/* number bytes -> number halfwords */

	/* check code alignment */
	if (offset & 0x1)
	{
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(fm3_flash_write_code),
			&fm3_info->write_algorithm) != ERROR_OK)
	{
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, fm3_info->write_algorithm->address,
		sizeof(fm3_flash_write_code), fm3_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* free working area, if write algorithm already allocated */
			if (fm3_info->write_algorithm)
			{
				target_free_working_area(target, fm3_info->write_algorithm);
			}

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); // source start address
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); // target start address
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); // number of halfwords to program
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN);  // result

	/* write code buffer and use Flash programming code within fm3           */
	/* Set breakpoint to 0 with time-out of 1000 ms                          */
	while (count > 0)
	{
		uint32_t thisrun_count = (count > (buffer_size / 2)) ? (buffer_size / 2) : count;

		/* for some reason the first 8 byte of code are corrupt when target_run_algorithm() returns */
		/* need some more investigation on this                                                     */
		retval = target_write_buffer(target,
				fm3_info->write_algorithm->address, 8, fm3_flash_write_code);
		if (retval != ERROR_OK)
			return retval;


		retval = target_write_buffer(target,
				source->address, thisrun_count * 2, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);


		retval = target_run_algorithm(target, 0, NULL, 4, reg_params,
				fm3_info->write_algorithm->address, 0, 1000, &armv7m_info);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("error executing fm3 Flash programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

#if 0
		/* debug the corrupted 8 bytes */
		unsigned char buf[256];
		retval = target_read_buffer(target, fm3_info->write_algorithm->address, 256, buf);
		if (retval != ERROR_OK)
			printf("cannot read buffer\n");
		unsigned int i;
		for ( i = 0; i < sizeof(fm3_flash_write_code); i++)
			if (buf[i] != fm3_flash_write_code[i])
				printf("broken: %d %02x != %02x\n", i, buf[i], fm3_flash_write_code[i]);
#endif

		if (buf_get_u32(reg_params[3].value, 0, 32) != ERROR_OK)
		{
			LOG_ERROR("Fujitsu MB9B500: FLASH programming ERROR (Timeout) -> Reg R3: %x",
					buf_get_u32(reg_params[3].value, 0, 32));
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer  += thisrun_count * 2;
		address += thisrun_count * 2;
		count   -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, fm3_info->write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}

static int fm3_probe(struct flash_bank *bank)
{
	struct fm3_flash_bank *fm3_info = bank->driver_priv;
	uint16_t num_pages;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	num_pages = 6;				/* max number of Flash pages for malloc */
	fm3_info->probed = 0;

	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	bank->base = 0x00000000;
	num_pages = 2;				/* start with smallest Flash pages number */
	bank->size = 32 * 1024;		/* bytes */

	bank->sectors[0].offset = 0;
	bank->sectors[0].size = 16 * 1024;
	bank->sectors[0].is_erased = -1;
	bank->sectors[0].is_protected = -1;

	bank->sectors[1].offset = 0x4000;
	bank->sectors[1].size = 16 * 1024;
	bank->sectors[1].is_erased = -1;
	bank->sectors[1].is_protected = -1;

	if (fm3_info->variant == mb9bfxx1)
	{
		num_pages = 3;
		bank->size = 64 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[2].offset = 0x8000;
		bank->sectors[2].size = 32 * 1024;
		bank->sectors[2].is_erased = -1;
		bank->sectors[2].is_protected = -1;
	}

	if (  (fm3_info->variant == mb9bfxx2)
		|| (fm3_info->variant == mb9bfxx4)
		|| (fm3_info->variant == mb9bfxx5)
		|| (fm3_info->variant == mb9bfxx6))
	{
		num_pages = 3;
		bank->size = 128 * 1024; // bytes
		bank->num_sectors = num_pages;

		bank->sectors[2].offset = 0x8000;
		bank->sectors[2].size = 96 * 1024;
		bank->sectors[2].is_erased = -1;
		bank->sectors[2].is_protected = -1;
	}

	if ( (fm3_info->variant == mb9bfxx4)
		|| (fm3_info->variant == mb9bfxx5)
		|| (fm3_info->variant == mb9bfxx6))
	{
		num_pages = 4;
		bank->size = 256 * 1024; // bytes
		bank->num_sectors = num_pages;

		bank->sectors[3].offset = 0x20000;
		bank->sectors[3].size = 128 * 1024;
		bank->sectors[3].is_erased = -1;
		bank->sectors[3].is_protected = -1;
	}

	if ( (fm3_info->variant == mb9bfxx5)
		|| (fm3_info->variant == mb9bfxx6))
	{
		num_pages = 5;
		bank->size = 384 * 1024; // bytes
		bank->num_sectors = num_pages;

		bank->sectors[4].offset = 0x40000;
		bank->sectors[4].size = 128 * 1024;
		bank->sectors[4].is_erased = -1;
		bank->sectors[4].is_protected = -1;
	}

	if (fm3_info->variant == mb9bfxx6)
	{
		num_pages = 6;
		bank->size = 512 * 1024; // bytes
		bank->num_sectors = num_pages;

		bank->sectors[5].offset = 0x60000;
		bank->sectors[5].size = 128 * 1024;
		bank->sectors[5].is_erased = -1;
		bank->sectors[5].is_protected = -1;
	}

	fm3_info->probed = 1;

	return ERROR_OK;
}

static int fm3_auto_probe(struct flash_bank *bank)
{
	struct fm3_flash_bank *fm3_info = bank->driver_priv;
	if (fm3_info->probed)
		return ERROR_OK;
	return fm3_probe(bank);
}

static int fm3_info(struct flash_bank *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "Fujitsu fm3 Device does not support Chip-ID (Type unknown)");
	return ERROR_OK;
}

static int fm3_chip_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval = ERROR_OK;
	uint32_t u32DummyRead;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Fujitsu MB9Bxxx: Chip Erase ... (may take several seconds)");

	/* Implement Flash chip erase (mass erase) completely on host */
	target_write_u32(target, 0x40000000, 0x0001);		/* FASZR = 0x01, Enables CPU Programming Mode (16-bit Flash access) */
	target_read_u32(target, 0x40000000, &u32DummyRead); /* dummy read of FASZR */

	target_write_u16(target, 0x00001550, 0x00AA); 		/* Flash unlock sequence */
	target_write_u16(target, 0x00000AA8, 0x0055);
	target_write_u16(target, 0x00001550, 0x0080);
	target_write_u16(target, 0x00001550, 0x00AA);
	target_write_u16(target, 0x00000AA8, 0x0055);
	target_write_u16(target, 0x00001550, 0x0010); 		/* Chip Erase command */

	retval = fm3_busy_wait(target, 0xAA8, 20000);

	target_write_u32(target, 0x40000000, 0x0002);
	target_read_u32(target, 0x40000000, &u32DummyRead); /* dummy read of FASZR */

	return retval;
}

COMMAND_HANDLER(fm3_handle_chip_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "fm3 chip_erase <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (fm3_chip_erase(bank) == ERROR_OK)
	{
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "fm3 chip erase complete");
	}
	else
	{
		command_print(CMD_CTX, "fm3 chip erase failed");
	}

	return ERROR_OK;
}

static const struct command_registration fm3_exec_command_handlers[] = {
	{
		.name = "chip_erase",
		.handler = fm3_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire Flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration fm3_command_handlers[] = {
	{
		.name = "fm3",
		.mode = COMMAND_ANY,
		.help = "fm3 Flash command group",
		.chain = fm3_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver fm3_flash = {
	.name = "fm3",
	.commands = fm3_command_handlers,
	.flash_bank_command = fm3_flash_bank_command,
	.erase = fm3_erase,
	.write = fm3_write_block,
	.probe = fm3_probe,
	.auto_probe = fm3_auto_probe,
	.erase_check = default_flash_mem_blank_check,
	.info = fm3_info,
};

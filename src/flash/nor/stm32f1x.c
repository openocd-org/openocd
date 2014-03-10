/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *
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

/* stm32x register locations */

#define FLASH_REG_BASE_B0 0x40022000
#define FLASH_REG_BASE_B1 0x40022040

#define STM32_FLASH_ACR     0x00
#define STM32_FLASH_KEYR    0x04
#define STM32_FLASH_OPTKEYR 0x08
#define STM32_FLASH_SR      0x0C
#define STM32_FLASH_CR      0x10
#define STM32_FLASH_AR      0x14
#define STM32_FLASH_OBR     0x1C
#define STM32_FLASH_WRPR    0x20

/* TODO: Check if code using these really should be hard coded to bank 0.
 * There are valid cases, on dual flash devices the protection of the
 * second bank is done on the bank0 reg's. */
#define STM32_FLASH_ACR_B0     0x40022000
#define STM32_FLASH_KEYR_B0    0x40022004
#define STM32_FLASH_OPTKEYR_B0 0x40022008
#define STM32_FLASH_SR_B0      0x4002200C
#define STM32_FLASH_CR_B0      0x40022010
#define STM32_FLASH_AR_B0      0x40022014
#define STM32_FLASH_OBR_B0     0x4002201C
#define STM32_FLASH_WRPR_B0    0x40022020

/* option byte location */

#define STM32_OB_RDP		0x1FFFF800
#define STM32_OB_USER		0x1FFFF802
#define STM32_OB_DATA0		0x1FFFF804
#define STM32_OB_DATA1		0x1FFFF806
#define STM32_OB_WRP0		0x1FFFF808
#define STM32_OB_WRP1		0x1FFFF80A
#define STM32_OB_WRP2		0x1FFFF80C
#define STM32_OB_WRP3		0x1FFFF80E

/* FLASH_CR register bits */

#define FLASH_PG		(1 << 0)
#define FLASH_PER		(1 << 1)
#define FLASH_MER		(1 << 2)
#define FLASH_OPTPG		(1 << 4)
#define FLASH_OPTER		(1 << 5)
#define FLASH_STRT		(1 << 6)
#define FLASH_LOCK		(1 << 7)
#define FLASH_OPTWRE	(1 << 9)

/* FLASH_SR register bits */

#define FLASH_BSY		(1 << 0)
#define FLASH_PGERR		(1 << 2)
#define FLASH_WRPRTERR	(1 << 4)
#define FLASH_EOP		(1 << 5)

/* STM32_FLASH_OBR bit definitions (reading) */

#define OPT_ERROR		0
#define OPT_READOUT		1
#define OPT_RDWDGSW		2
#define OPT_RDRSTSTOP	3
#define OPT_RDRSTSTDBY	4
#define OPT_BFB2		5	/* dual flash bank only */

/* register unlock keys */

#define KEY1			0x45670123
#define KEY2			0xCDEF89AB

/* timeout values */

#define FLASH_WRITE_TIMEOUT 10
#define FLASH_ERASE_TIMEOUT 100

struct stm32x_options {
	uint16_t RDP;
	uint16_t user_options;
	uint16_t user_data;
	uint16_t protection[4];
};

struct stm32x_flash_bank {
	struct stm32x_options option_bytes;
	int ppage_size;
	int probed;

	bool has_dual_banks;
	/* used to access dual flash bank stm32xl */
	uint32_t register_base;
	uint16_t default_rdp;
	int user_data_offset;
	int option_offset;
	uint32_t user_bank_size;
};

static int stm32x_mass_erase(struct flash_bank *bank);
static int stm32x_get_device_id(struct flash_bank *bank, uint32_t *device_id);
static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count);

/* flash bank stm32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32x_info = malloc(sizeof(struct stm32x_flash_bank));

	bank->driver_priv = stm32x_info;
	stm32x_info->probed = 0;
	stm32x_info->has_dual_banks = false;
	stm32x_info->register_base = FLASH_REG_BASE_B0;
	stm32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline int stm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	return reg + stm32x_info->register_base;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR), status);
}

static int stm32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPRTERR) {
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	if (status & FLASH_PGERR) {
		LOG_ERROR("stm32x device programming failed");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & (FLASH_WRPRTERR | FLASH_PGERR)) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR),
				FLASH_WRPRTERR | FLASH_PGERR);
	}
	return retval;
}

static int stm32x_check_operation_supported(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;

	/* if we have a dual flash bank device then
	 * we need to perform option byte stuff on bank0 only */
	if (stm32x_info->register_base != FLASH_REG_BASE_B0) {
		LOG_ERROR("Option Byte Operation's must use bank0");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int stm32x_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, STM32_FLASH_OBR_B0, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	stm32x_info->option_bytes.user_options = (optiondata >> stm32x_info->option_offset >> 2) & 0xffff;
	stm32x_info->option_bytes.user_data = (optiondata >> stm32x_info->user_data_offset) & 0xffff;
	stm32x_info->option_bytes.RDP = (optiondata & (1 << OPT_READOUT)) ? 0xFFFF : 0x5AA5;

	if (optiondata & (1 << OPT_READOUT))
		LOG_INFO("Device Security Bit Set");

	/* each bit refers to a 4bank protection */
	retval = target_read_u32(target, STM32_FLASH_WRPR_B0, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	stm32x_info->option_bytes.protection[0] = (uint16_t)optiondata;
	stm32x_info->option_bytes.protection[1] = (uint16_t)(optiondata >> 8);
	stm32x_info->option_bytes.protection[2] = (uint16_t)(optiondata >> 16);
	stm32x_info->option_bytes.protection[3] = (uint16_t)(optiondata >> 24);

	return ERROR_OK;
}

static int stm32x_erase_options(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* read current options */
	stm32x_read_options(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, STM32_FLASH_KEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_KEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, STM32_FLASH_OPTKEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, STM32_FLASH_OPTKEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* erase option bytes */
	retval = target_write_u32(target, STM32_FLASH_CR_B0, FLASH_OPTER | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, STM32_FLASH_CR_B0, FLASH_OPTER | FLASH_STRT | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	stm32x_info->option_bytes.RDP = stm32x_info->default_rdp;

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* unlock flash registers */
	int retval = target_write_u32(target, STM32_FLASH_KEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, STM32_FLASH_KEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, STM32_FLASH_OPTKEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, STM32_FLASH_OPTKEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* program option bytes */
	retval = target_write_u32(target, STM32_FLASH_CR_B0, FLASH_OPTPG | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;

	uint8_t opt_bytes[16];

	target_buffer_set_u16(target, opt_bytes, stm32x_info->option_bytes.RDP);
	target_buffer_set_u16(target, opt_bytes + 2, stm32x_info->option_bytes.user_options);
	target_buffer_set_u16(target, opt_bytes + 4, stm32x_info->option_bytes.user_data & 0xff);
	target_buffer_set_u16(target, opt_bytes + 6, (stm32x_info->option_bytes.user_data >> 8) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 8, stm32x_info->option_bytes.protection[0]);
	target_buffer_set_u16(target, opt_bytes + 10, stm32x_info->option_bytes.protection[1]);
	target_buffer_set_u16(target, opt_bytes + 12, stm32x_info->option_bytes.protection[2]);
	target_buffer_set_u16(target, opt_bytes + 14, stm32x_info->option_bytes.protection[3]);

	uint32_t offset = STM32_OB_RDP - bank->base;
	retval = stm32x_write_block(bank, opt_bytes, offset, sizeof(opt_bytes) / 2);
	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			LOG_ERROR("working area required to erase options bytes");
		return retval;
	}

	retval = target_write_u32(target, STM32_FLASH_CR_B0, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;

	uint32_t protection;
	int i, s;
	int num_bits;
	int set;

	int retval = stm32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	/* medium density - each bit refers to a 4bank protection
	 * high density - each bit refers to a 2bank protection */
	retval = target_read_u32(target, STM32_FLASH_WRPR_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	/* medium density - each protection bit is for 4 * 1K pages
	 * high density - each protection bit is for 2 * 2K pages */
	num_bits = (bank->num_sectors / stm32x_info->ppage_size);

	if (stm32x_info->ppage_size == 2) {
		/* high density flash/connectivity line protection */

		set = 1;

		if (protection & (1 << 31))
			set = 0;

		/* bit 31 controls sector 62 - 255 protection for high density
		 * bit 31 controls sector 62 - 127 protection for connectivity line */
		for (s = 62; s < bank->num_sectors; s++)
			bank->sectors[s].is_protected = set;

		if (bank->num_sectors > 61)
			num_bits = 31;

		for (i = 0; i < num_bits; i++) {
			set = 1;

			if (protection & (1 << i))
				set = 0;

			for (s = 0; s < stm32x_info->ppage_size; s++)
				bank->sectors[(i * stm32x_info->ppage_size) + s].is_protected = set;
		}
	} else {
		/* low/medium density flash protection */
		for (i = 0; i < num_bits; i++) {
			set = 1;

			if (protection & (1 << i))
				set = 0;

			for (s = 0; s < stm32x_info->ppage_size; s++)
				bank->sectors[(i * stm32x_info->ppage_size) + s].is_protected = set;
		}
	}

	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return stm32x_mass_erase(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	for (i = first; i <= last; i++) {
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_PER);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_AR),
				bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target,
				stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_PER | FLASH_STRT);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint16_t prot_reg[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	int i, reg, bit;
	int status;
	uint32_t protection;

	stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = stm32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if ((first % stm32x_info->ppage_size) != 0) {
		LOG_WARNING("aligned start protect sector to a %d sector boundary",
				stm32x_info->ppage_size);
		first = first - (first % stm32x_info->ppage_size);
	}
	if (((last + 1) % stm32x_info->ppage_size) != 0) {
		LOG_WARNING("aligned end protect sector to a %d sector boundary",
				stm32x_info->ppage_size);
		last++;
		last = last - (last % stm32x_info->ppage_size);
		last--;
	}

	/* medium density - each bit refers to a 4bank protection
	 * high density - each bit refers to a 2bank protection */
	retval = target_read_u32(target, STM32_FLASH_WRPR_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	prot_reg[0] = (uint16_t)protection;
	prot_reg[1] = (uint16_t)(protection >> 8);
	prot_reg[2] = (uint16_t)(protection >> 16);
	prot_reg[3] = (uint16_t)(protection >> 24);

	if (stm32x_info->ppage_size == 2) {
		/* high density flash */

		/* bit 7 controls sector 62 - 255 protection */
		if (last > 61) {
			if (set)
				prot_reg[3] &= ~(1 << 7);
			else
				prot_reg[3] |= (1 << 7);
		}

		if (first > 61)
			first = 62;
		if (last > 61)
			last = 61;

		for (i = first; i <= last; i++) {
			reg = (i / stm32x_info->ppage_size) / 8;
			bit = (i / stm32x_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	} else {
		/* medium density flash */
		for (i = first; i <= last; i++) {
			reg = (i / stm32x_info->ppage_size) / 8;
			bit = (i / stm32x_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	}

	status = stm32x_erase_options(bank);
	if (status != ERROR_OK)
		return status;

	stm32x_info->option_bytes.protection[0] = prot_reg[0];
	stm32x_info->option_bytes.protection[1] = prot_reg[1];
	stm32x_info->option_bytes.protection[2] = prot_reg[2];
	stm32x_info->option_bytes.protection[3] = prot_reg[3];

	return stm32x_write_options(bank);
}

static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* see contrib/loaders/flash/stm32f1x.S for src */

	static const uint8_t stm32x_flash_write_code[] = {
		/* #define STM32_FLASH_SR_OFFSET 0x0C */
		/* wait_fifo: */
			0x16, 0x68,   /* ldr   r6, [r2, #0] */
			0x00, 0x2e,   /* cmp   r6, #0 */
			0x18, 0xd0,   /* beq   exit */
			0x55, 0x68,   /* ldr   r5, [r2, #4] */
			0xb5, 0x42,   /* cmp   r5, r6 */
			0xf9, 0xd0,   /* beq   wait_fifo */
			0x2e, 0x88,   /* ldrh  r6, [r5, #0] */
			0x26, 0x80,   /* strh  r6, [r4, #0] */
			0x02, 0x35,   /* adds  r5, #2 */
			0x02, 0x34,   /* adds  r4, #2 */
		/* busy: */
			0xc6, 0x68,   /* ldr   r6, [r0, #STM32_FLASH_SR_OFFSET] */
			0x01, 0x27,   /* movs  r7, #1 */
			0x3e, 0x42,   /* tst   r6, r7 */
			0xfb, 0xd1,   /* bne   busy */
			0x14, 0x27,   /* movs  r7, #0x14 */
			0x3e, 0x42,   /* tst   r6, r7 */
			0x08, 0xd1,   /* bne   error */
			0x9d, 0x42,   /* cmp   r5, r3 */
			0x01, 0xd3,   /* bcc   no_wrap */
			0x15, 0x46,   /* mov   r5, r2 */
			0x08, 0x35,   /* adds  r5, #8 */
		/* no_wrap: */
			0x55, 0x60,   /* str   r5, [r2, #4] */
			0x01, 0x39,   /* subs  r1, r1, #1 */
			0x00, 0x29,   /* cmp   r1, #0 */
			0x02, 0xd0,   /* beq   exit */
			0xe5, 0xe7,   /* b     wait_fifo */
		/* error: */
			0x00, 0x20,   /* movs  r0, #0 */
			0x50, 0x60,   /* str   r0, [r2, #4] */
		/* exit: */
			0x30, 0x46,   /* mov   r0, r6 */
			0x00, 0xbe,   /* bkpt  #0 */
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32x_flash_write_code), stm32x_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	};

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, stm32x_info->register_base);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) & FLASH_PGERR) {
			LOG_ERROR("flash memory not erased before writing");
			/* Clear but report errors */
			target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR), FLASH_PGERR);
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) & FLASH_WRPRTERR) {
			LOG_ERROR("flash memory write protected");
			/* Clear but report errors */
			target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR), FLASH_WRPRTERR);
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int stm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint8_t *new_buffer = NULL;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of bytes, the data has to be padded. Duplicate
	 * the buffer and use the normal code path with a single block write since
	 * it's probably cheaper than to special case the last odd write using
	 * discrete accesses. */
	if (count & 1) {
		new_buffer = malloc(count + 1);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write, padding with 0xff");
		buffer = memcpy(new_buffer, buffer, count);
		new_buffer[count++] = 0xff;
	}

	uint32_t words_remaining = count / 2;
	int retval, retval2;

	/* unlock flash registers */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		goto cleanup;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_PG);
	if (retval != ERROR_OK)
		goto cleanup;

	/* try using a block write */
	retval = stm32x_write_block(bank, buffer, offset, words_remaining);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (words_remaining > 0) {
			uint16_t value;
			memcpy(&value, buffer, sizeof(uint16_t));

			retval = target_write_u16(target, bank->base + offset, value);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			retval = stm32x_wait_status_busy(bank, 5);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			words_remaining--;
			buffer += 2;
			offset += 2;
		}
	}

reset_pg_and_lock:
	retval2 = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
	if (retval == ERROR_OK)
		retval = retval2;

cleanup:
	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int stm32x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	/* This check the device CPUID core register to detect
	 * the M0 from the M3 devices. */

	struct target *target = bank->target;
	uint32_t cpuid, device_id_register = 0;

	/* Get the CPUID from the ARM Core
	 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0432c/DDI0432C_cortex_m0_r0p0_trm.pdf 4.2.1 */
	int retval = target_read_u32(target, 0xE000ED00, &cpuid);
	if (retval != ERROR_OK)
		return retval;

	if (((cpuid >> 4) & 0xFFF) == 0xC20) {
		/* 0xC20 is M0 devices */
		device_id_register = 0x40015800;
	} else if (((cpuid >> 4) & 0xFFF) == 0xC23) {
		/* 0xC23 is M3 devices */
		device_id_register = 0xE0042000;
	} else if (((cpuid >> 4) & 0xFFF) == 0xC24) {
		/* 0xC24 is M4 devices */
		device_id_register = 0xE0042000;
	} else {
		LOG_ERROR("Cannot identify target as a stm32x");
		return ERROR_FAIL;
	}

	/* read stm32 device id register */
	retval = target_read_u32(target, device_id_register, device_id);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32x_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
	struct target *target = bank->target;
	uint32_t cpuid, flash_size_reg;

	int retval = target_read_u32(target, 0xE000ED00, &cpuid);
	if (retval != ERROR_OK)
		return retval;

	if (((cpuid >> 4) & 0xFFF) == 0xC20) {
		/* 0xC20 is M0 devices */
		flash_size_reg = 0x1FFFF7CC;
	} else if (((cpuid >> 4) & 0xFFF) == 0xC23) {
		/* 0xC23 is M3 devices */
		flash_size_reg = 0x1FFFF7E0;
	} else if (((cpuid >> 4) & 0xFFF) == 0xC24) {
		/* 0xC24 is M4 devices */
		flash_size_reg = 0x1FFFF7CC;
	} else {
		LOG_ERROR("Cannot identify target as a stm32x");
		return ERROR_FAIL;
	}

	retval = target_read_u16(target, flash_size_reg, flash_size_in_kb);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32x_probe(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	int page_size;
	uint32_t base_address = 0x08000000;

	stm32x_info->probed = 0;
	stm32x_info->register_base = FLASH_REG_BASE_B0;
	stm32x_info->user_data_offset = 10;
	stm32x_info->option_offset = 0;

	/* default factory protection level */
	stm32x_info->default_rdp = 0x5AA5;

	/* read stm32 device id register */
	int retval = stm32x_get_device_id(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	/* set page size, protection granularity and max flash size depending on family */
	switch (device_id & 0xfff) {
	case 0x410: /* medium density */
		page_size = 1024;
		stm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 128;
		break;
	case 0x412: /* low density */
		page_size = 1024;
		stm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 32;
		break;
	case 0x414: /* high density */
		page_size = 2048;
		stm32x_info->ppage_size = 2;
		max_flash_size_in_kb = 512;
		break;
	case 0x418: /* connectivity line density */
		page_size = 2048;
		stm32x_info->ppage_size = 2;
		max_flash_size_in_kb = 256;
		break;
	case 0x420: /* value line density */
		page_size = 1024;
		stm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 128;
		break;
	case 0x422: /* stm32f30x */
		page_size = 2048;
		stm32x_info->ppage_size = 2;
		max_flash_size_in_kb = 256;
		stm32x_info->user_data_offset = 16;
		stm32x_info->option_offset = 6;
		stm32x_info->default_rdp = 0x55AA;
		break;
	case 0x428: /* value line High density */
		page_size = 2048;
		stm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 128;
		break;
	case 0x430: /* xl line density (dual flash banks) */
		page_size = 2048;
		stm32x_info->ppage_size = 2;
		max_flash_size_in_kb = 1024;
		stm32x_info->has_dual_banks = true;
		break;
	case 0x432: /* stm32f37x */
		page_size = 2048;
		stm32x_info->ppage_size = 2;
		max_flash_size_in_kb = 256;
		stm32x_info->user_data_offset = 16;
		stm32x_info->option_offset = 6;
		stm32x_info->default_rdp = 0x55AA;
		break;
	case 0x440: /* stm32f05x */
	case 0x444: /* stm32f03x */
		page_size = 1024;
		stm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 64;
		stm32x_info->user_data_offset = 16;
		stm32x_info->option_offset = 6;
		stm32x_info->default_rdp = 0x55AA;
		break;
	case 0x448: /* stm32f07x */
		page_size = 2048;
		stm32x_info->ppage_size = 4;
		max_flash_size_in_kb = 128;
		stm32x_info->user_data_offset = 16;
		stm32x_info->option_offset = 6;
		stm32x_info->default_rdp = 0x55AA;
		break;
	default:
		LOG_WARNING("Cannot identify target as a STM32 family.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = stm32x_get_flash_size(bank, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	if (stm32x_info->has_dual_banks) {
		/* split reported size into matching bank */
		if (bank->base != 0x08080000) {
			/* bank 0 will be fixed 512k */
			flash_size_in_kb = 512;
		} else {
			flash_size_in_kb -= 512;
			/* bank1 also uses a register offset */
			stm32x_info->register_base = FLASH_REG_BASE_B1;
			base_address = 0x08080000;
		}
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (stm32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = stm32x_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32x_info->probed = 1;

	return ERROR_OK;
}

static int stm32x_auto_probe(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	if (stm32x_info->probed)
		return ERROR_OK;
	return stm32x_probe(bank);
}

#if 0
COMMAND_HANDLER(stm32x_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int get_stm32x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint32_t dbgmcu_idcode;

		/* read stm32 device id register */
	int retval = stm32x_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint16_t rev_id = dbgmcu_idcode >> 16;
	const char *device_str;
	const char *rev_str = NULL;

	switch (device_id) {
	case 0x410:
		device_str = "STM32F10x (Medium Density)";

		switch (rev_id) {
		case 0x0000:
			rev_str = "A";
			break;

		case 0x2000:
			rev_str = "B";
			break;

		case 0x2001:
			rev_str = "Z";
			break;

		case 0x2003:
			rev_str = "Y";
			break;
		}
		break;

	case 0x412:
		device_str = "STM32F10x (Low Density)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;
		}
		break;

	case 0x414:
		device_str = "STM32F10x (High Density)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;

		case 0x1003:
			rev_str = "Y";
			break;
		}
		break;

	case 0x418:
		device_str = "STM32F10x (Connectivity)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;
		}
		break;

	case 0x420:
		device_str = "STM32F100 (Low/Medium Density)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;
		}
		break;

	case 0x422:
		device_str = "STM32F30x";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;

		case 0x1003:
			rev_str = "Y";
			break;

		case 0x2000:
			rev_str = "B";
			break;
		}
		break;

	case 0x428:
		device_str = "STM32F100 (High Density)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "Z";
			break;
		}
		break;

	case 0x430:
		device_str = "STM32F10x (XL Density)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;
		}
		break;

	case 0x432:
		device_str = "STM32F37x";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x2000:
			rev_str = "B";
			break;
		}
		break;

	case 0x444:
		device_str = "STM32F03x";

		switch (rev_id) {
		case 0x1000:
			rev_str = "1.0";
			break;

		case 0x2000:
			rev_str = "2.0";
			break;
		}
		break;

	case 0x440:
		device_str = "STM32F05x";

		switch (rev_id) {
		case 0x1000:
			rev_str = "1.0";
			break;

		case 0x2000:
			rev_str = "2.0";
			break;
		}
		break;

	case 0x448:
		device_str = "STM32F07x";

		switch (rev_id) {
		case 0x1000:
			rev_str = "1.0";
			break;

		case 0x2000:
			rev_str = "2.0";
			break;
		}
		break;

	default:
		snprintf(buf, buf_size, "Cannot identify target as a STM32F0/1/3\n");
		return ERROR_FAIL;
	}

	if (rev_str != NULL)
		snprintf(buf, buf_size, "%s - Rev: %s", device_str, rev_str);
	else
		snprintf(buf, buf_size, "%s - Rev: unknown (0x%04x)", device_str, rev_id);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if (stm32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	stm32x_info->option_bytes.RDP = 0;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x locked");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if (stm32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to unlock device");
		return ERROR_OK;
	}

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_options_read_command)
{
	uint32_t optionbyte;
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_OBR_B0, &optionbyte);
	if (retval != ERROR_OK)
		return retval;
	command_print(CMD_CTX, "Option Byte: 0x%" PRIx32 "", optionbyte);

	int user_data = optionbyte;

	if (optionbyte >> OPT_ERROR & 1)
		command_print(CMD_CTX, "Option Byte Complement Error");

	if (optionbyte >> OPT_READOUT & 1)
		command_print(CMD_CTX, "Readout Protection On");
	else
		command_print(CMD_CTX, "Readout Protection Off");

	/* user option bytes are offset depending on variant */
	optionbyte >>= stm32x_info->option_offset;

	if (optionbyte >> OPT_RDWDGSW & 1)
		command_print(CMD_CTX, "Software Watchdog");
	else
		command_print(CMD_CTX, "Hardware Watchdog");

	if (optionbyte >> OPT_RDRSTSTOP & 1)
		command_print(CMD_CTX, "Stop: No reset generated");
	else
		command_print(CMD_CTX, "Stop: Reset generated");

	if (optionbyte >> OPT_RDRSTSTDBY & 1)
		command_print(CMD_CTX, "Standby: No reset generated");
	else
		command_print(CMD_CTX, "Standby: Reset generated");

	if (stm32x_info->has_dual_banks) {
		if (optionbyte >> OPT_BFB2 & 1)
			command_print(CMD_CTX, "Boot: Bank 0");
		else
			command_print(CMD_CTX, "Boot: Bank 1");
	}

	command_print(CMD_CTX, "User Option0: 0x%02" PRIx8,
			(uint8_t)((user_data >> stm32x_info->user_data_offset) & 0xff));
	command_print(CMD_CTX, "User Option1: 0x%02" PRIx8,
			(uint8_t)((user_data >> (stm32x_info->user_data_offset + 8)) & 0xff));

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_options_write_command)
{
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;
	uint16_t optionbyte;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_read_options(bank);
	if (ERROR_OK != retval)
		return retval;

	/* start with current options */
	optionbyte = stm32x_info->option_bytes.user_options;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	while (CMD_ARGC) {
		if (strcmp("SWWDG", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 0);
		else if (strcmp("HWWDG", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 0);
		else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 1);
		else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 1);
		else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 2);
		else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 2);
		else if (stm32x_info->has_dual_banks) {
			if (strcmp("BOOT0", CMD_ARGV[0]) == 0)
				optionbyte |= (1 << 3);
			else if (strcmp("BOOT1", CMD_ARGV[0]) == 0)
				optionbyte &= ~(1 << 3);
			else
				return ERROR_COMMAND_SYNTAX_ERROR;
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}

	if (stm32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to erase options");
		return ERROR_OK;
	}

	stm32x_info->option_bytes.user_options = optionbyte;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to write options");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x write options complete.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.");

	return ERROR_OK;
}

static int stm32x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock option flash registers */
	int retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_MER);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR),
			FLASH_MER | FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "stm32x mass erase complete");
	} else
		command_print(CMD_CTX, "stm32x mass erase failed");

	return retval;
}

static const struct command_registration stm32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = stm32x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option byte.",
	},
	{
		.name = "options_write",
		.handler = stm32x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP')",
		.help = "Replace bits in device option byte.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32f1x",
		.mode = COMMAND_ANY,
		.help = "stm32f1x flash command group",
		.usage = "",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32f1x_flash = {
	.name = "stm32f1x",
	.commands = stm32x_command_handlers,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.read = default_flash_read,
	.probe = stm32x_probe,
	.auto_probe = stm32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32x_protect_check,
	.info = get_stm32x_info,
};

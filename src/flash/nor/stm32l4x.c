/***************************************************************************
 *   Copyright (C) 2015 by Uwe Bonnes                                      *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* STM32L4xxx series for reference.
 *
 * RM0351 (STM32L4x5/STM32L4x6)
 * http://www.st.com/resource/en/reference_manual/dm00083560.pdf
 *
 * RM0394 (STM32L43x/44x/45x/46x)
 * http://www.st.com/resource/en/reference_manual/dm00151940.pdf
 *
 * STM32L476RG Datasheet (for erase timing)
 * http://www.st.com/resource/en/datasheet/stm32l476rg.pdf
 *
 * The RM0351 devices have normally two banks, but on 512 and 256 kiB devices
 * an option byte is available to map all sectors to the first bank.
 * Both STM32 banks are treated as one OpenOCD bank, as other STM32 devices
 * handlers do!
 *
 * RM0394 devices have a single bank only.
 *
 */

/* Erase time can be as high as 25ms, 10x this and assume it's toast... */

#define FLASH_ERASE_TIMEOUT 250

#define STM32_FLASH_BASE    0x40022000
#define STM32_FLASH_ACR     0x40022000
#define STM32_FLASH_KEYR    0x40022008
#define STM32_FLASH_OPTKEYR 0x4002200c
#define STM32_FLASH_SR      0x40022010
#define STM32_FLASH_CR      0x40022014
#define STM32_FLASH_OPTR    0x40022020
#define STM32_FLASH_WRP1AR  0x4002202c
#define STM32_FLASH_WRP2AR  0x40022030
#define STM32_FLASH_WRP1BR  0x4002204c
#define STM32_FLASH_WRP2BR  0x40022050

/* FLASH_CR register bits */

#define FLASH_PG       (1 << 0)
#define FLASH_PER      (1 << 1)
#define FLASH_MER1     (1 << 2)
#define FLASH_PAGE_SHIFT     3
#define FLASH_CR_BKER  (1 << 11)
#define FLASH_MER2     (1 << 15)
#define FLASH_STRT     (1 << 16)
#define FLASH_EOPIE    (1 << 24)
#define FLASH_ERRIE    (1 << 25)
#define FLASH_OPTLOCK  (1 << 30)
#define FLASH_LOCK     (1 << 31)

/* FLASH_SR register bits */

#define FLASH_BSY      (1 << 16)
/* Fast programming not used => related errors not used*/
#define FLASH_PGSERR   (1 << 7) /* Programming sequence error */
#define FLASH_SIZERR   (1 << 6) /* Size  error */
#define FLASH_PGAERR   (1 << 5) /* Programming alignment error */
#define FLASH_WRPERR   (1 << 4) /* Write protection error */
#define FLASH_PROGERR  (1 << 3) /* Programming error */
#define FLASH_OPERR    (1 << 1) /* Operation error */
#define FLASH_EOP      (1 << 0) /* End of operation */

#define FLASH_ERROR (FLASH_PGSERR | FLASH_PGSERR | FLASH_PGAERR | FLASH_WRPERR | FLASH_OPERR)

/* STM32_FLASH_OBR bit definitions (reading) */

#define OPT_DUALBANK   21	/* dual flash bank only */

/* register unlock keys */

#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F


/* other registers */
#define DBGMCU_IDCODE	0xE0042000
#define FLASH_SIZE_REG	0x1FFF75E0

struct stm32l4_options {
	uint8_t RDP;
	uint16_t bank_b_start;
	uint8_t user_options;
	uint8_t wpr1a_start;
	uint8_t wpr1a_end;
	uint8_t wpr1b_start;
	uint8_t wpr1b_end;
	uint8_t wpr2a_start;
	uint8_t wpr2a_end;
	uint8_t wpr2b_start;
	uint8_t wpr2b_end;
    /* Fixme: Handle PCROP */
};

struct stm32l4_flash_bank {
	struct stm32l4_options option_bytes;
	int probed;
};

/* flash bank stm32l4x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32l4_flash_bank_command)
{
	struct stm32l4_flash_bank *stm32l4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32l4_info = malloc(sizeof(struct stm32l4_flash_bank));
	if (!stm32l4_info)
		return ERROR_FAIL; /* Checkme: What better error to use?*/
	bank->driver_priv = stm32l4_info;

	stm32l4_info->probed = 0;

	return ERROR_OK;
}

static inline int stm32l4_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	return reg;
}

static inline int stm32l4_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(
		target, stm32l4_get_flash_reg(bank, STM32_FLASH_SR), status);
}

static int stm32l4_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32l4_get_flash_status(bank, &status);
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


	if (status & FLASH_WRPERR) {
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, stm32l4_get_flash_reg(bank, STM32_FLASH_SR),
				status & FLASH_ERROR);
	}
	return retval;
}

static int stm32l4_unlock_reg(struct target *target)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on STM32_FLASH_KEYR will fail
	 */
	int retval = target_read_u32(target, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_LOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l4_unlock_option_reg(struct target *target)
{
	uint32_t ctrl;

	int retval = target_read_u32(target, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_OPTLOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, STM32_FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_OPTLOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l4_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct stm32l4_flash_bank *stm32l4_info = NULL;
	struct target *target = bank->target;

	stm32l4_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, STM32_FLASH_OPTR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	stm32l4_info->option_bytes.user_options = (optiondata >> 8) & 0x3ffff;
	stm32l4_info->option_bytes.RDP = optiondata & 0xff;

	retval = target_read_u32(target, STM32_FLASH_WRP1AR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32l4_info->option_bytes.wpr1a_start =  optiondata         & 0xff;
	stm32l4_info->option_bytes.wpr1a_end   = (optiondata >> 16)  & 0xff;

	retval = target_read_u32(target, STM32_FLASH_WRP2AR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32l4_info->option_bytes.wpr2a_start =  optiondata         & 0xff;
	stm32l4_info->option_bytes.wpr2a_end   = (optiondata >> 16)  & 0xff;

	retval = target_read_u32(target, STM32_FLASH_WRP1BR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32l4_info->option_bytes.wpr1b_start =  optiondata         & 0xff;
	stm32l4_info->option_bytes.wpr1b_end   = (optiondata >> 16)  & 0xff;

	retval = target_read_u32(target, STM32_FLASH_WRP2BR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32l4_info->option_bytes.wpr2b_start =  optiondata         & 0xff;
	stm32l4_info->option_bytes.wpr2b_end   = (optiondata >> 16)  & 0xff;

	if (stm32l4_info->option_bytes.RDP != 0xAA)
		LOG_INFO("Device Security Bit Set");

	return ERROR_OK;
}

static int stm32l4_write_options(struct flash_bank *bank)
{
	struct stm32l4_flash_bank *stm32l4_info = NULL;
	struct target *target = bank->target;
	uint32_t optiondata;

	stm32l4_info = bank->driver_priv;

	(void) optiondata;
	(void) stm32l4_info;

	int retval = stm32l4_unlock_option_reg(target);
	if (retval != ERROR_OK)
		return retval;
	/* FIXME: Implement Option writing!*/
	return ERROR_OK;
}

static int stm32l4_protect_check(struct flash_bank *bank)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	/* read write protection settings */
	int retval = stm32l4_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = 0; i < bank->num_sectors; i++) {
		if (i < stm32l4_info->option_bytes.bank_b_start) {
			if (((i >= stm32l4_info->option_bytes.wpr1a_start) &&
				 (i <= stm32l4_info->option_bytes.wpr1a_end)) ||
				((i >= stm32l4_info->option_bytes.wpr2a_start) &&
				 (i <= stm32l4_info->option_bytes.wpr2a_end)))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		} else {
			uint8_t snb;
			snb = i - stm32l4_info->option_bytes.bank_b_start + 256;
			if (((snb >= stm32l4_info->option_bytes.wpr1b_start) &&
				 (snb <= stm32l4_info->option_bytes.wpr1b_end)) ||
				((snb >= stm32l4_info->option_bytes.wpr2b_start) &&
				 (snb <= stm32l4_info->option_bytes.wpr2b_end)))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		}
	}
	return ERROR_OK;
}

static int stm32l4_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = stm32l4_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by
       checking the BSY bit in the FLASH_SR register
	2. Set the PER bit and select the page and bank
	   you wish to erase  in the FLASH_CR register
	3. Set the STRT bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	 */
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	for (i = first; i <= last; i++) {
		uint32_t erase_flags;
		erase_flags = FLASH_PER | FLASH_STRT;

		if  (i >= stm32l4_info->option_bytes.bank_b_start) {
			uint8_t snb;
			snb = (i - stm32l4_info->option_bytes.bank_b_start) + 256;
			erase_flags |= snb << FLASH_PAGE_SHIFT | FLASH_CR_BKER;
		} else
			erase_flags |= i << FLASH_PAGE_SHIFT;
		retval = target_write_u32(target,
				stm32l4_get_flash_reg(bank, STM32_FLASH_CR), erase_flags);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(
		target, stm32l4_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32l4_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read protection settings */
	int retval = stm32l4_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	(void)stm32l4_info;
	/* FIXME: Write First and last in a valid WRPxx_start/end combo*/
	retval = stm32l4_write_options(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/* Count is in halfwords */
static int stm32l4_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* See contrib/loaders/flash/stm32l4x.S for source and
	 * hints how to generate the data!
	 */

	static const uint8_t stm32l4_flash_write_code[] = {
		0xd0, 0xf8, 0x00, 0x80, 0xb8, 0xf1, 0x00, 0x0f, 0x21, 0xd0, 0x45, 0x68,
		0xb8, 0xeb, 0x05, 0x06, 0x44, 0xbf, 0x76, 0x18, 0x36, 0x1a, 0x08, 0x2e,
		0xf2, 0xd3, 0xdf, 0xf8, 0x36, 0x60, 0x66, 0x61, 0xf5, 0xe8, 0x02, 0x67,
		0xe2, 0xe8, 0x02, 0x67, 0xbf, 0xf3, 0x4f, 0x8f, 0x26, 0x69, 0x16, 0xf4,
		0x80, 0x3f, 0xfb, 0xd1, 0x16, 0xf0, 0xfa, 0x0f, 0x07, 0xd1, 0x8d, 0x42,
		0x28, 0xbf, 0x00, 0xf1, 0x08, 0x05, 0x45, 0x60, 0x01, 0x3b, 0x13, 0xb1,
		0xda, 0xe7, 0x00, 0x21, 0x41, 0x60, 0x30, 0x46, 0x00, 0xbe, 0x01, 0x00,
		0x00, 0x00
	};

	if (target_alloc_working_area(target, sizeof(stm32l4_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32l4_flash_write_code),
			stm32l4_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) !=
		   ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* count (double word-64bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);	/* flash base */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count / 4);
	buf_set_u32(reg_params[4].value, 0, 32, STM32_FLASH_BASE);

	retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32l4 flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR;

		if (error & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_SR, error);
			retval = ERROR_FAIL;
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

static int stm32l4_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x7) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 8-byte alignment",
					offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x7) {
		LOG_WARNING("Padding %d bytes to keep 8-byte write size",
					count & 7);
		count = (count + 7) & ~7;
		/* This pads the write chunk with random bytes by overrunning the
		 * write buffer. Padding with the erased pattern 0xff is purely
		 * cosmetical, as 8-byte flash words are ECC secured and the first
		 * write will program the ECC bits. A second write would need
		 * to reprogramm these ECC bits.
		 * But this can only be done after erase!
		 */
	}

	retval = stm32l4_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* Only full double words (8-byte) can be programmed*/
	retval = stm32l4_write_block(bank, buffer, offset, count / 2);
	if (retval != ERROR_OK) {
		LOG_WARNING("block write failed");
		return retval;
		}

	LOG_WARNING("block write succeeded");
	return target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);
}

static int stm32l4_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb = 0xffff;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	uint32_t options;
	uint32_t base_address = 0x08000000;

	stm32l4_info->probed = 0;

	/* read stm32 device id register */
	int retval = target_read_u32(target, DBGMCU_IDCODE, &device_id);
	if (retval != ERROR_OK)
		return retval;
	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	/* set max flash size depending on family */
	switch (device_id & 0xfff) {
	case 0x461:
	case 0x415:
		max_flash_size_in_kb = 1024;
		break;
	case 0x462:
		max_flash_size_in_kb = 512;
		break;
	case 0x435:
		max_flash_size_in_kb = 256;
		break;
	default:
		LOG_WARNING("Cannot identify target as a STM32L4 family.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = target_read_u16(target, FLASH_SIZE_REG, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* get options to for DUAL BANK. */
	retval = target_read_u32(target, STM32_FLASH_OPTR, &options);

	/* only devices with < 1024 kiB may be set to single bank dual banks */
	if ((flash_size_in_kb == 1024) || !(options & OPT_DUALBANK))
		stm32l4_info->option_bytes.bank_b_start = 256;
	else
		stm32l4_info->option_bytes.bank_b_start = flash_size_in_kb << 9;

	/* did we assign flash size? */
	assert((flash_size_in_kb != 0xffff) && flash_size_in_kb);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb / 2;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = num_pages * (1 << 11);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	if (!bank->sectors)
		return ERROR_FAIL; /* Checkme: What better error to use?*/

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i << 11;
		bank->sectors[i].size = 1 << 11;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32l4_info->probed = 1;

	return ERROR_OK;
}

static int stm32l4_auto_probe(struct flash_bank *bank)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	if (stm32l4_info->probed)
		return ERROR_OK;
	return stm32l4_probe(bank);
}

static int get_stm32l4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	uint32_t dbgmcu_idcode;

	/* read stm32 device id register */
	int retval = target_read_u32(target, DBGMCU_IDCODE, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint8_t rev_id = dbgmcu_idcode >> 28;
	uint8_t rev_minor = 0;
	int i;

	for (i = 16; i < 28; i++) {
		if (dbgmcu_idcode & (1 << i))
			rev_minor++;
		else
			break;
	}

	const char *device_str;

	switch (device_id) {
	case 0x461:
		device_str = "STM32L496/4A6";
		break;

	case 0x415:
		device_str = "STM32L475/476/486";
		break;

	case 0x462:
		device_str = "STM32L45x/46x";
		break;

	case 0x435:
		device_str = "STM32L43x/44x";
		break;

	default:
		snprintf(buf, buf_size, "Cannot identify target as a STM32L4\n");
		return ERROR_FAIL;
	}

	snprintf(buf, buf_size, "%s - Rev: %1d.%02d",
			 device_str, rev_id, rev_minor);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l4_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32l4_flash_bank *stm32l4_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32l4_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32l4_read_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to read options",
					  bank->driver->name);
		return ERROR_OK;
	}

	/* set readout protection */
	stm32l4_info->option_bytes.RDP = 0;

	if (stm32l4_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to lock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD_CTX, "%s locked", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l4_handle_unlock_command)
{
	struct target *target = NULL;
	struct stm32l4_flash_bank *stm32l4_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32l4_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32l4_read_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	stm32l4_info->option_bytes.RDP = 0xAA;

	if (stm32l4_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to unlock device",
					  bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD_CTX, "%s unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.", bank->driver->name);

	return ERROR_OK;
}

static int stm32l4_mass_erase(struct flash_bank *bank, uint32_t action)
{
	int retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32l4_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(
		target, stm32l4_get_flash_reg(bank, STM32_FLASH_CR), action);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(
		target, stm32l4_get_flash_reg(bank, STM32_FLASH_CR),
		action | FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_wait_status_busy(bank,  FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(
		target, stm32l4_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l4_handle_mass_erase_command)
{
	int i;
	uint32_t action;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "stm32x mass_erase <STM32L4 bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	action =  FLASH_MER1 |  FLASH_MER2;
	retval = stm32l4_mass_erase(bank, action);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "stm32x mass erase complete");
	} else {
		command_print(CMD_CTX, "stm32x mass erase failed");
	}

	return retval;
}

static const struct command_registration stm32l4_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32l4_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32l4_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32l4_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32l4_command_handlers[] = {
	{
		.name = "stm32l4x",
		.mode = COMMAND_ANY,
		.help = "stm32l4x flash command group",
		.usage = "",
		.chain = stm32l4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32l4x_flash = {
	.name = "stm32l4x",
	.commands = stm32l4_command_handlers,
	.flash_bank_command = stm32l4_flash_bank_command,
	.erase = stm32l4_erase,
	.protect = stm32l4_protect,
	.write = stm32l4_write,
	.read = default_flash_read,
	.probe = stm32l4_probe,
	.auto_probe = stm32l4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32l4_protect_check,
	.info = get_stm32l4_info,
};

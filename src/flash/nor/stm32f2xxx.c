/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
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

/* Regarding performance:
 *
 * Short story - it might be best to leave the performance at
 * current levels.
 *
 * You may see a jump in speed if you change to using
 * 32bit words for the block programming.
 *
 * Its a shame you cannot use the double word as its
 * even faster - but you require external VPP for that mode.
 *
 * Having said all that 16bit writes give us the widest vdd
 * operating range, so may be worth adding a note to that effect.
 *
 */


/* Danger!!!! The STM32F1xxxx and STM32F2xxxx series actually have
 * quite different flash controllers.
 *
 * What's more scary is that the names of the registers and their
 * addresses are the same, but the actual bits and what they do are
 * can be very different.
 *
 * To reduce testing complexity and dangers of regressions,
 * a seperate file is used for stm32fx2222.
 *
 * 1mByte part with 4 x 16, 1 x 64, 7 x 128kBytes sectors
 *
 * What's the protection page size???
 *
 * Tested with STM3220F-EVAL board.
 *
 * STM32F21xx series for reference.
 *
 * RM0033
 * http://www.st.com/internet/mcu/product/250192.jsp
 *
 * PM0059
 * www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/PROGRAMMING_MANUAL/CD00233952.pdf
 *
 * STM32F1xxx series - notice that this code was copy, pasted and knocked
 * into a stm32f2xxx driver, so in case something has been converted or
 * bugs haven't been fixed, here are the original manuals:
 *
 * RM0008 - Reference manual
 *
 * RM0042, the Flash programming manual for low-, medium- high-density and
 * connectivity line STM32F10xxx devices
 *
 * PM0068, the Flash programming manual for XL-density STM32F10xxx devices.
 *
 */

 // Erase time can be as high as 1000ms, 10x this and it's toast...
#define FLASH_ERASE_TIMEOUT 10000
#define FLASH_WRITE_TIMEOUT 5


#define STM32_FLASH_BASE	0x40023c00
#define STM32_FLASH_ACR		0x40023c00
#define STM32_FLASH_KEYR	0x40023c04
#define STM32_FLASH_OPTKEYR	0x40023c08
#define STM32_FLASH_SR		0x40023c0C
#define STM32_FLASH_CR		0x40023c10
#define STM32_FLASH_OPTCR	0x40023c14
#define STM32_FLASH_OBR		0x40023c1C



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
#define FLASH_SER		(1 << 1)
#define FLASH_MER		(1 << 2)
#define FLASH_STRT		(1 << 16)
#define FLASH_PSIZE_8	(0 << 8)
#define FLASH_PSIZE_16	(1 << 8)
#define FLASH_PSIZE_32	(2 << 8)
#define FLASH_PSIZE_64	(3 << 8)
#define FLASH_SNB(a) 	((a) << 3)
#define FLASH_LOCK		(1 << 31)

/* FLASH_SR register bits */

#define FLASH_BSY		(1 << 16)
#define FLASH_PGSERR	(1 << 7) // Programming sequence error
#define FLASH_PGPERR	(1 << 6) // Programming parallelism error
#define FLASH_PGAERR	(1 << 5) // Programming alignment error
#define FLASH_WRPERR	(1 << 4) // Write protection error
#define FLASH_OPERR		(1 << 1) // Operation error

#define FLASH_ERROR (FLASH_PGSERR | FLASH_PGPERR | FLASH_PGAERR| FLASH_WRPERR| FLASH_OPERR)

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

struct stm32x_flash_bank
{
	struct working_area *write_algorithm;
	int probed;
};


/* flash bank stm32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
	{
		LOG_WARNING("incomplete flash_bank stm32x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	stm32x_info = malloc(sizeof(struct stm32x_flash_bank));
	bank->driver_priv = stm32x_info;

	stm32x_info->write_algorithm = NULL;
	stm32x_info->probed = 0;

	return ERROR_OK;
}

static inline int stm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	return reg;
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
	for (;;)
	{
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0)
		{
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}


	if (status & FLASH_WRPERR)
	{
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR)
	{
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR),
				status & FLASH_ERROR);
	}
	return retval;
}

static int stm32x_unlock_reg(struct target *target)
{
	/* unlock flash registers */
	int retval = target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = stm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register
	2. Set the SER bit and select the sector (out of the 12 sectors in the main memory block)
	  you wish to erase (SNB) in the FLASH_CR register
	3. Set the STRT bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	 */

	for (i = first; i <= last; i++)
	{
		retval = target_write_u32(target,
				stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_SER | FLASH_SNB(i) | FLASH_STRT);
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
	return ERROR_OK;
}

static int stm32x_write_block(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* see contib/loaders/flash/stm32x.s for src */

	static const uint16_t stm32x_flash_write_code_16[] = {
//	00000000 <write>:
	   0x4b07, //     	ldr	r3, [pc, #28]	(20 <STM32_PROG16>)
	   0x6123,  //    	str	r3, [r4, #16]
       0xf830, 0x3b02, 	//ldrh.w	r3, [r0], #2
       0xf821, 0x3b02, 	//strh.w	r3, [r1], #2

	//0000000c <busy>:
	0x68e3,      	//ldr	r3, [r4, #12]
0xf413, 0x3f80, // 	tst.w	r3, #65536	; 0x10000
0xd0fb,      	//beq.n	c <busy>
0xf013, 0x0ff0, 	//tst.w	r3, #240	; 0xf0
0xd101,      	//bne.n	1e <exit>
0x3a01,      	//subs	r2, #1
0xd1f0,      	//bne.n	0 <write>
	            	   	//0000001e <exit>:
	0xbe00, //      	bkpt	0x0000

	//00000020 <STM32_PROG16>:
	0x0101, 0x0000, // 	.word	0x00000101

	};

	// Flip endian
	uint8_t stm32x_flash_write_code[sizeof(stm32x_flash_write_code_16)*2];
	for (unsigned i = 0; i < sizeof(stm32x_flash_write_code_16) / 2; i++)
	{
		stm32x_flash_write_code[i*2 + 0] = stm32x_flash_write_code_16[i] & 0xff;
		stm32x_flash_write_code[i*2 + 1] = (stm32x_flash_write_code_16[i] >> 8) & 0xff;
	}

	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code),
			&stm32x_info->write_algorithm) != ERROR_OK)
	{
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	if ((retval = target_write_buffer(target, stm32x_info->write_algorithm->address,
			sizeof(stm32x_flash_write_code),
			(uint8_t*)stm32x_flash_write_code)) != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			if (stm32x_info->write_algorithm)
				target_free_working_area(target, stm32x_info->write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	};

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);

	while (count > 0)
	{
		uint32_t thisrun_count = (count > (buffer_size / 2)) ?
				(buffer_size / 2) : count;

		if ((retval = target_write_buffer(target, source->address,
				thisrun_count * 2, buffer)) != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);
		// R3 is a return value only
		buf_set_u32(reg_params[4].value, 0, 32, STM32_FLASH_BASE);

		if ((retval = target_run_algorithm(target, 0, NULL,
				sizeof(reg_params) / sizeof(*reg_params),
				reg_params,
				stm32x_info->write_algorithm->address,
				0,
				10000, &armv7m_info)) != ERROR_OK)
		{
			LOG_ERROR("error executing stm32x flash write algorithm");
			break;
		}

		uint32_t error = buf_get_u32(reg_params[3].value, 0, 32) & FLASH_ERROR;

		if (error & FLASH_WRPERR)
		{
			LOG_ERROR("flash memory write protected");
		}

		if (error != 0)
		{
			LOG_ERROR("flash write failed = %08x", error);
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_SR, error);
			retval = ERROR_FAIL;
			break;
		}

		buffer += thisrun_count * 2;
		address += thisrun_count * 2;
		count -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, stm32x_info->write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int stm32x_write(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t words_remaining = (count / 2);
	uint32_t bytes_remaining = (count & 0x00000001);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1)
	{
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = stm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* multiple half words (2-byte) to be programmed? */
	if (words_remaining > 0)
	{
		/* try using a block write */
		if ((retval = stm32x_write_block(bank, buffer, offset, words_remaining)) != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
		}
		else
		{
			buffer += words_remaining * 2;
			address += words_remaining * 2;
			words_remaining = 0;
		}
	}

	if ((retval != ERROR_OK) && (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE))
		return retval;

	/*
	Standard programming
	The Flash memory programming sequence is as follows:
	1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register.
	2. Set the PG bit in the FLASH_CR register
	3. Perform the data write operation(s) to the desired memory address (inside main
	  memory block or OTP area):
	– – Half-word access in case of x16 parallelism
	– Word access in case of x32 parallelism
	–
	4.
	Byte access in case of x8 parallelism
	Double word access in case of x64 parallelism
	Wait for the BSY bit to be cleared
	*/
	while (words_remaining > 0)
	{
		uint16_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint16_t));

		retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR),
				FLASH_PG | FLASH_PSIZE_16);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u16(target, address, value);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 2;
		words_remaining--;
		address += 2;
	}

	if (bytes_remaining)
	{
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR),
				FLASH_PG | FLASH_PSIZE_8);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u8(target, address, buffer[bytes_written]);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	return target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);
}

static void setup_sector(struct flash_bank *bank, int start, int num, int size)
{
	for (int i = start; i < (start + num) ; i++)
	{
		bank->sectors[i].offset = bank->size;
		bank->sectors[i].size = size;
		bank->size += bank->sectors[i].size;
	}
}

static int stm32x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	int i;
	uint16_t num_pages;
	uint32_t device_id;
	uint32_t base_address = 0x08000000;

	stm32x_info->probed = 0;

	/* read stm32 device id register */
	int retval = target_read_u32(target, 0xE0042000, &device_id);
	if (retval != ERROR_OK)
		return retval;
	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	if ((device_id & 0x7ff) != 0x411)
	{
		LOG_WARNING("Cannot identify target as a STM32 family, try the other STM32 drivers.");
		return ERROR_FAIL;
	}

	/* sectors sizes vary, handle this in a different code path
	 * than the rest.
	 */
	// Uhhh.... what to use here?

	/* calculate numbers of pages*/
	num_pages = 4 + 1 + 7;

	if (bank->sectors)
	{
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	bank->size = 0;
	setup_sector(bank, 0, 4, 16 * 1024);
	setup_sector(bank, 4, 1, 64 * 1024);
	setup_sector(bank, 4+1, 7, 128 * 1024);

	for (i = 0; i < num_pages; i++)
	{
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	LOG_INFO("flash size = %dkBytes", bank->size / 1024);

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

static int get_stm32x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	uint32_t device_id;
	int printed;

	/* read stm32 device id register */
	int retval = target_read_u32(target, 0xE0042000, &device_id);
	if (retval != ERROR_OK)
		return retval;

	if ((device_id & 0x7ff) == 0x411)
	{
		printed = snprintf(buf, buf_size, "stm32x (1mByte part) - Rev: ");
		buf += printed;
		buf_size -= printed;

		switch (device_id >> 16)
		{
			case 0x1000:
				snprintf(buf, buf_size, "A");
				break;

			case 0x2000:
				snprintf(buf, buf_size, "B");
				break;

			case 0x1001:
				snprintf(buf, buf_size, "Z");
				break;

			case 0x2001:
				snprintf(buf, buf_size, "Y");
				break;

			default:
				snprintf(buf, buf_size, "unknown");
				break;
		}
	}
	else
	{
		snprintf(buf, buf_size, "Cannot identify target as a stm32x\n");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct command_registration stm32x_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32f2xxx",
		.mode = COMMAND_ANY,
		.help = "stm32f2xxx flash command group",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32xf2xxx_flash = {
	.name = "stm32f2xxx",
	.commands = stm32x_command_handlers,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.read = default_flash_read,
	.probe = stm32x_probe,
	.auto_probe = stm32x_auto_probe,
	.erase_check = default_flash_mem_blank_check,
	.protect_check = stm32x_protect_check,
	.info = get_stm32x_info,
};

/***************************************************************************
 *   Copyright (C) 2008 by Kevin McGuire                                   *
 *   Copyright (C) 2008 by Marcel Wijlaars                                 *
 *   Copyright (C) 2009 by Michael Ashton                                  *
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
#include "time_support.h"
#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv4_5.h"
#include "algorithm.h"
#include "binarybuffer.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int aduc702x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int aduc702x_register_commands(struct command_context_s *cmd_ctx);
int aduc702x_erase(struct flash_bank_s *bank, int first, int last);
int aduc702x_protect(struct flash_bank_s *bank, int set, int first, int last);
int aduc702x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int aduc702x_write_single(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int aduc702x_write_block(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int aduc702x_probe(struct flash_bank_s *bank);
int aduc702x_info(struct flash_bank_s *bank, char *buf, int buf_size);
int aduc702x_protect_check(struct flash_bank_s *bank);
int aduc702x_build_sector_list(struct flash_bank_s *bank);
int aduc702x_check_flash_completion(target_t* target, unsigned int timeout_ms);
int aduc702x_set_write_enable(target_t *target, int enable);

#define ADUC702x_FLASH				0xfffff800
#define ADUC702x_FLASH_FEESTA		(0*4)
#define ADUC702x_FLASH_FEEMOD		(1*4)
#define ADUC702x_FLASH_FEECON		(2*4)
#define ADUC702x_FLASH_FEEDAT		(3*4)
#define ADUC702x_FLASH_FEEADR		(4*4)
#define ADUC702x_FLASH_FEESIGN		(5*4)
#define ADUC702x_FLASH_FEEPRO		(6*4)
#define ADUC702x_FLASH_FEEHIDE		(7*4)

typedef struct {
	u32 feesta;
	u32 feemod;
	u32 feecon;
	u32 feedat;
	u32 feeadr;
	u32 feesign;
	u32 feepro;
	u32 feehide;
} ADUC702x_FLASH_MMIO;

typedef struct
{
	working_area_t *write_algorithm;
} aduc702x_flash_bank_t;

flash_driver_t aduc702x_flash =
{
	.name = "aduc702x",
	.register_commands = aduc702x_register_commands,
	.flash_bank_command = aduc702x_flash_bank_command,
	.erase = aduc702x_erase,
	.protect = aduc702x_protect,
	.write = aduc702x_write,
	.probe = aduc702x_probe,
	.auto_probe = aduc702x_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = aduc702x_protect_check,
	.info = aduc702x_info
};

int aduc702x_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

/* flash bank aduc702x 0 0 0 0 <target#>
 * The ADC7019-28 devices all have the same flash layout */
int aduc702x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	aduc702x_flash_bank_t *nbank;

	nbank = malloc(sizeof(aduc702x_flash_bank_t));

        bank->base = 0x80000;
        bank->size = 0xF800; // top 4k not accessible
	bank->driver_priv = nbank;

        aduc702x_build_sector_list(bank);

        return ERROR_OK;
}

int aduc702x_build_sector_list(struct flash_bank_s *bank)
{
	//aduc7026_flash_bank_t *aduc7026_info = bank->driver_priv;
	
        int i = 0;
        u32 offset = 0;
		
        // sector size is 512
        bank->num_sectors = bank->size / 512;
        bank->sectors = malloc(sizeof(flash_sector_t) * bank->num_sectors);
        for (i = 0; i < bank->num_sectors; ++i)
        {
                bank->sectors[i].offset = offset;
                bank->sectors[i].size = 512;
                offset += bank->sectors[i].size;
                bank->sectors[i].is_erased = -1;
                bank->sectors[i].is_protected = 0;
        }

	return ERROR_OK;
}

int aduc702x_protect_check(struct flash_bank_s *bank)
{
	printf("aduc702x_protect_check not implemented yet.\n");
	return ERROR_OK;
}

int aduc702x_erase(struct flash_bank_s *bank, int first, int last)
{
        //int res;
	int x;
	int count;
	//u32 v;
	target_t *target = bank->target;

        aduc702x_set_write_enable(target, 1);

	/* mass erase */
	if (((first | last) == 0) || ((first == 0) && (last >= bank->num_sectors))) {
		LOG_DEBUG("performing mass erase.\n");
		target_write_u16(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEDAT, 0x3cff);
		target_write_u16(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEADR, 0xffc3);
		target_write_u8(target, ADUC702x_FLASH + ADUC702x_FLASH_FEECON, 0x06);

                if (aduc702x_check_flash_completion(target, 3500) != ERROR_OK)
		{
			LOG_ERROR("mass erase failed\n");
                        aduc702x_set_write_enable(target, 0);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		LOG_DEBUG("mass erase successful.\n");
		return ERROR_OK;
	} else {
                unsigned long adr;

                count = last - first + 1;
                for (x = 0; x < count; ++x)
                {
                        adr = bank->base + ((first + x) * 512);

                        target_write_u16(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEADR, adr);
                        target_write_u8(target, ADUC702x_FLASH + ADUC702x_FLASH_FEECON, 0x05);

                        if (aduc702x_check_flash_completion(target, 50) != ERROR_OK)
                        {
                                LOG_ERROR("failed to erase sector at address 0x%08lX\n", adr);
                                aduc702x_set_write_enable(target, 0);
                                return ERROR_FLASH_SECTOR_NOT_ERASED;
                        }

                        LOG_DEBUG("erased sector at address 0x%08lX\n", adr);
                }
        }

        aduc702x_set_write_enable(target, 0);

	return ERROR_OK;
}

int aduc702x_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	printf("aduc702x_protect not implemented yet.\n");
	return ERROR_FLASH_OPERATION_FAILED;
}

int aduc702x_write_block(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	aduc702x_flash_bank_t *aduc702x_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 buffer_size = 7000;
	working_area_t *source;
	u32 address = bank->base + offset;
	reg_param_t reg_params[6];
	armv4_5_algorithm_t armv4_5_info;
	int retval = ERROR_OK;
	
        /* parameters:

        r0 - address of source data (absolute)
        r1 - number of halfwords to be copied
        r2 - start address in flash (offset from beginning of flash memory)
        r3 - exit code
        r4 - base address of flash controller (0xFFFFF800)

        registers:

        r5 - scratch
        r6 - set to 2, used to write flash command

        */
        u32 aduc702x_flash_write_code[] = {
        //<_start>:
                0xe3a05008,	// mov	r5, #8	; 0x8
                0xe5845004,	// str	r5, [r4, #4]
                0xe3a06002,	// mov	r6, #2	; 0x2
        //<next>:
                0xe1c421b0,	// strh	r2, [r4, #16]
                0xe0d050b2,	// ldrh	r5, [r0], #2
                0xe1c450bc,	// strh	r5, [r4, #12]
                0xe5c46008,	// strb	r6, [r4, #8]
        //<wait_complete>:
                0xe1d430b0,	// ldrh	r3, [r4]
                0xe3130004,	// tst	r3, #4	; 0x4
                0x1afffffc,	// bne	1001c <wait_complete>
                0xe2822002,	// add	r2, r2, #2	; 0x2
                0xe2511001,	// subs	r1, r1, #1	; 0x1
                0x0a000001,	// beq	1003c <done>
                0xe3130001,	// tst	r3, #1	; 0x1
                0x1afffff3,	// bne	1000c <next>
        //<done>:
                0xeafffffe 	// b	1003c <done>
	};
	
	/* flash write code */
	if (target_alloc_working_area(target, sizeof(aduc702x_flash_write_code),
                &aduc702x_info->write_algorithm) != ERROR_OK)
	{
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};
	
	target_write_buffer(target, aduc702x_info->write_algorithm->address, 
                sizeof(aduc702x_flash_write_code), (u8*)aduc702x_flash_write_code);

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
			if (aduc702x_info->write_algorithm)
				target_free_working_area(target, aduc702x_info->write_algorithm);
			
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
	
	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;
	
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);
	
	while (count > 0)
	{
		u32 thisrun_count = (count > (buffer_size / 2)) ? (buffer_size / 2) : count;
		
		target_write_buffer(target, source->address, thisrun_count * 2, buffer);

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, thisrun_count);
		buf_set_u32(reg_params[2].value, 0, 32, address);
		buf_set_u32(reg_params[4].value, 0, 32, 0xFFFFF800);

		if ((retval = target->type->run_algorithm(target, 0, NULL, 5, 
                        reg_params, aduc702x_info->write_algorithm->address, 
                        aduc702x_info->write_algorithm->address + sizeof(aduc702x_flash_write_code) - 4, 
                        10000, &armv4_5_info)) != ERROR_OK)
		{
			LOG_ERROR("error executing aduc702x flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
	
		if ((buf_get_u32(reg_params[3].value, 0, 32) & 1) != 1) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 2;
		address += thisrun_count * 2;
		count -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, aduc702x_info->write_algorithm);
	
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	
	return retval;
}

/* All-JTAG, single-access method.  Very slow.  Used only if there is no 
 * working area available. */
int aduc702x_write_single(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	int x;
        u8 b;
	target_t *target = bank->target;
	
        aduc702x_set_write_enable(target, 1);

	for (x = 0; x < count; x += 2) {
                // FEEADR = address
		target_write_u16(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEADR, offset + x);

                // set up data
		if ((x + 1) == count)
                {
                        // last byte
                        target_read_u8(target, offset + x + 1, &b);
                }
                else
                        b = buffer[x + 1];

                target_write_u16(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEDAT, buffer[x] | (b << 8));

                // do single-write command
		target_write_u8(target, ADUC702x_FLASH + ADUC702x_FLASH_FEECON, 0x02);

                if (aduc702x_check_flash_completion(target, 1) != ERROR_OK)
                {
			LOG_ERROR("single write failed for address 0x%08lX\n", (unsigned long)(offset + x));
                        aduc702x_set_write_enable(target, 0);
			return ERROR_FLASH_OPERATION_FAILED;
		}

	}
        LOG_DEBUG("wrote %d bytes at address 0x%08lX\n", (int)count, (unsigned long)(offset + x));

        aduc702x_set_write_enable(target, 0);

	return ERROR_OK;
}

int aduc702x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	int retval;

        /* try using a block write */
        if ((retval = aduc702x_write_block(bank, buffer, offset, count)) != ERROR_OK)
        {
                if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
                {
                        /* if block write failed (no sufficient working area),
                         * use normal (slow) JTAG method */ 
                        LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

                        if ((retval = aduc702x_write_single(bank, buffer, offset, count)) != ERROR_OK)
                        {
                                LOG_ERROR("slow write failed");
                                return ERROR_FLASH_OPERATION_FAILED; 
                        }
                }
                else if (retval == ERROR_FLASH_OPERATION_FAILED)
                {
                        LOG_ERROR("flash block writing failed");
                        return ERROR_FLASH_OPERATION_FAILED;
                }
        }

        return ERROR_OK;
}

int aduc702x_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int aduc702x_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "aduc702x flash driver info" );
	return ERROR_OK;
}

/* sets FEEMOD bit 3
 * enable = 1 enables writes & erases, 0 disables them */
int aduc702x_set_write_enable(target_t *target, int enable)
{
        // don't bother to preserve int enable bit here
        target_write_u16(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEMOD, enable ? 8 : 0);

        return ERROR_OK;
}

/* wait up to timeout_ms for controller to not be busy,
 * then check whether the command passed or failed.
 *
 * this function sleeps 1ms between checks (after the first one),
 * so in some cases may slow things down without a usleep after the first read */
int aduc702x_check_flash_completion(target_t* target, unsigned int timeout_ms)
{
        u8 v = 4;

        long long endtime = timeval_ms() + timeout_ms;
        while (1) {
                target_read_u8(target, ADUC702x_FLASH + ADUC702x_FLASH_FEESTA, &v);
                if ((v & 4) == 0) break;
                alive_sleep(1);
                if (timeval_ms() >= endtime) break;
        }

        if (v & 2) return ERROR_FAIL;
        // if a command is ignored, both the success and fail bits may be 0
        else if ((v & 3) == 0) return ERROR_FAIL;
        else return ERROR_OK;
}


/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gemini.h"
#include "gemini_bit.h"
#include "pld.h"
#include "helper/time_support.h"

//#define LOCAL_BUILD
//#define EMULATOR_BUILD

#ifdef LOCAL_BUILD
#define GEMINI_IDCODE         	0x20000913
#define GEMINI_DEBUG_CONTROL  	0x80001028
#define GEMINI_SPARE_REG      	0x800010f0
#define GEMINI_SRAM_ADDRESS   	0x80002000
#endif

#ifdef EMULATOR_BUILD
#define GEMINI_IDCODE         	0x1000563d
#define GEMINI_DEBUG_CONTROL  	0xf1000028
#define GEMINI_SPARE_REG      	0x800000f0
#define GEMINI_SRAM_ADDRESS   	0x80001000
#endif

#if !defined(LOCAL_BUILD) && !defined(EMULATOR_BUILD)
#define GEMINI_IDCODE         	0x1000563d
#define GEMINI_DEBUG_CONTROL  	0xf1000028
#define GEMINI_SPARE_REG      	0xf10000f0
#define GEMINI_SRAM_ADDRESS   	0x80000000
#endif

#define GEMINI_LOAD_ADDRESS   	0x00000000
#define GEMINI_SRAM_SIZE	   	(255 * 1024)
#define GEMINI_BOOTROM			1
#define GEMINI_FSBL				2
#define GEMINI_ACPU				1
#define GEMINI_BCPU				0
#define GEMINI_COMMAND_A   		1 /* For BootROM, load FSBL from SRAM, For FSBL, initialize DDR */
#define GEMINI_COMMAND_B   		2 /* For FSBL, write content from DDR to QSPI */
#define GEMINI_COMMAND_C   		3 /* For FSBL, process bitstream from DDR */
#define GEMINI_COMMAND_SUCCESS  1
#define GEMINI_COMMAND_POLLS	5
#define DM_SBCS                 0x38
#define DM_SBADDRESS0           0x39
#define DM_SBDATA0              0x3c

static int gemini_sysbus_write_reg32(struct target * target, target_addr_t address, uint32_t value)
{
#ifndef LOCAL_BUILD
	struct riscv_info * ri = target->arch_info;

	LOG_DEBUG("[RS] Writing 0x%08x to 0x%08x via System Bus on ACPU", value, (uint32_t)address);

	if (ri->dmi_write(target, DM_SBCS, (2u << 17) | (7u << 12)) != ERROR_OK)
		return ERROR_FAIL;

	if (ri->dmi_write(target, DM_SBADDRESS0, address) != ERROR_OK)
		return ERROR_FAIL;

	//if (ri->dmi_write(target, DM_SBDATA0, value) != ERROR_OK)
	//    return ERROR_FAIL;
	ri->dmi_write(target, DM_SBDATA0, value);
#endif
	return ERROR_OK;
}

static int gemini_write_reg32(struct target * target, target_addr_t address, uint32_t width, uint32_t offset, uint32_t value)
{
	int retval = ERROR_OK;
	uint32_t tmp;

	if (target_halt(target) == ERROR_OK) 
	{
		if (target_read_u32(target, address, &tmp) == ERROR_OK)
		{
			uint32_t bitmask = (1u << width) - 1;

			if (target_write_u32(target, address, (tmp & ~(bitmask << offset)) | ((value & bitmask) << offset)) != ERROR_OK)
			{
				LOG_WARNING("[RS] Failed to write to address 0x%08x", (uint32_t)address);
				retval = ERROR_FAIL;
			}
		}
		else
		{
			LOG_WARNING("[RS] Failed to read from address 0x%08x", (uint32_t)address);
			retval = ERROR_FAIL;
		}
		target_resume(target, true, 0, true, false);
	}
	else
	{
		LOG_WARNING("[RS] Failed to halt target");
		retval = ERROR_FAIL;
	}

	return retval;
}

static int gemini_read_reg32(struct target * target, target_addr_t address, uint32_t *value)
{
	int retval = ERROR_OK;

	if (target_halt(target) == ERROR_OK) 
	{
		if (target_read_u32(target, address, value) != ERROR_OK)
		{
			LOG_WARNING("[RS] Failed to read from address 0x%08lx", address);
			retval = ERROR_FAIL;
		}
		target_resume(target, true, 0, true, false);
	}
	else
	{
		LOG_WARNING("[RS] Failed to halt target");
		retval = ERROR_FAIL;
	}

	return retval;
}

static int gemini_get_cpu_type(struct target * target, uint32_t *cpu_type)
{
	uint32_t debug_control;

	if (gemini_read_reg32(target, GEMINI_DEBUG_CONTROL, &debug_control) == ERROR_OK)
	{
		*cpu_type = debug_control & 0x1 ? GEMINI_ACPU : GEMINI_BCPU;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_get_firmware_type(struct target * target, uint32_t *fw_type)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		spare_reg >>= 29;
		if (spare_reg == GEMINI_BOOTROM || spare_reg == GEMINI_FSBL)
		{
			*fw_type = spare_reg;
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int gemini_get_command_status(struct target * target, uint32_t *status)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		*status = (spare_reg >> 10) & 0x3f;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_switch_to_bcpu(struct target * target)
{
	LOG_INFO("[RS] Perform switching to BCPU...");

	if (gemini_sysbus_write_reg32(target, GEMINI_DEBUG_CONTROL, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write debug_control register");
		return ERROR_FAIL;
	}
#ifndef LOCAL_BUILD
	uint32_t cpu_type;
	jtag_add_tlr();
	jtag_execute_queue();
	target->examined = false;

	if (target_examine_one(target) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to re-initialize the target");
		return ERROR_FAIL;
	}

	if (gemini_get_cpu_type(target, &cpu_type) != ERROR_OK || cpu_type == GEMINI_ACPU)
	{
		LOG_INFO("[RS] Failed to switch to BCPU");
		return ERROR_FAIL;
	}
#endif
	LOG_INFO("[RS] Switched to BCPU");

	return ERROR_OK;
}

static int gemini_poll_command_complete_and_status(struct target * target, uint32_t *status)
{
	uint32_t num_polls = 0;

	while (1)
	{
		LOG_INFO("[RS] Poll command status in 1 second. Poll #%d...", num_polls+1);

		// wait for a second
		sleep(1);

		if (gemini_get_command_status(target, status) != ERROR_OK)
			return ERROR_FAIL;

		if (*status != 0)
			break;

		if (++num_polls >= GEMINI_COMMAND_POLLS)
		{
			LOG_ERROR("[RS] Timed out waiting for task to complete.");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int gemini_load_config_fsbl(struct target *target, gemini_bit_file_t *bit_file)
{
	int retval = ERROR_OK;
	uint8_t *bitstream = NULL;
	uint32_t filesize;
	uint32_t status;
	uint32_t spare_reg;

	LOG_INFO("[RS] Loading FSBL firmware...");

	if (gemini_create_helper_bitstream(bit_file, &bitstream, &filesize) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to create helper bitstream");
		return ERROR_FAIL;
	}

	if (filesize > GEMINI_SRAM_SIZE)
	{
		LOG_ERROR("[RS] Helper bitstream size (%d bytes) is larger than Gemini available SRAM (%d bytes)", filesize, GEMINI_SRAM_SIZE);
		free(bitstream);
		return ERROR_FAIL;	
	}

	if (target_halt(target) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to halt target");
		free(bitstream);
		return ERROR_FAIL;
	}

	if (retval == ERROR_OK)
	{
#ifdef LOCAL_BUILD
		if (filesize > 4096) filesize = 4096;
#endif
		if (target_write_memory(target, GEMINI_SRAM_ADDRESS, 4, filesize/4, bitstream) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to write bitstream of size %d byte(s) to SRAM at 0x%08x", filesize, GEMINI_SRAM_ADDRESS);
			retval = ERROR_FAIL;
		}
		else
		{
			LOG_INFO("[RS] Wrote %d byte(s) to SRAM at 0x%08x", filesize, GEMINI_SRAM_ADDRESS);
		}
	}

	if (retval == ERROR_OK)
	{
		if (target_read_u32(target, GEMINI_SPARE_REG, &spare_reg) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to read spare_reg");
			retval = ERROR_FAIL;
		}
	}

	if (retval == ERROR_OK)
	{
		if (target_write_u32(target, GEMINI_SPARE_REG, (spare_reg & ~0xffff) | GEMINI_COMMAND_A) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to write command to spare_reg");
			retval = ERROR_FAIL;
		}
		else
		{
			LOG_INFO("[RS] Wrote command to spare_reg");
		}
	}

	target_resume(target, true, 0, true, false);
	free(bitstream);
		
	if (retval == ERROR_OK)
	{
		if (gemini_poll_command_complete_and_status(target, &status) == ERROR_OK)
		{
			if (status != GEMINI_COMMAND_SUCCESS)
			{
				LOG_ERROR("[RS] Command completed with error status %d", status);
				retval = ERROR_FAIL;
			}
		}
		else
			retval = ERROR_FAIL;
	}

	gemini_write_reg32(target, GEMINI_SPARE_REG, 16, 0, 0);

	if (retval != ERROR_OK)
		LOG_ERROR("[RS] Failed to load FSBL firmware");
	else
		LOG_INFO("[RS] Loaded FSBL firmware of size %d byte(s)", filesize);

	return retval;
}

static int gemini_load(struct pld_device *pld_device, const char *filename)
{
	struct gemini_pld_device *gemini_info = pld_device->driver_priv;
	gemini_bit_file_t bit_file;
	uint32_t fw_type;
	uint32_t cpu;

	if (gemini_read_bit_file(&bit_file, filename) != ERROR_OK)
		return ERROR_FAIL;

	if (gemini_info->tap->idcode != GEMINI_IDCODE)
	{
		LOG_ERROR("[RS] Not gemini device");
		return ERROR_FAIL;
	}

	if (gemini_get_cpu_type(gemini_info->target, &cpu) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to get connected cpu type");
		return ERROR_FAIL;
	}

	if (cpu == GEMINI_ACPU)
	{
		LOG_INFO("[RS] Connected to ACPU");
		if (gemini_switch_to_bcpu(gemini_info->target) != ERROR_OK)
			return ERROR_FAIL;
	}
	else
	{
		LOG_INFO("[RS] Connected to BCPU");
	}

	if (gemini_get_firmware_type(gemini_info->target, &fw_type) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to determine the firmware type");
		return ERROR_FAIL;
	}

	if (fw_type == GEMINI_BOOTROM)
	{
		LOG_INFO("[RS] Bootrom firmware type detected");
		if (gemini_load_config_fsbl(gemini_info->target, &bit_file) != ERROR_OK)
			return ERROR_FAIL;
	}
	else
	{
		LOG_INFO("[RS] FSBL firmware type detected");
	}

	// todo: check ddr init state and trigger fsbl to init ddr if not
	// todo: load bitstream onto DDR & trigger fsbl to do bitstream programming to fabric

	if (gemini_info->target->state == TARGET_HALTED)
		target_resume(gemini_info->target, true, 0, true, false);

	return ERROR_OK;
}

PLD_DEVICE_COMMAND_HANDLER(gemini_pld_device_command)
{
	struct gemini_pld_device *gemini_info;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_target(CMD_ARGV[1]);
	if (!target) {
		command_print(CMD, "Target: %s does not exist", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	if (!target->tap) {
		command_print(CMD, "Target: %s is not on jtag chain", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	gemini_info = malloc(sizeof(struct gemini_pld_device));
	gemini_info->target = target;
	gemini_info->tap = target->tap;

	pld->driver_priv = gemini_info;

	return ERROR_OK;
}

struct pld_driver gemini_pld = {
	.name = "gemini",
	.pld_device_command = &gemini_pld_device_command,
	.load = &gemini_load,
};

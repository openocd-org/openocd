/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gemini.h"
#include "pld.h"
#include "helper/time_support.h"

//#define LOCAL_BUILD

#ifdef LOCAL_BUILD
#  define GEMINI_IDCODE         0x20000913
#  define GEMINI_SPARE_REG      0x800010f0
#  define GEMINI_DEBUG_CONTROL  0x80001028
#  define GEMINI_LOAD_ADDRESS   0x80000000
#else
#  define GEMINI_IDCODE         0x1000563d
#  define GEMINI_SPARE_REG      0xf10000f0
#  define GEMINI_DEBUG_CONTROL  0xf1000028
#  define GEMINI_LOAD_ADDRESS   0x00000000
#  define GEMINI_SRAM_ADDRESS   0x80000000
#endif

#define DM_SBCS                 0x38
#define DM_SBADDRESS0           0x39
#define DM_SBDATA0              0x3c
#define GEMINI_BOOTROM			1
#define GEMINI_FSBL				2
#define GEMINI_ACPU				1
#define GEMINI_BCPU				0
#define GEMINI_COMMAND_A   		1 /* For BootROM, load FSBL from SRAM, For FSBL, initialize DDR */
#define GEMINI_COMMAND_B   		2 /* For FSBL, write content from DDR to QSPI */
#define GEMINI_COMMAND_C   		3 /* For FSBL, process bitstream from DDR */
#define GEMINI_COMMAND_SUCCESS  1
#define GEMINI_COMMAND_POLLS	5
#define GEMINI_COMMAND_WAIT_MS	50

static int gemini_sysbus_write_reg32(struct target * target, target_addr_t address, uint32_t value)
{
    struct riscv_info * ri = target->arch_info;

    LOG_DEBUG("[RS] Writing 0x%08x to 0x%08x via System Bus on ACPU", value, (uint32_t)address);

    if (ri->dmi_write(target, DM_SBCS, (2u << 17) | (7u << 12)) != ERROR_OK)
        return ERROR_FAIL;

    if (ri->dmi_write(target, DM_SBADDRESS0, address) != ERROR_OK)
        return ERROR_FAIL;

    if (ri->dmi_write(target, DM_SBDATA0, value) != ERROR_OK)
        return ERROR_FAIL;

    return ERROR_OK;
}

static int gemini_get_cpu_type(struct target * target, uint32_t *cpu)
{
    int retval = ERROR_FAIL;
    uint32_t debug_control;

    if (target_halt(target) != ERROR_OK)
        return ERROR_FAIL;

    if (target_read_u32(target, GEMINI_DEBUG_CONTROL, &debug_control) == ERROR_OK)
	{
        *cpu = debug_control & 0x1 ? GEMINI_ACPU : GEMINI_BCPU;
		retval = ERROR_OK;
	}

    target_resume(target, true, 0, true, false);

    return retval;
}

static int gemini_get_firmware_type(struct target * target, uint32_t *fw_type)
{
    int retval = ERROR_FAIL;
	uint32_t spare_reg;

    if (target_halt(target) != ERROR_OK)
        return ERROR_FAIL;

    if (target_read_u32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		spare_reg >>= 29;
		if (spare_reg == GEMINI_BOOTROM || spare_reg == GEMINI_FSBL) {
        	*fw_type = spare_reg;
			retval = ERROR_OK;
		}
	}

    target_resume(target, true, 0, true, false);

    return retval;
}

static int gemini_get_command_status(struct target * target, uint32_t *status)
{
    int retval = ERROR_FAIL;
	uint32_t spare_reg;

    if (target_halt(target) != ERROR_OK)
        return ERROR_FAIL;

    if (target_read_u32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		*status = (spare_reg >> 10) & 0x3f;
		retval = ERROR_OK;
	}

    target_resume(target, true, 0, true, false);

    return retval;
}

static int gemini_switch_to_bcpu(struct target * target)
{
	int retval = ERROR_FAIL;
    uint32_t cpu = 0;

    if (gemini_sysbus_write_reg32(target, GEMINI_DEBUG_CONTROL, 0) != ERROR_OK)
        return ERROR_FAIL;

	// todo: reset target to reflect bcpu state

    if (gemini_get_cpu_type(target, &cpu) == ERROR_OK)
	{
		if (cpu == GEMINI_BCPU) {
			retval = ERROR_OK;
		}
	}

    return retval;
}

static int gemini_write_command(struct target * target, uint32_t cmd_id)
{
	int retval = ERROR_FAIL;
	uint32_t spare_reg = 0;

	if (target_halt(target) != ERROR_OK)
        return ERROR_FAIL;

    if (target_read_u32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		// clear task command and status fields
		spare_reg &= ~(0xffff << 0);

		// set task command
		spare_reg |= ((cmd_id & 0x3FF) << 0);

		// write back spare reg
		if (target_write_u32(target, GEMINI_SPARE_REG, spare_reg) == ERROR_OK)
		{
			retval = ERROR_OK;
		}
	}

    target_resume(target, true, 0, true, false);

	return retval;
}

static void gemini_wait(uint32_t miliseconds)
{
	int64_t start = timeval_ms();
	while ((timeval_ms() - start) < miliseconds);
}

static int gemini_poll_command_complete_and_status(struct target * target, uint32_t *status)
{
	uint32_t attempts = 0;

	while (1)
	{
		if (gemini_get_command_status(target, status) != ERROR_OK)
			return ERROR_FAIL;

		if (*status != 0)
		{
			if (gemini_write_command(target, 0) != ERROR_OK) // clear command and status
				return ERROR_FAIL;
			return ERROR_OK;
		}

		if (++attempts >= GEMINI_COMMAND_POLLS) {
			LOG_WARNING("[RS] Timed out waiting for task to complete.");
			// todo: clear command and status and reset cpu to known state
			return ERROR_FAIL;
		}
		gemini_wait(GEMINI_COMMAND_WAIT_MS);
	}

	return ERROR_FAIL;
}

static int gemini_load_config_fsbl(struct target * target, uint8_t *fsbl, uint32_t size)
{
	int retval = ERROR_FAIL;
	uint32_t cmd_status;

	if (target_halt(target) == ERROR_OK)
	{
		if (target_write_memory(target, GEMINI_SRAM_ADDRESS, 4, size / 4, fsbl) == ERROR_OK)
			retval = ERROR_OK;
		target_resume(target, true, 0, true, false);
	}

	if (retval == ERROR_OK)
	{
		if (gemini_write_command(target, GEMINI_COMMAND_A) != ERROR_OK)
			return ERROR_FAIL;

		if (gemini_poll_command_complete_and_status(target, &cmd_status) == ERROR_OK)
		{
			if (cmd_status != GEMINI_COMMAND_SUCCESS) {
				LOG_ERROR("[RS] Command completed with error status %d. Quit.", cmd_status);
				return ERROR_FAIL;
			}
			return ERROR_OK;
		}
	}

	return retval;
}

static int gemini_load(struct pld_device *pld_device, const char *filename)
{
	struct gemini_pld_device *gemini_info = pld_device->driver_priv;
	uint32_t fw_type;
	uint32_t cpu;

    if (gemini_info->tap->idcode != GEMINI_IDCODE)
    {
		LOG_ERROR("[RS] Not gemini device. Quit.");
        return ERROR_FAIL;
    }

    if (gemini_get_cpu_type(gemini_info->target, &cpu) != ERROR_OK)
    {
        LOG_ERROR("[RS] Failed to determine jtag is connected to ACPU or BCPU. Quit.");
        return ERROR_FAIL;
    }

	if (cpu == GEMINI_ACPU)
    {
        if (gemini_switch_to_bcpu(gemini_info->target) != ERROR_OK)
        {
            LOG_ERROR("[RS] Failed to switch to BCPU. Quit.");
            return ERROR_FAIL;          
        }
    }
	
	if (gemini_get_firmware_type(gemini_info->target, &fw_type) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to determine the firmware type currently running on BCPU. Quit.");
        return ERROR_FAIL;
	}
	
	if (fw_type == GEMINI_BOOTROM)
	{
		// todo: prepare helper bitstream to bring up configuration fsbl
		//       mutiple of 32-bit
		uint8_t fsbl[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

		// todo: check fsbl size to fix sram

		if (gemini_load_config_fsbl(gemini_info->target, &fsbl[0], sizeof(fsbl)) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to load config FSBL. Quit.");
        	return ERROR_FAIL;
		}
	}

	// todo: check ddr init state
	// todo: load bitstream onto DDR

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

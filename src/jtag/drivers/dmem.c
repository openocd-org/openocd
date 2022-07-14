// SPDX-License-Identifier: GPL-2.0-or-later

/* Copyright (C) 2022 Texas Instruments Incorporated - https://www.ti.com/ */

/**
 * @file
 * This file implements support for the Direct memory access to CoreSight
 * Access Ports (APs) or emulate the same to access CoreSight debug registers
 * directly.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/mman.h>

#include <helper/align.h>
#include <helper/types.h>
#include <helper/system.h>
#include <helper/time_support.h>
#include <helper/list.h>
#include <jtag/interface.h>

#include <target/arm_adi_v5.h>
#include <transport/transport.h>

static void *dmem_map_base, *dmem_virt_base_addr;
static size_t dmem_mapped_size;

/* Default dmem device. */
#define DMEM_DEV_PATH_DEFAULT   "/dev/mem"
static char *dmem_dev_path;
static uint64_t dmem_dap_base_address;
static unsigned int dmem_dap_max_aps = 1;
static uint32_t dmem_dap_ap_offset = 0x100;

/* AP MODE */
static uint32_t dmem_get_ap_reg_offset(struct adiv5_ap *ap, unsigned int reg)
{
	return (dmem_dap_ap_offset * ap->ap_num) + reg;
}

static void dmem_set_ap_reg(struct adiv5_ap *ap, unsigned int reg, uint32_t val)
{
	*(volatile uint32_t *)((uintptr_t)dmem_virt_base_addr +
						   dmem_get_ap_reg_offset(ap, reg)) = val;
}

static uint32_t dmem_get_ap_reg(struct adiv5_ap *ap, unsigned int reg)
{
	return *(volatile uint32_t *)((uintptr_t)dmem_virt_base_addr +
								  dmem_get_ap_reg_offset(ap, reg));
}

static int dmem_dp_q_read(struct adiv5_dap *dap, unsigned int reg, uint32_t *data)
{
	if (!data)
		return ERROR_OK;

	switch (reg) {
		case DP_CTRL_STAT:
			*data = CDBGPWRUPACK | CSYSPWRUPACK;
			break;

		default:
			*data = 0;
			break;
	}

	return ERROR_OK;
}

static int dmem_dp_q_write(struct adiv5_dap *dap, unsigned int reg, uint32_t data)
{
	return ERROR_OK;
}

static int dmem_ap_q_read(struct adiv5_ap *ap, unsigned int reg, uint32_t *data)
{
	if (is_adiv6(ap->dap)) {
		static bool error_flagged;

		if (!error_flagged)
			LOG_ERROR("ADIv6 dap not supported by dmem dap-direct mode");

		error_flagged = true;

		return ERROR_FAIL;
	}

	*data = dmem_get_ap_reg(ap, reg);

	return ERROR_OK;
}

static int dmem_ap_q_write(struct adiv5_ap *ap, unsigned int reg, uint32_t data)
{
	if (is_adiv6(ap->dap)) {
		static bool error_flagged;

		if (!error_flagged)
			LOG_ERROR("ADIv6 dap not supported by dmem dap-direct mode");

		error_flagged = true;

		return ERROR_FAIL;
	}

	dmem_set_ap_reg(ap, reg, data);

	return ERROR_OK;
}

static int dmem_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	return ERROR_OK;
}

static int dmem_dp_run(struct adiv5_dap *dap)
{
	return ERROR_OK;
}

static int dmem_connect(struct adiv5_dap *dap)
{
	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_device_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(dmem_dev_path);
	dmem_dev_path = strdup(CMD_ARGV[0]);

	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_base_address_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], dmem_dap_base_address);

	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_max_aps_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], dmem_dap_max_aps);

	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_ap_offset_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], dmem_dap_ap_offset);

	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_config_info_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "dmem (Direct Memory) AP Adapter Configuration:");
	command_print(CMD, " Device       : %s",
				  dmem_dev_path ? dmem_dev_path : DMEM_DEV_PATH_DEFAULT);
	command_print(CMD, " Base Address : 0x%" PRIx64, dmem_dap_base_address);
	command_print(CMD, " Max APs      : %u", dmem_dap_max_aps);
	command_print(CMD, " AP offset    : 0x%08" PRIx32, dmem_dap_ap_offset);

	return ERROR_OK;
}

static const struct command_registration dmem_dap_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = dmem_dap_config_info_command,
		.mode = COMMAND_ANY,
		.help = "print the config info",
		.usage = "",
	},
	{
		.name = "device",
		.handler = dmem_dap_device_command,
		.mode = COMMAND_CONFIG,
		.help = "set the dmem memory access device (default: /dev/mem)",
		.usage = "device_path",
	},
	{
		.name = "base_address",
		.handler = dmem_dap_base_address_command,
		.mode = COMMAND_CONFIG,
		.help = "set the dmem dap AP memory map base address",
		.usage = "base_address",
	},
	{
		.name = "ap_address_offset",
		.handler = dmem_dap_ap_offset_command,
		.mode = COMMAND_CONFIG,
		.help = "set the offsets of each ap index",
		.usage = "offset_address",
	},
	{
		.name = "max_aps",
		.handler = dmem_dap_max_aps_command,
		.mode = COMMAND_CONFIG,
		.help = "set the maximum number of APs this will support",
		.usage = "n",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dmem_dap_command_handlers[] = {
	{
		.name = "dmem",
		.mode = COMMAND_ANY,
		.help = "Perform dmem (Direct Memory) DAP management and configuration",
		.chain = dmem_dap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int dmem_dap_init(void)
{
	char *path = dmem_dev_path ? dmem_dev_path : DMEM_DEV_PATH_DEFAULT;
	uint32_t dmem_total_memory_window_size;
	long page_size = sysconf(_SC_PAGESIZE);
	size_t dmem_mapped_start, dmem_mapped_end;
	long start_delta;
	int dmem_fd;

	if (!dmem_dap_base_address) {
		LOG_ERROR("dmem DAP Base address NOT set? value is 0");
		return ERROR_FAIL;
	}

	dmem_fd = open(path, O_RDWR | O_SYNC);
	if (dmem_fd == -1) {
		LOG_ERROR("Unable to open %s", path);
		return ERROR_FAIL;
	}

	dmem_total_memory_window_size = (dmem_dap_max_aps + 1) * dmem_dap_ap_offset;

	dmem_mapped_start = dmem_dap_base_address;
	dmem_mapped_end = dmem_dap_base_address + dmem_total_memory_window_size;
	 /* mmap() requires page aligned offsets */
	dmem_mapped_start = ALIGN_DOWN(dmem_mapped_start, page_size);
	dmem_mapped_end = ALIGN_UP(dmem_mapped_end, page_size);

	dmem_mapped_size = dmem_mapped_end - dmem_mapped_start;
	start_delta = dmem_mapped_start - dmem_dap_base_address;

	dmem_map_base = mmap(NULL,
						 dmem_mapped_size,
						 (PROT_READ | PROT_WRITE),
						 MAP_SHARED, dmem_fd,
						 dmem_mapped_start);

	close(dmem_fd);

	if (dmem_map_base == MAP_FAILED) {
		LOG_ERROR("Mapping address 0x%lx for 0x%lx bytes failed!",
			dmem_mapped_start, dmem_mapped_size);
		return ERROR_FAIL;
	}

	dmem_virt_base_addr = (void *)((uintptr_t)dmem_map_base + start_delta);

	return ERROR_OK;
}

static int dmem_dap_quit(void)
{
	if (munmap(dmem_map_base, dmem_mapped_size) == -1)
		LOG_ERROR("%s: Failed to unmap mapped memory!", __func__);

	return ERROR_OK;
}

static int dmem_dap_reset(int req_trst, int req_srst)
{
	return ERROR_OK;
}

static int dmem_dap_speed(int speed)
{
	return ERROR_OK;
}

static int dmem_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

static int dmem_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

/* DAP operations. */
static const struct dap_ops dmem_dap_ops = {
	.connect = dmem_connect,
	.queue_dp_read = dmem_dp_q_read,
	.queue_dp_write = dmem_dp_q_write,
	.queue_ap_read = dmem_ap_q_read,
	.queue_ap_write = dmem_ap_q_write,
	.queue_ap_abort = dmem_ap_q_abort,
	.run = dmem_dp_run,
};

static const char *const dmem_dap_transport[] = { "dapdirect_swd", NULL };

struct adapter_driver dmem_dap_adapter_driver = {
	.name = "dmem",
	.transports = dmem_dap_transport,
	.commands = dmem_dap_command_handlers,

	.init = dmem_dap_init,
	.quit = dmem_dap_quit,
	.reset = dmem_dap_reset,
	.speed = dmem_dap_speed,
	.khz = dmem_dap_khz,
	.speed_div = dmem_dap_speed_div,

	.dap_swd_ops = &dmem_dap_ops,
};

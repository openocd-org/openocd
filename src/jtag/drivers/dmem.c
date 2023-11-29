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

struct dmem_emu_ap_info {
	uint64_t ap_num;
	/* Emulation mode AP state variables */
	uint32_t apbap_tar;
	uint32_t apbap_csw;
};

/*
 * This bit tells if the transaction is coming in from jtag or not
 * we just mask this out to emulate direct address access
 */
#define ARM_APB_PADDR31  BIT(31)

static void *dmem_map_base, *dmem_virt_base_addr;
static size_t dmem_mapped_size;

/* Default dmem device. */
#define DMEM_DEV_PATH_DEFAULT   "/dev/mem"
static char *dmem_dev_path;
static uint64_t dmem_dap_base_address;
static unsigned int dmem_dap_max_aps = 1;
static uint32_t dmem_dap_ap_offset = 0x100;

/* DAP error code. */
static int dmem_dap_retval = ERROR_OK;

/* AP Emulation Mode */
static uint64_t dmem_emu_base_address;
static uint64_t dmem_emu_size;
static void *dmem_emu_map_base, *dmem_emu_virt_base_addr;
static size_t dmem_emu_mapped_size;
#define DMEM_MAX_EMULATE_APS 5
static unsigned int dmem_emu_ap_count;
static struct dmem_emu_ap_info dmem_emu_ap_list[DMEM_MAX_EMULATE_APS];

/*
 * This helper is used to determine the TAR increment size in bytes.  The AP's
 * CSW encoding for SIZE supports byte count decode using "1 << SIZE".
 */
static uint32_t dmem_memap_tar_inc(uint32_t csw)
{
	if ((csw & CSW_ADDRINC_MASK) != 0)
		return 1 << (csw & CSW_SIZE_MASK);
	return 0;
}

/*
 * EMULATION MODE: In Emulation MODE, we assume the following:
 * TCL still describes as system is operational from the view of AP (ex. jtag)
 * However, the hardware doesn't permit direct memory access to these APs
 * (only permitted via JTAG).
 *
 * So, the access to these APs have to be decoded to a memory map
 * access which we can directly access.
 *
 * A few TI processors have this issue.
 */
static bool dmem_is_emulated_ap(struct adiv5_ap *ap, unsigned int *idx)
{
	for (unsigned int i = 0; i < dmem_emu_ap_count; i++) {
		if (ap->ap_num == dmem_emu_ap_list[i].ap_num) {
			*idx = i;
			return true;
		}
	}
	return false;
}

static void dmem_emu_set_ap_reg(uint64_t addr, uint32_t val)
{
	addr &= ~ARM_APB_PADDR31;

	*(volatile uint32_t *)((uintptr_t)dmem_emu_virt_base_addr + addr) = val;
}

static uint32_t dmem_emu_get_ap_reg(uint64_t addr)
{
	uint32_t val;

	addr &= ~ARM_APB_PADDR31;

	val = *(volatile uint32_t *)((uintptr_t)dmem_emu_virt_base_addr + addr);

	return val;
}

static int dmem_emu_ap_q_read(unsigned int ap_idx, unsigned int reg, uint32_t *data)
{
	uint64_t addr;
	int ret = ERROR_OK;
	struct dmem_emu_ap_info *ap_info = &dmem_emu_ap_list[ap_idx];

	switch (reg) {
		case ADIV5_MEM_AP_REG_CSW:
			*data = ap_info->apbap_csw;
			break;
		case ADIV5_MEM_AP_REG_TAR:
			*data = ap_info->apbap_tar;
			break;
		case ADIV5_MEM_AP_REG_CFG:
			*data = 0;
			break;
		case ADIV5_MEM_AP_REG_BASE:
			*data = 0;
			break;
		case ADIV5_AP_REG_IDR:
			*data = 0;
			break;
		case ADIV5_MEM_AP_REG_BD0:
		case ADIV5_MEM_AP_REG_BD1:
		case ADIV5_MEM_AP_REG_BD2:
		case ADIV5_MEM_AP_REG_BD3:
			addr = (ap_info->apbap_tar & ~0xf) + (reg & 0x0C);

			*data = dmem_emu_get_ap_reg(addr);

			break;
		case ADIV5_MEM_AP_REG_DRW:
			addr = ap_info->apbap_tar;

			*data = dmem_emu_get_ap_reg(addr);

			ap_info->apbap_tar += dmem_memap_tar_inc(ap_info->apbap_csw);
			break;
		default:
			LOG_INFO("%s: Unknown reg: 0x%02x", __func__, reg);
			ret = ERROR_FAIL;
			break;
	}

	/* Track the last error code. */
	if (ret != ERROR_OK)
		dmem_dap_retval = ret;

	return ret;
}

static int dmem_emu_ap_q_write(unsigned int ap_idx, unsigned int reg, uint32_t data)
{
	uint64_t addr;
	int ret = ERROR_OK;
	struct dmem_emu_ap_info *ap_info = &dmem_emu_ap_list[ap_idx];

	switch (reg) {
		case ADIV5_MEM_AP_REG_CSW:
			/*
			 * This implementation only supports 32-bit accesses.
			 * Force this by ensuring CSW_SIZE field indicates 32-BIT.
			 */
			ap_info->apbap_csw = ((data & ~CSW_SIZE_MASK) | CSW_32BIT);
			break;
		case ADIV5_MEM_AP_REG_TAR:
			/*
			 * This implementation only supports 32-bit accesses.
			 * Force LS 2-bits of TAR to 00b
			 */
			ap_info->apbap_tar = (data & ~0x3);
			break;

		case ADIV5_MEM_AP_REG_CFG:
		case ADIV5_MEM_AP_REG_BASE:
		case ADIV5_AP_REG_IDR:
			/* We don't use this, so we don't need to store */
			break;

		case ADIV5_MEM_AP_REG_BD0:
		case ADIV5_MEM_AP_REG_BD1:
		case ADIV5_MEM_AP_REG_BD2:
		case ADIV5_MEM_AP_REG_BD3:
			addr = (ap_info->apbap_tar & ~0xf) + (reg & 0x0C);

			dmem_emu_set_ap_reg(addr, data);

			break;
		case ADIV5_MEM_AP_REG_DRW:
			addr = ap_info->apbap_tar;
			dmem_emu_set_ap_reg(addr, data);

			ap_info->apbap_tar += dmem_memap_tar_inc(ap_info->apbap_csw);
			break;
		default:
			LOG_INFO("%s: Unknown reg: 0x%02x", __func__, reg);
			ret = EINVAL;
			break;
	}

	/* Track the last error code. */
	if (ret != ERROR_OK)
		dmem_dap_retval = ret;

	return ret;
}

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
	unsigned int idx;

	if (is_adiv6(ap->dap)) {
		static bool error_flagged;

		if (!error_flagged)
			LOG_ERROR("ADIv6 dap not supported by dmem dap-direct mode");

		error_flagged = true;

		return ERROR_FAIL;
	}

	if (dmem_is_emulated_ap(ap, &idx))
		return dmem_emu_ap_q_read(idx, reg, data);

	*data = dmem_get_ap_reg(ap, reg);

	return ERROR_OK;
}

static int dmem_ap_q_write(struct adiv5_ap *ap, unsigned int reg, uint32_t data)
{
	unsigned int idx;

	if (is_adiv6(ap->dap)) {
		static bool error_flagged;

		if (!error_flagged)
			LOG_ERROR("ADIv6 dap not supported by dmem dap-direct mode");

		error_flagged = true;

		return ERROR_FAIL;
	}

	if (dmem_is_emulated_ap(ap, &idx))
		return dmem_emu_ap_q_write(idx, reg, data);

	dmem_set_ap_reg(ap, reg, data);

	return ERROR_OK;
}

static int dmem_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	return ERROR_OK;
}

static int dmem_dp_run(struct adiv5_dap *dap)
{
	int retval = dmem_dap_retval;

	/* Clear the error code. */
	dmem_dap_retval = ERROR_OK;

	return retval;
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

COMMAND_HANDLER(dmem_emu_base_address_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], dmem_emu_base_address);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], dmem_emu_size);

	return ERROR_OK;
}

COMMAND_HANDLER(dmem_emu_ap_list_command)
{
	uint64_t em_ap;

	if (CMD_ARGC < 1 || CMD_ARGC > DMEM_MAX_EMULATE_APS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[i], em_ap);
		dmem_emu_ap_list[i].ap_num = em_ap;
	}

	dmem_emu_ap_count = CMD_ARGC;

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
	command_print(CMD, " Emulated AP Count : %u", dmem_emu_ap_count);

	if (dmem_emu_ap_count) {
		command_print(CMD, " Emulated AP details:");
		command_print(CMD, " Emulated address  : 0x%" PRIx64, dmem_emu_base_address);
		command_print(CMD, " Emulated size     : 0x%" PRIx64, dmem_emu_size);
		for (unsigned int i = 0; i < dmem_emu_ap_count; i++)
			command_print(CMD, " Emulated AP [%u]  : %" PRIx64, i,
				      dmem_emu_ap_list[i].ap_num);
	}
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
	{
		.name = "emu_ap_list",
		.handler = dmem_emu_ap_list_command,
		.mode = COMMAND_CONFIG,
		.help = "set the list of AP indices to be emulated (upto max)",
		.usage = "n",
	},
	{
		.name = "emu_base_address_range",
		.handler = dmem_emu_base_address_command,
		.mode = COMMAND_CONFIG,
		.help = "set the base address and size of emulated AP range (all emulated APs access this range)",
		.usage = "base_address address_window_size",
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
	if (dmem_map_base == MAP_FAILED) {
		LOG_ERROR("Mapping address 0x%lx for 0x%lx bytes failed!",
			dmem_mapped_start, dmem_mapped_size);
		goto error_fail;
	}

	dmem_virt_base_addr = (void *)((uintptr_t)dmem_map_base + start_delta);

	/* Lets Map the emulated address if necessary */
	if (dmem_emu_ap_count) {
		dmem_mapped_start = dmem_emu_base_address;
		dmem_mapped_end = dmem_emu_base_address + dmem_emu_size;
		/* mmap() requires page aligned offsets */
		dmem_mapped_start = ALIGN_DOWN(dmem_mapped_start, page_size);
		dmem_mapped_end = ALIGN_UP(dmem_mapped_end, page_size);

		dmem_emu_mapped_size = dmem_mapped_end - dmem_mapped_start;
		start_delta = dmem_mapped_start - dmem_emu_base_address;

		dmem_emu_map_base = mmap(NULL,
									   dmem_emu_mapped_size,
									   (PROT_READ | PROT_WRITE),
									   MAP_SHARED, dmem_fd,
									   dmem_mapped_start);
		if (dmem_emu_map_base == MAP_FAILED) {
			LOG_ERROR("Mapping EMU address 0x%lx for 0x%lx bytes failed!",
					  dmem_emu_base_address, dmem_emu_size);
			goto error_fail;
		}
		dmem_emu_virt_base_addr = (void *)((uintptr_t)dmem_emu_map_base +
										   start_delta);
	}

	close(dmem_fd);
	return ERROR_OK;

error_fail:
	close(dmem_fd);
	return ERROR_FAIL;
}

static int dmem_dap_quit(void)
{
	if (munmap(dmem_map_base, dmem_mapped_size) == -1)
		LOG_ERROR("%s: Failed to unmap mapped memory!", __func__);

	if (dmem_emu_ap_count
		&& munmap(dmem_emu_map_base, dmem_emu_mapped_size) == -1)
		LOG_ERROR("%s: Failed to unmap emu mapped memory!", __func__);

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

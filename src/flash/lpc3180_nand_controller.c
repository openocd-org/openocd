/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "lpc3180_nand_controller.h"

#include "replacements.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

#include "nand.h"
#include "target.h"

int lpc3180_nand_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct nand_device_s *device);
int lpc3180_register_commands(struct command_context_s *cmd_ctx);
int lpc3180_init(struct nand_device_s *device);
int lpc3180_reset(struct nand_device_s *device);
int lpc3180_command(struct nand_device_s *device, u8 command);
int lpc3180_address(struct nand_device_s *device, u8 address);
int lpc3180_write_data(struct nand_device_s *device, u16 data);
int lpc3180_read_data(struct nand_device_s *device, void *data);
int lpc3180_write_page(struct nand_device_s *device, u32 page, u8 *data, u32 data_size, u8 *oob, u32 oob_size);
int lpc3180_read_page(struct nand_device_s *device, u32 page, u8 *data, u32 data_size, u8 *oob, u32 oob_size);
int lpc3180_controller_ready(struct nand_device_s *device, int timeout);
int lpc3180_nand_ready(struct nand_device_s *device, int timeout);

int handle_lpc3180_select_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

nand_flash_controller_t lpc3180_nand_controller =
{
	.name = "lpc3180",
	.nand_device_command = lpc3180_nand_device_command,
	.register_commands = lpc3180_register_commands,
	.init = lpc3180_init,
	.reset = lpc3180_reset,
	.command = lpc3180_command,
	.address = lpc3180_address,
	.write_data = lpc3180_write_data,
	.read_data = lpc3180_read_data,
	.write_page = lpc3180_write_page,
	.read_page = lpc3180_read_page,
	.controller_ready = lpc3180_controller_ready,
	.nand_ready = lpc3180_nand_ready,
};

/* nand device lpc3180 <target#> <oscillator_frequency>
 */
int lpc3180_nand_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct nand_device_s *device)
{
	lpc3180_nand_controller_t *lpc3180_info;
	
	if (argc < 3)
	{
		LOG_WARNING("incomplete 'lpc3180' nand flash configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	lpc3180_info = malloc(sizeof(lpc3180_nand_controller_t));
	device->controller_priv = lpc3180_info;

	lpc3180_info->target = get_target_by_num(strtoul(args[1], NULL, 0));
	if (!lpc3180_info->target)
	{
		LOG_ERROR("no target '%s' configured", args[1]);
		return ERROR_NAND_DEVICE_INVALID;
	}

	lpc3180_info->osc_freq = strtoul(args[2], NULL, 0);
	if ((lpc3180_info->osc_freq < 1000) || (lpc3180_info->osc_freq > 20000))
	{
		LOG_WARNING("LPC3180 oscillator frequency should be between 1000 and 20000 kHz, was %i", lpc3180_info->osc_freq); 
	}
	lpc3180_info->selected_controller = LPC3180_NO_CONTROLLER;
	lpc3180_info->sw_write_protection = 0;
	lpc3180_info->sw_wp_lower_bound = 0x0;
	lpc3180_info->sw_wp_upper_bound = 0x0;
		
	return ERROR_OK;
}

int lpc3180_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *lpc3180_cmd = register_command(cmd_ctx, NULL, "lpc3180", NULL, COMMAND_ANY, "commands specific to the LPC3180 NAND flash controllers");
	
	register_command(cmd_ctx, lpc3180_cmd, "select", handle_lpc3180_select_command, COMMAND_EXEC, "select <'mlc'|'slc'> controller (default is mlc)");
	
	return ERROR_OK;
}

int lpc3180_pll(int fclkin, u32 pll_ctrl)
{
	int bypass = (pll_ctrl & 0x8000) >> 15;
	int direct = (pll_ctrl & 0x4000) >> 14;
	int feedback = (pll_ctrl & 0x2000) >> 13;
	int p = (1 << ((pll_ctrl & 0x1800) >> 11) * 2);
	int n = ((pll_ctrl & 0x0600) >> 9) + 1;
	int m = ((pll_ctrl & 0x01fe) >> 1) + 1;
	int lock = (pll_ctrl & 0x1);

	if (!lock)
		LOG_WARNING("PLL is not locked");
	
	if (!bypass && direct)	/* direct mode */
		return (m * fclkin) / n;
	
	if (bypass && !direct)	/* bypass mode */
		return fclkin / (2 * p);
	
	if (bypass & direct)	/* direct bypass mode */
		return fclkin;
	
	if (feedback)			/* integer mode */
		return m * (fclkin / n);
	else					/* non-integer mode */
		return (m / (2 * p)) * (fclkin / n); 
}

float lpc3180_cycle_time(lpc3180_nand_controller_t *lpc3180_info)
{
	target_t *target = lpc3180_info->target;
	u32 sysclk_ctrl, pwr_ctrl, hclkdiv_ctrl, hclkpll_ctrl;
	int sysclk;
	int hclk;
	int hclk_pll;
	float cycle;
	
	/* calculate timings */
	
	/* determine current SYSCLK (13'MHz or main oscillator) */ 
	target_read_u32(target, 0x40004050, &sysclk_ctrl);
	
	if ((sysclk_ctrl & 1) == 0)
		sysclk = lpc3180_info->osc_freq;
	else
		sysclk = 13000;
	
	/* determine selected HCLK source */
	target_read_u32(target, 0x40004044, &pwr_ctrl);
	
	if ((pwr_ctrl & (1 << 2)) == 0) /* DIRECT RUN mode */
	{
		hclk = sysclk;
	}
	else
	{
		target_read_u32(target, 0x40004058, &hclkpll_ctrl);
		hclk_pll = lpc3180_pll(sysclk, hclkpll_ctrl);

		target_read_u32(target, 0x40004040, &hclkdiv_ctrl);
		
		if (pwr_ctrl & (1 << 10)) /* ARM_CLK and HCLK use PERIPH_CLK */
		{
			hclk = hclk_pll / (((hclkdiv_ctrl & 0x7c) >> 2) + 1);
		}
		else /* HCLK uses HCLK_PLL */
		{
			hclk = hclk_pll / (1 << (hclkdiv_ctrl & 0x3)); 
		}
	}
	
	LOG_DEBUG("LPC3180 HCLK currently clocked at %i kHz", hclk);
	
	cycle = (1.0 / hclk) * 1000000.0;
	
	return cycle;
}

int lpc3180_init(struct nand_device_s *device)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	int bus_width = (device->bus_width) ? (device->bus_width) : 8;
	int address_cycles = (device->address_cycles) ? (device->address_cycles) : 3;
	int page_size = (device->page_size) ? (device->page_size) : 512;
		
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	/* sanitize arguments */
	if ((bus_width != 8) && (bus_width != 16))
	{
		LOG_ERROR("LPC3180 only supports 8 or 16 bit bus width, not %i", bus_width);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}
	
	/* The LPC3180 only brings out 8 bit NAND data bus, but the controller
	 * would support 16 bit, too, so we just warn about this for now
	 */
	if (bus_width == 16)
	{
		LOG_WARNING("LPC3180 only supports 8 bit bus width");
	}
	
	/* inform calling code about selected bus width */
	device->bus_width = bus_width;
	
	if ((address_cycles != 3) && (address_cycles != 4))
	{
		LOG_ERROR("LPC3180 only supports 3 or 4 address cycles, not %i", address_cycles);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}
	
	if ((page_size != 512) && (page_size != 2048))
	{
		LOG_ERROR("LPC3180 only supports 512 or 2048 byte pages, not %i", page_size);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}
	
	/* select MLC controller if none is currently selected */
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_DEBUG("no LPC3180 NAND flash controller selected, using default 'mlc'");
		lpc3180_info->selected_controller = LPC3180_MLC_CONTROLLER;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		u32 mlc_icr_value = 0x0;
		float cycle;
		int twp, twh, trp, treh, trhz, trbwb, tcea;
		
		/* FLASHCLK_CTRL = 0x22 (enable clock for MLC flash controller) */
		target_write_u32(target, 0x400040c8, 0x22);
		
		/* MLC_CEH = 0x0 (Force nCE assert) */
		target_write_u32(target, 0x200b804c, 0x0);
		
		/* MLC_LOCK = 0xa25e (unlock protected registers) */
		target_write_u32(target, 0x200b8044, 0xa25e);
		
		/* MLC_ICR = configuration */
		if (lpc3180_info->sw_write_protection)
			mlc_icr_value |= 0x8;
		if (page_size == 2048)
			mlc_icr_value |= 0x4;
		if (address_cycles == 4)
			mlc_icr_value |= 0x2;
		if (bus_width == 16)
			mlc_icr_value |= 0x1;
		target_write_u32(target, 0x200b8030, mlc_icr_value);
		
		/* calculate NAND controller timings */
		cycle = lpc3180_cycle_time(lpc3180_info);
		
		twp = ((40 / cycle) + 1);
		twh = ((20 / cycle) + 1);
		trp = ((30 / cycle) + 1);
		treh = ((15 / cycle) + 1);
		trhz = ((30 / cycle) + 1);
		trbwb = ((100 / cycle) + 1);
		tcea = ((45 / cycle) + 1);
		
		/* MLC_LOCK = 0xa25e (unlock protected registers) */
		target_write_u32(target, 0x200b8044, 0xa25e);
	
		/* MLC_TIME_REG */
		target_write_u32(target, 0x200b8034, (twp & 0xf) | ((twh & 0xf) << 4) | 
			((trp & 0xf) << 8) | ((treh & 0xf) << 12) | ((trhz & 0x7) << 16) | 
			((trbwb & 0x1f) << 19) | ((tcea & 0x3) << 24)); 

		lpc3180_reset(device);
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		float cycle;
		int r_setup, r_hold, r_width, r_rdy;
		int w_setup, w_hold, w_width, w_rdy;
		
		/* FLASHCLK_CTRL = 0x05 (enable clock for SLC flash controller) */
		target_write_u32(target, 0x400040c8, 0x05);
		
		/* SLC_CFG = 0x (Force nCE assert, ECC enabled, WIDTH = bus_width) */
		target_write_u32(target, 0x20020014, 0x28 | (bus_width == 16) ? 1 : 0);
		
		/* calculate NAND controller timings */
		cycle = lpc3180_cycle_time(lpc3180_info);
		
		r_setup = w_setup = 0;
		r_hold = w_hold = 10 / cycle;
		r_width = 30 / cycle;
		w_width = 40 / cycle;
		r_rdy = w_rdy = 100 / cycle;
		
		/* SLC_TAC: SLC timing arcs register */
		target_write_u32(target, 0x2002002c, (r_setup & 0xf) | ((r_hold & 0xf) << 4) |
			((r_width & 0xf) << 8) | ((r_rdy & 0xf) << 12) |  ((w_setup & 0xf) << 16) |
			((w_hold & 0xf) << 20) | ((w_width & 0xf) << 24) | ((w_rdy & 0xf) << 28)); 
		
		lpc3180_reset(device);
	}
	
	return ERROR_OK;
}

int lpc3180_reset(struct nand_device_s *device)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	}
	else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		/* MLC_CMD = 0xff (reset controller and NAND device) */
		target_write_u32(target, 0x200b8000, 0xff);

		if (!lpc3180_controller_ready(device, 100))
		{
			LOG_ERROR("LPC3180 NAND controller timed out after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		/* SLC_CTRL = 0x6 (ECC_CLEAR, SW_RESET) */
		target_write_u32(target, 0x20020010, 0x6);
		
		if (!lpc3180_controller_ready(device, 100))
		{
			LOG_ERROR("LPC3180 NAND controller timed out after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	}
	
	return ERROR_OK;
}

int lpc3180_command(struct nand_device_s *device, u8 command)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	}
	else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		/* MLC_CMD = command */
		target_write_u32(target, 0x200b8000, command);
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		/* SLC_CMD = command */
		target_write_u32(target, 0x20020008, command);
	}	
	
	return ERROR_OK;
}

int lpc3180_address(struct nand_device_s *device, u8 address)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	}
	else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		/* MLC_ADDR = address */
		target_write_u32(target, 0x200b8004, address);
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		/* SLC_ADDR = address */
		target_write_u32(target, 0x20020004, address);
	}
		
	return ERROR_OK;
}

int lpc3180_write_data(struct nand_device_s *device, u16 data)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	}
	else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		/* MLC_DATA = data */
		target_write_u32(target, 0x200b0000, data);
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		/* SLC_DATA = data */
		target_write_u32(target, 0x20020000, data);
	}
	
	return ERROR_OK;
}

int lpc3180_read_data(struct nand_device_s *device, void *data)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	}
	else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		/* data = MLC_DATA, use sized access */
		if (device->bus_width == 8)
		{
			u8 *data8 = data;
			target_read_u8(target, 0x200b0000, data8);
		}
		else if (device->bus_width == 16)
		{
			u16 *data16 = data;
			target_read_u16(target, 0x200b0000, data16);
		}
		else
		{
			LOG_ERROR("BUG: bus_width neither 8 nor 16 bit");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		u32 data32;

		/* data = SLC_DATA, must use 32-bit access */
		target_read_u32(target, 0x20020000, &data32);
		
		if (device->bus_width == 8)
		{
			u8 *data8 = data;
			*data8 = data32 & 0xff;
		}
		else if (device->bus_width == 16)
		{
			u16 *data16 = data;
			*data16 = data32 & 0xffff;
		}
		else
		{
			LOG_ERROR("BUG: bus_width neither 8 nor 16 bit");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}	
	
	return ERROR_OK;
}

int lpc3180_write_page(struct nand_device_s *device, u32 page, u8 *data, u32 data_size, u8 *oob, u32 oob_size)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	int retval;
	u8 status;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	}
	else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		u8 *page_buffer;
		u8 *oob_buffer;
		int quarter, num_quarters;
		
		if (!data && oob)
		{
			LOG_ERROR("LPC3180 MLC controller can't write OOB data only");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
		
		if (oob && (oob_size > 6))
		{
			LOG_ERROR("LPC3180 MLC controller can't write more than 6 bytes of OOB data");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
		
		if (data_size > device->page_size)
		{
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
		
		/* MLC_CMD = sequential input */
		target_write_u32(target, 0x200b8000, NAND_CMD_SEQIN);

		page_buffer = malloc(512);
		oob_buffer = malloc(6);		

		if (device->page_size == 512)
		{
			/* MLC_ADDR = 0x0 (one column cycle) */
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);
			
			if (device->address_cycles == 4)
				target_write_u32(target, 0x200b8004, (page >> 16) & 0xff);
		}
		else
		{
			/* MLC_ADDR = 0x0 (two column cycles) */
			target_write_u32(target, 0x200b8004, 0x0);
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);
		}
		
		/* when using the MLC controller, we have to treat a large page device
		 * as being made out of four quarters, each the size of a small page device
		 */
		num_quarters = (device->page_size == 2048) ? 4 : 1;
		 
		for (quarter = 0; quarter < num_quarters; quarter++)
		{
			int thisrun_data_size = (data_size > 512) ? 512 : data_size;
			int thisrun_oob_size = (oob_size > 6) ? 6 : oob_size;
			
			memset(page_buffer, 0xff, 512);
			if (data)
			{
				memcpy(page_buffer, data, thisrun_data_size);
				data_size -= thisrun_data_size;
				data += thisrun_data_size;
			}
			
			memset(oob_buffer, 0xff, (device->page_size == 512) ? 6 : 24);
			if (oob)
			{
				memcpy(page_buffer, oob, thisrun_oob_size);
				oob_size -= thisrun_oob_size;
				oob += thisrun_oob_size;
			}
			
			/* write MLC_ECC_ENC_REG to start encode cycle */
			target_write_u32(target, 0x200b8008, 0x0);
			
			target->type->write_memory(target, 0x200a8000, 4, 128, page_buffer + (quarter * 512));
			target->type->write_memory(target, 0x200a8000, 1, 6, oob_buffer + (quarter * 6));
			
			/* write MLC_ECC_AUTO_ENC_REG to start auto encode */
			target_write_u32(target, 0x200b8010, 0x0);
			
			if (!lpc3180_controller_ready(device, 1000))
			{
				LOG_ERROR("timeout while waiting for completion of auto encode cycle");
				return ERROR_NAND_OPERATION_FAILED;
			}
		}
		
		/* MLC_CMD = auto program command */
		target_write_u32(target, 0x200b8000, NAND_CMD_PAGEPROG);
		
		if ((retval = nand_read_status(device, &status)) != ERROR_OK)
		{
			LOG_ERROR("couldn't read status");
			return ERROR_NAND_OPERATION_FAILED;
		}
			
		if (status & NAND_STATUS_FAIL)
		{
			LOG_ERROR("write operation didn't pass, status: 0x%2.2x", status);
			return ERROR_NAND_OPERATION_FAILED;
		}
	
		free(page_buffer);
		free(oob_buffer);
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		return nand_write_page_raw(device, page, data, data_size, oob, oob_size);
	}
	
	return ERROR_OK;
}

int lpc3180_read_page(struct nand_device_s *device, u32 page, u8 *data, u32 data_size, u8 *oob, u32 oob_size)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER)
	{
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	}
	else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
	{
		u8 *page_buffer;
		u8 *oob_buffer;
		u32 page_bytes_done = 0;
		u32 oob_bytes_done = 0;
		u32 mlc_isr;

#if 0
		if (oob && (oob_size > 6))
		{
			LOG_ERROR("LPC3180 MLC controller can't read more than 6 bytes of OOB data");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
#endif
		
		if (data_size > device->page_size)
		{
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
		
		if (device->page_size == 2048)
		{
			page_buffer = malloc(2048);
			oob_buffer = malloc(64);
		}
		else
		{
			page_buffer = malloc(512);
			oob_buffer = malloc(16);
		}
		
		if (!data && oob)
		{
			/* MLC_CMD = Read OOB 
			 * we can use the READOOB command on both small and large page devices,
			 * as the controller translates the 0x50 command to a 0x0 with appropriate
			 * positioning of the serial buffer read pointer
			 */
			target_write_u32(target, 0x200b8000, NAND_CMD_READOOB);
		}
		else
		{
			/* MLC_CMD = Read0 */
			target_write_u32(target, 0x200b8000, NAND_CMD_READ0);
		}
		
		if (device->page_size == 512)
		{
			/* small page device */
			/* MLC_ADDR = 0x0 (one column cycle) */
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);
			
			if (device->address_cycles == 4)
				target_write_u32(target, 0x200b8004, (page >> 16) & 0xff);
		}
		else
		{
			/* large page device */
			/* MLC_ADDR = 0x0 (two column cycles) */
			target_write_u32(target, 0x200b8004, 0x0);
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);
			
			/* MLC_CMD = Read Start */
			target_write_u32(target, 0x200b8000, NAND_CMD_READSTART);
		}
		
		while (page_bytes_done < device->page_size)
		{
			/* MLC_ECC_AUTO_DEC_REG = dummy */
			target_write_u32(target, 0x200b8014, 0xaa55aa55);
			
			if (!lpc3180_controller_ready(device, 1000))
			{
				LOG_ERROR("timeout while waiting for completion of auto decode cycle");
				return ERROR_NAND_OPERATION_FAILED;
			}
		
			target_read_u32(target, 0x200b8048, &mlc_isr);
			
			if (mlc_isr & 0x8)
			{
				if (mlc_isr & 0x40)
				{
					LOG_ERROR("uncorrectable error detected: 0x%2.2x", mlc_isr);
					return ERROR_NAND_OPERATION_FAILED;
				}
				
				LOG_WARNING("%i symbol error detected and corrected", ((mlc_isr & 0x30) >> 4) + 1);
			}
			
			if (data)
			{
				target->type->read_memory(target, 0x200a8000, 4, 128, page_buffer + page_bytes_done);
			}
			
			if (oob)
			{
				target->type->read_memory(target, 0x200a8000, 4, 4, oob_buffer + oob_bytes_done);
			}

			page_bytes_done += 512;
			oob_bytes_done += 16;
		}
		
		if (data)
			memcpy(data, page_buffer, data_size);
		
		if (oob)
			memcpy(oob, oob_buffer, oob_size);
		
		free(page_buffer);
		free(oob_buffer);
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		return nand_read_page_raw(device, page, data, data_size, oob, oob_size);
	}
	
	return ERROR_OK;
}

int lpc3180_controller_ready(struct nand_device_s *device, int timeout)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	u8 status = 0x0;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
			
	do
	{
		if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
		{
			/* Read MLC_ISR, wait for controller to become ready */
			target_read_u8(target, 0x200b8048, &status);
			
			if (status & 2)
				return 1;
		}
		else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
		{
			/* we pretend that the SLC controller is always ready */
			return 1;
		}

		alive_sleep(1);
	} while (timeout-- > 0);
	
	return 0;
}

int lpc3180_nand_ready(struct nand_device_s *device, int timeout)
{
	lpc3180_nand_controller_t *lpc3180_info = device->controller_priv;
	target_t *target = lpc3180_info->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
			
	do
	{
		if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
		{	
			u8 status = 0x0;
			
			/* Read MLC_ISR, wait for NAND flash device to become ready */
			target_read_u8(target, 0x200b8048, &status);
			
			if (status & 1)
				return 1;
		}
		else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
		{
			u32 status = 0x0;
			
			/* Read SLC_STAT and check READY bit */
			target_read_u32(target, 0x20020018, &status);
			
			if (status & 1)
				return 1;
		}
		
		alive_sleep(1);
	} while (timeout-- > 0);
	
	return 0;	
}

int handle_lpc3180_select_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	nand_device_t *device = NULL;
	lpc3180_nand_controller_t *lpc3180_info = NULL;
	char *selected[] = 
	{
		"no", "mlc", "slc"
	};
	
	if ((argc < 1) || (argc > 2))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	device = get_nand_device_by_num(strtoul(args[0], NULL, 0));
	if (!device)
	{
		command_print(cmd_ctx, "nand device '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	lpc3180_info = device->controller_priv;
	
	if (argc == 2)
	{
		if (strcmp(args[1], "mlc") == 0)
		{
			lpc3180_info->selected_controller = LPC3180_MLC_CONTROLLER;
		}
		else if (strcmp(args[1], "slc") == 0)
		{
			lpc3180_info->selected_controller = LPC3180_SLC_CONTROLLER;
		}
		else
		{
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}
	
	command_print(cmd_ctx, "%s controller selected", selected[lpc3180_info->selected_controller]);
	
	return ERROR_OK;
}

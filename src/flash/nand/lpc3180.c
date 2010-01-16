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

#include "imp.h"
#include "lpc3180.h"
#include <target/target.h>


static int lpc3180_reset(struct nand_device *nand);
static int lpc3180_controller_ready(struct nand_device *nand, int timeout);

/* nand device lpc3180 <target#> <oscillator_frequency>
 */
NAND_DEVICE_COMMAND_HANDLER(lpc3180_nand_device_command)
{
	if (CMD_ARGC < 3)
	{
		LOG_WARNING("incomplete 'lpc3180' nand flash configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	struct target *target = get_target(CMD_ARGV[1]);
	if (NULL == target)
	{
		LOG_ERROR("target '%s' not defined", CMD_ARGV[1]);
		return ERROR_NAND_DEVICE_INVALID;
	}

	uint32_t osc_freq;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], osc_freq);

	struct lpc3180_nand_controller *lpc3180_info;
	lpc3180_info = malloc(sizeof(struct lpc3180_nand_controller));
	nand->controller_priv = lpc3180_info;

	lpc3180_info->target = target;
	lpc3180_info->osc_freq = osc_freq;

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

static int lpc3180_pll(int fclkin, uint32_t pll_ctrl)
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

static float lpc3180_cycle_time(struct lpc3180_nand_controller *lpc3180_info)
{
	struct target *target = lpc3180_info->target;
	uint32_t sysclk_ctrl, pwr_ctrl, hclkdiv_ctrl, hclkpll_ctrl;
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

static int lpc3180_init(struct nand_device *nand)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;
	int bus_width = nand->bus_width ? : 8;
	int address_cycles = nand->address_cycles ? : 3;
	int page_size = nand->page_size ? : 512;

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
	nand->bus_width = bus_width;

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
		uint32_t mlc_icr_value = 0x0;
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

		lpc3180_reset(nand);
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

		lpc3180_reset(nand);
	}

	return ERROR_OK;
}

static int lpc3180_reset(struct nand_device *nand)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

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

		if (!lpc3180_controller_ready(nand, 100))
		{
			LOG_ERROR("LPC3180 NAND controller timed out after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	}
	else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
	{
		/* SLC_CTRL = 0x6 (ECC_CLEAR, SW_RESET) */
		target_write_u32(target, 0x20020010, 0x6);

		if (!lpc3180_controller_ready(nand, 100))
		{
			LOG_ERROR("LPC3180 NAND controller timed out after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static int lpc3180_command(struct nand_device *nand, uint8_t command)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

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

static int lpc3180_address(struct nand_device *nand, uint8_t address)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

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

static int lpc3180_write_data(struct nand_device *nand, uint16_t data)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

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

static int lpc3180_read_data(struct nand_device *nand, void *data)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

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
		if (nand->bus_width == 8)
		{
			uint8_t *data8 = data;
			target_read_u8(target, 0x200b0000, data8);
		}
		else if (nand->bus_width == 16)
		{
			uint16_t *data16 = data;
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
		uint32_t data32;

		/* data = SLC_DATA, must use 32-bit access */
		target_read_u32(target, 0x20020000, &data32);

		if (nand->bus_width == 8)
		{
			uint8_t *data8 = data;
			*data8 = data32 & 0xff;
		}
		else if (nand->bus_width == 16)
		{
			uint16_t *data16 = data;
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

static int lpc3180_write_page(struct nand_device *nand, uint32_t page, uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;
	int retval;
	uint8_t status;

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
		uint8_t *page_buffer;
		uint8_t *oob_buffer;
		int quarter, num_quarters;

		if (!data && oob)
		{
			LOG_ERROR("LPC3180 MLC controller can't write OOB data only");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (oob && (oob_size > 24))
		{
			LOG_ERROR("LPC3180 MLC controller can't write more "
				"than 6 bytes for each quarter's OOB data");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (data_size > (uint32_t)nand->page_size)
		{
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		/* MLC_CMD = sequential input */
		target_write_u32(target, 0x200b8000, NAND_CMD_SEQIN);

		page_buffer = malloc(512);
		oob_buffer = malloc(6);

		if (nand->page_size == 512)
		{
			/* MLC_ADDR = 0x0 (one column cycle) */
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);

			if (nand->address_cycles == 4)
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
		num_quarters = (nand->page_size == 2048) ? 4 : 1;

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

			memset(oob_buffer, 0xff, 6);
			if (oob)
			{
				memcpy(oob_buffer, oob, thisrun_oob_size);
				oob_size -= thisrun_oob_size;
				oob += thisrun_oob_size;
			}

			/* write MLC_ECC_ENC_REG to start encode cycle */
			target_write_u32(target, 0x200b8008, 0x0);

			target_write_memory(target, 0x200a8000,
					4, 128, page_buffer);
			target_write_memory(target, 0x200a8000,
					1, 6, oob_buffer);

			/* write MLC_ECC_AUTO_ENC_REG to start auto encode */
			target_write_u32(target, 0x200b8010, 0x0);

			if (!lpc3180_controller_ready(nand, 1000))
			{
				LOG_ERROR("timeout while waiting for completion of auto encode cycle");
				return ERROR_NAND_OPERATION_FAILED;
			}
		}

		/* MLC_CMD = auto program command */
		target_write_u32(target, 0x200b8000, NAND_CMD_PAGEPROG);

		if ((retval = nand_read_status(nand, &status)) != ERROR_OK)
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
		return nand_write_page_raw(nand, page, data, data_size, oob, oob_size);
	}

	return ERROR_OK;
}

static int lpc3180_read_page(struct nand_device *nand, uint32_t page, uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

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
		uint8_t *page_buffer;
		uint8_t *oob_buffer;
		uint32_t page_bytes_done = 0;
		uint32_t oob_bytes_done = 0;
		uint32_t mlc_isr;

#if 0
		if (oob && (oob_size > 6))
		{
			LOG_ERROR("LPC3180 MLC controller can't read more than 6 bytes of OOB data");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
#endif

		if (data_size > (uint32_t)nand->page_size)
		{
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (nand->page_size == 2048)
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

		if (nand->page_size == 512)
		{
			/* small page device */
			/* MLC_ADDR = 0x0 (one column cycle) */
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);

			if (nand->address_cycles == 4)
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

		while (page_bytes_done < (uint32_t)nand->page_size)
		{
			/* MLC_ECC_AUTO_DEC_REG = dummy */
			target_write_u32(target, 0x200b8014, 0xaa55aa55);

			if (!lpc3180_controller_ready(nand, 1000))
			{
				LOG_ERROR("timeout while waiting for completion of auto decode cycle");
				return ERROR_NAND_OPERATION_FAILED;
			}

			target_read_u32(target, 0x200b8048, &mlc_isr);

			if (mlc_isr & 0x8)
			{
				if (mlc_isr & 0x40)
				{
					LOG_ERROR("uncorrectable error detected: 0x%2.2x", (unsigned)mlc_isr);
					return ERROR_NAND_OPERATION_FAILED;
				}

				LOG_WARNING("%i symbol error detected and corrected", ((int)(((mlc_isr & 0x30) >> 4) + 1)));
			}

			if (data)
			{
				target_read_memory(target, 0x200a8000, 4, 128, page_buffer + page_bytes_done);
			}

			if (oob)
			{
				target_read_memory(target, 0x200a8000, 4, 4, oob_buffer + oob_bytes_done);
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
		return nand_read_page_raw(nand, page, data, data_size, oob, oob_size);
	}

	return ERROR_OK;
}

static int lpc3180_controller_ready(struct nand_device *nand, int timeout)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("lpc3180_controller_ready count start=%d", timeout);

	do
	{
		if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
		{
			uint8_t status;

			/* Read MLC_ISR, wait for controller to become ready */
			target_read_u8(target, 0x200b8048, &status);

			if (status & 2) {
				LOG_DEBUG("lpc3180_controller_ready count=%d",
						timeout);
				return 1;
			}
		}
		else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
		{
			uint32_t status;

			/* Read SLC_STAT and check READY bit */
			target_read_u32(target, 0x20020018, &status);

			if (status & 1) {
				LOG_DEBUG("lpc3180_controller_ready count=%d",
						timeout);
				return 1;
			}
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

static int lpc3180_nand_ready(struct nand_device *nand, int timeout)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = lpc3180_info->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("lpc3180_nand_ready count start=%d", timeout);

	do
	{
		if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
		{
			uint8_t status = 0x0;

			/* Read MLC_ISR, wait for NAND flash device to become ready */
			target_read_u8(target, 0x200b8048, &status);

			if (status & 1) {
				LOG_DEBUG("lpc3180_nand_ready count end=%d",
						timeout);
				return 1;
			}
		}
		else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER)
		{
			uint32_t status = 0x0;

			/* Read SLC_STAT and check READY bit */
			target_read_u32(target, 0x20020018, &status);

			if (status & 1) {
				LOG_DEBUG("lpc3180_nand_ready count end=%d",
						timeout);
				return 1;
			}
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

COMMAND_HANDLER(handle_lpc3180_select_command)
{
	struct lpc3180_nand_controller *lpc3180_info = NULL;
	char *selected[] =
	{
		"no", "mlc", "slc"
	};

	if ((CMD_ARGC < 1) || (CMD_ARGC > 2))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	unsigned num;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num);
	struct nand_device *nand = get_nand_device_by_num(num);
	if (!nand)
	{
		command_print(CMD_CTX, "nand device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	lpc3180_info = nand->controller_priv;

	if (CMD_ARGC == 2)
	{
		if (strcmp(CMD_ARGV[1], "mlc") == 0)
		{
			lpc3180_info->selected_controller = LPC3180_MLC_CONTROLLER;
		}
		else if (strcmp(CMD_ARGV[1], "slc") == 0)
		{
			lpc3180_info->selected_controller = LPC3180_SLC_CONTROLLER;
		}
		else
		{
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	command_print(CMD_CTX, "%s controller selected", selected[lpc3180_info->selected_controller]);

	return ERROR_OK;
}

static const struct command_registration lpc3180_exec_command_handlers[] = {
	{
		.name = "select",
		.handler = handle_lpc3180_select_command,
		.mode = COMMAND_EXEC,
		.help = "select MLC or SLC controller (default is MLC)",
		.usage = "bank_id ['mlc'|'slc']",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration lpc3180_command_handler[] = {
	{
		.name = "lpc3180",
		.mode = COMMAND_ANY,
		.help = "LPC3180 NAND flash controller commands",
		.chain = lpc3180_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct nand_flash_controller lpc3180_nand_controller = {
	.name = "lpc3180",
	.commands = lpc3180_command_handler,
	.nand_device_command = lpc3180_nand_device_command,
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

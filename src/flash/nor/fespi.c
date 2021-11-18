/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 *   Modified by Megan Wachs <megan@sifive.com> from the original stmsmi.c *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/* The Freedom E SPI controller is a SPI bus controller
 * specifically designed for SPI Flash Memories on Freedom E platforms.
 *
 * Two working modes are available:
 * - SW mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus. Writes are only possible in this mode.
 * - HW mode: Memory content is directly
 *   accessible in CPU memory space. CPU can read and execute memory content.
 */

/* ATTENTION:
 * To have flash memory mapped in CPU memory space, the controller
 * must have "HW mode" enabled.
 * 1) The command "reset init" has to initialize the controller and put
 *    it in HW mode (this is actually the default out of reset for Freedom E systems).
 * 2) every command in this file have to return to prompt in HW mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "target/riscv/riscv.h"

/* Register offsets */

#define FESPI_REG_SCKDIV          0x00
#define FESPI_REG_SCKMODE         0x04
#define FESPI_REG_CSID            0x10
#define FESPI_REG_CSDEF           0x14
#define FESPI_REG_CSMODE          0x18

#define FESPI_REG_DCSSCK          0x28
#define FESPI_REG_DSCKCS          0x2a
#define FESPI_REG_DINTERCS        0x2c
#define FESPI_REG_DINTERXFR       0x2e

#define FESPI_REG_FMT             0x40
#define FESPI_REG_TXFIFO          0x48
#define FESPI_REG_RXFIFO          0x4c
#define FESPI_REG_TXCTRL          0x50
#define FESPI_REG_RXCTRL          0x54

#define FESPI_REG_FCTRL           0x60
#define FESPI_REG_FFMT            0x64

#define FESPI_REG_IE              0x70
#define FESPI_REG_IP              0x74

/* Fields */

#define FESPI_SCK_POL             0x1
#define FESPI_SCK_PHA             0x2

#define FESPI_FMT_PROTO(x)        ((x) & 0x3)
#define FESPI_FMT_ENDIAN(x)       (((x) & 0x1) << 2)
#define FESPI_FMT_DIR(x)          (((x) & 0x1) << 3)
#define FESPI_FMT_LEN(x)          (((x) & 0xf) << 16)

/* TXCTRL register */
#define FESPI_TXWM(x)             ((x) & 0xffff)
/* RXCTRL register */
#define FESPI_RXWM(x)             ((x) & 0xffff)

#define FESPI_IP_TXWM             0x1
#define FESPI_IP_RXWM             0x2

#define FESPI_FCTRL_EN            0x1

#define FESPI_INSN_CMD_EN         0x1
#define FESPI_INSN_ADDR_LEN(x)    (((x) & 0x7) << 1)
#define FESPI_INSN_PAD_CNT(x)     (((x) & 0xf) << 4)
#define FESPI_INSN_CMD_PROTO(x)   (((x) & 0x3) << 8)
#define FESPI_INSN_ADDR_PROTO(x)  (((x) & 0x3) << 10)
#define FESPI_INSN_DATA_PROTO(x)  (((x) & 0x3) << 12)
#define FESPI_INSN_CMD_CODE(x)    (((x) & 0xff) << 16)
#define FESPI_INSN_PAD_CODE(x)    (((x) & 0xff) << 24)

/* Values */

#define FESPI_CSMODE_AUTO         0
#define FESPI_CSMODE_HOLD         2
#define FESPI_CSMODE_OFF          3

#define FESPI_DIR_RX              0
#define FESPI_DIR_TX              1

#define FESPI_PROTO_S             0
#define FESPI_PROTO_D             1
#define FESPI_PROTO_Q             2

#define FESPI_ENDIAN_MSB          0
#define FESPI_ENDIAN_LSB          1


/* Timeout in ms */
#define FESPI_CMD_TIMEOUT   (100)
#define FESPI_PROBE_TIMEOUT (100)
#define FESPI_MAX_TIMEOUT  (3000)


struct fespi_flash_bank {
	bool probed;
	target_addr_t ctrl_base;
	const struct flash_device *dev;
};

struct fespi_target {
	char *name;
	uint32_t tap_idcode;
	uint32_t ctrl_base;
};

/* TODO !!! What is the right naming convention here? */
static const struct fespi_target target_devices[] = {
	/* name,   tap_idcode, ctrl_base */
	{ "Freedom E310-G000 SPI Flash", 0x10e31913, 0x10014000 },
	{ "Freedom E310-G002 SPI Flash", 0x20000913, 0x10014000 },
	{ NULL, 0, 0 }
};

FLASH_BANK_COMMAND_HANDLER(fespi_flash_bank_command)
{
	struct fespi_flash_bank *fespi_info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	fespi_info = malloc(sizeof(struct fespi_flash_bank));
	if (!fespi_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = fespi_info;
	fespi_info->probed = false;
	fespi_info->ctrl_base = 0;
	if (CMD_ARGC >= 7) {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[6], fespi_info->ctrl_base);
		LOG_DEBUG("ASSUMING FESPI device at ctrl_base = " TARGET_ADDR_FMT,
				fespi_info->ctrl_base);
	}

	return ERROR_OK;
}

static int fespi_read_reg(struct flash_bank *bank, uint32_t *value, target_addr_t address)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;

	int result = target_read_u32(target, fespi_info->ctrl_base + address, value);
	if (result != ERROR_OK) {
		LOG_ERROR("fespi_read_reg() error at " TARGET_ADDR_FMT,
				fespi_info->ctrl_base + address);
		return result;
	}
	return ERROR_OK;
}

static int fespi_write_reg(struct flash_bank *bank, target_addr_t address, uint32_t value)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;

	int result = target_write_u32(target, fespi_info->ctrl_base + address, value);
	if (result != ERROR_OK) {
		LOG_ERROR("fespi_write_reg() error writing 0x%" PRIx32 " to " TARGET_ADDR_FMT,
				value, fespi_info->ctrl_base + address);
		return result;
	}
	return ERROR_OK;
}

static int fespi_disable_hw_mode(struct flash_bank *bank)
{
	uint32_t fctrl;
	if (fespi_read_reg(bank, &fctrl, FESPI_REG_FCTRL) != ERROR_OK)
		return ERROR_FAIL;
	return fespi_write_reg(bank, FESPI_REG_FCTRL, fctrl & ~FESPI_FCTRL_EN);
}

static int fespi_enable_hw_mode(struct flash_bank *bank)
{
	uint32_t fctrl;
	if (fespi_read_reg(bank, &fctrl, FESPI_REG_FCTRL) != ERROR_OK)
		return ERROR_FAIL;
	return fespi_write_reg(bank, FESPI_REG_FCTRL, fctrl | FESPI_FCTRL_EN);
}

static int fespi_set_dir(struct flash_bank *bank, bool dir)
{
	uint32_t fmt;
	if (fespi_read_reg(bank, &fmt, FESPI_REG_FMT) != ERROR_OK)
		return ERROR_FAIL;

	return fespi_write_reg(bank, FESPI_REG_FMT,
			(fmt & ~(FESPI_FMT_DIR(0xFFFFFFFF))) | FESPI_FMT_DIR(dir));
}

static int fespi_txwm_wait(struct flash_bank *bank)
{
	int64_t start = timeval_ms();

	while (1) {
		uint32_t ip;
		if (fespi_read_reg(bank, &ip, FESPI_REG_IP) != ERROR_OK)
			return ERROR_FAIL;
		if (ip & FESPI_IP_TXWM)
			break;
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("ip.txwm didn't get set.");
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static int fespi_tx(struct flash_bank *bank, uint8_t in)
{
	int64_t start = timeval_ms();

	while (1) {
		uint32_t txfifo;
		if (fespi_read_reg(bank, &txfifo, FESPI_REG_TXFIFO) != ERROR_OK)
			return ERROR_FAIL;
		if (!(txfifo >> 31))
			break;
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("txfifo stayed negative.");
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return fespi_write_reg(bank, FESPI_REG_TXFIFO, in);
}

static int fespi_rx(struct flash_bank *bank, uint8_t *out)
{
	int64_t start = timeval_ms();
	uint32_t value;

	while (1) {
		if (fespi_read_reg(bank, &value, FESPI_REG_RXFIFO) != ERROR_OK)
			return ERROR_FAIL;
		if (!(value >> 31))
			break;
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("rxfifo didn't go positive (value=0x%" PRIx32 ").", value);
			return ERROR_TARGET_TIMEOUT;
		}
	}

	if (out)
		*out = value & 0xff;

	return ERROR_OK;
}

/* TODO!!! Why don't we need to call this after writing? */
static int fespi_wip(struct flash_bank *bank, int timeout)
{
	int64_t endtime;

	fespi_set_dir(bank, FESPI_DIR_RX);

	if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;
	endtime = timeval_ms() + timeout;

	fespi_tx(bank, SPIFLASH_READ_STATUS);
	if (fespi_rx(bank, NULL) != ERROR_OK)
		return ERROR_FAIL;

	do {
		alive_sleep(1);

		fespi_tx(bank, 0);
		uint8_t rx;
		if (fespi_rx(bank, &rx) != ERROR_OK)
			return ERROR_FAIL;
		if ((rx & SPIFLASH_BSY_BIT) == 0) {
			if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO) != ERROR_OK)
				return ERROR_FAIL;
			fespi_set_dir(bank, FESPI_DIR_TX);
			return ERROR_OK;
		}
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

static int fespi_erase_sector(struct flash_bank *bank, int sector)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	int retval;

	retval = fespi_tx(bank, SPIFLASH_WRITE_ENABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = fespi_txwm_wait(bank);
	if (retval != ERROR_OK)
		return retval;

	if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;
	retval = fespi_tx(bank, fespi_info->dev->erase_cmd);
	if (retval != ERROR_OK)
		return retval;
	sector = bank->sectors[sector].offset;
	if (bank->size > 0x1000000) {
		retval = fespi_tx(bank, sector >> 24);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = fespi_tx(bank, sector >> 16);
	if (retval != ERROR_OK)
		return retval;
	retval = fespi_tx(bank, sector >> 8);
	if (retval != ERROR_OK)
		return retval;
	retval = fespi_tx(bank, sector);
	if (retval != ERROR_OK)
		return retval;
	retval = fespi_txwm_wait(bank);
	if (retval != ERROR_OK)
		return retval;
	if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	retval = fespi_wip(bank, FESPI_MAX_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int fespi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: from sector %u to sector %u", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(fespi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	if (fespi_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	if (fespi_write_reg(bank, FESPI_REG_TXCTRL, FESPI_TXWM(1)) != ERROR_OK)
		return ERROR_FAIL;
	retval = fespi_txwm_wait(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("WM Didn't go high before attempting.");
		return retval;
	}

	/* Disable Hardware accesses*/
	if (fespi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	/* poll WIP */
	retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto done;

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = fespi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			goto done;
		keep_alive();
	}

	/* Switch to HW mode before return to prompt */
done:
	if (fespi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;
	return retval;
}

static int fespi_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int slow_fespi_write_buffer(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t len)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ii;

	/* TODO!!! assert that len < page size */

	if (fespi_tx(bank, SPIFLASH_WRITE_ENABLE) != ERROR_OK)
		return ERROR_FAIL;
	if (fespi_txwm_wait(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	if (fespi_tx(bank, fespi_info->dev->pprog_cmd) != ERROR_OK)
		return ERROR_FAIL;

	if (bank->size > 0x1000000 && fespi_tx(bank, offset >> 24) != ERROR_OK)
		return ERROR_FAIL;
	if (fespi_tx(bank, offset >> 16) != ERROR_OK)
		return ERROR_FAIL;
	if (fespi_tx(bank, offset >> 8) != ERROR_OK)
		return ERROR_FAIL;
	if (fespi_tx(bank, offset) != ERROR_OK)
		return ERROR_FAIL;

	for (ii = 0; ii < len; ii++) {
		if (fespi_tx(bank, buffer[ii]) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (fespi_txwm_wait(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	keep_alive();

	return ERROR_OK;
}

static const uint8_t riscv32_bin[] = {
#include "../../../contrib/loaders/flash/fespi/riscv32_fespi.inc"
};

static const uint8_t riscv64_bin[] = {
#include "../../../contrib/loaders/flash/fespi/riscv64_fespi.inc"
};

static int fespi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t cur_count, page_size;
	int retval = ERROR_OK;

	LOG_DEBUG("bank->size=0x%x offset=0x%08" PRIx32 " count=0x%08" PRIx32,
			bank->size, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > fespi_info->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded.");
		count = fespi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
					(bank->sectors[sector].offset + bank->sectors[sector].size))
				&& ((offset + count - 1) >= bank->sectors[sector].offset)
				&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	unsigned int xlen = riscv_xlen(target);
	struct working_area *algorithm_wa = NULL;
	struct working_area *data_wa = NULL;
	const uint8_t *bin;
	size_t bin_size;
	if (xlen == 32) {
		bin = riscv32_bin;
		bin_size = sizeof(riscv32_bin);
	} else {
		bin = riscv64_bin;
		bin_size = sizeof(riscv64_bin);
	}

	unsigned data_wa_size = 0;
	if (target_alloc_working_area(target, bin_size, &algorithm_wa) == ERROR_OK) {
		retval = target_write_buffer(target, algorithm_wa->address,
				bin_size, bin);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
					algorithm_wa->address, retval);
			target_free_working_area(target, algorithm_wa);
			algorithm_wa = NULL;

		} else {
			data_wa_size = MIN(target_get_working_area_avail(target), count);
			if (data_wa_size < 128) {
				LOG_WARNING("Couldn't allocate data working area.");
				target_free_working_area(target, algorithm_wa);
				algorithm_wa = NULL;
			} else if (target_alloc_working_area(target, data_wa_size, &data_wa) != ERROR_OK) {
				target_free_working_area(target, algorithm_wa);
				algorithm_wa = NULL;
			}
		}
	} else {
		LOG_WARNING("Couldn't allocate %zd-byte working area.", bin_size);
		algorithm_wa = NULL;
	}

	/* If no valid page_size, use reasonable default. */
	page_size = fespi_info->dev->pagesize ?
		fespi_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	if (algorithm_wa) {
		struct reg_param reg_params[6];
		init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
		init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
		init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
		init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);
		init_reg_param(&reg_params[4], "a4", xlen, PARAM_OUT);
		init_reg_param(&reg_params[5], "a5", xlen, PARAM_OUT);

		while (count > 0) {
			cur_count = MIN(count, data_wa_size);
			buf_set_u64(reg_params[0].value, 0, xlen, fespi_info->ctrl_base);
			buf_set_u64(reg_params[1].value, 0, xlen, page_size);
			buf_set_u64(reg_params[2].value, 0, xlen, data_wa->address);
			buf_set_u64(reg_params[3].value, 0, xlen, offset);
			buf_set_u64(reg_params[4].value, 0, xlen, cur_count);
			buf_set_u64(reg_params[5].value, 0, xlen,
					fespi_info->dev->pprog_cmd | (bank->size > 0x1000000 ? 0x100 : 0));

			retval = target_write_buffer(target, data_wa->address, cur_count,
					buffer);
			if (retval != ERROR_OK) {
				LOG_DEBUG("Failed to write %d bytes to " TARGET_ADDR_FMT ": %d",
						cur_count, data_wa->address, retval);
				goto err;
			}

			LOG_DEBUG("write(ctrl_base=0x%" TARGET_PRIxADDR ", page_size=0x%x, "
					"address=0x%" TARGET_PRIxADDR ", offset=0x%" PRIx32
					", count=0x%" PRIx32 "), buffer=%02x %02x %02x %02x %02x %02x ..." PRIx32,
					fespi_info->ctrl_base, page_size, data_wa->address, offset, cur_count,
					buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
			retval = target_run_algorithm(target, 0, NULL,
					ARRAY_SIZE(reg_params), reg_params,
					algorithm_wa->address, 0, cur_count * 2, NULL);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
						algorithm_wa->address, retval);
				goto err;
			}

			uint64_t algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
			if (algorithm_result != 0) {
				LOG_ERROR("Algorithm returned error %" PRId64, algorithm_result);
				retval = ERROR_FAIL;
				goto err;
			}

			buffer += cur_count;
			offset += cur_count;
			count -= cur_count;
		}

		target_free_working_area(target, data_wa);
		target_free_working_area(target, algorithm_wa);

	} else {
		fespi_txwm_wait(bank);

		/* Disable Hardware accesses*/
		if (fespi_disable_hw_mode(bank) != ERROR_OK)
			return ERROR_FAIL;

		/* poll WIP */
		retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
		if (retval != ERROR_OK)
			goto err;

		uint32_t page_offset = offset % page_size;
		/* central part, aligned words */
		while (count > 0) {
			/* clip block at page boundary */
			if (page_offset + count > page_size)
				cur_count = page_size - page_offset;
			else
				cur_count = count;

			retval = slow_fespi_write_buffer(bank, buffer, offset, cur_count);
			if (retval != ERROR_OK)
				goto err;

			page_offset = 0;
			buffer += cur_count;
			offset += cur_count;
			count -= cur_count;
		}

		/* Switch to HW mode before return to prompt */
		if (fespi_enable_hw_mode(bank) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;

err:
	target_free_working_area(target, data_wa);
	target_free_working_area(target, algorithm_wa);

	/* Switch to HW mode before return to prompt */
	if (fespi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int fespi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	fespi_txwm_wait(bank);

	/* poll WIP */
	retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	fespi_set_dir(bank, FESPI_DIR_RX);

	/* Send SPI command "read ID" */
	if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	fespi_tx(bank, SPIFLASH_READ_ID);
	/* Send dummy bytes to actually read the ID.*/
	fespi_tx(bank, 0);
	fespi_tx(bank, 0);
	fespi_tx(bank, 0);

	/* read ID from Receive Register */
	*id = 0;
	if (fespi_rx(bank, NULL) != ERROR_OK)
		return ERROR_FAIL;
	uint8_t rx;
	if (fespi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id = rx;
	if (fespi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 8);
	if (fespi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 16);

	if (fespi_write_reg(bank, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	fespi_set_dir(bank, FESPI_DIR_TX);

	return ERROR_OK;
}

static int fespi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	const struct fespi_target *target_device;
	int retval;
	uint32_t sectorsize;

	if (fespi_info->probed)
		free(bank->sectors);
	fespi_info->probed = false;

	if (fespi_info->ctrl_base == 0) {
		for (target_device = target_devices ; target_device->name ; ++target_device)
			if (target_device->tap_idcode == target->tap->idcode)
				break;

		if (!target_device->name) {
			LOG_ERROR("Device ID 0x%" PRIx32 " is not known as FESPI capable",
					target->tap->idcode);
			return ERROR_FAIL;
		}

		fespi_info->ctrl_base = target_device->ctrl_base;

		LOG_DEBUG("Valid FESPI on device %s at address " TARGET_ADDR_FMT,
				target_device->name, bank->base);

	} else {
	  LOG_DEBUG("Assuming FESPI as specified at address " TARGET_ADDR_FMT
			  " with ctrl at " TARGET_ADDR_FMT, fespi_info->ctrl_base,
			  bank->base);
	}

	/* read and decode flash ID; returns in SW mode */
	if (fespi_write_reg(bank, FESPI_REG_TXCTRL, FESPI_TXWM(1)) != ERROR_OK)
		return ERROR_FAIL;
	fespi_set_dir(bank, FESPI_DIR_TX);

	/* Disable Hardware accesses*/
	if (fespi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	retval = fespi_read_flash_id(bank, &id);

	if (fespi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;
	if (retval != ERROR_OK)
		return retval;

	fespi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			fespi_info->dev = p;
			break;
		}

	if (!fespi_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
			fespi_info->dev->name, fespi_info->dev->device_id);

	/* Set correct size value */
	bank->size = fespi_info->dev->size_in_bytes;

	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = fespi_info->dev->sectorsize ?
		fespi_info->dev->sectorsize : fespi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = fespi_info->dev->size_in_bytes / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	fespi_info->probed = true;
	return ERROR_OK;
}

static int fespi_auto_probe(struct flash_bank *bank)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	if (fespi_info->probed)
		return ERROR_OK;
	return fespi_probe(bank);
}

static int fespi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_fespi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;

	if (!(fespi_info->probed)) {
		command_print_sameline(cmd, "\nFESPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "\nFESPI flash information:\n"
			"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
			fespi_info->dev->name, fespi_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver fespi_flash = {
	.name = "fespi",
	.flash_bank_command = fespi_flash_bank_command,
	.erase = fespi_erase,
	.protect = fespi_protect,
	.write = fespi_write,
	.read = default_flash_read,
	.probe = fespi_probe,
	.auto_probe = fespi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = fespi_protect_check,
	.info = get_fespi_info,
	.free_driver_priv = default_flash_free_driver_priv
};

/***************************************************************************
 *   Copyright (C) 2015 Robert Jordens <jordens@gmail.com>                 *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <jtag/jtag.h>
#include <flash/nor/spi.h>
#include <helper/time_support.h>

#define JTAGSPI_MAX_TIMEOUT 3000


struct jtagspi_flash_bank {
	struct jtag_tap *tap;
	struct flash_device dev;
	char devname[32];
	bool probed;
	bool always_4byte;			/* use always 4-byte address except for basic read 0x03 */
	uint32_t ir;
	unsigned int addr_len;		/* address length in bytes */
};

FLASH_BANK_COMMAND_HANDLER(jtagspi_flash_bank_command)
{
	struct jtagspi_flash_bank *info;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	info = malloc(sizeof(struct jtagspi_flash_bank));
	if (!info) {
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	bank->sectors = NULL;
	bank->driver_priv = info;

	info->tap = NULL;
	info->probed = false;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->ir);

	return ERROR_OK;
}

static void jtagspi_set_ir(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct scan_field field;
	uint8_t buf[4] = { 0 };

	LOG_DEBUG("loading jtagspi ir");
	buf_set_u32(buf, 0, info->tap->ir_length, info->ir);
	field.num_bits = info->tap->ir_length;
	field.out_value = buf;
	field.in_value = NULL;
	jtag_add_ir_scan(info->tap, &field, TAP_IDLE);
}

static void flip_u8(const uint8_t *in, uint8_t *out, unsigned int len)
{
	for (unsigned int i = 0; i < len; i++)
		out[i] = flip_u32(in[i], 8);
}

static int jtagspi_cmd(struct flash_bank *bank, uint8_t cmd,
		uint8_t *write_buffer, unsigned int write_len, uint8_t *data_buffer, int data_len)
{
	assert(write_buffer || write_len == 0);
	assert(data_buffer || data_len == 0);

	struct scan_field fields[6];

	LOG_DEBUG("cmd=0x%02x write_len=%d data_len=%d", cmd, write_len, data_len);

	/* negative data_len == read operation */
	const bool is_read = (data_len < 0);
	if (is_read)
		data_len = -data_len;

	int n = 0;
	const uint8_t marker = 1;
	fields[n].num_bits = 1;
	fields[n].out_value = &marker;
	fields[n].in_value = NULL;
	n++;

	/* transfer length = cmd + address + read/write,
	 * -1 due to the counter implementation */
	uint8_t xfer_bits[4];
	h_u32_to_be(xfer_bits, ((sizeof(cmd) + write_len + data_len) * CHAR_BIT) - 1);
	flip_u8(xfer_bits, xfer_bits, sizeof(xfer_bits));
	fields[n].num_bits = sizeof(xfer_bits) * CHAR_BIT;
	fields[n].out_value = xfer_bits;
	fields[n].in_value = NULL;
	n++;

	flip_u8(&cmd, &cmd, sizeof(cmd));
	fields[n].num_bits = sizeof(cmd) * CHAR_BIT;
	fields[n].out_value = &cmd;
	fields[n].in_value = NULL;
	n++;

	if (write_len) {
		flip_u8(write_buffer, write_buffer, write_len);
		fields[n].num_bits = write_len * CHAR_BIT;
		fields[n].out_value = write_buffer;
		fields[n].in_value = NULL;
		n++;
	}

	if (data_len > 0) {
		if (is_read) {
			fields[n].num_bits = jtag_tap_count_enabled();
			fields[n].out_value = NULL;
			fields[n].in_value = NULL;
			n++;

			fields[n].out_value = NULL;
			fields[n].in_value = data_buffer;
		} else {
			flip_u8(data_buffer, data_buffer, data_len);
			fields[n].out_value = data_buffer;
			fields[n].in_value = NULL;
		}
		fields[n].num_bits = data_len * CHAR_BIT;
		n++;
	}

	jtagspi_set_ir(bank);
	/* passing from an IR scan to SHIFT-DR clears BYPASS registers */
	struct jtagspi_flash_bank *info = bank->driver_priv;
	jtag_add_dr_scan(info->tap, n, fields, TAP_IDLE);
	int retval = jtag_execute_queue();

	if (is_read)
		flip_u8(data_buffer, data_buffer, data_len);
	return retval;
}

COMMAND_HANDLER(jtagspi_handle_set)
{
	struct flash_bank *bank = NULL;
	struct jtagspi_flash_bank *info = NULL;
	struct flash_sector *sectors = NULL;
	uint32_t temp;
	unsigned int index = 1;
	int retval;

	LOG_DEBUG("%s", __func__);

	/* there are 6 mandatory arguments:
	 * devname, size_in_bytes, pagesize, read_cmd, unused, pprog_cmd */
	if (index + 6 > CMD_ARGC) {
		command_print(CMD, "jtagspi: not enough arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;
	info = bank->driver_priv;

	/* invalidate all old info */
	if (info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		if (bank->sectors)
			free(bank->sectors);
		bank->sectors = NULL;
		info->always_4byte = false;
		info->probed = false;
	}
	memset(&info->dev, 0, sizeof(info->dev));

	strncpy(info->devname, CMD_ARGV[index++], sizeof(info->devname) - 1);
	info->devname[sizeof(info->devname) - 1] = '\0';

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
	info->dev.size_in_bytes = temp;
	if ((temp & (temp - 1)) || (temp < (1UL << 8))) {
		command_print(CMD, "jtagspi: device size must be 2^n with n >= 8");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
	info->dev.pagesize = temp;
	if (info->dev.pagesize == 0)
		info->dev.pagesize = SPIFLASH_DEF_PAGESIZE;
	if ((temp & (temp - 1)) || (temp > info->dev.size_in_bytes)) {
		command_print(CMD, "jtagspi: page size must be 2^n and <= device size");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.read_cmd);
	if ((info->dev.read_cmd != 0x03) &&
		(info->dev.read_cmd != 0x13)) {
		command_print(CMD, "jtagspi: only 0x03/0x13 READ allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.qread_cmd);

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.pprog_cmd);
	if ((info->dev.pprog_cmd != 0x02) &&
		(info->dev.pprog_cmd != 0x12)) {
		command_print(CMD, "jtagspi: only 0x02/0x12 PPRG allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* remaining params are optional */
	if (index < CMD_ARGC)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.chip_erase_cmd);
	else
		info->dev.chip_erase_cmd = 0x00;

	if (index < CMD_ARGC) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
		info->dev.sectorsize = temp;
		if ((info->dev.sectorsize > info->dev.size_in_bytes) ||
			(info->dev.sectorsize < info->dev.pagesize) || (temp & (temp - 1))) {
			command_print(CMD, "jtagspi: sector size must be 2^n and <= device size");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (index < CMD_ARGC)
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.erase_cmd);
		else {
			command_print(CMD, "jtagspi: erase command missing");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else {
		/* no sector size / sector erase cmd given, treat whole bank as a single sector */
		info->dev.erase_cmd = 0x00;
		info->dev.sectorsize = info->dev.size_in_bytes;
	}

	if (index < CMD_ARGC) {
		command_print(CMD, "jtagspi: extra arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* set correct size value */
	bank->size = info->dev.size_in_bytes;

	/* calculate address length in bytes */
	if (bank->size <= (1UL << 8))
		info->addr_len = 1;
	else if (bank->size <= (1UL << 16))
		info->addr_len = 2;
	else if (bank->size <= (1UL << 24))
		info->addr_len = 3;
	else {
		info->addr_len = 4;
		LOG_WARNING("4-byte addresses needed, might need extra command to enable");
	}

	/* create and fill sectors array */
	bank->num_sectors =
		info->dev.size_in_bytes / info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("Not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * (info->dev.sectorsize);
		sectors[sector].size = info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	info->dev.name = info->devname;
	if (info->dev.size_in_bytes / 4096)
		LOG_INFO("flash \'%s\' id = unknown\nflash size = %" PRIu32 " kbytes",
			info->dev.name, info->dev.size_in_bytes / 1024);
	else
		LOG_INFO("flash \'%s\' id = unknown\nflash size = %" PRIu32 " bytes",
			info->dev.name, info->dev.size_in_bytes);
	info->probed = true;

	return ERROR_OK;
}

COMMAND_HANDLER(jtagspi_handle_cmd)
{
	struct flash_bank *bank;
	unsigned int index = 1;
	const int max = 21;
	uint8_t num_write, num_read, write_buffer[max], read_buffer[1 << CHAR_BIT];
	uint8_t data, *ptr;
	char temp[4], output[(2 + max + (1 << CHAR_BIT)) * 3 + 8];
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 3) {
		command_print(CMD, "jtagspi: not enough arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	num_write = CMD_ARGC - 2;
	if (num_write > max) {
		LOG_ERROR("at most %d bytes may be send", max);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], num_read);

	snprintf(output, sizeof(output), "spi: ");
	for (ptr = &write_buffer[0] ; index < CMD_ARGC; index++) {
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index], data);
		*ptr++ = data;
		snprintf(temp, sizeof(temp), "%02" PRIx8 " ", data);
		strncat(output, temp, sizeof(output) - strlen(output) - 1);
	}
	strncat(output, "-> ", sizeof(output) - strlen(output) - 1);

	/* process command */
	ptr = &read_buffer[0];
	jtagspi_cmd(bank, write_buffer[0], &write_buffer[1], num_write - 1, ptr, -num_read);
	if (retval != ERROR_OK)
		return retval;

	for ( ; num_read > 0; num_read--) {
		snprintf(temp, sizeof(temp), "%02" PRIx8 " ", *ptr++);
		strncat(output, temp, sizeof(output) - strlen(output) - 1);
	}
	command_print(CMD, "%s", output);

	return ERROR_OK;
}

COMMAND_HANDLER(jtagspi_handle_always_4byte)
{
	struct flash_bank *bank;
	struct jtagspi_flash_bank *jtagspi_info;
	int retval;

	LOG_DEBUG("%s", __func__);

	if ((CMD_ARGC != 1) && (CMD_ARGC != 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	jtagspi_info = bank->driver_priv;

	if (CMD_ARGC == 1)
		command_print(CMD, jtagspi_info->always_4byte ? "on" : "off");
	else
		COMMAND_PARSE_BOOL(CMD_ARGV[1], jtagspi_info->always_4byte, "on", "off");

	return ERROR_OK;
}

static int jtagspi_probe(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct flash_sector *sectors;
	const struct flash_device *p;
	uint8_t in_buf[3];
	uint32_t id, sectorsize;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}
	info->probed = false;

	if (!bank->target->tap) {
		LOG_ERROR("Target has no JTAG tap");
		return ERROR_FAIL;
	}
	info->tap = bank->target->tap;

	jtagspi_cmd(bank, SPIFLASH_READ_ID, NULL, 0, in_buf, -3);
	/* the table in spi.c has the manufacturer byte (first) as the lsb */
	id = le_to_h_u24(in_buf);

	memset(&info->dev, 0, sizeof(info->dev));
	for (p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			memcpy(&info->dev, p, sizeof(info->dev));
			break;
		}

	if (!(p->name)) {
		LOG_ERROR("Unknown flash device (ID 0x%06" PRIx32 ")", id & 0xFFFFFF);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%06" PRIx32 ")",
		info->dev.name, info->dev.device_id & 0xFFFFFF);

	/* Set correct size value */
	bank->size = info->dev.size_in_bytes;

	/* calculate address length in bytes */
	if (bank->size <= (1UL << 8))
		info->addr_len = 1;
	else if (bank->size <= (1UL << 16))
		info->addr_len = 2;
	else if (bank->size <= (1UL << 24))
		info->addr_len = 3;
	else {
		info->addr_len = 4;
		LOG_WARNING("4-byte addresses needed, might need extra command to enable");
	}

	/* if no sectors, treat whole bank as single sector */
	sectorsize = info->dev.sectorsize ?
		info->dev.sectorsize : info->dev.size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = info->dev.size_in_bytes / sectorsize;
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
	info->probed = true;
	return ERROR_OK;
}

static int jtagspi_auto_probe(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;

	if (info->probed)
		return ERROR_OK;
	return jtagspi_probe(bank);
}

static int jtagspi_read_status(struct flash_bank *bank, uint32_t *status)
{
	uint8_t buf;
	int err = jtagspi_cmd(bank, SPIFLASH_READ_STATUS, NULL, 0, &buf, -1);
	if (err == ERROR_OK) {
		*status = buf;
		LOG_DEBUG("status=0x%02" PRIx32, *status);
	}
	return err;
}

static int jtagspi_wait(struct flash_bank *bank, int timeout_ms)
{
	int64_t t0 = timeval_ms();
	int64_t dt;

	do {
		dt = timeval_ms() - t0;

		uint32_t status = (uint32_t)-1;
		int retval = jtagspi_read_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & SPIFLASH_BSY_BIT) == 0) {
			LOG_DEBUG("waited %" PRId64 " ms", dt);
			return ERROR_OK;
		}
		alive_sleep(1);
	} while (dt <= timeout_ms);

	LOG_ERROR("timeout, device still busy");
	return ERROR_FAIL;
}

static int jtagspi_write_enable(struct flash_bank *bank)
{
	jtagspi_cmd(bank, SPIFLASH_WRITE_ENABLE, NULL, 0, NULL, 0);

	uint32_t status = (uint32_t)-1;
	int retval = jtagspi_read_status(bank, &status);
	if (retval != ERROR_OK)
		return retval;

	if ((status & SPIFLASH_WE_BIT) == 0) {
		LOG_ERROR("Cannot enable write to flash. Status=0x%02" PRIx32, status);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int jtagspi_bulk_erase(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;
	int64_t t0 = timeval_ms();

	if (info->dev.chip_erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	jtagspi_cmd(bank, info->dev.chip_erase_cmd, NULL, 0, NULL, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = jtagspi_wait(bank, bank->num_sectors * JTAGSPI_MAX_TIMEOUT);
	LOG_INFO("took %" PRId64 " ms", timeval_ms() - t0);
	return retval;
}

static uint8_t *fill_addr(uint32_t addr, unsigned int addr_len, uint8_t *buffer)
{
	for (buffer += addr_len; addr_len > 0; --addr_len) {
		*--buffer = addr;
		addr >>= 8;
	}

	return buffer;
}

static int jtagspi_sector_erase(struct flash_bank *bank, unsigned int sector)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;
	uint8_t addr[sizeof(uint32_t)];
	int64_t t0 = timeval_ms();

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	/* ATXP032/064/128 use always 4-byte addresses except for 0x03 read */
	unsigned int addr_len = info->always_4byte ? 4 : info->addr_len;

	retval = jtagspi_cmd(bank, info->dev.erase_cmd, fill_addr(bank->sectors[sector].offset, addr_len, addr),
			addr_len, NULL, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = jtagspi_wait(bank, JTAGSPI_MAX_TIMEOUT);
	LOG_INFO("sector %u took %" PRId64 " ms", sector, timeval_ms() - t0);
	return retval;
}

static int jtagspi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("erase from sector %u to sector %u", first, last);

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	if (first == 0 && last == (bank->num_sectors - 1) &&
		info->dev.chip_erase_cmd != 0x00 &&
		info->dev.chip_erase_cmd != info->dev.erase_cmd) {
		LOG_DEBUG("Trying bulk erase.");
		retval = jtagspi_bulk_erase(bank);
		if (retval == ERROR_OK)
			return retval;
		else
			LOG_WARNING("Bulk flash erase failed. Falling back to sector erase.");
	}

	if (info->dev.erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = jtagspi_sector_erase(bank, sector);
		if (retval != ERROR_OK) {
			LOG_ERROR("Sector erase failed.");
			break;
		}
	}

	return retval;
}

static int jtagspi_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int jtagspi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	uint32_t pagesize, currsize;
	uint8_t addr[sizeof(uint32_t)];
	int retval;

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* if no sectorsize, use reasonable default */
	pagesize = info->dev.sectorsize ? info->dev.sectorsize : info->dev.pagesize;
	if (pagesize == 0)
		pagesize = (info->dev.size_in_bytes <= SPIFLASH_DEF_PAGESIZE) ?
			info->dev.size_in_bytes : SPIFLASH_DEF_PAGESIZE;

	/* ATXP032/064/128 use always 4-byte addresses except for 0x03 read */
	unsigned int addr_len = ((info->dev.read_cmd != 0x03) && info->always_4byte) ? 4 : info->addr_len;

	while (count > 0) {
		/* length up to end of current page */
		currsize = ((offset + pagesize) & ~(pagesize - 1)) - offset;
		/* but no more than remaining size */
		currsize = (count < currsize) ? count : currsize;

		retval = jtagspi_cmd(bank, info->dev.read_cmd, fill_addr(offset, addr_len, addr),
			addr_len, buffer, -currsize);
		if (retval != ERROR_OK) {
			LOG_ERROR("page read error");
			return retval;
		}
		LOG_DEBUG("read page at 0x%08" PRIx32, offset);
		offset += currsize;
		buffer += currsize;
		count -= currsize;
	}
	return ERROR_OK;
}

static int jtagspi_page_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	uint8_t addr[sizeof(uint32_t)];
	int retval;

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	/* ATXP032/064/128 use always 4-byte addresses except for 0x03 read */
	unsigned int addr_len = ((info->dev.read_cmd != 0x03) && info->always_4byte) ? 4 : info->addr_len;

	retval = jtagspi_cmd(bank, info->dev.pprog_cmd, fill_addr(offset, addr_len, addr),
		addr_len, (uint8_t *) buffer, count);
	if (retval != ERROR_OK)
		return retval;
	return jtagspi_wait(bank, JTAGSPI_MAX_TIMEOUT);
}

static int jtagspi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	uint32_t pagesize, currsize;
	int retval;

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* if no write pagesize, use reasonable default */
	pagesize = info->dev.pagesize ? info->dev.pagesize : SPIFLASH_DEF_PAGESIZE;

	while (count > 0) {
		/* length up to end of current page */
		currsize = ((offset + pagesize) & ~(pagesize - 1)) - offset;
		/* but no more than remaining size */
		currsize = (count < currsize) ? count : currsize;

		retval = jtagspi_page_write(bank, buffer, offset, currsize);
		if (retval != ERROR_OK) {
			LOG_ERROR("page write error");
			return retval;
		}
		LOG_DEBUG("wrote page at 0x%08" PRIx32, offset);
		offset += currsize;
		buffer += currsize;
		count -= currsize;
	}
	return ERROR_OK;
}

static int jtagspi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;

	if (!(info->probed)) {
		command_print_sameline(cmd, "\nJTAGSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "flash \'%s\', device id = 0x%06" PRIx32
		", flash size = %" PRIu32 " %sbytes\n(page size = %" PRIu32
		", read = 0x%02" PRIx8 ", qread = 0x%02" PRIx8
		", pprog = 0x%02" PRIx8 ", mass_erase = 0x%02" PRIx8
		", sector size = %" PRIu32 " %sbytes, sector_erase = 0x%02" PRIx8 ")",
		info->dev.name, info->dev.device_id & 0xFFFFFF,
		bank->size / 4096 ? bank->size / 1024 : bank->size,
		bank->size / 4096 ? "k" : "", info->dev.pagesize,
		info->dev.read_cmd, info->dev.qread_cmd,
		info->dev.pprog_cmd, info->dev.chip_erase_cmd,
		info->dev.sectorsize / 4096 ?
		info->dev.sectorsize / 1024 : info->dev.sectorsize,
		info->dev.sectorsize / 4096 ? "k" : "",
		info->dev.erase_cmd);

	return ERROR_OK;
}

static const struct command_registration jtagspi_exec_command_handlers[] = {
	{
		.name = "set",
		.handler = jtagspi_handle_set,
		.mode = COMMAND_EXEC,
		.usage = "bank_id name chip_size page_size read_cmd unused pprg_cmd "
			"[ mass_erase_cmd ] [ sector_size sector_erase_cmd ]",
		.help = "Set device parameters if not autodetected.",
	},
	{
		.name = "cmd",
		.handler = jtagspi_handle_cmd,
		.mode = COMMAND_EXEC,
		.usage = "bank_id num_resp cmd_byte ...",
		.help = "Send low-level command cmd_byte and following bytes, read num_bytes.",
	},
	{
		.name = "always_4byte",
		.handler = jtagspi_handle_always_4byte,
		.mode = COMMAND_EXEC,
		.usage = "bank_id [ on | off ]",
		.help = "Use always 4-byte address except for basic 0x03.",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration jtagspi_command_handlers[] = {
	{
		.name = "jtagspi",
		.mode = COMMAND_ANY,
		.help = "jtagspi command group",
		.usage = "",
		.chain = jtagspi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver jtagspi_flash = {
	.name = "jtagspi",
	.commands = jtagspi_command_handlers,
	.flash_bank_command = jtagspi_flash_bank_command,
	.erase = jtagspi_erase,
	.protect = jtagspi_protect,
	.write = jtagspi_write,
	.read = jtagspi_read,
	.probe = jtagspi_probe,
	.auto_probe = jtagspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = jtagspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

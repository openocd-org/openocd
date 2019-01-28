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
	int probed;
	uint32_t ir;
	int addr_len;
};

FLASH_BANK_COMMAND_HANDLER(jtagspi_flash_bank_command)
{
	struct jtagspi_flash_bank *info;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	info = malloc(sizeof(struct jtagspi_flash_bank));
	if (info == NULL) {
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	bank->driver_priv = info;

	info->tap = NULL;
	info->probed = 0;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->ir);

	return ERROR_OK;
}

static void jtagspi_set_ir(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct scan_field field;
	uint8_t buf[4];

	LOG_DEBUG("loading jtagspi ir");
	buf_set_u32(buf, 0, info->tap->ir_length, info->ir);
	field.num_bits = info->tap->ir_length;
	field.out_value = buf;
	field.in_value = NULL;
	jtag_add_ir_scan(info->tap, &field, TAP_IDLE);
}

static void flip_u8(uint8_t *in, uint8_t *out, int len)
{
	for (int i = 0; i < len; i++)
		out[i] = flip_u32(in[i], 8);
}

static int jtagspi_cmd(struct flash_bank *bank, uint8_t cmd,
		uint32_t *addr, int addr_len, uint8_t *data, int data_len)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct scan_field fields[6];
	uint8_t marker = 1;
	uint8_t xfer_bits_buf[4];
	uint8_t addr_buf[4];
	uint8_t *data_buf;
	uint32_t xfer_bits;
	int is_read, lenb, n;

	LOG_DEBUG("cmd=0x%02x addr=0x%0*x len=%i", cmd, addr_len * 2, addr ? *addr : 0, data_len);

	is_read = (data_len < 0);
	if (is_read)
		data_len = -data_len;

	n = 0;

	fields[n].num_bits = 1;
	fields[n].out_value = &marker;
	fields[n].in_value = NULL;
	n++;

	xfer_bits = 8 + data_len - 1;
	/* cmd + read/write - 1 due to the counter implementation */
	if (addr)
		xfer_bits += addr_len * 8;
	h_u32_to_be(xfer_bits_buf, xfer_bits);
	flip_u8(xfer_bits_buf, xfer_bits_buf, 4);
	fields[n].num_bits = 32;
	fields[n].out_value = xfer_bits_buf;
	fields[n].in_value = NULL;
	n++;

	cmd = flip_u32(cmd, 8);
	fields[n].num_bits = 8;
	fields[n].out_value = &cmd;
	fields[n].in_value = NULL;
	n++;

	if (addr) {
		h_u32_to_be(addr_buf, *addr);
		flip_u8(addr_buf, addr_buf, addr_len);
		fields[n].num_bits = addr_len * 8;
		fields[n].out_value = addr_buf + (4 - addr_len);
		fields[n].in_value = NULL;
		n++;
	}

	lenb = DIV_ROUND_UP(data_len, 8);
	data_buf = malloc(lenb);
	if (lenb > 0) {
		if (data_buf == NULL) {
			LOG_ERROR("no memory for spi buffer");
			return ERROR_FAIL;
		}
		if (is_read) {
			fields[n].num_bits = jtag_tap_count_enabled();
			fields[n].out_value = NULL;
			fields[n].in_value = NULL;
			n++;

			fields[n].out_value = NULL;
			fields[n].in_value = data_buf;
		} else {
			flip_u8(data, data_buf, lenb);
			fields[n].out_value = data_buf;
			fields[n].in_value = NULL;
		}
		fields[n].num_bits = data_len;
		n++;
	}

	jtagspi_set_ir(bank);
	/* passing from an IR scan to SHIFT-DR clears BYPASS registers */
	jtag_add_dr_scan(info->tap, n, fields, TAP_IDLE);
	jtag_execute_queue();

	if (is_read)
		flip_u8(data_buf, data, lenb);
	free(data_buf);
	return ERROR_OK;
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

	/* there are 3 mandatory arguments: devname, size_in_bytes, pagesize */
	if (index + 3 > CMD_ARGC) {
		command_print(CMD_CTX, "jtagspi: not enough arguments");
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
		info->probed = false;
	}
	memset(&info->dev, 0, sizeof(info->dev));

	strncpy(info->devname, CMD_ARGV[index++], sizeof(info->devname) - 1);
	info->devname[sizeof(info->devname) - 1] = '\0';

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
	info->dev.size_in_bytes = temp;
	if ((temp & (temp - 1)) || (temp < (1UL << 8))) {
		command_print(CMD_CTX, "jtagspi: device size must be 2^n with n >= 8");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
	info->dev.pagesize = temp;
	if (info->dev.pagesize == 0)
		info->dev.pagesize = SPIFLASH_DEF_PAGESIZE;
	if ((temp & (temp - 1)) || (temp > info->dev.size_in_bytes)) {
		command_print(CMD_CTX, "jtagspi: page size must be 2^n and <= device size");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (index + 3 > CMD_ARGC) {
		command_print(CMD_CTX, "jtagspi: not enough arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.read_cmd);
	if ((info->dev.read_cmd != 0x03) &&
		(info->dev.read_cmd != 0x13)) {
		command_print(CMD_CTX, "jtagspi: only 0x03/0x13 READ allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.qread_cmd);

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.pprog_cmd);
	if ((info->dev.pprog_cmd != 0x02) &&
		(info->dev.pprog_cmd != 0x12)) {
		command_print(CMD_CTX, "jtagspi: only 0x02/0x12 PPRG allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (index < CMD_ARGC)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.chip_erase_cmd);
	else
		info->dev.chip_erase_cmd = 0x00;

	if (index < CMD_ARGC) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
		info->dev.sectorsize = temp;
		if ((info->dev.sectorsize > info->dev.size_in_bytes) ||
			(info->dev.sectorsize < info->dev.pagesize) || (temp & (temp - 1))) {
			command_print(CMD_CTX, "jtagspi: sector size must be 2^n and <= device size");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (index < CMD_ARGC)
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], info->dev.erase_cmd);
		else {
			command_print(CMD_CTX, "jtagspi: erase command missing");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else {
		/* no sector size / sector erase cmd given, treat whole bank as a single sector */
		info->dev.erase_cmd = 0x00;
		info->dev.sectorsize = info->dev.size_in_bytes;
	}

	if (index < CMD_ARGC) {
		command_print(CMD_CTX, "jtagspi: extra arguments");
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
	if (sectors == NULL) {
		LOG_ERROR("Not enough memory");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * (info->dev.sectorsize);
		sectors[sector].size = info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	info->dev.name = info->devname;
	if (info->dev.size_in_bytes / 1024)
		LOG_INFO("flash \'%s\' id = unknown\nflash size = %" PRIu32 "kbytes",
			info->dev.name, info->dev.size_in_bytes / 1024);
	else
		LOG_INFO("flash \'%s\' id = unknown\nflash size = %" PRIu32 "bytes",
			info->dev.name, info->dev.size_in_bytes);
	info->probed = true;

	return ERROR_OK;
}

COMMAND_HANDLER(jtagspi_handle_cmd)
{
	struct flash_bank *bank;
	uint32_t addr;
	uint16_t num_bytes;
	uint8_t cmd_byte, data_byte, buffer[1 << CHAR_BIT], *ptr;
	unsigned int index = 1;
	const int max = sizeof(uint32_t) + 1;
	char temp[6], output[(2 + max + (1 << CHAR_BIT)) * 3 + 10];
	int addr_len, retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 2) {
		command_print(CMD_CTX, "jtagspi: not enough arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC > max + 3)	{
		LOG_ERROR("at most %d bytes may be send", max);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	/* at most 256 bytes may be read */
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[index++], num_bytes);
	if (num_bytes > (1 << CHAR_BIT)) {
		LOG_ERROR("at most %d bytes may be read", (1 << CHAR_BIT));
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (index >= CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmd_byte);

	/* command byte */
	snprintf(output, sizeof(output), "spi: %02" PRIx8 " ", cmd_byte);
	if (retval != ERROR_OK)
		goto err;

	/* additional bytes */
	addr_len = 0;
	for ( ; index < CMD_ARGC; index++) {
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index], data_byte);
		addr = (addr << 8) | data_byte;
		addr_len++;
		snprintf(temp, sizeof(temp), "%02" PRIx8 " ", data_byte);
		if (retval != ERROR_OK)
			goto err;
		strncat(output, temp, sizeof(output) - strlen(output) - 1);
	}

	strncat(output, "-> ", sizeof(output) - strlen(output) - 1);

	/* now read the response bytes */
	ptr = &buffer[0];
	jtagspi_cmd(bank, cmd_byte, &addr, addr_len, ptr, -num_bytes * 8);
	if (retval != ERROR_OK)
		goto err;
	for ( ; num_bytes > 0; num_bytes--) {
		snprintf(temp, sizeof(temp), "%02" PRIx8 " ", *ptr++);
		strncat(output, temp, sizeof(output) - strlen(output) - 1);
	}
	command_print(CMD_CTX, "%s", output);

err:

	return retval;
}

static int jtagspi_probe(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct flash_sector *sectors;
	const struct flash_device *p;
	uint8_t in_buf[3];
	uint32_t id, sectorsize;

	if (info->probed)
		free(bank->sectors);
	info->probed = 0;

	if (bank->target->tap == NULL) {
		LOG_ERROR("Target has no JTAG tap");
		return ERROR_FAIL;
	}
	info->tap = bank->target->tap;

	jtagspi_cmd(bank, SPIFLASH_READ_ID, NULL, 0, in_buf, -24);
	/* the table in spi.c has the manufacturer byte (first) as the lsb */
	id = le_to_h_u24(in_buf);

	memset(&info->dev, 0, sizeof(info->dev));
	for (p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			memcpy(&info->dev, p, sizeof(info->dev));
			break;
		}

	if (!(p->name)) {
		LOG_ERROR("Unknown flash device (ID 0x%06" PRIx32 ")",
			id & 0xFFFFFF);
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
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	info->probed = 1;
	return ERROR_OK;
}

static int jtagspi_auto_probe(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;

	if (info->probed)
		return ERROR_OK;
	return jtagspi_probe(bank);
}

static void jtagspi_read_status(struct flash_bank *bank, uint32_t *status)
{
	uint8_t buf;
	if (jtagspi_cmd(bank, SPIFLASH_READ_STATUS, NULL, 0, &buf, -8) == ERROR_OK) {
		*status = buf;
		/* LOG_DEBUG("status=0x%08" PRIx32, *status); */
	}
}

static int jtagspi_wait(struct flash_bank *bank, int timeout_ms)
{
	uint32_t status;
	int64_t t0 = timeval_ms();
	int64_t dt;

	do {
		dt = timeval_ms() - t0;
		jtagspi_read_status(bank, &status);
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
	uint32_t status;

	jtagspi_cmd(bank, SPIFLASH_WRITE_ENABLE, NULL, 0, NULL, 0);
	jtagspi_read_status(bank, &status);
	if ((status & SPIFLASH_WE_BIT) == 0) {
		LOG_ERROR("Cannot enable write to flash. Status=0x%08" PRIx32, status);
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
	retval = jtagspi_wait(bank, bank->num_sectors*JTAGSPI_MAX_TIMEOUT);
	LOG_INFO("took %" PRId64 " ms", timeval_ms() - t0);
	return retval;
}

static int jtagspi_sector_erase(struct flash_bank *bank, int sector)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;
	int64_t t0 = timeval_ms();

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;
	jtagspi_cmd(bank, info->dev.erase_cmd, &bank->sectors[sector].offset,
		info->addr_len, NULL, 0);
	retval = jtagspi_wait(bank, JTAGSPI_MAX_TIMEOUT);
	LOG_INFO("sector %d took %" PRId64 " ms", sector, timeval_ms() - t0);
	return retval;
}

static int jtagspi_erase(struct flash_bank *bank, int first, int last)
{
	int sector;
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("erase from sector %d to sector %d", first, last);

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
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

	for (sector = first; sector <= last; sector++) {
		retval = jtagspi_sector_erase(bank, sector);
		if (retval != ERROR_OK) {
			LOG_ERROR("Sector erase failed.");
			break;
		}
	}

	return retval;
}

static int jtagspi_protect(struct flash_bank *bank, int set, int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int jtagspi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not yet probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	jtagspi_cmd(bank, SPIFLASH_READ, &offset, info->addr_len, buffer, -count*8);
	return ERROR_OK;
}

static int jtagspi_page_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;
	jtagspi_cmd(bank, SPIFLASH_PAGE_PROGRAM, &offset, info->addr_len, (uint8_t *) buffer, count*8);
	return jtagspi_wait(bank, JTAGSPI_MAX_TIMEOUT);
}

static int jtagspi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;
	uint32_t n, pagesize;

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not yet probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* if no write pagesize, use reasonable default */
	pagesize = info->dev.pagesize ? info->dev.pagesize : SPIFLASH_DEF_PAGESIZE;

	for (n = 0; n < count; n += pagesize) {
		retval = jtagspi_page_write(bank, buffer + n, offset + n,
				MIN(count - n, pagesize));
		if (retval != ERROR_OK) {
			LOG_ERROR("page write error");
			return retval;
		}
		LOG_DEBUG("wrote page at 0x%08" PRIx32, offset + n);
	}
	return ERROR_OK;
}

static int jtagspi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;

	if (!(info->probed)) {
		snprintf(buf, buf_size, "\nJTAGSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nSPIFI flash information:\n"
		"  Device \'%s\' (ID 0x%06" PRIx32 ")\n",
		info->dev.name, info->dev.device_id & 0xFFFFFF);

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

struct flash_driver jtagspi_flash = {
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

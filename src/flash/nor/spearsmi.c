/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
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

/* SPEAr Serial Memory Interface (SMI) controller is a SPI bus controller
 * specifically designed for SPI memories.
 * Only SPI "mode 3" (CPOL=1 and CPHA=1) is supported.
 * Two working modes are available:
 * - SW mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus.
 * - HW mode: the SPI but is under SMI control. Memory content is directly
 *   accessible in CPU memory space. CPU can read, write and execute memory
 *   content. */

/* ATTENTION:
 * To have flash memory mapped in CPU memory space, the SMI controller
 * have to be in "HW mode". This requires following constraints:
 * 1) The command "reset init" have to initialize SMI controller and put
 *    it in HW mode;
 * 2) every command in this file have to return to prompt in HW mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>

#define JTAG_ID_3XX_6XX  (0x07926041)

#define SMI_READ_REG(a) (_SMI_READ_REG(a))
#define _SMI_READ_REG(a)        	\
{	                                \
	int __a;                        \
	uint32_t __v;                   \
	                                \
	__a = target_read_u32(target, io_base + (a), &__v); \
	if (__a != ERROR_OK)            \
	    return __a;                 \
	__v;                            \
}

#define SMI_WRITE_REG(a,v)      	\
{	                                \
	int __r;                        \
	                                \
	__r = target_write_u32(target, io_base + (a), (v)); \
	if (__r != ERROR_OK)            \
	    return __r;                 \
}

#define SMI_POLL_TFF(timeout)   	\
{	                                \
	int __r;                        \
	                                \
	__r = poll_tff(target, io_base, timeout); \
	if (__r != ERROR_OK)            \
	    return __r;                 \
}

#define SMI_SET_SW_MODE()	SMI_WRITE_REG(SMI_CR1, \
	SMI_READ_REG(SMI_CR1) | SMI_SW_MODE)
#define SMI_SET_HWWB_MODE() SMI_WRITE_REG(SMI_CR1, \
	(SMI_READ_REG(SMI_CR1) | SMI_WB_MODE) & ~SMI_SW_MODE)
#define SMI_SET_HW_MODE()	SMI_WRITE_REG(SMI_CR1, \
	SMI_READ_REG(SMI_CR1) & ~(SMI_SW_MODE | SMI_WB_MODE))
#define SMI_CLEAR_TFF()		SMI_WRITE_REG(SMI_SR, ~SMI_TFF)

#define SMI_BANK_SIZE      (0x01000000)

#define SMI_BASE_3XX_6XX   (0xf8000000)
#define SMI_CFGREG_3XX_6XX (0xfc000000)

/* #define SMI_BASE_13XX      (0xe6000000) */
/* #define SMI_CFGREG_13XX    (0xea000000) */

#define SMI_CR1 (0x00) /* Control register 1 */
#define SMI_CR2 (0x04) /* Control register 2 */
#define SMI_SR  (0x08) /* Status register */
#define SMI_TR  (0x0c) /* TX */
#define SMI_RR  (0x10) /* RX */

/* fields in SMI_CR1 */
#define SMI_SW_MODE       0x10000000 /* set to enable SW Mode */
#define SMI_WB_MODE       0x20000000 /* Write Burst Mode */

/* fields in SMI_CR2 */
#define SMI_TX_LEN_1      0x00000001 /* data length = 1 byte */
#define SMI_TX_LEN_4      0x00000004 /* data length = 4 byte */
#define SMI_RX_LEN_3      0x00000030 /* data length = 3 byte */
#define SMI_SEND          0x00000080 /* Send data */
#define SMI_RSR           0x00000400 /* reads status reg */
#define SMI_WE            0x00000800 /* Write Enable */
#define SMI_SEL_BANK0     0x00000000 /* Select Bank0 */
#define SMI_SEL_BANK1     0x00001000 /* Select Bank1 */
#define SMI_SEL_BANK2     0x00002000 /* Select Bank2 */
#define SMI_SEL_BANK3     0x00003000 /* Select Bank3 */

/* fields in SMI_SR */
#define SMI_WIP_BIT       0x00000001 /* WIP Bit of SPI SR on SMI SR */
#define SMI_WEL_BIT       0x00000002 /* WEL Bit of SPI SR on SMI SR */
#define SMI_TFF           0x00000100 /* Transfer Finished Flag */

/* Commands */
#define SMI_READ_ID       0x0000009F /* Read Flash Identification */

/* Timeout in ms */
#define SMI_CMD_TIMEOUT   (100)
#define SMI_PROBE_TIMEOUT (100)
#define SMI_MAX_TIMEOUT  (3000)

struct spearsmi_flash_bank
{
	int probed;
	uint32_t io_base;
	uint32_t bank_num;
	struct flash_device *dev;
};

/* data structure to maintain flash ids from different vendors */
struct flash_device {
	char *name;
	uint8_t erase_cmd;
	uint32_t device_id;
	uint32_t pagesize;
	unsigned long sectorsize;
	unsigned long size_in_bytes;
};

#define FLASH_ID(n, es, id, psize, ssize, size) \
{	                        \
	.name = n,              \
	.erase_cmd = es,        \
	.device_id = id,        \
	.pagesize = psize,      \
	.sectorsize = ssize,    \
	.size_in_bytes = size   \
}

/* List below is taken from Linux driver. It is not exhaustive of all the
 * possible SPI memories, nor exclusive for SMI. Could be shared with
 * other SPI drivers. */
static struct flash_device flash_devices[] = {
	/* name, erase_cmd, device_id, pagesize, sectorsize, size_in_bytes */
	FLASH_ID("st m25p05",      0xd8, 0x00102020, 0x80,  0x8000,  0x10000),
	FLASH_ID("st m25p10",      0xd8, 0x00112020, 0x80,  0x8000,  0x20000),
	FLASH_ID("st m25p20",      0xd8, 0x00122020, 0x100, 0x10000, 0x40000),
	FLASH_ID("st m25p40",      0xd8, 0x00132020, 0x100, 0x10000, 0x80000),
	FLASH_ID("st m25p80",      0xd8, 0x00142020, 0x100, 0x10000, 0x100000),
	FLASH_ID("st m25p16",      0xd8, 0x00152020, 0x100, 0x10000, 0x200000),
	FLASH_ID("st m25p32",      0xd8, 0x00162020, 0x100, 0x10000, 0x400000),
	FLASH_ID("st m25p64",      0xd8, 0x00172020, 0x100, 0x10000, 0x800000),
	FLASH_ID("st m25p128",     0xd8, 0x00182020, 0x100, 0x40000, 0x1000000),
	FLASH_ID("st m45pe10",     0xd8, 0x00114020, 0x100, 0x10000, 0x20000),
	FLASH_ID("st m45pe20",     0xd8, 0x00124020, 0x100, 0x10000, 0x40000),
	FLASH_ID("st m45pe40",     0xd8, 0x00134020, 0x100, 0x10000, 0x80000),
	FLASH_ID("st m45pe80",     0xd8, 0x00144020, 0x100, 0x10000, 0x100000),
	FLASH_ID("sp s25fl004",    0xd8, 0x00120201, 0x100, 0x10000, 0x80000),
	FLASH_ID("sp s25fl008",    0xd8, 0x00130201, 0x100, 0x10000, 0x100000),
	FLASH_ID("sp s25fl016",    0xd8, 0x00140201, 0x100, 0x10000, 0x200000),
	FLASH_ID("sp s25fl032",    0xd8, 0x00150201, 0x100, 0x10000, 0x400000),
	FLASH_ID("sp s25fl064",    0xd8, 0x00160201, 0x100, 0x10000, 0x800000),
	FLASH_ID("atmel 25f512",   0x52, 0x0065001f, 0x80,  0x8000,  0x10000),
	FLASH_ID("atmel 25f1024",  0x52, 0x0060001f, 0x100, 0x8000,  0x20000),
	FLASH_ID("atmel 25f2048",  0x52, 0x0063001f, 0x100, 0x10000, 0x40000),
	FLASH_ID("atmel 25f4096",  0x52, 0x0064001f, 0x100, 0x10000, 0x80000),
	FLASH_ID("atmel 25fs040",  0xd7, 0x0004661f, 0x100, 0x10000, 0x80000),
	FLASH_ID("mac 25l512",     0xd8, 0x001020c2, 0x010, 0x10000, 0x10000),
	FLASH_ID("mac 25l1005",    0xd8, 0x001120c2, 0x010, 0x10000, 0x20000),
	FLASH_ID("mac 25l2005",    0xd8, 0x001220c2, 0x010, 0x10000, 0x40000),
	FLASH_ID("mac 25l4005",    0xd8, 0x001320c2, 0x010, 0x10000, 0x80000),
	FLASH_ID("mac 25l8005",    0xd8, 0x001420c2, 0x010, 0x10000, 0x100000),
	FLASH_ID("mac 25l1605",    0xd8, 0x001520c2, 0x100, 0x10000, 0x200000),
	FLASH_ID("mac 25l3205",    0xd8, 0x001620c2, 0x100, 0x10000, 0x400000),
	FLASH_ID("mac 25l6405",    0xd8, 0x001720c2, 0x100, 0x10000, 0x800000),
	FLASH_ID(NULL,             0,    0,          0,     0,       0)
};

FLASH_BANK_COMMAND_HANDLER(spearsmi_flash_bank_command)
{
	struct spearsmi_flash_bank *spearsmi_info;

	LOG_DEBUG(__FUNCTION__);

	if (CMD_ARGC < 6)
	{
		LOG_WARNING("incomplete flash_bank spearsmi configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	spearsmi_info = malloc(sizeof(struct spearsmi_flash_bank));
	if (spearsmi_info == NULL)
	{
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = spearsmi_info;
	spearsmi_info->probed = 0;

	return ERROR_OK;
}

/* Poll transmit finished flag */
/* timeout in ms */
static int poll_tff(struct target *target, uint32_t io_base, int timeout)
{
	long long endtime;

	if (SMI_READ_REG(SMI_SR) & SMI_TFF)
		return ERROR_OK;

	endtime = timeval_ms() + timeout;
	do {
		alive_sleep(1);
		if (SMI_READ_REG(SMI_SR) & SMI_TFF)
			return ERROR_OK;
	} while (timeval_ms() < endtime);

	LOG_ERROR("Timeout while polling TFF");
	return ERROR_FLASH_OPERATION_FAILED;
}

/* Read the status register of the external SPI flash chip.
 * The operation is triggered by setting SMI_RSR bit.
 * SMI sends the proper SPI command (0x05) and returns value in SMI_SR */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base = spearsmi_info->io_base;

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* Read status */
	SMI_WRITE_REG(SMI_CR2, spearsmi_info->bank_num | SMI_RSR);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	*status = SMI_READ_REG(SMI_SR) & 0x0000ffff;

	/* clean-up SMI_CR2 */
	SMI_WRITE_REG(SMI_CR2, 0); /* AB: Required ? */

	return ERROR_OK;
}

/* check for WIP (write in progress) bit in status register */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval;
	long long endtime;

    endtime = timeval_ms() + timeout;
    do {
        /* read flash status register */
        retval = read_status_reg(bank, &status);
        if (retval != ERROR_OK)
            return retval;

		if ((status & SMI_WIP_BIT) == 0)
			return ERROR_OK;
		alive_sleep(1);
    } while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

/* Send "write enable" command to SPI flash chip.
 * The operation is triggered by setting SMI_WE bit, and SMI sends
 * the proper SPI command (0x06) */
static int smi_write_enable(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base = spearsmi_info->io_base;
	uint32_t status;
	int retval;

	/* Enter in HW mode */
	SMI_SET_HW_MODE(); /* AB: is this correct ?*/

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* Send write enable command */
	SMI_WRITE_REG(SMI_CR2, spearsmi_info->bank_num | SMI_WE);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* read flash status register */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		return retval;

	/* Check write enabled */
	if ((status & SMI_WEL_BIT) == 0)
	{
		LOG_ERROR("Cannot enable write to flash. Status=0x%08" PRIx32, status);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static uint32_t erase_command(struct spearsmi_flash_bank *spearsmi_info,
	uint32_t offset)
{
	union {
		uint32_t command;
		uint8_t x[4];
	} cmd;

	cmd.x[0] = spearsmi_info->dev->erase_cmd;
	cmd.x[1] = offset >> 16;
	cmd.x[2] = offset >> 8;
	cmd.x[3] = offset;

	return cmd.command;
}

static int smi_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base = spearsmi_info->io_base;
	uint32_t cmd;
	int retval;

	retval = smi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Switch to SW mode to send sector erase command */
	SMI_SET_SW_MODE();

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* send SPI command "block erase" */
	cmd = erase_command(spearsmi_info, bank->sectors[sector].offset);
	SMI_WRITE_REG(SMI_TR, cmd);
	SMI_WRITE_REG(SMI_CR2, spearsmi_info->bank_num | SMI_SEND | SMI_TX_LEN_4);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* poll WIP for end of self timed Sector Erase cycle */
	retval = wait_till_ready(bank, SMI_MAX_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int spearsmi_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base = spearsmi_info->io_base;
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("%s: from sector %d to sector %d", __FUNCTION__, first, last);

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(spearsmi_info->probed))
	{
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = first; sector <= last; sector++)
	{
		if (bank->sectors[sector].is_protected)
		{
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	for (sector = first; sector <= last; sector++)
	{
		retval = smi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	/* Switch to HW mode before return to prompt */
	SMI_SET_HW_MODE();
	return retval;
}

static int spearsmi_protect(struct flash_bank *bank, int set,
	int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int smi_write_buffer(struct flash_bank *bank, uint8_t *buffer,
	uint32_t address, uint32_t len)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base = spearsmi_info->io_base;
	int retval;

	LOG_DEBUG("%s: address=0x%08" PRIx32 " len=0x%08" PRIx32,
		__FUNCTION__, address, len);

	retval = smi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	/* HW mode, write burst mode */
	SMI_SET_HWWB_MODE();

	retval = target_write_buffer(target, address, len, buffer);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int spearsmi_write(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base = spearsmi_info->io_base;
	uint32_t cur_count, page_size, page_offset;
	int sector;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__FUNCTION__, offset, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > spearsmi_info->dev->size_in_bytes)
	{
		LOG_WARNING("Write pasts end of flash. Extra data discarded.");
		count = spearsmi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++)
	{
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ( (offset <
				(bank->sectors[sector].offset + bank->sectors[sector].size))
			&& ((offset + count - 1) >= bank->sectors[sector].offset)
			&& bank->sectors[sector].is_protected )
		{
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	page_size = spearsmi_info->dev->pagesize;

	/* unaligned buffer head */
	if (count > 0 && (offset & 3) != 0)
	{
		cur_count = 4 - (offset & 3);
		if (cur_count > count)
			cur_count = count;
		retval = smi_write_buffer(bank, buffer, bank->base + offset,
			cur_count);
		if (retval != ERROR_OK)
			goto err;
		offset += cur_count;
		buffer += cur_count;
		count -= cur_count;
	}

	page_offset = offset % page_size;
	/* central part, aligned words */
	while (count >= 4)
	{
		/* clip block at page boundary */
		if (page_offset + count > page_size)
			cur_count = page_size - page_offset;
		else
			cur_count = count & ~3;

		retval = smi_write_buffer(bank, buffer, bank->base + offset,
			cur_count);
		if (retval != ERROR_OK)
			goto err;

		page_offset = 0;
		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;

		keep_alive();
	}

	/* buffer tail */
	if (count > 0)
		retval = smi_write_buffer(bank, buffer, bank->base + offset, count);

err:
	/* Switch to HW mode before return to prompt */
	SMI_SET_HW_MODE();
	return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base = spearsmi_info->io_base;
	int retval;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* poll WIP */
	retval = wait_till_ready(bank, SMI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* enter in SW mode */
	SMI_SET_SW_MODE();

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* Send SPI command "read ID" */
	SMI_WRITE_REG(SMI_TR, SMI_READ_ID);
	SMI_WRITE_REG(SMI_CR2,
		spearsmi_info->bank_num | SMI_SEND | SMI_RX_LEN_3 | SMI_TX_LEN_1);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* read ID from Receive Register */
	*id = SMI_READ_REG(SMI_RR) & 0x00ffffff;
	return ERROR_OK;
}

static int spearsmi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	uint32_t io_base;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	int retval;

	if (spearsmi_info->probed)
		free(bank->sectors);
	spearsmi_info->probed = 0;

	/* check for SPEAr device */
	switch (target->tap->idcode)
	{
		case JTAG_ID_3XX_6XX:
			/* SPEAr3xx/6xx */
			spearsmi_info->io_base = SMI_CFGREG_3XX_6XX;
			switch (bank->base)
			{
				case SMI_BASE_3XX_6XX:
					spearsmi_info->bank_num = SMI_SEL_BANK0;
					break;
				case SMI_BASE_3XX_6XX + SMI_BANK_SIZE:
					spearsmi_info->bank_num = SMI_SEL_BANK1;
					break;
				case SMI_BASE_3XX_6XX + 2*SMI_BANK_SIZE:
					spearsmi_info->bank_num = SMI_SEL_BANK2;
					break;
				case SMI_BASE_3XX_6XX + 3*SMI_BANK_SIZE:
					spearsmi_info->bank_num = SMI_SEL_BANK3;
					break;
				default:
					LOG_ERROR("Invalid base address 0x%" PRIx32, bank->base);
					return ERROR_FAIL;
			}
			break;

		default:
			LOG_ERROR("0x%" PRIx32 " is invalid id for SPEAr device",
				target->tap->idcode);
			return ERROR_FAIL;
	}
	io_base = spearsmi_info->io_base;

	/* read and decode flash ID; returns in SW mode */
	retval = read_flash_id(bank, &id);
	SMI_SET_HW_MODE();
	if (retval != ERROR_OK)
		return retval;

	spearsmi_info->dev = NULL;
	for (struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			spearsmi_info->dev = p;
			break;
		}

	if (!spearsmi_info->dev)
	{
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		spearsmi_info->dev->name, spearsmi_info->dev->device_id);

	/* Set correct size value */
	bank->size = spearsmi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors =
		spearsmi_info->dev->size_in_bytes / spearsmi_info->dev->sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL)
	{
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++)
	{
		sectors[sector].offset = sector * spearsmi_info->dev->sectorsize;
		sectors[sector].size = spearsmi_info->dev->sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 1;
	}

	bank->sectors = sectors;
	spearsmi_info->probed = 1;
	return ERROR_OK;
}

static int spearsmi_auto_probe(struct flash_bank *bank)
{
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	if (spearsmi_info->probed)
		return ERROR_OK;
	return spearsmi_probe(bank);
}

static int spearsmi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_spearsmi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct spearsmi_flash_bank *spearsmi_info = bank->driver_priv;
	int printed;

	if (!(spearsmi_info->probed))
	{
		printed = snprintf(buf, buf_size,
			"\nSPEAr SMI flash bank not probed yet\n");
		return ERROR_OK;
	}

	printed = snprintf(buf, buf_size, "\nSPEAr SMI flash information:\n"
		"  Device \'%s\' (ID 0x%08x)\n",
		spearsmi_info->dev->name, spearsmi_info->dev->device_id);
	buf += printed;
	buf_size -= printed;

	return ERROR_OK;
}

struct flash_driver spearsmi_flash = {
	.name = "spearsmi",
	.flash_bank_command = spearsmi_flash_bank_command,
	.erase = spearsmi_erase,
	.protect = spearsmi_protect,
	.write = spearsmi_write,
	.read = default_flash_read,
	.probe = spearsmi_probe,
	.auto_probe = spearsmi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = spearsmi_protect_check,
	.info = get_spearsmi_info,
};

/***************************************************************************
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mips32_dmaacc.h"
#include <helper/time_support.h>

static int mips32_dmaacc_read_mem8(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint8_t *buf);
static int mips32_dmaacc_read_mem16(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint16_t *buf);
static int mips32_dmaacc_read_mem32(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint32_t *buf);

static int mips32_dmaacc_write_mem8(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, const uint8_t *buf);
static int mips32_dmaacc_write_mem16(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, const uint16_t *buf);
static int mips32_dmaacc_write_mem32(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, const uint32_t *buf);

/*
 * The following logic shamelessly cloned from HairyDairyMaid's wrt54g_debrick
 * to support the Broadcom BCM5352 SoC in the Linksys WRT54GL wireless router
 * (and any others that support EJTAG DMA transfers).
 * Note: This only supports memory read/write. Since the BCM5352 doesn't
 * appear to support PRACC accesses, all debug functions except halt
 * do not work.  Still, this does allow erasing/writing flash as well as
 * displaying/modifying memory and memory mapped registers.
 */

static int ejtag_dma_dstrt_poll(struct mips_ejtag *ejtag_info)
{
	uint32_t ejtag_ctrl;
	int64_t start = timeval_ms();

	do {
		if (timeval_ms() - start > 1000) {
			LOG_ERROR("DMA time out");
			return -ETIMEDOUT;
		}
		ejtag_ctrl = EJTAG_CTRL_DMAACC | ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	} while (ejtag_ctrl & EJTAG_CTRL_DSTRT);
	return 0;
}

static int ejtag_dma_read(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t *data)
{
	uint32_t v;
	uint32_t ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Read & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DRWN | EJTAG_CTRL_DMA_WORD | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	ejtag_dma_dstrt_poll(ejtag_info);

	/* Read Data */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_32(ejtag_info, data);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR) {
		if (retries--) {
			LOG_ERROR("DMA Read Addr = %08" PRIx32 "  Data = ERROR ON READ (retrying)", addr);
			goto begin_ejtag_dma_read;
		} else
			LOG_ERROR("DMA Read Addr = %08" PRIx32 "  Data = ERROR ON READ", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int ejtag_dma_read_h(struct mips_ejtag *ejtag_info, uint32_t addr, uint16_t *data)
{
	uint32_t v;
	uint32_t ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read_h:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Read & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DRWN | EJTAG_CTRL_DMA_HALFWORD |
			EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	ejtag_dma_dstrt_poll(ejtag_info);

	/* Read Data */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR) {
		if (retries--) {
			LOG_ERROR("DMA Read Addr = %08" PRIx32 "  Data = ERROR ON READ (retrying)", addr);
			goto begin_ejtag_dma_read_h;
		} else
			LOG_ERROR("DMA Read Addr = %08" PRIx32 "  Data = ERROR ON READ", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Handle the bigendian/littleendian */
	if (addr & 0x2)
		*data = (v >> 16) & 0xffff;
	else
		*data = (v & 0x0000ffff);

	return ERROR_OK;
}

static int ejtag_dma_read_b(struct mips_ejtag *ejtag_info, uint32_t addr, uint8_t *data)
{
	uint32_t v;
	uint32_t ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read_b:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Read & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DRWN | EJTAG_CTRL_DMA_BYTE | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	ejtag_dma_dstrt_poll(ejtag_info);

	/* Read Data */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR) {
		if (retries--) {
			LOG_ERROR("DMA Read Addr = %08" PRIx32 "  Data = ERROR ON READ (retrying)", addr);
			goto begin_ejtag_dma_read_b;
		} else
			LOG_ERROR("DMA Read Addr = %08" PRIx32 "  Data = ERROR ON READ", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Handle the bigendian/littleendian */
	switch (addr & 0x3) {
		case 0:
			*data = v & 0xff;
			break;
		case 1:
			*data = (v >> 8) & 0xff;
			break;
		case 2:
			*data = (v >> 16) & 0xff;
			break;
		case 3:
			*data = (v >> 24) & 0xff;
			break;
	}

	return ERROR_OK;
}

static int ejtag_dma_write(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t data)
{
	uint32_t v;
	uint32_t ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_write:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Setup Data */
	v = data;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Write & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DMA_WORD | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	ejtag_dma_dstrt_poll(ejtag_info);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR) {
		if (retries--) {
			LOG_ERROR("DMA Write Addr = %08" PRIx32 "  Data = ERROR ON WRITE (retrying)", addr);
			goto begin_ejtag_dma_write;
		} else
			LOG_ERROR("DMA Write Addr = %08" PRIx32 "  Data = ERROR ON WRITE", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int ejtag_dma_write_h(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t data)
{
	uint32_t v;
	uint32_t ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

	/* Handle the bigendian/littleendian */
	data &= 0xffff;
	data |= data << 16;

begin_ejtag_dma_write_h:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Setup Data */
	v = data;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Write & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DMA_HALFWORD | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	ejtag_dma_dstrt_poll(ejtag_info);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl & EJTAG_CTRL_DERR) {
		if (retries--) {
			LOG_ERROR("DMA Write Addr = %08" PRIx32 "  Data = ERROR ON WRITE (retrying)", addr);
			goto begin_ejtag_dma_write_h;
		} else
			LOG_ERROR("DMA Write Addr = %08" PRIx32 "  Data = ERROR ON WRITE", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int ejtag_dma_write_b(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t data)
{
	uint32_t v;
	uint32_t ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

	/* Handle the bigendian/littleendian */
	data &= 0xff;
	data |= data << 8;
	data |= data << 16;

begin_ejtag_dma_write_b:

	/*  Setup Address*/
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Setup Data */
	v = data;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Write & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DMA_BYTE | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	ejtag_dma_dstrt_poll(ejtag_info);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl & EJTAG_CTRL_DERR) {
		if (retries--) {
			LOG_ERROR("DMA Write Addr = %08" PRIx32 "  Data = ERROR ON WRITE (retrying)", addr);
			goto begin_ejtag_dma_write_b;
		} else
			LOG_ERROR("DMA Write Addr = %08" PRIx32 "  Data = ERROR ON WRITE", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

int mips32_dmaacc_read_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, void *buf)
{
	switch (size) {
		case 1:
			return mips32_dmaacc_read_mem8(ejtag_info, addr, count, (uint8_t *)buf);
		case 2:
			return mips32_dmaacc_read_mem16(ejtag_info, addr, count, (uint16_t *)buf);
		case 4:
			return mips32_dmaacc_read_mem32(ejtag_info, addr, count, (uint32_t *)buf);
	}

	return ERROR_OK;
}

static int mips32_dmaacc_read_mem32(struct mips_ejtag *ejtag_info, uint32_t addr, int count, uint32_t *buf)
{
	int i;
	int	retval;

	for (i = 0; i < count; i++) {
		retval = ejtag_dma_read(ejtag_info, addr + i * sizeof(*buf), &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mips32_dmaacc_read_mem16(struct mips_ejtag *ejtag_info, uint32_t addr, int count, uint16_t *buf)
{
	int i;
	int retval;

	for (i = 0; i < count; i++) {
		retval = ejtag_dma_read_h(ejtag_info, addr + i * sizeof(*buf), &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mips32_dmaacc_read_mem8(struct mips_ejtag *ejtag_info, uint32_t addr, int count, uint8_t *buf)
{
	int i;
	int retval;

	for (i = 0; i < count; i++) {
		retval = ejtag_dma_read_b(ejtag_info, addr + i * sizeof(*buf), &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips32_dmaacc_write_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, const void *buf)
{
	switch (size) {
		case 1:
			return mips32_dmaacc_write_mem8(ejtag_info, addr, count, buf);
		case 2:
			return mips32_dmaacc_write_mem16(ejtag_info, addr, count, buf);
		case 4:
			return mips32_dmaacc_write_mem32(ejtag_info, addr, count, buf);
	}

	return ERROR_OK;
}

static int mips32_dmaacc_write_mem32(struct mips_ejtag *ejtag_info, uint32_t addr, int count, const uint32_t *buf)
{
	int i;
	int retval;

	for (i = 0; i < count; i++) {
		retval = ejtag_dma_write(ejtag_info, addr + i * sizeof(*buf), buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mips32_dmaacc_write_mem16(struct mips_ejtag *ejtag_info, uint32_t addr, int count, const uint16_t *buf)
{
	int i;
	int retval;

	for (i = 0; i < count; i++) {
		retval = ejtag_dma_write_h(ejtag_info, addr + i * sizeof(*buf), buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mips32_dmaacc_write_mem8(struct mips_ejtag *ejtag_info, uint32_t addr, int count, const uint8_t *buf)
{
	int i;
	int retval;

	for (i = 0; i < count; i++) {
		retval = ejtag_dma_write_b(ejtag_info, addr + i * sizeof(*buf), buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

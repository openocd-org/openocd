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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include "log.h"
#include "mips32.h"
#include "mips32_dmaacc.h"

/*
 * The following logic shamelessly cloned from HairyDairyMaid's wrt54g_debrick
 * to support the Broadcom BCM5352 SoC in the Linksys WRT54GL wireless router
 * (and any others that support EJTAG DMA transfers).
 * Note: This only supports memory read/write. Since the BCM5352 doesn't
 * appear to support PRACC accesses, all debug functions except halt
 * do not work.  Still, this does allow erasing/writing flash as well as
 * displaying/modifying memory and memory mapped registers.
 */

static int ejtag_dma_read(mips_ejtag_t *ejtag_info, u32 addr, u32 *data)
{
	u32 v;
	u32 ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Read & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DRWN | EJTAG_CTRL_DMA_WORD | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	do {
		ejtag_ctrl = EJTAG_CTRL_DMAACC | ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	} while(ejtag_ctrl & EJTAG_CTRL_DSTRT);

	/* Read Data */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA, NULL);
	mips_ejtag_drscan_32(ejtag_info, data);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR)
	{
		if (retries--) {
			LOG_ERROR("DMA Read Addr = %08x  Data = ERROR ON READ (retrying)\n", addr);
			goto begin_ejtag_dma_read;
		}
		else
			LOG_ERROR("DMA Read Addr = %08x  Data = ERROR ON READ\n", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int ejtag_dma_read_h(mips_ejtag_t *ejtag_info, u32 addr, u16 *data)
{
	u32 v;
	u32 ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read_h:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Read & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DRWN | EJTAG_CTRL_DMA_HALFWORD | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	do {
		ejtag_ctrl = EJTAG_CTRL_DMAACC | ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	} while(ejtag_ctrl & EJTAG_CTRL_DSTRT);

	/* Read Data */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR)
	{
		if (retries--) {
			LOG_ERROR("DMA Read Addr = %08x  Data = ERROR ON READ (retrying)\n", addr);
			goto begin_ejtag_dma_read_h;
		}
		else
			LOG_ERROR("DMA Read Addr = %08x  Data = ERROR ON READ\n", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Handle the bigendian/littleendian */
	if (addr & 0x2)
		*data = (v >> 16) & 0xffff;
	else
		*data = (v & 0x0000ffff);

	return ERROR_OK;
}

static int ejtag_dma_read_b(mips_ejtag_t *ejtag_info, u32 addr, u8 *data)
{
	u32 v;
	u32 ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read_b:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Read & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DRWN | EJTAG_CTRL_DMA_BYTE | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	do {
		ejtag_ctrl = EJTAG_CTRL_DMAACC | ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	} while(ejtag_ctrl & EJTAG_CTRL_DSTRT);

	/* Read Data */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR)
	{
		if (retries--) {
			LOG_ERROR("DMA Read Addr = %08x  Data = ERROR ON READ (retrying)\n", addr);
			goto begin_ejtag_dma_read_b;
		}
		else
			LOG_ERROR("DMA Read Addr = %08x  Data = ERROR ON READ\n", addr);
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

static int ejtag_dma_write(mips_ejtag_t *ejtag_info, u32 addr, u32 data)
{
	u32 v;
	u32 ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_write:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Setup Data */
	v = data;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Write & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DMA_WORD | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	do {
		ejtag_ctrl = EJTAG_CTRL_DMAACC | ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	} while(ejtag_ctrl & EJTAG_CTRL_DSTRT);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR)
	{
		if (retries--) {
			LOG_ERROR("DMA Write Addr = %08x  Data = ERROR ON WRITE (retrying)\n", addr);
			goto begin_ejtag_dma_write;
		}
		else
			LOG_ERROR("DMA Write Addr = %08x  Data = ERROR ON WRITE\n", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int ejtag_dma_write_h(mips_ejtag_t *ejtag_info, u32 addr, u32 data)
{
	u32 v;
	u32 ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

	/* Handle the bigendian/littleendian */
	data &= 0xffff;
	data |= data << 16;

begin_ejtag_dma_write_h:

	/* Setup Address */
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Setup Data */
	v = data;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Write & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DMA_HALFWORD | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	do {
		ejtag_ctrl = EJTAG_CTRL_DMAACC | ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	} while(ejtag_ctrl & EJTAG_CTRL_DSTRT);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl  & EJTAG_CTRL_DERR)
	{
		if (retries--) {
			LOG_ERROR("DMA Write Addr = %08x  Data = ERROR ON WRITE (retrying)\n", addr);
			goto begin_ejtag_dma_write_h;
		}
		else
			LOG_ERROR("DMA Write Addr = %08x  Data = ERROR ON WRITE\n", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int ejtag_dma_write_b(mips_ejtag_t *ejtag_info, u32 addr, u32 data)
{
	u32 v;
	u32 ejtag_ctrl;
	int retries = RETRY_ATTEMPTS;

	/* Handle the bigendian/littleendian */
	data &= 0xff;
	data |= data << 8;
	data |= data << 16;

begin_ejtag_dma_write_b:

	/*  Setup Address*/
	v = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Setup Data */
	v = data;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA, NULL);
	mips_ejtag_drscan_32(ejtag_info, &v);

	/* Initiate DMA Write & set DSTRT */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = EJTAG_CTRL_DMAACC | EJTAG_CTRL_DMA_BYTE | EJTAG_CTRL_DSTRT | ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* Wait for DSTRT to Clear */
	do {
		ejtag_ctrl = EJTAG_CTRL_DMAACC | ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	} while(ejtag_ctrl & EJTAG_CTRL_DSTRT);

	/* Clear DMA & Check DERR */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (ejtag_ctrl & EJTAG_CTRL_DERR)
	{
		if (retries--) {
			LOG_ERROR("DMA Write Addr = %08x  Data = ERROR ON WRITE (retrying)\n", addr);
			goto begin_ejtag_dma_write_b;
		}
		else
			LOG_ERROR("DMA Write Addr = %08x  Data = ERROR ON WRITE\n", addr);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

int mips32_dmaacc_read_mem(mips_ejtag_t *ejtag_info, u32 addr, int size, int count, void *buf)
{
	switch (size)
	{
		case 1:
			return mips32_dmaacc_read_mem8(ejtag_info, addr, count, (u8*)buf);
		case 2:
			return mips32_dmaacc_read_mem16(ejtag_info, addr, count, (u16*)buf);
		case 4:
			return mips32_dmaacc_read_mem32(ejtag_info, addr, count, (u32*)buf);
	}

	return ERROR_OK;
}

int mips32_dmaacc_read_mem32(mips_ejtag_t *ejtag_info, u32 addr, int count, u32 *buf)
{
	int i;
	int	retval;

	for (i=0; i<count; i++) {
		if ((retval = ejtag_dma_read(ejtag_info, addr+i*sizeof(*buf), &buf[i])) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips32_dmaacc_read_mem16(mips_ejtag_t *ejtag_info, u32 addr, int count, u16 *buf)
{
	int i;
	int retval;

	for (i=0; i<count; i++) {
		if ((retval = ejtag_dma_read_h(ejtag_info, addr+i*sizeof(*buf), &buf[i])) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips32_dmaacc_read_mem8(mips_ejtag_t *ejtag_info, u32 addr, int count, u8 *buf)
{
	int i;
	int retval;

	for (i=0; i<count; i++) {
		if ((retval = ejtag_dma_read_b(ejtag_info, addr+i*sizeof(*buf), &buf[i])) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips32_dmaacc_write_mem(mips_ejtag_t *ejtag_info, u32 addr, int size, int count, void *buf)
{
	switch (size)
	{
		case 1:
			return mips32_dmaacc_write_mem8(ejtag_info, addr, count, (u8*)buf);
		case 2:
			return mips32_dmaacc_write_mem16(ejtag_info, addr, count,(u16*)buf);
		case 4:
			return mips32_dmaacc_write_mem32(ejtag_info, addr, count, (u32*)buf);
	}

	return ERROR_OK;
}

int mips32_dmaacc_write_mem32(mips_ejtag_t *ejtag_info, u32 addr, int count, u32 *buf)
{
	int i;
	int retval;

	for (i=0; i<count; i++) {
		if ((retval = ejtag_dma_write(ejtag_info, addr+i*sizeof(*buf), buf[i])) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips32_dmaacc_write_mem16(mips_ejtag_t *ejtag_info, u32 addr, int count, u16 *buf)
{
	int i;
	int retval;

	for (i=0; i<count; i++) {
		if ((retval = ejtag_dma_write_h(ejtag_info, addr+i*sizeof(*buf), buf[i])) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips32_dmaacc_write_mem8(mips_ejtag_t *ejtag_info, u32 addr, int count, u8 *buf)
{
	int i;
	int retval;

	for (i=0; i<count; i++) {
		if ((retval = ejtag_dma_write_b(ejtag_info, addr+i*sizeof(*buf), buf[i])) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009 by Oyvind Harboe                                   *
 *   oyvind.harboe@zylin.com                                               *
 *																		   *
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
/***************************************************************************
 *                                                                         *
 * This file implements support for the ARM Debug Interface v5  (ADI_V5)   *
 *                                                                         *
 * ARM(tm) Debug Interface v5 Architecture Specification    ARM IHI 0031A  *
 *                                                                         *
 * CoreSight(tm) DAP-Lite TRM, ARM DDI 0316A                               *
 * Cortex-M3(tm) TRM, ARM DDI 0337C                                        *
 *                                                                         *
***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm_adi_v5.h"
#include "time_support.h"

/*
 * Transaction Mode:
 * swjdp->trans_mode = TRANS_MODE_COMPOSITE;
 * Uses Overrun checking mode and does not do actual JTAG send/receive or transaction
 * result checking until swjdp_end_transaction()
 * This must be done before using or deallocating any return variables.
 * swjdp->trans_mode == TRANS_MODE_ATOMIC
 * All reads and writes to the AHB bus are checked for valid completion, and return values
 * are immediatley available.
*/

/***************************************************************************
 *                                                                         *
 * DPACC and APACC scanchain access through JTAG-DP                        *
 *                                                                         *
***************************************************************************/

/* Scan out and in from target ordered u8 buffers */
int adi_jtag_dp_scan(swjdp_common_t *swjdp, u8 instr, u8 reg_addr, u8 RnW, u8 *outvalue, u8 *invalue, u8 *ack)
{
	arm_jtag_t *jtag_info = swjdp->jtag_info;
	scan_field_t fields[2];
	u8 out_addr_buf;

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	/* Add specified number of tck clocks before accessing memory bus */
	if ((instr == DAP_IR_APACC) && ((reg_addr == AP_REG_DRW)||((reg_addr&0xF0) == AP_REG_BD0) )&& (swjdp->memaccess_tck != 0))
		jtag_add_runtest(swjdp->memaccess_tck, jtag_add_end_state(TAP_IDLE));

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].in_value = ack;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = outvalue;
	fields[1].in_value = invalue;

	jtag_add_dr_scan(2, fields, jtag_get_end_state());

	return ERROR_OK;
}

/* Scan out and in from host ordered u32 variables */
int adi_jtag_dp_scan_u32(swjdp_common_t *swjdp, u8 instr, u8 reg_addr, u8 RnW, u32 outvalue, u32 *invalue, u8 *ack)
{
	arm_jtag_t *jtag_info = swjdp->jtag_info;
	scan_field_t fields[2];
	u8 out_value_buf[4];
	u8 out_addr_buf;

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	/* Add specified number of tck clocks before accessing memory bus */
	if ((instr == DAP_IR_APACC) && ((reg_addr == AP_REG_DRW)||((reg_addr&0xF0) == AP_REG_BD0) )&& (swjdp->memaccess_tck != 0))
		jtag_add_runtest(swjdp->memaccess_tck, jtag_add_end_state(TAP_IDLE));

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].in_value = ack;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	buf_set_u32(out_value_buf, 0, 32, outvalue);
	fields[1].out_value = out_value_buf;
	fields[1].in_value = NULL;

	if (invalue)
	{
		fields[1].in_value = (u8 *)invalue;
		jtag_add_dr_scan(2, fields, jtag_get_end_state());

		jtag_add_callback(arm_le_to_h_u32, (u8 *)invalue);
	} else
	{

		jtag_add_dr_scan(2, fields, jtag_get_end_state());
	}

	return ERROR_OK;
}

/* scan_inout_check adds one extra inscan for DPAP_READ commands to read variables */
int scan_inout_check(swjdp_common_t *swjdp, u8 instr, u8 reg_addr, u8 RnW, u8 *outvalue, u8 *invalue)
{
	adi_jtag_dp_scan(swjdp, instr, reg_addr, RnW, outvalue, NULL, NULL);

	if ((RnW == DPAP_READ) && (invalue != NULL))
	{
		adi_jtag_dp_scan(swjdp, DAP_IR_DPACC, DP_RDBUFF, DPAP_READ, 0, invalue, &swjdp->ack);
	}

	/* In TRANS_MODE_ATOMIC all DAP_IR_APACC transactions wait for ack=OK/FAULT and the check CTRL_STAT */
	if ((instr == DAP_IR_APACC) && (swjdp->trans_mode == TRANS_MODE_ATOMIC))
	{
		return swjdp_transaction_endcheck(swjdp);
	}

	return ERROR_OK;
}

int scan_inout_check_u32(swjdp_common_t *swjdp, u8 instr, u8 reg_addr, u8 RnW, u32 outvalue, u32 *invalue)
{
	adi_jtag_dp_scan_u32(swjdp, instr, reg_addr, RnW, outvalue, NULL, NULL);

	if ((RnW==DPAP_READ) && (invalue != NULL))
	{
		adi_jtag_dp_scan_u32(swjdp, DAP_IR_DPACC, DP_RDBUFF, DPAP_READ, 0, invalue, &swjdp->ack);
	}

	/* In TRANS_MODE_ATOMIC all DAP_IR_APACC transactions wait for ack=OK/FAULT and then check CTRL_STAT */
	if ((instr == DAP_IR_APACC) && (swjdp->trans_mode == TRANS_MODE_ATOMIC))
	{
		return swjdp_transaction_endcheck(swjdp);
	}

	return ERROR_OK;
}

int swjdp_transaction_endcheck(swjdp_common_t *swjdp)
{
	int retval;
	u32 ctrlstat;

	/* too expensive to call keep_alive() here */

#if 0
	/* Danger!!!! BROKEN!!!! */
	scan_inout_check_u32(swjdp, DAP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	/* Danger!!!! BROKEN!!!! Why will jtag_execute_queue() fail here????
	R956 introduced the check on return value here and now Michael Schwingen reports
	that this code no longer works....

	https://lists.berlios.de/pipermail/openocd-development/2008-September/003107.html
	*/
	if ((retval=jtag_execute_queue())!=ERROR_OK)
	{
		LOG_ERROR("BUG: Why does this fail the first time????");
	}
	/* Why??? second time it works??? */
#endif

	scan_inout_check_u32(swjdp, DAP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	if ((retval=jtag_execute_queue())!=ERROR_OK)
		return retval;

	swjdp->ack = swjdp->ack & 0x7;

	if (swjdp->ack != 2)
	{
		long long then=timeval_ms();
		while (swjdp->ack != 2)
		{
			if (swjdp->ack == 1)
			{
				if ((timeval_ms()-then) > 1000)
				{
					LOG_WARNING("Timeout (1000ms) waiting for ACK = OK/FAULT in SWJDP transaction");
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}
			else
			{
				LOG_WARNING("Invalid ACK in SWJDP transaction");
				return ERROR_JTAG_DEVICE_ERROR;
			}

			scan_inout_check_u32(swjdp, DAP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if ((retval=jtag_execute_queue())!=ERROR_OK)
				return retval;
			swjdp->ack = swjdp->ack & 0x7;
		}
	} else
	{
		/* common code path avoids fn to timeval_ms() */
	}

	/* Check for STICKYERR and STICKYORUN */
	if (ctrlstat & (SSTICKYORUN | SSTICKYERR))
	{
		LOG_DEBUG("swjdp: CTRL/STAT error 0x%x", ctrlstat);
		/* Check power to debug regions */
		if ((ctrlstat & 0xf0000000) != 0xf0000000)
		{
			 ahbap_debugport_init(swjdp);
		}
		else
		{
			u32 mem_ap_csw, mem_ap_tar;

			/* Print information about last AHBAP access */
			LOG_ERROR("AHBAP Cached values: dp_select 0x%x, ap_csw 0x%x, ap_tar 0x%x", swjdp->dp_select_value, swjdp->ap_csw_value, swjdp->ap_tar_value);
			if (ctrlstat & SSTICKYORUN)
				LOG_ERROR("SWJ-DP OVERRUN - check clock or reduce jtag speed");

			if (ctrlstat & SSTICKYERR)
				LOG_ERROR("SWJ-DP STICKY ERROR");

			/* Clear Sticky Error Bits */
			scan_inout_check_u32(swjdp, DAP_IR_DPACC, DP_CTRL_STAT, DPAP_WRITE, swjdp->dp_ctrl_stat | SSTICKYORUN | SSTICKYERR, NULL);
			scan_inout_check_u32(swjdp, DAP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if ((retval=jtag_execute_queue())!=ERROR_OK)
				return retval;

			LOG_DEBUG("swjdp: status 0x%x", ctrlstat);

			dap_ap_read_reg_u32(swjdp, AP_REG_CSW, &mem_ap_csw);
			dap_ap_read_reg_u32(swjdp, AP_REG_TAR, &mem_ap_tar);
			if ((retval=jtag_execute_queue())!=ERROR_OK)
				return retval;
			LOG_ERROR("Read MEM_AP_CSW 0x%x, MEM_AP_TAR 0x%x", mem_ap_csw, mem_ap_tar);

		}
		if ((retval=jtag_execute_queue())!=ERROR_OK)
			return retval;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/***************************************************************************
 *                                                                         *
 * DP and MEM-AP  register access  through APACC and DPACC                 *
 *                                                                         *
***************************************************************************/

int dap_dp_write_reg(swjdp_common_t *swjdp, u32 value, u8 reg_addr)
{
	return scan_inout_check_u32(swjdp, DAP_IR_DPACC, reg_addr, DPAP_WRITE, value, NULL);
}

int dap_dp_read_reg(swjdp_common_t *swjdp, u32 *value, u8 reg_addr)
{
	return scan_inout_check_u32(swjdp, DAP_IR_DPACC, reg_addr, DPAP_READ, 0, value);
}

int dap_ap_select(swjdp_common_t *swjdp,u8 apsel)
{
	u32 select;
	select = (apsel<<24) & 0xFF000000;

	if (select != swjdp->apsel)
	{
		swjdp->apsel = select;
		/* Switching AP invalidates cached values */
		swjdp->dp_select_value = -1;
		swjdp->ap_csw_value = -1;
		swjdp->ap_tar_value = -1;
	}

	return ERROR_OK;
}

int dap_dp_bankselect(swjdp_common_t *swjdp,u32 ap_reg)
{
	u32 select;
	select = (ap_reg & 0x000000F0);

	if (select != swjdp->dp_select_value)
	{
		dap_dp_write_reg(swjdp, select | swjdp->apsel, DP_SELECT);
		swjdp->dp_select_value = select;
	}

	return ERROR_OK;
}

int dap_ap_write_reg(swjdp_common_t *swjdp, u32 reg_addr, u8* out_value_buf)
{
	dap_dp_bankselect(swjdp, reg_addr);
	scan_inout_check(swjdp, DAP_IR_APACC, reg_addr, DPAP_WRITE, out_value_buf, NULL);

	return ERROR_OK;
}

int dap_ap_read_reg(swjdp_common_t *swjdp, u32 reg_addr, u8 *in_value_buf)
{
	dap_dp_bankselect(swjdp, reg_addr);
	scan_inout_check(swjdp, DAP_IR_APACC, reg_addr, DPAP_READ, 0, in_value_buf);

	return ERROR_OK;
}
int dap_ap_write_reg_u32(swjdp_common_t *swjdp, u32 reg_addr, u32 value)
{
	u8 out_value_buf[4];

	buf_set_u32(out_value_buf, 0, 32, value);
	dap_dp_bankselect(swjdp, reg_addr);
	scan_inout_check(swjdp, DAP_IR_APACC, reg_addr, DPAP_WRITE, out_value_buf, NULL);

	return ERROR_OK;
}

int dap_ap_read_reg_u32(swjdp_common_t *swjdp, u32 reg_addr, u32 *value)
{
	dap_dp_bankselect(swjdp, reg_addr);
	scan_inout_check_u32(swjdp, DAP_IR_APACC, reg_addr, DPAP_READ, 0, value);

	return ERROR_OK;
}

/***************************************************************************
 *                                                                         *
 * AHB-AP access to memory and system registers on AHB bus                 *
 *                                                                         *
***************************************************************************/

int dap_setup_accessport(swjdp_common_t *swjdp, u32 csw, u32 tar)
{
	csw = csw | CSW_DBGSWENABLE | CSW_MASTER_DEBUG | CSW_HPROT;
	if (csw != swjdp->ap_csw_value)
	{
		/* LOG_DEBUG("swjdp : Set CSW %x",csw); */
		dap_ap_write_reg_u32(swjdp, AP_REG_CSW, csw );
		swjdp->ap_csw_value = csw;
	}
	if (tar != swjdp->ap_tar_value)
	{
		/* LOG_DEBUG("swjdp : Set TAR %x",tar); */
		dap_ap_write_reg_u32(swjdp, AP_REG_TAR, tar );
		swjdp->ap_tar_value = tar;
	}
	if (csw & CSW_ADDRINC_MASK)
	{
		/* Do not cache TAR value when autoincrementing */
		swjdp->ap_tar_value = -1;
	}
	return ERROR_OK;
}

/*****************************************************************************
*                                                                            *
* mem_ap_read_u32(swjdp_common_t *swjdp, u32 address, u32 *value)      *
*                                                                            *
* Read a u32 value from memory or system register                            *
* Functionally equivalent to target_read_u32(target, address, u32 *value),   *
* but with less overhead                                                     *
*****************************************************************************/
int mem_ap_read_u32(swjdp_common_t *swjdp, u32 address, u32 *value)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, address & 0xFFFFFFF0);
	dap_ap_read_reg_u32(swjdp, AP_REG_BD0 | (address & 0xC), value );

	return ERROR_OK;
}

int mem_ap_read_atomic_u32(swjdp_common_t *swjdp, u32 address, u32 *value)
{
	mem_ap_read_u32(swjdp, address, value);

	return swjdp_transaction_endcheck(swjdp);
}

/*****************************************************************************
*                                                                            *
* mem_ap_write_u32(swjdp_common_t *swjdp, u32 address, u32 value)      *
*                                                                            *
* Write a u32 value to memory or memory mapped register                              *
*                                                                            *
*****************************************************************************/
int mem_ap_write_u32(swjdp_common_t *swjdp, u32 address, u32 value)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, address & 0xFFFFFFF0);
	dap_ap_write_reg_u32(swjdp, AP_REG_BD0 | (address & 0xC), value );

	return ERROR_OK;
}

int mem_ap_write_atomic_u32(swjdp_common_t *swjdp, u32 address, u32 value)
{
	mem_ap_write_u32(swjdp, address, value);

	return swjdp_transaction_endcheck(swjdp);
}

/*****************************************************************************
*                                                                            *
* mem_ap_write_buf(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address) *
*                                                                            *
* Write a buffer in target order (little endian)                             *
*                                                                            *
*****************************************************************************/
int mem_ap_write_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	int wcount, blocksize, writecount, errorcount = 0, retval = ERROR_OK;
	u32 adr = address;
	u8* pBuffer = buffer;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	count >>= 2;
	wcount = count;

	/* if we have an unaligned access - reorder data */
	if (adr & 0x3u)
	{
		for (writecount = 0; writecount < count; writecount++)
		{
			int i;
			u32 outvalue;
			memcpy(&outvalue, pBuffer, sizeof(u32));

			for (i = 0; i < 4; i++ )
			{
				*((u8*)pBuffer + (adr & 0x3)) = outvalue;
				outvalue >>= 8;
				adr++;
			}
			pBuffer += sizeof(u32);
		}
	}

	while (wcount > 0)
	{
		/* Adjust to write blocks within 4K aligned boundaries */
		blocksize = (0x1000 - (0xFFF & address)) >> 2;
		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);

		for (writecount = 0; writecount < blocksize; writecount++)
		{
			dap_ap_write_reg(swjdp, AP_REG_DRW, buffer + 4 * writecount );
		}

		if (swjdp_transaction_endcheck(swjdp) == ERROR_OK)
		{
			wcount = wcount - blocksize;
			address = address + 4 * blocksize;
			buffer = buffer + 4 * blocksize;
		}
		else
		{
			errorcount++;
		}

		if (errorcount > 1)
		{
			LOG_WARNING("Block write error address 0x%x, wcount 0x%x", address, wcount);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	return retval;
}

int mem_ap_write_buf_packed_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	int retval = ERROR_OK;
	int wcount, blocksize, writecount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count >> 1;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address)) >> 1;

		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		dap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_PACKED, address);
		writecount = blocksize;

		do
		{
			nbytes = MIN((writecount << 1), 4);

			if (nbytes < 4 )
			{
				if (mem_ap_write_buf_u16(swjdp, buffer, nbytes, address) != ERROR_OK)
				{
					LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}

				address += nbytes >> 1;
			}
			else
			{
				u32 outvalue;
				memcpy(&outvalue, buffer, sizeof(u32));

				for (i = 0; i < nbytes; i++ )
				{
					*((u8*)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				memcpy(&outvalue, buffer, sizeof(u32));
				dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue);
				if (swjdp_transaction_endcheck(swjdp) != ERROR_OK)
				{
					LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}

			buffer += nbytes >> 1;
			writecount -= nbytes >> 1;

		} while (writecount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_write_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_write_buf_packed_u16(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		u16 svalue;
		memcpy(&svalue, buffer, sizeof(u16));
		u32 outvalue = (u32)svalue << 8 * (address & 0x3);
		dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue );
		retval = swjdp_transaction_endcheck(swjdp);
		count -= 2;
		address += 2;
		buffer += 2;
	}

	return retval;
}

int mem_ap_write_buf_packed_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	int retval = ERROR_OK;
	int wcount, blocksize, writecount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address));

		if (wcount < blocksize)
			blocksize = wcount;

		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		writecount = blocksize;

		do
		{
			nbytes = MIN(writecount, 4);

			if (nbytes < 4 )
			{
				if (mem_ap_write_buf_u8(swjdp, buffer, nbytes, address) != ERROR_OK)
				{
					LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}

				address += nbytes;
			}
			else
			{
				u32 outvalue;
				memcpy(&outvalue, buffer, sizeof(u32));

				for (i = 0; i < nbytes; i++ )
				{
					*((u8*)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				memcpy(&outvalue, buffer, sizeof(u32));
				dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue);
				if (swjdp_transaction_endcheck(swjdp) != ERROR_OK)
				{
					LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}

			buffer += nbytes;
			writecount -= nbytes;

		} while (writecount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_write_buf_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_write_buf_packed_u8(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		u32 outvalue = (u32)*buffer << 8 * (address & 0x3);
		dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue );
		retval = swjdp_transaction_endcheck(swjdp);
		count--;
		address++;
		buffer++;
	}

	return retval;
}

/*********************************************************************************
*                                                                                *
* mem_ap_read_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)  *
*                                                                                *
* Read block fast in target order (little endian) into a buffer                  *
*                                                                                *
**********************************************************************************/
int mem_ap_read_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	int wcount, blocksize, readcount, errorcount = 0, retval = ERROR_OK;
	u32 adr = address;
	u8* pBuffer = buffer;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	count >>= 2;
	wcount = count;

	while (wcount > 0)
	{
		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address)) >> 2;
		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);

		/* Scan out first read */
		adi_jtag_dp_scan(swjdp, DAP_IR_APACC, AP_REG_DRW, DPAP_READ, 0, NULL, NULL);
		for (readcount = 0; readcount < blocksize - 1; readcount++)
		{
			/* Scan out read instruction and scan in previous value */
			adi_jtag_dp_scan(swjdp, DAP_IR_APACC, AP_REG_DRW, DPAP_READ, 0, buffer + 4 * readcount, &swjdp->ack);
		}

		/* Scan in last value */
		adi_jtag_dp_scan(swjdp, DAP_IR_DPACC, DP_RDBUFF, DPAP_READ, 0, buffer + 4 * readcount, &swjdp->ack);
		if (swjdp_transaction_endcheck(swjdp) == ERROR_OK)
		{
			wcount = wcount - blocksize;
			address += 4 * blocksize;
			buffer += 4 * blocksize;
		}
		else
		{
			errorcount++;
		}

		if (errorcount > 1)
		{
			LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	/* if we have an unaligned access - reorder data */
	if (adr & 0x3u)
	{
		for (readcount = 0; readcount < count; readcount++)
		{
			int i;
			u32 data;
			memcpy(&data, pBuffer, sizeof(u32));

			for (i = 0; i < 4; i++ )
			{
				*((u8*)pBuffer) = (data >> 8 * (adr & 0x3));
				pBuffer++;
				adr++;
			}
		}
	}

	return retval;
}

int mem_ap_read_buf_packed_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue;
	int retval = ERROR_OK;
	int wcount, blocksize, readcount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count >> 1;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address)) >> 1;
		if (wcount < blocksize)
			blocksize = wcount;

		dap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_PACKED, address);

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;
		readcount = blocksize;

		do
		{
			dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue );
			if (swjdp_transaction_endcheck(swjdp) != ERROR_OK)
			{
				LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
				return ERROR_JTAG_DEVICE_ERROR;
			}

			nbytes = MIN((readcount << 1), 4);

			for (i = 0; i < nbytes; i++ )
			{
				*((u8*)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}

			readcount -= (nbytes >> 1);
		} while (readcount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_read_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue, i;
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_read_buf_packed_u16(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue );
		retval = swjdp_transaction_endcheck(swjdp);
		if (address & 0x1)
		{
			for (i = 0; i < 2; i++ )
			{
				*((u8*)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}
		}
		else
		{
			u16 svalue = (invalue >> 8 * (address & 0x3));
			memcpy(buffer, &svalue, sizeof(u16));
			address += 2;
			buffer += 2;
		}
		count -= 2;
	}

	return retval;
}

/* FIX!!! is this a potential performance bottleneck w.r.t. requiring too many
 * roundtrips when jtag_execute_queue() has a large overhead(e.g. for USB)s?
 *
 * The solution is to arrange for a large out/in scan in this loop and
 * and convert data afterwards.
 */
int mem_ap_read_buf_packed_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue;
	int retval = ERROR_OK;
	int wcount, blocksize, readcount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address));

		if (wcount < blocksize)
			blocksize = wcount;

		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		readcount = blocksize;

		do
		{
			dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue );
			if (swjdp_transaction_endcheck(swjdp) != ERROR_OK)
			{
				LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
				return ERROR_JTAG_DEVICE_ERROR;
			}

			nbytes = MIN(readcount, 4);

			for (i = 0; i < nbytes; i++ )
			{
				*((u8*)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}

			readcount -= nbytes;
		} while (readcount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_read_buf_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue;
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_read_buf_packed_u8(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue );
		retval = swjdp_transaction_endcheck(swjdp);
		*((u8*)buffer) = (invalue >> 8 * (address & 0x3));
		count--;
		address++;
		buffer++;
	}

	return retval;
}

int ahbap_debugport_init(swjdp_common_t *swjdp)
{
	u32 idreg, romaddr, dummy;
	u32 ctrlstat;
	int cnt = 0;
	int retval;

	LOG_DEBUG(" ");

	swjdp->apsel = 0;
	swjdp->ap_csw_value = -1;
	swjdp->ap_tar_value = -1;
	swjdp->trans_mode = TRANS_MODE_ATOMIC;
	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);
	dap_dp_write_reg(swjdp, SSTICKYERR, DP_CTRL_STAT);
	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);

	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	dap_dp_write_reg(swjdp, swjdp->dp_ctrl_stat, DP_CTRL_STAT);
	dap_dp_read_reg(swjdp, &ctrlstat, DP_CTRL_STAT);
	if ((retval=jtag_execute_queue())!=ERROR_OK)
		return retval;

	/* Check that we have debug power domains activated */
	while (!(ctrlstat & CDBGPWRUPACK) && (cnt++ < 10))
	{
		LOG_DEBUG("swjdp: wait CDBGPWRUPACK");
		dap_dp_read_reg(swjdp, &ctrlstat, DP_CTRL_STAT);
		if ((retval=jtag_execute_queue())!=ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	while (!(ctrlstat & CSYSPWRUPACK) && (cnt++ < 10))
	{
		LOG_DEBUG("swjdp: wait CSYSPWRUPACK");
		dap_dp_read_reg(swjdp, &ctrlstat, DP_CTRL_STAT);
		if ((retval=jtag_execute_queue())!=ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);
	/* With debug power on we can activate OVERRUN checking */
	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ | CORUNDETECT;
	dap_dp_write_reg(swjdp, swjdp->dp_ctrl_stat, DP_CTRL_STAT);
	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);

	dap_ap_read_reg_u32(swjdp, 0xFC, &idreg);
	dap_ap_read_reg_u32(swjdp, 0xF8, &romaddr);

	LOG_DEBUG("AHB-AP ID Register 0x%x, Debug ROM Address 0x%x", idreg, romaddr);

	return ERROR_OK;
}


char * class_description[16] ={
	"Reserved",
	"ROM table","Reserved","Reserved","Reserved","Reserved","Reserved","Reserved","Reserved",
	"CoreSight component","Reserved","Peripheral Test Block","Reserved","DESS","Generic IP component","Non standard layout"};

int dap_info_command(struct command_context_s *cmd_ctx, swjdp_common_t *swjdp, int apsel)
{

	u32 dbgbase,apid;
	int romtable_present = 0;
	u8 mem_ap;
	u32 apselold;

	apselold = swjdp->apsel;
	dap_ap_select(swjdp, apsel);
	dap_ap_read_reg_u32(swjdp, 0xF8, &dbgbase);
	dap_ap_read_reg_u32(swjdp, 0xFC, &apid);
	swjdp_transaction_endcheck(swjdp);
	/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
	mem_ap = ((apid&0x10000)&&((apid&0x0F)!=0));
	command_print(cmd_ctx, "ap identification register 0x%8.8x", apid);
	if (apid)
	{
		switch (apid&0x0F)
		{
			case 0:
				command_print(cmd_ctx, "\tType is jtag-ap");
				break;
			case 1:
				command_print(cmd_ctx, "\tType is mem-ap AHB");
				break;
			case 2:
				command_print(cmd_ctx, "\tType is mem-ap APB");
				break;
			default:
				command_print(cmd_ctx, "\tUnknown AP-type");
			break;
		}
		command_print(cmd_ctx, "ap debugbase 0x%8.8x", dbgbase);
	}
	else
	{
		command_print(cmd_ctx, "No AP found at this apsel 0x%x", apsel);
	}

	romtable_present = ((mem_ap)&&(dbgbase != 0xFFFFFFFF));
	if (romtable_present)
	{
		u32 cid0,cid1,cid2,cid3,memtype,romentry;
		u16 entry_offset;
		/* bit 16 of apid indicates a memory access port */
		if (dbgbase&0x02)
		{
			command_print(cmd_ctx, "\tValid ROM table present");
		}
		else
		{
			command_print(cmd_ctx, "\tROM table in legacy format" );
		}
		/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000)|0xFF0, &cid0);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000)|0xFF4, &cid1);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000)|0xFF8, &cid2);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000)|0xFFC, &cid3);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000)|0xFCC, &memtype);
		swjdp_transaction_endcheck(swjdp);
		command_print(cmd_ctx, "\tCID3 0x%x, CID2 0x%x, CID1 0x%x, CID0, 0x%x",cid3,cid2,cid1,cid0);
		if (memtype&0x01)
		{
			command_print(cmd_ctx, "\tMEMTYPE system memory present on bus");
		}
		else
		{
			command_print(cmd_ctx, "\tMEMTYPE system memory not present. Dedicated debug bus" );
		}

		/* Now we read ROM table entries from dbgbase&0xFFFFF000)|0x000 until we get 0x00000000 */
		entry_offset = 0;
		do
		{
			mem_ap_read_atomic_u32(swjdp, (dbgbase&0xFFFFF000)|entry_offset, &romentry);
			command_print(cmd_ctx, "\tROMTABLE[0x%x] = 0x%x",entry_offset,romentry);
			if (romentry&0x01)
			{
				u32 c_cid0,c_cid1,c_cid2,c_cid3,c_pid0,c_pid1,c_pid2,c_pid3,c_pid4,component_start;
				u32 component_base = (u32)((dbgbase&0xFFFFF000)+(int)(romentry&0xFFFFF000));
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFE0, &c_pid0);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFE4, &c_pid1);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFE8, &c_pid2);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFEC, &c_pid3);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFD0, &c_pid4);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFF0, &c_cid0);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFF4, &c_cid1);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFF8, &c_cid2);
				mem_ap_read_atomic_u32(swjdp, (component_base&0xFFFFF000)|0xFFC, &c_cid3);
				component_start = component_base - 0x1000*(c_pid4>>4);
				command_print(cmd_ctx, "\t\tComponent base address 0x%x, pid4 0x%x, start address 0x%x",component_base,c_pid4,component_start);
				command_print(cmd_ctx, "\t\tComponent cid1 0x%x, class is %s",c_cid1,class_description[(c_cid1>>4)&0xF]); /* Se ARM DDI 0314 C Table 2.2 */
				command_print(cmd_ctx, "\t\tCID3 0x%x, CID2 0x%x, CID1 0x%x, CID0, 0x%x",c_cid3,c_cid2,c_cid1,c_cid0);
				command_print(cmd_ctx, "\t\tPID3 0x%x, PID2 0x%x, PID1 0x%x, PID0, 0x%x",c_pid3,c_pid2,c_pid1,c_pid0);
				/* For CoreSight components,  (c_cid1>>4)&0xF==9 , we also read 0xFC8 DevId and 0xFCC DevType */
			}
			else
			{
				if (romentry)
					command_print(cmd_ctx, "\t\tComponent not present");
				else
					command_print(cmd_ctx, "\t\tEnd of ROM table");
			}
			entry_offset += 4;
		} while (romentry>0);
	}
	else
	{
		command_print(cmd_ctx, "\tNo ROM table present");
	}
	dap_ap_select(swjdp, apselold);

	return ERROR_OK;
}


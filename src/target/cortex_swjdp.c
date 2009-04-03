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
 * CoreSight (Light?) SerialWireJtagDebugPort                              *
 *                                                                         *
 * CoreSight(tm) DAP-Lite TRM, ARM DDI 0316A                                *
 * Cortex-M3(tm) TRM, ARM DDI 0337C                                         *
 *                                                                         *
***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "cortex_m3.h"
#include "cortex_swjdp.h"
#include "jtag.h"
#include "log.h"
#include "time_support.h"
#include <stdlib.h>

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
 * DPACC and APACC scanchain access through JTAG-DR                        *
 *                                                                         *
***************************************************************************/

/* Scan out and in from target ordered u8 buffers */
int swjdp_scan(arm_jtag_t *jtag_info, u8 instr, u8 reg_addr, u8 RnW, u8 *outvalue, u8 *invalue, u8 *ack)
{
	scan_field_t fields[2];
	u8 out_addr_buf;

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = ack;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = outvalue;
	fields[1].out_mask = NULL;
	fields[1].in_value = invalue;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	jtag_add_dr_scan(2, fields, -1);

	return ERROR_OK;
}

/* Scan out and in from host ordered u32 variables */
int swjdp_scan_u32(arm_jtag_t *jtag_info, u8 instr, u8 reg_addr, u8 RnW, u32 outvalue, u32 *invalue, u8 *ack)
{
	scan_field_t fields[2];
	u8 out_value_buf[4];
	u8 out_addr_buf;

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = ack;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	buf_set_u32(out_value_buf, 0, 32, outvalue);
	fields[1].out_value = out_value_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	if (invalue)
	{
		fields[1].in_handler = arm_jtag_buf_to_u32;
		fields[1].in_handler_priv = invalue;
	}
	else
	{
		fields[1].in_handler = NULL;
		fields[1].in_handler_priv = NULL;
	}
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	jtag_add_dr_scan(2, fields, -1);

	return ERROR_OK;
}

/* scan_inout_check adds one extra inscan for DPAP_READ commands to read variables */
int scan_inout_check(swjdp_common_t *swjdp, u8 instr, u8 reg_addr, u8 RnW, u8 *outvalue, u8 *invalue)
{
	swjdp_scan(swjdp->jtag_info, instr, reg_addr, RnW, outvalue, NULL, NULL);
	if ((RnW == DPAP_READ) && (invalue != NULL))
	{
		swjdp_scan(swjdp->jtag_info, SWJDP_IR_DPACC, DP_RDBUFF, DPAP_READ, 0, invalue, &swjdp->ack);
	}

	/* In TRANS_MODE_ATOMIC all SWJDP_IR_APACC transactions wait for ack=OK/FAULT and the check CTRL_STAT */
	if ((instr == SWJDP_IR_APACC) && (swjdp->trans_mode == TRANS_MODE_ATOMIC))
	{
		return swjdp_transaction_endcheck(swjdp);
	}

	return ERROR_OK;
}

int scan_inout_check_u32(swjdp_common_t *swjdp, u8 instr, u8 reg_addr, u8 RnW, u32 outvalue, u32 *invalue)
{
	swjdp_scan_u32(swjdp->jtag_info, instr, reg_addr, RnW, outvalue, NULL, NULL);
	if ((RnW==DPAP_READ) && (invalue != NULL))
	{
		swjdp_scan_u32(swjdp->jtag_info, SWJDP_IR_DPACC, DP_RDBUFF, DPAP_READ, 0, invalue, &swjdp->ack);
	}

	/* In TRANS_MODE_ATOMIC all SWJDP_IR_APACC transactions wait for ack=OK/FAULT and then check CTRL_STAT */
	if ((instr == SWJDP_IR_APACC) && (swjdp->trans_mode == TRANS_MODE_ATOMIC))
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
	scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
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

	scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
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

			scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
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
			u32 dcb_dhcsr,nvic_shcsr, nvic_bfar, nvic_cfsr;

			/* Print information about last AHBAP access */
			LOG_ERROR("AHBAP: dp_select 0x%x, ap_csw 0x%x, ap_tar 0x%x", swjdp->dp_select_value, swjdp->ap_csw_value, swjdp->ap_tar_value);
			if (ctrlstat & SSTICKYORUN)
				LOG_ERROR("SWJ-DP OVERRUN - check clock or reduce jtag speed");

			if (ctrlstat & SSTICKYERR)
				LOG_ERROR("SWJ-DP STICKY ERROR");

			/* Clear Sticky Error Bits */
			scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_WRITE, swjdp->dp_ctrl_stat | SSTICKYORUN | SSTICKYERR, NULL);
			scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if ((retval=jtag_execute_queue())!=ERROR_OK)
				return retval;

			LOG_DEBUG("swjdp: status 0x%x", ctrlstat);

			/* Can we find out the reason for the error ?? */
			ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &dcb_dhcsr);
			ahbap_read_system_atomic_u32(swjdp, NVIC_SHCSR, &nvic_shcsr);
			ahbap_read_system_atomic_u32(swjdp, NVIC_CFSR, &nvic_cfsr);
			ahbap_read_system_atomic_u32(swjdp, NVIC_BFAR, &nvic_bfar);
			LOG_ERROR("dcb_dhcsr 0x%x, nvic_shcsr 0x%x, nvic_cfsr 0x%x, nvic_bfar 0x%x", dcb_dhcsr, nvic_shcsr, nvic_cfsr, nvic_bfar);
		}
		if ((retval=jtag_execute_queue())!=ERROR_OK)
			return retval;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/***************************************************************************
 *                                                                         *
 * DP and AHB-AP  register access  through APACC and DPACC                 *
 *                                                                         *
***************************************************************************/

int swjdp_write_dpacc(swjdp_common_t *swjdp, u32 value, u8 reg_addr)
{
	return scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, reg_addr, DPAP_WRITE, value, NULL);
}

int swjdp_read_dpacc(swjdp_common_t *swjdp, u32 *value, u8 reg_addr)
{
	return scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, reg_addr, DPAP_READ, 0, value);
}

int swjdp_bankselect_apacc(swjdp_common_t *swjdp,u32 reg_addr)
{
	u32 select;
	select = (reg_addr & 0xFF0000F0);

	if (select != swjdp->dp_select_value)
	{
		swjdp_write_dpacc(swjdp, select, DP_SELECT);
		swjdp->dp_select_value = select;
	}

	return ERROR_OK;
}

int ahbap_write_reg(swjdp_common_t *swjdp, u32 reg_addr, u8* out_value_buf)
{
	swjdp_bankselect_apacc(swjdp, reg_addr);
	scan_inout_check(swjdp, SWJDP_IR_APACC, reg_addr, DPAP_WRITE, out_value_buf, NULL);

	return ERROR_OK;
}

int ahbap_read_reg(swjdp_common_t *swjdp, u32 reg_addr, u8 *in_value_buf)
{
	swjdp_bankselect_apacc(swjdp, reg_addr);
	scan_inout_check(swjdp, SWJDP_IR_APACC, reg_addr, DPAP_READ, 0, in_value_buf);

	return ERROR_OK;
}
int ahbap_write_reg_u32(swjdp_common_t *swjdp, u32 reg_addr, u32 value)
{
	u8 out_value_buf[4];

	buf_set_u32(out_value_buf, 0, 32, value);
	swjdp_bankselect_apacc(swjdp, reg_addr);
	scan_inout_check(swjdp, SWJDP_IR_APACC, reg_addr, DPAP_WRITE, out_value_buf, NULL);

	return ERROR_OK;
}

int ahbap_read_reg_u32(swjdp_common_t *swjdp, u32 reg_addr, u32 *value)
{
	swjdp_bankselect_apacc(swjdp, reg_addr);
	scan_inout_check_u32(swjdp, SWJDP_IR_APACC, reg_addr, DPAP_READ, 0, value);

	return ERROR_OK;
}

/***************************************************************************
 *                                                                         *
 * AHB-AP access to memory and system registers on AHB bus                 *
 *                                                                         *
***************************************************************************/

int ahbap_setup_accessport(swjdp_common_t *swjdp, u32 csw, u32 tar)
{
	csw = csw | CSW_DBGSWENABLE | CSW_MASTER_DEBUG | CSW_HPROT;
	if (csw != swjdp->ap_csw_value)
	{
		/* LOG_DEBUG("swjdp : Set CSW %x",csw); */
		ahbap_write_reg_u32(swjdp, AHBAP_CSW, csw );
		swjdp->ap_csw_value = csw;
	}
	if (tar != swjdp->ap_tar_value)
	{
		/* LOG_DEBUG("swjdp : Set TAR %x",tar); */
		ahbap_write_reg_u32(swjdp, AHBAP_TAR, tar );
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
* ahbap_read_system_u32(swjdp_common_t *swjdp, u32 address, u32 *value)      *
*                                                                            *
* Read a u32 value from memory or system register                            *
* Functionally equivalent to target_read_u32(target, address, u32 *value),   *
* but with less overhead                                                     *
*****************************************************************************/
int ahbap_read_system_u32(swjdp_common_t *swjdp, u32 address, u32 *value)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, address & 0xFFFFFFF0);
	ahbap_read_reg_u32(swjdp, AHBAP_BD0 | (address & 0xC), value );

	return ERROR_OK;
}

int ahbap_read_system_atomic_u32(swjdp_common_t *swjdp, u32 address, u32 *value)
{
	ahbap_read_system_u32(swjdp, address, value);

	return swjdp_transaction_endcheck(swjdp);
}

/*****************************************************************************
*                                                                            *
* ahbap_write_system_u32(swjdp_common_t *swjdp, u32 address, u32 value)      *
*                                                                            *
* Write a u32 value to memory or system register                             *
*                                                                            *
*****************************************************************************/
int ahbap_write_system_u32(swjdp_common_t *swjdp, u32 address, u32 value)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, address & 0xFFFFFFF0);
	ahbap_write_reg_u32(swjdp, AHBAP_BD0 | (address & 0xC), value );

	return ERROR_OK;
}

int ahbap_write_system_atomic_u32(swjdp_common_t *swjdp, u32 address, u32 value)
{
	ahbap_write_system_u32(swjdp, address, value);

	return swjdp_transaction_endcheck(swjdp);
}

/*****************************************************************************
*                                                                            *
* ahbap_write_buf(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address) *
*                                                                            *
* Write a buffer in target order (little endian)                             *
*                                                                            *
*****************************************************************************/
int ahbap_write_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 outvalue;
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
			outvalue = *((u32*)pBuffer);

			for (i = 0; i < 4; i++ )
			{
				*((u8*)pBuffer + (adr & 0x3)) = outvalue;
				outvalue >>= 8;
				adr++;
			}
			pBuffer += 4;
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

		ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);

		for (writecount = 0; writecount < blocksize; writecount++)
		{
			ahbap_write_reg(swjdp, AHBAP_DRW, buffer + 4 * writecount );
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

int ahbap_write_buf_packed_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 outvalue;
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

		ahbap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_PACKED, address);
		writecount = blocksize;

		do
		{
			nbytes = MIN((writecount << 1), 4);

			if (nbytes < 4 )
			{
				if (ahbap_write_buf_u16(swjdp, buffer, nbytes, address) != ERROR_OK)
				{
					LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}

				address += nbytes >> 1;
			}
			else
			{
				outvalue = *((u32*)buffer);

				for (i = 0; i < nbytes; i++ )
				{
					*((u8*)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				outvalue = *((u32*)buffer);
				ahbap_write_reg_u32(swjdp, AHBAP_DRW, outvalue);
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

int ahbap_write_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 outvalue;
	int retval = ERROR_OK;

	if (count >= 4)
		return ahbap_write_buf_packed_u16(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		ahbap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		outvalue = *((u16*)buffer) << 8 * (address & 0x3);
		ahbap_write_reg_u32(swjdp, AHBAP_DRW, outvalue );
		retval = swjdp_transaction_endcheck(swjdp);
		count -= 2;
		address += 2;
		buffer += 2;
	}

	return retval;
}

int ahbap_write_buf_packed_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 outvalue;
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

		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		writecount = blocksize;

		do
		{
			nbytes = MIN(writecount, 4);

			if (nbytes < 4 )
			{
				if (ahbap_write_buf_u8(swjdp, buffer, nbytes, address) != ERROR_OK)
				{
					LOG_WARNING("Block read error address 0x%x, count 0x%x", address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}

				address += nbytes;
			}
			else
			{
				outvalue = *((u32*)buffer);

				for (i = 0; i < nbytes; i++ )
				{
					*((u8*)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				outvalue = *((u32*)buffer);
				ahbap_write_reg_u32(swjdp, AHBAP_DRW, outvalue);
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

int ahbap_write_buf_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 outvalue;
	int retval = ERROR_OK;

	if (count >= 4)
		return ahbap_write_buf_packed_u8(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		outvalue = *((u8*)buffer) << 8 * (address & 0x3);
		ahbap_write_reg_u32(swjdp, AHBAP_DRW, outvalue );
		retval = swjdp_transaction_endcheck(swjdp);
		count--;
		address++;
		buffer++;
	}

	return retval;
}

/*********************************************************************************
*                                                                                *
* ahbap_read_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)  *
*                                                                                *
* Read block fast in target order (little endian) into a buffer                  *
*                                                                                *
**********************************************************************************/
int ahbap_read_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
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

		ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);

		/* Scan out first read */
		swjdp_scan(swjdp->jtag_info, SWJDP_IR_APACC, AHBAP_DRW, DPAP_READ, 0, NULL, NULL);
		for (readcount = 0; readcount < blocksize - 1; readcount++)
		{
			/* Scan out read instruction and scan in previous value */
			swjdp_scan(swjdp->jtag_info, SWJDP_IR_APACC, AHBAP_DRW, DPAP_READ, 0, buffer + 4 * readcount, &swjdp->ack);
		}

		/* Scan in last value */
		swjdp_scan(swjdp->jtag_info, SWJDP_IR_DPACC, DP_RDBUFF, DPAP_READ, 0, buffer + 4 * readcount, &swjdp->ack);
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
			u32 data = *((u32*)pBuffer);

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

int ahbap_read_buf_packed_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
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

		ahbap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_PACKED, address);

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;
		readcount = blocksize;

		do
		{
			ahbap_read_reg_u32(swjdp, AHBAP_DRW, &invalue );
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

int ahbap_read_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue, i;
	int retval = ERROR_OK;

	if (count >= 4)
		return ahbap_read_buf_packed_u16(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		ahbap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		ahbap_read_reg_u32(swjdp, AHBAP_DRW, &invalue );
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
			*((u16*)buffer) = (invalue >> 8 * (address & 0x3));
			address += 2;
			buffer += 2;
		}
		count -= 2;
	}

	return retval;
}

int ahbap_read_buf_packed_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
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

		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		readcount = blocksize;

		do
		{
			ahbap_read_reg_u32(swjdp, AHBAP_DRW, &invalue );
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

int ahbap_read_buf_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue;
	int retval = ERROR_OK;

	if (count >= 4)
		return ahbap_read_buf_packed_u8(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		ahbap_read_reg_u32(swjdp, AHBAP_DRW, &invalue );
		retval = swjdp_transaction_endcheck(swjdp);
		*((u8*)buffer) = (invalue >> 8 * (address & 0x3));
		count--;
		address++;
		buffer++;
	}

	return retval;
}

int ahbap_read_coreregister_u32(swjdp_common_t *swjdp, u32 *value, int regnum)
{
	int retval;
	u32 dcrdr;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we gave to save/restore the DCB_DCRDR when used */

	ahbap_read_system_u32(swjdp, DCB_DCRDR, &dcrdr);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	/* ahbap_write_system_u32(swjdp, DCB_DCRSR, regnum); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	ahbap_write_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRSR & 0xC), regnum );

	/* ahbap_read_system_u32(swjdp, DCB_DCRDR, value); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	ahbap_read_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRDR & 0xC), value );

	ahbap_write_system_u32(swjdp, DCB_DCRDR, dcrdr);
	retval = swjdp_transaction_endcheck(swjdp);
	return retval;
}

int ahbap_write_coreregister_u32(swjdp_common_t *swjdp, u32 value, int regnum)
{
	int retval;
	u32 dcrdr;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we gave to save/restore the DCB_DCRDR when used */

	ahbap_read_system_u32(swjdp, DCB_DCRDR, &dcrdr);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	/* ahbap_write_system_u32(swjdp, DCB_DCRDR, core_regs[i]); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	ahbap_write_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRDR & 0xC), value );

	/* ahbap_write_system_u32(swjdp, DCB_DCRSR, i | DCRSR_WnR	); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	ahbap_write_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRSR & 0xC), regnum | DCRSR_WnR );

	ahbap_write_system_u32(swjdp, DCB_DCRDR, dcrdr);
	retval = swjdp_transaction_endcheck(swjdp);
	return retval;
}

int ahbap_debugport_init(swjdp_common_t *swjdp)
{
	u32 idreg, romaddr, dummy;
	u32 ctrlstat;
	int cnt = 0;
	int retval;

	LOG_DEBUG(" ");

	swjdp->ap_csw_value = -1;
	swjdp->ap_tar_value = -1;
	swjdp->trans_mode = TRANS_MODE_ATOMIC;
	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);
	swjdp_write_dpacc(swjdp, SSTICKYERR, DP_CTRL_STAT);
	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);

	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	swjdp_write_dpacc(swjdp, swjdp->dp_ctrl_stat, DP_CTRL_STAT);
	swjdp_read_dpacc(swjdp, &ctrlstat, DP_CTRL_STAT);
	if ((retval=jtag_execute_queue())!=ERROR_OK)
		return retval;

	/* Check that we have debug power domains activated */
	while (!(ctrlstat & CDBGPWRUPACK) && (cnt++ < 10))
	{
		LOG_DEBUG("swjdp: wait CDBGPWRUPACK");
		swjdp_read_dpacc(swjdp, &ctrlstat, DP_CTRL_STAT);
		if ((retval=jtag_execute_queue())!=ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	while (!(ctrlstat & CSYSPWRUPACK) && (cnt++ < 10))
	{
		LOG_DEBUG("swjdp: wait CSYSPWRUPACK");
		swjdp_read_dpacc(swjdp, &ctrlstat, DP_CTRL_STAT);
		if ((retval=jtag_execute_queue())!=ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);
	/* With debug power on we can activate OVERRUN checking */
	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ | CORUNDETECT;
	swjdp_write_dpacc(swjdp, swjdp->dp_ctrl_stat, DP_CTRL_STAT);
	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);

	ahbap_read_reg_u32(swjdp, 0xFC, &idreg);
	ahbap_read_reg_u32(swjdp, 0xF8, &romaddr);

	LOG_DEBUG("AHB-AP ID Register 0x%x, Debug ROM Address 0x%x", idreg, romaddr);

	return ERROR_OK;
}

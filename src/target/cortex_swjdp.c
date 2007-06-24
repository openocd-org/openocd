/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
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
/***************************************************************************
 *                                                                         *
 * CoreSight (Light?) SerialWireJtagDebugPort                              *
 *                                                                         *
 * CoreSight™ DAP-Lite TRM, ARM DDI 0316A                                  *
 * Cortex-M3™ TRM, ARM DDI 0337C                                            *
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
#include <stdlib.h>

/*

Transaction Mode:
swjdp->trans_mode = TRANS_MODE_COMPOSITE;
Uses Overrun checking mode and does not do actual JTAG send/receive or transaction 
result checking until swjdp_end_transaction()
This must be done before using or deallocating any return variables.

swjdp->trans_mode == TRANS_MODE_ATOMIC
All reads and writes to the AHB bus are checked for valid completion, and return values
are immediatley available.

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
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = ack;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = outvalue;
	fields[1].out_mask = NULL;
	fields[1].in_value = invalue;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	jtag_add_dr_scan(2, fields, -1, NULL);

	return ERROR_OK;
}

/* Scan out and in from host ordered u32 variables */
int swjdp_scan_u32(arm_jtag_t *jtag_info, u8 instr, u8 reg_addr, u8 RnW, u32 outvalue, u32 *invalue, u8 *ack)
{
	scan_field_t fields[2];
	u8 out_value_buf[4];
	u8 out_addr_buf;
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = ack;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
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

	jtag_add_dr_scan(2, fields, -1, NULL);

	return ERROR_OK;
}

/* scan_inout_check adds one extra inscan for DPAP_READ commands to read variables */ 
int scan_inout_check(swjdp_common_t *swjdp, u8 instr, u8 reg_addr, u8 RnW, u8 *outvalue, u8 *invalue)
{
	swjdp_scan(swjdp->jtag_info, instr, reg_addr, RnW, outvalue, NULL, NULL);
	if ((RnW == DPAP_READ) && (invalue != NULL))
	{
		swjdp_scan(swjdp->jtag_info, SWJDP_IR_DPACC, 0xC, DPAP_READ, 0, invalue, &swjdp->ack);
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
		swjdp_scan_u32(swjdp->jtag_info, SWJDP_IR_DPACC, 0xC, DPAP_READ, 0, invalue, &swjdp->ack);
	}
	
	/* In TRANS_MODE_ATOMIC all SWJDP_IR_APACC transactions wait for ack=OK/FAULT and the check CTRL_STAT */
	if ((instr == SWJDP_IR_APACC) && (swjdp->trans_mode == TRANS_MODE_ATOMIC))
	{
		return swjdp_transaction_endcheck(swjdp);
	}

	return ERROR_OK;
}

int swjdp_transaction_endcheck(swjdp_common_t *swjdp)
{
	int waitcount = 0;
	u32 ctrlstat;

	scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	jtag_execute_queue();
	
	swjdp->ack = swjdp->ack & 0x7;
	
	while (swjdp->ack != 2)
	{
		if (swjdp->ack == 1)
		{
			waitcount++;
			if (waitcount > 100)
			{
				WARNING("Timeout waiting for ACK = OK/FAULT in SWJDP transaction");
				return ERROR_JTAG_DEVICE_ERROR;
			}
		}
		else
		{
			WARNING("Invalid ACK in SWJDP transaction");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
		jtag_execute_queue();
		swjdp->ack = swjdp->ack & 0x7;
	}

	/* Check for STICKYERR and STICKYORUN */
	if (ctrlstat & (SSTICKYORUN | SSTICKYERR))
	{
		DEBUG("swjdp: CTRL/STAT error 0x%x", ctrlstat);
		/* Check power to debug regions */
		if ((ctrlstat & 0xf0000000) != 0xf0000000)
		{
			 ahbap_debugport_init(swjdp);
		}
		else
		{
			u32 dcb_dhcsr,nvic_shcsr, nvic_bfar, nvic_cfsr;
			
			if (ctrlstat & SSTICKYORUN)
				ERROR("SWJ-DP OVERRUN - check clock or reduce jtag speed");
			
			if (ctrlstat & SSTICKYERR)
				ERROR("SWJ-DP STICKY ERROR");
			
			/* Clear Sticky Error Bits */
			scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_WRITE, swjdp->dp_ctrl_stat | SSTICKYORUN | SSTICKYERR, NULL);
			scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			jtag_execute_queue();

			DEBUG("swjdp: status 0x%x", ctrlstat);
			
			/* Can we find out the reason for the error ?? */			
			ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &dcb_dhcsr);
			ahbap_read_system_atomic_u32(swjdp, NVIC_SHCSR, &nvic_shcsr);
			ahbap_read_system_atomic_u32(swjdp, NVIC_CFSR, &nvic_cfsr);
			ahbap_read_system_atomic_u32(swjdp, NVIC_BFAR, &nvic_bfar);
			ERROR("dcb_dhcsr 0x%x, nvic_shcsr 0x%x, nvic_cfsr 0x%x, nvic_bfar 0x%x", dcb_dhcsr, nvic_shcsr, nvic_cfsr, nvic_bfar);
		}
		jtag_execute_queue();
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
	u8 out_value_buf[4];
	
	buf_set_u32(out_value_buf, 0, 32, value);
	return scan_inout_check(swjdp, SWJDP_IR_DPACC, reg_addr, DPAP_WRITE, out_value_buf, NULL);
}

int swjdp_read_dpacc(swjdp_common_t *swjdp, u32 *value, u8 reg_addr)
{
	scan_inout_check_u32(swjdp, SWJDP_IR_DPACC, reg_addr, DPAP_READ, 0, value);

    return ERROR_OK;
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
		//DEBUG("swjdp : Set CSW %x",csw);
		ahbap_write_reg_u32(swjdp, AHBAP_CSW, csw ); 
		swjdp->ap_csw_value = csw;
	}
	if (tar != swjdp->ap_tar_value)
	{
		//DEBUG("swjdp : Set TAR %x",tar);
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
int ahbap_write_buf(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 outvalue;
	int wcount, blocksize, writecount, errorcount = 0, retval = ERROR_OK;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while ((address & 0x3) && (count > 0))
	{
		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		outvalue = (*buffer++) << 8 * (address & 0x3);
		ahbap_write_reg_u32(swjdp, AHBAP_DRW, outvalue );
		swjdp_transaction_endcheck(swjdp);
		count--;
		address++;
	}
	wcount = count >> 2;
	count = count - 4 * wcount;
	while (wcount > 0)
	{
		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address)) >> 2;
		if (wcount < blocksize)
			blocksize = wcount;
		ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);
		for (writecount=0; writecount<blocksize; writecount++)
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
			WARNING("Block read error address %x, count %x", address, count);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}
	
	while (count > 0)
	{
		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		outvalue = (*buffer++) << 8 * (address & 0x3);
		ahbap_write_reg_u32(swjdp, AHBAP_DRW, outvalue );
		retval = swjdp_transaction_endcheck(swjdp);
		count--;
		address++;
	}

	return retval;
}

int ahbap_write_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 outvalue;
	int retval = ERROR_OK;
	
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

/*****************************************************************************
*                                                                            *
* ahbap_read_buf(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)  *
*                                                                            *
* Read block fast in target order (little endian) into a buffer       *
*                                                                            *
*****************************************************************************/
int ahbap_read_buf(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue;
	int wcount, blocksize, readcount, errorcount = 0, retval = ERROR_OK;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while ((address & 0x3) && (count > 0))
	{
		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		ahbap_read_reg_u32(swjdp, AHBAP_DRW, &invalue);
		swjdp_transaction_endcheck(swjdp);
		*buffer++ = (invalue >> 8 * (address & 0x3)) & 0xFF;
		count--;
		address++;
	}
	wcount = count >> 2;
	count = count - 4 * wcount;
	while (wcount > 0)
	{
		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address)) >> 2;
		if (wcount < blocksize)
			blocksize = wcount;
		ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);
		/* Scan out first read */
		swjdp_scan(swjdp->jtag_info, SWJDP_IR_APACC, AHBAP_DRW, DPAP_READ, 0, NULL, NULL);
		for (readcount = 0; readcount < blocksize - 1; readcount++)
		{
			/* Scan out read instruction and scan in previous value */
			swjdp_scan(swjdp->jtag_info, SWJDP_IR_APACC, AHBAP_DRW, DPAP_READ, 0, buffer + 4 * readcount, &swjdp->ack);
		}
		/* Scan in last value */
		swjdp_scan(swjdp->jtag_info, SWJDP_IR_DPACC, 0xC, DPAP_READ, 0, buffer + 4 * readcount, &swjdp->ack);
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
			WARNING("Block read error address %x, count %x", address, count);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	while (count > 0)
	{
		ahbap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		ahbap_read_reg_u32(swjdp, AHBAP_DRW, &invalue );
		retval = swjdp_transaction_endcheck(swjdp);
		*buffer++ = (invalue >> 8 * (address & 0x3)) & 0xFF;
		count--;
		address++;
	}

	return retval;
}

int ahbap_read_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address)
{
	u32 invalue;
	int retval = ERROR_OK;
	
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;
	
	while (count > 0)
	{
		ahbap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		ahbap_read_reg_u32(swjdp, AHBAP_DRW, &invalue );
		retval = swjdp_transaction_endcheck(swjdp);
		*((u16*)buffer) = (invalue >> 8 * (address & 0x3));
		count -= 2;
		address += 2;
		buffer += 2;
	}

	return retval;
}

int ahbap_block_read_u32(swjdp_common_t *swjdp, u32 *buffer, int count, u32 address)
{
	int readcount, errorcount = 0;
	u32 blockmax, blocksize;
	
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;
	
	while (count > 0)
	{
		/* Adjust to read within 4K block boundaries */
		blocksize = (0x1000 - (0xFFF & address)) >> 2;
		if (count < blocksize)
			blocksize = count;
		ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);
		for (readcount = 0; readcount < blocksize; readcount++)
		{
			ahbap_read_reg_u32(swjdp, AHBAP_DRW, buffer + readcount );
		}
		if (swjdp_transaction_endcheck(swjdp) == ERROR_OK)
		{
			count = count - blocksize;
			address = address + 4 * blocksize;
			buffer = buffer + blocksize;
		}
		else
		{
			errorcount++;
		}
		if (errorcount > 1)
		{
			WARNING("Block read error address %x, count %x", address, count);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	return ERROR_OK;
}

int ahbap_read_coreregister_u32(swjdp_common_t *swjdp, u32 *value, int regnum)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	/* ahbap_write_system_u32(swjdp, DCB_DCRSR, regnum); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	ahbap_write_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRSR & 0xC), regnum );

	/* ahbap_read_system_u32(swjdp, DCB_DCRDR, value); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	ahbap_read_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRDR & 0xC), value );
	
	return swjdp_transaction_endcheck(swjdp);
}

int ahbap_write_coreregister_u32(swjdp_common_t *swjdp, u32 value, int regnum)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;
	
	/* ahbap_write_system_u32(swjdp, DCB_DCRDR, core_regs[i]); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	ahbap_write_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRDR & 0xC), value );

	/* ahbap_write_system_u32(swjdp, DCB_DCRSR, i | DCRSR_WnR	); */
	ahbap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	ahbap_write_reg_u32(swjdp, AHBAP_BD0 | (DCB_DCRSR & 0xC), regnum | DCRSR_WnR );

	return swjdp_transaction_endcheck(swjdp);
}

int ahbap_debugport_init(swjdp_common_t *swjdp)
{
	u32 idreg, romaddr, dummy;
	u32 ctrlstat;
	int cnt = 0;
	
	DEBUG(" ");
	
	swjdp->ap_csw_value = -1;
	swjdp->ap_tar_value = -1;
	swjdp->trans_mode = TRANS_MODE_ATOMIC;
	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);
	swjdp_write_dpacc(swjdp, SSTICKYERR, DP_CTRL_STAT);
	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);
	
	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	swjdp_write_dpacc(swjdp, swjdp->dp_ctrl_stat, DP_CTRL_STAT);
	swjdp_read_dpacc(swjdp, &ctrlstat, DP_CTRL_STAT);
	jtag_execute_queue();

	/* Check that we have debug power domains activated */
	while (!(ctrlstat & CDBGPWRUPACK) && (cnt++ < 10))
	{
		DEBUG("swjdp: wait CDBGPWRUPACK");
		swjdp_read_dpacc(swjdp, &ctrlstat, DP_CTRL_STAT);
		jtag_execute_queue();
		usleep(10000);
	}

	while (!(ctrlstat & CSYSPWRUPACK) && (cnt++ < 10))
	{
		DEBUG("swjdp: wait CSYSPWRUPACK");
		swjdp_read_dpacc(swjdp, &ctrlstat, DP_CTRL_STAT);
		jtag_execute_queue();
		usleep(10000);
	}

	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);
	/* With debug power on we can activate OVERRUN checking */
	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ | CORUNDETECT;
	swjdp_write_dpacc(swjdp, swjdp->dp_ctrl_stat , DP_CTRL_STAT);
	swjdp_read_dpacc(swjdp, &dummy, DP_CTRL_STAT);
	
	ahbap_read_reg_u32(swjdp, 0xFC, &idreg);
	ahbap_read_reg_u32(swjdp, 0xF8, &romaddr);
	
	DEBUG("AHB-AP ID Register 0x%x, Debug ROM Address 0x%x", idreg, romaddr);	
	
	return ERROR_OK;
}

/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009 by Oyvind Harboe                                   *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009-2010 by David Brownell                             *
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

/**
 * @file
 * This file implements support for the ARM Debug Interface version 5 (ADIv5)
 * debugging architecture.  Compared with previous versions, this includes
 * a low pin-count Serial Wire Debug (SWD) alternative to JTAG for message
 * transport, and focusses on memory mapped resources as defined by the
 * CoreSight architecture.
 *
 * A key concept in ADIv5 is the Debug Access Port, or DAP.  A DAP has two
 * basic components:  a Debug Port (DP) transporting messages to and from a
 * debugger, and an Access Port (AP) accessing resources.  Three types of DP
 * are defined.  One uses only JTAG for communication, and is called JTAG-DP.
 * One uses only SWD for communication, and is called SW-DP.  The third can
 * use either SWD or JTAG, and is called SWJ-DP.  The most common type of AP
 * is used to access memory mapped resources and is called a MEM-AP.  Also a
 * JTAG-AP is also defined, bridging to JTAG resources; those are uncommon.
 *
 * @todo Remove modality (queued/nonqueued, via DAP trans_mode) from all
 * procedure interfaces.  Modal programming interfaces are very error prone.
 * Procedures should be either queued, or synchronous.  Otherwise input
 * and output constraints are context-sensitive, and it's hard to know
 * what a block of code will do just by reading it.
 */

/*
 * Relevant specifications from ARM include:
 *
 * ARM(tm) Debug Interface v5 Architecture Specification    ARM IHI 0031A
 * CoreSight(tm) v1.0 Architecture Specification            ARM IHI 0029B
 *
 * CoreSight(tm) DAP-Lite TRM, ARM DDI 0316D
 * Cortex-M3(tm) TRM, ARM DDI 0337G
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm_adi_v5.h"
#include <helper/time_support.h>

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


/* ARM ADI Specification requires at least 10 bits used for TAR autoincrement  */

/*
	uint32_t tar_block_size(uint32_t address)
	Return the largest block starting at address that does not cross a tar block size alignment boundary
*/
static uint32_t max_tar_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	return (tar_autoincr_block - ((tar_autoincr_block - 1) & address)) >> 2;
}

/***************************************************************************
 *                                                                         *
 * DPACC and APACC scanchain access through JTAG-DP                        *
 *                                                                         *
***************************************************************************/

/**
 * Scan DPACC or APACC using target ordered uint8_t buffers.  No endianness
 * conversions are performed.  See section 4.4.3 of the ADIv5 spec, which
 * discusses operations which access these registers.
 *
 * Note that only one scan is performed.  If RnW is set, a separate scan
 * will be needed to collect the data which was read; the "invalue" collects
 * the posted result of a preceding operation, not the current one.
 *
 * @param swjdp the DAP
 * @param instr JTAG_DP_APACC (AP access) or JTAG_DP_DPACC (DP access)
 * @param reg_addr two significant bits; A[3:2]; for APACC access, the
 *	SELECT register has more addressing bits.
 * @param RnW false iff outvalue will be written to the DP or AP
 * @param outvalue points to a 32-bit (little-endian) integer
 * @param invalue NULL, or points to a 32-bit (little-endian) integer
 * @param ack points to where the three bit JTAG_ACK_* code will be stored
 */
static int adi_jtag_dp_scan(struct swjdp_common *swjdp,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint8_t *outvalue, uint8_t *invalue, uint8_t *ack)
{
	struct arm_jtag *jtag_info = swjdp->jtag_info;
	struct scan_field fields[2];
	uint8_t out_addr_buf;

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	/* Add specified number of tck clocks before accessing memory bus */

	/* REVISIT these TCK cycles should be *AFTER*  updating APACC, since
	 * they provide more time for the (MEM) AP to complete the read ...
	 * See "Minimum Response Time" for JTAG-DP, in the ADIv5 spec.
	 */
	if ((instr == JTAG_DP_APACC)
			&& ((reg_addr == AP_REG_DRW)
				|| ((reg_addr & 0xF0) == AP_REG_BD0))
			&& (swjdp->memaccess_tck != 0))
		jtag_add_runtest(swjdp->memaccess_tck, jtag_set_end_state(TAP_IDLE));

	/* Scan out a read or write operation using some DP or AP register.
	 * For APACC access with any sticky error flag set, this is discarded.
	 */
	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].in_value = ack;

	/* NOTE: if we receive JTAG_ACK_WAIT, the previous operation did not
	 * complete; data we write is discarded, data we read is unpredictable.
	 * When overrun detect is active, STICKYORUN is set.
	 */

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = outvalue;
	fields[1].in_value = invalue;

	jtag_add_dr_scan(2, fields, jtag_get_end_state());

	return ERROR_OK;
}

/* Scan out and in from host ordered uint32_t variables */
static int adi_jtag_dp_scan_u32(struct swjdp_common *swjdp,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue, uint8_t *ack)
{
	struct arm_jtag *jtag_info = swjdp->jtag_info;
	struct scan_field fields[2];
	uint8_t out_value_buf[4];
	uint8_t out_addr_buf;

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_set_instr(jtag_info, instr, NULL);

	/* Add specified number of tck clocks before accessing memory bus */

	/* REVISIT these TCK cycles should be *AFTER*  updating APACC, since
	 * they provide more time for the (MEM) AP to complete the read ...
	 */
	if ((instr == JTAG_DP_APACC)
			&& ((reg_addr == AP_REG_DRW)
				|| ((reg_addr & 0xF0) == AP_REG_BD0))
			&& (swjdp->memaccess_tck != 0))
		jtag_add_runtest(swjdp->memaccess_tck, jtag_set_end_state(TAP_IDLE));

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
		fields[1].in_value = (uint8_t *)invalue;
		jtag_add_dr_scan(2, fields, jtag_get_end_state());

		jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t) invalue);
	} else
	{

		jtag_add_dr_scan(2, fields, jtag_get_end_state());
	}

	return ERROR_OK;
}

/* scan_inout_check adds one extra inscan for DPAP_READ commands to read variables */
static int scan_inout_check(struct swjdp_common *swjdp,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint8_t *outvalue, uint8_t *invalue)
{
	adi_jtag_dp_scan(swjdp, instr, reg_addr, RnW, outvalue, NULL, NULL);

	if ((RnW == DPAP_READ) && (invalue != NULL))
		adi_jtag_dp_scan(swjdp, JTAG_DP_DPACC,
				DP_RDBUFF, DPAP_READ, 0, invalue, &swjdp->ack);

	/* In TRANS_MODE_ATOMIC all JTAG_DP_APACC transactions wait for
	 * ack = OK/FAULT and the check CTRL_STAT
	 */
	if ((instr == JTAG_DP_APACC)
			&& (swjdp->trans_mode == TRANS_MODE_ATOMIC))
		return jtagdp_transaction_endcheck(swjdp);

	return ERROR_OK;
}

static int scan_inout_check_u32(struct swjdp_common *swjdp,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue)
{
	/* Issue the read or write */
	adi_jtag_dp_scan_u32(swjdp, instr, reg_addr, RnW, outvalue, NULL, NULL);

	/* For reads,  collect posted value; RDBUFF has no other effect.
	 * Assumes read gets acked with OK/FAULT, and CTRL_STAT says "OK".
	 */
	if ((RnW == DPAP_READ) && (invalue != NULL))
		adi_jtag_dp_scan_u32(swjdp, JTAG_DP_DPACC,
				DP_RDBUFF, DPAP_READ, 0, invalue, &swjdp->ack);

	/* In TRANS_MODE_ATOMIC all JTAG_DP_APACC transactions wait for
	 * ack = OK/FAULT and then check CTRL_STAT
	 */
	if ((instr == JTAG_DP_APACC)
			&& (swjdp->trans_mode == TRANS_MODE_ATOMIC))
		return jtagdp_transaction_endcheck(swjdp);

	return ERROR_OK;
}

int jtagdp_transaction_endcheck(struct swjdp_common *swjdp)
{
	int retval;
	uint32_t ctrlstat;

	/* too expensive to call keep_alive() here */

#if 0
	/* Danger!!!! BROKEN!!!! */
	scan_inout_check_u32(swjdp, JTAG_DP_DPACC,
			DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	/* Danger!!!! BROKEN!!!! Why will jtag_execute_queue() fail here????
	R956 introduced the check on return value here and now Michael Schwingen reports
	that this code no longer works....

	https://lists.berlios.de/pipermail/openocd-development/2008-September/003107.html
	*/
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("BUG: Why does this fail the first time????");
	}
	/* Why??? second time it works??? */
#endif

	/* Post CTRL/STAT read; discard any previous posted read value
	 * but collect its ACK status.
	 */
	scan_inout_check_u32(swjdp, JTAG_DP_DPACC,
			DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;

	swjdp->ack = swjdp->ack & 0x7;

	/* common code path avoids calling timeval_ms() */
	if (swjdp->ack != JTAG_ACK_OK_FAULT)
	{
		long long then = timeval_ms();

		while (swjdp->ack != JTAG_ACK_OK_FAULT)
		{
			if (swjdp->ack == JTAG_ACK_WAIT)
			{
				if ((timeval_ms()-then) > 1000)
				{
					/* NOTE:  this would be a good spot
					 * to use JTAG_DP_ABORT.
					 */
					LOG_WARNING("Timeout (1000ms) waiting "
						"for ACK=OK/FAULT "
						"in JTAG-DP transaction");
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}
			else
			{
				LOG_WARNING("Invalid ACK %#x "
						"in JTAG-DP transaction",
						swjdp->ack);
				return ERROR_JTAG_DEVICE_ERROR;
			}

			scan_inout_check_u32(swjdp, JTAG_DP_DPACC,
					DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if ((retval = jtag_execute_queue()) != ERROR_OK)
				return retval;
			swjdp->ack = swjdp->ack & 0x7;
		}
	}

	/* REVISIT also STICKYCMP, for pushed comparisons (nyet used) */

	/* Check for STICKYERR and STICKYORUN */
	if (ctrlstat & (SSTICKYORUN | SSTICKYERR))
	{
		LOG_DEBUG("jtag-dp: CTRL/STAT error, 0x%" PRIx32, ctrlstat);
		/* Check power to debug regions */
		if ((ctrlstat & 0xf0000000) != 0xf0000000)
			 ahbap_debugport_init(swjdp);
		else
		{
			uint32_t mem_ap_csw, mem_ap_tar;

			/* Maybe print information about last intended
			 * MEM-AP access; but not if autoincrementing.
			 * *Real* CSW and TAR values are always shown.
			 */
			if (swjdp->ap_tar_value != (uint32_t) -1)
				LOG_DEBUG("MEM-AP Cached values: "
					"ap_bank 0x%" PRIx32
					", ap_csw 0x%" PRIx32
					", ap_tar 0x%" PRIx32,
					swjdp->dp_select_value,
					swjdp->ap_csw_value,
					swjdp->ap_tar_value);

			if (ctrlstat & SSTICKYORUN)
				LOG_ERROR("JTAG-DP OVERRUN - check clock, "
					"memaccess, or reduce jtag speed");

			if (ctrlstat & SSTICKYERR)
				LOG_ERROR("JTAG-DP STICKY ERROR");

			/* Clear Sticky Error Bits */
			scan_inout_check_u32(swjdp, JTAG_DP_DPACC,
					DP_CTRL_STAT, DPAP_WRITE,
					swjdp->dp_ctrl_stat | SSTICKYORUN
						| SSTICKYERR, NULL);
			scan_inout_check_u32(swjdp, JTAG_DP_DPACC,
					DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if ((retval = jtag_execute_queue()) != ERROR_OK)
				return retval;

			LOG_DEBUG("jtag-dp: CTRL/STAT 0x%" PRIx32, ctrlstat);

			dap_ap_read_reg_u32(swjdp, AP_REG_CSW, &mem_ap_csw);
			dap_ap_read_reg_u32(swjdp, AP_REG_TAR, &mem_ap_tar);
			if ((retval = jtag_execute_queue()) != ERROR_OK)
				return retval;
			LOG_ERROR("MEM_AP_CSW 0x%" PRIx32 ", MEM_AP_TAR 0x%"
					PRIx32, mem_ap_csw, mem_ap_tar);

		}
		if ((retval = jtag_execute_queue()) != ERROR_OK)
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

static int dap_dp_write_reg(struct swjdp_common *swjdp,
		uint32_t value, uint8_t reg_addr)
{
	return scan_inout_check_u32(swjdp, JTAG_DP_DPACC,
			reg_addr, DPAP_WRITE, value, NULL);
}

static int dap_dp_read_reg(struct swjdp_common *swjdp,
		uint32_t *value, uint8_t reg_addr)
{
	return scan_inout_check_u32(swjdp, JTAG_DP_DPACC,
			reg_addr, DPAP_READ, 0, value);
}

/**
 * Select one of the APs connected to the specified DAP.  The
 * selection is implicitly used with future AP transactions.
 * This is a NOP if the specified AP is already selected.
 *
 * @param swjdp The DAP
 * @param apsel Number of the AP to (implicitly) use with further
 *	transactions.  This normally identifies a MEM-AP.
 */
void dap_ap_select(struct swjdp_common *swjdp,uint8_t apsel)
{
	uint32_t select;
	select = (apsel << 24) & 0xFF000000;

	if (select != swjdp->apsel)
	{
		swjdp->apsel = select;
		/* Switching AP invalidates cached values */
		swjdp->dp_select_value = -1;
		swjdp->ap_csw_value = -1;
		swjdp->ap_tar_value = -1;
	}
}

static int dap_dp_bankselect(struct swjdp_common *swjdp, uint32_t ap_reg)
{
	uint32_t select;
	select = (ap_reg & 0x000000F0);

	if (select != swjdp->dp_select_value)
	{
		dap_dp_write_reg(swjdp, select | swjdp->apsel, DP_SELECT);
		swjdp->dp_select_value = select;
	}

	/* FIXME return any fault code from write() call */
	return ERROR_OK;
}

static int dap_ap_write_reg(struct swjdp_common *swjdp,
		uint32_t reg_addr, uint8_t *out_value_buf)
{
	dap_dp_bankselect(swjdp, reg_addr);
	scan_inout_check(swjdp, JTAG_DP_APACC, reg_addr,
			DPAP_WRITE, out_value_buf, NULL);

	/* FIXME return fault code from above calls */
	return ERROR_OK;
}

/**
 * Write an AP register value.
 * This is synchronous iff the mode is set to ATOMIC, in which
 * case any queued transactions are flushed.
 *
 * @param swjdp The DAP whose currently selected AP will be written.
 * @param reg_addr Eight bit AP register address.
 * @param value Word to be written at reg_addr
 *
 * @return In synchronous mode: ERROR_OK for success, and the register holds
 * the specified value; else a fault code.  In asynchronous mode, a status
 * code reflecting whether the transaction was properly queued.
 */
int dap_ap_write_reg_u32(struct swjdp_common *swjdp,
		uint32_t reg_addr, uint32_t value)
{
	uint8_t out_value_buf[4];

	buf_set_u32(out_value_buf, 0, 32, value);
	dap_dp_bankselect(swjdp, reg_addr);
	scan_inout_check(swjdp, JTAG_DP_APACC, reg_addr,
			DPAP_WRITE, out_value_buf, NULL);

	/* FIXME return any fault code from above calls */
	return ERROR_OK;
}

/**
 * Read an AP register value.
 * This is synchronous iff the mode is set to ATOMIC, in which
 * case any queued transactions are flushed.
 *
 * @param swjdp The DAP whose currently selected AP will be read.
 * @param reg_addr Eight bit AP register address.
 * @param value Points to where the 32-bit (little-endian) word will be stored.
 *
 * @return In synchronous mode: ERROR_OK for success, and *value holds
 * the specified value; else a fault code.  In asynchronous mode, a status
 * code reflecting whether the transaction was properly queued.
 */
int dap_ap_read_reg_u32(struct swjdp_common *swjdp,
		uint32_t reg_addr, uint32_t *value)
{
	dap_dp_bankselect(swjdp, reg_addr);
	scan_inout_check_u32(swjdp, JTAG_DP_APACC, reg_addr,
			DPAP_READ, 0, value);

	/* FIXME return any fault code from above calls */
	return ERROR_OK;
}

/**
 * Set up transfer parameters for the currently selected MEM-AP.
 * This is synchronous iff the mode is set to ATOMIC, in which
 * case any queued transactions are flushed.
 *
 * Subsequent transfers using registers like AP_REG_DRW or AP_REG_BD2
 * initiate data reads or writes using memory or peripheral addresses.
 * If the CSW is configured for it, the TAR may be automatically
 * incremented after each transfer.
 *
 * @todo Rename to reflect it being specifically a MEM-AP function.
 *
 * @param swjdp The DAP connected to the MEM-AP.
 * @param csw MEM-AP Control/Status Word (CSW) register to assign.  If this
 *	matches the cached value, the register is not changed.
 * @param tar MEM-AP Transfer Address Register (TAR) to assign.  If this
 *	matches the cached address, the register is not changed.
 *
 * @return In synchronous mode: ERROR_OK for success, and the AP is set
 * up as requested else a fault code.  In asynchronous mode, a status
 * code reflecting whether the transaction was properly queued.
 */
int dap_setup_accessport(struct swjdp_common *swjdp, uint32_t csw, uint32_t tar)
{
	csw = csw | CSW_DBGSWENABLE | CSW_MASTER_DEBUG | CSW_HPROT;
	if (csw != swjdp->ap_csw_value)
	{
		/* LOG_DEBUG("DAP: Set CSW %x",csw); */
		/* FIXME if this call fails, fail this procedure! */
		dap_ap_write_reg_u32(swjdp, AP_REG_CSW, csw);
		swjdp->ap_csw_value = csw;
	}
	if (tar != swjdp->ap_tar_value)
	{
		/* LOG_DEBUG("DAP: Set TAR %x",tar); */
		/* FIXME if this call fails, fail this procedure! */
		dap_ap_write_reg_u32(swjdp, AP_REG_TAR, tar);
		swjdp->ap_tar_value = tar;
	}
	/* Disable TAR cache when autoincrementing */
	if (csw & CSW_ADDRINC_MASK)
		swjdp->ap_tar_value = -1;
	return ERROR_OK;
}

/**
 * Asynchronous (queued) read of a word from memory or a system register.
 *
 * @param swjdp The DAP connected to the MEM-AP performing the read.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the word will be stored when the
 *	transaction queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_read_u32(struct swjdp_common *swjdp, uint32_t address,
		uint32_t *value)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when reading several consecutive addresses.
	 */
	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF,
			address & 0xFFFFFFF0);
	dap_ap_read_reg_u32(swjdp, AP_REG_BD0 | (address & 0xC), value);

	/* FIXME return any fault code from above calls */
	return ERROR_OK;
}

/**
 * Synchronous read of a word from memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param swjdp The DAP connected to the MEM-AP performing the read.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the result will be stored.
 *
 * @return ERROR_OK for success; *value holds the result.
 * Otherwise a fault code.
 */
int mem_ap_read_atomic_u32(struct swjdp_common *swjdp, uint32_t address,
		uint32_t *value)
{
	mem_ap_read_u32(swjdp, address, value);
	/* FIXME return any fault code from above call */

	return jtagdp_transaction_endcheck(swjdp);
}

/**
 * Asynchronous (queued) write of a word to memory or a system register.
 *
 * @param swjdp The DAP connected to the MEM-AP.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written to the address when transaction
 *	queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_write_u32(struct swjdp_common *swjdp, uint32_t address,
		uint32_t value)
{
	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when writing several consecutive addresses.
	 */
	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF,
			address & 0xFFFFFFF0);
	dap_ap_write_reg_u32(swjdp, AP_REG_BD0 | (address & 0xC), value);

	/* FIXME return any fault code from above calls */
	return ERROR_OK;
}

/**
 * Synchronous write of a word to memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param swjdp The DAP connected to the MEM-AP.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written.
 *
 * @return ERROR_OK for success; the data was written.  Otherwise a fault code.
 */
int mem_ap_write_atomic_u32(struct swjdp_common *swjdp, uint32_t address,
		uint32_t value)
{
	mem_ap_write_u32(swjdp, address, value);
	/* FIXME return any fault code from above call */

	return jtagdp_transaction_endcheck(swjdp);
}

/*****************************************************************************
*                                                                            *
* mem_ap_write_buf(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address) *
*                                                                            *
* Write a buffer in target order (little endian)                             *
*                                                                            *
*****************************************************************************/
int mem_ap_write_buf_u32(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address)
{
	int wcount, blocksize, writecount, errorcount = 0, retval = ERROR_OK;
	uint32_t adr = address;
	uint8_t* pBuffer = buffer;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	count >>= 2;
	wcount = count;

	/* if we have an unaligned access - reorder data */
	if (adr & 0x3u)
	{
		for (writecount = 0; writecount < count; writecount++)
		{
			int i;
			uint32_t outvalue;
			memcpy(&outvalue, pBuffer, sizeof(uint32_t));

			for (i = 0; i < 4; i++)
			{
				*((uint8_t*)pBuffer + (adr & 0x3)) = outvalue;
				outvalue >>= 8;
				adr++;
			}
			pBuffer += sizeof(uint32_t);
		}
	}

	while (wcount > 0)
	{
		/* Adjust to write blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(swjdp->tar_autoincr_block, address);
		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);

		for (writecount = 0; writecount < blocksize; writecount++)
		{
			dap_ap_write_reg(swjdp, AP_REG_DRW, buffer + 4 * writecount);
		}

		if (jtagdp_transaction_endcheck(swjdp) == ERROR_OK)
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
			LOG_WARNING("Block write error address 0x%" PRIx32 ", wcount 0x%x", address, wcount);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	return retval;
}

static int mem_ap_write_buf_packed_u16(struct swjdp_common *swjdp,
		uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;
	int wcount, blocksize, writecount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count >> 1;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to write blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(swjdp->tar_autoincr_block, address);

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

			if (nbytes < 4)
			{
				if (mem_ap_write_buf_u16(swjdp, buffer,
						nbytes, address) != ERROR_OK)
				{
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}

				address += nbytes >> 1;
			}
			else
			{
				uint32_t outvalue;
				memcpy(&outvalue, buffer, sizeof(uint32_t));

				for (i = 0; i < nbytes; i++)
				{
					*((uint8_t*)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				memcpy(&outvalue, buffer, sizeof(uint32_t));
				dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue);
				if (jtagdp_transaction_endcheck(swjdp) != ERROR_OK)
				{
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
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

int mem_ap_write_buf_u16(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_write_buf_packed_u16(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		uint16_t svalue;
		memcpy(&svalue, buffer, sizeof(uint16_t));
		uint32_t outvalue = (uint32_t)svalue << 8 * (address & 0x3);
		dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue);
		retval = jtagdp_transaction_endcheck(swjdp);
		count -= 2;
		address += 2;
		buffer += 2;
	}

	return retval;
}

static int mem_ap_write_buf_packed_u8(struct swjdp_common *swjdp,
		uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;
	int wcount, blocksize, writecount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to write blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(swjdp->tar_autoincr_block, address);

		if (wcount < blocksize)
			blocksize = wcount;

		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		writecount = blocksize;

		do
		{
			nbytes = MIN(writecount, 4);

			if (nbytes < 4)
			{
				if (mem_ap_write_buf_u8(swjdp, buffer, nbytes, address) != ERROR_OK)
				{
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
					return ERROR_JTAG_DEVICE_ERROR;
				}

				address += nbytes;
			}
			else
			{
				uint32_t outvalue;
				memcpy(&outvalue, buffer, sizeof(uint32_t));

				for (i = 0; i < nbytes; i++)
				{
					*((uint8_t*)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				memcpy(&outvalue, buffer, sizeof(uint32_t));
				dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue);
				if (jtagdp_transaction_endcheck(swjdp) != ERROR_OK)
				{
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
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

int mem_ap_write_buf_u8(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_write_buf_packed_u8(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		uint32_t outvalue = (uint32_t)*buffer << 8 * (address & 0x3);
		dap_ap_write_reg_u32(swjdp, AP_REG_DRW, outvalue);
		retval = jtagdp_transaction_endcheck(swjdp);
		count--;
		address++;
		buffer++;
	}

	return retval;
}

/*********************************************************************************
*                                                                                *
* mem_ap_read_buf_u32(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address)  *
*                                                                                *
* Read block fast in target order (little endian) into a buffer                  *
*                                                                                *
**********************************************************************************/
int mem_ap_read_buf_u32(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address)
{
	int wcount, blocksize, readcount, errorcount = 0, retval = ERROR_OK;
	uint32_t adr = address;
	uint8_t* pBuffer = buffer;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	count >>= 2;
	wcount = count;

	while (wcount > 0)
	{
		/* Adjust to read blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(swjdp->tar_autoincr_block, address);
		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_SINGLE, address);

		/* Scan out first read */
		adi_jtag_dp_scan(swjdp, JTAG_DP_APACC, AP_REG_DRW,
				DPAP_READ, 0, NULL, NULL);
		for (readcount = 0; readcount < blocksize - 1; readcount++)
		{
			/* Scan out next read; scan in posted value for the
			 * previous one.  Assumes read is acked "OK/FAULT",
			 * and CTRL_STAT says that meant "OK".
			 */
			adi_jtag_dp_scan(swjdp, JTAG_DP_APACC, AP_REG_DRW,
					DPAP_READ, 0, buffer + 4 * readcount,
					&swjdp->ack);
		}

		/* Scan in last posted value; RDBUFF has no other effect,
		 * assuming ack is OK/FAULT and CTRL_STAT says "OK".
		 */
		adi_jtag_dp_scan(swjdp, JTAG_DP_DPACC, DP_RDBUFF,
				DPAP_READ, 0, buffer + 4 * readcount,
				&swjdp->ack);
		if (jtagdp_transaction_endcheck(swjdp) == ERROR_OK)
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
			LOG_WARNING("Block read error address 0x%" PRIx32 ", count 0x%x", address, count);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	/* if we have an unaligned access - reorder data */
	if (adr & 0x3u)
	{
		for (readcount = 0; readcount < count; readcount++)
		{
			int i;
			uint32_t data;
			memcpy(&data, pBuffer, sizeof(uint32_t));

			for (i = 0; i < 4; i++)
			{
				*((uint8_t*)pBuffer) = (data >> 8 * (adr & 0x3));
				pBuffer++;
				adr++;
			}
		}
	}

	return retval;
}

static int mem_ap_read_buf_packed_u16(struct swjdp_common *swjdp,
		uint8_t *buffer, int count, uint32_t address)
{
	uint32_t invalue;
	int retval = ERROR_OK;
	int wcount, blocksize, readcount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count >> 1;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to read blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(swjdp->tar_autoincr_block, address);
		if (wcount < blocksize)
			blocksize = wcount;

		dap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_PACKED, address);

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;
		readcount = blocksize;

		do
		{
			dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue);
			if (jtagdp_transaction_endcheck(swjdp) != ERROR_OK)
			{
				LOG_WARNING("Block read error address 0x%" PRIx32 ", count 0x%x", address, count);
				return ERROR_JTAG_DEVICE_ERROR;
			}

			nbytes = MIN((readcount << 1), 4);

			for (i = 0; i < nbytes; i++)
			{
				*((uint8_t*)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}

			readcount -= (nbytes >> 1);
		} while (readcount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_read_buf_u16(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address)
{
	uint32_t invalue, i;
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_read_buf_packed_u16(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue);
		retval = jtagdp_transaction_endcheck(swjdp);
		if (address & 0x1)
		{
			for (i = 0; i < 2; i++)
			{
				*((uint8_t*)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}
		}
		else
		{
			uint16_t svalue = (invalue >> 8 * (address & 0x3));
			memcpy(buffer, &svalue, sizeof(uint16_t));
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
static int mem_ap_read_buf_packed_u8(struct swjdp_common *swjdp,
		uint8_t *buffer, int count, uint32_t address)
{
	uint32_t invalue;
	int retval = ERROR_OK;
	int wcount, blocksize, readcount, i;

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	wcount = count;

	while (wcount > 0)
	{
		int nbytes;

		/* Adjust to read blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(swjdp->tar_autoincr_block, address);

		if (wcount < blocksize)
			blocksize = wcount;

		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		readcount = blocksize;

		do
		{
			dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue);
			if (jtagdp_transaction_endcheck(swjdp) != ERROR_OK)
			{
				LOG_WARNING("Block read error address 0x%" PRIx32 ", count 0x%x", address, count);
				return ERROR_JTAG_DEVICE_ERROR;
			}

			nbytes = MIN(readcount, 4);

			for (i = 0; i < nbytes; i++)
			{
				*((uint8_t*)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}

			readcount -= nbytes;
		} while (readcount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_read_buf_u8(struct swjdp_common *swjdp, uint8_t *buffer, int count, uint32_t address)
{
	uint32_t invalue;
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_read_buf_packed_u8(swjdp, buffer, count, address);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	while (count > 0)
	{
		dap_setup_accessport(swjdp, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		dap_ap_read_reg_u32(swjdp, AP_REG_DRW, &invalue);
		retval = jtagdp_transaction_endcheck(swjdp);
		*((uint8_t*)buffer) = (invalue >> 8 * (address & 0x3));
		count--;
		address++;
		buffer++;
	}

	return retval;
}

/**
 * Initialize a DAP.  This sets up the power domains, prepares the DP
 * for further use, and arranges to use AP #0 for all AP operations
 * until dap_ap-select() changes that policy.
 *
 * @param swjdp The DAP being initialized.
 *
 * @todo Rename this.  We also need an initialization scheme which account
 * for SWD transports not just JTAG; that will need to address differences
 * in layering.  (JTAG is useful without any debug target; but not SWD.)
 * And this may not even use an AHB-AP ... e.g. DAP-Lite uses an APB-AP.
 */
int ahbap_debugport_init(struct swjdp_common *swjdp)
{
	uint32_t idreg, romaddr, dummy;
	uint32_t ctrlstat;
	int cnt = 0;
	int retval;

	LOG_DEBUG(" ");

	/* Default MEM-AP setup.
	 *
	 * REVISIT AP #0 may be an inappropriate default for this.
	 * Should we probe, or receve a hint from the caller?
	 * Presumably we can ignore the possibility of multiple APs.
	 */
	swjdp->apsel = 0;
	swjdp->ap_csw_value = -1;
	swjdp->ap_tar_value = -1;

	/* DP initialization */
	swjdp->trans_mode = TRANS_MODE_ATOMIC;
	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);
	dap_dp_write_reg(swjdp, SSTICKYERR, DP_CTRL_STAT);
	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);

	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	dap_dp_write_reg(swjdp, swjdp->dp_ctrl_stat, DP_CTRL_STAT);
	dap_dp_read_reg(swjdp, &ctrlstat, DP_CTRL_STAT);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;

	/* Check that we have debug power domains activated */
	while (!(ctrlstat & CDBGPWRUPACK) && (cnt++ < 10))
	{
		LOG_DEBUG("DAP: wait CDBGPWRUPACK");
		dap_dp_read_reg(swjdp, &ctrlstat, DP_CTRL_STAT);
		if ((retval = jtag_execute_queue()) != ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	while (!(ctrlstat & CSYSPWRUPACK) && (cnt++ < 10))
	{
		LOG_DEBUG("DAP: wait CSYSPWRUPACK");
		dap_dp_read_reg(swjdp, &ctrlstat, DP_CTRL_STAT);
		if ((retval = jtag_execute_queue()) != ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);
	/* With debug power on we can activate OVERRUN checking */
	swjdp->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ | CORUNDETECT;
	dap_dp_write_reg(swjdp, swjdp->dp_ctrl_stat, DP_CTRL_STAT);
	dap_dp_read_reg(swjdp, &dummy, DP_CTRL_STAT);

	/*
	 * REVISIT this isn't actually *initializing* anything in an AP,
	 * and doesn't care if it's a MEM-AP at all (much less AHB-AP).
	 * Should it?  If the ROM address is valid, is this the right
	 * place to scan the table and do any topology detection?
	 */
	dap_ap_read_reg_u32(swjdp, AP_REG_IDR, &idreg);
	dap_ap_read_reg_u32(swjdp, AP_REG_BASE, &romaddr);

	LOG_DEBUG("MEM-AP #%d ID Register 0x%" PRIx32
		", Debug ROM Address 0x%" PRIx32,
		swjdp->apsel, idreg, romaddr);

	return ERROR_OK;
}

/* CID interpretation -- see ARM IHI 0029B section 3
 * and ARM IHI 0031A table 13-3.
 */
static const char *class_description[16] ={
	"Reserved", "ROM table", "Reserved", "Reserved",
	"Reserved", "Reserved", "Reserved", "Reserved",
	"Reserved", "CoreSight component", "Reserved", "Peripheral Test Block",
	"Reserved", "OptimoDE DESS",
		"Generic IP component", "PrimeCell or System component"
};

static bool
is_dap_cid_ok(uint32_t cid3, uint32_t cid2, uint32_t cid1, uint32_t cid0)
{
	return cid3 == 0xb1 && cid2 == 0x05
			&& ((cid1 & 0x0f) == 0) && cid0 == 0x0d;
}

int dap_info_command(struct command_context *cmd_ctx,
		struct swjdp_common *swjdp, int apsel)
{

	uint32_t dbgbase, apid;
	int romtable_present = 0;
	uint8_t mem_ap;
	uint32_t apselold;

	/* AP address is in bits 31:24 of DP_SELECT */
	if (apsel >= 256)
		return ERROR_INVALID_ARGUMENTS;

	apselold = swjdp->apsel;
	dap_ap_select(swjdp, apsel);
	dap_ap_read_reg_u32(swjdp, AP_REG_BASE, &dbgbase);
	dap_ap_read_reg_u32(swjdp, AP_REG_IDR, &apid);
	jtagdp_transaction_endcheck(swjdp);
	/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
	mem_ap = ((apid&0x10000) && ((apid&0x0F) != 0));
	command_print(cmd_ctx, "AP ID register 0x%8.8" PRIx32, apid);
	if (apid)
	{
		switch (apid&0x0F)
		{
			case 0:
				command_print(cmd_ctx, "\tType is JTAG-AP");
				break;
			case 1:
				command_print(cmd_ctx, "\tType is MEM-AP AHB");
				break;
			case 2:
				command_print(cmd_ctx, "\tType is MEM-AP APB");
				break;
			default:
				command_print(cmd_ctx, "\tUnknown AP type");
				break;
		}

		/* NOTE: a MEM-AP may have a single CoreSight component that's
		 * not a ROM table ... or have no such components at all.
		 */
		if (mem_ap)
			command_print(cmd_ctx, "AP BASE 0x%8.8" PRIx32,
					dbgbase);
	}
	else
	{
		command_print(cmd_ctx, "No AP found at this apsel 0x%x", apsel);
	}

	romtable_present = ((mem_ap) && (dbgbase != 0xFFFFFFFF));
	if (romtable_present)
	{
		uint32_t cid0,cid1,cid2,cid3,memtype,romentry;
		uint16_t entry_offset;

		/* bit 16 of apid indicates a memory access port */
		if (dbgbase & 0x02)
			command_print(cmd_ctx, "\tValid ROM table present");
		else
			command_print(cmd_ctx, "\tROM table in legacy format");

		/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000) | 0xFF0, &cid0);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000) | 0xFF4, &cid1);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000) | 0xFF8, &cid2);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000) | 0xFFC, &cid3);
		mem_ap_read_u32(swjdp, (dbgbase&0xFFFFF000) | 0xFCC, &memtype);
		jtagdp_transaction_endcheck(swjdp);
		if (!is_dap_cid_ok(cid3, cid2, cid1, cid0))
			command_print(cmd_ctx, "\tCID3 0x%2.2" PRIx32
					", CID2 0x%2.2" PRIx32
					", CID1 0x%2.2" PRIx32
					", CID0 0x%2.2" PRIx32,
					cid3, cid2, cid1, cid0);
		if (memtype & 0x01)
			command_print(cmd_ctx, "\tMEMTYPE system memory present on bus");
		else
			command_print(cmd_ctx, "\tMEMTYPE System memory not present. "
					"Dedicated debug bus.");

		/* Now we read ROM table entries from dbgbase&0xFFFFF000) | 0x000 until we get 0x00000000 */
		entry_offset = 0;
		do
		{
			mem_ap_read_atomic_u32(swjdp, (dbgbase&0xFFFFF000) | entry_offset, &romentry);
			command_print(cmd_ctx, "\tROMTABLE[0x%x] = 0x%" PRIx32 "",entry_offset,romentry);
			if (romentry&0x01)
			{
				uint32_t c_cid0, c_cid1, c_cid2, c_cid3;
				uint32_t c_pid0, c_pid1, c_pid2, c_pid3, c_pid4;
				uint32_t component_start, component_base;
				unsigned part_num;
				char *type, *full;

				component_base = (uint32_t)((dbgbase & 0xFFFFF000)
						+ (int)(romentry & 0xFFFFF000));
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFE0, &c_pid0);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFE4, &c_pid1);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFE8, &c_pid2);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFEC, &c_pid3);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFD0, &c_pid4);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFF0, &c_cid0);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFF4, &c_cid1);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFF8, &c_cid2);
				mem_ap_read_atomic_u32(swjdp,
						(component_base & 0xFFFFF000) | 0xFFC, &c_cid3);
				component_start = component_base - 0x1000*(c_pid4 >> 4);

				command_print(cmd_ctx, "\t\tComponent base address 0x%" PRIx32
						", start address 0x%" PRIx32,
						component_base, component_start);
				command_print(cmd_ctx, "\t\tComponent class is 0x%x, %s",
						(int) (c_cid1 >> 4) & 0xf,
						/* See ARM IHI 0029B Table 3-3 */
						class_description[(c_cid1 >> 4) & 0xf]);

				/* CoreSight component? */
				if (((c_cid1 >> 4) & 0x0f) == 9) {
					uint32_t devtype;
					unsigned minor;
					char *major = "Reserved", *subtype = "Reserved";

					mem_ap_read_atomic_u32(swjdp,
							(component_base & 0xfffff000) | 0xfcc,
							&devtype);
					minor = (devtype >> 4) & 0x0f;
					switch (devtype & 0x0f) {
					case 0:
						major = "Miscellaneous";
						switch (minor) {
						case 0:
							subtype = "other";
							break;
						case 4:
							subtype = "Validation component";
							break;
						}
						break;
					case 1:
						major = "Trace Sink";
						switch (minor) {
						case 0:
							subtype = "other";
							break;
						case 1:
							subtype = "Port";
							break;
						case 2:
							subtype = "Buffer";
							break;
						}
						break;
					case 2:
						major = "Trace Link";
						switch (minor) {
						case 0:
							subtype = "other";
							break;
						case 1:
							subtype = "Funnel, router";
							break;
						case 2:
							subtype = "Filter";
							break;
						case 3:
							subtype = "FIFO, buffer";
							break;
						}
						break;
					case 3:
						major = "Trace Source";
						switch (minor) {
						case 0:
							subtype = "other";
							break;
						case 1:
							subtype = "Processor";
							break;
						case 2:
							subtype = "DSP";
							break;
						case 3:
							subtype = "Engine/Coprocessor";
							break;
						case 4:
							subtype = "Bus";
							break;
						}
						break;
					case 4:
						major = "Debug Control";
						switch (minor) {
						case 0:
							subtype = "other";
							break;
						case 1:
							subtype = "Trigger Matrix";
							break;
						case 2:
							subtype = "Debug Auth";
							break;
						}
						break;
					case 5:
						major = "Debug Logic";
						switch (minor) {
						case 0:
							subtype = "other";
							break;
						case 1:
							subtype = "Processor";
							break;
						case 2:
							subtype = "DSP";
							break;
						case 3:
							subtype = "Engine/Coprocessor";
							break;
						}
						break;
					}
					command_print(cmd_ctx, "\t\tType is 0x%2.2x, %s, %s",
							(unsigned) (devtype & 0xff),
							major, subtype);
					/* REVISIT also show 0xfc8 DevId */
				}

				if (!is_dap_cid_ok(cid3, cid2, cid1, cid0))
					command_print(cmd_ctx, "\t\tCID3 0x%2.2" PRIx32
							", CID2 0x%2.2" PRIx32
							", CID1 0x%2.2" PRIx32
							", CID0 0x%2.2" PRIx32,
							c_cid3, c_cid2, c_cid1, c_cid0);
				command_print(cmd_ctx, "\t\tPeripheral ID[4..0] = hex "
						"%2.2x %2.2x %2.2x %2.2x %2.2x",
						(int) c_pid4,
						(int) c_pid3, (int) c_pid2,
						(int) c_pid1, (int) c_pid0);

				/* Part number interpretations are from Cortex
				 * core specs, the CoreSight components TRM
				 * (ARM DDI 0314H), and ETM specs; also from
				 * chip observation (e.g. TI SDTI).
				 */
				part_num = c_pid0 & 0xff;
				part_num |= (c_pid1 & 0x0f) << 8;
				switch (part_num) {
				case 0x000:
					type = "Cortex-M3 NVIC";
					full = "(Interrupt Controller)";
					break;
				case 0x001:
					type = "Cortex-M3 ITM";
					full = "(Instrumentation Trace Module)";
					break;
				case 0x002:
					type = "Cortex-M3 DWT";
					full = "(Data Watchpoint and Trace)";
					break;
				case 0x003:
					type = "Cortex-M3 FBP";
					full = "(Flash Patch and Breakpoint)";
					break;
				case 0x00d:
					type = "CoreSight ETM11";
					full = "(Embedded Trace)";
					break;
				// case 0x113: what?
				case 0x120:		/* from OMAP3 memmap */
					type = "TI SDTI";
					full = "(System Debug Trace Interface)";
					break;
				case 0x343:		/* from OMAP3 memmap */
					type = "TI DAPCTL";
					full = "";
					break;
				case 0x4e0:
					type = "Cortex-M3 ETM";
					full = "(Embedded Trace)";
					break;
				case 0x906:
					type = "Coresight CTI";
					full = "(Cross Trigger)";
					break;
				case 0x907:
					type = "Coresight ETB";
					full = "(Trace Buffer)";
					break;
				case 0x908:
					type = "Coresight CSTF";
					full = "(Trace Funnel)";
					break;
				case 0x910:
					type = "CoreSight ETM9";
					full = "(Embedded Trace)";
					break;
				case 0x912:
					type = "Coresight TPIU";
					full = "(Trace Port Interface Unit)";
					break;
				case 0x921:
					type = "Cortex-A8 ETM";
					full = "(Embedded Trace)";
					break;
				case 0x922:
					type = "Cortex-A8 CTI";
					full = "(Cross Trigger)";
					break;
				case 0x923:
					type = "Cortex-M3 TPIU";
					full = "(Trace Port Interface Unit)";
					break;
				case 0xc08:
					type = "Cortex-A8 Debug";
					full = "(Debug Unit)";
					break;
				default:
					type = "-*- unrecognized -*-";
					full = "";
					break;
				}
				command_print(cmd_ctx, "\t\tPart is %s %s",
						type, full);
			}
			else
			{
				if (romentry)
					command_print(cmd_ctx, "\t\tComponent not present");
				else
					command_print(cmd_ctx, "\t\tEnd of ROM table");
			}
			entry_offset += 4;
		} while (romentry > 0);
	}
	else
	{
		command_print(cmd_ctx, "\tNo ROM table present");
	}
	dap_ap_select(swjdp, apselold);

	return ERROR_OK;
}

DAP_COMMAND_HANDLER(dap_baseaddr_command)
{
	uint32_t apsel, apselsave, baseaddr;
	int retval;

	apselsave = swjdp->apsel;
	switch (CMD_ARGC) {
	case 0:
		apsel = swjdp->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel >= 256)
			return ERROR_INVALID_ARGUMENTS;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (apselsave != apsel)
		dap_ap_select(swjdp, apsel);

	/* NOTE:  assumes we're talking to a MEM-AP, which
	 * has a base address.  There are other kinds of AP,
	 * though they're not common for now.  This should
	 * use the ID register to verify it's a MEM-AP.
	 */
	dap_ap_read_reg_u32(swjdp, AP_REG_BASE, &baseaddr);
	retval = jtagdp_transaction_endcheck(swjdp);
	command_print(CMD_CTX, "0x%8.8" PRIx32, baseaddr);

	if (apselsave != apsel)
		dap_ap_select(swjdp, apselsave);

	return retval;
}

DAP_COMMAND_HANDLER(dap_memaccess_command)
{
	uint32_t memaccess_tck;

	switch (CMD_ARGC) {
	case 0:
		memaccess_tck = swjdp->memaccess_tck;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], memaccess_tck);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	swjdp->memaccess_tck = memaccess_tck;

	command_print(CMD_CTX, "memory bus access delay set to %" PRIi32 " tck",
			swjdp->memaccess_tck);

	return ERROR_OK;
}

DAP_COMMAND_HANDLER(dap_apsel_command)
{
	uint32_t apsel, apid;
	int retval;

	switch (CMD_ARGC) {
	case 0:
		apsel = 0;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel >= 256)
			return ERROR_INVALID_ARGUMENTS;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap_ap_select(swjdp, apsel);
	dap_ap_read_reg_u32(swjdp, AP_REG_IDR, &apid);
	retval = jtagdp_transaction_endcheck(swjdp);
	command_print(CMD_CTX, "ap %" PRIi32 " selected, identification register 0x%8.8" PRIx32,
			apsel, apid);

	return retval;
}

DAP_COMMAND_HANDLER(dap_apid_command)
{
	uint32_t apsel, apselsave, apid;
	int retval;

	apselsave = swjdp->apsel;
	switch (CMD_ARGC) {
	case 0:
		apsel = swjdp->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel >= 256)
			return ERROR_INVALID_ARGUMENTS;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (apselsave != apsel)
		dap_ap_select(swjdp, apsel);

	dap_ap_read_reg_u32(swjdp, AP_REG_IDR, &apid);
	retval = jtagdp_transaction_endcheck(swjdp);
	command_print(CMD_CTX, "0x%8.8" PRIx32, apid);
	if (apselsave != apsel)
		dap_ap_select(swjdp, apselsave);

	return retval;
}

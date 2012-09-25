/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009-2010 by Oyvind Harboe                              *
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
 * This programming interface allows DAP pipelined operations through a
 * transaction queue.  This primarily affects AP operations (such as using
 * a MEM-AP to access memory or registers).  If the current transaction has
 * not finished by the time the next one must begin, and the ORUNDETECT bit
 * is set in the DP_CTRL_STAT register, the SSTICKYORUN status is set and
 * further AP operations will fail.  There are two basic methods to avoid
 * such overrun errors.  One involves polling for status instead of using
 * transaction piplining.  The other involves adding delays to ensure the
 * AP has enough time to complete one operation before starting the next
 * one.  (For JTAG these delays are controlled by memaccess_tck.)
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

#include "jtag/interface.h"
#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

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
 * DP and MEM-AP  register access  through APACC and DPACC                 *
 *                                                                         *
***************************************************************************/

/**
 * Select one of the APs connected to the specified DAP.  The
 * selection is implicitly used with future AP transactions.
 * This is a NOP if the specified AP is already selected.
 *
 * @param dap The DAP
 * @param apsel Number of the AP to (implicitly) use with further
 *	transactions.  This normally identifies a MEM-AP.
 */
void dap_ap_select(struct adiv5_dap *dap, uint8_t ap)
{
	uint32_t new_ap = (ap << 24) & 0xFF000000;

	if (new_ap != dap->ap_current) {
		dap->ap_current = new_ap;
		/* Switching AP invalidates cached values.
		 * Values MUST BE UPDATED BEFORE AP ACCESS.
		 */
		dap->ap_bank_value = -1;
		dap->ap_csw_value = -1;
		dap->ap_tar_value = -1;
	}
}

/**
 * Queue transactions setting up transfer parameters for the
 * currently selected MEM-AP.
 *
 * Subsequent transfers using registers like AP_REG_DRW or AP_REG_BD2
 * initiate data reads or writes using memory or peripheral addresses.
 * If the CSW is configured for it, the TAR may be automatically
 * incremented after each transfer.
 *
 * @todo Rename to reflect it being specifically a MEM-AP function.
 *
 * @param dap The DAP connected to the MEM-AP.
 * @param csw MEM-AP Control/Status Word (CSW) register to assign.  If this
 *	matches the cached value, the register is not changed.
 * @param tar MEM-AP Transfer Address Register (TAR) to assign.  If this
 *	matches the cached address, the register is not changed.
 *
 * @return ERROR_OK if the transaction was properly queued, else a fault code.
 */
int dap_setup_accessport(struct adiv5_dap *dap, uint32_t csw, uint32_t tar)
{
	int retval;

	csw = csw | CSW_DBGSWENABLE | CSW_MASTER_DEBUG | CSW_HPROT;
	if (csw != dap->ap_csw_value) {
		/* LOG_DEBUG("DAP: Set CSW %x",csw); */
		retval = dap_queue_ap_write(dap, AP_REG_CSW, csw);
		if (retval != ERROR_OK)
			return retval;
		dap->ap_csw_value = csw;
	}
	if (tar != dap->ap_tar_value) {
		/* LOG_DEBUG("DAP: Set TAR %x",tar); */
		retval = dap_queue_ap_write(dap, AP_REG_TAR, tar);
		if (retval != ERROR_OK)
			return retval;
		dap->ap_tar_value = tar;
	}
	/* Disable TAR cache when autoincrementing */
	if (csw & CSW_ADDRINC_MASK)
		dap->ap_tar_value = -1;
	return ERROR_OK;
}

/**
 * Asynchronous (queued) read of a word from memory or a system register.
 *
 * @param dap The DAP connected to the MEM-AP performing the read.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the word will be stored when the
 *	transaction queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_read_u32(struct adiv5_dap *dap, uint32_t address,
		uint32_t *value)
{
	int retval;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when reading several consecutive addresses.
	 */
	retval = dap_setup_accessport(dap, CSW_32BIT | CSW_ADDRINC_OFF,
			address & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;

	return dap_queue_ap_read(dap, AP_REG_BD0 | (address & 0xC), value);
}

/**
 * Synchronous read of a word from memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param dap The DAP connected to the MEM-AP performing the read.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the result will be stored.
 *
 * @return ERROR_OK for success; *value holds the result.
 * Otherwise a fault code.
 */
int mem_ap_read_atomic_u32(struct adiv5_dap *dap, uint32_t address,
		uint32_t *value)
{
	int retval;

	retval = mem_ap_read_u32(dap, address, value);
	if (retval != ERROR_OK)
		return retval;

	return dap_run(dap);
}

/**
 * Asynchronous (queued) write of a word to memory or a system register.
 *
 * @param dap The DAP connected to the MEM-AP.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written to the address when transaction
 *	queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_write_u32(struct adiv5_dap *dap, uint32_t address,
		uint32_t value)
{
	int retval;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when writing several consecutive addresses.
	 */
	retval = dap_setup_accessport(dap, CSW_32BIT | CSW_ADDRINC_OFF,
			address & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;

	return dap_queue_ap_write(dap, AP_REG_BD0 | (address & 0xC),
			value);
}

/**
 * Synchronous write of a word to memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param dap The DAP connected to the MEM-AP.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written.
 *
 * @return ERROR_OK for success; the data was written.  Otherwise a fault code.
 */
int mem_ap_write_atomic_u32(struct adiv5_dap *dap, uint32_t address,
		uint32_t value)
{
	int retval = mem_ap_write_u32(dap, address, value);

	if (retval != ERROR_OK)
		return retval;

	return dap_run(dap);
}

/*****************************************************************************
*                                                                            *
* mem_ap_write_buf(struct adiv5_dap *dap, uint8_t *buffer, int count, uint32_t address) *
*                                                                            *
* Write a buffer in target order (little endian)                             *
*                                                                            *
*****************************************************************************/
int mem_ap_write_buf_u32(struct adiv5_dap *dap, const uint8_t *buffer, int count, uint32_t address)
{
	int wcount, blocksize, writecount, errorcount = 0, retval = ERROR_OK;
	uint32_t adr = address;
	const uint8_t *pBuffer = buffer;

	count >>= 2;
	wcount = count;

	/* if we have an unaligned access - reorder data */
	if (adr & 0x3u) {
		for (writecount = 0; writecount < count; writecount++) {
			int i;
			uint32_t outvalue;
			memcpy(&outvalue, pBuffer, sizeof(uint32_t));

			for (i = 0; i < 4; i++) {
				*((uint8_t *)pBuffer + (adr & 0x3)) = outvalue;
				outvalue >>= 8;
				adr++;
			}
			pBuffer += sizeof(uint32_t);
		}
	}

	while (wcount > 0) {
		/* Adjust to write blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(dap->tar_autoincr_block, address);
		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		retval = dap_setup_accessport(dap, CSW_32BIT | CSW_ADDRINC_SINGLE, address);
		if (retval != ERROR_OK)
			return retval;

		for (writecount = 0; writecount < blocksize; writecount++) {
			retval = dap_queue_ap_write(dap, AP_REG_DRW,
				*(uint32_t *) ((void *) (buffer + 4 * writecount)));
			if (retval != ERROR_OK)
				break;
		}

		retval = dap_run(dap);
		if (retval == ERROR_OK) {
			wcount = wcount - blocksize;
			address = address + 4 * blocksize;
			buffer = buffer + 4 * blocksize;
		} else
			errorcount++;

		if (errorcount > 1) {
			LOG_WARNING("Block write error address 0x%" PRIx32 ", wcount 0x%x", address, wcount);
			return retval;
		}
	}

	return retval;
}

static int mem_ap_write_buf_packed_u16(struct adiv5_dap *dap,
		const uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;
	int wcount, blocksize, writecount, i;

	wcount = count >> 1;

	while (wcount > 0) {
		int nbytes;

		/* Adjust to write blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(dap->tar_autoincr_block, address);

		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		retval = dap_setup_accessport(dap, CSW_16BIT | CSW_ADDRINC_PACKED, address);
		if (retval != ERROR_OK)
			return retval;
		writecount = blocksize;

		do {
			nbytes = MIN((writecount << 1), 4);

			if (nbytes < 4) {
				retval = mem_ap_write_buf_u16(dap, buffer,
						nbytes, address);
				if (retval != ERROR_OK) {
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
					return retval;
				}

				address += nbytes >> 1;
			} else {
				uint32_t outvalue;
				memcpy(&outvalue, buffer, sizeof(uint32_t));

				for (i = 0; i < nbytes; i++) {
					*((uint8_t *)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				memcpy(&outvalue, buffer, sizeof(uint32_t));
				retval = dap_queue_ap_write(dap,
						AP_REG_DRW, outvalue);
				if (retval != ERROR_OK)
					break;

				retval = dap_run(dap);
				if (retval != ERROR_OK) {
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
					return retval;
				}
			}

			buffer += nbytes >> 1;
			writecount -= nbytes >> 1;

		} while (writecount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_write_buf_u16(struct adiv5_dap *dap, const uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_write_buf_packed_u16(dap, buffer, count, address);

	while (count > 0) {
		retval = dap_setup_accessport(dap, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		if (retval != ERROR_OK)
			return retval;
		uint16_t svalue;
		memcpy(&svalue, buffer, sizeof(uint16_t));
		uint32_t outvalue = (uint32_t)svalue << 8 * (address & 0x3);
		retval = dap_queue_ap_write(dap, AP_REG_DRW, outvalue);
		if (retval != ERROR_OK)
			break;

		retval = dap_run(dap);
		if (retval != ERROR_OK)
			break;

		count -= 2;
		address += 2;
		buffer += 2;
	}

	return retval;
}

static int mem_ap_write_buf_packed_u8(struct adiv5_dap *dap,
		const uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;
	int wcount, blocksize, writecount, i;

	wcount = count;

	while (wcount > 0) {
		int nbytes;

		/* Adjust to write blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(dap->tar_autoincr_block, address);

		if (wcount < blocksize)
			blocksize = wcount;

		retval = dap_setup_accessport(dap, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		if (retval != ERROR_OK)
			return retval;
		writecount = blocksize;

		do {
			nbytes = MIN(writecount, 4);

			if (nbytes < 4) {
				retval = mem_ap_write_buf_u8(dap, buffer, nbytes, address);
				if (retval != ERROR_OK) {
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
					return retval;
				}

				address += nbytes;
			} else {
				uint32_t outvalue;
				memcpy(&outvalue, buffer, sizeof(uint32_t));

				for (i = 0; i < nbytes; i++) {
					*((uint8_t *)buffer + (address & 0x3)) = outvalue;
					outvalue >>= 8;
					address++;
				}

				memcpy(&outvalue, buffer, sizeof(uint32_t));
				retval = dap_queue_ap_write(dap,
						AP_REG_DRW, outvalue);
				if (retval != ERROR_OK)
					break;

				retval = dap_run(dap);
				if (retval != ERROR_OK) {
					LOG_WARNING("Block write error address "
						"0x%" PRIx32 ", count 0x%x",
						address, count);
					return retval;
				}
			}

			buffer += nbytes;
			writecount -= nbytes;

		} while (writecount);
		wcount -= blocksize;
	}

	return retval;
}

int mem_ap_write_buf_u8(struct adiv5_dap *dap, const uint8_t *buffer, int count, uint32_t address)
{
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_write_buf_packed_u8(dap, buffer, count, address);

	while (count > 0) {
		retval = dap_setup_accessport(dap, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		if (retval != ERROR_OK)
			return retval;
		uint32_t outvalue = (uint32_t)*buffer << 8 * (address & 0x3);
		retval = dap_queue_ap_write(dap, AP_REG_DRW, outvalue);
		if (retval != ERROR_OK)
			break;

		retval = dap_run(dap);
		if (retval != ERROR_OK)
			break;

		count--;
		address++;
		buffer++;
	}

	return retval;
}

/* FIXME don't import ... this is a temporary workaround for the
 * mem_ap_read_buf_u32() mess, until it's no longer JTAG-specific.
 */
extern int adi_jtag_dp_scan(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint8_t *outvalue, uint8_t *invalue, uint8_t *ack);

/**
 * Synchronously read a block of 32-bit words into a buffer
 * @param dap The DAP connected to the MEM-AP.
 * @param buffer where the words will be stored (in host byte order).
 * @param count How many words to read.
 * @param address Memory address from which to read words; all the
 *	words must be readable by the currently selected MEM-AP.
 */
int mem_ap_read_buf_u32(struct adiv5_dap *dap, uint8_t *buffer,
		int count, uint32_t address)
{
	int wcount, blocksize, readcount, errorcount = 0, retval = ERROR_OK;
	uint32_t adr = address;
	uint8_t *pBuffer = buffer;

	count >>= 2;
	wcount = count;

	while (wcount > 0) {
		/* Adjust to read blocks within boundaries aligned to the
		 * TAR autoincrement size (at least 2^10).  Autoincrement
		 * mode avoids an extra per-word roundtrip to update TAR.
		 */
		blocksize = max_tar_block_size(dap->tar_autoincr_block,
				address);
		if (wcount < blocksize)
			blocksize = wcount;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;

		retval = dap_setup_accessport(dap, CSW_32BIT | CSW_ADDRINC_SINGLE,
				address);
		if (retval != ERROR_OK)
			return retval;

		/* FIXME remove these three calls to adi_jtag_dp_scan(),
		 * so this routine becomes transport-neutral.  Be careful
		 * not to cause performance problems with JTAG; would it
		 * suffice to loop over dap_queue_ap_read(), or would that
		 * be slower when JTAG is the chosen transport?
		 */

		/* Scan out first read */
		retval = adi_jtag_dp_scan(dap, JTAG_DP_APACC, AP_REG_DRW,
				DPAP_READ, 0, NULL, NULL);
		if (retval != ERROR_OK)
			return retval;
		for (readcount = 0; readcount < blocksize - 1; readcount++) {
			/* Scan out next read; scan in posted value for the
			 * previous one.  Assumes read is acked "OK/FAULT",
			 * and CTRL_STAT says that meant "OK".
			 */
			retval = adi_jtag_dp_scan(dap, JTAG_DP_APACC, AP_REG_DRW,
					DPAP_READ, 0, buffer + 4 * readcount,
					&dap->ack);
			if (retval != ERROR_OK)
				return retval;
		}

		/* Scan in last posted value; RDBUFF has no other effect,
		 * assuming ack is OK/FAULT and CTRL_STAT says "OK".
		 */
		retval = adi_jtag_dp_scan(dap, JTAG_DP_DPACC, DP_RDBUFF,
				DPAP_READ, 0, buffer + 4 * readcount,
				&dap->ack);
		if (retval != ERROR_OK)
			return retval;

		retval = dap_run(dap);
		if (retval != ERROR_OK) {
			errorcount++;
			if (errorcount <= 1) {
				/* try again */
				continue;
			}
			LOG_WARNING("Block read error address 0x%" PRIx32, address);
			return retval;
		}
		wcount = wcount - blocksize;
		address += 4 * blocksize;
		buffer += 4 * blocksize;
	}

	/* if we have an unaligned access - reorder data */
	if (adr & 0x3u) {
		for (readcount = 0; readcount < count; readcount++) {
			int i;
			uint32_t data;
			memcpy(&data, pBuffer, sizeof(uint32_t));

			for (i = 0; i < 4; i++) {
				*((uint8_t *)pBuffer) =
						(data >> 8 * (adr & 0x3));
				pBuffer++;
				adr++;
			}
		}
	}

	return retval;
}

static int mem_ap_read_buf_packed_u16(struct adiv5_dap *dap,
		uint8_t *buffer, int count, uint32_t address)
{
	uint32_t invalue;
	int retval = ERROR_OK;
	int wcount, blocksize, readcount, i;

	wcount = count >> 1;

	while (wcount > 0) {
		int nbytes;

		/* Adjust to read blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(dap->tar_autoincr_block, address);
		if (wcount < blocksize)
			blocksize = wcount;

		retval = dap_setup_accessport(dap, CSW_16BIT | CSW_ADDRINC_PACKED, address);
		if (retval != ERROR_OK)
			return retval;

		/* handle unaligned data at 4k boundary */
		if (blocksize == 0)
			blocksize = 1;
		readcount = blocksize;

		do {
			retval = dap_queue_ap_read(dap, AP_REG_DRW, &invalue);
			if (retval != ERROR_OK)
				return retval;
			retval = dap_run(dap);
			if (retval != ERROR_OK) {
				LOG_WARNING("Block read error address 0x%" PRIx32 ", count 0x%x", address, count);
				return retval;
			}

			nbytes = MIN((readcount << 1), 4);

			for (i = 0; i < nbytes; i++) {
				*((uint8_t *)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}

			readcount -= (nbytes >> 1);
		} while (readcount);
		wcount -= blocksize;
	}

	return retval;
}

/**
 * Synchronously read a block of 16-bit halfwords into a buffer
 * @param dap The DAP connected to the MEM-AP.
 * @param buffer where the halfwords will be stored (in host byte order).
 * @param count How many halfwords to read.
 * @param address Memory address from which to read words; all the
 *	words must be readable by the currently selected MEM-AP.
 */
int mem_ap_read_buf_u16(struct adiv5_dap *dap, uint8_t *buffer,
		int count, uint32_t address)
{
	uint32_t invalue, i;
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_read_buf_packed_u16(dap, buffer, count, address);

	while (count > 0) {
		retval = dap_setup_accessport(dap, CSW_16BIT | CSW_ADDRINC_SINGLE, address);
		if (retval != ERROR_OK)
			return retval;
		retval = dap_queue_ap_read(dap, AP_REG_DRW, &invalue);
		if (retval != ERROR_OK)
			break;

		retval = dap_run(dap);
		if (retval != ERROR_OK)
			break;

		if (address & 0x1) {
			for (i = 0; i < 2; i++) {
				*((uint8_t *)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}
		} else {
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
static int mem_ap_read_buf_packed_u8(struct adiv5_dap *dap,
		uint8_t *buffer, int count, uint32_t address)
{
	uint32_t invalue;
	int retval = ERROR_OK;
	int wcount, blocksize, readcount, i;

	wcount = count;

	while (wcount > 0) {
		int nbytes;

		/* Adjust to read blocks within boundaries aligned to the TAR autoincremnent size*/
		blocksize = max_tar_block_size(dap->tar_autoincr_block, address);

		if (wcount < blocksize)
			blocksize = wcount;

		retval = dap_setup_accessport(dap, CSW_8BIT | CSW_ADDRINC_PACKED, address);
		if (retval != ERROR_OK)
			return retval;
		readcount = blocksize;

		do {
			retval = dap_queue_ap_read(dap, AP_REG_DRW, &invalue);
			if (retval != ERROR_OK)
				return retval;
			retval = dap_run(dap);
			if (retval != ERROR_OK) {
				LOG_WARNING("Block read error address 0x%" PRIx32 ", count 0x%x", address, count);
				return retval;
			}

			nbytes = MIN(readcount, 4);

			for (i = 0; i < nbytes; i++) {
				*((uint8_t *)buffer) = (invalue >> 8 * (address & 0x3));
				buffer++;
				address++;
			}

			readcount -= nbytes;
		} while (readcount);
		wcount -= blocksize;
	}

	return retval;
}

/**
 * Synchronously read a block of bytes into a buffer
 * @param dap The DAP connected to the MEM-AP.
 * @param buffer where the bytes will be stored.
 * @param count How many bytes to read.
 * @param address Memory address from which to read data; all the
 *	data must be readable by the currently selected MEM-AP.
 */
int mem_ap_read_buf_u8(struct adiv5_dap *dap, uint8_t *buffer,
		int count, uint32_t address)
{
	uint32_t invalue;
	int retval = ERROR_OK;

	if (count >= 4)
		return mem_ap_read_buf_packed_u8(dap, buffer, count, address);

	while (count > 0) {
		retval = dap_setup_accessport(dap, CSW_8BIT | CSW_ADDRINC_SINGLE, address);
		if (retval != ERROR_OK)
			return retval;
		retval = dap_queue_ap_read(dap, AP_REG_DRW, &invalue);
		if (retval != ERROR_OK)
			return retval;
		retval = dap_run(dap);
		if (retval != ERROR_OK)
			break;

		*((uint8_t *)buffer) = (invalue >> 8 * (address & 0x3));
		count--;
		address++;
		buffer++;
	}

	return retval;
}

/*--------------------------------------------------------------------*/
/*          Wrapping function with selection of AP                    */
/*--------------------------------------------------------------------*/
int mem_ap_sel_read_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t *value)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_read_u32(swjdp, address, value);
}

int mem_ap_sel_write_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t value)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_write_u32(swjdp, address, value);
}

int mem_ap_sel_read_atomic_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t *value)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_read_atomic_u32(swjdp, address, value);
}

int mem_ap_sel_write_atomic_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t value)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_write_atomic_u32(swjdp, address, value);
}

int mem_ap_sel_read_buf_u8(struct adiv5_dap *swjdp, uint8_t ap,
		uint8_t *buffer, int count, uint32_t address)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_read_buf_u8(swjdp, buffer, count, address);
}

int mem_ap_sel_read_buf_u16(struct adiv5_dap *swjdp, uint8_t ap,
		uint8_t *buffer, int count, uint32_t address)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_read_buf_u16(swjdp, buffer, count, address);
}

int mem_ap_sel_read_buf_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint8_t *buffer, int count, uint32_t address)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_read_buf_u32(swjdp, buffer, count, address);
}

int mem_ap_sel_write_buf_u8(struct adiv5_dap *swjdp, uint8_t ap,
		const uint8_t *buffer, int count, uint32_t address)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_write_buf_u8(swjdp, buffer, count, address);
}

int mem_ap_sel_write_buf_u16(struct adiv5_dap *swjdp, uint8_t ap,
		const uint8_t *buffer, int count, uint32_t address)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_write_buf_u16(swjdp, buffer, count, address);
}

int mem_ap_sel_write_buf_u32(struct adiv5_dap *swjdp, uint8_t ap,
		const uint8_t *buffer, int count, uint32_t address)
{
	dap_ap_select(swjdp, ap);
	return mem_ap_write_buf_u32(swjdp, buffer, count, address);
}

#define MDM_REG_STAT		0x00
#define MDM_REG_CTRL		0x04
#define MDM_REG_ID		0xfc

#define MDM_STAT_FMEACK		(1<<0)
#define MDM_STAT_FREADY		(1<<1)
#define MDM_STAT_SYSSEC		(1<<2)
#define MDM_STAT_SYSRES		(1<<3)
#define MDM_STAT_FMEEN		(1<<5)
#define MDM_STAT_BACKDOOREN	(1<<6)
#define MDM_STAT_LPEN		(1<<7)
#define MDM_STAT_VLPEN		(1<<8)
#define MDM_STAT_LLSMODEXIT	(1<<9)
#define MDM_STAT_VLLSXMODEXIT	(1<<10)
#define MDM_STAT_CORE_HALTED	(1<<16)
#define MDM_STAT_CORE_SLEEPDEEP	(1<<17)
#define MDM_STAT_CORESLEEPING	(1<<18)

#define MEM_CTRL_FMEIP		(1<<0)
#define MEM_CTRL_DBG_DIS	(1<<1)
#define MEM_CTRL_DBG_REQ	(1<<2)
#define MEM_CTRL_SYS_RES_REQ	(1<<3)
#define MEM_CTRL_CORE_HOLD_RES	(1<<4)
#define MEM_CTRL_VLLSX_DBG_REQ	(1<<5)
#define MEM_CTRL_VLLSX_DBG_ACK	(1<<6)
#define MEM_CTRL_VLLSX_STAT_ACK	(1<<7)

/**
 *
 */
int dap_syssec_kinetis_mdmap(struct adiv5_dap *dap)
{
	uint32_t val;
	int retval;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	dap_ap_select(dap, 1);

	/* first check mdm-ap id register */
	retval = dap_queue_ap_read(dap, MDM_REG_ID, &val);
	if (retval != ERROR_OK)
		return retval;
	dap_run(dap);

	if (val != 0x001C0000) {
		LOG_DEBUG("id doesn't match %08X != 0x001C0000", val);
		dap_ap_select(dap, 0);
		return ERROR_FAIL;
	}

	/* read and parse status register
	 * it's important that the device is out of
	 * reset here
	 */
	retval = dap_queue_ap_read(dap, MDM_REG_STAT, &val);
	if (retval != ERROR_OK)
		return retval;
	dap_run(dap);

	LOG_DEBUG("MDM_REG_STAT %08X", val);

	if ((val & (MDM_STAT_SYSSEC|MDM_STAT_FREADY)) != (MDM_STAT_FREADY)) {
		LOG_DEBUG("MDMAP: system is secured, masserase needed");

		if (!(val & MDM_STAT_FMEEN))
			LOG_DEBUG("MDMAP: masserase is disabled");
		else {
			/* we need to assert reset */
			if (jtag_reset_config & RESET_HAS_SRST) {
				/* default to asserting srst */
				adapter_assert_reset();
			} else {
				LOG_DEBUG("SRST not configured");
				dap_ap_select(dap, 0);
				return ERROR_FAIL;
			}

			while (1) {
				retval = dap_queue_ap_write(dap, MDM_REG_CTRL, MEM_CTRL_FMEIP);
				if (retval != ERROR_OK)
					return retval;
				dap_run(dap);
				/* read status register and wait for ready */
				retval = dap_queue_ap_read(dap, MDM_REG_STAT, &val);
				if (retval != ERROR_OK)
					return retval;
				dap_run(dap);
				LOG_DEBUG("MDM_REG_STAT %08X", val);

				if ((val & 1))
					break;
			}

			while (1) {
				retval = dap_queue_ap_write(dap, MDM_REG_CTRL, 0);
				if (retval != ERROR_OK)
					return retval;
				dap_run(dap);
				/* read status register */
				retval = dap_queue_ap_read(dap, MDM_REG_STAT, &val);
				if (retval != ERROR_OK)
					return retval;
				dap_run(dap);
				LOG_DEBUG("MDM_REG_STAT %08X", val);
				/* read control register and wait for ready */
				retval = dap_queue_ap_read(dap, MDM_REG_CTRL, &val);
				if (retval != ERROR_OK)
					return retval;
				dap_run(dap);
				LOG_DEBUG("MDM_REG_CTRL %08X", val);

				if (val == 0x00)
					break;
			}
		}
	}

	dap_ap_select(dap, 0);

	return ERROR_OK;
}

/** */
struct dap_syssec_filter {
	/** */
	uint32_t idcode;
	/** */
	int (*dap_init)(struct adiv5_dap *dap);
};

/** */
static struct dap_syssec_filter dap_syssec_filter_data[] = {
	{ 0x4BA00477, dap_syssec_kinetis_mdmap }
};

/**
 *
 */
int dap_syssec(struct adiv5_dap *dap)
{
	unsigned int i;
	struct jtag_tap *tap;

	for (i = 0; i < sizeof(dap_syssec_filter_data); i++) {
		tap = dap->jtag_info->tap;

		while (tap != NULL) {
			if (tap->hasidcode && (dap_syssec_filter_data[i].idcode == tap->idcode)) {
				LOG_DEBUG("DAP: mdmap_init for idcode: %08x", tap->idcode);
				dap_syssec_filter_data[i].dap_init(dap);
			}
			tap = tap->next_tap;
		}
	}

	return ERROR_OK;
}

/*--------------------------------------------------------------------------*/


/* FIXME don't import ... just initialize as
 * part of DAP transport setup
*/
extern const struct dap_ops jtag_dp_ops;

/*--------------------------------------------------------------------------*/

/**
 * Initialize a DAP.  This sets up the power domains, prepares the DP
 * for further use, and arranges to use AP #0 for all AP operations
 * until dap_ap-select() changes that policy.
 *
 * @param dap The DAP being initialized.
 *
 * @todo Rename this.  We also need an initialization scheme which account
 * for SWD transports not just JTAG; that will need to address differences
 * in layering.  (JTAG is useful without any debug target; but not SWD.)
 * And this may not even use an AHB-AP ... e.g. DAP-Lite uses an APB-AP.
 */
int ahbap_debugport_init(struct adiv5_dap *dap)
{
	uint32_t ctrlstat;
	int cnt = 0;
	int retval;

	LOG_DEBUG(" ");

	/* JTAG-DP or SWJ-DP, in JTAG mode
	 * ... for SWD mode this is patched as part
	 * of link switchover
	 */
	if (!dap->ops)
		dap->ops = &jtag_dp_ops;

	/* Default MEM-AP setup.
	 *
	 * REVISIT AP #0 may be an inappropriate default for this.
	 * Should we probe, or take a hint from the caller?
	 * Presumably we can ignore the possibility of multiple APs.
	 */
	dap->ap_current = !0;
	dap_ap_select(dap, 0);

	/* DP initialization */

	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, SSTICKYERR);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;

	dap->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;
	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, dap->dp_ctrl_stat);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, &ctrlstat);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	/* Check that we have debug power domains activated */
	while (!(ctrlstat & CDBGPWRUPACK) && (cnt++ < 10)) {
		LOG_DEBUG("DAP: wait CDBGPWRUPACK");
		retval = dap_queue_dp_read(dap, DP_CTRL_STAT, &ctrlstat);
		if (retval != ERROR_OK)
			return retval;
		retval = dap_run(dap);
		if (retval != ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	while (!(ctrlstat & CSYSPWRUPACK) && (cnt++ < 10)) {
		LOG_DEBUG("DAP: wait CSYSPWRUPACK");
		retval = dap_queue_dp_read(dap, DP_CTRL_STAT, &ctrlstat);
		if (retval != ERROR_OK)
			return retval;
		retval = dap_run(dap);
		if (retval != ERROR_OK)
			return retval;
		alive_sleep(10);
	}

	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;
	/* With debug power on we can activate OVERRUN checking */
	dap->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ | CORUNDETECT;
	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, dap->dp_ctrl_stat);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;

	dap_syssec(dap);

	return ERROR_OK;
}

/* CID interpretation -- see ARM IHI 0029B section 3
 * and ARM IHI 0031A table 13-3.
 */
static const char *class_description[16] = {
	"Reserved", "ROM table", "Reserved", "Reserved",
	"Reserved", "Reserved", "Reserved", "Reserved",
	"Reserved", "CoreSight component", "Reserved", "Peripheral Test Block",
	"Reserved", "OptimoDE DESS",
	"Generic IP component", "PrimeCell or System component"
};

static bool is_dap_cid_ok(uint32_t cid3, uint32_t cid2, uint32_t cid1, uint32_t cid0)
{
	return cid3 == 0xb1 && cid2 == 0x05
			&& ((cid1 & 0x0f) == 0) && cid0 == 0x0d;
}

int dap_get_debugbase(struct adiv5_dap *dap, int ap,
			uint32_t *out_dbgbase, uint32_t *out_apid)
{
	uint32_t ap_old;
	int retval;
	uint32_t dbgbase, apid;

	/* AP address is in bits 31:24 of DP_SELECT */
	if (ap >= 256)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ap_old = dap->ap_current;
	dap_ap_select(dap, ap);

	retval = dap_queue_ap_read(dap, AP_REG_BASE, &dbgbase);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_ap_read(dap, AP_REG_IDR, &apid);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	/* Excavate the device ID code */
	struct jtag_tap *tap = dap->jtag_info->tap;
	while (tap != NULL) {
		if (tap->hasidcode)
			break;
		tap = tap->next_tap;
	}
	if (tap == NULL || !tap->hasidcode)
		return ERROR_OK;

	dap_ap_select(dap, ap_old);

	/* The asignment happens only here to prevent modification of these
	 * values before they are certain. */
	*out_dbgbase = dbgbase;
	*out_apid = apid;

	return ERROR_OK;
}

int dap_lookup_cs_component(struct adiv5_dap *dap, int ap,
			uint32_t dbgbase, uint8_t type, uint32_t *addr)
{
	uint32_t ap_old;
	uint32_t romentry, entry_offset = 0, component_base, devtype;
	int retval = ERROR_FAIL;

	if (ap >= 256)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ap_old = dap->ap_current;
	dap_ap_select(dap, ap);

	do {
		retval = mem_ap_read_atomic_u32(dap, (dbgbase&0xFFFFF000) |
						entry_offset, &romentry);
		if (retval != ERROR_OK)
			return retval;

		component_base = (dbgbase & 0xFFFFF000)
			+ (romentry & 0xFFFFF000);

		if (romentry & 0x1) {
			retval = mem_ap_read_atomic_u32(dap,
					(component_base & 0xfffff000) | 0xfcc,
					&devtype);
			if (retval != ERROR_OK)
				return retval;
			if ((devtype & 0xff) == type) {
				*addr = component_base;
				retval = ERROR_OK;
				break;
			}
		}
		entry_offset += 4;
	} while (romentry > 0);

	dap_ap_select(dap, ap_old);

	return retval;
}

static int dap_info_command(struct command_context *cmd_ctx,
		struct adiv5_dap *dap, int ap)
{
	int retval;
	uint32_t dbgbase = 0, apid = 0; /* Silence gcc by initializing */
	int romtable_present = 0;
	uint8_t mem_ap;
	uint32_t ap_old;

	retval = dap_get_debugbase(dap, ap, &dbgbase, &apid);
	if (retval != ERROR_OK)
		return retval;

	ap_old = dap->ap_current;
	dap_ap_select(dap, ap);

	/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
	mem_ap = ((apid&0x10000) && ((apid&0x0F) != 0));
	command_print(cmd_ctx, "AP ID register 0x%8.8" PRIx32, apid);
	if (apid) {
		switch (apid&0x0F) {
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
			command_print(cmd_ctx, "AP BASE 0x%8.8" PRIx32, dbgbase);
	} else
		command_print(cmd_ctx, "No AP found at this ap 0x%x", ap);

	romtable_present = ((mem_ap) && (dbgbase != 0xFFFFFFFF));
	if (romtable_present) {
		uint32_t cid0, cid1, cid2, cid3, memtype, romentry;
		uint16_t entry_offset;

		/* bit 16 of apid indicates a memory access port */
		if (dbgbase & 0x02)
			command_print(cmd_ctx, "\tValid ROM table present");
		else
			command_print(cmd_ctx, "\tROM table in legacy format");

		/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
		retval = mem_ap_read_u32(dap, (dbgbase&0xFFFFF000) | 0xFF0, &cid0);
		if (retval != ERROR_OK)
			return retval;
		retval = mem_ap_read_u32(dap, (dbgbase&0xFFFFF000) | 0xFF4, &cid1);
		if (retval != ERROR_OK)
			return retval;
		retval = mem_ap_read_u32(dap, (dbgbase&0xFFFFF000) | 0xFF8, &cid2);
		if (retval != ERROR_OK)
			return retval;
		retval = mem_ap_read_u32(dap, (dbgbase&0xFFFFF000) | 0xFFC, &cid3);
		if (retval != ERROR_OK)
			return retval;
		retval = mem_ap_read_u32(dap, (dbgbase&0xFFFFF000) | 0xFCC, &memtype);
		if (retval != ERROR_OK)
			return retval;
		retval = dap_run(dap);
		if (retval != ERROR_OK)
			return retval;

		if (!is_dap_cid_ok(cid3, cid2, cid1, cid0))
			command_print(cmd_ctx, "\tCID3 0x%2.2x"
					", CID2 0x%2.2x"
					", CID1 0x%2.2x"
					", CID0 0x%2.2x",
					(unsigned) cid3, (unsigned)cid2,
					(unsigned) cid1, (unsigned) cid0);
		if (memtype & 0x01)
			command_print(cmd_ctx, "\tMEMTYPE system memory present on bus");
		else
			command_print(cmd_ctx, "\tMEMTYPE System memory not present. "
					"Dedicated debug bus.");

		/* Now we read ROM table entries from dbgbase&0xFFFFF000) | 0x000 until we get 0x00000000 */
		entry_offset = 0;
		do {
			retval = mem_ap_read_atomic_u32(dap, (dbgbase&0xFFFFF000) | entry_offset, &romentry);
			if (retval != ERROR_OK)
				return retval;
			command_print(cmd_ctx, "\tROMTABLE[0x%x] = 0x%" PRIx32 "", entry_offset, romentry);
			if (romentry & 0x01) {
				uint32_t c_cid0, c_cid1, c_cid2, c_cid3;
				uint32_t c_pid0, c_pid1, c_pid2, c_pid3, c_pid4;
				uint32_t component_base;
				unsigned part_num;
				char *type, *full;

				component_base = (dbgbase & 0xFFFFF000) + (romentry & 0xFFFFF000);

				/* IDs are in last 4K section */
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFE0, &c_pid0);
				if (retval != ERROR_OK)
					return retval;
				c_pid0 &= 0xff;
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFE4, &c_pid1);
				if (retval != ERROR_OK)
					return retval;
				c_pid1 &= 0xff;
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFE8, &c_pid2);
				if (retval != ERROR_OK)
					return retval;
				c_pid2 &= 0xff;
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFEC, &c_pid3);
				if (retval != ERROR_OK)
					return retval;
				c_pid3 &= 0xff;
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFD0, &c_pid4);
				if (retval != ERROR_OK)
					return retval;
				c_pid4 &= 0xff;

				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFF0, &c_cid0);
				if (retval != ERROR_OK)
					return retval;
				c_cid0 &= 0xff;
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFF4, &c_cid1);
				if (retval != ERROR_OK)
					return retval;
				c_cid1 &= 0xff;
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFF8, &c_cid2);
				if (retval != ERROR_OK)
					return retval;
				c_cid2 &= 0xff;
				retval = mem_ap_read_atomic_u32(dap, component_base + 0xFFC, &c_cid3);
				if (retval != ERROR_OK)
					return retval;
				c_cid3 &= 0xff;

				command_print(cmd_ctx, "\t\tComponent base address 0x%" PRIx32 ","
						"start address 0x%" PRIx32, component_base,
				/* component may take multiple 4K pages */
				component_base - 0x1000*(c_pid4 >> 4));
				command_print(cmd_ctx, "\t\tComponent class is 0x%x, %s",
						(int) (c_cid1 >> 4) & 0xf,
						/* See ARM IHI 0029B Table 3-3 */
						class_description[(c_cid1 >> 4) & 0xf]);

				/* CoreSight component? */
				if (((c_cid1 >> 4) & 0x0f) == 9) {
					uint32_t devtype;
					unsigned minor;
					char *major = "Reserved", *subtype = "Reserved";

					retval = mem_ap_read_atomic_u32(dap,
							(component_base & 0xfffff000) | 0xfcc,
							&devtype);
					if (retval != ERROR_OK)
						return retval;
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
					command_print(cmd_ctx,
							"\t\tCID3 0%2.2x"
							", CID2 0%2.2x"
							", CID1 0%2.2x"
							", CID0 0%2.2x",
							(int) c_cid3,
							(int) c_cid2,
							(int)c_cid1,
							(int)c_cid0);
				command_print(cmd_ctx,
				"\t\tPeripheral ID[4..0] = hex "
				"%2.2x %2.2x %2.2x %2.2x %2.2x",
				(int) c_pid4, (int) c_pid3, (int) c_pid2,
				(int) c_pid1, (int) c_pid0);

				/* Part number interpretations are from Cortex
				 * core specs, the CoreSight components TRM
				 * (ARM DDI 0314H), CoreSight System Design
				 * Guide (ARM DGI 0012D) and ETM specs; also
				 * from chip observation (e.g. TI SDTI).
				 */
				part_num = (c_pid0 & 0xff);
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
				case 0x00c:
					type = "Cortex-M4 SCS";
					full = "(System Control Space)";
					break;
				case 0x00d:
					type = "CoreSight ETM11";
					full = "(Embedded Trace)";
					break;
				/* case 0x113: what? */
				case 0x120:		/* from OMAP3 memmap */
					type = "TI SDTI";
					full = "(System Debug Trace Interface)";
					break;
				case 0x343:		/* from OMAP3 memmap */
					type = "TI DAPCTL";
					full = "";
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
				case 0x924:
					type = "Cortex-M3 ETM";
					full = "(Embedded Trace)";
					break;
				case 0x925:
					type = "Cortex-M4 ETM";
					full = "(Embedded Trace)";
					break;
				case 0x930:
					type = "Cortex-R4 ETM";
					full = "(Embedded Trace)";
					break;
				case 0x9a1:
					type = "Cortex-M4 TPUI";
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
			} else {
				if (romentry)
					command_print(cmd_ctx, "\t\tComponent not present");
				else
					command_print(cmd_ctx, "\t\tEnd of ROM table");
			}
			entry_offset += 4;
		} while (romentry > 0);
	} else
		command_print(cmd_ctx, "\tNo ROM table present");
	dap_ap_select(dap, ap_old);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_dap_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;
	uint32_t apsel;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return dap_info_command(CMD_CTX, dap, apsel);
}

COMMAND_HANDLER(dap_baseaddr_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;

	uint32_t apsel, baseaddr;
	int retval;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel >= 256)
			return ERROR_COMMAND_SYNTAX_ERROR;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap_ap_select(dap, apsel);

	/* NOTE:  assumes we're talking to a MEM-AP, which
	 * has a base address.  There are other kinds of AP,
	 * though they're not common for now.  This should
	 * use the ID register to verify it's a MEM-AP.
	 */
	retval = dap_queue_ap_read(dap, AP_REG_BASE, &baseaddr);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD_CTX, "0x%8.8" PRIx32, baseaddr);

	return retval;
}

COMMAND_HANDLER(dap_memaccess_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;

	uint32_t memaccess_tck;

	switch (CMD_ARGC) {
	case 0:
		memaccess_tck = dap->memaccess_tck;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], memaccess_tck);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	dap->memaccess_tck = memaccess_tck;

	command_print(CMD_CTX, "memory bus access delay set to %" PRIi32 " tck",
			dap->memaccess_tck);

	return ERROR_OK;
}

COMMAND_HANDLER(dap_apsel_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;

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
			return ERROR_COMMAND_SYNTAX_ERROR;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap->apsel = apsel;
	dap_ap_select(dap, apsel);

	retval = dap_queue_ap_read(dap, AP_REG_IDR, &apid);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD_CTX, "ap %" PRIi32 " selected, identification register 0x%8.8" PRIx32,
			apsel, apid);

	return retval;
}

COMMAND_HANDLER(dap_apid_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;

	uint32_t apsel, apid;
	int retval;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel >= 256)
			return ERROR_COMMAND_SYNTAX_ERROR;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap_ap_select(dap, apsel);

	retval = dap_queue_ap_read(dap, AP_REG_IDR, &apid);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD_CTX, "0x%8.8" PRIx32, apid);

	return retval;
}

static const struct command_registration dap_commands[] = {
	{
		.name = "info",
		.handler = handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "display ROM table for MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "apsel",
		.handler = dap_apsel_command,
		.mode = COMMAND_EXEC,
		.help = "Set the currently selected AP (default 0) "
			"and display the result",
		.usage = "[ap_num]",
	},
	{
		.name = "apid",
		.handler = dap_apid_command,
		.mode = COMMAND_EXEC,
		.help = "return ID register from AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "baseaddr",
		.handler = dap_baseaddr_command,
		.mode = COMMAND_EXEC,
		.help = "return debug base address from MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "memaccess",
		.handler = dap_memaccess_command,
		.mode = COMMAND_EXEC,
		.help = "set/get number of extra tck for MEM-AP memory "
			"bus access [0-255]",
		.usage = "[cycles]",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration dap_command_handlers[] = {
	{
		.name = "dap",
		.mode = COMMAND_EXEC,
		.help = "DAP command group",
		.usage = "",
		.chain = dap_commands,
	},
	COMMAND_REGISTRATION_DONE
};

// SPDX-License-Identifier: GPL-2.0-or-later

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
 *   Copyright (C) 2013 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2019-2021, Ampere Computing LLC                         *
 ***************************************************************************/

/**
 * @file
 * This file implements support for the ARM Debug Interface version 5 (ADIv5)
 * debugging architecture.  Compared with previous versions, this includes
 * a low pin-count Serial Wire Debug (SWD) alternative to JTAG for message
 * transport, and focuses on memory mapped resources as defined by the
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
 * transaction pipelining.  The other involves adding delays to ensure the
 * AP has enough time to complete one operation before starting the next
 * one.  (For JTAG these delays are controlled by memaccess_tck.)
 */

/*
 * Relevant specifications from ARM include:
 *
 * ARM(tm) Debug Interface v5 Architecture Specification    ARM IHI 0031E
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
#include "arm_coresight.h"
#include "jtag/swd.h"
#include "transport/transport.h"
#include <helper/align.h>
#include <helper/jep106.h>
#include <helper/time_support.h>
#include <helper/list.h>
#include <helper/jim-nvp.h>

/* ARM ADI Specification requires at least 10 bits used for TAR autoincrement  */

/*
	uint32_t tar_block_size(uint32_t address)
	Return the largest block starting at address that does not cross a tar block size alignment boundary
*/
static uint32_t max_tar_block_size(uint32_t tar_autoincr_block, target_addr_t address)
{
	return tar_autoincr_block - ((tar_autoincr_block - 1) & address);
}

/***************************************************************************
 *                                                                         *
 * DP and MEM-AP  register access  through APACC and DPACC                 *
 *                                                                         *
***************************************************************************/

static int mem_ap_setup_csw(struct adiv5_ap *ap, uint32_t csw)
{
	csw |= ap->csw_default;

	if (csw != ap->csw_value) {
		/* LOG_DEBUG("DAP: Set CSW %x",csw); */
		int retval = dap_queue_ap_write(ap, MEM_AP_REG_CSW(ap->dap), csw);
		if (retval != ERROR_OK) {
			ap->csw_value = 0;
			return retval;
		}
		ap->csw_value = csw;
	}
	return ERROR_OK;
}

static int mem_ap_setup_tar(struct adiv5_ap *ap, target_addr_t tar)
{
	if (!ap->tar_valid || tar != ap->tar_value) {
		/* LOG_DEBUG("DAP: Set TAR %x",tar); */
		int retval = dap_queue_ap_write(ap, MEM_AP_REG_TAR(ap->dap), (uint32_t)(tar & 0xffffffffUL));
		if (retval == ERROR_OK && is_64bit_ap(ap)) {
			/* See if bits 63:32 of tar is different from last setting */
			if ((ap->tar_value >> 32) != (tar >> 32))
				retval = dap_queue_ap_write(ap, MEM_AP_REG_TAR64(ap->dap), (uint32_t)(tar >> 32));
		}
		if (retval != ERROR_OK) {
			ap->tar_valid = false;
			return retval;
		}
		ap->tar_value = tar;
		ap->tar_valid = true;
	}
	return ERROR_OK;
}

static int mem_ap_read_tar(struct adiv5_ap *ap, target_addr_t *tar)
{
	uint32_t lower;
	uint32_t upper = 0;

	int retval = dap_queue_ap_read(ap, MEM_AP_REG_TAR(ap->dap), &lower);
	if (retval == ERROR_OK && is_64bit_ap(ap))
		retval = dap_queue_ap_read(ap, MEM_AP_REG_TAR64(ap->dap), &upper);

	if (retval != ERROR_OK) {
		ap->tar_valid = false;
		return retval;
	}

	retval = dap_run(ap->dap);
	if (retval != ERROR_OK) {
		ap->tar_valid = false;
		return retval;
	}

	*tar = (((target_addr_t)upper) << 32) | (target_addr_t)lower;

	ap->tar_value = *tar;
	ap->tar_valid = true;
	return ERROR_OK;
}

static uint32_t mem_ap_get_tar_increment(struct adiv5_ap *ap)
{
	switch (ap->csw_value & CSW_ADDRINC_MASK) {
	case CSW_ADDRINC_SINGLE:
		switch (ap->csw_value & CSW_SIZE_MASK) {
		case CSW_8BIT:
			return 1;
		case CSW_16BIT:
			return 2;
		case CSW_32BIT:
			return 4;
		default:
			return 0;
		}
	case CSW_ADDRINC_PACKED:
		return 4;
	}
	return 0;
}

/* mem_ap_update_tar_cache is called after an access to MEM_AP_REG_DRW
 */
static void mem_ap_update_tar_cache(struct adiv5_ap *ap)
{
	if (!ap->tar_valid)
		return;

	uint32_t inc = mem_ap_get_tar_increment(ap);
	if (inc >= max_tar_block_size(ap->tar_autoincr_block, ap->tar_value))
		ap->tar_valid = false;
	else
		ap->tar_value += inc;
}

/**
 * Queue transactions setting up transfer parameters for the
 * currently selected MEM-AP.
 *
 * Subsequent transfers using registers like MEM_AP_REG_DRW or MEM_AP_REG_BD2
 * initiate data reads or writes using memory or peripheral addresses.
 * If the CSW is configured for it, the TAR may be automatically
 * incremented after each transfer.
 *
 * @param ap The MEM-AP.
 * @param csw MEM-AP Control/Status Word (CSW) register to assign.  If this
 *	matches the cached value, the register is not changed.
 * @param tar MEM-AP Transfer Address Register (TAR) to assign.  If this
 *	matches the cached address, the register is not changed.
 *
 * @return ERROR_OK if the transaction was properly queued, else a fault code.
 */
static int mem_ap_setup_transfer(struct adiv5_ap *ap, uint32_t csw, target_addr_t tar)
{
	int retval;
	retval = mem_ap_setup_csw(ap, csw);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_setup_tar(ap, tar);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

/**
 * Asynchronous (queued) read of a word from memory or a system register.
 *
 * @param ap The MEM-AP to access.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the word will be stored when the
 *	transaction queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_read_u32(struct adiv5_ap *ap, target_addr_t address,
		uint32_t *value)
{
	int retval;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when reading several consecutive addresses.
	 */
	retval = mem_ap_setup_transfer(ap,
			CSW_32BIT | (ap->csw_value & CSW_ADDRINC_MASK),
			address & 0xFFFFFFFFFFFFFFF0ull);
	if (retval != ERROR_OK)
		return retval;

	return dap_queue_ap_read(ap, MEM_AP_REG_BD0(ap->dap) | (address & 0xC), value);
}

/**
 * Synchronous read of a word from memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param ap The MEM-AP to access.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the result will be stored.
 *
 * @return ERROR_OK for success; *value holds the result.
 * Otherwise a fault code.
 */
int mem_ap_read_atomic_u32(struct adiv5_ap *ap, target_addr_t address,
		uint32_t *value)
{
	int retval;

	retval = mem_ap_read_u32(ap, address, value);
	if (retval != ERROR_OK)
		return retval;

	return dap_run(ap->dap);
}

/**
 * Asynchronous (queued) write of a word to memory or a system register.
 *
 * @param ap The MEM-AP to access.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written to the address when transaction
 *	queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_write_u32(struct adiv5_ap *ap, target_addr_t address,
		uint32_t value)
{
	int retval;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when writing several consecutive addresses.
	 */
	retval = mem_ap_setup_transfer(ap,
			CSW_32BIT | (ap->csw_value & CSW_ADDRINC_MASK),
			address & 0xFFFFFFFFFFFFFFF0ull);
	if (retval != ERROR_OK)
		return retval;

	return dap_queue_ap_write(ap, MEM_AP_REG_BD0(ap->dap) | (address & 0xC),
			value);
}

/**
 * Synchronous write of a word to memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param ap The MEM-AP to access.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written.
 *
 * @return ERROR_OK for success; the data was written.  Otherwise a fault code.
 */
int mem_ap_write_atomic_u32(struct adiv5_ap *ap, target_addr_t address,
		uint32_t value)
{
	int retval = mem_ap_write_u32(ap, address, value);

	if (retval != ERROR_OK)
		return retval;

	return dap_run(ap->dap);
}

/**
 * Synchronous write of a block of memory, using a specific access size.
 *
 * @param ap The MEM-AP to access.
 * @param buffer The data buffer to write. No particular alignment is assumed.
 * @param size Which access size to use, in bytes. 1, 2 or 4.
 * @param count The number of writes to do (in size units, not bytes).
 * @param address Address to be written; it must be writable by the currently selected MEM-AP.
 * @param addrinc Whether the target address should be increased for each write or not. This
 *  should normally be true, except when writing to e.g. a FIFO.
 * @return ERROR_OK on success, otherwise an error code.
 */
static int mem_ap_write(struct adiv5_ap *ap, const uint8_t *buffer, uint32_t size, uint32_t count,
		target_addr_t address, bool addrinc)
{
	struct adiv5_dap *dap = ap->dap;
	size_t nbytes = size * count;
	const uint32_t csw_addrincr = addrinc ? CSW_ADDRINC_SINGLE : CSW_ADDRINC_OFF;
	uint32_t csw_size;
	target_addr_t addr_xor;
	int retval = ERROR_OK;

	/* TI BE-32 Quirks mode:
	 * Writes on big-endian TMS570 behave very strangely. Observed behavior:
	 *   size   write address   bytes written in order
	 *   4      TAR ^ 0         (val >> 24), (val >> 16), (val >> 8), (val)
	 *   2      TAR ^ 2         (val >> 8), (val)
	 *   1      TAR ^ 3         (val)
	 * For example, if you attempt to write a single byte to address 0, the processor
	 * will actually write a byte to address 3.
	 *
	 * To make writes of size < 4 work as expected, we xor a value with the address before
	 * setting the TAP, and we set the TAP after every transfer rather then relying on
	 * address increment. */

	if (size == 4) {
		csw_size = CSW_32BIT;
		addr_xor = 0;
	} else if (size == 2) {
		csw_size = CSW_16BIT;
		addr_xor = dap->ti_be_32_quirks ? 2 : 0;
	} else if (size == 1) {
		csw_size = CSW_8BIT;
		addr_xor = dap->ti_be_32_quirks ? 3 : 0;
	} else {
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	if (ap->unaligned_access_bad && (address % size != 0))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	while (nbytes > 0) {
		uint32_t this_size = size;

		/* Select packed transfer if possible */
		if (addrinc && ap->packed_transfers && nbytes >= 4
				&& max_tar_block_size(ap->tar_autoincr_block, address) >= 4) {
			this_size = 4;
			retval = mem_ap_setup_csw(ap, csw_size | CSW_ADDRINC_PACKED);
		} else {
			retval = mem_ap_setup_csw(ap, csw_size | csw_addrincr);
		}

		if (retval != ERROR_OK)
			break;

		retval = mem_ap_setup_tar(ap, address ^ addr_xor);
		if (retval != ERROR_OK)
			return retval;

		/* How many source bytes each transfer will consume, and their location in the DRW,
		 * depends on the type of transfer and alignment. See ARM document IHI0031C. */
		uint32_t outvalue = 0;
		uint32_t drw_byte_idx = address;
		if (dap->ti_be_32_quirks) {
			switch (this_size) {
			case 4:
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx & 3) ^ addr_xor);
				break;
			case 2:
				outvalue |= (uint32_t)*buffer++ << 8 * (1 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (1 ^ (drw_byte_idx & 3) ^ addr_xor);
				break;
			case 1:
				outvalue |= (uint32_t)*buffer++ << 8 * (0 ^ (drw_byte_idx & 3) ^ addr_xor);
				break;
			}
		} else if (dap->nu_npcx_quirks) {
			switch (this_size) {
			case 4:
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx & 3);
				break;
			case 2:
				outvalue |= (uint32_t)*buffer << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*(buffer+1) << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx & 3);
				break;
			case 1:
				outvalue |= (uint32_t)*buffer << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx & 3);
			}
		} else {
			switch (this_size) {
			case 4:
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				/* fallthrough */
			case 2:
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				/* fallthrough */
			case 1:
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx & 3);
			}
		}

		nbytes -= this_size;

		retval = dap_queue_ap_write(ap, MEM_AP_REG_DRW(dap), outvalue);
		if (retval != ERROR_OK)
			break;

		mem_ap_update_tar_cache(ap);
		if (addrinc)
			address += this_size;
	}

	/* REVISIT: Might want to have a queued version of this function that does not run. */
	if (retval == ERROR_OK)
		retval = dap_run(dap);

	if (retval != ERROR_OK) {
		target_addr_t tar;
		if (mem_ap_read_tar(ap, &tar) == ERROR_OK)
			LOG_ERROR("Failed to write memory at " TARGET_ADDR_FMT, tar);
		else
			LOG_ERROR("Failed to write memory and, additionally, failed to find out where");
	}

	return retval;
}

/**
 * Synchronous read of a block of memory, using a specific access size.
 *
 * @param ap The MEM-AP to access.
 * @param buffer The data buffer to receive the data. No particular alignment is assumed.
 * @param size Which access size to use, in bytes. 1, 2 or 4.
 * @param count The number of reads to do (in size units, not bytes).
 * @param adr Address to be read; it must be readable by the currently selected MEM-AP.
 * @param addrinc Whether the target address should be increased after each read or not. This
 *  should normally be true, except when reading from e.g. a FIFO.
 * @return ERROR_OK on success, otherwise an error code.
 */
static int mem_ap_read(struct adiv5_ap *ap, uint8_t *buffer, uint32_t size, uint32_t count,
		target_addr_t adr, bool addrinc)
{
	struct adiv5_dap *dap = ap->dap;
	size_t nbytes = size * count;
	const uint32_t csw_addrincr = addrinc ? CSW_ADDRINC_SINGLE : CSW_ADDRINC_OFF;
	uint32_t csw_size;
	target_addr_t address = adr;
	int retval = ERROR_OK;

	/* TI BE-32 Quirks mode:
	 * Reads on big-endian TMS570 behave strangely differently than writes.
	 * They read from the physical address requested, but with DRW byte-reversed.
	 * For example, a byte read from address 0 will place the result in the high bytes of DRW.
	 * Also, packed 8-bit and 16-bit transfers seem to sometimes return garbage in some bytes,
	 * so avoid them. */

	if (size == 4)
		csw_size = CSW_32BIT;
	else if (size == 2)
		csw_size = CSW_16BIT;
	else if (size == 1)
		csw_size = CSW_8BIT;
	else
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (ap->unaligned_access_bad && (adr % size != 0))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Allocate buffer to hold the sequence of DRW reads that will be made. This is a significant
	 * over-allocation if packed transfers are going to be used, but determining the real need at
	 * this point would be messy. */
	uint32_t *read_buf = calloc(count, sizeof(uint32_t));
	/* Multiplication count * sizeof(uint32_t) may overflow, calloc() is safe */
	uint32_t *read_ptr = read_buf;
	if (!read_buf) {
		LOG_ERROR("Failed to allocate read buffer");
		return ERROR_FAIL;
	}

	/* Queue up all reads. Each read will store the entire DRW word in the read buffer. How many
	 * useful bytes it contains, and their location in the word, depends on the type of transfer
	 * and alignment. */
	while (nbytes > 0) {
		uint32_t this_size = size;

		/* Select packed transfer if possible */
		if (addrinc && ap->packed_transfers && nbytes >= 4
				&& max_tar_block_size(ap->tar_autoincr_block, address) >= 4) {
			this_size = 4;
			retval = mem_ap_setup_csw(ap, csw_size | CSW_ADDRINC_PACKED);
		} else {
			retval = mem_ap_setup_csw(ap, csw_size | csw_addrincr);
		}
		if (retval != ERROR_OK)
			break;

		retval = mem_ap_setup_tar(ap, address);
		if (retval != ERROR_OK)
			break;

		retval = dap_queue_ap_read(ap, MEM_AP_REG_DRW(dap), read_ptr++);
		if (retval != ERROR_OK)
			break;

		nbytes -= this_size;
		if (addrinc)
			address += this_size;

		mem_ap_update_tar_cache(ap);
	}

	if (retval == ERROR_OK)
		retval = dap_run(dap);

	/* Restore state */
	address = adr;
	nbytes = size * count;
	read_ptr = read_buf;

	/* If something failed, read TAR to find out how much data was successfully read, so we can
	 * at least give the caller what we have. */
	if (retval != ERROR_OK) {
		target_addr_t tar;
		if (mem_ap_read_tar(ap, &tar) == ERROR_OK) {
			/* TAR is incremented after failed transfer on some devices (eg Cortex-M4) */
			LOG_ERROR("Failed to read memory at " TARGET_ADDR_FMT, tar);
			if (nbytes > tar - address)
				nbytes = tar - address;
		} else {
			LOG_ERROR("Failed to read memory and, additionally, failed to find out where");
			nbytes = 0;
		}
	}

	/* Replay loop to populate caller's buffer from the correct word and byte lane */
	while (nbytes > 0) {
		uint32_t this_size = size;

		if (addrinc && ap->packed_transfers && nbytes >= 4
				&& max_tar_block_size(ap->tar_autoincr_block, address) >= 4) {
			this_size = 4;
		}

		if (dap->ti_be_32_quirks) {
			switch (this_size) {
			case 4:
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
				/* fallthrough */
			case 2:
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
				/* fallthrough */
			case 1:
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
			}
		} else {
			switch (this_size) {
			case 4:
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
				/* fallthrough */
			case 2:
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
				/* fallthrough */
			case 1:
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
			}
		}

		read_ptr++;
		nbytes -= this_size;
	}

	free(read_buf);
	return retval;
}

int mem_ap_read_buf(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address)
{
	return mem_ap_read(ap, buffer, size, count, address, true);
}

int mem_ap_write_buf(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address)
{
	return mem_ap_write(ap, buffer, size, count, address, true);
}

int mem_ap_read_buf_noincr(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address)
{
	return mem_ap_read(ap, buffer, size, count, address, false);
}

int mem_ap_write_buf_noincr(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address)
{
	return mem_ap_write(ap, buffer, size, count, address, false);
}

/*--------------------------------------------------------------------------*/


#define DAP_POWER_DOMAIN_TIMEOUT (10)

/*--------------------------------------------------------------------------*/

/**
 * Invalidate cached DP select and cached TAR and CSW of all APs
 */
void dap_invalidate_cache(struct adiv5_dap *dap)
{
	dap->select = DP_SELECT_INVALID;
	dap->last_read = NULL;

	int i;
	for (i = 0; i <= DP_APSEL_MAX; i++) {
		/* force csw and tar write on the next mem-ap access */
		dap->ap[i].tar_valid = false;
		dap->ap[i].csw_value = 0;
	}
}

/**
 * Initialize a DAP.  This sets up the power domains, prepares the DP
 * for further use and activates overrun checking.
 *
 * @param dap The DAP being initialized.
 */
int dap_dp_init(struct adiv5_dap *dap)
{
	int retval;

	LOG_DEBUG("%s", adiv5_dap_name(dap));

	dap->do_reconnect = false;
	dap_invalidate_cache(dap);

	/*
	 * Early initialize dap->dp_ctrl_stat.
	 * In jtag mode only, if the following queue run (in dap_dp_poll_register)
	 * fails and sets the sticky error, it will trigger the clearing
	 * of the sticky. Without this initialization system and debug power
	 * would be disabled while clearing the sticky error bit.
	 */
	dap->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	/*
	 * This write operation clears the sticky error bit in jtag mode only and
	 * is ignored in swd mode. It also powers-up system and debug domains in
	 * both jtag and swd modes, if not done before.
	 */
	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, dap->dp_ctrl_stat | SSTICKYERR);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, dap->dp_ctrl_stat);
	if (retval != ERROR_OK)
		return retval;

	/* Check that we have debug power domains activated */
	LOG_DEBUG("DAP: wait CDBGPWRUPACK");
	retval = dap_dp_poll_register(dap, DP_CTRL_STAT,
				      CDBGPWRUPACK, CDBGPWRUPACK,
				      DAP_POWER_DOMAIN_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	if (!dap->ignore_syspwrupack) {
		LOG_DEBUG("DAP: wait CSYSPWRUPACK");
		retval = dap_dp_poll_register(dap, DP_CTRL_STAT,
					      CSYSPWRUPACK, CSYSPWRUPACK,
					      DAP_POWER_DOMAIN_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
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

	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

/**
 * Initialize a DAP or do reconnect if DAP is not accessible.
 *
 * @param dap The DAP being initialized.
 */
int dap_dp_init_or_reconnect(struct adiv5_dap *dap)
{
	LOG_DEBUG("%s", adiv5_dap_name(dap));

	/*
	 * Early initialize dap->dp_ctrl_stat.
	 * In jtag mode only, if the following atomic reads fail and set the
	 * sticky error, it will trigger the clearing of the sticky. Without this
	 * initialization system and debug power would be disabled while clearing
	 * the sticky error bit.
	 */
	dap->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	dap->do_reconnect = false;

	dap_dp_read_atomic(dap, DP_CTRL_STAT, NULL);
	if (dap->do_reconnect) {
		/* dap connect calls dap_dp_init() after transport dependent initialization */
		return dap->ops->connect(dap);
	} else {
		return dap_dp_init(dap);
	}
}

/**
 * Initialize a DAP.  This sets up the power domains, prepares the DP
 * for further use, and arranges to use AP #0 for all AP operations
 * until dap_ap-select() changes that policy.
 *
 * @param ap The MEM-AP being initialized.
 */
int mem_ap_init(struct adiv5_ap *ap)
{
	/* check that we support packed transfers */
	uint32_t csw, cfg;
	int retval;
	struct adiv5_dap *dap = ap->dap;

	/* Set ap->cfg_reg before calling mem_ap_setup_transfer(). */
	/* mem_ap_setup_transfer() needs to know if the MEM_AP supports LPAE. */
	retval = dap_queue_ap_read(ap, MEM_AP_REG_CFG(dap), &cfg);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	ap->cfg_reg = cfg;
	ap->tar_valid = false;
	ap->csw_value = 0;      /* force csw and tar write */
	retval = mem_ap_setup_transfer(ap, CSW_8BIT | CSW_ADDRINC_PACKED, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_ap_read(ap, MEM_AP_REG_CSW(dap), &csw);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	if (csw & CSW_ADDRINC_PACKED)
		ap->packed_transfers = true;
	else
		ap->packed_transfers = false;

	/* Packed transfers on TI BE-32 processors do not work correctly in
	 * many cases. */
	if (dap->ti_be_32_quirks)
		ap->packed_transfers = false;

	LOG_DEBUG("MEM_AP Packed Transfers: %s",
			ap->packed_transfers ? "enabled" : "disabled");

	/* The ARM ADI spec leaves implementation-defined whether unaligned
	 * memory accesses work, only work partially, or cause a sticky error.
	 * On TI BE-32 processors, reads seem to return garbage in some bytes
	 * and unaligned writes seem to cause a sticky error.
	 * TODO: it would be nice to have a way to detect whether unaligned
	 * operations are supported on other processors. */
	ap->unaligned_access_bad = dap->ti_be_32_quirks;

	LOG_DEBUG("MEM_AP CFG: large data %d, long address %d, big-endian %d",
			!!(cfg & MEM_AP_REG_CFG_LD), !!(cfg & MEM_AP_REG_CFG_LA), !!(cfg & MEM_AP_REG_CFG_BE));

	return ERROR_OK;
}

/**
 * Put the debug link into SWD mode, if the target supports it.
 * The link's initial mode may be either JTAG (for example,
 * with SWJ-DP after reset) or SWD.
 *
 * Note that targets using the JTAG-DP do not support SWD, and that
 * some targets which could otherwise support it may have been
 * configured to disable SWD signaling
 *
 * @param dap The DAP used
 * @return ERROR_OK or else a fault code.
 */
int dap_to_swd(struct adiv5_dap *dap)
{
	LOG_DEBUG("Enter SWD mode");

	return dap_send_sequence(dap, JTAG_TO_SWD);
}

/**
 * Put the debug link into JTAG mode, if the target supports it.
 * The link's initial mode may be either SWD or JTAG.
 *
 * Note that targets implemented with SW-DP do not support JTAG, and
 * that some targets which could otherwise support it may have been
 * configured to disable JTAG signaling
 *
 * @param dap The DAP used
 * @return ERROR_OK or else a fault code.
 */
int dap_to_jtag(struct adiv5_dap *dap)
{
	LOG_DEBUG("Enter JTAG mode");

	return dap_send_sequence(dap, SWD_TO_JTAG);
}

/* CID interpretation -- see ARM IHI 0029E table B2-7
 * and ARM IHI 0031E table D1-2.
 *
 * From 2009/11/25 commit 21378f58b604:
 *   "OptimoDE DESS" is ARM's semicustom DSPish stuff.
 * Let's keep it as is, for the time being
 */
static const char *class_description[16] = {
	[0x0] = "Generic verification component",
	[0x1] = "ROM table",
	[0x2] = "Reserved",
	[0x3] = "Reserved",
	[0x4] = "Reserved",
	[0x5] = "Reserved",
	[0x6] = "Reserved",
	[0x7] = "Reserved",
	[0x8] = "Reserved",
	[0x9] = "CoreSight component",
	[0xA] = "Reserved",
	[0xB] = "Peripheral Test Block",
	[0xC] = "Reserved",
	[0xD] = "OptimoDE DESS", /* see above */
	[0xE] = "Generic IP component",
	[0xF] = "CoreLink, PrimeCell or System component",
};

#define ARCH_ID(architect, archid) ( \
	(((architect) << ARM_CS_C9_DEVARCH_ARCHITECT_SHIFT) & ARM_CS_C9_DEVARCH_ARCHITECT_MASK) | \
	(((archid) << ARM_CS_C9_DEVARCH_ARCHID_SHIFT) & ARM_CS_C9_DEVARCH_ARCHID_MASK) \
)

static const struct {
	uint32_t arch_id;
	const char *description;
} class0x9_devarch[] = {
	/* keep same unsorted order as in ARM IHI0029E */
	{ ARCH_ID(ARM_ID, 0x0A00), "RAS architecture" },
	{ ARCH_ID(ARM_ID, 0x1A01), "Instrumentation Trace Macrocell (ITM) architecture" },
	{ ARCH_ID(ARM_ID, 0x1A02), "DWT architecture" },
	{ ARCH_ID(ARM_ID, 0x1A03), "Flash Patch and Breakpoint unit (FPB) architecture" },
	{ ARCH_ID(ARM_ID, 0x2A04), "Processor debug architecture (ARMv8-M)" },
	{ ARCH_ID(ARM_ID, 0x6A05), "Processor debug architecture (ARMv8-R)" },
	{ ARCH_ID(ARM_ID, 0x0A10), "PC sample-based profiling" },
	{ ARCH_ID(ARM_ID, 0x4A13), "Embedded Trace Macrocell (ETM) architecture" },
	{ ARCH_ID(ARM_ID, 0x1A14), "Cross Trigger Interface (CTI) architecture" },
	{ ARCH_ID(ARM_ID, 0x6A15), "Processor debug architecture (v8.0-A)" },
	{ ARCH_ID(ARM_ID, 0x7A15), "Processor debug architecture (v8.1-A)" },
	{ ARCH_ID(ARM_ID, 0x8A15), "Processor debug architecture (v8.2-A)" },
	{ ARCH_ID(ARM_ID, 0x2A16), "Processor Performance Monitor (PMU) architecture" },
	{ ARCH_ID(ARM_ID, 0x0A17), "Memory Access Port v2 architecture" },
	{ ARCH_ID(ARM_ID, 0x0A27), "JTAG Access Port v2 architecture" },
	{ ARCH_ID(ARM_ID, 0x0A31), "Basic trace router" },
	{ ARCH_ID(ARM_ID, 0x0A37), "Power requestor" },
	{ ARCH_ID(ARM_ID, 0x0A47), "Unknown Access Port v2 architecture" },
	{ ARCH_ID(ARM_ID, 0x0A50), "HSSTP architecture" },
	{ ARCH_ID(ARM_ID, 0x0A63), "System Trace Macrocell (STM) architecture" },
	{ ARCH_ID(ARM_ID, 0x0A75), "CoreSight ELA architecture" },
	{ ARCH_ID(ARM_ID, 0x0AF7), "CoreSight ROM architecture" },
};

#define DEVARCH_ID_MASK         (ARM_CS_C9_DEVARCH_ARCHITECT_MASK | ARM_CS_C9_DEVARCH_ARCHID_MASK)
#define DEVARCH_MEM_AP          ARCH_ID(ARM_ID, 0x0A17)
#define DEVARCH_ROM_C_0X9       ARCH_ID(ARM_ID, 0x0AF7)
#define DEVARCH_UNKNOWN_V2      ARCH_ID(ARM_ID, 0x0A47)

static const char *class0x9_devarch_description(uint32_t devarch)
{
	if (!(devarch & ARM_CS_C9_DEVARCH_PRESENT))
		return "not present";

	for (unsigned int i = 0; i < ARRAY_SIZE(class0x9_devarch); i++)
		if ((devarch & DEVARCH_ID_MASK) == class0x9_devarch[i].arch_id)
			return class0x9_devarch[i].description;

	return "unknown";
}

static const struct {
	enum ap_type type;
	const char *description;
} ap_types[] = {
	{ AP_TYPE_JTAG_AP,  "JTAG-AP" },
	{ AP_TYPE_COM_AP,   "COM-AP" },
	{ AP_TYPE_AHB3_AP,  "MEM-AP AHB3" },
	{ AP_TYPE_APB_AP,   "MEM-AP APB2 or APB3" },
	{ AP_TYPE_AXI_AP,   "MEM-AP AXI3 or AXI4" },
	{ AP_TYPE_AHB5_AP,  "MEM-AP AHB5" },
	{ AP_TYPE_APB4_AP,  "MEM-AP APB4" },
	{ AP_TYPE_AXI5_AP,  "MEM-AP AXI5" },
	{ AP_TYPE_AHB5H_AP, "MEM-AP AHB5 with enhanced HPROT" },
};

static const char *ap_type_to_description(enum ap_type type)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ap_types); i++)
		if (type == ap_types[i].type)
			return ap_types[i].description;

	return "Unknown";
}

bool is_ap_num_valid(struct adiv5_dap *dap, uint64_t ap_num)
{
	if (!dap)
		return false;

	/* no autodetection, by now, so uninitialized is equivalent to ADIv5 for
	 * backward compatibility */
	if (!is_adiv6(dap)) {
		if (ap_num > DP_APSEL_MAX)
			return false;
		return true;
	}

	if (is_adiv6(dap)) {
		if (ap_num & 0x0fffULL)
			return false;
		if (dap->asize != 0)
			if (ap_num & ((~0ULL) << dap->asize))
				return false;
		return true;
	}

	return false;
}

/*
 * This function checks the ID for each access port to find the requested Access Port type
 * It also calls dap_get_ap() to increment the AP refcount
 */
int dap_find_get_ap(struct adiv5_dap *dap, enum ap_type type_to_find, struct adiv5_ap **ap_out)
{
	if (is_adiv6(dap)) {
		/* TODO: scan the ROM table and detect the AP available */
		LOG_DEBUG("On ADIv6 we cannot scan all the possible AP");
		return ERROR_FAIL;
	}

	/* Maximum AP number is 255 since the SELECT register is 8 bits */
	for (unsigned int ap_num = 0; ap_num <= DP_APSEL_MAX; ap_num++) {
		struct adiv5_ap *ap = dap_get_ap(dap, ap_num);
		if (!ap)
			continue;

		/* read the IDR register of the Access Port */
		uint32_t id_val = 0;

		int retval = dap_queue_ap_read(ap, AP_REG_IDR(dap), &id_val);
		if (retval != ERROR_OK) {
			dap_put_ap(ap);
			return retval;
		}

		retval = dap_run(dap);

		/* Reading register for a non-existent AP should not cause an error,
		 * but just to be sure, try to continue searching if an error does happen.
		 */
		if (retval == ERROR_OK && (id_val & AP_TYPE_MASK) == type_to_find) {
			LOG_DEBUG("Found %s at AP index: %d (IDR=0x%08" PRIX32 ")",
						ap_type_to_description(type_to_find),
						ap_num, id_val);

			*ap_out = ap;
			return ERROR_OK;
		}
		dap_put_ap(ap);
	}

	LOG_DEBUG("No %s found", ap_type_to_description(type_to_find));
	return ERROR_FAIL;
}

static inline bool is_ap_in_use(struct adiv5_ap *ap)
{
	return ap->refcount > 0 || ap->config_ap_never_release;
}

static struct adiv5_ap *_dap_get_ap(struct adiv5_dap *dap, uint64_t ap_num)
{
	if (!is_ap_num_valid(dap, ap_num)) {
		LOG_ERROR("Invalid AP#0x%" PRIx64, ap_num);
		return NULL;
	}
	if (is_adiv6(dap)) {
		for (unsigned int i = 0; i <= DP_APSEL_MAX; i++) {
			struct adiv5_ap *ap = &dap->ap[i];
			if (is_ap_in_use(ap) && ap->ap_num == ap_num) {
				++ap->refcount;
				return ap;
			}
		}
		for (unsigned int i = 0; i <= DP_APSEL_MAX; i++) {
			struct adiv5_ap *ap = &dap->ap[i];
			if (!is_ap_in_use(ap)) {
				ap->ap_num = ap_num;
				++ap->refcount;
				return ap;
			}
		}
		LOG_ERROR("No more AP available!");
		return NULL;
	}

	/* ADIv5 */
	struct adiv5_ap *ap = &dap->ap[ap_num];
	ap->ap_num = ap_num;
	++ap->refcount;
	return ap;
}

/* Return AP with specified ap_num. Increment AP refcount */
struct adiv5_ap *dap_get_ap(struct adiv5_dap *dap, uint64_t ap_num)
{
	struct adiv5_ap *ap = _dap_get_ap(dap, ap_num);
	if (ap)
		LOG_DEBUG("refcount AP#0x%" PRIx64 " get %u", ap_num, ap->refcount);
	return ap;
}

/* Return AP with specified ap_num. Increment AP refcount and keep it non-zero */
struct adiv5_ap *dap_get_config_ap(struct adiv5_dap *dap, uint64_t ap_num)
{
	struct adiv5_ap *ap = _dap_get_ap(dap, ap_num);
	if (ap) {
		ap->config_ap_never_release = true;
		LOG_DEBUG("refcount AP#0x%" PRIx64 " get_config %u", ap_num, ap->refcount);
	}
	return ap;
}

/* Decrement AP refcount and release the AP when refcount reaches zero */
int dap_put_ap(struct adiv5_ap *ap)
{
	if (ap->refcount == 0) {
		LOG_ERROR("BUG: refcount AP#0x%" PRIx64 " put underflow", ap->ap_num);
		return ERROR_FAIL;
	}

	--ap->refcount;

	LOG_DEBUG("refcount AP#0x%" PRIx64 " put %u", ap->ap_num, ap->refcount);
	if (!is_ap_in_use(ap)) {
		/* defaults from dap_instance_init() */
		ap->ap_num = DP_APSEL_INVALID;
		ap->memaccess_tck = 255;
		ap->tar_autoincr_block = (1 << 10);
		ap->csw_default = CSW_AHB_DEFAULT;
		ap->cfg_reg = MEM_AP_REG_CFG_INVALID;
	}
	return ERROR_OK;
}

static int dap_get_debugbase(struct adiv5_ap *ap,
			target_addr_t *dbgbase, uint32_t *apid)
{
	struct adiv5_dap *dap = ap->dap;
	int retval;
	uint32_t baseptr_upper, baseptr_lower;

	if (ap->cfg_reg == MEM_AP_REG_CFG_INVALID) {
		retval = dap_queue_ap_read(ap, MEM_AP_REG_CFG(dap), &ap->cfg_reg);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = dap_queue_ap_read(ap, MEM_AP_REG_BASE(dap), &baseptr_lower);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_ap_read(ap, AP_REG_IDR(dap), apid);
	if (retval != ERROR_OK)
		return retval;
	/* MEM_AP_REG_BASE64 is defined as 'RES0'; can be read and then ignored on 32 bits AP */
	if (ap->cfg_reg == MEM_AP_REG_CFG_INVALID || is_64bit_ap(ap)) {
		retval = dap_queue_ap_read(ap, MEM_AP_REG_BASE64(dap), &baseptr_upper);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	if (!is_64bit_ap(ap))
		baseptr_upper = 0;
	*dbgbase = (((target_addr_t)baseptr_upper) << 32) | baseptr_lower;

	return ERROR_OK;
}

int adiv6_dap_read_baseptr(struct command_invocation *cmd, struct adiv5_dap *dap, uint64_t *baseptr)
{
	uint32_t baseptr_lower, baseptr_upper = 0;
	int retval;

	if (dap->asize > 32) {
		retval = dap_queue_dp_read(dap, DP_BASEPTR1, &baseptr_upper);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = dap_dp_read_atomic(dap, DP_BASEPTR0, &baseptr_lower);
	if (retval != ERROR_OK)
		return retval;

	if ((baseptr_lower & DP_BASEPTR0_VALID) != DP_BASEPTR0_VALID) {
		command_print(cmd, "System root table not present");
		return ERROR_FAIL;
	}

	baseptr_lower &= ~0x0fff;
	*baseptr = (((uint64_t)baseptr_upper) << 32) | baseptr_lower;

	return ERROR_OK;
}

/**
 * Method to access the CoreSight component.
 * On ADIv5, CoreSight components are on the bus behind a MEM-AP.
 * On ADIv6, CoreSight components can either be on the bus behind a MEM-AP
 * or directly in the AP.
 */
enum coresight_access_mode {
	CS_ACCESS_AP,
	CS_ACCESS_MEM_AP,
};

/** Holds registers and coordinates of a CoreSight component */
struct cs_component_vals {
	struct adiv5_ap *ap;
	target_addr_t component_base;
	uint64_t pid;
	uint32_t cid;
	uint32_t devarch;
	uint32_t devid;
	uint32_t devtype_memtype;
	enum coresight_access_mode mode;
};

/**
 * Helper to read CoreSight component's registers, either on the bus
 * behind a MEM-AP or directly in the AP.
 *
 * @param mode           Method to access the component (AP or MEM-AP).
 * @param ap             Pointer to AP containing the component.
 * @param component_base On MEM-AP access method, base address of the component.
 * @param reg            Offset of the component's register to read.
 * @param value          Pointer to the store the read value.
 *
 * @return ERROR_OK on success, else a fault code.
 */
static int dap_queue_read_reg(enum coresight_access_mode mode, struct adiv5_ap *ap,
		uint64_t component_base, unsigned int reg, uint32_t *value)
{
	if (mode == CS_ACCESS_AP)
		return dap_queue_ap_read(ap, reg, value);

	/* mode == CS_ACCESS_MEM_AP */
	return mem_ap_read_u32(ap, component_base + reg, value);
}

/**
 * Read the CoreSight registers needed during ROM Table Parsing (RTP).
 *
 * @param mode           Method to access the component (AP or MEM-AP).
 * @param ap             Pointer to AP containing the component.
 * @param component_base On MEM-AP access method, base address of the component.
 * @param v              Pointer to the struct holding the value of registers.
 *
 * @return ERROR_OK on success, else a fault code.
 */
static int rtp_read_cs_regs(enum coresight_access_mode mode, struct adiv5_ap *ap,
		target_addr_t component_base, struct cs_component_vals *v)
{
	assert(IS_ALIGNED(component_base, ARM_CS_ALIGN));
	assert(ap && v);

	uint32_t cid0, cid1, cid2, cid3;
	uint32_t pid0, pid1, pid2, pid3, pid4;
	int retval = ERROR_OK;

	v->ap = ap;
	v->component_base = component_base;
	v->mode = mode;

	/* sort by offset to gain speed */

	/*
	 * Registers DEVARCH, DEVID and DEVTYPE are valid on Class 0x9 devices
	 * only, but are at offset above 0xf00, so can be read on any device
	 * without triggering error. Read them for eventual use on Class 0x9.
	 */
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_C9_DEVARCH, &v->devarch);

	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_C9_DEVID, &v->devid);

	/* Same address as ARM_CS_C1_MEMTYPE */
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_C9_DEVTYPE, &v->devtype_memtype);

	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_PIDR4, &pid4);

	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_PIDR0, &pid0);
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_PIDR1, &pid1);
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_PIDR2, &pid2);
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_PIDR3, &pid3);

	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_CIDR0, &cid0);
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_CIDR1, &cid1);
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_CIDR2, &cid2);
	if (retval == ERROR_OK)
		retval = dap_queue_read_reg(mode, ap, component_base, ARM_CS_CIDR3, &cid3);

	if (retval == ERROR_OK)
		retval = dap_run(ap->dap);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Failed read CoreSight registers");
		return retval;
	}

	v->cid = (cid3 & 0xff) << 24
			| (cid2 & 0xff) << 16
			| (cid1 & 0xff) << 8
			| (cid0 & 0xff);
	v->pid = (uint64_t)(pid4 & 0xff) << 32
			| (pid3 & 0xff) << 24
			| (pid2 & 0xff) << 16
			| (pid1 & 0xff) << 8
			| (pid0 & 0xff);

	return ERROR_OK;
}

/* Part number interpretations are from Cortex
 * core specs, the CoreSight components TRM
 * (ARM DDI 0314H), CoreSight System Design
 * Guide (ARM DGI 0012D) and ETM specs; also
 * from chip observation (e.g. TI SDTI).
 */

static const struct dap_part_nums {
	uint16_t designer_id;
	uint16_t part_num;
	const char *type;
	const char *full;
} dap_part_nums[] = {
	{ ARM_ID, 0x000, "Cortex-M3 SCS",              "(System Control Space)", },
	{ ARM_ID, 0x001, "Cortex-M3 ITM",              "(Instrumentation Trace Module)", },
	{ ARM_ID, 0x002, "Cortex-M3 DWT",              "(Data Watchpoint and Trace)", },
	{ ARM_ID, 0x003, "Cortex-M3 FPB",              "(Flash Patch and Breakpoint)", },
	{ ARM_ID, 0x008, "Cortex-M0 SCS",              "(System Control Space)", },
	{ ARM_ID, 0x00a, "Cortex-M0 DWT",              "(Data Watchpoint and Trace)", },
	{ ARM_ID, 0x00b, "Cortex-M0 BPU",              "(Breakpoint Unit)", },
	{ ARM_ID, 0x00c, "Cortex-M4 SCS",              "(System Control Space)", },
	{ ARM_ID, 0x00d, "CoreSight ETM11",            "(Embedded Trace)", },
	{ ARM_ID, 0x00e, "Cortex-M7 FPB",              "(Flash Patch and Breakpoint)", },
	{ ARM_ID, 0x193, "SoC-600 TSGEN",              "(Timestamp Generator)", },
	{ ARM_ID, 0x470, "Cortex-M1 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x471, "Cortex-M0 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x490, "Cortex-A15 GIC",             "(Generic Interrupt Controller)", },
	{ ARM_ID, 0x492, "Cortex-R52 GICD",            "(Distributor)", },
	{ ARM_ID, 0x493, "Cortex-R52 GICR",            "(Redistributor)", },
	{ ARM_ID, 0x4a1, "Cortex-A53 ROM",             "(v8 Memory Map ROM Table)", },
	{ ARM_ID, 0x4a2, "Cortex-A57 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4a3, "Cortex-A53 ROM",             "(v7 Memory Map ROM Table)", },
	{ ARM_ID, 0x4a4, "Cortex-A72 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4a9, "Cortex-A9 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4aa, "Cortex-A35 ROM",             "(v8 Memory Map ROM Table)", },
	{ ARM_ID, 0x4af, "Cortex-A15 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4b5, "Cortex-R5 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4b8, "Cortex-R52 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4c0, "Cortex-M0+ ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4c3, "Cortex-M3 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4c4, "Cortex-M4 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4c7, "Cortex-M7 PPB ROM",          "(Private Peripheral Bus ROM Table)", },
	{ ARM_ID, 0x4c8, "Cortex-M7 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4e0, "Cortex-A35 ROM",             "(v7 Memory Map ROM Table)", },
	{ ARM_ID, 0x4e4, "Cortex-A76 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x906, "CoreSight CTI",              "(Cross Trigger)", },
	{ ARM_ID, 0x907, "CoreSight ETB",              "(Trace Buffer)", },
	{ ARM_ID, 0x908, "CoreSight CSTF",             "(Trace Funnel)", },
	{ ARM_ID, 0x909, "CoreSight ATBR",             "(Advanced Trace Bus Replicator)", },
	{ ARM_ID, 0x910, "CoreSight ETM9",             "(Embedded Trace)", },
	{ ARM_ID, 0x912, "CoreSight TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x913, "CoreSight ITM",              "(Instrumentation Trace Macrocell)", },
	{ ARM_ID, 0x914, "CoreSight SWO",              "(Single Wire Output)", },
	{ ARM_ID, 0x917, "CoreSight HTM",              "(AHB Trace Macrocell)", },
	{ ARM_ID, 0x920, "CoreSight ETM11",            "(Embedded Trace)", },
	{ ARM_ID, 0x921, "Cortex-A8 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x922, "Cortex-A8 CTI",              "(Cross Trigger)", },
	{ ARM_ID, 0x923, "Cortex-M3 TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x924, "Cortex-M3 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x925, "Cortex-M4 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x930, "Cortex-R4 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x931, "Cortex-R5 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x932, "CoreSight MTB-M0+",          "(Micro Trace Buffer)", },
	{ ARM_ID, 0x941, "CoreSight TPIU-Lite",        "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x950, "Cortex-A9 PTM",              "(Program Trace Macrocell)", },
	{ ARM_ID, 0x955, "Cortex-A5 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x95a, "Cortex-A72 ETM",             "(Embedded Trace)", },
	{ ARM_ID, 0x95b, "Cortex-A17 PTM",             "(Program Trace Macrocell)", },
	{ ARM_ID, 0x95d, "Cortex-A53 ETM",             "(Embedded Trace)", },
	{ ARM_ID, 0x95e, "Cortex-A57 ETM",             "(Embedded Trace)", },
	{ ARM_ID, 0x95f, "Cortex-A15 PTM",             "(Program Trace Macrocell)", },
	{ ARM_ID, 0x961, "CoreSight TMC",              "(Trace Memory Controller)", },
	{ ARM_ID, 0x962, "CoreSight STM",              "(System Trace Macrocell)", },
	{ ARM_ID, 0x975, "Cortex-M7 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x9a0, "CoreSight PMU",              "(Performance Monitoring Unit)", },
	{ ARM_ID, 0x9a1, "Cortex-M4 TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x9a4, "CoreSight GPR",              "(Granular Power Requester)", },
	{ ARM_ID, 0x9a5, "Cortex-A5 PMU",              "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9a7, "Cortex-A7 PMU",              "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9a8, "Cortex-A53 CTI",             "(Cross Trigger)", },
	{ ARM_ID, 0x9a9, "Cortex-M7 TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x9ae, "Cortex-A17 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9af, "Cortex-A15 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9b6, "Cortex-R52 PMU/CTI/ETM",     "(Performance Monitor Unit/Cross Trigger/ETM)", },
	{ ARM_ID, 0x9b7, "Cortex-R7 PMU",              "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9d3, "Cortex-A53 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9d7, "Cortex-A57 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9d8, "Cortex-A72 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9da, "Cortex-A35 PMU/CTI/ETM",     "(Performance Monitor Unit/Cross Trigger/ETM)", },
	{ ARM_ID, 0x9e2, "SoC-600 APB-AP",             "(APB4 Memory Access Port)", },
	{ ARM_ID, 0x9e3, "SoC-600 AHB-AP",             "(AHB5 Memory Access Port)", },
	{ ARM_ID, 0x9e4, "SoC-600 AXI-AP",             "(AXI Memory Access Port)", },
	{ ARM_ID, 0x9e5, "SoC-600 APv1 Adapter",       "(Access Port v1 Adapter)", },
	{ ARM_ID, 0x9e6, "SoC-600 JTAG-AP",            "(JTAG Access Port)", },
	{ ARM_ID, 0x9e7, "SoC-600 TPIU",               "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x9e8, "SoC-600 TMC ETR/ETS",        "(Embedded Trace Router/Streamer)", },
	{ ARM_ID, 0x9e9, "SoC-600 TMC ETB",            "(Embedded Trace Buffer)", },
	{ ARM_ID, 0x9ea, "SoC-600 TMC ETF",            "(Embedded Trace FIFO)", },
	{ ARM_ID, 0x9eb, "SoC-600 ATB Funnel",         "(Trace Funnel)", },
	{ ARM_ID, 0x9ec, "SoC-600 ATB Replicator",     "(Trace Replicator)", },
	{ ARM_ID, 0x9ed, "SoC-600 CTI",                "(Cross Trigger)", },
	{ ARM_ID, 0x9ee, "SoC-600 CATU",               "(Address Translation Unit)", },
	{ ARM_ID, 0xc05, "Cortex-A5 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc07, "Cortex-A7 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc08, "Cortex-A8 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc09, "Cortex-A9 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc0e, "Cortex-A17 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xc0f, "Cortex-A15 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xc14, "Cortex-R4 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc15, "Cortex-R5 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc17, "Cortex-R7 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xd03, "Cortex-A53 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd04, "Cortex-A35 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd07, "Cortex-A57 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd08, "Cortex-A72 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd0b, "Cortex-A76 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd0c, "Neoverse N1",                "(Debug Unit)", },
	{ ARM_ID, 0xd13, "Cortex-R52 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd49, "Neoverse N2",                "(Debug Unit)", },
	{ 0x017,  0x120, "TI SDTI",                    "(System Debug Trace Interface)", }, /* from OMAP3 memmap */
	{ 0x017,  0x343, "TI DAPCTL",                  "", }, /* from OMAP3 memmap */
	{ 0x017,  0x9af, "MSP432 ROM",                 "(ROM Table)" },
	{ 0x01f,  0xcd0, "Atmel CPU with DSU",         "(CPU)" },
	{ 0x041,  0x1db, "XMC4500 ROM",                "(ROM Table)" },
	{ 0x041,  0x1df, "XMC4700/4800 ROM",           "(ROM Table)" },
	{ 0x041,  0x1ed, "XMC1000 ROM",                "(ROM Table)" },
	{ 0x065,  0x000, "SHARC+/Blackfin+",           "", },
	{ 0x070,  0x440, "Qualcomm QDSS Component v1", "(Qualcomm Designed CoreSight Component v1)", },
	{ 0x0bf,  0x100, "Brahma-B53 Debug",           "(Debug Unit)", },
	{ 0x0bf,  0x9d3, "Brahma-B53 PMU",             "(Performance Monitor Unit)", },
	{ 0x0bf,  0x4a1, "Brahma-B53 ROM",             "(ROM Table)", },
	{ 0x0bf,  0x721, "Brahma-B53 ROM",             "(ROM Table)", },
	{ 0x1eb,  0x181, "Tegra 186 ROM",              "(ROM Table)", },
	{ 0x1eb,  0x202, "Denver ETM",                 "(Denver Embedded Trace)", },
	{ 0x1eb,  0x211, "Tegra 210 ROM",              "(ROM Table)", },
	{ 0x1eb,  0x302, "Denver Debug",               "(Debug Unit)", },
	{ 0x1eb,  0x402, "Denver PMU",                 "(Performance Monitor Unit)", },
};

static const struct dap_part_nums *pidr_to_part_num(unsigned int designer_id, unsigned int part_num)
{
	static const struct dap_part_nums unknown = {
		.type = "Unrecognized",
		.full = "",
	};

	for (unsigned int i = 0; i < ARRAY_SIZE(dap_part_nums); i++)
		if (dap_part_nums[i].designer_id == designer_id && dap_part_nums[i].part_num == part_num)
			return &dap_part_nums[i];

	return &unknown;
}

static int dap_devtype_display(struct command_invocation *cmd, uint32_t devtype)
{
	const char *major = "Reserved", *subtype = "Reserved";
	const unsigned int minor = (devtype & ARM_CS_C9_DEVTYPE_SUB_MASK) >> ARM_CS_C9_DEVTYPE_SUB_SHIFT;
	const unsigned int devtype_major = (devtype & ARM_CS_C9_DEVTYPE_MAJOR_MASK) >> ARM_CS_C9_DEVTYPE_MAJOR_SHIFT;
	switch (devtype_major) {
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
		case 3:
			subtype = "Router";
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
		case 6:
			subtype = "Software";
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
		case 3:
			subtype = "Power Requestor";
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
		case 4:
			subtype = "Bus";
			break;
		case 5:
			subtype = "Memory";
			break;
		}
		break;
	case 6:
		major = "Performance Monitor";
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
		case 5:
			subtype = "Memory";
			break;
		}
		break;
	}
	command_print(cmd, "\t\tType is 0x%02x, %s, %s",
			devtype & ARM_CS_C9_DEVTYPE_MASK,
			major, subtype);
	return ERROR_OK;
}

/**
 * Actions/operations to be executed while parsing ROM tables.
 */
struct rtp_ops {
	/**
	 * Executed at the start of a new AP, typically to print the AP header.
	 * @param ap        Pointer to AP.
	 * @param depth     The current depth level of ROM table.
	 * @param priv      Pointer to private data.
	 * @return          ERROR_OK on success, else a fault code.
	 */
	int (*ap_header)(struct adiv5_ap *ap, int depth, void *priv);
	/**
	 * Executed at the start of a new MEM-AP, typically to print the MEM-AP header.
	 * @param retval    Error encountered while reading AP.
	 * @param ap        Pointer to AP.
	 * @param dbgbase   Value of MEM-AP Debug Base Address register.
	 * @param apid      Value of MEM-AP IDR Identification Register.
	 * @param depth     The current depth level of ROM table.
	 * @param priv      Pointer to private data.
	 * @return          ERROR_OK on success, else a fault code.
	 */
	int (*mem_ap_header)(int retval, struct adiv5_ap *ap, uint64_t dbgbase,
			uint32_t apid, int depth, void *priv);
	/**
	 * Executed when a CoreSight component is parsed, typically to print
	 * information on the component.
	 * @param retval    Error encountered while reading component's registers.
	 * @param v         Pointer to a container of the component's registers.
	 * @param depth     The current depth level of ROM table.
	 * @param priv      Pointer to private data.
	 * @return          ERROR_OK on success, else a fault code.
	 */
	int (*cs_component)(int retval, struct cs_component_vals *v, int depth, void *priv);
	/**
	 * Executed for each entry of a ROM table, typically to print the entry
	 * and information about validity or end-of-table mark.
	 * @param retval    Error encountered while reading the ROM table entry.
	 * @param depth     The current depth level of ROM table.
	 * @param offset    The offset of the entry in the ROM table.
	 * @param romentry  The value of the ROM table entry.
	 * @param priv      Pointer to private data.
	 * @return          ERROR_OK on success, else a fault code.
	 */
	int (*rom_table_entry)(int retval, int depth, unsigned int offset, uint64_t romentry,
			void *priv);
	/**
	 * Private data
	 */
	void *priv;
};

/**
 * Wrapper around struct rtp_ops::ap_header.
 */
static int rtp_ops_ap_header(const struct rtp_ops *ops,
		struct adiv5_ap *ap, int depth)
{
	if (ops->ap_header)
		return ops->ap_header(ap, depth, ops->priv);

	return ERROR_OK;
}

/**
 * Wrapper around struct rtp_ops::mem_ap_header.
 * Input parameter @a retval is propagated.
 */
static int rtp_ops_mem_ap_header(const struct rtp_ops *ops,
		int retval, struct adiv5_ap *ap, uint64_t dbgbase, uint32_t apid, int depth)
{
	if (!ops->mem_ap_header)
		return retval;

	int retval1 = ops->mem_ap_header(retval, ap, dbgbase, apid, depth, ops->priv);
	if (retval != ERROR_OK)
		return retval;
	return retval1;
}

/**
 * Wrapper around struct rtp_ops::cs_component.
 * Input parameter @a retval is propagated.
 */
static int rtp_ops_cs_component(const struct rtp_ops *ops,
		int retval, struct cs_component_vals *v, int depth)
{
	if (!ops->cs_component)
		return retval;

	int retval1 = ops->cs_component(retval, v, depth, ops->priv);
	if (retval != ERROR_OK)
		return retval;
	return retval1;
}

/**
 * Wrapper around struct rtp_ops::rom_table_entry.
 * Input parameter @a retval is propagated.
 */
static int rtp_ops_rom_table_entry(const struct rtp_ops *ops,
		int retval, int depth, unsigned int offset, uint64_t romentry)
{
	if (!ops->rom_table_entry)
		return retval;

	int retval1 = ops->rom_table_entry(retval, depth, offset, romentry, ops->priv);
	if (retval != ERROR_OK)
		return retval;
	return retval1;
}

/* Broken ROM tables can have circular references. Stop after a while */
#define ROM_TABLE_MAX_DEPTH (16)

/**
 * Value used only during lookup of a CoreSight component in ROM table.
 * Return CORESIGHT_COMPONENT_FOUND when component is found.
 * Return ERROR_OK when component is not found yet.
 * Return any other ERROR_* in case of error.
 */
#define CORESIGHT_COMPONENT_FOUND (1)

static int rtp_ap(const struct rtp_ops *ops, struct adiv5_ap *ap, int depth);
static int rtp_cs_component(enum coresight_access_mode mode, const struct rtp_ops *ops,
		struct adiv5_ap *ap, target_addr_t dbgbase, bool *is_mem_ap, int depth);

static int rtp_rom_loop(enum coresight_access_mode mode, const struct rtp_ops *ops,
		struct adiv5_ap *ap, target_addr_t base_address, int depth,
		unsigned int width, unsigned int max_entries)
{
	/* ADIv6 AP ROM table provide offset from current AP */
	if (mode == CS_ACCESS_AP)
		base_address = ap->ap_num;

	assert(IS_ALIGNED(base_address, ARM_CS_ALIGN));

	unsigned int offset = 0;
	while (max_entries--) {
		uint64_t romentry;
		uint32_t romentry_low, romentry_high;
		target_addr_t component_base;
		unsigned int saved_offset = offset;

		int retval = dap_queue_read_reg(mode, ap, base_address, offset, &romentry_low);
		offset += 4;
		if (retval == ERROR_OK && width == 64) {
			retval = dap_queue_read_reg(mode, ap, base_address, offset, &romentry_high);
			offset += 4;
		}
		if (retval == ERROR_OK)
			retval = dap_run(ap->dap);
		if (retval != ERROR_OK) {
			LOG_DEBUG("Failed read ROM table entry");
			return retval;
		}

		if (width == 64) {
			romentry = (((uint64_t)romentry_high) << 32) | romentry_low;
			component_base = base_address +
				((((uint64_t)romentry_high) << 32) | (romentry_low & ARM_CS_ROMENTRY_OFFSET_MASK));
		} else {
			romentry = romentry_low;
			/* "romentry" is signed */
			component_base = base_address + (int32_t)(romentry_low & ARM_CS_ROMENTRY_OFFSET_MASK);
			if (!is_64bit_ap(ap))
				component_base = (uint32_t)component_base;
		}
		retval = rtp_ops_rom_table_entry(ops, retval, depth, saved_offset, romentry);
		if (retval != ERROR_OK)
			return retval;

		if (romentry == 0) {
			/* End of ROM table */
			break;
		}

		if (!(romentry & ARM_CS_ROMENTRY_PRESENT))
			continue;

		/* Recurse */
		if (mode == CS_ACCESS_AP) {
			struct adiv5_ap *next_ap = dap_get_ap(ap->dap, component_base);
			if (!next_ap) {
				LOG_DEBUG("Wrong AP # 0x%" PRIx64, component_base);
				continue;
			}
			retval = rtp_ap(ops, next_ap, depth + 1);
			dap_put_ap(next_ap);
		} else {
			/* mode == CS_ACCESS_MEM_AP */
			retval = rtp_cs_component(mode, ops, ap, component_base, NULL, depth + 1);
		}
		if (retval == CORESIGHT_COMPONENT_FOUND)
			return CORESIGHT_COMPONENT_FOUND;
		if (retval != ERROR_OK) {
			/* TODO: do we need to send an ABORT before continuing? */
			LOG_DEBUG("Ignore error parsing CoreSight component");
			continue;
		}
	}

	return ERROR_OK;
}

static int rtp_cs_component(enum coresight_access_mode mode, const struct rtp_ops *ops,
		struct adiv5_ap *ap, target_addr_t base_address, bool *is_mem_ap, int depth)
{
	struct cs_component_vals v;
	int retval;

	assert(IS_ALIGNED(base_address, ARM_CS_ALIGN));

	if (is_mem_ap)
		*is_mem_ap = false;

	if (depth > ROM_TABLE_MAX_DEPTH)
		retval = ERROR_FAIL;
	else
		retval = rtp_read_cs_regs(mode, ap, base_address, &v);

	retval = rtp_ops_cs_component(ops, retval, &v, depth);
	if (retval == CORESIGHT_COMPONENT_FOUND)
		return CORESIGHT_COMPONENT_FOUND;
	if (retval != ERROR_OK)
		return ERROR_OK; /* Don't abort recursion */

	if (!is_valid_arm_cs_cidr(v.cid))
		return ERROR_OK; /* Don't abort recursion */

	const unsigned int class = ARM_CS_CIDR_CLASS(v.cid);

	if (class == ARM_CS_CLASS_0X1_ROM_TABLE)
		return rtp_rom_loop(mode, ops, ap, base_address, depth, 32, 960);

	if (class == ARM_CS_CLASS_0X9_CS_COMPONENT) {
		if ((v.devarch & ARM_CS_C9_DEVARCH_PRESENT) == 0)
			return ERROR_OK;

		if (is_mem_ap) {
			if ((v.devarch & DEVARCH_ID_MASK) == DEVARCH_MEM_AP)
				*is_mem_ap = true;

			/* SoC-600 APv1 Adapter */
			if ((v.devarch & DEVARCH_ID_MASK) == DEVARCH_UNKNOWN_V2 &&
					ARM_CS_PIDR_DESIGNER(v.pid) == ARM_ID &&
					ARM_CS_PIDR_PART(v.pid) == 0x9e5)
				*is_mem_ap = true;
		}

		/* quit if not ROM table */
		if ((v.devarch & DEVARCH_ID_MASK) != DEVARCH_ROM_C_0X9)
			return ERROR_OK;

		if ((v.devid & ARM_CS_C9_DEVID_FORMAT_MASK) == ARM_CS_C9_DEVID_FORMAT_64BIT)
			return rtp_rom_loop(mode, ops, ap, base_address, depth, 64, 256);
		else
			return rtp_rom_loop(mode, ops, ap, base_address, depth, 32, 512);
	}

	/* Class other than 0x1 and 0x9 */
	return ERROR_OK;
}

static int rtp_ap(const struct rtp_ops *ops, struct adiv5_ap *ap, int depth)
{
	uint32_t apid;
	target_addr_t dbgbase, invalid_entry;

	int retval = rtp_ops_ap_header(ops, ap, depth);
	if (retval != ERROR_OK || depth > ROM_TABLE_MAX_DEPTH)
		return ERROR_OK; /* Don't abort recursion */

	if (is_adiv6(ap->dap)) {
		bool is_mem_ap;
		retval = rtp_cs_component(CS_ACCESS_AP, ops, ap, 0, &is_mem_ap, depth);
		if (retval == CORESIGHT_COMPONENT_FOUND)
			return CORESIGHT_COMPONENT_FOUND;
		if (retval != ERROR_OK)
			return ERROR_OK; /* Don't abort recursion */

		if (!is_mem_ap)
			return ERROR_OK;
		/* Continue for an ADIv6 MEM-AP or SoC-600 APv1 Adapter */
	}

	/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
	retval = dap_get_debugbase(ap, &dbgbase, &apid);
	if (retval != ERROR_OK)
		return retval;
	retval = rtp_ops_mem_ap_header(ops, retval, ap, dbgbase, apid, depth);
	if (retval != ERROR_OK)
		return retval;

	if (apid == 0)
		return ERROR_FAIL;

	/* NOTE: a MEM-AP may have a single CoreSight component that's
	 * not a ROM table ... or have no such components at all.
	 */
	const unsigned int class = (apid & AP_REG_IDR_CLASS_MASK) >> AP_REG_IDR_CLASS_SHIFT;

	if (class == AP_REG_IDR_CLASS_MEM_AP) {
		if (is_64bit_ap(ap))
			invalid_entry = 0xFFFFFFFFFFFFFFFFull;
		else
			invalid_entry = 0xFFFFFFFFul;

		if (dbgbase != invalid_entry && (dbgbase & 0x3) != 0x2) {
			retval = rtp_cs_component(CS_ACCESS_MEM_AP, ops, ap,
					dbgbase & 0xFFFFFFFFFFFFF000ull, NULL, depth);
			if (retval == CORESIGHT_COMPONENT_FOUND)
				return CORESIGHT_COMPONENT_FOUND;
		}
	}

	return ERROR_OK;
}

/* Actions for command "dap info" */

static int dap_info_ap_header(struct adiv5_ap *ap, int depth, void *priv)
{
	struct command_invocation *cmd = priv;

	if (depth > ROM_TABLE_MAX_DEPTH) {
		command_print(cmd, "\tTables too deep");
		return ERROR_FAIL;
	}

	command_print(cmd, "%sAP # 0x%" PRIx64, (depth) ? "\t\t" : "", ap->ap_num);
	return ERROR_OK;
}

static int dap_info_mem_ap_header(int retval, struct adiv5_ap *ap,
		target_addr_t dbgbase, uint32_t apid, int depth, void *priv)
{
	struct command_invocation *cmd = priv;
	target_addr_t invalid_entry;
	char tabs[17] = "";

	if (retval != ERROR_OK) {
		command_print(cmd, "\t\tCan't read MEM-AP, the corresponding core might be turned off");
		return retval;
	}

	if (depth > ROM_TABLE_MAX_DEPTH) {
		command_print(cmd, "\tTables too deep");
		return ERROR_FAIL;
	}

	if (depth)
		snprintf(tabs, sizeof(tabs), "\t[L%02d] ", depth);

	command_print(cmd, "\t\tAP ID register 0x%8.8" PRIx32, apid);
	if (apid == 0) {
		command_print(cmd, "\t\tNo AP found at this AP#0x%" PRIx64, ap->ap_num);
		return ERROR_FAIL;
	}

	command_print(cmd, "\t\tType is %s", ap_type_to_description(apid & AP_TYPE_MASK));

	/* NOTE: a MEM-AP may have a single CoreSight component that's
	 * not a ROM table ... or have no such components at all.
	 */
	const unsigned int class = (apid & AP_REG_IDR_CLASS_MASK) >> AP_REG_IDR_CLASS_SHIFT;

	if (class == AP_REG_IDR_CLASS_MEM_AP) {
		if (is_64bit_ap(ap))
			invalid_entry = 0xFFFFFFFFFFFFFFFFull;
		else
			invalid_entry = 0xFFFFFFFFul;

		command_print(cmd, "%sMEM-AP BASE " TARGET_ADDR_FMT, tabs, dbgbase);

		if (dbgbase == invalid_entry || (dbgbase & 0x3) == 0x2) {
			command_print(cmd, "\t\tNo ROM table present");
		} else {
			if (dbgbase & 0x01)
				command_print(cmd, "\t\tValid ROM table present");
			else
				command_print(cmd, "\t\tROM table in legacy format");
		}
	}

	return ERROR_OK;
}

static int dap_info_cs_component(int retval, struct cs_component_vals *v, int depth, void *priv)
{
	struct command_invocation *cmd = priv;

	if (depth > ROM_TABLE_MAX_DEPTH) {
		command_print(cmd, "\tTables too deep");
		return ERROR_FAIL;
	}

	if (v->mode == CS_ACCESS_MEM_AP)
		command_print(cmd, "\t\tComponent base address " TARGET_ADDR_FMT, v->component_base);

	if (retval != ERROR_OK) {
		command_print(cmd, "\t\tCan't read component, the corresponding core might be turned off");
		return retval;
	}

	if (!is_valid_arm_cs_cidr(v->cid)) {
		command_print(cmd, "\t\tInvalid CID 0x%08" PRIx32, v->cid);
		return ERROR_OK; /* Don't abort recursion */
	}

	/* component may take multiple 4K pages */
	uint32_t size = ARM_CS_PIDR_SIZE(v->pid);
	if (size > 0)
		command_print(cmd, "\t\tStart address " TARGET_ADDR_FMT, v->component_base - 0x1000 * size);

	command_print(cmd, "\t\tPeripheral ID 0x%010" PRIx64, v->pid);

	const unsigned int part_num = ARM_CS_PIDR_PART(v->pid);
	unsigned int designer_id = ARM_CS_PIDR_DESIGNER(v->pid);

	if (v->pid & ARM_CS_PIDR_JEDEC) {
		/* JEP106 code */
		command_print(cmd, "\t\tDesigner is 0x%03x, %s",
				designer_id, jep106_manufacturer(designer_id));
	} else {
		/* Legacy ASCII ID, clear invalid bits */
		designer_id &= 0x7f;
		command_print(cmd, "\t\tDesigner ASCII code 0x%02x, %s",
				designer_id, designer_id == 0x41 ? "ARM" : "<unknown>");
	}

	const struct dap_part_nums *partnum = pidr_to_part_num(designer_id, part_num);
	command_print(cmd, "\t\tPart is 0x%03x, %s %s", part_num, partnum->type, partnum->full);

	const unsigned int class = ARM_CS_CIDR_CLASS(v->cid);
	command_print(cmd, "\t\tComponent class is 0x%x, %s", class, class_description[class]);

	if (class == ARM_CS_CLASS_0X1_ROM_TABLE) {
		if (v->devtype_memtype & ARM_CS_C1_MEMTYPE_SYSMEM_MASK)
			command_print(cmd, "\t\tMEMTYPE system memory present on bus");
		else
			command_print(cmd, "\t\tMEMTYPE system memory not present: dedicated debug bus");
		return ERROR_OK;
	}

	if (class == ARM_CS_CLASS_0X9_CS_COMPONENT) {
		dap_devtype_display(cmd, v->devtype_memtype);

		/* REVISIT also show ARM_CS_C9_DEVID */

		if ((v->devarch & ARM_CS_C9_DEVARCH_PRESENT) == 0)
			return ERROR_OK;

		unsigned int architect_id = ARM_CS_C9_DEVARCH_ARCHITECT(v->devarch);
		unsigned int revision = ARM_CS_C9_DEVARCH_REVISION(v->devarch);
		command_print(cmd, "\t\tDev Arch is 0x%08" PRIx32 ", %s \"%s\" rev.%u", v->devarch,
				jep106_manufacturer(architect_id), class0x9_devarch_description(v->devarch),
				revision);

		if ((v->devarch & DEVARCH_ID_MASK) == DEVARCH_ROM_C_0X9) {
			command_print(cmd, "\t\tType is ROM table");

			if (v->devid & ARM_CS_C9_DEVID_SYSMEM_MASK)
				command_print(cmd, "\t\tMEMTYPE system memory present on bus");
			else
				command_print(cmd, "\t\tMEMTYPE system memory not present: dedicated debug bus");
		}
		return ERROR_OK;
	}

	/* Class other than 0x1 and 0x9 */
	return ERROR_OK;
}

static int dap_info_rom_table_entry(int retval, int depth,
		unsigned int offset, uint64_t romentry, void *priv)
{
	struct command_invocation *cmd = priv;
	char tabs[16] = "";

	if (depth)
		snprintf(tabs, sizeof(tabs), "[L%02d] ", depth);

	if (retval != ERROR_OK) {
		command_print(cmd, "\t%sROMTABLE[0x%x] Read error", tabs, offset);
		command_print(cmd, "\t\tUnable to continue");
		command_print(cmd, "\t%s\tStop parsing of ROM table", tabs);
		return retval;
	}

	command_print(cmd, "\t%sROMTABLE[0x%x] = 0x%08" PRIx64,
			tabs, offset, romentry);

	if (romentry == 0) {
		command_print(cmd, "\t%s\tEnd of ROM table", tabs);
		return ERROR_OK;
	}

	if (!(romentry & ARM_CS_ROMENTRY_PRESENT)) {
		command_print(cmd, "\t\tComponent not present");
		return ERROR_OK;
	}

	return ERROR_OK;
}

int dap_info_command(struct command_invocation *cmd, struct adiv5_ap *ap)
{
	struct rtp_ops dap_info_ops = {
		.ap_header       = dap_info_ap_header,
		.mem_ap_header   = dap_info_mem_ap_header,
		.cs_component    = dap_info_cs_component,
		.rom_table_entry = dap_info_rom_table_entry,
		.priv            = cmd,
	};

	return rtp_ap(&dap_info_ops, ap, 0);
}

/* Actions for dap_lookup_cs_component() */

struct dap_lookup_data {
	/* input */
	unsigned int idx;
	unsigned int type;
	/* output */
	uint64_t component_base;
	uint64_t ap_num;
};

static int dap_lookup_cs_component_cs_component(int retval,
		struct cs_component_vals *v, int depth, void *priv)
{
	struct dap_lookup_data *lookup = priv;

	if (retval != ERROR_OK)
		return retval;

	if (!is_valid_arm_cs_cidr(v->cid))
		return ERROR_OK;

	const unsigned int class = ARM_CS_CIDR_CLASS(v->cid);
	if (class != ARM_CS_CLASS_0X9_CS_COMPONENT)
		return ERROR_OK;

	if ((v->devtype_memtype & ARM_CS_C9_DEVTYPE_MASK) != lookup->type)
		return ERROR_OK;

	if (lookup->idx) {
		/* search for next one */
		--lookup->idx;
		return ERROR_OK;
	}

	/* Found! */
	lookup->component_base = v->component_base;
	lookup->ap_num = v->ap->ap_num;
	return CORESIGHT_COMPONENT_FOUND;
}

int dap_lookup_cs_component(struct adiv5_ap *ap, uint8_t type,
		target_addr_t *addr, int32_t core_id)
{
	struct dap_lookup_data lookup = {
		.type = type,
		.idx  = core_id,
	};
	struct rtp_ops dap_lookup_cs_component_ops = {
		.ap_header       = NULL,
		.mem_ap_header   = NULL,
		.cs_component    = dap_lookup_cs_component_cs_component,
		.rom_table_entry = NULL,
		.priv            = &lookup,
	};

	int retval = rtp_ap(&dap_lookup_cs_component_ops, ap, 0);
	if (retval == CORESIGHT_COMPONENT_FOUND) {
		if (lookup.ap_num != ap->ap_num) {
			/* TODO: handle search from root ROM table */
			LOG_DEBUG("CS lookup ended in AP # 0x%" PRIx64 ". Ignore it", lookup.ap_num);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		LOG_DEBUG("CS lookup found at 0x%" PRIx64, lookup.component_base);
		*addr = lookup.component_base;
		return ERROR_OK;
	}
	if (retval != ERROR_OK) {
		LOG_DEBUG("CS lookup error %d", retval);
		return retval;
	}
	LOG_DEBUG("CS lookup not found");
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

enum adiv5_cfg_param {
	CFG_DAP,
	CFG_AP_NUM,
	CFG_BASEADDR,
	CFG_CTIBASE, /* DEPRECATED */
};

static const struct jim_nvp nvp_config_opts[] = {
	{ .name = "-dap",       .value = CFG_DAP },
	{ .name = "-ap-num",    .value = CFG_AP_NUM },
	{ .name = "-baseaddr",  .value = CFG_BASEADDR },
	{ .name = "-ctibase",   .value = CFG_CTIBASE }, /* DEPRECATED */
	{ .name = NULL, .value = -1 }
};

static int adiv5_jim_spot_configure(struct jim_getopt_info *goi,
		struct adiv5_dap **dap_p, uint64_t *ap_num_p, uint32_t *base_p)
{
	assert(dap_p && ap_num_p);

	if (!goi->argc)
		return JIM_OK;

	Jim_SetEmptyResult(goi->interp);

	struct jim_nvp *n;
	int e = jim_nvp_name2value_obj(goi->interp, nvp_config_opts,
				goi->argv[0], &n);
	if (e != JIM_OK)
		return JIM_CONTINUE;

	/* base_p can be NULL, then '-baseaddr' option is treated as unknown */
	if (!base_p && (n->value == CFG_BASEADDR || n->value == CFG_CTIBASE))
		return JIM_CONTINUE;

	e = jim_getopt_obj(goi, NULL);
	if (e != JIM_OK)
		return e;

	switch (n->value) {
	case CFG_DAP:
		if (goi->isconfigure) {
			Jim_Obj *o_t;
			struct adiv5_dap *dap;
			e = jim_getopt_obj(goi, &o_t);
			if (e != JIM_OK)
				return e;
			dap = dap_instance_by_jim_obj(goi->interp, o_t);
			if (!dap) {
				Jim_SetResultString(goi->interp, "DAP name invalid!", -1);
				return JIM_ERR;
			}
			if (*dap_p && *dap_p != dap) {
				Jim_SetResultString(goi->interp,
					"DAP assignment cannot be changed!", -1);
				return JIM_ERR;
			}
			*dap_p = dap;
		} else {
			if (goi->argc)
				goto err_no_param;
			if (!*dap_p) {
				Jim_SetResultString(goi->interp, "DAP not configured", -1);
				return JIM_ERR;
			}
			Jim_SetResultString(goi->interp, adiv5_dap_name(*dap_p), -1);
		}
		break;

	case CFG_AP_NUM:
		if (goi->isconfigure) {
			/* jim_wide is a signed 64 bits int, ap_num is unsigned with max 52 bits */
			jim_wide ap_num;
			e = jim_getopt_wide(goi, &ap_num);
			if (e != JIM_OK)
				return e;
			/* we still don't know dap->adi_version */
			if (ap_num < 0 || (ap_num > DP_APSEL_MAX && (ap_num & 0xfff))) {
				Jim_SetResultString(goi->interp, "Invalid AP number!", -1);
				return JIM_ERR;
			}
			*ap_num_p = ap_num;
		} else {
			if (goi->argc)
				goto err_no_param;
			if (*ap_num_p == DP_APSEL_INVALID) {
				Jim_SetResultString(goi->interp, "AP number not configured", -1);
				return JIM_ERR;
			}
			Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, *ap_num_p));
		}
		break;

	case CFG_CTIBASE:
		LOG_WARNING("DEPRECATED! use \'-baseaddr' not \'-ctibase\'");
		/* fall through */
	case CFG_BASEADDR:
		if (goi->isconfigure) {
			jim_wide base;
			e = jim_getopt_wide(goi, &base);
			if (e != JIM_OK)
				return e;
			*base_p = (uint32_t)base;
		} else {
			if (goi->argc)
				goto err_no_param;
			Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, *base_p));
		}
		break;
	};

	return JIM_OK;

err_no_param:
	Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "NO PARAMS");
	return JIM_ERR;
}

int adiv5_jim_configure(struct target *target, struct jim_getopt_info *goi)
{
	struct adiv5_private_config *pc;
	int e;

	pc = (struct adiv5_private_config *)target->private_config;
	if (!pc) {
		pc = calloc(1, sizeof(struct adiv5_private_config));
		if (!pc) {
			LOG_ERROR("Out of memory");
			return JIM_ERR;
		}
		pc->ap_num = DP_APSEL_INVALID;
		target->private_config = pc;
	}

	target->has_dap = true;

	e = adiv5_jim_spot_configure(goi, &pc->dap, &pc->ap_num, NULL);
	if (e != JIM_OK)
		return e;

	if (pc->dap && !target->dap_configured) {
		if (target->tap_configured) {
			pc->dap = NULL;
			Jim_SetResultString(goi->interp,
				"-chain-position and -dap configparams are mutually exclusive!", -1);
			return JIM_ERR;
		}
		target->tap = pc->dap->tap;
		target->dap_configured = true;
	}

	return JIM_OK;
}

int adiv5_verify_config(struct adiv5_private_config *pc)
{
	if (!pc)
		return ERROR_FAIL;

	if (!pc->dap)
		return ERROR_FAIL;

	return ERROR_OK;
}

int adiv5_jim_mem_ap_spot_configure(struct adiv5_mem_ap_spot *cfg,
		struct jim_getopt_info *goi)
{
	return adiv5_jim_spot_configure(goi, &cfg->dap, &cfg->ap_num, &cfg->base);
}

int adiv5_mem_ap_spot_init(struct adiv5_mem_ap_spot *p)
{
	p->dap = NULL;
	p->ap_num = DP_APSEL_INVALID;
	p->base = 0;
	return ERROR_OK;
}

COMMAND_HANDLER(handle_dap_info_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint64_t apsel;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		if (!strcmp(CMD_ARGV[0], "root")) {
			if (!is_adiv6(dap)) {
				command_print(CMD, "Option \"root\" not allowed with ADIv5 DAP");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
			int retval = adiv6_dap_read_baseptr(CMD, dap, &apsel);
			if (retval != ERROR_OK) {
				command_print(CMD, "Failed reading DAP baseptr");
				return retval;
			}
			break;
		}
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], apsel);
		if (!is_ap_num_valid(dap, apsel)) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct adiv5_ap *ap = dap_get_ap(dap, apsel);
	if (!ap) {
		command_print(CMD, "Cannot get AP");
		return ERROR_FAIL;
	}

	int retval = dap_info_command(CMD, ap);
	dap_put_ap(ap);
	return retval;
}

COMMAND_HANDLER(dap_baseaddr_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint64_t apsel;
	uint32_t baseaddr_lower, baseaddr_upper;
	struct adiv5_ap *ap;
	target_addr_t baseaddr;
	int retval;

	baseaddr_upper = 0;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], apsel);
		if (!is_ap_num_valid(dap, apsel)) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* NOTE:  assumes we're talking to a MEM-AP, which
	 * has a base address.  There are other kinds of AP,
	 * though they're not common for now.  This should
	 * use the ID register to verify it's a MEM-AP.
	 */

	ap = dap_get_ap(dap, apsel);
	if (!ap) {
		command_print(CMD, "Cannot get AP");
		return ERROR_FAIL;
	}

	retval = dap_queue_ap_read(ap, MEM_AP_REG_BASE(dap), &baseaddr_lower);

	if (retval == ERROR_OK && ap->cfg_reg == MEM_AP_REG_CFG_INVALID)
		retval = dap_queue_ap_read(ap, MEM_AP_REG_CFG(dap), &ap->cfg_reg);

	if (retval == ERROR_OK && (ap->cfg_reg == MEM_AP_REG_CFG_INVALID || is_64bit_ap(ap))) {
		/* MEM_AP_REG_BASE64 is defined as 'RES0'; can be read and then ignored on 32 bits AP */
		retval = dap_queue_ap_read(ap, MEM_AP_REG_BASE64(dap), &baseaddr_upper);
	}

	if (retval == ERROR_OK)
		retval = dap_run(dap);
	dap_put_ap(ap);
	if (retval != ERROR_OK)
		return retval;

	if (is_64bit_ap(ap)) {
		baseaddr = (((target_addr_t)baseaddr_upper) << 32) | baseaddr_lower;
		command_print(CMD, "0x%016" PRIx64, baseaddr);
	} else
		command_print(CMD, "0x%08" PRIx32, baseaddr_lower);

	return ERROR_OK;
}

COMMAND_HANDLER(dap_memaccess_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	struct adiv5_ap *ap;
	uint32_t memaccess_tck;

	switch (CMD_ARGC) {
	case 0:
		ap = dap_get_ap(dap, dap->apsel);
		if (!ap) {
			command_print(CMD, "Cannot get AP");
			return ERROR_FAIL;
		}
		memaccess_tck = ap->memaccess_tck;
		break;
	case 1:
		ap = dap_get_config_ap(dap, dap->apsel);
		if (!ap) {
			command_print(CMD, "Cannot get AP");
			return ERROR_FAIL;
		}
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], memaccess_tck);
		ap->memaccess_tck = memaccess_tck;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap_put_ap(ap);

	command_print(CMD, "memory bus access delay set to %" PRIu32 " tck",
			memaccess_tck);

	return ERROR_OK;
}

COMMAND_HANDLER(dap_apsel_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint64_t apsel;

	switch (CMD_ARGC) {
	case 0:
		command_print(CMD, "0x%" PRIx64, dap->apsel);
		return ERROR_OK;
	case 1:
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], apsel);
		if (!is_ap_num_valid(dap, apsel)) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap->apsel = apsel;
	return ERROR_OK;
}

COMMAND_HANDLER(dap_apcsw_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	struct adiv5_ap *ap;
	uint32_t csw_val, csw_mask;

	switch (CMD_ARGC) {
	case 0:
		ap = dap_get_ap(dap, dap->apsel);
		if (!ap) {
			command_print(CMD, "Cannot get AP");
			return ERROR_FAIL;
		}
		command_print(CMD, "AP#0x%" PRIx64 " selected, csw 0x%8.8" PRIx32,
			dap->apsel, ap->csw_default);
		break;
	case 1:
		if (strcmp(CMD_ARGV[0], "default") == 0)
			csw_val = CSW_AHB_DEFAULT;
		else
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], csw_val);

		if (csw_val & (CSW_SIZE_MASK | CSW_ADDRINC_MASK)) {
			LOG_ERROR("CSW value cannot include 'Size' and 'AddrInc' bit-fields");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		ap = dap_get_config_ap(dap, dap->apsel);
		if (!ap) {
			command_print(CMD, "Cannot get AP");
			return ERROR_FAIL;
		}
		ap->csw_default = csw_val;
		break;
	case 2:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], csw_val);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], csw_mask);
		if (csw_mask & (CSW_SIZE_MASK | CSW_ADDRINC_MASK)) {
			LOG_ERROR("CSW mask cannot include 'Size' and 'AddrInc' bit-fields");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		ap = dap_get_config_ap(dap, dap->apsel);
		if (!ap) {
			command_print(CMD, "Cannot get AP");
			return ERROR_FAIL;
		}
		ap->csw_default = (ap->csw_default & ~csw_mask) | (csw_val & csw_mask);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	dap_put_ap(ap);

	return ERROR_OK;
}



COMMAND_HANDLER(dap_apid_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint64_t apsel;
	uint32_t apid;
	int retval;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], apsel);
		if (!is_ap_num_valid(dap, apsel)) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct adiv5_ap *ap = dap_get_ap(dap, apsel);
	if (!ap) {
		command_print(CMD, "Cannot get AP");
		return ERROR_FAIL;
	}
	retval = dap_queue_ap_read(ap, AP_REG_IDR(dap), &apid);
	if (retval != ERROR_OK) {
		dap_put_ap(ap);
		return retval;
	}
	retval = dap_run(dap);
	dap_put_ap(ap);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%8.8" PRIx32, apid);

	return retval;
}

COMMAND_HANDLER(dap_apreg_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint64_t apsel;
	uint32_t reg, value;
	int retval;

	if (CMD_ARGC < 2 || CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], apsel);
	if (!is_ap_num_valid(dap, apsel)) {
		command_print(CMD, "Invalid AP number");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg);
	if (is_adiv6(dap)) {
		if (reg >= 4096 || (reg & 3)) {
			command_print(CMD, "Invalid reg value (should be less than 4096 and 4 bytes aligned)");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	} else {	/* ADI version 5 */
		if (reg >= 256 || (reg & 3)) {
			command_print(CMD, "Invalid reg value (should be less than 256 and 4 bytes aligned)");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	struct adiv5_ap *ap = dap_get_ap(dap, apsel);
	if (!ap) {
		command_print(CMD, "Cannot get AP");
		return ERROR_FAIL;
	}

	if (CMD_ARGC == 3) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], value);
		/* see if user supplied register address is a match for the CSW or TAR register */
		if (reg == MEM_AP_REG_CSW(dap)) {
			ap->csw_value = 0;  /* invalid, in case write fails */
			retval = dap_queue_ap_write(ap, reg, value);
			if (retval == ERROR_OK)
				ap->csw_value = value;
		} else if (reg == MEM_AP_REG_TAR(dap)) {
			retval = dap_queue_ap_write(ap, reg, value);
			if (retval == ERROR_OK)
				ap->tar_value = (ap->tar_value & ~0xFFFFFFFFull) | value;
			else {
				/* To track independent writes to TAR and TAR64, two tar_valid flags */
				/* should be used. To keep it simple, tar_valid is only invalidated on a */
				/* write fail. This approach causes a later re-write of the TAR and TAR64 */
				/* if tar_valid is false. */
				ap->tar_valid = false;
			}
		} else if (reg == MEM_AP_REG_TAR64(dap)) {
			retval = dap_queue_ap_write(ap, reg, value);
			if (retval == ERROR_OK)
				ap->tar_value = (ap->tar_value & 0xFFFFFFFFull) | (((target_addr_t)value) << 32);
			else {
				/* See above comment for the MEM_AP_REG_TAR failed write case */
				ap->tar_valid = false;
			}
		} else {
			retval = dap_queue_ap_write(ap, reg, value);
		}
	} else {
		retval = dap_queue_ap_read(ap, reg, &value);
	}
	if (retval == ERROR_OK)
		retval = dap_run(dap);

	dap_put_ap(ap);

	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC == 2)
		command_print(CMD, "0x%08" PRIx32, value);

	return retval;
}

COMMAND_HANDLER(dap_dpreg_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t reg, value;
	int retval;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);
	if (reg >= 256 || (reg & 3)) {
		command_print(CMD, "Invalid reg value (should be less than 256 and 4 bytes aligned)");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
		retval = dap_queue_dp_write(dap, reg, value);
	} else {
		retval = dap_queue_dp_read(dap, reg, &value);
	}
	if (retval == ERROR_OK)
		retval = dap_run(dap);

	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC == 1)
		command_print(CMD, "0x%08" PRIx32, value);

	return retval;
}

COMMAND_HANDLER(dap_ti_be_32_quirks_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	return CALL_COMMAND_HANDLER(handle_command_parse_bool, &dap->ti_be_32_quirks,
		"TI BE-32 quirks mode");
}

COMMAND_HANDLER(dap_nu_npcx_quirks_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	return CALL_COMMAND_HANDLER(handle_command_parse_bool, &dap->nu_npcx_quirks,
								"Nuvoton NPCX quirks mode");
}

const struct command_registration dap_instance_commands[] = {
	{
		.name = "info",
		.handler = handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "display ROM table for specified MEM-AP (default currently selected AP) "
			"or the ADIv6 root ROM table",
		.usage = "[ap_num | 'root']",
	},
	{
		.name = "apsel",
		.handler = dap_apsel_command,
		.mode = COMMAND_ANY,
		.help = "Set the currently selected AP (default 0) "
			"and display the result",
		.usage = "[ap_num]",
	},
	{
		.name = "apcsw",
		.handler = dap_apcsw_command,
		.mode = COMMAND_ANY,
		.help = "Set CSW default bits",
		.usage = "[value [mask]]",
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
		.name = "apreg",
		.handler = dap_apreg_command,
		.mode = COMMAND_EXEC,
		.help = "read/write a register from AP "
			"(reg is byte address of a word register, like 0 4 8...)",
		.usage = "ap_num reg [value]",
	},
	{
		.name = "dpreg",
		.handler = dap_dpreg_command,
		.mode = COMMAND_EXEC,
		.help = "read/write a register from DP "
			"(reg is byte address (bank << 4 | reg) of a word register, like 0 4 8...)",
		.usage = "reg [value]",
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
	{
		.name = "ti_be_32_quirks",
		.handler = dap_ti_be_32_quirks_command,
		.mode = COMMAND_CONFIG,
		.help = "set/get quirks mode for TI TMS450/TMS570 processors",
		.usage = "[enable]",
	},
	{
		.name = "nu_npcx_quirks",
		.handler = dap_nu_npcx_quirks_command,
		.mode = COMMAND_CONFIG,
		.help = "set/get quirks mode for Nuvoton NPCX controllers",
		.usage = "[enable]",
	},
	COMMAND_REGISTRATION_DONE
};

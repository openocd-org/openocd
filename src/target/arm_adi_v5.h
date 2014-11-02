/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#ifndef ARM_ADI_V5_H
#define ARM_ADI_V5_H

/**
 * @file
 * This defines formats and data structures used to talk to ADIv5 entities.
 * Those include a DAP, different types of Debug Port (DP), and memory mapped
 * resources accessed through a MEM-AP.
 */

#include "arm_jtag.h"

/* FIXME remove these JTAG-specific decls when mem_ap_read_buf_u32()
 * is no longer JTAG-specific
 */
#define JTAG_DP_DPACC		0xA
#define JTAG_DP_APACC		0xB

/* three-bit ACK values for SWD access (sent LSB first) */
#define SWD_ACK_OK    0x1
#define SWD_ACK_WAIT  0x2
#define SWD_ACK_FAULT 0x4

#define DPAP_WRITE		0
#define DPAP_READ		1

#define BANK_REG(bank, reg)	(((bank) << 4) | (reg))

/* A[3:0] for DP registers; A[1:0] are always zero.
 * - JTAG accesses all of these via JTAG_DP_DPACC, except for
 *   IDCODE (JTAG_DP_IDCODE) and ABORT (JTAG_DP_ABORT).
 * - SWD accesses these directly, sometimes needing SELECT.CTRLSEL
 */
#define DP_IDCODE		BANK_REG(0x0, 0x0)	/* SWD: read */
#define DP_ABORT		BANK_REG(0x0, 0x0)	/* SWD: write */
#define DP_CTRL_STAT		BANK_REG(0x0, 0x4)	/* r/w */
#define DP_RESEND		BANK_REG(0x0, 0x8)	/* SWD: read */
#define DP_SELECT		BANK_REG(0x0, 0x8)	/* JTAG: r/w; SWD: write */
#define DP_RDBUFF		BANK_REG(0x0, 0xC)	/* read-only */
#define DP_WCR			BANK_REG(0x1, 0x4)	/* SWD: r/w */

#define WCR_TO_TRN(wcr) ((uint32_t)(1 + (3 & ((wcr)) >> 8)))	/* 1..4 clocks */
#define WCR_TO_PRESCALE(wcr) ((uint32_t)(7 & ((wcr))))		/* impl defined */

/* Fields of the DP's AP ABORT register */
#define DAPABORT        (1UL << 0)
#define STKCMPCLR       (1UL << 1) /* SWD-only */
#define STKERRCLR       (1UL << 2) /* SWD-only */
#define WDERRCLR        (1UL << 3) /* SWD-only */
#define ORUNERRCLR      (1UL << 4) /* SWD-only */

/* Fields of the DP's CTRL/STAT register */
#define CORUNDETECT     (1UL << 0)
#define SSTICKYORUN     (1UL << 1)
/* 3:2 - transaction mode (e.g. pushed compare) */
#define SSTICKYCMP      (1UL << 4)
#define SSTICKYERR      (1UL << 5)
#define READOK          (1UL << 6) /* SWD-only */
#define WDATAERR        (1UL << 7) /* SWD-only */
/* 11:8 - mask lanes for pushed compare or verify ops */
/* 21:12 - transaction counter */
#define CDBGRSTREQ      (1UL << 26)
#define CDBGRSTACK      (1UL << 27)
#define CDBGPWRUPREQ    (1UL << 28)
#define CDBGPWRUPACK    (1UL << 29)
#define CSYSPWRUPREQ    (1UL << 30)
#define CSYSPWRUPACK    (1UL << 31)

/* MEM-AP register addresses */
/* TODO: rename as MEM_AP_REG_* */
#define AP_REG_CSW		0x00
#define AP_REG_TAR		0x04
#define AP_REG_DRW		0x0C
#define AP_REG_BD0		0x10
#define AP_REG_BD1		0x14
#define AP_REG_BD2		0x18
#define AP_REG_BD3		0x1C
#define AP_REG_CFG		0xF4		/* big endian? */
#define AP_REG_BASE		0xF8

/* Generic AP register address */
#define AP_REG_IDR		0xFC

/* Fields of the MEM-AP's CSW register */
#define CSW_8BIT		0
#define CSW_16BIT		1
#define CSW_32BIT		2
#define CSW_ADDRINC_MASK    (3UL << 4)
#define CSW_ADDRINC_OFF     0UL
#define CSW_ADDRINC_SINGLE  (1UL << 4)
#define CSW_ADDRINC_PACKED  (2UL << 4)
#define CSW_DEVICE_EN       (1UL << 6)
#define CSW_TRIN_PROG       (1UL << 7)
#define CSW_SPIDEN          (1UL << 23)
/* 30:24 - implementation-defined! */
#define CSW_HPROT           (1UL << 25) /* ? */
#define CSW_MASTER_DEBUG    (1UL << 29) /* ? */
#define CSW_SPROT           (1UL << 30)
#define CSW_DBGSWENABLE     (1UL << 31)

/**
 * This represents an ARM Debug Interface (v5) Debug Access Port (DAP).
 * A DAP has two types of component:  one Debug Port (DP), which is a
 * transport agent; and at least one Access Port (AP), controlling
 * resource access.  Most common is a MEM-AP, for memory access.
 *
 * There are two basic DP transports: JTAG, and ARM's low pin-count SWD.
 * Accordingly, this interface is responsible for hiding the transport
 * differences so upper layer code can largely ignore them.
 *
 * When the chip is implemented with JTAG-DP or SW-DP, the transport is
 * fixed as JTAG or SWD, respectively.  Chips incorporating SWJ-DP permit
 * a choice made at board design time (by only using the SWD pins), or
 * as part of setting up a debug session (if all the dual-role JTAG/SWD
 * signals are available).
 */
struct adiv5_dap {
	const struct dap_ops *ops;

	struct arm_jtag *jtag_info;
	/* Control config */
	uint32_t dp_ctrl_stat;

	uint32_t apcsw[256];
	uint32_t apsel;

	/**
	 * Cache for DP_SELECT bits identifying the current AP.  A DAP may
	 * connect to multiple APs, such as one MEM-AP for general access,
	 * another reserved for accessing debug modules, and a JTAG-DP.
	 * "-1" indicates no cached value.
	 */
	uint32_t ap_current;

	/**
	 * Cache for DP_SELECT bits identifying the current four-word AP
	 * register bank.  This caches AP register addresss bits 7:4; JTAG
	 * and SWD access primitves pass address bits 3:2; bits 1:0 are zero.
	 * "-1" indicates no cached value.
	 */
	uint32_t ap_bank_value;

	/**
	 * Cache for DP_SELECT bits identifying the current four-word DP
	 * register bank.  This caches DP register addresss bits 7:4; JTAG
	 * and SWD access primitves pass address bits 3:2; bits 1:0 are zero.
	 */
	uint32_t dp_bank_value;

	/**
	 * Cache for (MEM-AP) AP_REG_CSW register value.  This is written to
	 * configure an access mode, such as autoincrementing AP_REG_TAR during
	 * word access.  "-1" indicates no cached value.
	 */
	uint32_t ap_csw_value;

	/**
	 * Cache for (MEM-AP) AP_REG_TAR register value This is written to
	 * configure the address being read or written
	 * "-1" indicates no cached value.
	 */
	uint32_t ap_tar_value;

	/* information about current pending SWjDP-AHBAP transaction */
	uint8_t  ack;

	/**
	 * Holds the pointer to the destination word for the last queued read,
	 * for use with posted AP read sequence optimization.
	 */
	uint32_t *last_read;

	/**
	 * Configures how many extra tck clocks are added after starting a
	 * MEM-AP access before we try to read its status (and/or result).
	 */
	uint32_t	memaccess_tck;

	/* Size of TAR autoincrement block, ARM ADI Specification requires at least 10 bits */
	uint32_t tar_autoincr_block;

	/* true if packed transfers are supported by the MEM-AP */
	bool packed_transfers;

	/* true if unaligned memory access is not supported by the MEM-AP */
	bool unaligned_access_bad;

	/* The TI TMS470 and TMS570 series processors use a BE-32 memory ordering
	 * despite lack of support in the ARMv7 architecture. Memory access through
	 * the AHB-AP has strange byte ordering these processors, and we need to
	 * swizzle appropriately. */
	bool ti_be_32_quirks;

	/**
	 * Signals that an attempt to reestablish communication afresh
	 * should be performed before the next access.
	 */
	bool do_reconnect;
};

/**
 * Transport-neutral representation of queued DAP transactions, supporting
 * both JTAG and SWD transports.  All submitted transactions are logically
 * queued, until the queue is executed by run().  Some implementations might
 * execute transactions as soon as they're submitted, but no status is made
 * available until run().
 */
struct dap_ops {
	/** If the DAP transport isn't SWD, it must be JTAG.  Upper level
	 * code may need to care about the difference in some cases.
	 */
	bool	is_swd;

	/** DP register read. */
	int (*queue_dp_read)(struct adiv5_dap *dap, unsigned reg,
			uint32_t *data);
	/** DP register write. */
	int (*queue_dp_write)(struct adiv5_dap *dap, unsigned reg,
			uint32_t data);

	/** AP register read. */
	int (*queue_ap_read)(struct adiv5_dap *dap, unsigned reg,
			uint32_t *data);
	/** AP register write. */
	int (*queue_ap_write)(struct adiv5_dap *dap, unsigned reg,
			uint32_t data);

	/** AP operation abort. */
	int (*queue_ap_abort)(struct adiv5_dap *dap, uint8_t *ack);

	/** Executes all queued DAP operations. */
	int (*run)(struct adiv5_dap *dap);
};

/*
 * Access Port types
 */
enum ap_type {
	AP_TYPE_AHB_AP  = 0x01,  /* AHB Memory-AP */
	AP_TYPE_APB_AP  = 0x02,  /* APB Memory-AP */
	AP_TYPE_JTAG_AP = 0x10   /* JTAG-AP - JTAG master for controlling other JTAG devices */
};

/**
 * Queue a DP register read.
 * Note that not all DP registers are readable; also, that JTAG and SWD
 * have slight differences in DP register support.
 *
 * @param dap The DAP used for reading.
 * @param reg The two-bit number of the DP register being read.
 * @param data Pointer saying where to store the register's value
 * (in host endianness).
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_dp_read(struct adiv5_dap *dap,
		unsigned reg, uint32_t *data)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_dp_read(dap, reg, data);
}

/**
 * Queue a DP register write.
 * Note that not all DP registers are writable; also, that JTAG and SWD
 * have slight differences in DP register support.
 *
 * @param dap The DAP used for writing.
 * @param reg The two-bit number of the DP register being written.
 * @param data Value being written (host endianness)
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_dp_write(struct adiv5_dap *dap,
		unsigned reg, uint32_t data)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_dp_write(dap, reg, data);
}

/**
 * Queue an AP register read.
 *
 * @param dap The DAP used for reading.
 * @param reg The number of the AP register being read.
 * @param data Pointer saying where to store the register's value
 * (in host endianness).
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_read(struct adiv5_dap *dap,
		unsigned reg, uint32_t *data)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_ap_read(dap, reg, data);
}

/**
 * Queue an AP register write.
 *
 * @param dap The DAP used for writing.
 * @param reg The number of the AP register being written.
 * @param data Value being written (host endianness)
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_write(struct adiv5_dap *dap,
		unsigned reg, uint32_t data)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_ap_write(dap, reg, data);
}

/**
 * Queue an AP abort operation.  The current AP transaction is aborted,
 * including any update of the transaction counter.  The AP is left in
 * an unknown state (so it must be re-initialized).  For use only after
 * the AP has reported WAIT status for an extended period.
 *
 * @param dap The DAP used for writing.
 * @param ack Pointer to where transaction status will be stored.
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_ap_abort(dap, ack);
}

/**
 * Perform all queued DAP operations, and clear any errors posted in the
 * CTRL_STAT register when they are done.  Note that if more than one AP
 * operation will be queued, one of the first operations in the queue
 * should probably enable CORUNDETECT in the CTRL/STAT register.
 *
 * @param dap The DAP used.
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_run(struct adiv5_dap *dap)
{
	assert(dap->ops != NULL);
	return dap->ops->run(dap);
}

static inline int dap_dp_read_atomic(struct adiv5_dap *dap, unsigned reg,
				     uint32_t *value)
{
	int retval;

	retval = dap_queue_dp_read(dap, reg, value);
	if (retval != ERROR_OK)
		return retval;

	return dap_run(dap);
}

static inline int dap_dp_poll_register(struct adiv5_dap *dap, unsigned reg,
				       uint32_t mask, uint32_t value, int timeout)
{
	assert(timeout > 0);
	assert((value & mask) == value);

	int ret;
	uint32_t regval;
	LOG_DEBUG("DAP: poll %x, mask 0x08%" PRIx32 ", value 0x%08" PRIx32,
		  reg, mask, value);
	do {
		ret = dap_dp_read_atomic(dap, reg, &regval);
		if (ret != ERROR_OK)
			return ret;

		if ((regval & mask) == value)
			break;

		alive_sleep(10);
	} while (--timeout);

	if (!timeout) {
		LOG_DEBUG("DAP: poll %x timeout", reg);
		return ERROR_FAIL;
	} else {
		return ERROR_OK;
	}
}

/** Accessor for currently selected DAP-AP number (0..255) */
static inline uint8_t dap_ap_get_select(struct adiv5_dap *swjdp)
{
	return (uint8_t)(swjdp->ap_current >> 24);
}

/* AP selection applies to future AP transactions */
void dap_ap_select(struct adiv5_dap *dap, uint8_t ap);

/* Queued AP transactions */
int dap_setup_accessport(struct adiv5_dap *swjdp,
		uint32_t csw, uint32_t tar);

/* Queued MEM-AP memory mapped single word transfers */
int mem_ap_read_u32(struct adiv5_dap *swjdp, uint32_t address, uint32_t *value);
int mem_ap_write_u32(struct adiv5_dap *swjdp, uint32_t address, uint32_t value);

/* Synchronous MEM-AP memory mapped single word transfers */
int mem_ap_read_atomic_u32(struct adiv5_dap *swjdp,
		uint32_t address, uint32_t *value);
int mem_ap_write_atomic_u32(struct adiv5_dap *swjdp,
		uint32_t address, uint32_t value);

/* Queued MEM-AP memory mapped single word transfers with selection of ap */
int mem_ap_sel_read_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t *value);
int mem_ap_sel_write_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t value);

/* Synchronous MEM-AP memory mapped single word transfers with selection of ap */
int mem_ap_sel_read_atomic_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t *value);
int mem_ap_sel_write_atomic_u32(struct adiv5_dap *swjdp, uint8_t ap,
		uint32_t address, uint32_t value);

/* Synchronous MEM-AP memory mapped bus block transfers */
int mem_ap_read(struct adiv5_dap *dap, uint8_t *buffer, uint32_t size,
		uint32_t count, uint32_t address, bool addrinc);
int mem_ap_write(struct adiv5_dap *dap, const uint8_t *buffer, uint32_t size,
		uint32_t count, uint32_t address, bool addrinc);

/* Synchronous MEM-AP memory mapped bus block transfers with selection of ap */
int mem_ap_sel_read_buf(struct adiv5_dap *swjdp, uint8_t ap,
		uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);
int mem_ap_sel_write_buf(struct adiv5_dap *swjdp, uint8_t ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);

/* Synchronous, non-incrementing buffer functions for accessing fifos, with
 * selection of ap */
int mem_ap_sel_read_buf_noincr(struct adiv5_dap *swjdp, uint8_t ap,
		uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);
int mem_ap_sel_write_buf_noincr(struct adiv5_dap *swjdp, uint8_t ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);

/* Initialisation of the debug system, power domains and registers */
int ahbap_debugport_init(struct adiv5_dap *swjdp);

/* Probe the AP for ROM Table location */
int dap_get_debugbase(struct adiv5_dap *dap, int ap,
			uint32_t *dbgbase, uint32_t *apid);

/* Probe Access Ports to find a particular type */
int dap_find_ap(struct adiv5_dap *dap,
			enum ap_type type_to_find,
			uint8_t *ap_num_out);

/* Lookup CoreSight component */
int dap_lookup_cs_component(struct adiv5_dap *dap, int ap,
			uint32_t dbgbase, uint8_t type, uint32_t *addr, int32_t *idx);

struct target;

/* Put debug link into SWD mode */
int dap_to_swd(struct target *target);

/* Put debug link into JTAG mode */
int dap_to_jtag(struct target *target);

extern const struct command_registration dap_command_handlers[];

#endif

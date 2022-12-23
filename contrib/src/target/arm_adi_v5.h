/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2019-2021, Ampere Computing LLC                         *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM_ADI_V5_H
#define OPENOCD_TARGET_ARM_ADI_V5_H

/**
 * @file
 * This defines formats and data structures used to talk to ADIv5 entities.
 * Those include a DAP, different types of Debug Port (DP), and memory mapped
 * resources accessed through a MEM-AP.
 */

#include <helper/list.h>
#include "arm_jtag.h"
#include "helper/bits.h"

/* JEP106 ID for ARM */
#define ARM_ID 0x23B

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
 * - SWD accesses these directly, sometimes needing SELECT.DPBANKSEL
 */
#define DP_DPIDR        BANK_REG(0x0, 0x0) /* DPv1+: ro */
#define DP_ABORT        BANK_REG(0x0, 0x0) /* DPv1+: SWD: wo */
#define DP_DPIDR1       BANK_REG(0x1, 0x0) /* DPv3: ro */
#define DP_BASEPTR0     BANK_REG(0x2, 0x0) /* DPv3: ro */
#define DP_BASEPTR1     BANK_REG(0x3, 0x0) /* DPv3: ro */
#define DP_CTRL_STAT    BANK_REG(0x0, 0x4) /* DPv0+: rw */
#define DP_DLCR         BANK_REG(0x1, 0x4) /* DPv1+: SWD: rw */
#define DP_TARGETID     BANK_REG(0x2, 0x4) /* DPv2: ro */
#define DP_DLPIDR       BANK_REG(0x3, 0x4) /* DPv2: ro */
#define DP_EVENTSTAT    BANK_REG(0x4, 0x4) /* DPv2: ro */
#define DP_SELECT1      BANK_REG(0x5, 0x4) /* DPv3: ro */
#define DP_RESEND       BANK_REG(0x0, 0x8) /* DPv1+: SWD: ro */
#define DP_SELECT       BANK_REG(0x0, 0x8) /* DPv0+: JTAG: rw; SWD: wo */
#define DP_RDBUFF       BANK_REG(0x0, 0xC) /* DPv0+: ro */
#define DP_TARGETSEL    BANK_REG(0x0, 0xC) /* DPv2: SWD: wo */

#define DLCR_TO_TRN(dlcr) ((uint32_t)(1 + ((3 & (dlcr)) >> 8))) /* 1..4 clocks */

/* Fields of DP_DPIDR register */
#define DP_DPIDR_VERSION_SHIFT	12
#define DP_DPIDR_VERSION_MASK	(0xFUL << DP_DPIDR_VERSION_SHIFT)

/* Fields of the DP's AP ABORT register */
#define DAPABORT        (1UL << 0)
#define STKCMPCLR       (1UL << 1) /* SWD-only */
#define STKERRCLR       (1UL << 2) /* SWD-only */
#define WDERRCLR        (1UL << 3) /* SWD-only */
#define ORUNERRCLR      (1UL << 4) /* SWD-only */

/* Fields of register DP_DPIDR1 */
#define DP_DPIDR1_ASIZE_MASK    (0x7F)
#define DP_DPIDR1_ERRMODE       BIT(7)

/* Fields of register DP_BASEPTR0 */
#define DP_BASEPTR0_VALID       BIT(0)

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

#define DP_DLPIDR_PROTVSN	1u

#define DP_SELECT_APSEL 0xFF000000
#define DP_SELECT_APBANK 0x000000F0
#define DP_SELECT_DPBANK 0x0000000F
#define DP_SELECT_INVALID 0x00FFFF00 /* Reserved bits one */

#define DP_APSEL_MAX        (255) /* for ADIv5 only */
#define DP_APSEL_INVALID    0xF00 /* more than DP_APSEL_MAX and not ADIv6 aligned 4k */

#define DP_TARGETSEL_INVALID 0xFFFFFFFFU
#define DP_TARGETSEL_DPID_MASK 0x0FFFFFFFU
#define DP_TARGETSEL_INSTANCEID_MASK 0xF0000000U
#define DP_TARGETSEL_INSTANCEID_SHIFT 28


/* MEM-AP register addresses */
#define ADIV5_MEM_AP_REG_CSW    (0x00)
#define ADIV5_MEM_AP_REG_TAR    (0x04)
#define ADIV5_MEM_AP_REG_TAR64  (0x08)		/* RW: Large Physical Address Extension */
#define ADIV5_MEM_AP_REG_DRW    (0x0C)		/* RW: Data Read/Write register */
#define ADIV5_MEM_AP_REG_BD0    (0x10)		/* RW: Banked Data register 0-3 */
#define ADIV5_MEM_AP_REG_BD1    (0x14)
#define ADIV5_MEM_AP_REG_BD2    (0x18)
#define ADIV5_MEM_AP_REG_BD3    (0x1C)
#define ADIV5_MEM_AP_REG_MBT    (0x20)		/* --: Memory Barrier Transfer register */
#define ADIV5_MEM_AP_REG_BASE64 (0xF0)		/* RO: Debug Base Address (LA) register */
#define ADIV5_MEM_AP_REG_CFG    (0xF4)		/* RO: Configuration register */
#define ADIV5_MEM_AP_REG_BASE   (0xF8)		/* RO: Debug Base Address register */

#define ADIV6_MEM_AP_REG_CSW    (0xD00 + ADIV5_MEM_AP_REG_CSW)
#define ADIV6_MEM_AP_REG_TAR    (0xD00 + ADIV5_MEM_AP_REG_TAR)
#define ADIV6_MEM_AP_REG_TAR64  (0xD00 + ADIV5_MEM_AP_REG_TAR64)
#define ADIV6_MEM_AP_REG_DRW    (0xD00 + ADIV5_MEM_AP_REG_DRW)
#define ADIV6_MEM_AP_REG_BD0    (0xD00 + ADIV5_MEM_AP_REG_BD0)
#define ADIV6_MEM_AP_REG_BD1    (0xD00 + ADIV5_MEM_AP_REG_BD1)
#define ADIV6_MEM_AP_REG_BD2    (0xD00 + ADIV5_MEM_AP_REG_BD2)
#define ADIV6_MEM_AP_REG_BD3    (0xD00 + ADIV5_MEM_AP_REG_BD3)
#define ADIV6_MEM_AP_REG_MBT    (0xD00 + ADIV5_MEM_AP_REG_MBT)
#define ADIV6_MEM_AP_REG_BASE64 (0xD00 + ADIV5_MEM_AP_REG_BASE64)
#define ADIV6_MEM_AP_REG_CFG    (0xD00 + ADIV5_MEM_AP_REG_CFG)
#define ADIV6_MEM_AP_REG_BASE   (0xD00 + ADIV5_MEM_AP_REG_BASE)

#define MEM_AP_REG_CSW(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_CSW    : ADIV5_MEM_AP_REG_CSW)
#define MEM_AP_REG_TAR(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_TAR    : ADIV5_MEM_AP_REG_TAR)
#define MEM_AP_REG_TAR64(dap)   (is_adiv6(dap) ? ADIV6_MEM_AP_REG_TAR64  : ADIV5_MEM_AP_REG_TAR64)
#define MEM_AP_REG_DRW(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_DRW    : ADIV5_MEM_AP_REG_DRW)
#define MEM_AP_REG_BD0(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_BD0    : ADIV5_MEM_AP_REG_BD0)
#define MEM_AP_REG_BD1(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_BD1    : ADIV5_MEM_AP_REG_BD1)
#define MEM_AP_REG_BD2(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_BD2    : ADIV5_MEM_AP_REG_BD2)
#define MEM_AP_REG_BD3(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_BD3    : ADIV5_MEM_AP_REG_BD3)
#define MEM_AP_REG_MBT(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_MBT    : ADIV5_MEM_AP_REG_MBT)
#define MEM_AP_REG_BASE64(dap)  (is_adiv6(dap) ? ADIV6_MEM_AP_REG_BASE64 : ADIV5_MEM_AP_REG_BASE64)
#define MEM_AP_REG_CFG(dap)     (is_adiv6(dap) ? ADIV6_MEM_AP_REG_CFG    : ADIV5_MEM_AP_REG_CFG)
#define MEM_AP_REG_BASE(dap)    (is_adiv6(dap) ? ADIV6_MEM_AP_REG_BASE   : ADIV5_MEM_AP_REG_BASE)

/* Generic AP register address */
#define ADIV5_AP_REG_IDR        (0xFC)		/* RO: Identification Register */
#define ADIV6_AP_REG_IDR        (0xD00 + ADIV5_AP_REG_IDR)
#define AP_REG_IDR(dap)         (is_adiv6(dap) ? ADIV6_AP_REG_IDR : ADIV5_AP_REG_IDR)

/* Fields of the MEM-AP's CSW register */
#define CSW_SIZE_MASK		7
#define CSW_8BIT		0
#define CSW_16BIT		1
#define CSW_32BIT		2
#define CSW_ADDRINC_MASK    (3UL << 4)
#define CSW_ADDRINC_OFF     0UL
#define CSW_ADDRINC_SINGLE  (1UL << 4)
#define CSW_ADDRINC_PACKED  (2UL << 4)
#define CSW_DEVICE_EN       (1UL << 6)
#define CSW_TRIN_PROG       (1UL << 7)

/* All fields in bits 12 and above are implementation-defined
 * Defaults for AHB/AXI in "Standard Memory Access Port Definitions" from ADI
 * Some bits are shared between buses
 */
#define CSW_SPIDEN          (1UL << 23)
#define CSW_DBGSWENABLE     (1UL << 31)

/* AHB: Privileged */
#define CSW_AHB_HPROT1          (1UL << 25)
/* AHB: set HMASTER signals to AHB-AP ID */
#define CSW_AHB_MASTER_DEBUG    (1UL << 29)
/* AHB5: non-secure access via HNONSEC
 * AHB3: SBO, UNPREDICTABLE if zero */
#define CSW_AHB_SPROT           (1UL << 30)
/* AHB: initial value of csw_default */
#define CSW_AHB_DEFAULT         (CSW_AHB_HPROT1 | CSW_AHB_MASTER_DEBUG | CSW_DBGSWENABLE)

/* AXI: Privileged */
#define CSW_AXI_ARPROT0_PRIV    (1UL << 28)
/* AXI: Non-secure */
#define CSW_AXI_ARPROT1_NONSEC  (1UL << 29)
/* AXI: initial value of csw_default */
#define CSW_AXI_DEFAULT         (CSW_AXI_ARPROT0_PRIV | CSW_AXI_ARPROT1_NONSEC | CSW_DBGSWENABLE)

/* APB: initial value of csw_default */
#define CSW_APB_DEFAULT         (CSW_DBGSWENABLE)

/* Fields of the MEM-AP's CFG register */
#define MEM_AP_REG_CFG_BE       BIT(0)
#define MEM_AP_REG_CFG_LA       BIT(1)
#define MEM_AP_REG_CFG_LD       BIT(2)
#define MEM_AP_REG_CFG_INVALID  0xFFFFFFF8

/* Fields of the MEM-AP's IDR register */
#define AP_REG_IDR_REVISION_MASK        (0xF0000000)
#define AP_REG_IDR_REVISION_SHIFT       (28)
#define AP_REG_IDR_DESIGNER_MASK        (0x0FFE0000)
#define AP_REG_IDR_DESIGNER_SHIFT       (17)
#define AP_REG_IDR_CLASS_MASK           (0x0001E000)
#define AP_REG_IDR_CLASS_SHIFT          (13)
#define AP_REG_IDR_VARIANT_MASK         (0x000000F0)
#define AP_REG_IDR_VARIANT_SHIFT        (4)
#define AP_REG_IDR_TYPE_MASK            (0x0000000F)
#define AP_REG_IDR_TYPE_SHIFT           (0)

#define AP_REG_IDR_CLASS_NONE           (0x0)
#define AP_REG_IDR_CLASS_COM            (0x1)
#define AP_REG_IDR_CLASS_MEM_AP         (0x8)

#define AP_REG_IDR_VALUE(d, c, t) (\
	(((d) << AP_REG_IDR_DESIGNER_SHIFT) & AP_REG_IDR_DESIGNER_MASK) | \
	(((c) << AP_REG_IDR_CLASS_SHIFT) & AP_REG_IDR_CLASS_MASK) | \
	(((t) << AP_REG_IDR_TYPE_SHIFT) & AP_REG_IDR_TYPE_MASK) \
)

#define AP_TYPE_MASK (AP_REG_IDR_DESIGNER_MASK | AP_REG_IDR_CLASS_MASK | AP_REG_IDR_TYPE_MASK)

/* FIXME: not SWD specific; should be renamed, e.g. adiv5_special_seq */
enum swd_special_seq {
	LINE_RESET,
	JTAG_TO_SWD,
	JTAG_TO_DORMANT,
	SWD_TO_JTAG,
	SWD_TO_DORMANT,
	DORMANT_TO_SWD,
	DORMANT_TO_JTAG,
};

/**
 * This represents an ARM Debug Interface (v5) Access Port (AP).
 * Most common is a MEM-AP, for memory access.
 */
struct adiv5_ap {
	/**
	 * DAP this AP belongs to.
	 */
	struct adiv5_dap *dap;

	/**
	 * ADIv5: Number of this AP (0~255)
	 * ADIv6: Base address of this AP (4k aligned)
	 * TODO: to be more coherent, it should be renamed apsel
	 */
	uint64_t ap_num;

	/**
	 * Default value for (MEM-AP) AP_REG_CSW register.
	 */
	uint32_t csw_default;

	/**
	 * Cache for (MEM-AP) AP_REG_CSW register value.  This is written to
	 * configure an access mode, such as autoincrementing AP_REG_TAR during
	 * word access.  "-1" indicates no cached value.
	 */
	uint32_t csw_value;

	/**
	 * Cache for (MEM-AP) AP_REG_TAR register value This is written to
	 * configure the address being read or written
	 * "-1" indicates no cached value.
	 */
	target_addr_t tar_value;

	/**
	 * Configures how many extra tck clocks are added after starting a
	 * MEM-AP access before we try to read its status (and/or result).
	 */
	uint32_t memaccess_tck;

	/* Size of TAR autoincrement block, ARM ADI Specification requires at least 10 bits */
	uint32_t tar_autoincr_block;

	/* true if packed transfers are supported by the MEM-AP */
	bool packed_transfers;

	/* true if unaligned memory access is not supported by the MEM-AP */
	bool unaligned_access_bad;

	/* true if tar_value is in sync with TAR register */
	bool tar_valid;

	/* MEM AP configuration register indicating LPAE support */
	uint32_t cfg_reg;

	/* references counter */
	unsigned int refcount;

	/* AP referenced during config. Never put it, even when refcount reaches zero */
	bool config_ap_never_release;
};


/**
 * This represents an ARM Debug Interface (v5) Debug Access Port (DAP).
 * A DAP has two types of component:  one Debug Port (DP), which is a
 * transport agent; and at least one Access Port (AP), controlling
 * resource access.
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

	/* dap transaction list for WAIT support */
	struct list_head cmd_journal;

	/* pool for dap_cmd objects */
	struct list_head cmd_pool;

	/* number of dap_cmd objects in the pool */
	size_t cmd_pool_size;

	struct jtag_tap *tap;
	/* Control config */
	uint32_t dp_ctrl_stat;

	struct adiv5_ap ap[DP_APSEL_MAX + 1];

	/* The current manually selected AP by the "dap apsel" command */
	uint64_t apsel;

	/**
	 * Cache for DP_SELECT register. A value of DP_SELECT_INVALID
	 * indicates no cached value and forces rewrite of the register.
	 */
	uint64_t select;

	/* information about current pending SWjDP-AHBAP transaction */
	uint8_t  ack;

	/**
	 * Holds the pointer to the destination word for the last queued read,
	 * for use with posted AP read sequence optimization.
	 */
	uint32_t *last_read;

	/* The TI TMS470 and TMS570 series processors use a BE-32 memory ordering
	 * despite lack of support in the ARMv7 architecture. Memory access through
	 * the AHB-AP has strange byte ordering these processors, and we need to
	 * swizzle appropriately. */
	bool ti_be_32_quirks;

	/* The Nuvoton NPCX M4 has an issue with writing to non-4-byte-aligned mmios.
	 * The work around is to repeat the data in all 4 bytes of DRW */
	bool nu_npcx_quirks;

	/**
	 * STLINK adapter need to know if last AP operation was read or write, and
	 * in case of write has to flush it with a dummy read from DP_RDBUFF
	 */
	bool stlink_flush_ap_write;

	/**
	 * Signals that an attempt to reestablish communication afresh
	 * should be performed before the next access.
	 */
	bool do_reconnect;

	/** Flag saying whether to ignore the syspwrupack flag in DAP. Some devices
	 *  do not set this bit until later in the bringup sequence */
	bool ignore_syspwrupack;

	/** Value to select DP in SWD multidrop mode or DP_TARGETSEL_INVALID */
	uint32_t multidrop_targetsel;
	/** TPARTNO and TDESIGNER fields of multidrop_targetsel have been configured */
	bool multidrop_dp_id_valid;
	/** TINSTANCE field of multidrop_targetsel has been configured */
	bool multidrop_instance_id_valid;

	/**
	 * Record if enter in SWD required passing through DORMANT
	 */
	bool switch_through_dormant;

	/** Indicates ADI version (5, 6 or 0 for unknown) being used */
	unsigned int adi_version;

	/* ADIv6 only field indicating ROM Table address size */
	unsigned int asize;
};

/**
 * Transport-neutral representation of queued DAP transactions, supporting
 * both JTAG and SWD transports.  All submitted transactions are logically
 * queued, until the queue is executed by run().  Some implementations might
 * execute transactions as soon as they're submitted, but no status is made
 * available until run().
 */
struct dap_ops {
	/** connect operation for SWD */
	int (*connect)(struct adiv5_dap *dap);

	/** send a sequence to the DAP */
	int (*send_sequence)(struct adiv5_dap *dap, enum swd_special_seq seq);

	/** DP register read. */
	int (*queue_dp_read)(struct adiv5_dap *dap, unsigned reg,
			uint32_t *data);
	/** DP register write. */
	int (*queue_dp_write)(struct adiv5_dap *dap, unsigned reg,
			uint32_t data);

	/** AP register read. */
	int (*queue_ap_read)(struct adiv5_ap *ap, unsigned reg,
			uint32_t *data);
	/** AP register write. */
	int (*queue_ap_write)(struct adiv5_ap *ap, unsigned reg,
			uint32_t data);

	/** AP operation abort. */
	int (*queue_ap_abort)(struct adiv5_dap *dap, uint8_t *ack);

	/** Executes all queued DAP operations. */
	int (*run)(struct adiv5_dap *dap);

	/** Executes all queued DAP operations but doesn't check
	 * sticky error conditions */
	int (*sync)(struct adiv5_dap *dap);

	/** Optional; called at OpenOCD exit */
	void (*quit)(struct adiv5_dap *dap);
};

/*
 * Access Port types
 */
enum ap_type {
	AP_TYPE_JTAG_AP  = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_NONE,   0),  /* JTAG-AP */
	AP_TYPE_COM_AP   = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_COM,    0),  /* COM-AP */
	AP_TYPE_AHB3_AP  = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_MEM_AP, 1),  /* AHB3 Memory-AP */
	AP_TYPE_APB_AP   = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_MEM_AP, 2),  /* APB2 or APB3 Memory-AP */
	AP_TYPE_AXI_AP   = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_MEM_AP, 4),  /* AXI3 or AXI4 Memory-AP */
	AP_TYPE_AHB5_AP  = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_MEM_AP, 5),  /* AHB5 Memory-AP */
	AP_TYPE_APB4_AP  = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_MEM_AP, 6),  /* APB4 Memory-AP */
	AP_TYPE_AXI5_AP  = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_MEM_AP, 7),  /* AXI5 Memory-AP */
	AP_TYPE_AHB5H_AP = AP_REG_IDR_VALUE(ARM_ID, AP_REG_IDR_CLASS_MEM_AP, 8),  /* AHB5 with enhanced HPROT Memory-AP */
};

/* Check the ap->cfg_reg Long Address field (bit 1)
 *
 * 0b0: The AP only supports physical addresses 32 bits or smaller
 * 0b1: The AP supports physical addresses larger than 32 bits
 *
 * @param ap The AP used for reading.
 *
 * @return true for 64 bit, false for 32 bit
 */
static inline bool is_64bit_ap(struct adiv5_ap *ap)
{
	return (ap->cfg_reg & MEM_AP_REG_CFG_LA) != 0;
}

/**
 * Check if DAP is ADIv6
 *
 * @param dap The DAP to test
 *
 * @return true for ADIv6, false for either ADIv5 or unknown version
 */
static inline bool is_adiv6(const struct adiv5_dap *dap)
{
	return dap->adi_version == 6;
}

/**
 * Send an adi-v5 sequence to the DAP.
 *
 * @param dap The DAP used for reading.
 * @param seq The sequence to send.
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_send_sequence(struct adiv5_dap *dap,
		enum swd_special_seq seq)
{
	assert(dap->ops);
	return dap->ops->send_sequence(dap, seq);
}

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
	assert(dap->ops);
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
	assert(dap->ops);
	return dap->ops->queue_dp_write(dap, reg, data);
}

/**
 * Queue an AP register read.
 *
 * @param ap The AP used for reading.
 * @param reg The number of the AP register being read.
 * @param data Pointer saying where to store the register's value
 * (in host endianness).
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_read(struct adiv5_ap *ap,
		unsigned reg, uint32_t *data)
{
	assert(ap->dap->ops);
	if (ap->refcount == 0) {
		ap->refcount = 1;
		LOG_ERROR("BUG: refcount AP#0x%" PRIx64 " used without get", ap->ap_num);
	}
	return ap->dap->ops->queue_ap_read(ap, reg, data);
}

/**
 * Queue an AP register write.
 *
 * @param ap The AP used for writing.
 * @param reg The number of the AP register being written.
 * @param data Value being written (host endianness)
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_write(struct adiv5_ap *ap,
		unsigned reg, uint32_t data)
{
	assert(ap->dap->ops);
	if (ap->refcount == 0) {
		ap->refcount = 1;
		LOG_ERROR("BUG: refcount AP#0x%" PRIx64 " used without get", ap->ap_num);
	}
	return ap->dap->ops->queue_ap_write(ap, reg, data);
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
	assert(dap->ops);
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
	assert(dap->ops);
	return dap->ops->run(dap);
}

static inline int dap_sync(struct adiv5_dap *dap)
{
	assert(dap->ops);
	if (dap->ops->sync)
		return dap->ops->sync(dap);
	return ERROR_OK;
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
	LOG_DEBUG("DAP: poll %x, mask 0x%08" PRIx32 ", value 0x%08" PRIx32,
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
		return ERROR_WAIT;
	} else {
		return ERROR_OK;
	}
}

/* Queued MEM-AP memory mapped single word transfers. */
int mem_ap_read_u32(struct adiv5_ap *ap,
		target_addr_t address, uint32_t *value);
int mem_ap_write_u32(struct adiv5_ap *ap,
		target_addr_t address, uint32_t value);

/* Synchronous MEM-AP memory mapped single word transfers. */
int mem_ap_read_atomic_u32(struct adiv5_ap *ap,
		target_addr_t address, uint32_t *value);
int mem_ap_write_atomic_u32(struct adiv5_ap *ap,
		target_addr_t address, uint32_t value);

/* Synchronous MEM-AP memory mapped bus block transfers. */
int mem_ap_read_buf(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address);
int mem_ap_write_buf(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address);

/* Synchronous, non-incrementing buffer functions for accessing fifos. */
int mem_ap_read_buf_noincr(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address);
int mem_ap_write_buf_noincr(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, target_addr_t address);

/* Initialisation of the debug system, power domains and registers */
int dap_dp_init(struct adiv5_dap *dap);
int dap_dp_init_or_reconnect(struct adiv5_dap *dap);
int mem_ap_init(struct adiv5_ap *ap);

/* Invalidate cached DP select and cached TAR and CSW of all APs */
void dap_invalidate_cache(struct adiv5_dap *dap);

/* read ADIv6 baseptr register */
int adiv6_dap_read_baseptr(struct command_invocation *cmd, struct adiv5_dap *dap, target_addr_t *baseptr);

/* test if ap_num is valid, based on current knowledge of dap */
bool is_ap_num_valid(struct adiv5_dap *dap, uint64_t ap_num);

/* Probe Access Ports to find a particular type. Increment AP refcount */
int dap_find_get_ap(struct adiv5_dap *dap,
			enum ap_type type_to_find,
			struct adiv5_ap **ap_out);

/* Return AP with specified ap_num. Increment AP refcount */
struct adiv5_ap *dap_get_ap(struct adiv5_dap *dap, uint64_t ap_num);

/* Return AP with specified ap_num. Increment AP refcount and keep it non-zero */
struct adiv5_ap *dap_get_config_ap(struct adiv5_dap *dap, uint64_t ap_num);

/* Decrement AP refcount and release the AP when refcount reaches zero */
int dap_put_ap(struct adiv5_ap *ap);

/** Check if SWD multidrop configuration is valid */
static inline bool dap_is_multidrop(struct adiv5_dap *dap)
{
	return dap->multidrop_dp_id_valid && dap->multidrop_instance_id_valid;
}

/* Lookup CoreSight component */
int dap_lookup_cs_component(struct adiv5_ap *ap,
			uint8_t type, target_addr_t *addr, int32_t idx);

struct target;

/* Put debug link into SWD mode */
int dap_to_swd(struct adiv5_dap *dap);

/* Put debug link into JTAG mode */
int dap_to_jtag(struct adiv5_dap *dap);

extern const struct command_registration dap_instance_commands[];

struct arm_dap_object;
extern struct adiv5_dap *dap_instance_by_jim_obj(Jim_Interp *interp, Jim_Obj *o);
extern struct adiv5_dap *adiv5_get_dap(struct arm_dap_object *obj);
extern int dap_info_command(struct command_invocation *cmd,
					 struct adiv5_ap *ap);
extern int dap_register_commands(struct command_context *cmd_ctx);
extern const char *adiv5_dap_name(struct adiv5_dap *self);
extern const struct swd_driver *adiv5_dap_swd_driver(struct adiv5_dap *self);
extern int dap_cleanup_all(void);

struct adiv5_private_config {
	uint64_t ap_num;
	struct adiv5_dap *dap;
};

extern int adiv5_verify_config(struct adiv5_private_config *pc);
extern int adiv5_jim_configure(struct target *target, struct jim_getopt_info *goi);

struct adiv5_mem_ap_spot {
	struct adiv5_dap *dap;
	uint64_t ap_num;
	uint32_t base;
};

extern int adiv5_mem_ap_spot_init(struct adiv5_mem_ap_spot *p);
extern int adiv5_jim_mem_ap_spot_configure(struct adiv5_mem_ap_spot *cfg,
		struct jim_getopt_info *goi);

#endif /* OPENOCD_TARGET_ARM_ADI_V5_H */

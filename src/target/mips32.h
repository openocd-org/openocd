/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_MIPS32_H
#define OPENOCD_TARGET_MIPS32_H

#include <helper/bits.h>

#include "target.h"
#include "mips32_pracc.h"

#define MIPS32_COMMON_MAGIC		0xB320B320U

/**
 * Memory segments (32bit kernel mode addresses)
 * These are the traditional names used in the 32-bit universe.
 */
#define KUSEG			0x00000000
#define KSEG0			0x80000000
#define KSEG1			0xa0000000
#define KSEG2			0xc0000000
#define KSEG3			0xe0000000

/** Returns the kernel segment base of a given address */
#define KSEGX(a)		((a) & 0xe0000000)

/** CP0 CONFIG register fields */
#define MIPS32_CONFIG0_KU_SHIFT 25
#define MIPS32_CONFIG0_KU_MASK (0x7 << MIPS32_CONFIG0_KU_SHIFT)

#define MIPS32_CONFIG0_K0_SHIFT 0
#define MIPS32_CONFIG0_K0_MASK (0x7 << MIPS32_CONFIG0_K0_SHIFT)

#define MIPS32_CONFIG0_K23_SHIFT 28
#define MIPS32_CONFIG0_K23_MASK (0x7 << MIPS32_CONFIG0_K23_SHIFT)

#define MIPS32_CONFIG0_AR_SHIFT 10
#define MIPS32_CONFIG0_AR_MASK (0x7 << MIPS32_CONFIG0_AR_SHIFT)

#define MIPS32_CONFIG1_FP_SHIFT	0
#define MIPS32_CONFIG1_FP_MASK	BIT(MIPS32_CONFIG1_FP_SHIFT)

#define MIPS32_CONFIG1_DL_SHIFT 10
#define MIPS32_CONFIG1_DL_MASK (0x7 << MIPS32_CONFIG1_DL_SHIFT)

#define MIPS32_CONFIG3_CDMM_SHIFT	3
#define MIPS32_CONFIG3_CDMM_MASK	BIT(MIPS32_CONFIG3_CDMM_SHIFT)

#define MIPS32_CONFIG3_DSPP_SHIFT	10
#define MIPS32_CONFIG3_DSPP_MASK	BIT(MIPS32_CONFIG3_DSPP_SHIFT)

#define MIPS32_CONFIG3_DSPREV_SHIFT	11
#define MIPS32_CONFIG3_DSPREV_MASK	BIT(MIPS32_CONFIG3_DSPREV_SHIFT)

#define MIPS32_CONFIG3_ISA_SHIFT	14
#define MIPS32_CONFIG3_ISA_MASK		(3 << MIPS32_CONFIG3_ISA_SHIFT)

#define MIPS32_ARCH_REL1 0x0
#define MIPS32_ARCH_REL2 0x1

#define MIPS32_SCAN_DELAY_LEGACY_MODE 2000000

#define MIPS32NUMDSPREGS		7

/* Bit Mask indicating CP0 register supported by this core */
#define	MIPS_CP0_MK4		0x0001
#define	MIPS_CP0_MAPTIV_UC	0x0002
#define	MIPS_CP0_MAPTIV_UP	0x0004
#define MIPS_CP0_IAPTIV		0x0008

/* CP0 Status register fields */
#define MIPS32_CP0_STATUS_MX_SHIFT	24
#define MIPS32_CP0_STATUS_FR_SHIFT	26
#define MIPS32_CP0_STATUS_CU1_SHIFT	29

/* CP1 FIR register fields */
#define MIPS32_CP1_FIR_F64_SHIFT	22

static const struct mips32_cp0 {
	unsigned int reg;
	unsigned int sel;
	const char *name;
	const unsigned int core;
} mips32_cp0_regs[] = {
	{0, 0, "index", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{0, 1, "mvpcontrol", MIPS_CP0_IAPTIV},
	{0, 2, "mvpconf0", MIPS_CP0_IAPTIV},
	{0, 3, "mvpconf1", MIPS_CP0_IAPTIV},
	{1, 0, "random", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{1, 1, "vpecontrol", MIPS_CP0_IAPTIV},
	{1, 2, "vpeconf0", MIPS_CP0_IAPTIV},
	{1, 3, "vpeconf1", MIPS_CP0_IAPTIV},
	{1, 4, "yqmask", MIPS_CP0_IAPTIV},
	{1, 5, "vpeschedule", MIPS_CP0_IAPTIV},
	{1, 6, "vpeschefback", MIPS_CP0_IAPTIV},
	{1, 7, "vpeopt", MIPS_CP0_IAPTIV},
	{2, 0, "entrylo0", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{2, 1, "tcstatus", MIPS_CP0_IAPTIV},
	{2, 2, "tcbind", MIPS_CP0_IAPTIV},
	{2, 3, "tcrestart", MIPS_CP0_IAPTIV},
	{2, 4, "tchalt", MIPS_CP0_IAPTIV},
	{2, 5, "tccontext", MIPS_CP0_IAPTIV},
	{2, 6, "tcschedule", MIPS_CP0_IAPTIV},
	{2, 7, "tcschefback", MIPS_CP0_IAPTIV},
	{3, 0, "entrylo1", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{3, 7, "tcopt", MIPS_CP0_IAPTIV},
	{4, 0, "context", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{4, 2, "userlocal", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{5, 0, "pagemask", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{5, 1, "pagegrain", MIPS_CP0_MAPTIV_UP},
	{5, 2, "segctl0", MIPS_CP0_IAPTIV},
	{5, 3, "segctl1", MIPS_CP0_IAPTIV},
	{5, 4, "segctl2", MIPS_CP0_IAPTIV},
	{6, 0, "wired", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{6, 1, "srsconf0", MIPS_CP0_IAPTIV},
	{6, 2, "srsconf1", MIPS_CP0_IAPTIV},
	{6, 3, "srsconf2", MIPS_CP0_IAPTIV},
	{6, 4, "srsconf3", MIPS_CP0_IAPTIV},
	{6, 5, "srsconf4", MIPS_CP0_IAPTIV},
	{7, 0, "hwrena", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{8, 0, "badvaddr", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{8, 1, "badinstr", MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP},
	{8, 2, "badinstrp", MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP},
	{9, 0, "count", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{10, 0, "entryhi", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP},
	{10, 4, "guestctl1", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MK4},
	{10, 5, "guestctl2", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MK4},
	{10, 6, "guestctl3", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MK4},
	{11, 0, "compare", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{11, 4, "guestctl0ext", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MK4},
	{12, 0, "status", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{12, 1, "intctl", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{12, 2, "srsctl", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{12, 3, "srsmap", MIPS_CP0_IAPTIV},
	{12, 3, "srsmap1", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP},
	{12, 4, "view_ipl", MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{12, 5, "srsmap2", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP},
	{12, 6, "guestctl0", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MK4},
	{12, 7, "gtoffset", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MK4},
	{13, 0, "cause", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{13, 5, "nestedexc", MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{14, 0, "epc", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{14, 2, "nestedepc", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{15, 0, "prid", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{15, 1, "ebase", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{15, 2, "cdmmbase", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{15, 3, "cmgcrbase", MIPS_CP0_IAPTIV},
	{16, 0, "config", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{16, 1, "config1", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{16, 2, "config2", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{16, 3, "config3", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{16, 4, "config4", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{16, 5, "config5", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{16, 7, "config7", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{17, 0, "lladdr", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{18, 0, "watchlo0", MIPS_CP0_IAPTIV},
	{18, 1, "watchlo1", MIPS_CP0_IAPTIV},
	{18, 2, "watchlo2", MIPS_CP0_IAPTIV},
	{18, 3, "watchlo3", MIPS_CP0_IAPTIV},
	{19, 0, "watchhi0", MIPS_CP0_IAPTIV},
	{19, 1, "watchhi1", MIPS_CP0_IAPTIV},
	{19, 2, "watchhi2", MIPS_CP0_IAPTIV},
	{19, 3, "watchhi3", MIPS_CP0_IAPTIV},
	{23, 0, "debug", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{23, 1, "tracecontrol", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{23, 2, "tracecontrol2", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{23, 3, "usertracedata1", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{23, 4, "tracebpc", MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{23, 4, "traceibpc", MIPS_CP0_IAPTIV},
	{23, 5, "tracedbpc", MIPS_CP0_IAPTIV},
	{24, 0, "depc", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{24, 2, "tracecontrol3", MIPS_CP0_IAPTIV},
	{24, 3, "usertracedata2", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{25, 0, "perfctl0", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{25, 1, "perfcnt0", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{25, 2, "perfctl1", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{25, 3, "perfcnt1", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{26, 0, "errctl", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{27, 0, "cacheerr", MIPS_CP0_IAPTIV},
	{28, 0, "itaglo", MIPS_CP0_IAPTIV},
	{28, 0, "taglo", MIPS_CP0_IAPTIV},
	{28, 1, "idatalo", MIPS_CP0_IAPTIV},
	{28, 1, "datalo", MIPS_CP0_IAPTIV},
	{28, 2, "dtaglo", MIPS_CP0_IAPTIV},
	{28, 3, "ddatalo", MIPS_CP0_IAPTIV},
	{28, 4, "l23taglo", MIPS_CP0_IAPTIV},
	{28, 5, "l23datalo", MIPS_CP0_IAPTIV},
	{29, 1, "idatahi", MIPS_CP0_IAPTIV},
	{29, 2, "dtaghi", MIPS_CP0_IAPTIV},
	{29, 5, "l23datahi", MIPS_CP0_IAPTIV},
	{30, 0, "errorepc", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{31, 0, "desave", MIPS_CP0_IAPTIV | MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP | MIPS_CP0_MK4},
	{31, 2, "kscratch1", MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP},
	{31, 3, "kscratch2", MIPS_CP0_MAPTIV_UC | MIPS_CP0_MAPTIV_UP},
};

#define MIPS32NUMCP0REGS (ARRAY_SIZE(mips32_cp0_regs))

/* Insert extra NOPs after the DRET instruction on exit from debug. */
#define	EJTAG_QUIRK_PAD_DRET		BIT(0)

/* offsets into mips32 core register cache */
enum {
	MIPS32_PC = 37,
	MIPS32_FIR = 71,
	MIPS32_DSPCTL = 78,
	MIPS32NUMCOREREGS
};

/* offsets into mips32 core register cache */

#define MIPS32_REG_GP_COUNT			34
#define MIPS32_REG_FP_COUNT			32
#define MIPS32_REG_FPC_COUNT			2
#define MIPS32_REG_C0_COUNT			5
#define MIPS32_REG_DSP_COUNT		7

#define MIPS32_REGLIST_GP_INDEX			0
#define MIPS32_REGLIST_FP_INDEX			(MIPS32_REGLIST_GP_INDEX + MIPS32_REG_GP_COUNT)
#define MIPS32_REGLIST_FPC_INDEX		(MIPS32_REGLIST_FP_INDEX + MIPS32_REG_FP_COUNT)
#define MIPS32_REGLIST_C0_INDEX			(MIPS32_REGLIST_FPC_INDEX + MIPS32_REG_FPC_COUNT)
#define MIPS32_REGLIST_DSP_INDEX		(MIPS32_REGLIST_C0_INDEX + MIPS32_REG_C0_COUNT)

#define MIPS32_REGLIST_C0_STATUS_INDEX		(MIPS32_REGLIST_C0_INDEX + 0)
#define MIPS32_REGLIST_C0_BADVADDR_INDEX	(MIPS32_REGLIST_C0_INDEX + 1)
#define MIPS32_REGLIST_C0_CAUSE_INDEX		(MIPS32_REGLIST_C0_INDEX + 2)
#define MIPS32_REGLIST_C0_PC_INDEX		(MIPS32_REGLIST_C0_INDEX + 3)
#define MIPS32_REGLIST_C0_GUESTCTL1_INDEX	(MIPS32_REGLIST_C0_INDEX + 4)

#define MIPS32_REG_C0_STATUS_INDEX		0
#define MIPS32_REG_C0_BADVADDR_INDEX		1
#define MIPS32_REG_C0_CAUSE_INDEX		2
#define MIPS32_REG_C0_PC_INDEX			3
#define MIPS32_REG_C0_GUESTCTL1_INDEX		4

#define MIPS32_REGLIST_DSP_DSPCTL_INDEX		(MIPS32_REGLIST_DSP_INDEX + 6)

#define MIPS32_REG_DSP_DSPCTL_INDEX		6

enum mips32_isa_mode {
	MIPS32_ISA_MIPS32 = 0,
	MIPS32_ISA_MIPS16E = 1,
	MIPS32_ISA_MMIPS32 = 3,
};

enum mips32_isa_imp {
	MIPS32_ONLY = 0,
	MMIPS32_ONLY = 1,
	MIPS32_MIPS16 = 2,
	MIPS32_MMIPS32 = 3,
};

/* Release 2~5 does not have much change regarding to the ISA under User mode,
* therefore no new Architecture Revision(AR) level is assigned to them.
* Release 6 changed some instruction's encoding/mnemonic, removed instructions that
* has lost its purposes/none are using, and added some new instructions as well.
*/
enum mips32_isa_rel {
	MIPS32_RELEASE_1 = 0,
	MIPS32_RELEASE_2 = 1,
	MIPS32_RELEASE_6 = 2,
	MIPS32_RELEASE_UNKNOWN,
};

enum mips32_isa_supported {
	MIPS16,
	MIPS32,
	MIPS64,
	MICROMIPS_ONLY,
	MIPS32_AT_RESET_AND_MICROMIPS,
	MICROMIPS_AT_RESET_AND_MIPS32,
};
#define MIPS32_CORE_MASK	0xFFFFFF00
#define MIPS32_VARIANT_MASK	0x00FF

/* This struct contains mips cpu types with their name respectively.
 * The PrID register format is as following:
 *  - Company Optionsp[31:24]
 *  - Company ID[23:16]
 *  - Processor ID[15:8]
 *  - Revision[7:0]
 * Here the revision field represents the maximum value of revision.
 */
static const struct cpu_entry {
	uint32_t prid;
	enum mips32_isa_supported isa;
	const char *vendor;
	const char *cpu_name;
} mips32_cpu_entry[] = {
	/* MIPS Technologies cores */
	{0x000180FF, MIPS32, "MIPS", "4Kc"},
	{0x000181FF, MIPS64, "MIPS", "5Kc"},
	{0x000182FF, MIPS64, "MIPS", "20Kc"},
	{0x000183FF, MIPS32, "MIPS", "4KM"},

	{0x000184FF, MIPS32, "MIPS", "4KEc"},
	{0x000190FF, MIPS32, "MIPS", "4KEc"},

	{0x000185FF, MIPS32, "MIPS", "4KEm"},
	{0x000191FF, MIPS32, "MIPS", "4KEm"},

	{0x000186FF, MIPS32, "MIPS", "4KSc"},
	{0x000187FF, MIPS32, "MIPS", "M4K"},
	{0x000188FF, MIPS64, "MIPS", "25Kf"},
	{0x000189FF, MIPS64, "MIPS", "5KEc"},
	{0x000192FF, MIPS32, "MIPS", "4KSD"},
	{0x000193FF, MIPS32, "MIPS", "24Kc"},
	{0x000195FF, MIPS32, "MIPS", "34Kc"},
	{0x000196FF, MIPS32, "MIPS", "24KEc"},
	{0x000197FF, MIPS32, "MIPS", "74Kc"},
	{0x000199FF, MIPS32, "MIPS", "1004Kc"},
	{0x00019AFF, MIPS32, "MIPS", "1074Kc"},
	{0x00019BFF, MIPS32, "MIPS", "M14K"},
	{0x00019CFF, MIPS32, "MIPS", "M14Kc"},
	{0x00019DFF, MIPS32, "MIPS", "microAptiv_UC(M14KE)"},
	{0x00019EFF, MIPS32, "MIPS", "microAptiv_UP(M14KEc)"},
	{0x0001A0FF, MIPS32, "MIPS", "interAptiv"},
	{0x0001A1FF, MIPS32, "MIPS", "interAptiv_CM"},
	{0x0001A2FF, MIPS32, "MIPS", "proAptiv"},
	{0x0001A3FF, MIPS32, "MIPS", "proAptiv_CM"},
	{0x0001A6FF, MIPS32, "MIPS", "M5100"},
	{0x0001A7FF, MIPS32, "MIPS", "M5150"},
	{0x0001A8FF, MIPS32, "MIPS", "P5600"},
	{0x0001A9FF, MIPS32, "MIPS", "I5500"},

	/* Broadcom */
	{0x000200FF, MIPS32, "Broadcom", "Broadcom"},

	/* AMD Alchemy Series*/
	/* NOTE: AMD/Alchemy series uses Company Option instead of
	 * Processor ID, to match the find function, Processor ID field
	 * is the copy of Company Option field */
	{0x000300FF, MIPS32, "AMD Alchemy", "AU1000"},
	{0x010301FF, MIPS32, "AMD Alchemy", "AU1500"},
	{0x020302FF, MIPS32, "AMD Alchemy", "AU1100"},
	{0x030303FF, MIPS32, "AMD Alchemy", "AU1550"},
	{0x04030401, MIPS32, "AMD Alchemy", "AU1200"},
	{0x040304FF, MIPS32, "AMD Alchemy", "AU1250"},
	{0x050305FF, MIPS32, "AMD Alchemy", "AU1210"},

	/* Altera */
	{0x001000FF, MIPS32, "Altera", "Altera"},

	/* Lexra */
	{0x000B00FF, MIPS32, "Lexra", "Lexra"},

	/* Ingenic */
	{0x00e102FF, MIPS32, "Ingenic", "Ingenic XBurst rev1"},

	{0xFFFFFFFF, MIPS32, "Unknown", "Unknown"}
};

#define MIPS32_NUM_CPU_ENTRIES (ARRAY_SIZE(mips32_cpu_entry))

enum mips32_fp_imp {
	MIPS32_FP_IMP_NONE = 0,
	MIPS32_FP_IMP_32 = 1,
	MIPS32_FP_IMP_64 = 2,
	MIPS32_FP_IMP_UNKNOWN = 3,
};

enum mips32_dsp_imp {
	MIPS32_DSP_IMP_NONE = 0,
	MIPS32_DSP_IMP_REV1 = 1,
	MIPS32_DSP_IMP_REV2 = 2,
};

struct mips32_comparator {
	int used;
	uint32_t bp_value;
	uint32_t reg_address;
};

struct mips32_core_regs {
	uint32_t gpr[MIPS32_REG_GP_COUNT];
	uint64_t fpr[MIPS32_REG_FP_COUNT];
	uint32_t fpcr[MIPS32_REG_FPC_COUNT];
	uint32_t cp0[MIPS32_REG_C0_COUNT];
	uint32_t dsp[MIPS32_REG_DSP_COUNT];
};

struct mips32_common {
	unsigned int common_magic;

	void *arch_info;
	struct reg_cache *core_cache;
	struct mips_ejtag ejtag_info;

	struct mips32_core_regs core_regs;

	enum mips32_isa_mode isa_mode;
	enum mips32_isa_imp isa_imp;
	enum mips32_isa_rel isa_rel;
	enum mips32_fp_imp fp_imp;
	enum mips32_dsp_imp dsp_imp;

	int fdc;
	int semihosting;

	/* The cp0 registers implemented on different processor cores could be different, too.
	 * Here you can see most of the registers are implemented on interAptiv, which is
	 * a 2c4t SMP processor, it has more features than M-class processors, like vpe
	 * and other config registers for multhreading. */
	uint32_t cp0_mask;

	/* FPU enabled (cp0.status.cu1) */
	bool fpu_enabled;
	/* FPU mode (cp0.status.fr) */
	bool fpu_in_64bit;

	/* processor identification register */
	uint32_t prid;
	/* detected CPU type */
	const struct cpu_entry *cpu_info;
	/* CPU specific quirks */
	uint32_t cpu_quirks;

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	int bp_scanned;
	int num_inst_bpoints;
	int num_data_bpoints;
	int num_inst_bpoints_avail;
	int num_data_bpoints_avail;
	struct mips32_comparator *inst_break_list;
	struct mips32_comparator *data_break_list;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned int num);
	int (*write_core_reg)(struct target *target, unsigned int num);
};

static inline struct mips32_common *
target_to_mips32(struct target *target)
{
	return target->arch_info;
}

struct mips32_core_reg {
	uint32_t num;
	struct target *target;
	struct mips32_common *mips32_common;
};

struct mips32_algorithm {
	unsigned int common_magic;
	enum mips32_isa_mode isa_mode;
};

#define MIPS32_OP_ADDU	0x21u
#define MIPS32_OP_ADDIU	0x09u
#define MIPS32_OP_ANDI	0x0Cu
#define MIPS32_OP_BEQ	0x04u
#define MIPS32_OP_BGTZ	0x07u
#define MIPS32_OP_BNE	0x05u
#define MIPS32_OP_ADD	0x20u
#define MIPS32_OP_ADDI	0x08u
#define MIPS32_OP_AND	0x24u
#define MIPS32_OP_CACHE	0x2Fu
#define MIPS32_OP_COP0	0x10u
#define MIPS32_OP_COP1	0x11u
#define MIPS32_OP_J	0x02u
#define MIPS32_OP_JR	0x08u
#define MIPS32_OP_LUI	0x0Fu
#define MIPS32_OP_LW	0x23u
#define MIPS32_OP_LWC1	0x31u
#define MIPS32_OP_LDC1	0x35u
#define MIPS32_OP_LB	0x20u
#define MIPS32_OP_LBU	0x24u
#define MIPS32_OP_LHU	0x25u
#define MIPS32_OP_MFHI	0x10u
#define MIPS32_OP_MTHI	0x11u
#define MIPS32_OP_MFLO	0x12u
#define MIPS32_OP_MTLO	0x13u
#define MIPS32_OP_MUL	0x02u
#define MIPS32_OP_RDHWR	0x3Bu
#define MIPS32_OP_SB	0x28u
#define MIPS32_OP_SH	0x29u
#define MIPS32_OP_SW	0x2Bu
#define MIPS32_OP_ORI	0x0Du
#define MIPS32_OP_XORI	0x0Eu
#define MIPS32_OP_XOR	0x26u
#define MIPS32_OP_SLTU	0x2Bu
#define MIPS32_OP_SRL	0x02u
#define MIPS32_OP_SRA	0x03u
#define MIPS32_OP_SYNCI	0x1Fu
#define MIPS32_OP_SLL	0x00u
#define MIPS32_OP_SLLV	0x04u
#define MIPS32_OP_SLTI	0x0Au
#define MIPS32_OP_MOVN	0x0Bu
#define MIPS32_OP_SWC1	0x39u
#define MIPS32_OP_SDC1	0x3Du

#define MIPS32_OP_REGIMM	0x01u
#define MIPS32_OP_SDBBP	0x3Fu
#define MIPS32_OP_SPECIAL	0x00u
#define MIPS32_OP_SPECIAL2	0x07u
#define MIPS32_OP_SPECIAL3	0x1Fu

#define MIPS32_COP_MF	0x00u
#define MIPS32_COP_CF	0x02u
#define MIPS32_COP_MFH	0x03u
#define MIPS32_COP_MT	0x04u
#define MIPS32_COP_MTH	0x07u

#define MIPS32_R_INST(opcode, rs, rt, rd, shamt, funct) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | ((rd) << 11) | ((shamt) << 6) | (funct))
#define MIPS32_I_INST(opcode, rs, rt, immd) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | (immd))
#define MIPS32_J_INST(opcode, addr)	(((opcode) << 26) | (addr))

#define MIPS32_ISA_NOP				0
#define MIPS32_ISA_ADD(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_ADD)
#define MIPS32_ISA_ADDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ADDI, src, tar, val)
#define MIPS32_ISA_ADDIU(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ADDIU, src, tar, val)
#define MIPS32_ISA_ADDU(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_ADDU)
#define MIPS32_ISA_AND(dst, src, tar)		MIPS32_R_INST(0, src, tar, dst, 0, MIPS32_OP_AND)
#define MIPS32_ISA_ANDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ANDI, src, tar, val)

#define MIPS32_ISA_B(off)			MIPS32_ISA_BEQ(0, 0, off)
#define MIPS32_ISA_BEQ(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BEQ, src, tar, off)
#define MIPS32_ISA_BGTZ(reg, off)		MIPS32_I_INST(MIPS32_OP_BGTZ, reg, 0, off)
#define MIPS32_ISA_BNE(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BNE, src, tar, off)
#define MIPS32_ISA_CACHE(op, off, base)		MIPS32_I_INST(MIPS32_OP_CACHE, base, op, off)
#define MIPS32_ISA_CFC1(gpr, cpr)		MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP_CF, gpr, cpr, 0, 0)
#define MIPS32_ISA_J(tar)			MIPS32_J_INST(MIPS32_OP_J, (0x0FFFFFFFu & (tar)) >> 2)
#define MIPS32_ISA_JR(reg)			MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_JR)
#define MIPS32_ISA_JRHB(reg)		MIPS32_R_INST(0, reg, 0, 0, 0x10, MIPS32_OP_JR)

#define MIPS32_ISA_LB(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LB, base, reg, off)
#define MIPS32_ISA_LBU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LBU, base, reg, off)
#define MIPS32_ISA_LHU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LHU, base, reg, off)
#define MIPS32_ISA_LUI(reg, val)		MIPS32_I_INST(MIPS32_OP_LUI, 0, reg, val)
#define MIPS32_ISA_LW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LW, base, reg, off)
#define MIPS32_ISA_LWC1(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LWC1, base, reg, off)
#define MIPS32_ISA_LDC1(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LDC1, base, reg, off)

#define MIPS32_ISA_MFC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP_MF, gpr, cpr, 0, sel)
#define MIPS32_ISA_MTC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP_MT, gpr, cpr, 0, sel)
#define MIPS32_ISA_MFC1(gpr, cpr)		MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP_MF, gpr, cpr, 0, 0)
#define MIPS32_ISA_MFHC1(gpr, cpr)		MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP_MFH, gpr, cpr, 0, 0)
#define MIPS32_ISA_MTC1(gpr, cpr)		MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP_MT, gpr, cpr, 0, 0)
#define MIPS32_ISA_MTHC1(gpr, cpr)		MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP_MTH, gpr, cpr, 0, 0)
#define MIPS32_ISA_MFLO(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFLO)
#define MIPS32_ISA_MFHI(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFHI)
#define MIPS32_ISA_MTLO(reg)			MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTLO)
#define MIPS32_ISA_MTHI(reg)			MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTHI)

#define MIPS32_ISA_MUL(dst, src, t)		MIPS32_R_INST(28, src, t, dst, 0, MIPS32_OP_MUL)
#define MIPS32_ISA_MOVN(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_MOVN)
#define MIPS32_ISA_OR(dst, src, val)		MIPS32_R_INST(0, src, val, dst, 0, 37)
#define MIPS32_ISA_ORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ORI, src, tar, val)
#define MIPS32_ISA_RDHWR(tar, dst)		MIPS32_R_INST(MIPS32_OP_SPECIAL3, 0, tar, dst, 0, MIPS32_OP_RDHWR)
#define MIPS32_ISA_SB(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SB, base, reg, off)
#define MIPS32_ISA_SH(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SH, base, reg, off)
#define MIPS32_ISA_SW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SW, base, reg, off)
#define MIPS32_ISA_SWC1(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SWC1, base, reg, off)
#define MIPS32_ISA_SDC1(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SDC1, base, reg, off)

#define MIPS32_ISA_SLL(dst, src, sa)		MIPS32_R_INST(MIPS32_OP_SPECIAL, 0, src, dst, sa, MIPS32_OP_SLL)
#define MIPS32_ISA_SLLV(dst, src, sa)		MIPS32_R_INST(MIPS32_OP_SPECIAL, 0, src, dst, sa, MIPS32_OP_SLLV)
#define MIPS32_ISA_SLTI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_SLTI, src, tar, val)
#define MIPS32_ISA_SLTU(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_SLTU)
#define MIPS32_ISA_SRA(reg, src, off)		MIPS32_R_INST(MIPS32_OP_SPECIAL, 0, src, reg, off, MIPS32_OP_SRA)
#define MIPS32_ISA_SRL(reg, src, off)		MIPS32_R_INST(MIPS32_OP_SPECIAL, 0, src, reg, off, MIPS32_OP_SRL)
#define MIPS32_ISA_SYNC				0xFu
#define MIPS32_ISA_SYNCI(off, base)		MIPS32_I_INST(MIPS32_OP_REGIMM, base, MIPS32_OP_SYNCI, off)

#define MIPS32_ISA_XOR(reg, val1, val2)		MIPS32_R_INST(0, val1, val2, reg, 0, MIPS32_OP_XOR)
#define MIPS32_ISA_XORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_XORI, src, tar, val)

#define MIPS32_ISA_SYNCI_STEP		0x1	/* reg num od address step size to be used with synci instruction */

/**
 * Cache operations definitions
 * Operation field is 5 bits long :
 * 1) bits 1..0 hold cache type
 * 2) bits 4..2 hold operation code
 */
#define MIPS32_CACHE_D_HIT_WRITEBACK ((0x1 << 0) | (0x6 << 2))
#define MIPS32_CACHE_I_HIT_INVALIDATE ((0x0 << 0) | (0x4 << 2))

/* ejtag specific instructions */
#define MIPS32_ISA_DRET				0x4200001Fu
/* MIPS32_ISA_J_INST(MIPS32_ISA_OP_SPECIAL2, MIPS32_ISA_OP_SDBBP) */
#define MIPS32_ISA_SDBBP			0x7000003Fu
#define MIPS16_ISA_SDBBP			0xE801u

/*MICRO MIPS INSTRUCTIONS, see doc MD00582 */
#define MMIPS32_POOL32A					0x00u
#define MMIPS32_POOL32F					0x15u
#define MMIPS32_POOL32FXF				0x3Bu
#define MMIPS32_POOL32AXF				0x3Cu
#define MMIPS32_POOL32B					0x08u
#define MMIPS32_POOL32I					0x10u
#define MMIPS32_OP_ADDI			0x04u
#define MMIPS32_OP_ADDIU		0x0Cu
#define MMIPS32_OP_ADDU			0x150u
#define MMIPS32_OP_AND			0x250u
#define MMIPS32_OP_ANDI			0x34u
#define MMIPS32_OP_BEQ			0x25u
#define MMIPS32_OP_BGTZ			0x06u
#define MMIPS32_OP_BNE			0x2Du
#define MMIPS32_OP_CACHE		0x06u
#define MMIPS32_OP_CFC1			0x40u
#define MMIPS32_OP_J			0x35u
#define MMIPS32_OP_JALR			0x03Cu
#define MMIPS32_OP_JALRHB		0x07Cu
#define MMIPS32_OP_LB			0x07u
#define MMIPS32_OP_LBU			0x05u
#define MMIPS32_OP_LHU			0x0Du
#define MMIPS32_OP_LUI			0x0Du
#define MMIPS32_OP_LW			0x3Fu
#define MMIPS32_OP_LWC1			0x27u
#define MMIPS32_OP_LDC1			0x2Fu
#define MMIPS32_OP_MFC0			0x03u
#define MMIPS32_OP_MFC1			0x80u
#define MMIPS32_OP_MFHC1		0xC0u
#define MMIPS32_OP_MTC0			0x0Bu
#define MMIPS32_OP_MTC1			0xA0u
#define MMIPS32_OP_MTHC1		0xE0u
#define MMIPS32_OP_MFLO			0x075u
#define MMIPS32_OP_MFHI			0x035u
#define MMIPS32_OP_MTLO			0x0F5u
#define MMIPS32_OP_MTHI			0x0B5u
#define MMIPS32_OP_MOVN			0x018u
#define MMIPS32_OP_ORI			0x14u
#define MMIPS32_OP_RDHWR		0x1ACu
#define MMIPS32_OP_SB			0x06u
#define MMIPS32_OP_SH			0x0Eu
#define MMIPS32_OP_SW			0x3Eu
#define MMIPS32_OP_SWC1			0x26u
#define MMIPS32_OP_SDC1			0x2Eu
#define MMIPS32_OP_SLTU			0x390u
#define MMIPS32_OP_SLL			0x000u
#define MMIPS32_OP_SLTI			0x24u
#define MMIPS32_OP_SRL			0x040u
#define MMIPS32_OP_SYNCI		0x10u
#define MMIPS32_OP_XOR			0x310u
#define MMIPS32_OP_XORI			0x1Cu

#define MMIPS32_ADDI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ADDI, tar, src, val)
#define MMIPS32_ADDIU(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ADDIU, tar, src, val)
#define MMIPS32_ADDU(dst, src, tar)		MIPS32_R_INST(MMIPS32_POOL32A, tar, src, dst, 0, MMIPS32_OP_ADDU)
#define MMIPS32_AND(dst, src, tar)		MIPS32_R_INST(MMIPS32_POOL32A, tar, src, dst, 0, MMIPS32_OP_AND)
#define MMIPS32_ANDI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ANDI, tar, src, val)

#define MMIPS32_B(off)				MMIPS32_BEQ(0, 0, off)
#define MMIPS32_BEQ(src, tar, off)		MIPS32_I_INST(MMIPS32_OP_BEQ, tar, src, off)
#define MMIPS32_BGTZ(reg, off)			MIPS32_I_INST(MMIPS32_POOL32I, MMIPS32_OP_BGTZ, reg, off)
#define MMIPS32_BNE(src, tar, off)		MIPS32_I_INST(MMIPS32_OP_BNE, tar, src, off)
#define MMIPS32_CACHE(op, off, base)		MIPS32_R_INST(MMIPS32_POOL32B, op, base, MMIPS32_OP_CACHE << 1, 0, off)
#define MMIPS32_CFC1(gpr, cpr)			MIPS32_R_INST(MMIPS32_POOL32F, gpr, cpr, 0, MMIPS32_OP_CFC1, MMIPS32_POOL32FXF)

#define MMIPS32_J(tar)				MIPS32_J_INST(MMIPS32_OP_J, ((0x07FFFFFFu & ((tar) >> 1))))
#define MMIPS32_JR(reg)				MIPS32_R_INST(MMIPS32_POOL32A, 0, reg, 0, MMIPS32_OP_JALR, MMIPS32_POOL32AXF)
#define MMIPS32_JRHB(reg)			MIPS32_R_INST(MMIPS32_POOL32A, 0, reg, 0, MMIPS32_OP_JALRHB, MMIPS32_POOL32AXF)
#define MMIPS32_LB(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LB, reg, base, off)
#define MMIPS32_LBU(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LBU, reg, base, off)
#define MMIPS32_LHU(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LHU, reg, base, off)
#define MMIPS32_LUI(reg, val)			MIPS32_I_INST(MMIPS32_POOL32I, MMIPS32_OP_LUI, reg, val)
#define MMIPS32_LW(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LW, reg, base, off)
#define MMIPS32_LWC1(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LWC1, reg, base, off)
#define MMIPS32_LDC1(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LDC1, reg, base, off)

#define MMIPS32_MFC0(gpr, cpr, sel)		MIPS32_R_INST(MMIPS32_POOL32A, gpr, cpr, sel,\
														MMIPS32_OP_MFC0, MMIPS32_POOL32AXF)
#define MMIPS32_MFC1(gpr, cpr)			MIPS32_R_INST(MMIPS32_POOL32F, gpr, cpr, 0, MMIPS32_OP_MFC1, MMIPS32_POOL32FXF)
#define MMIPS32_MFHC1(gpr, cpr)			MIPS32_R_INST(MMIPS32_POOL32F, gpr, cpr, 0, MMIPS32_OP_MFHC1, MMIPS32_POOL32FXF)
#define MMIPS32_MFLO(reg)			MIPS32_R_INST(MMIPS32_POOL32A, 0, reg, 0, MMIPS32_OP_MFLO, MMIPS32_POOL32AXF)
#define MMIPS32_MFHI(reg)			MIPS32_R_INST(MMIPS32_POOL32A, 0, reg, 0, MMIPS32_OP_MFHI, MMIPS32_POOL32AXF)
#define MMIPS32_MTC0(gpr, cpr, sel)		MIPS32_R_INST(MMIPS32_POOL32A, gpr, cpr, sel,\
														MMIPS32_OP_MTC0, MMIPS32_POOL32AXF)
#define MMIPS32_MTC1(gpr, cpr)			MIPS32_R_INST(MMIPS32_POOL32F, gpr, cpr, 0, MMIPS32_OP_MTC1, MMIPS32_POOL32FXF)
#define MMIPS32_MTHC1(gpr, cpr)			MIPS32_R_INST(MMIPS32_POOL32F, gpr, cpr, 0, MMIPS32_OP_MTHC1, MMIPS32_POOL32FXF)
#define MMIPS32_MTLO(reg)			MIPS32_R_INST(MMIPS32_POOL32A, 0, reg, 0, MMIPS32_OP_MTLO, MMIPS32_POOL32AXF)
#define MMIPS32_MTHI(reg)			MIPS32_R_INST(MMIPS32_POOL32A, 0, reg, 0, MMIPS32_OP_MTHI, MMIPS32_POOL32AXF)

#define MMIPS32_MOVN(dst, src, tar)		MIPS32_R_INST(MMIPS32_POOL32A, tar, src, dst, 0, MMIPS32_OP_MOVN)
#define MMIPS32_NOP				0
#define MMIPS32_ORI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ORI, tar, src, val)
#define MMIPS32_RDHWR(tar, dst)			MIPS32_R_INST(MMIPS32_POOL32A, dst, tar, 0, MMIPS32_OP_RDHWR, MMIPS32_POOL32AXF)
#define MMIPS32_SB(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SB, reg, base, off)
#define MMIPS32_SH(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SH, reg, base, off)
#define MMIPS32_SW(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SW, reg, base, off)
#define MMIPS32_SWC1(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SWC1, reg, base, off)
#define MMIPS32_SDC1(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SDC1, reg, base, off)

#define MMIPS32_SRL(reg, src, off)		MIPS32_R_INST(MMIPS32_POOL32A, reg, src, off, 0, MMIPS32_OP_SRL)
#define MMIPS32_SLTU(dst, src, tar)		MIPS32_R_INST(MMIPS32_POOL32A, tar, src, dst, 0, MMIPS32_OP_SLTU)
#define MMIPS32_SYNCI(off, base)		MIPS32_I_INST(MMIPS32_POOL32I, MMIPS32_OP_SYNCI, base, off)
#define MMIPS32_SLL(dst, src, sa)		MIPS32_R_INST(MMIPS32_POOL32A, dst, src, sa, 0, MMIPS32_OP_SLL)
#define MMIPS32_SLLV(dst, src, sa)		MIPS32_R_INST(MMIPS32_POOL32A, dst, src, sa, 0, MMIPS32_OP_SLLV)
#define MMIPS32_SLTI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_SLTI, tar, src, val)
#define MMIPS32_SYNC				0x00001A7Cu /* MIPS32_R_INST(MMIPS32_POOL32A, 0, 0, 0, 0x1ADu, MMIPS32_POOL32AXF) */

#define MMIPS32_XOR(reg, val1, val2)		MIPS32_R_INST(MMIPS32_POOL32A, val1, val2, reg, 0, MMIPS32_OP_XOR)
#define MMIPS32_XORI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_XORI, tar, src, val)

#define MMIPS32_SYNCI_STEP	0x1u	/* reg num od address step size to be used with synci instruction */


/* ejtag specific instructions */
#define MMIPS32_DRET			0x0000E37Cu	/* MIPS32_R_INST(MMIPS32_POOL32A, 0, 0, 0, 0x38D, MMIPS32_POOL32AXF) */
#define MMIPS32_SDBBP			0x0000DB7Cu	/* MIPS32_R_INST(MMIPS32_POOL32A, 0, 0, 0, 0x1BD, MMIPS32_POOL32AXF) */
#define MMIPS16_SDBBP			0x46C0u		/* POOL16C instr */

/* instruction code with isa selection */
#define MIPS32_NOP				0	/* same for both isa's */
#define MIPS32_ADDI(isa, tar, src, val)		(isa ? MMIPS32_ADDI(tar, src, val) : MIPS32_ISA_ADDI(tar, src, val))
#define MIPS32_ADDIU(isa, tar, src, val)	(isa ? MMIPS32_ADDIU(tar, src, val) : MIPS32_ISA_ADDIU(tar, src, val))
#define MIPS32_ADDU(isa, dst, src, tar)		(isa ? MMIPS32_ADDU(dst, src, tar) : MIPS32_ISA_ADDU(dst, src, tar))
#define MIPS32_AND(isa, dst, src, tar)		(isa ? MMIPS32_AND(dst, src, tar) : MIPS32_ISA_AND(dst, src, tar))
#define MIPS32_ANDI(isa, tar, src, val)		(isa ? MMIPS32_ANDI(tar, src, val) : MIPS32_ISA_ANDI(tar, src, val))

#define MIPS32_B(isa, off)			(isa ? MMIPS32_B(off) : MIPS32_ISA_B(off))
#define MIPS32_BEQ(isa, src, tar, off)		(isa ? MMIPS32_BEQ(src, tar, off) : MIPS32_ISA_BEQ(src, tar, off))
#define MIPS32_BGTZ(isa, reg, off)		(isa ? MMIPS32_BGTZ(reg, off) : MIPS32_ISA_BGTZ(reg, off))
#define MIPS32_BNE(isa, src, tar, off)		(isa ? MMIPS32_BNE(src, tar, off) : MIPS32_ISA_BNE(src, tar, off))
#define MIPS32_CACHE(isa, op, off, base)	(isa ? MMIPS32_CACHE(op, off, base) : MIPS32_ISA_CACHE(op, off, base))
#define MIPS32_CFC1(isa, gpr, cpr)		(isa ? MMIPS32_CFC1(gpr, cpr) : MIPS32_ISA_CFC1(gpr, cpr))

#define MIPS32_J(isa, tar)			(isa ? MMIPS32_J(tar) : MIPS32_ISA_J(tar))
#define MIPS32_JR(isa, reg)			(isa ? MMIPS32_JR(reg) : MIPS32_ISA_JR(reg))
#define MIPS32_JRHB(isa, reg)		(isa ? MMIPS32_JRHB(reg) : MIPS32_ISA_JRHB(reg))
#define MIPS32_LB(isa, reg, off, base)		(isa ? MMIPS32_LB(reg, off, base) : MIPS32_ISA_LB(reg, off, base))
#define MIPS32_LBU(isa, reg, off, base)		(isa ? MMIPS32_LBU(reg, off, base) : MIPS32_ISA_LBU(reg, off, base))
#define MIPS32_LHU(isa, reg, off, base)		(isa ? MMIPS32_LHU(reg, off, base) : MIPS32_ISA_LHU(reg, off, base))
#define MIPS32_LW(isa, reg, off, base)		(isa ? MMIPS32_LW(reg, off, base) : MIPS32_ISA_LW(reg, off, base))
#define MIPS32_LWC1(isa, reg, off, base)	(isa ? MMIPS32_LWC1(reg, off, base) : MIPS32_ISA_LWC1(reg, off, base))
#define MIPS32_LUI(isa, reg, val)		(isa ? MMIPS32_LUI(reg, val) : MIPS32_ISA_LUI(reg, val))

#define MIPS32_MFC0(isa, gpr, cpr, sel)		(isa ? MMIPS32_MFC0(gpr, cpr, sel) : MIPS32_ISA_MFC0(gpr, cpr, sel))
#define MIPS32_MTC0(isa, gpr, cpr, sel)		(isa ? MMIPS32_MTC0(gpr, cpr, sel) : MIPS32_ISA_MTC0(gpr, cpr, sel))
#define MIPS32_MFC1(isa, gpr, cpr)		(isa ? MMIPS32_MFC1(gpr, cpr) : MIPS32_ISA_MFC1(gpr, cpr))
#define MIPS32_MFHC1(isa, gpr, cpr)		(isa ? MMIPS32_MFHC1(gpr, cpr) : MIPS32_ISA_MFHC1(gpr, cpr))
#define MIPS32_MTC1(isa, gpr, cpr)		(isa ? MMIPS32_MTC1(gpr, cpr) : MIPS32_ISA_MTC1(gpr, cpr))
#define MIPS32_MTHC1(isa, gpr, cpr)		(isa ? MMIPS32_MTHC1(gpr, cpr) : MIPS32_ISA_MTHC1(gpr, cpr))
#define MIPS32_MFLO(isa, reg)			(isa ? MMIPS32_MFLO(reg) : MIPS32_ISA_MFLO(reg))
#define MIPS32_MFHI(isa, reg)			(isa ? MMIPS32_MFHI(reg) : MIPS32_ISA_MFHI(reg))
#define MIPS32_MTLO(isa, reg)			(isa ? MMIPS32_MTLO(reg) : MIPS32_ISA_MTLO(reg))
#define MIPS32_MTHI(isa, reg)			(isa ? MMIPS32_MTHI(reg) : MIPS32_ISA_MTHI(reg))

#define MIPS32_MUL(isa, dst, src, t)		(MIPS32_ISA_MUL(dst, src, t))
#define MIPS32_MOVN(isa, dst, src, tar)		(isa ? MMIPS32_MOVN(dst, src, tar) : MIPS32_ISA_MOVN(dst, src, tar))
#define MIPS32_ORI(isa, tar, src, val)		(isa ? MMIPS32_ORI(tar, src, val) : MIPS32_ISA_ORI(tar, src, val))
#define MIPS32_RDHWR(isa, tar, dst)		(isa ? MMIPS32_RDHWR(tar, dst) : MIPS32_ISA_RDHWR(tar, dst))
#define MIPS32_SB(isa, reg, off, base)		(isa ? MMIPS32_SB(reg, off, base) : MIPS32_ISA_SB(reg, off, base))
#define MIPS32_SH(isa, reg, off, base)		(isa ? MMIPS32_SH(reg, off, base) : MIPS32_ISA_SH(reg, off, base))
#define MIPS32_SW(isa, reg, off, base)		(isa ? MMIPS32_SW(reg, off, base) : MIPS32_ISA_SW(reg, off, base))
#define MIPS32_SWC1(isa, reg, off, base)	(isa ? MMIPS32_SWC1(reg, off, base) : MIPS32_ISA_SWC1(reg, off, base))
#define MIPS32_SDC1(isa, reg, off, base)	(isa ? MMIPS32_SDC1(reg, off, base) : MIPS32_ISA_SDC1(reg, off, base))

#define MIPS32_SLL(isa, dst, src, sa)		(isa ? MMIPS32_SLL(dst, src, sa) : MIPS32_ISA_SLL(dst, src, sa))
#define MIPS32_EHB(isa)		                (isa ? MMIPS32_SLL(0, 0, 3) : MIPS32_ISA_SLL(0, 0, 3))
#define MIPS32_SLLV(isa, dst, src, sa)		(MIPS32_ISA_SLLV(dst, src, sa))
#define MIPS32_SLTI(isa, tar, src, val)		(isa ? MMIPS32_SLTI(tar, src, val) : MIPS32_ISA_SLTI(tar, src, val))
#define MIPS32_SLTU(isa, dst, src, tar)		(isa ? MMIPS32_SLTU(dst, src, tar) : MIPS32_ISA_SLTU(dst, src, tar))
#define MIPS32_SRL(isa, reg, src, off)		(isa ? MMIPS32_SRL(reg, src, off) : MIPS32_ISA_SRL(reg, src, off))

#define MIPS32_SYNCI(isa, off, base)		(isa ? MMIPS32_SYNCI(off, base) : MIPS32_ISA_SYNCI(off, base))
#define MIPS32_SYNC(isa)			(isa ? MMIPS32_SYNC : MIPS32_ISA_SYNC)
#define MIPS32_XOR(isa, reg, val1, val2)	(isa ? MMIPS32_XOR(reg, val1, val2) : MIPS32_ISA_XOR(reg, val1, val2))
#define MIPS32_XORI(isa, tar, src, val)		(isa ? MMIPS32_XORI(tar, src, val) : MIPS32_ISA_XORI(tar, src, val))

#define MIPS32_SYNCI_STEP			0x1

/* ejtag specific instructions */
#define MIPS32_DRET(isa)			(isa ? MMIPS32_DRET : MIPS32_ISA_DRET)
#define MIPS32_SDBBP(isa)			(isa ? MMIPS32_SDBBP : MIPS32_ISA_SDBBP)

#define MIPS16_SDBBP(isa)			(isa ? MMIPS16_SDBBP : MIPS16_ISA_SDBBP)

/* ejtag specific instructions */
#define MICRO_MIPS32_SDBBP			0x000046C0
#define MICRO_MIPS_SDBBP			0x46C0
#define MIPS32_DSP_ENABLE			0x1000000

#define MIPS32_S_INST(rs, rac, opcode)			\
	(((rs) << 21) | ((rac) << 11) | (opcode))

#define MIPS32_DSP_R_INST(rt, immd, opcode, extrw) \
	((0x1F << 26) | ((immd) << 16) | ((rt) << 11) | ((opcode) << 6) | (extrw))
#define MIPS32_DSP_W_INST(rs, immd, opcode, extrw) \
	((0x1F << 26) | ((rs) << 21) | ((immd) << 11) | ((opcode) << 6) | (extrw))

#define MIPS32_DSP_MFHI(reg, ac)		MIPS32_R_INST(0, ac, 0, reg, 0, MIPS32_OP_MFHI)
#define MIPS32_DSP_MFLO(reg, ac)	MIPS32_R_INST(0, ac, 0, reg, 0, MIPS32_OP_MFLO)
#define MIPS32_DSP_MTLO(reg, ac)	MIPS32_S_INST(reg, ac, MIPS32_OP_MTLO)
#define MIPS32_DSP_MTHI(reg, ac)	MIPS32_S_INST(reg, ac, MIPS32_OP_MTHI)
#define MIPS32_DSP_RDDSP(rt, mask)	MIPS32_DSP_R_INST(rt, mask, 0x12, 0x38)
#define MIPS32_DSP_WRDSP(rs, mask)	MIPS32_DSP_W_INST(rs, mask, 0x13, 0x38)


/*
 * MIPS32 Config1 Register (CP0 Register 16, Select 1)
 */
#define MIPS32_CFG1_M			0x80000000		/* Config2 implemented */
#define MIPS32_CFG1_MMUSMASK		0x7e000000		/* mmu size - 1 */
#define MIPS32_CFG1_MMUSSHIFT		25
#define MIPS32_CFG1_ISMASK		0x01c00000		/* icache lines 64<<n */
#define MIPS32_CFG1_ISSHIFT		22
#define MIPS32_CFG1_ILMASK		0x00380000		/* icache line size 2<<n */
#define MIPS32_CFG1_ILSHIFT		19
#define MIPS32_CFG1_IAMASK		0x00070000		/* icache ways - 1 */
#define MIPS32_CFG1_IASHIFT		16
#define MIPS32_CFG1_DSMASK		0x0000e000		/* dcache lines 64<<n */
#define MIPS32_CFG1_DSSHIFT		13
#define MIPS32_CFG1_DLMASK		0x00001c00		/* dcache line size 2<<n */
#define MIPS32_CFG1_DLSHIFT		10
#define MIPS32_CFG1_DAMASK		0x00000380		/* dcache ways - 1 */
#define MIPS32_CFG1_DASHIFT		7
#define MIPS32_CFG1_C2			0x00000040		/* Coprocessor 2 present */
#define MIPS32_CFG1_MD			0x00000020		/* MDMX implemented */
#define MIPS32_CFG1_PC			0x00000010		/* performance counters implemented */
#define MIPS32_CFG1_WR			0x00000008		/* watch registers implemented */
#define MIPS32_CFG1_CA			0x00000004		/* compression (mips16) implemented */
#define MIPS32_CFG1_EP			0x00000002		/* ejtag implemented */
#define MIPS32_CFG1_FP			0x00000001		/* fpu implemented */

/*
 * MIPS32 Coprocessor 0 register numbers
 */
#define MIPS32_C0_INDEX			0
#define MIPS32_C0_INX			0
#define MIPS32_C0_RANDOM		1
#define MIPS32_C0_RAND			1
#define MIPS32_C0_ENTRYLO0		2
#define MIPS32_C0_TLBLO0		2
#define MIPS32_C0_ENTRYLO1		3
#define MIPS32_C0_TLBLO1		3
#define MIPS32_C0_CONTEXT		4
#define MIPS32_C0_CTXT			4
#define MIPS32_C0_PAGEMASK		5
#define MIPS32_C0_PAGEGRAIN		(5, 1)
#define MIPS32_C0_WIRED			6
#define MIPS32_C0_HWRENA		7
#define MIPS32_C0_BADVADDR		8
#define MIPS32_C0_VADDR			8
#define MIPS32_C0_COUNT			9
#define MIPS32_C0_ENTRYHI		10
#define MIPS32_C0_TLBHI			10
#define MIPS32_C0_GUESTCTL1		10
#define MIPS32_C0_COMPARE		11
#define MIPS32_C0_STATUS		12
#define MIPS32_C0_SR			12
#define MIPS32_C0_INTCTL		(12, 1)
#define MIPS32_C0_SRSCTL		(12, 2)
#define MIPS32_C0_SRSMAP		(12, 3)
#define MIPS32_C0_CAUSE			13
#define MIPS32_C0_CR			13
#define MIPS32_C0_EPC			14
#define MIPS32_C0_PRID			15
#define MIPS32_C0_EBASE			(15, 1)
#define MIPS32_C0_CONFIG		16
#define MIPS32_C0_CONFIG0		(16, 0)
#define MIPS32_C0_CONFIG1		(16, 1)
#define MIPS32_C0_CONFIG2		(16, 2)
#define MIPS32_C0_CONFIG3		(16, 3)
#define MIPS32_C0_LLADDR		17
#define MIPS32_C0_WATCHLO		18
#define MIPS32_C0_WATCHHI		19
#define MIPS32_C0_DEBUG			23
#define MIPS32_C0_DEPC			24
#define MIPS32_C0_PERFCNT		25
#define MIPS32_C0_ERRCTL		26
#define MIPS32_C0_CACHEERR		27
#define MIPS32_C0_TAGLO			28
#define MIPS32_C0_ITAGLO		28
#define MIPS32_C0_DTAGLO		(28, 2)
#define MIPS32_C0_TAGLO2		(28, 4)
#define MIPS32_C0_DATALO		(28, 1)
#define MIPS32_C0_IDATALO		(28, 1)
#define MIPS32_C0_DDATALO		(28, 3)
#define MIPS32_C0_DATALO2		(28, 5)
#define MIPS32_C0_TAGHI			29
#define MIPS32_C0_ITAGHI		29
#define MIPS32_C0_DATAHI		(29, 1)
#define MIPS32_C0_ERRPC			30
#define MIPS32_C0_DESAVE		31

/*
 * MIPS32 MMU types
 */
#define MIPS32_MMU_TLB			1
#define MIPS32_MMU_BAT			2
#define MIPS32_MMU_FIXED		3
#define MIPS32_MMU_DUAL_VTLB_FTLB	4

extern const struct command_registration mips32_command_handlers[];

int mips32_arch_state(struct target *target);

int mips32_init_arch_info(struct target *target,
		struct mips32_common *mips32, struct jtag_tap *tap);

int mips32_restore_context(struct target *target);
int mips32_save_context(struct target *target);

struct reg_cache *mips32_build_reg_cache(struct target *target);

int mips32_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t entry_point, target_addr_t exit_point,
		unsigned int timeout_ms, void *arch_info);

int mips32_configure_break_unit(struct target *target);

int mips32_enable_interrupts(struct target *target, int enable);

int mips32_examine(struct target *target);

int mips32_cpu_probe(struct target *target);

int mips32_read_config_regs(struct target *target);

int mips32_register_commands(struct command_context *cmd_ctx);

int mips32_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);
int mips32_checksum_memory(struct target *target, target_addr_t address,
		uint32_t count, uint32_t *checksum);
int mips32_blank_check_memory(struct target *target,
		struct target_memory_check_block *blocks, int num_blocks, uint8_t erased_value);

bool mips32_cpu_support_sync(struct mips_ejtag *ejtag_info);
bool mips32_cpu_support_hazard_barrier(struct mips_ejtag *ejtag_info);

#endif /* OPENOCD_TARGET_MIPS32_H */

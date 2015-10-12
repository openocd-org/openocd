/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   kesmtp@freenet.de                                                     *
 *                                                                         *
 *   Copyright (C) 2011 sleep(5) ltd                                       *
 *   tomas@sleepfive.com                                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Christopher D. Kilgour                          *
 *   techie at whiterocker.com                                             *
 *                                                                         *
 *   Copyright (C) 2013 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   Copyright (C) 2015 Tomas Vanek                                        *
 *   vanekt@fbl.cz                                                         *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/target_type.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

/*
 * Implementation Notes
 *
 * The persistent memories in the Kinetis chip families K10 through
 * K70 are all manipulated with the Flash Memory Module.  Some
 * variants call this module the FTFE, others call it the FTFL.  To
 * indicate that both are considered here, we use FTFX.
 *
 * Within the module, according to the chip variant, the persistent
 * memory is divided into what Freescale terms Program Flash, FlexNVM,
 * and FlexRAM.  All chip variants have Program Flash.  Some chip
 * variants also have FlexNVM and FlexRAM, which always appear
 * together.
 *
 * A given Kinetis chip may have 1, 2 or 4 blocks of flash.  Here we map
 * each block to a separate bank.  Each block size varies by chip and
 * may be determined by the read-only SIM_FCFG1 register.  The sector
 * size within each bank/block varies by chip, and may be 1, 2 or 4k.
 * The sector size may be different for flash and FlexNVM.
 *
 * The first half of the flash (1 or 2 blocks) is always Program Flash
 * and always starts at address 0x00000000.  The "PFLSH" flag, bit 23
 * of the read-only SIM_FCFG2 register, determines whether the second
 * half of the flash is also Program Flash or FlexNVM+FlexRAM.  When
 * PFLSH is set, the second from the first half.  When PFLSH is clear,
 * the second half of flash is FlexNVM and always starts at address
 * 0x10000000.  FlexRAM, which is also present when PFLSH is clear,
 * always starts at address 0x14000000.
 *
 * The Flash Memory Module provides a register set where flash
 * commands are loaded to perform flash operations like erase and
 * program.  Different commands are available depending on whether
 * Program Flash or FlexNVM/FlexRAM is being manipulated.  Although
 * the commands used are quite consistent between flash blocks, the
 * parameters they accept differ according to the flash sector size.
 *
 */

/* Addressess */
#define FLEXRAM		0x14000000

#define FMC_PFB01CR	0x4001f004
#define FTFx_FSTAT	0x40020000
#define FTFx_FCNFG	0x40020001
#define FTFx_FCCOB3	0x40020004
#define FTFx_FPROT3	0x40020010
#define FTFx_FDPROT	0x40020017
#define SIM_SDID	0x40048024
#define SIM_SOPT1	0x40047000
#define SIM_FCFG1	0x4004804c
#define SIM_FCFG2	0x40048050
#define WDOG_STCTRH	0x40052000
#define SMC_PMCTRL	0x4007E001
#define SMC_PMSTAT	0x4007E003

/* Values */
#define PM_STAT_RUN		0x01
#define PM_STAT_VLPR		0x04
#define PM_CTRL_RUNM_RUN	0x00

/* Commands */
#define FTFx_CMD_BLOCKSTAT  0x00
#define FTFx_CMD_SECTSTAT   0x01
#define FTFx_CMD_LWORDPROG  0x06
#define FTFx_CMD_SECTERASE  0x09
#define FTFx_CMD_SECTWRITE  0x0b
#define FTFx_CMD_MASSERASE  0x44
#define FTFx_CMD_PGMPART    0x80
#define FTFx_CMD_SETFLEXRAM 0x81

/* The older Kinetis K series uses the following SDID layout :
 * Bit 31-16 : 0
 * Bit 15-12 : REVID
 * Bit 11-7  : DIEID
 * Bit 6-4   : FAMID
 * Bit 3-0   : PINID
 *
 * The newer Kinetis series uses the following SDID layout :
 * Bit 31-28 : FAMID
 * Bit 27-24 : SUBFAMID
 * Bit 23-20 : SERIESID
 * Bit 19-16 : SRAMSIZE
 * Bit 15-12 : REVID
 * Bit 6-4   : Reserved (0)
 * Bit 3-0   : PINID
 *
 * We assume that if bits 31-16 are 0 then it's an older
 * K-series MCU.
 */

#define KINETIS_SOPT1_RAMSIZE_MASK  0x0000F000
#define KINETIS_SOPT1_RAMSIZE_K24FN1M 0x0000B000

#define KINETIS_SDID_K_SERIES_MASK  0x0000FFFF

#define KINETIS_SDID_DIEID_MASK 0x00000F80

#define KINETIS_SDID_DIEID_K22FN128	0x00000680 /* smaller pflash with FTFA */
#define KINETIS_SDID_DIEID_K22FN256	0x00000A80
#define KINETIS_SDID_DIEID_K22FN512	0x00000E80
#define KINETIS_SDID_DIEID_K24FN256	0x00000700

#define KINETIS_SDID_DIEID_K24FN1M	0x00000300 /* Detect Errata 7534 */

/* We can't rely solely on the FAMID field to determine the MCU
 * type since some FAMID values identify multiple MCUs with
 * different flash sector sizes (K20 and K22 for instance).
 * Therefore we combine it with the DIEID bits which may possibly
 * break if Freescale bumps the DIEID for a particular MCU. */
#define KINETIS_K_SDID_TYPE_MASK 0x00000FF0
#define KINETIS_K_SDID_K10_M50	 0x00000000
#define KINETIS_K_SDID_K10_M72	 0x00000080
#define KINETIS_K_SDID_K10_M100	 0x00000100
#define KINETIS_K_SDID_K10_M120	 0x00000180
#define KINETIS_K_SDID_K11		 0x00000220
#define KINETIS_K_SDID_K12		 0x00000200
#define KINETIS_K_SDID_K20_M50	 0x00000010
#define KINETIS_K_SDID_K20_M72	 0x00000090
#define KINETIS_K_SDID_K20_M100	 0x00000110
#define KINETIS_K_SDID_K20_M120	 0x00000190
#define KINETIS_K_SDID_K21_M50   0x00000230
#define KINETIS_K_SDID_K21_M120	 0x00000330
#define KINETIS_K_SDID_K22_M50   0x00000210
#define KINETIS_K_SDID_K22_M120	 0x00000310
#define KINETIS_K_SDID_K30_M72   0x000000A0
#define KINETIS_K_SDID_K30_M100  0x00000120
#define KINETIS_K_SDID_K40_M72   0x000000B0
#define KINETIS_K_SDID_K40_M100  0x00000130
#define KINETIS_K_SDID_K50_M72   0x000000E0
#define KINETIS_K_SDID_K51_M72	 0x000000F0
#define KINETIS_K_SDID_K53		 0x00000170
#define KINETIS_K_SDID_K60_M100  0x00000140
#define KINETIS_K_SDID_K60_M150  0x000001C0
#define KINETIS_K_SDID_K70_M150  0x000001D0

#define KINETIS_SDID_SERIESID_MASK 0x00F00000
#define KINETIS_SDID_SERIESID_K   0x00000000
#define KINETIS_SDID_SERIESID_KL   0x00100000
#define KINETIS_SDID_SERIESID_KW   0x00500000
#define KINETIS_SDID_SERIESID_KV   0x00600000

#define KINETIS_SDID_SUBFAMID_MASK  0x0F000000
#define KINETIS_SDID_SUBFAMID_KX0   0x00000000
#define KINETIS_SDID_SUBFAMID_KX1   0x01000000
#define KINETIS_SDID_SUBFAMID_KX2   0x02000000
#define KINETIS_SDID_SUBFAMID_KX3   0x03000000
#define KINETIS_SDID_SUBFAMID_KX4   0x04000000
#define KINETIS_SDID_SUBFAMID_KX5   0x05000000
#define KINETIS_SDID_SUBFAMID_KX6   0x06000000

#define KINETIS_SDID_FAMILYID_MASK  0xF0000000
#define KINETIS_SDID_FAMILYID_K0X   0x00000000
#define KINETIS_SDID_FAMILYID_K1X   0x10000000
#define KINETIS_SDID_FAMILYID_K2X   0x20000000
#define KINETIS_SDID_FAMILYID_K3X   0x30000000
#define KINETIS_SDID_FAMILYID_K4X   0x40000000
#define KINETIS_SDID_FAMILYID_K6X   0x60000000
#define KINETIS_SDID_FAMILYID_K7X   0x70000000

struct kinetis_flash_bank {
	bool probed;
	uint32_t sector_size;
	uint32_t max_flash_prog_size;
	uint32_t protection_size;
	uint32_t prog_base;		/* base address for FTFx operations */
					/* same as bank->base for pflash, differs for FlexNVM */
	uint32_t protection_block;	/* number of first protection block in this bank */

	uint32_t sim_sdid;
	uint32_t sim_fcfg1;
	uint32_t sim_fcfg2;

	enum {
		FC_AUTO = 0,
		FC_PFLASH,
		FC_FLEX_NVM,
		FC_FLEX_RAM,
	} flash_class;

	enum {
		FS_PROGRAM_SECTOR = 1,
		FS_PROGRAM_LONGWORD = 2,
		FS_PROGRAM_PHRASE = 4, /* Unsupported */
		FS_INVALIDATE_CACHE = 8,
	} flash_support;
};

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

#define MDM_ACCESS_TIMEOUT	3000 /* iterations */

static int kinetis_mdm_write_register(struct adiv5_dap *dap, unsigned reg, uint32_t value)
{
	int retval;
	LOG_DEBUG("MDM_REG[0x%02x] <- %08" PRIX32, reg, value);

	retval = dap_queue_ap_write(dap_ap(dap, 1), reg, value);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: failed to queue a write request");
		return retval;
	}

	retval = dap_run(dap);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: dap_run failed");
		return retval;
	}


	return ERROR_OK;
}

static int kinetis_mdm_read_register(struct adiv5_dap *dap, unsigned reg, uint32_t *result)
{
	int retval;

	retval = dap_queue_ap_read(dap_ap(dap, 1), reg, result);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: failed to queue a read request");
		return retval;
	}

	retval = dap_run(dap);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: dap_run failed");
		return retval;
	}

	LOG_DEBUG("MDM_REG[0x%02x]: %08" PRIX32, reg, *result);
	return ERROR_OK;
}

static int kinetis_mdm_poll_register(struct adiv5_dap *dap, unsigned reg, uint32_t mask, uint32_t value)
{
	uint32_t val;
	int retval;
	int timeout = MDM_ACCESS_TIMEOUT;

	do {
		retval = kinetis_mdm_read_register(dap, reg, &val);
		if (retval != ERROR_OK || (val & mask) == value)
			return retval;

		alive_sleep(1);
	} while (timeout--);

	LOG_DEBUG("MDM: polling timed out");
	return ERROR_FAIL;
}

/*
 * This function implements the procedure to mass erase the flash via
 * SWD/JTAG on Kinetis K and L series of devices as it is described in
 * AN4835 "Production Flash Programming Best Practices for Kinetis K-
 * and L-series MCUs" Section 4.2.1
 */
COMMAND_HANDLER(kinetis_mdm_mass_erase)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct adiv5_dap *dap = cortex_m->armv7m.arm.dap;

	if (!dap) {
		LOG_ERROR("Cannot perform mass erase with a high-level adapter");
		return ERROR_FAIL;
	}

	int retval;

	/*
	 * ... Power on the processor, or if power has already been
	 * applied, assert the RESET pin to reset the processor. For
	 * devices that do not have a RESET pin, write the System
	 * Reset Request bit in the MDM-AP control register after
	 * establishing communication...
	 */

	/* assert SRST */
	if (jtag_get_reset_config() & RESET_HAS_SRST)
		adapter_assert_reset();
	else
		LOG_WARNING("Attempting mass erase without hardware reset. This is not reliable; "
			    "it's recommended you connect SRST and use ``reset_config srst_only''.");

	retval = kinetis_mdm_write_register(dap, MDM_REG_CTRL, MEM_CTRL_SYS_RES_REQ);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * ... Read the MDM-AP status register until the Flash Ready bit sets...
	 */
	retval = kinetis_mdm_poll_register(dap, MDM_REG_STAT,
					   MDM_STAT_FREADY | MDM_STAT_SYSRES,
					   MDM_STAT_FREADY);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM : flash ready timeout");
		return retval;
	}

	/*
	 * ... Write the MDM-AP control register to set the Flash Mass
	 * Erase in Progress bit. This will start the mass erase
	 * process...
	 */
	retval = kinetis_mdm_write_register(dap, MDM_REG_CTRL,
					    MEM_CTRL_SYS_RES_REQ | MEM_CTRL_FMEIP);
	if (retval != ERROR_OK)
		return retval;

	/* As a sanity check make sure that device started mass erase procedure */
	retval = kinetis_mdm_poll_register(dap, MDM_REG_STAT,
					   MDM_STAT_FMEACK, MDM_STAT_FMEACK);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * ... Read the MDM-AP control register until the Flash Mass
	 * Erase in Progress bit clears...
	 */
	retval = kinetis_mdm_poll_register(dap, MDM_REG_CTRL,
					   MEM_CTRL_FMEIP,
					   0);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * ... Negate the RESET signal or clear the System Reset Request
	 * bit in the MDM-AP control register...
	 */
	retval = kinetis_mdm_write_register(dap, MDM_REG_CTRL, 0);
	if (retval != ERROR_OK)
		return retval;

	if (jtag_get_reset_config() & RESET_HAS_SRST) {
		/* halt MCU otherwise it loops in hard fault - WDOG reset cycle */
		target->reset_halt = true;
		target->type->assert_reset(target);
		target->type->deassert_reset(target);
	}

	return ERROR_OK;
}

static const uint32_t kinetis_known_mdm_ids[] = {
	0x001C0000,	/* Kinetis-K Series */
	0x001C0020,	/* Kinetis-L/M/V/E Series */
};

/*
 * This function implements the procedure to connect to
 * SWD/JTAG on Kinetis K and L series of devices as it is described in
 * AN4835 "Production Flash Programming Best Practices for Kinetis K-
 * and L-series MCUs" Section 4.1.1
 */
COMMAND_HANDLER(kinetis_check_flash_security_status)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct adiv5_dap *dap = cortex_m->armv7m.arm.dap;

	if (!dap) {
		LOG_WARNING("Cannot check flash security status with a high-level adapter");
		return ERROR_OK;
	}

	uint32_t val;
	int retval;

	/*
	 * ... The MDM-AP ID register can be read to verify that the
	 * connection is working correctly...
	 */
	retval = kinetis_mdm_read_register(dap, MDM_REG_ID, &val);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM: failed to read ID register");
		goto fail;
	}

	bool found = false;
	for (size_t i = 0; i < ARRAY_SIZE(kinetis_known_mdm_ids); i++) {
		if (val == kinetis_known_mdm_ids[i]) {
			found = true;
			break;
		}
	}

	if (!found)
		LOG_WARNING("MDM: unknown ID %08" PRIX32, val);

	/*
	 * ... Read the MDM-AP status register until the Flash Ready bit sets...
	 */
	retval = kinetis_mdm_poll_register(dap, MDM_REG_STAT,
					   MDM_STAT_FREADY,
					   MDM_STAT_FREADY);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM: flash ready timeout");
		goto fail;
	}

	/*
	 * ... Read the System Security bit to determine if security is enabled.
	 * If System Security = 0, then proceed. If System Security = 1, then
	 * communication with the internals of the processor, including the
	 * flash, will not be possible without issuing a mass erase command or
	 * unsecuring the part through other means (backdoor key unlock)...
	 */
	retval = kinetis_mdm_read_register(dap, MDM_REG_STAT, &val);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM: failed to read MDM_REG_STAT");
		goto fail;
	}

	if ((val & (MDM_STAT_SYSSEC | MDM_STAT_CORE_HALTED)) == MDM_STAT_SYSSEC) {
		LOG_WARNING("MDM: Secured MCU state detected however it may be a false alarm");
		LOG_WARNING("MDM: Halting target to detect secured state reliably");

		retval = target_halt(target);
		if (retval == ERROR_OK)
			retval = target_wait_state(target, TARGET_HALTED, 100);

		if (retval != ERROR_OK) {
			LOG_WARNING("MDM: Target not halted, trying reset halt");
			target->reset_halt = true;
			target->type->assert_reset(target);
			target->type->deassert_reset(target);
		}

		/* re-read status */
		retval = kinetis_mdm_read_register(dap, MDM_REG_STAT, &val);
		if (retval != ERROR_OK) {
			LOG_ERROR("MDM: failed to read MDM_REG_STAT");
			goto fail;
		}
	}

	if (val & MDM_STAT_SYSSEC) {
		jtag_poll_set_enabled(false);

		LOG_WARNING("*********** ATTENTION! ATTENTION! ATTENTION! ATTENTION! **********");
		LOG_WARNING("****                                                          ****");
		LOG_WARNING("**** Your Kinetis MCU is in secured state, which means that,  ****");
		LOG_WARNING("**** with exception for very basic communication, JTAG/SWD    ****");
		LOG_WARNING("**** interface will NOT work. In order to restore its         ****");
		LOG_WARNING("**** functionality please issue 'kinetis mdm mass_erase'      ****");
		LOG_WARNING("**** command, power cycle the MCU and restart OpenOCD.        ****");
		LOG_WARNING("****                                                          ****");
		LOG_WARNING("*********** ATTENTION! ATTENTION! ATTENTION! ATTENTION! **********");
	} else {
		LOG_INFO("MDM: Chip is unsecured. Continuing.");
		jtag_poll_set_enabled(true);
	}

	return ERROR_OK;

fail:
	LOG_ERROR("MDM: Failed to check security status of the MCU. Cannot proceed further");
	jtag_poll_set_enabled(false);
	return retval;
}

FLASH_BANK_COMMAND_HANDLER(kinetis_flash_bank_command)
{
	struct kinetis_flash_bank *bank_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_INFO("add flash_bank kinetis %s", bank->name);

	bank_info = malloc(sizeof(struct kinetis_flash_bank));

	memset(bank_info, 0, sizeof(struct kinetis_flash_bank));

	bank->driver_priv = bank_info;

	return ERROR_OK;
}

/* Disable the watchdog on Kinetis devices */
int kinetis_disable_wdog(struct target *target, uint32_t sim_sdid)
{
	struct working_area *wdog_algorithm;
	struct armv7m_algorithm armv7m_info;
	uint16_t wdog;
	int retval;

	static const uint8_t kinetis_unlock_wdog_code[] = {
		/* WDOG_UNLOCK = 0xC520 */
		0x4f, 0xf4, 0x00, 0x53,    /* mov.w   r3, #8192     ; 0x2000  */
		0xc4, 0xf2, 0x05, 0x03,    /* movt    r3, #16389    ; 0x4005  */
		0x4c, 0xf2, 0x20, 0x52,   /* movw    r2, #50464    ; 0xc520  */
		0xda, 0x81,               /* strh    r2, [r3, #14]  */

		/* WDOG_UNLOCK = 0xD928 */
		0x4f, 0xf4, 0x00, 0x53,   /* mov.w   r3, #8192     ; 0x2000  */
		0xc4, 0xf2, 0x05, 0x03,   /* movt    r3, #16389    ; 0x4005  */
		0x4d, 0xf6, 0x28, 0x12,   /* movw    r2, #55592    ; 0xd928  */
		0xda, 0x81,               /* strh    r2, [r3, #14]  */

		/* WDOG_SCR = 0x1d2 */
		0x4f, 0xf4, 0x00, 0x53,   /* mov.w   r3, #8192     ; 0x2000  */
		0xc4, 0xf2, 0x05, 0x03,   /* movt    r3, #16389    ; 0x4005  */
		0x4f, 0xf4, 0xe9, 0x72,   /* mov.w   r2, #466      ; 0x1d2  */
		0x1a, 0x80,               /* strh    r2, [r3, #0]  */

		/* END */
		0x00, 0xBE,               /* bkpt #0 */
	};

	/* Decide whether the connected device needs watchdog disabling.
	 * Disable for all Kx devices, i.e., return if it is a KLx */

	if ((sim_sdid & KINETIS_SDID_SERIESID_MASK) == KINETIS_SDID_SERIESID_KL)
		return ERROR_OK;

	/* The connected device requires watchdog disabling. */
	retval = target_read_u16(target, WDOG_STCTRH, &wdog);
	if (retval != ERROR_OK)
		return retval;

	if ((wdog & 0x1) == 0) {
		/* watchdog already disabled */
		return ERROR_OK;
	}
	LOG_INFO("Disabling Kinetis watchdog (initial WDOG_STCTRLH = 0x%x)", wdog);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = target_alloc_working_area(target, sizeof(kinetis_unlock_wdog_code), &wdog_algorithm);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_buffer(target, wdog_algorithm->address,
			sizeof(kinetis_unlock_wdog_code), (uint8_t *)kinetis_unlock_wdog_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, wdog_algorithm);
		return retval;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_algorithm(target, 0, NULL, 0, NULL, wdog_algorithm->address,
			wdog_algorithm->address + (sizeof(kinetis_unlock_wdog_code) - 2),
			10000, &armv7m_info);

	if (retval != ERROR_OK)
		LOG_ERROR("error executing kinetis wdog unlock algorithm");

	retval = target_read_u16(target, WDOG_STCTRH, &wdog);
	if (retval != ERROR_OK)
		return retval;
	LOG_INFO("WDOG_STCTRLH = 0x%x", wdog);

	target_free_working_area(target, wdog_algorithm);

	return retval;
}

COMMAND_HANDLER(kinetis_disable_wdog_handler)
{
	int result;
	uint32_t sim_sdid;
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	result = target_read_u32(target, SIM_SDID, &sim_sdid);
	if (result != ERROR_OK) {
		LOG_ERROR("Failed to read SIMSDID");
		return result;
	}

	result = kinetis_disable_wdog(target, sim_sdid);
	return result;
}


/* Kinetis Program-LongWord Microcodes */
static const uint8_t kinetis_flash_write_code[] = {
	/* Params:
	 * r0 - workarea buffer
	* r1 - target address
	* r2 - wordcount
	* Clobbered:
	* r4 - tmp
	* r5 - tmp
	* r6 - tmp
	* r7 - tmp
	*/

							/* .L1: */
						/* for(register uint32_t i=0;i<wcount;i++){ */
	0x04, 0x1C,					/* mov    r4, r0          */
	0x00, 0x23,					/* mov    r3, #0          */
							/* .L2: */
	0x0E, 0x1A,					/* sub    r6, r1, r0      */
	0xA6, 0x19,					/* add    r6, r4, r6      */
	0x93, 0x42,					/* cmp    r3, r2          */
	0x16, 0xD0,					/* beq    .L9             */
							/* .L5: */
						/* while((FTFx_FSTAT&FTFA_FSTAT_CCIF_MASK) != FTFA_FSTAT_CCIF_MASK){}; */
	0x0B, 0x4D,					/* ldr    r5, .L10        */
	0x2F, 0x78,					/* ldrb   r7, [r5]        */
	0x7F, 0xB2,					/* sxtb   r7, r7          */
	0x00, 0x2F,					/* cmp    r7, #0          */
	0xFA, 0xDA,					/* bge    .L5             */
						/* FTFx_FSTAT = FTFA_FSTAT_ACCERR_MASK|FTFA_FSTAT_FPVIOL_MASK|FTFA_FSTAT_RDCO */
	0x70, 0x27,					/* mov    r7, #112        */
	0x2F, 0x70,					/* strb   r7, [r5]        */
						/* FTFx_FCCOB3 = faddr; */
	0x09, 0x4F,					/* ldr    r7, .L10+4      */
	0x3E, 0x60,					/* str    r6, [r7]        */
	0x06, 0x27,					/* mov    r7, #6          */
						/* FTFx_FCCOB0 = 0x06;  */
	0x08, 0x4E,					/* ldr    r6, .L10+8      */
	0x37, 0x70,					/* strb   r7, [r6]        */
						/* FTFx_FCCOB7 = *pLW;  */
	0x80, 0xCC,					/* ldmia  r4!, {r7}       */
	0x08, 0x4E,					/* ldr    r6, .L10+12     */
	0x37, 0x60,					/* str    r7, [r6]        */
						/* FTFx_FSTAT = FTFA_FSTAT_CCIF_MASK; */
	0x80, 0x27,					/* mov    r7, #128        */
	0x2F, 0x70,					/* strb   r7, [r5]        */
							/* .L4: */
						/* while((FTFx_FSTAT&FTFA_FSTAT_CCIF_MASK) != FTFA_FSTAT_CCIF_MASK){}; */
	0x2E, 0x78,					/* ldrb    r6, [r5]       */
	0x77, 0xB2,					/* sxtb    r7, r6         */
	0x00, 0x2F,					/* cmp     r7, #0         */
	0xFB, 0xDA,					/* bge     .L4            */
	0x01, 0x33,					/* add     r3, r3, #1     */
	0xE4, 0xE7,					/* b       .L2            */
							/* .L9: */
	0x00, 0xBE,					/* bkpt #0                */
							/* .L10: */
	0x00, 0x00, 0x02, 0x40,		/* .word    1073872896    */
	0x04, 0x00, 0x02, 0x40,		/* .word    1073872900    */
	0x07, 0x00, 0x02, 0x40,		/* .word    1073872903    */
	0x08, 0x00, 0x02, 0x40,		/* .word    1073872904    */
};

/* Program LongWord Block Write */
static int kinetis_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t wcount)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 2048;		/* Default minimum value */
	struct working_area *write_algorithm;
	struct working_area *source;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;
	uint32_t address = kinfo->prog_base + offset;
	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* Params:
	 * r0 - workarea buffer
	 * r1 - target address
	 * r2 - wordcount
	 * Clobbered:
	 * r4 - tmp
	 * r5 - tmp
	 * r6 - tmp
	 * r7 - tmp
	 */

	/* Increase buffer_size if needed */
	if (buffer_size < (target->working_area_size/2))
		buffer_size = (target->working_area_size/2);

	LOG_INFO("Kinetis: FLASH Write ...");

	/* check code alignment */
	if (offset & 0x1) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(kinetis_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
		sizeof(kinetis_flash_write_code), kinetis_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 4;
		if (buffer_size <= 256) {
			/* free working area, write algorithm already allocated */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("No large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* *pLW (*buffer) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* faddr */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* number of words to program */

	/* write code buffer and use Flash programming code within kinetis       */
	/* Set breakpoint to 0 with time-out of 1000 ms                          */
	while (wcount > 0) {
		uint32_t thisrun_count = (wcount > (buffer_size / 4)) ? (buffer_size / 4) : wcount;

		retval = target_write_buffer(target, source->address, thisrun_count * 4, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);

		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
				write_algorithm->address, 0, 100000, &armv7m_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error executing kinetis Flash programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 4;
		address += thisrun_count * 4;
		wcount -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return retval;
}

static int kinetis_protect(struct flash_bank *bank, int set, int first, int last)
{
	LOG_WARNING("kinetis_protect not supported yet");
	/* FIXME: TODO */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_FLASH_BANK_INVALID;
}

static int kinetis_protect_check(struct flash_bank *bank)
{
	struct kinetis_flash_bank *kinfo = bank->driver_priv;
	int result;
	int i, b;
	uint32_t fprot, psec;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (kinfo->flash_class == FC_PFLASH) {
		uint8_t buffer[4];

		/* read protection register */
		result = target_read_memory(bank->target, FTFx_FPROT3, 1, 4, buffer);

		if (result != ERROR_OK)
			return result;

		fprot = target_buffer_get_u32(bank->target, buffer);
		/* Every bit protects 1/32 of the full flash (not necessarily just this bank) */

	} else if (kinfo->flash_class == FC_FLEX_NVM) {
		uint8_t fdprot;

		/* read protection register */
		result = target_read_memory(bank->target, FTFx_FDPROT, 1, 1, &fdprot);

		if (result != ERROR_OK)
			return result;

		fprot = fdprot;

	} else {
		LOG_ERROR("Protection checks for FlexRAM not supported");
		return ERROR_FLASH_BANK_INVALID;
	}

	b = kinfo->protection_block;
	for (psec = 0, i = 0; i < bank->num_sectors; i++) {
		if ((fprot >> b) & 1)
			bank->sectors[i].is_protected = 0;
		else
			bank->sectors[i].is_protected = 1;

		psec += bank->sectors[i].size;

		if (psec >= kinfo->protection_size) {
			psec = 0;
			b++;
		}
	}

	return ERROR_OK;
}

static int kinetis_ftfx_command(struct target *target, uint8_t fcmd, uint32_t faddr,
				uint8_t fccob4, uint8_t fccob5, uint8_t fccob6, uint8_t fccob7,
				uint8_t fccob8, uint8_t fccob9, uint8_t fccoba, uint8_t fccobb,
				uint8_t *ftfx_fstat)
{
	uint8_t command[12] = {faddr & 0xff, (faddr >> 8) & 0xff, (faddr >> 16) & 0xff, fcmd,
			fccob7, fccob6, fccob5, fccob4,
			fccobb, fccoba, fccob9, fccob8};
	int result, i;
	uint8_t buffer;

	/* wait for done */
	for (i = 0; i < 50; i++) {
		result =
			target_read_memory(target, FTFx_FSTAT, 1, 1, &buffer);

		if (result != ERROR_OK)
			return result;

		if (buffer & 0x80)
			break;

		buffer = 0x00;
	}

	if (buffer != 0x80) {
		/* reset error flags */
		buffer = 0x30;
		result =
			target_write_memory(target, FTFx_FSTAT, 1, 1, &buffer);
		if (result != ERROR_OK)
			return result;
	}

	result = target_write_memory(target, FTFx_FCCOB3, 4, 3, command);

	if (result != ERROR_OK)
		return result;

	/* start command */
	buffer = 0x80;
	result = target_write_memory(target, FTFx_FSTAT, 1, 1, &buffer);
	if (result != ERROR_OK)
		return result;

	/* wait for done */
	for (i = 0; i < 240; i++) { /* Need longtime for "Mass Erase" Command Nemui Changed */
		result =
			target_read_memory(target, FTFx_FSTAT, 1, 1, ftfx_fstat);

		if (result != ERROR_OK)
			return result;

		if (*ftfx_fstat & 0x80)
			break;
	}

	if ((*ftfx_fstat & 0xf0) != 0x80) {
		LOG_ERROR
			("ftfx command failed FSTAT: %02X FCCOB: %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
			 *ftfx_fstat, command[3], command[2], command[1], command[0],
			 command[7], command[6], command[5], command[4],
			 command[11], command[10], command[9], command[8]);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}


static int kinetis_check_run_mode(struct target *target)
{
	int result, i;
	uint8_t pmctrl, pmstat;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = target_read_u8(target, SMC_PMSTAT, &pmstat);
	if (result != ERROR_OK)
		return result;

	if (pmstat == PM_STAT_RUN)
		return ERROR_OK;

	if (pmstat == PM_STAT_VLPR) {
		/* It is safe to switch from VLPR to RUN mode without changing clock */
		LOG_INFO("Switching from VLPR to RUN mode.");
		pmctrl = PM_CTRL_RUNM_RUN;
		result = target_write_u8(target, SMC_PMCTRL, pmctrl);
		if (result != ERROR_OK)
			return result;

		for (i = 100; i; i--) {
			result = target_read_u8(target, SMC_PMSTAT, &pmstat);
			if (result != ERROR_OK)
				return result;

			if (pmstat == PM_STAT_RUN)
				return ERROR_OK;
		}
	}

	LOG_ERROR("Flash operation not possible in current run mode: SMC_PMSTAT: 0x%x", pmstat);
	LOG_ERROR("Issue a 'reset init' command.");
	return ERROR_TARGET_NOT_HALTED;
}


static void kinetis_invalidate_flash_cache(struct flash_bank *bank)
{
	struct kinetis_flash_bank *kinfo = bank->driver_priv;
	uint8_t pfb01cr_byte2 = 0xf0;

	if (!(kinfo->flash_support & FS_INVALIDATE_CACHE))
		return;

	target_write_memory(bank->target, FMC_PFB01CR + 2, 1, 1, &pfb01cr_byte2);
	return;
}


static int kinetis_erase(struct flash_bank *bank, int first, int last)
{
	int result, i;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	result = kinetis_check_run_mode(bank->target);
	if (result != ERROR_OK)
		return result;

	if ((first > bank->num_sectors) || (last > bank->num_sectors))
		return ERROR_FLASH_OPERATION_FAILED;

	/*
	 * FIXME: TODO: use the 'Erase Flash Block' command if the
	 * requested erase is PFlash or NVM and encompasses the entire
	 * block.  Should be quicker.
	 */
	for (i = first; i <= last; i++) {
		uint8_t ftfx_fstat;
		/* set command and sector address */
		result = kinetis_ftfx_command(bank->target, FTFx_CMD_SECTERASE, kinfo->prog_base + bank->sectors[i].offset,
				0, 0, 0, 0,  0, 0, 0, 0,  &ftfx_fstat);

		if (result != ERROR_OK) {
			LOG_WARNING("erase sector %d failed", i);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bank->sectors[i].is_erased = 1;
	}

	kinetis_invalidate_flash_cache(bank);

	if (first == 0) {
		LOG_WARNING
			("flash configuration field erased, please reset the device");
	}

	return ERROR_OK;
}

static int kinetis_make_ram_ready(struct target *target)
{
	int result;
	uint8_t ftfx_fstat;
	uint8_t ftfx_fcnfg;

	/* check if ram ready */
	result = target_read_memory(target, FTFx_FCNFG, 1, 1, &ftfx_fcnfg);
	if (result != ERROR_OK)
		return result;

	if (ftfx_fcnfg & (1 << 1))
		return ERROR_OK;	/* ram ready */

	/* make flex ram available */
	result = kinetis_ftfx_command(target, FTFx_CMD_SETFLEXRAM, 0x00ff0000,
				 0, 0, 0, 0,  0, 0, 0, 0,  &ftfx_fstat);
	if (result != ERROR_OK)
		return ERROR_FLASH_OPERATION_FAILED;

	/* check again */
	result = target_read_memory(target, FTFx_FCNFG, 1, 1, &ftfx_fcnfg);
	if (result != ERROR_OK)
		return result;

	if (ftfx_fcnfg & (1 << 1))
		return ERROR_OK;	/* ram ready */

	return ERROR_FLASH_OPERATION_FAILED;
}

static int kinetis_write(struct flash_bank *bank, const uint8_t *buffer,
			 uint32_t offset, uint32_t count)
{
	unsigned int i, result, fallback = 0;
	uint32_t wc;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;
	uint8_t *new_buffer = NULL;

	result = kinetis_check_run_mode(bank->target);
	if (result != ERROR_OK)
		return result;

	if (!(kinfo->flash_support & FS_PROGRAM_SECTOR)) {
		/* fallback to longword write */
		fallback = 1;
		LOG_WARNING("This device supports Program Longword execution only.");
	} else {
		result = kinetis_make_ram_ready(bank->target);
		if (result != ERROR_OK) {
			fallback = 1;
			LOG_WARNING("FlexRAM not ready, fallback to slow longword write.");
		}
	}

	LOG_DEBUG("flash write @08%" PRIX32, offset);


	/* program section command */
	if (fallback == 0) {
		/*
		 * Kinetis uses different terms for the granularity of
		 * sector writes, e.g. "phrase" or "128 bits".  We use
		 * the generic term "chunk". The largest possible
		 * Kinetis "chunk" is 16 bytes (128 bits).
		 */
		unsigned prog_section_chunk_bytes = kinfo->sector_size >> 8;
		unsigned prog_size_bytes = kinfo->max_flash_prog_size;
		for (i = 0; i < count; i += prog_size_bytes) {
			uint8_t residual_buffer[16];
			uint8_t ftfx_fstat;
			uint32_t section_count = prog_size_bytes / prog_section_chunk_bytes;
			uint32_t residual_wc = 0;

			/*
			 * Assume the word count covers an entire
			 * sector.
			 */
			wc = prog_size_bytes / 4;

			/*
			 * If bytes to be programmed are less than the
			 * full sector, then determine the number of
			 * full-words to program, and put together the
			 * residual buffer so that a full "section"
			 * may always be programmed.
			 */
			if ((count - i) < prog_size_bytes) {
				/* number of bytes to program beyond full section */
				unsigned residual_bc = (count-i) % prog_section_chunk_bytes;

				/* number of complete words to copy directly from buffer */
				wc = (count - i - residual_bc) / 4;

				/* number of total sections to write, including residual */
				section_count = DIV_ROUND_UP((count-i), prog_section_chunk_bytes);

				/* any residual bytes delivers a whole residual section */
				residual_wc = (residual_bc ? prog_section_chunk_bytes : 0)/4;

				/* clear residual buffer then populate residual bytes */
				(void) memset(residual_buffer, 0xff, prog_section_chunk_bytes);
				(void) memcpy(residual_buffer, &buffer[i+4*wc], residual_bc);
			}

			LOG_DEBUG("write section @ %08" PRIX32 " with length %" PRIu32 " bytes",
				  offset + i, (uint32_t)wc*4);

			/* write data to flexram as whole-words */
			result = target_write_memory(bank->target, FLEXRAM, 4, wc,
					buffer + i);

			if (result != ERROR_OK) {
				LOG_ERROR("target_write_memory failed");
				return result;
			}

			/* write the residual words to the flexram */
			if (residual_wc) {
				result = target_write_memory(bank->target,
						FLEXRAM+4*wc,
						4, residual_wc,
						residual_buffer);

				if (result != ERROR_OK) {
					LOG_ERROR("target_write_memory failed");
					return result;
				}
			}

			/* execute section-write command */
			result = kinetis_ftfx_command(bank->target, FTFx_CMD_SECTWRITE, kinfo->prog_base + offset + i,
					section_count>>8, section_count, 0, 0,
					0, 0, 0, 0,  &ftfx_fstat);

			if (result != ERROR_OK)
				return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	/* program longword command, not supported in "SF3" devices */
	else if (kinfo->flash_support & FS_PROGRAM_LONGWORD) {
		if (count & 0x3) {
			uint32_t old_count = count;
			count = (old_count | 3) + 1;
			new_buffer = malloc(count);
			if (new_buffer == NULL) {
				LOG_ERROR("odd number of bytes to write and no memory "
					"for padding buffer");
				return ERROR_FAIL;
			}
			LOG_INFO("odd number of bytes to write (%" PRIu32 "), extending to %" PRIu32 " "
				"and padding with 0xff", old_count, count);
			memset(new_buffer, 0xff, count);
			buffer = memcpy(new_buffer, buffer, old_count);
		}

		uint32_t words_remaining = count / 4;

		kinetis_disable_wdog(bank->target, kinfo->sim_sdid);

		/* try using a block write */
		int retval = kinetis_write_block(bank, buffer, offset, words_remaining);

		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* if block write failed (no sufficient working area),
			 * we use normal (slow) single word accesses */
			LOG_WARNING("couldn't use block writes, falling back to single "
				"memory accesses");

			for (i = 0; i < count; i += 4) {
				uint8_t ftfx_fstat;

				LOG_DEBUG("write longword @ %08" PRIX32, (uint32_t)(offset + i));

				uint8_t padding[4] = {0xff, 0xff, 0xff, 0xff};
				memcpy(padding, buffer + i, MIN(4, count-i));

				result = kinetis_ftfx_command(bank->target, FTFx_CMD_LWORDPROG, kinfo->prog_base + offset + i,
						padding[3], padding[2], padding[1], padding[0],
						0, 0, 0, 0,  &ftfx_fstat);

				if (result != ERROR_OK)
					return ERROR_FLASH_OPERATION_FAILED;
			}
		}
	} else {
		LOG_ERROR("Flash write strategy not implemented");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	kinetis_invalidate_flash_cache(bank);
	return ERROR_OK;
}

static int kinetis_read_part_info(struct flash_bank *bank)
{
	int result, i;
	uint32_t offset = 0;
	uint8_t fcfg1_nvmsize, fcfg1_pfsize, fcfg1_eesize, fcfg1_depart;
	uint8_t fcfg2_maxaddr0, fcfg2_pflsh, fcfg2_maxaddr1;
	uint32_t nvm_size = 0, pf_size = 0, df_size = 0, ee_size = 0;
	unsigned num_blocks = 0, num_pflash_blocks = 0, num_nvm_blocks = 0, first_nvm_bank = 0,
			pflash_sector_size_bytes = 0, nvm_sector_size_bytes = 0;
	struct target *target = bank->target;
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	kinfo->probed = false;

	result = target_read_u32(target, SIM_SDID, &kinfo->sim_sdid);
	if (result != ERROR_OK)
		return result;

	if ((kinfo->sim_sdid & (~KINETIS_SDID_K_SERIES_MASK)) == 0) {
		/* older K-series MCU */
		uint32_t mcu_type = kinfo->sim_sdid & KINETIS_K_SDID_TYPE_MASK;

		switch (mcu_type) {
		case KINETIS_K_SDID_K10_M50:
		case KINETIS_K_SDID_K20_M50:
			/* 1kB sectors */
			pflash_sector_size_bytes = 1<<10;
			nvm_sector_size_bytes = 1<<10;
			num_blocks = 2;
			kinfo->flash_support = FS_PROGRAM_LONGWORD | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
			break;
		case KINETIS_K_SDID_K10_M72:
		case KINETIS_K_SDID_K20_M72:
		case KINETIS_K_SDID_K30_M72:
		case KINETIS_K_SDID_K30_M100:
		case KINETIS_K_SDID_K40_M72:
		case KINETIS_K_SDID_K40_M100:
		case KINETIS_K_SDID_K50_M72:
			/* 2kB sectors, 1kB FlexNVM sectors */
			pflash_sector_size_bytes = 2<<10;
			nvm_sector_size_bytes = 1<<10;
			num_blocks = 2;
			kinfo->flash_support = FS_PROGRAM_LONGWORD | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
			kinfo->max_flash_prog_size = 1<<10;
			break;
		case KINETIS_K_SDID_K10_M100:
		case KINETIS_K_SDID_K20_M100:
		case KINETIS_K_SDID_K11:
		case KINETIS_K_SDID_K12:
		case KINETIS_K_SDID_K21_M50:
		case KINETIS_K_SDID_K22_M50:
		case KINETIS_K_SDID_K51_M72:
		case KINETIS_K_SDID_K53:
		case KINETIS_K_SDID_K60_M100:
			/* 2kB sectors */
			pflash_sector_size_bytes = 2<<10;
			nvm_sector_size_bytes = 2<<10;
			num_blocks = 2;
			kinfo->flash_support = FS_PROGRAM_LONGWORD | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
			break;
		case KINETIS_K_SDID_K21_M120:
		case KINETIS_K_SDID_K22_M120:
			/* 4kB sectors (MK21FN1M0, MK21FX512, MK22FN1M0, MK22FX512) */
			pflash_sector_size_bytes = 4<<10;
			kinfo->max_flash_prog_size = 1<<10;
			nvm_sector_size_bytes = 4<<10;
			num_blocks = 2;
			kinfo->flash_support = FS_PROGRAM_PHRASE | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
			break;
		case KINETIS_K_SDID_K10_M120:
		case KINETIS_K_SDID_K20_M120:
		case KINETIS_K_SDID_K60_M150:
		case KINETIS_K_SDID_K70_M150:
			/* 4kB sectors */
			pflash_sector_size_bytes = 4<<10;
			nvm_sector_size_bytes = 4<<10;
			num_blocks = 4;
			kinfo->flash_support = FS_PROGRAM_PHRASE | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
			break;
		default:
			LOG_ERROR("Unsupported K-family FAMID");
		}
	} else {
		/* Newer K-series or KL series MCU */
		switch (kinfo->sim_sdid & KINETIS_SDID_SERIESID_MASK) {
		case KINETIS_SDID_SERIESID_K:
			switch (kinfo->sim_sdid & (KINETIS_SDID_FAMILYID_MASK | KINETIS_SDID_SUBFAMID_MASK)) {
			case KINETIS_SDID_FAMILYID_K0X | KINETIS_SDID_SUBFAMID_KX2:
				/* K02FN64, K02FN128: FTFA, 2kB sectors */
				pflash_sector_size_bytes = 2<<10;
				num_blocks = 1;
				kinfo->flash_support = FS_PROGRAM_LONGWORD | FS_INVALIDATE_CACHE;
				break;

			case KINETIS_SDID_FAMILYID_K2X | KINETIS_SDID_SUBFAMID_KX2: {
				/* MK24FN1M reports as K22, this should detect it (according to errata note 1N83J) */
				uint32_t sopt1;
				result = target_read_u32(target, SIM_SOPT1, &sopt1);
				if (result != ERROR_OK)
					return result;

				if (((kinfo->sim_sdid & (KINETIS_SDID_DIEID_MASK)) == KINETIS_SDID_DIEID_K24FN1M) &&
						((sopt1 & KINETIS_SOPT1_RAMSIZE_MASK) == KINETIS_SOPT1_RAMSIZE_K24FN1M)) {
					/* MK24FN1M */
					pflash_sector_size_bytes = 4<<10;
					num_blocks = 2;
					kinfo->flash_support = FS_PROGRAM_PHRASE | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
					kinfo->max_flash_prog_size = 1<<10;
					break;
				}
				if ((kinfo->sim_sdid & (KINETIS_SDID_DIEID_MASK)) == KINETIS_SDID_DIEID_K22FN128
					|| (kinfo->sim_sdid & (KINETIS_SDID_DIEID_MASK)) == KINETIS_SDID_DIEID_K22FN256
					|| (kinfo->sim_sdid & (KINETIS_SDID_DIEID_MASK)) == KINETIS_SDID_DIEID_K22FN512) {
					/* K22 with new-style SDID - smaller pflash with FTFA, 2kB sectors */
					pflash_sector_size_bytes = 2<<10;
					/* autodetect 1 or 2 blocks */
					kinfo->flash_support = FS_PROGRAM_LONGWORD | FS_INVALIDATE_CACHE;
					break;
				}
				LOG_ERROR("Unsupported Kinetis K22 DIEID");
				break;
			}
			case KINETIS_SDID_FAMILYID_K2X | KINETIS_SDID_SUBFAMID_KX4:
				pflash_sector_size_bytes = 4<<10;
				if ((kinfo->sim_sdid & (KINETIS_SDID_DIEID_MASK)) == KINETIS_SDID_DIEID_K24FN256) {
					/* K24FN256 - smaller pflash with FTFA */
					num_blocks = 1;
					kinfo->flash_support = FS_PROGRAM_LONGWORD | FS_INVALIDATE_CACHE;
					break;
				}
				/* K24FN1M without errata 7534 */
				num_blocks = 2;
				kinfo->flash_support = FS_PROGRAM_PHRASE | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
				kinfo->max_flash_prog_size = 1<<10;
				break;

			case KINETIS_SDID_FAMILYID_K6X | KINETIS_SDID_SUBFAMID_KX3:
			case KINETIS_SDID_FAMILYID_K6X | KINETIS_SDID_SUBFAMID_KX1:	/* errata 7534 - should be K63 */
				/* K63FN1M0 */
			case KINETIS_SDID_FAMILYID_K6X | KINETIS_SDID_SUBFAMID_KX4:
			case KINETIS_SDID_FAMILYID_K6X | KINETIS_SDID_SUBFAMID_KX2:	/* errata 7534 - should be K64 */
				/* K64FN1M0, K64FX512 */
				pflash_sector_size_bytes = 4<<10;
				nvm_sector_size_bytes = 4<<10;
				kinfo->max_flash_prog_size = 1<<10;
				num_blocks = 2;
				kinfo->flash_support = FS_PROGRAM_PHRASE | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
				break;

			case KINETIS_SDID_FAMILYID_K2X | KINETIS_SDID_SUBFAMID_KX6:
				/* K26FN2M0 */
			case KINETIS_SDID_FAMILYID_K6X | KINETIS_SDID_SUBFAMID_KX6:
				/* K66FN2M0, K66FX1M0 */
				pflash_sector_size_bytes = 4<<10;
				nvm_sector_size_bytes = 4<<10;
				kinfo->max_flash_prog_size = 1<<10;
				num_blocks = 4;
				kinfo->flash_support = FS_PROGRAM_PHRASE | FS_PROGRAM_SECTOR | FS_INVALIDATE_CACHE;
				break;
			default:
				LOG_ERROR("Unsupported Kinetis FAMILYID SUBFAMID");
			}
			break;
		case KINETIS_SDID_SERIESID_KL:
			/* KL-series */
			pflash_sector_size_bytes = 1<<10;
			nvm_sector_size_bytes = 1<<10;
			/* autodetect 1 or 2 blocks */
			kinfo->flash_support = FS_PROGRAM_LONGWORD;
			break;
		default:
			LOG_ERROR("Unsupported K-series");
		}
	}

	if (pflash_sector_size_bytes == 0) {
		LOG_ERROR("MCU is unsupported, SDID 0x%08" PRIx32, kinfo->sim_sdid);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	result = target_read_u32(target, SIM_FCFG1, &kinfo->sim_fcfg1);
	if (result != ERROR_OK)
		return result;

	result = target_read_u32(target, SIM_FCFG2, &kinfo->sim_fcfg2);
	if (result != ERROR_OK)
		return result;

	LOG_DEBUG("SDID: 0x%08" PRIX32 " FCFG1: 0x%08" PRIX32 " FCFG2: 0x%08" PRIX32, kinfo->sim_sdid,
			kinfo->sim_fcfg1, kinfo->sim_fcfg2);

	fcfg1_nvmsize = (uint8_t)((kinfo->sim_fcfg1 >> 28) & 0x0f);
	fcfg1_pfsize = (uint8_t)((kinfo->sim_fcfg1 >> 24) & 0x0f);
	fcfg1_eesize = (uint8_t)((kinfo->sim_fcfg1 >> 16) & 0x0f);
	fcfg1_depart = (uint8_t)((kinfo->sim_fcfg1 >> 8) & 0x0f);

	fcfg2_pflsh = (uint8_t)((kinfo->sim_fcfg2 >> 23) & 0x01);
	fcfg2_maxaddr0 = (uint8_t)((kinfo->sim_fcfg2 >> 24) & 0x7f);
	fcfg2_maxaddr1 = (uint8_t)((kinfo->sim_fcfg2 >> 16) & 0x7f);

	if (num_blocks == 0)
		num_blocks = fcfg2_maxaddr1 ? 2 : 1;
	else if (fcfg2_maxaddr1 == 0 && num_blocks >= 2) {
		num_blocks = 1;
		LOG_WARNING("MAXADDR1 is zero, number of flash banks adjusted to 1");
	} else if (fcfg2_maxaddr1 != 0 && num_blocks == 1) {
		num_blocks = 2;
		LOG_WARNING("MAXADDR1 is non zero, number of flash banks adjusted to 2");
	}

	/* when the PFLSH bit is set, there is no FlexNVM/FlexRAM */
	if (!fcfg2_pflsh) {
		switch (fcfg1_nvmsize) {
		case 0x03:
		case 0x05:
		case 0x07:
		case 0x09:
		case 0x0b:
			nvm_size = 1 << (14 + (fcfg1_nvmsize >> 1));
			break;
		case 0x0f:
			if (pflash_sector_size_bytes >= 4<<10)
				nvm_size = 512<<10;
			else
				/* K20_100 */
				nvm_size = 256<<10;
			break;
		default:
			nvm_size = 0;
			break;
		}

		switch (fcfg1_eesize) {
		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
		case 0x08:
		case 0x09:
			ee_size = (16 << (10 - fcfg1_eesize));
			break;
		default:
			ee_size = 0;
			break;
		}

		switch (fcfg1_depart) {
		case 0x01:
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05:
		case 0x06:
			df_size = nvm_size - (4096 << fcfg1_depart);
			break;
		case 0x08:
			df_size = 0;
			break;
		case 0x09:
		case 0x0a:
		case 0x0b:
		case 0x0c:
		case 0x0d:
			df_size = 4096 << (fcfg1_depart & 0x7);
			break;
		default:
			df_size = nvm_size;
			break;
		}
	}

	switch (fcfg1_pfsize) {
	case 0x03:
	case 0x05:
	case 0x07:
	case 0x09:
	case 0x0b:
	case 0x0d:
		pf_size = 1 << (14 + (fcfg1_pfsize >> 1));
		break;
	case 0x0f:
		/* a peculiar case: Freescale states different sizes for 0xf
		 * K02P64M100SFARM	128 KB ... duplicate of code 0x7
		 * K22P121M120SF8RM	256 KB ... duplicate of code 0x9
		 * K22P121M120SF7RM	512 KB ... duplicate of code 0xb
		 * K22P100M120SF5RM	1024 KB ... duplicate of code 0xd
		 * K26P169M180SF5RM	2048 KB ... the only unique value
		 * fcfg2_maxaddr0 seems to be the only clue to pf_size
		 * Checking fcfg2_maxaddr0 later in this routine is pointless then
		 */
		if (fcfg2_pflsh)
			pf_size = ((uint32_t)fcfg2_maxaddr0 << 13) * num_blocks;
		else
			pf_size = ((uint32_t)fcfg2_maxaddr0 << 13) * num_blocks / 2;
		if (pf_size != 2048<<10)
			LOG_WARNING("SIM_FCFG1 PFSIZE = 0xf: please check if pflash is %u KB", pf_size>>10);

		break;
	default:
		pf_size = 0;
		break;
	}

	LOG_DEBUG("FlexNVM: %" PRIu32 " PFlash: %" PRIu32 " FlexRAM: %" PRIu32 " PFLSH: %d",
		  nvm_size, pf_size, ee_size, fcfg2_pflsh);

	num_pflash_blocks = num_blocks / (2 - fcfg2_pflsh);
	first_nvm_bank = num_pflash_blocks;
	num_nvm_blocks = num_blocks - num_pflash_blocks;

	LOG_DEBUG("%d blocks total: %d PFlash, %d FlexNVM",
			num_blocks, num_pflash_blocks, num_nvm_blocks);

	LOG_INFO("Probing flash info for bank %d", bank->bank_number);

	if ((unsigned)bank->bank_number < num_pflash_blocks) {
		/* pflash, banks start at address zero */
		kinfo->flash_class = FC_PFLASH;
		bank->size = (pf_size / num_pflash_blocks);
		bank->base = 0x00000000 + bank->size * bank->bank_number;
		kinfo->prog_base = bank->base;
		kinfo->sector_size = pflash_sector_size_bytes;
		kinfo->protection_size = pf_size / 32;
		kinfo->protection_block = (32 / num_pflash_blocks) * bank->bank_number;

	} else if ((unsigned)bank->bank_number < num_blocks) {
		/* nvm, banks start at address 0x10000000 */
		unsigned nvm_ord = bank->bank_number - first_nvm_bank;
		uint32_t limit;

		kinfo->flash_class = FC_FLEX_NVM;
		bank->size = (nvm_size / num_nvm_blocks);
		bank->base = 0x10000000 + bank->size * nvm_ord;
		kinfo->prog_base = 0x00800000 + bank->size * nvm_ord;
		kinfo->sector_size = nvm_sector_size_bytes;
		if (df_size == 0) {
			kinfo->protection_size = 0;
		} else {
			for (i = df_size; ~i & 1; i >>= 1)
				;
			if (i == 1)
				kinfo->protection_size = df_size / 8;	/* data flash size = 2^^n */
			else
				kinfo->protection_size = nvm_size / 8;	/* TODO: verify on SF1, not documented in RM */
		}
		kinfo->protection_block = (8 / num_nvm_blocks) * nvm_ord;

		/* EEPROM backup part of FlexNVM is not accessible, use df_size as a limit */
		if (df_size > bank->size * nvm_ord)
			limit = df_size - bank->size * nvm_ord;
		else
			limit = 0;

		if (bank->size > limit) {
			bank->size = limit;
			LOG_DEBUG("FlexNVM bank %d limited to 0x%08" PRIx32 " due to active EEPROM backup",
				bank->bank_number, limit);
		}

	} else if ((unsigned)bank->bank_number == num_blocks) {
		LOG_ERROR("FlexRAM support not yet implemented");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	} else {
		LOG_ERROR("Cannot determine parameters for bank %d, only %d banks on device",
				bank->bank_number, num_blocks);
		return ERROR_FLASH_BANK_INVALID;
	}

	if (bank->bank_number == 0 && ((uint32_t)fcfg2_maxaddr0 << 13) != bank->size)
		LOG_WARNING("MAXADDR0 0x%02" PRIx8 " check failed,"
				" please report to OpenOCD mailing list", fcfg2_maxaddr0);
	if (fcfg2_pflsh) {
		if (bank->bank_number == 1 && ((uint32_t)fcfg2_maxaddr1 << 13) != bank->size)
			LOG_WARNING("MAXADDR1 0x%02" PRIx8 " check failed,"
				" please report to OpenOCD mailing list", fcfg2_maxaddr1);
	} else {
		if ((unsigned)bank->bank_number == first_nvm_bank
				&& ((uint32_t)fcfg2_maxaddr1 << 13) != df_size)
			LOG_WARNING("FlexNVM MAXADDR1 0x%02" PRIx8 " check failed,"
				" please report to OpenOCD mailing list", fcfg2_maxaddr1);
	}

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	if (kinfo->sector_size == 0) {
		LOG_ERROR("Unknown sector size for bank %d", bank->bank_number);
		return ERROR_FLASH_BANK_INVALID;
	}

	if (kinfo->flash_support & FS_PROGRAM_SECTOR
			 && kinfo->max_flash_prog_size == 0) {
		kinfo->max_flash_prog_size = kinfo->sector_size;
		/* Program section size is equal to sector size by default */
	}

	bank->num_sectors = bank->size / kinfo->sector_size;

	if (bank->num_sectors > 0) {
		/* FlexNVM bank can be used for EEPROM backup therefore zero sized */
		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			bank->sectors[i].size = kinfo->sector_size;
			offset += kinfo->sector_size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;
		}
	}

	kinfo->probed = true;

	return ERROR_OK;
}

static int kinetis_probe(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	return kinetis_read_part_info(bank);
}

static int kinetis_auto_probe(struct flash_bank *bank)
{
	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	if (kinfo && kinfo->probed)
		return ERROR_OK;

	return kinetis_probe(bank);
}

static int kinetis_info(struct flash_bank *bank, char *buf, int buf_size)
{
	const char *bank_class_names[] = {
		"(ANY)", "PFlash", "FlexNVM", "FlexRAM"
	};

	struct kinetis_flash_bank *kinfo = bank->driver_priv;

	(void) snprintf(buf, buf_size,
			"%s driver for %s flash bank %s at 0x%8.8" PRIx32 "",
			bank->driver->name, bank_class_names[kinfo->flash_class],
			bank->name, bank->base);

	return ERROR_OK;
}

static int kinetis_blank_check(struct flash_bank *bank)
{
	struct kinetis_flash_bank *kinfo = bank->driver_priv;
	int result;

	/* suprisingly blank check does not work in VLPR and HSRUN modes */
	result = kinetis_check_run_mode(bank->target);
	if (result != ERROR_OK)
		return result;

	if (kinfo->flash_class == FC_PFLASH || kinfo->flash_class == FC_FLEX_NVM) {
		bool block_dirty = false;
		uint8_t ftfx_fstat;

		if (kinfo->flash_class == FC_FLEX_NVM) {
			uint8_t fcfg1_depart = (uint8_t)((kinfo->sim_fcfg1 >> 8) & 0x0f);
			/* block operation cannot be used on FlexNVM when EEPROM backup partition is set */
			if (fcfg1_depart != 0xf && fcfg1_depart != 0)
				block_dirty = true;
		}

		if (!block_dirty) {
			/* check if whole bank is blank */
			result = kinetis_ftfx_command(bank->target, FTFx_CMD_BLOCKSTAT, kinfo->prog_base,
							 0, 0, 0, 0,  0, 0, 0, 0, &ftfx_fstat);

			if (result != ERROR_OK || (ftfx_fstat & 0x01))
				block_dirty = true;
		}

		if (block_dirty) {
			/* the whole bank is not erased, check sector-by-sector */
			int i;
			for (i = 0; i < bank->num_sectors; i++) {
				/* normal margin */
				result = kinetis_ftfx_command(bank->target, FTFx_CMD_SECTSTAT,
						kinfo->prog_base + bank->sectors[i].offset,
						1, 0, 0, 0,  0, 0, 0, 0, &ftfx_fstat);

				if (result == ERROR_OK) {
					bank->sectors[i].is_erased = !(ftfx_fstat & 0x01);
				} else {
					LOG_DEBUG("Ignoring errored PFlash sector blank-check");
					bank->sectors[i].is_erased = -1;
				}
			}
		} else {
			/* the whole bank is erased, update all sectors */
			int i;
			for (i = 0; i < bank->num_sectors; i++)
				bank->sectors[i].is_erased = 1;
		}
	} else {
		LOG_WARNING("kinetis_blank_check not supported yet for FlexRAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}


COMMAND_HANDLER(kinetis_nvm_partition)
{
	int result, i;
	unsigned long par, log2 = 0, ee1 = 0, ee2 = 0;
	enum { SHOW_INFO, DF_SIZE, EEBKP_SIZE } sz_type = SHOW_INFO;
	bool enable;
	uint8_t ftfx_fstat;
	uint8_t load_flex_ram = 1;
	uint8_t ee_size_code = 0x3f;
	uint8_t flex_nvm_partition_code = 0;
	uint8_t ee_split = 3;
	struct target *target = get_current_target(CMD_CTX);
	struct flash_bank *bank;
	struct kinetis_flash_bank *kinfo;
	uint32_t sim_fcfg1;

	if (CMD_ARGC >= 2) {
		if (strcmp(CMD_ARGV[0], "dataflash") == 0)
			sz_type = DF_SIZE;
		else if (strcmp(CMD_ARGV[0], "eebkp") == 0)
			sz_type = EEBKP_SIZE;

		par = strtoul(CMD_ARGV[1], NULL, 10);
		while (par >> (log2 + 3))
			log2++;
	}
	switch (sz_type) {
	case SHOW_INFO:
		result = target_read_u32(target, SIM_FCFG1, &sim_fcfg1);
		if (result != ERROR_OK)
			return result;

		flex_nvm_partition_code = (uint8_t)((sim_fcfg1 >> 8) & 0x0f);
		switch (flex_nvm_partition_code) {
		case 0:
			command_print(CMD_CTX, "No EEPROM backup, data flash only");
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			command_print(CMD_CTX, "EEPROM backup %d KB", 4 << flex_nvm_partition_code);
			break;
		case 8:
			command_print(CMD_CTX, "No data flash, EEPROM backup only");
			break;
		case 0x9:
		case 0xA:
		case 0xB:
		case 0xC:
		case 0xD:
		case 0xE:
			command_print(CMD_CTX, "data flash %d KB", 4 << (flex_nvm_partition_code & 7));
			break;
		case 0xf:
			command_print(CMD_CTX, "No EEPROM backup, data flash only (DEPART not set)");
			break;
		default:
			command_print(CMD_CTX, "Unsupported EEPROM backup size code 0x%02" PRIx8, flex_nvm_partition_code);
		}
		return ERROR_OK;

	case DF_SIZE:
		flex_nvm_partition_code = 0x8 | log2;
		break;

	case EEBKP_SIZE:
		flex_nvm_partition_code = log2;
		break;
	}

	if (CMD_ARGC == 3)
		ee1 = ee2 = strtoul(CMD_ARGV[2], NULL, 10) / 2;
	else if (CMD_ARGC >= 4) {
		ee1 = strtoul(CMD_ARGV[2], NULL, 10);
		ee2 = strtoul(CMD_ARGV[3], NULL, 10);
	}

	enable = ee1 + ee2 > 0;
	if (enable) {
		for (log2 = 2; ; log2++) {
			if (ee1 + ee2 == (16u << 10) >> log2)
				break;
			if (ee1 + ee2 > (16u << 10) >> log2 || log2 >= 9) {
				LOG_ERROR("Unsupported EEPROM size");
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}

		if (ee1 * 3 == ee2)
			ee_split = 1;
		else if (ee1 * 7 == ee2)
			ee_split = 0;
		else if (ee1 != ee2) {
			LOG_ERROR("Unsupported EEPROM sizes ratio");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		ee_size_code = log2 | ee_split << 4;
	}

	if (CMD_ARGC >= 5)
		COMMAND_PARSE_ON_OFF(CMD_ARGV[4], enable);
	if (enable)
		load_flex_ram = 0;

	LOG_INFO("DEPART 0x%" PRIx8 ", EEPROM size code 0x%" PRIx8,
		 flex_nvm_partition_code, ee_size_code);

	result = kinetis_check_run_mode(target);
	if (result != ERROR_OK)
		return result;

	result = kinetis_ftfx_command(target, FTFx_CMD_PGMPART, load_flex_ram,
				      ee_size_code, flex_nvm_partition_code, 0, 0,
				      0, 0, 0, 0,  &ftfx_fstat);
	if (result != ERROR_OK)
		return result;

	command_print(CMD_CTX, "FlexNVM partition set. Please reset MCU.");

	for (i = 1; i < 4; i++) {
		bank = get_flash_bank_by_num_noprobe(i);
		if (bank == NULL)
			break;

		kinfo = bank->driver_priv;
		if (kinfo && kinfo->flash_class == FC_FLEX_NVM)
			kinfo->probed = false;	/* re-probe before next use */
	}

	command_print(CMD_CTX, "FlexNVM banks will be re-probed to set new data flash size.");
	return ERROR_OK;
}


static const struct command_registration kinetis_securtiy_command_handlers[] = {
	{
		.name = "check_security",
		.mode = COMMAND_EXEC,
		.help = "",
		.usage = "",
		.handler = kinetis_check_flash_security_status,
	},
	{
		.name = "mass_erase",
		.mode = COMMAND_EXEC,
		.help = "",
		.usage = "",
		.handler = kinetis_mdm_mass_erase,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration kinetis_exec_command_handlers[] = {
	{
		.name = "mdm",
		.mode = COMMAND_ANY,
		.help = "",
		.usage = "",
		.chain = kinetis_securtiy_command_handlers,
	},
	{
		.name = "disable_wdog",
		.mode = COMMAND_EXEC,
		.help = "Disable the watchdog timer",
		.usage = "",
		.handler = kinetis_disable_wdog_handler,
	},
	{
		.name = "nvm_partition",
		.mode = COMMAND_EXEC,
		.help = "Show/set data flash or EEPROM backup size in kilobytes,"
			" set two EEPROM sizes in bytes and FlexRAM loading during reset",
		.usage = "('info'|'dataflash' size|'eebkp' size) [eesize1 eesize2] ['on'|'off']",
		.handler = kinetis_nvm_partition,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration kinetis_command_handler[] = {
	{
		.name = "kinetis",
		.mode = COMMAND_ANY,
		.help = "kinetis flash controller commands",
		.usage = "",
		.chain = kinetis_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};



struct flash_driver kinetis_flash = {
	.name = "kinetis",
	.commands = kinetis_command_handler,
	.flash_bank_command = kinetis_flash_bank_command,
	.erase = kinetis_erase,
	.protect = kinetis_protect,
	.write = kinetis_write,
	.read = default_flash_read,
	.probe = kinetis_probe,
	.auto_probe = kinetis_auto_probe,
	.erase_check = kinetis_blank_check,
	.protect_check = kinetis_protect_check,
	.info = kinetis_info,
};

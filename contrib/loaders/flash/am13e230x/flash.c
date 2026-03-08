// SPDX-License-Identifier: GPL-2.0-or-later

/******************************************************************************
 *
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Flash operations for AM13E230X, translated from the OpenOCD driver
 * (src/flash/nor/am13e230x.c) to run on-device with direct register access.
 *
 * Memory barriers (__DSB/__ISB) are required around CMDEXEC writes per the
 * SDK's DL_FlashCTL_executeCommand — the Cortex-M33 write buffer can
 * reorder peripheral writes without them.
 *
 *****************************************************************************/

#include "flash.h"

/*
 * ARM Cortex-M memory barrier intrinsics.
 * These are compiler built-ins for DSB and ISB instructions.
 */
#define __DSB() __asm volatile("dsb 0xF" ::: "memory")
#define __ISB() __asm volatile("isb 0xF" ::: "memory")

int flash_request_gsc_semaphore(void)
{
	HWREG(GSC_REG_FPC_FLSEMREQ) = 0x00000001;
	/* Ensure the semaphore request write reaches the peripheral before
	 * reading FLSEMSTAT */
	__DSB();
	__ISB();
	uint32_t stat = HWREG(GSC_REG_FPC_FLSEMSTAT);
	/* ASSIGNED must be set and DBGACC must be clear (CPU holds it) */
	if ((stat & (GSC_FLSEMSTAT_ASSIGNED | GSC_FLSEMSTAT_DBGACC)) != GSC_FLSEMSTAT_ASSIGNED)
		return -1;
	return 0;
}

int flash_clear_gsc_semaphore(void)
{
	HWREG(GSC_REG_FPC_FLSEMCLR) = 0x00000001;
	__DSB();
	__ISB();
	uint32_t stat = HWREG(GSC_REG_FPC_FLSEMSTAT);
	/* ASSIGNED must be clear (semaphore released) */
	if (stat & GSC_FLSEMSTAT_ASSIGNED)
		return -1;
	return 0;
}

/*
 * Execute a flash command and wait for completion.
 * Must be called after all command registers (CMDTYPE, CMDADDR, etc.)
 * are configured. Matches the SDK's DL_FlashCTL_executeCommand.
 *
 * Returns 0 on success, -1 on timeout, -2 on command failure.
 */
static int flash_execute_cmd(void)
{
	/* Ensure all config register writes complete before execute */
	__DSB();
	__ISB();

	HWREG(FCTL_REG_CMDEXEC) = FCTL_CMDEXEC_EXECUTE;

	/* Ensure execute write completes before polling status */
	__DSB();
	__ISB();

	uint32_t status;
	uint32_t timeout = FLASH_TIMEOUT;

	do {
		status = HWREG(FCTL_REG_STATCMD);
		if (--timeout == 0)
			return -1;
	} while ((status & FCTL_STATCMD_CMDDONE) == 0);

	if ((status & FCTL_STATCMD_CMDPASS) == 0)
		return -2;

	return 0;
}

/*
 * Clear the STATCMD register by issuing a CLEARSTATUS command.
 * Must be called before each erase operation.
 */
static int flash_clear_status(void)
{
	HWREG(FCTL_REG_CMDTYPE) = FCTL_CMDTYPE_CLEARSTATUS;

	__DSB();
	__ISB();

	HWREG(FCTL_REG_CMDEXEC) = FCTL_CMDEXEC_EXECUTE;

	__DSB();
	__ISB();

	uint32_t timeout = FLASH_TIMEOUT;
	while (HWREG(FCTL_REG_STATCMD) & FCTL_STATCMD_CMDINPROGRESS) {
		if (--timeout == 0)
			return -1;
	}

	return 0;
}

/*
 * Unprotect all sectors by clearing protection registers.
 * Protection is automatically re-enabled by hardware after each operation.
 */
static void flash_unprotect(void)
{
	HWREG(FCTL_REG_CMDWEPROTA) = 0;
	HWREG(FCTL_REG_CMDWEPROTB) = 0;
	HWREG(FCTL_REG_CMDWEPROTNM) = 0;
}

int flash_sector_erase(uint32_t addr)
{
	if (flash_clear_status() != 0)
		return -1;

	flash_unprotect();

	HWREG(FCTL_REG_CMDTYPE) = FCTL_CMDTYPE_ERASE | FCTL_CMDTYPE_SIZE_SECTOR;
	HWREG(FCTL_REG_CMDADDR) = addr & 0xFFFFFFF0;

	return flash_execute_cmd();
}

int flash_write_sector(const uint8_t *data, uint32_t addr, uint32_t count)
{
	while (count > 0) {
		uint32_t num_bytes = (count < FLASH_WORD_SIZE) ? count : FLASH_WORD_SIZE;
		uint32_t num_send = num_bytes;
		const uint8_t *write_buf = data;

		/* Byte enable: one bit per data byte + ECC chunk bits */
		uint32_t bytes_en = (1u << num_send) - 1;
		bytes_en |= (1u << 16);           /* ECC chunk 0 */
		if (num_send > 8)
			bytes_en |= (1u << 17);   /* ECC chunk 1 */

		/* SDK calls executeClearStatus + unprotect before every word write */
		if (flash_clear_status() != 0)
			return -1;
		flash_unprotect();

		HWREG(FCTL_REG_CMDTYPE) = FCTL_CMDTYPE_PROGRAM | FCTL_CMDTYPE_SIZE_ONEWORD;
		HWREG(FCTL_REG_CMDBYTEN) = bytes_en;
		HWREG(FCTL_REG_CMDADDR) = addr & 0xFFFFFFF0;

		/* Flash word index: (address >> 4) & 3, per SDK */
		HWREG(FCTL_REG_CMDDATAINDEX) = (addr >> 4) & 3;

		/* Write data to flash data register (up to 4 x 32-bit words) */
		volatile uint32_t *data_reg = (volatile uint32_t *)FCTL_REG_CMDDATA0;
		for (uint32_t i = 0; i < num_send; i += 4) {
			data_reg[i / 4] = (uint32_t)write_buf[i]
				| ((uint32_t)write_buf[i + 1] << 8)
				| ((uint32_t)write_buf[i + 2] << 16)
				| ((uint32_t)write_buf[i + 3] << 24);
		}

		if (flash_execute_cmd() != 0)
			return -1;

		addr += num_bytes;
		data += num_bytes;
		count -= num_bytes;
	}

	return 0;
}

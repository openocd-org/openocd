// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * NOR flash driver for AM13E230X class of uC from Texas Instruments.
 *
 * Erase uses direct register writes via the debug probe.
 * Write uses an on-device algorithm loaded into SRAM
 * (see contrib/loaders/flash/am13e230x/ and flash.h).
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* Region memory map */
#define AM13_FLASH_BASE_MAIN           0x00000000
#define AM13_FLASH_BASE_NONMAIN        0x60100000
#define AM13_NONMAIN_SIZE_PER_BANK     0x1000

/*
 * Factory region registers (at 0x60111000, in the non-main flash area).
 *
 * AM13_DID (0x60111004) is the DEVICEID / JTAGIDCODE register ("the
 * JTAGIDCODE that comes from the Ramp system", per TRM SPRUJF2A Table
 * 18-1).  Its manufacturer ID field lives in bits[11:1] per the JTAG
 * IEEE 1149.1 encoding, which is what am13_probe() validates.
 *
 * This is distinct from PARTIDL/PARTIDH at 0x0005D008, which encode
 * package type and quality grade. Those fields are not used here
 * because they vary by package variant and are not needed to confirm
 * that the target is an AM13E230X-class device.
 */
#define AM13_FACTORYREGION             0x60111000
#define AM13_TRACEID                   (AM13_FACTORYREGION + 0x000)
#define AM13_DID                       (AM13_FACTORYREGION + 0x004)  /* JTAGIDCODE */
#define AM13_SRAMFLASH                 (AM13_FACTORYREGION + 0x01C)

/* Flash controller (NVMNW) registers */
#define FLASH_CONTROL_BASE              0x40042000
#define FCTL_REG_CMDEXEC                (FLASH_CONTROL_BASE + 0x1100)
#define FCTL_REG_CMDTYPE                (FLASH_CONTROL_BASE + 0x1104)
#define FCTL_REG_CMDADDR                (FLASH_CONTROL_BASE + 0x1120)
#define FCTL_REG_CMDBYTEN               (FLASH_CONTROL_BASE + 0x1124)
#define FCTL_REG_CMDDATAINDEX           (FLASH_CONTROL_BASE + 0x112C)
#define FCTL_REG_CMDDATA0               (FLASH_CONTROL_BASE + 0x1130)
#define FCTL_REG_CMDWEPROTA             (FLASH_CONTROL_BASE + 0x11D0)
#define FCTL_REG_CMDWEPROTB             (FLASH_CONTROL_BASE + 0x11D4)
#define FCTL_REG_CMDWEPROTNM            (FLASH_CONTROL_BASE + 0x1210)
#define FCTL_REG_STATCMD                (FLASH_CONTROL_BASE + 0x13D0)

/* GSC flash semaphore registers */
#define GSC_BASE                        0x40046000
#define GSC_REG_FPC_FLSEMREQ            (GSC_BASE + 0x1800)
#define GSC_REG_FPC_FLSEMCLR            (GSC_BASE + 0x1804)

/* STATCMD bits */
#define FCTL_STATCMD_CMDDONE            0x00000001
#define FCTL_STATCMD_CMDPASS            0x00000002
#define FCTL_STATCMD_CMDINPROGRESS      0x00000004

/* CMDEXEC / CMDTYPE constants */
#define FCTL_CMDEXEC_EXECUTE            0x00000001
#define FCTL_CMDTYPE_PROGRAM            0x00000001
#define FCTL_CMDTYPE_ERASE              0x00000002
#define FCTL_CMDTYPE_CLEARSTATUS        0x00000005
#define FCTL_CMDTYPE_SIZE_ONEWORD       0x00000000
#define FCTL_CMDTYPE_SIZE_SECTOR        0x00000040

#define AM13_FLASH_WORD_SIZE            16

#define AM13_FLASH_TIMEOUT_MS           8000
#define AM13_SECTOR_SIZE_BYTES          0x800
#define TI_MANUFACTURER_ID              0x17

/* Algorithm is loaded at the base of SRAM */
#define AM13_ALGO_BASE          0x20000000

/* Parameter blocks (two, for ping-pong) */
#define AM13_ALGO_PARAMS_0      0x20002000
#define AM13_ALGO_PARAMS_1      0x20002014

/* Data buffers (each one sector = 2 KB) */
#define AM13_ALGO_BUFFER_0      0x20002100
#define AM13_ALGO_BUFFER_1      0x20002900

/* Total working area needed */
#define AM13_ALGO_WORKING_SIZE  (AM13_ALGO_BUFFER_1 + 0x800 - AM13_ALGO_BASE)

/* Handshake values (must match loader main.c) */
#define AM13_BUFFER_EMPTY       0x00000000
#define AM13_BUFFER_FULL        0xFFFFFFFF

/* Offset of the status field within flash_params */
#define AM13_STATUS_OFFSET      0x0C

/* Commands (must match loader main.c) */
#define AM13_CMD_NO_ACTION              0
#define AM13_CMD_PROGRAM                1
#define AM13_CMD_ERASE_AND_PROGRAM      2
#define AM13_CMD_ERASE_SECTORS          3

/* Parameter block layout (matches struct flash_params in loader) */
struct am13_algo_params {
	uint8_t dest[4];
	uint8_t len[4];
	uint8_t cmd[4];
	uint8_t status[4];
	uint8_t buf_addr[4];
};

/* Flash loader algorithm binary */
static const uint8_t am13_algo[] = {
#include "../../../contrib/loaders/flash/am13e230x/am13e230x_algo.inc"
};

struct am13_flash_bank {
	uint32_t did;
	uint32_t traceid;
	unsigned int main_flash_size_kb;
	unsigned int main_flash_num_banks;
	unsigned int sram_size_kb;
	unsigned int sector_size;

	/* Algorithm state */
	struct working_area *working_area;
	struct armv7m_algorithm armv7m_info;
};

static int am13_auto_probe(struct flash_bank *bank);

/*
 * GSC flash semaphore must be held before any flash operation.
 * The debug probe and the CPU have separate ownership tracking
 * (FLSEMSTAT.DBGACC), so the erase path (debug probe) and the
 * write path (on-device algorithm) each acquire it independently.
 */

static int am13_request_gsc_semaphore(struct flash_bank *bank)
{
	int retval = target_write_u32(bank->target, GSC_REG_FPC_FLSEMREQ, 1);
	if (retval != ERROR_OK)
		LOG_ERROR("AM13: failed to acquire GSC flash semaphore");
	return retval;
}

static int am13_clear_gsc_semaphore(struct flash_bank *bank)
{
	int retval = target_write_u32(bank->target, GSC_REG_FPC_FLSEMCLR, 1);
	if (retval != ERROR_OK)
		LOG_ERROR("AM13: failed to release GSC flash semaphore");
	return retval;
}

/* ---------- Flash controller helpers (erase path only) ---------- */

static const struct {
	unsigned char bit;
	const char *name;
} am13_fctl_errors[] = {
	{ 2,  "CMDINPROGRESS" },
	{ 4,  "FAILWEPROT" },
	{ 5,  "FAILVERIFY" },
	{ 6,  "FAILILLADDR" },
	{ 7,  "FAILMODE" },
	{ 12, "FAILMISC" },
};

static const char *am13_fctl_strerror(uint32_t status)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(am13_fctl_errors); i++) {
		if (status & BIT(am13_fctl_errors[i].bit))
			return am13_fctl_errors[i].name;
	}
	return "FAILUNKNOWN";
}

static int am13_fctl_wait_done(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status = 0;
	int64_t start_ms = timeval_ms();
	bool timed_out = false;

	while ((status & FCTL_STATCMD_CMDDONE) == 0) {
		int retval = target_read_u32(target, FCTL_REG_STATCMD, &status);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() - start_ms > AM13_FLASH_TIMEOUT_MS) {
			timed_out = true;
			break;
		}
		keep_alive();
	}

	if ((status & FCTL_STATCMD_CMDPASS) == 0) {
		if (timed_out)
			LOG_ERROR("AM13: flash command timed out after %d ms", AM13_FLASH_TIMEOUT_MS);
		else
			LOG_ERROR("AM13: flash command failed: %s", am13_fctl_strerror(status));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int am13_fctl_clear_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;
	int64_t start_ms;
	int retval;

	retval = target_write_u32(target, FCTL_REG_CMDTYPE, FCTL_CMDTYPE_CLEARSTATUS);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FCTL_REG_CMDEXEC, FCTL_CMDEXEC_EXECUTE);
	if (retval != ERROR_OK)
		return retval;

	start_ms = timeval_ms();
	do {
		retval = target_read_u32(target, FCTL_REG_STATCMD, &status);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() - start_ms > AM13_FLASH_TIMEOUT_MS) {
			LOG_ERROR("Timeout waiting for clear status");
			return ERROR_FAIL;
		}
		keep_alive();
	} while (status & FCTL_STATCMD_CMDINPROGRESS);

	return ERROR_OK;
}

static int am13_fctl_unprotect(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;

	switch (bank->base) {
	case AM13_FLASH_BASE_MAIN:
		retval = target_write_u32(target, FCTL_REG_CMDWEPROTA, 0);
		if (retval != ERROR_OK)
			return retval;
		return target_write_u32(target, FCTL_REG_CMDWEPROTB, 0);
	case AM13_FLASH_BASE_NONMAIN:
		return target_write_u32(target, FCTL_REG_CMDWEPROTNM, 0);
	default:
		return ERROR_FLASH_BANK_INVALID;
	}
}

static int am13_fctl_erase(struct flash_bank *bank, uint32_t addr, uint32_t cmd_size)
{
	struct target *target = bank->target;
	int retval;

	retval = am13_fctl_clear_status(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = am13_fctl_unprotect(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FCTL_REG_CMDTYPE,
				  FCTL_CMDTYPE_ERASE | cmd_size);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FCTL_REG_CMDADDR, addr & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FCTL_REG_CMDEXEC, FCTL_CMDEXEC_EXECUTE);
	if (retval != ERROR_OK)
		return retval;

	return am13_fctl_wait_done(bank);
}

/* ---------- Flash driver callbacks ---------- */

FLASH_BANK_COMMAND_HANDLER(am13_flash_bank_command)
{
	switch (bank->base) {
	case AM13_FLASH_BASE_MAIN:
	case AM13_FLASH_BASE_NONMAIN:
		break;
	default:
		LOG_ERROR("Invalid bank address " TARGET_ADDR_FMT, bank->base);
		return ERROR_FAIL;
	}

	struct am13_flash_bank *am13_info = calloc(1, sizeof(*am13_info));
	if (!am13_info)
		return ERROR_FAIL;

	bank->driver_priv = am13_info;
	am13_info->sector_size = AM13_SECTOR_SIZE_BYTES;

	bank->write_start_alignment = AM13_FLASH_WORD_SIZE;
	bank->write_end_alignment = AM13_FLASH_WORD_SIZE;

	return ERROR_OK;
}

static int am13_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct am13_flash_bank *am13_info = bank->driver_priv;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = am13_request_gsc_semaphore(bank);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int s = first; s <= last; s++) {
		if (bank->sectors[s].is_protected) {
			LOG_WARNING("AM13: NONMAIN sector %u (0x%08" TARGET_PRIxADDR
						") is factory-protected, skipping",
						s, bank->base + s * am13_info->sector_size);
			continue;
		}

		uint32_t addr = bank->base + s * am13_info->sector_size;

		retval = am13_fctl_erase(bank, addr, FCTL_CMDTYPE_SIZE_SECTOR);
		if (retval != ERROR_OK) {
			LOG_ERROR("AM13: Sector erase failed at 0x%08" PRIx32 ", returned %d", addr, retval);
			am13_clear_gsc_semaphore(bank);
			return retval;
		}
	}

	return am13_clear_gsc_semaphore(bank);
}

/* ---------- Direct write fallback (via debug probe) ---------- */

/*
 * Word-by-word flash write using direct register access from the debug
 * probe. Slower than the on-device algorithm but does not require SRAM,
 * so it works as a fallback when the algorithm cannot be loaded (e.g.,
 * SRAM is needed by a RAM-resident application).
 */
static int am13_write_direct(struct flash_bank *bank, const uint8_t *buffer,
							 uint32_t offset, uint32_t count)
{
	if (count == 0)
		return ERROR_OK;

	struct target *target = bank->target;
	uint32_t addr = bank->base + offset;
	int retval = 0;

	retval = am13_request_gsc_semaphore(bank);
	if (retval != ERROR_OK)
		return retval;

	while (count > 0) {
		/* Clear status + unprotect before every word (per SDK) */
		retval = am13_fctl_clear_status(bank);
		if (retval != ERROR_OK)
			break;

		retval = am13_fctl_unprotect(bank);
		if (retval != ERROR_OK)
			break;

		retval = target_write_u32(target, FCTL_REG_CMDTYPE,
					  FCTL_CMDTYPE_PROGRAM | FCTL_CMDTYPE_SIZE_ONEWORD);
		if (retval != ERROR_OK)
			break;

		/* All 16 data bytes + both ECC chunks (bits 16-17) enabled */
		retval = target_write_u32(target, FCTL_REG_CMDBYTEN, 0x0003FFFF);
		if (retval != ERROR_OK)
			break;

		retval = target_write_u32(target, FCTL_REG_CMDADDR, addr);
		if (retval != ERROR_OK)
			break;

		retval = target_write_u32(target, FCTL_REG_CMDDATAINDEX,
					  (addr >> 4) & 3);
		if (retval != ERROR_OK)
			break;

		retval = target_write_buffer(target, FCTL_REG_CMDDATA0, AM13_FLASH_WORD_SIZE, buffer);
		if (retval != ERROR_OK)
			break;

		retval = target_write_u32(target, FCTL_REG_CMDEXEC,
					  FCTL_CMDEXEC_EXECUTE);
		if (retval != ERROR_OK)
			break;

		retval = am13_fctl_wait_done(bank);
		if (retval != ERROR_OK)
			break;

		addr += AM13_FLASH_WORD_SIZE;
		buffer += AM13_FLASH_WORD_SIZE;
		count -= AM13_FLASH_WORD_SIZE;
	}

	int sem_ret = am13_clear_gsc_semaphore(bank);
	return (retval != ERROR_OK) ? retval : sem_ret;
}

/* ---------- Flash loader algorithm ---------- */

static int am13_algo_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct am13_flash_bank *am13_info = bank->driver_priv;
	int retval;

	/* Allocate SRAM for algorithm + buffers */
	target_free_working_area(target, am13_info->working_area);
	am13_info->working_area = NULL;

	retval = target_alloc_working_area(target, AM13_ALGO_WORKING_SIZE,
					   &am13_info->working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("AM13: insufficient SRAM for flash loader (%u bytes)",
			  AM13_ALGO_WORKING_SIZE);
		return retval;
	}

	/*
	 * The flash loader is not position-independent: it references
	 * parameter blocks (AM13_ALGO_PARAMS_0/1) and data buffers
	 * (AM13_ALGO_BUFFER_0/1) at absolute SRAM addresses assembled
	 * into the binary. The working area must start exactly at
	 * AM13_ALGO_BASE (0x20000000).
	 */
	if (am13_info->working_area->address != AM13_ALGO_BASE) {
		LOG_ERROR("AM13: working area allocated at 0x%08" TARGET_PRIxADDR
			  ", need 0x%08x; set '_WORKAREABASE 0x%08x' in target config",
			  am13_info->working_area->address,
			  AM13_ALGO_BASE, AM13_ALGO_BASE);
		target_free_working_area(target, am13_info->working_area);
		am13_info->working_area = NULL;
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, AM13_ALGO_BASE, sizeof(am13_algo), am13_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("AM13: failed to load flash algorithm");
		target_free_working_area(target, am13_info->working_area);
		am13_info->working_area = NULL;
		return retval;
	}

	/*
	 * Release any debug-held GSC semaphore so the algorithm (CPU)
	 * can acquire it — the flash controller tracks ownership via
	 * FLSEMSTAT.DBGACC and rejects mismatched access.
	 */
	retval = am13_clear_gsc_semaphore(bank);
	if (retval != ERROR_OK) {
		target_free_working_area(target, am13_info->working_area);
		am13_info->working_area = NULL;
		return retval;
	}

	am13_info->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	am13_info->armv7m_info.core_mode = ARM_MODE_THREAD;

	LOG_DEBUG("AM13: starting flash algorithm");
	retval = target_start_algorithm(target, 0, NULL, 0, NULL,
					AM13_ALGO_BASE, 0,
					&am13_info->armv7m_info);

	if (retval != ERROR_OK) {
		LOG_ERROR("AM13: failed to start flash algorithm");
		goto err_free_working_area;
	}

	return ERROR_OK;

err_free_working_area:
	target_free_working_area(target, am13_info->working_area);
	am13_info->working_area = NULL;
	return retval;
}

static int am13_algo_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct am13_flash_bank *am13_info = bank->driver_priv;

	(void)target_halt(target);

	int retval = target_wait_algorithm(target, 0, NULL, 0, NULL,
					   0, AM13_FLASH_TIMEOUT_MS,
					   &am13_info->armv7m_info);

	int sem_ret = am13_clear_gsc_semaphore(bank);

	target_free_working_area(target, am13_info->working_area);
	am13_info->working_area = NULL;

	return (retval != ERROR_OK) ? retval : sem_ret;
}

static int am13_algo_wait_done(struct flash_bank *bank, uint32_t params_addr)
{
	struct target *target = bank->target;
	uint32_t status = AM13_BUFFER_FULL;
	int64_t start_ms = timeval_ms();
	int retval;
	bool timed_out = false;

	while (status == AM13_BUFFER_FULL) {
		retval = target_read_u32(target, params_addr + AM13_STATUS_OFFSET,
					 &status);
		if (retval != ERROR_OK)
			return retval;

		int64_t elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
		if (elapsed_ms > AM13_FLASH_TIMEOUT_MS) {
			timed_out = true;
			break;
		}
	}

	if (timed_out) {
		LOG_ERROR("AM13: flash algorithm timed out after %d ms", AM13_FLASH_TIMEOUT_MS);
		return ERROR_FAIL;
	}

	if (status != AM13_BUFFER_EMPTY) {
		LOG_ERROR("AM13: flash algorithm error, status=0x%08" PRIx32,
			  status);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int am13_write(struct flash_bank *bank, const uint8_t *buffer,
					  uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct am13_algo_params algo_params[2];
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = am13_algo_init(bank);
	if (retval != ERROR_OK) {
		LOG_INFO("AM13: algorithm unavailable, falling back to direct writes");
		return am13_write_direct(bank, buffer, offset, count);
	}

	static const uint32_t params_addr[] = {
		AM13_ALGO_PARAMS_0, AM13_ALGO_PARAMS_1
	};
	static const uint32_t buffer_addr[] = {
		AM13_ALGO_BUFFER_0, AM13_ALGO_BUFFER_1
	};

	uint32_t address = bank->base + offset;
	uint32_t index = 0;
	int64_t start_ms = timeval_ms();

	buf_set_u32(algo_params[0].cmd, 0, 32, AM13_CMD_PROGRAM);
	buf_set_u32(algo_params[1].cmd, 0, 32, AM13_CMD_PROGRAM);

	while (count > 0) {
		uint32_t size = MIN(count, AM13_SECTOR_SIZE_BYTES - (address % AM13_SECTOR_SIZE_BYTES));

		retval = target_write_buffer(target, buffer_addr[index], size, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(algo_params[index].dest, 0, 32, address);
		buf_set_u32(algo_params[index].len, 0, 32, size);
		buf_set_u32(algo_params[index].status, 0, 32, AM13_BUFFER_FULL);

		/* Write dest/len/cmd/status only — preserve algorithm's buf_addr */
		retval = target_write_buffer(target, params_addr[index],
									 offsetof(struct am13_algo_params, buf_addr),
									 (uint8_t *)&algo_params[index]);
		if (retval != ERROR_OK)
			break;

		index ^= 1;
		retval = am13_algo_wait_done(bank, params_addr[index]);
		if (retval != ERROR_OK)
			break;

		count -= size;
		buffer += size;
		address += size;

		if (timeval_ms() - start_ms > 500)
			keep_alive();
	}

	/* Wait for the last submitted buffer */
	if (retval == ERROR_OK) {
		index ^= 1;
		retval = am13_algo_wait_done(bank, params_addr[index]);
	}

	int quit_ret = am13_algo_quit(bank);
	return (retval != ERROR_OK) ? retval : quit_ret;
}

/* ---------- Probe ---------- */

static int am13_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct am13_flash_bank *am13_info = bank->driver_priv;
	uint32_t did, sramflash;
	int retval;

	retval = target_read_u32(target, AM13_DID, &did);
	if (retval != ERROR_OK)
		return retval;

	if (((did & GENMASK(11, 1)) >> 1) != TI_MANUFACTURER_ID) {
		LOG_ERROR("AM13: unexpected manufacturer ID in DID=0x%08" PRIx32,
			  did);
		return ERROR_FAIL;
	}

	am13_info->did = did;

	retval = target_read_u32(target, AM13_TRACEID, &am13_info->traceid);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, AM13_SRAMFLASH, &sramflash);
	if (retval != ERROR_OK)
		return retval;

	am13_info->main_flash_size_kb  = sramflash & GENMASK(11, 0);
	am13_info->main_flash_num_banks = (sramflash & GENMASK(13, 12)) >> 12;
	am13_info->sram_size_kb        = (sramflash & GENMASK(25, 16)) >> 16;

	free(bank->sectors);
	bank->sectors = NULL;

	unsigned int num_sectors;
	switch (bank->base) {
	case AM13_FLASH_BASE_MAIN:
		bank->size = am13_info->main_flash_size_kb * 1024;
		num_sectors = bank->size / am13_info->sector_size;
		break;
	case AM13_FLASH_BASE_NONMAIN:
		bank->size = am13_info->main_flash_num_banks * AM13_NONMAIN_SIZE_PER_BANK;
		num_sectors = bank->size / am13_info->sector_size;
		break;
	default:
		LOG_ERROR("Invalid bank address " TARGET_ADDR_FMT, bank->base);
		return ERROR_FAIL;
	}

	if (num_sectors == 0) {
		bank->num_sectors = 0;
		bank->size = 0;
		return ERROR_OK;
	}

	bank->num_sectors = num_sectors;
	bank->sectors = calloc(num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors)
		return ERROR_FAIL;

	/*
	 * NONMAIN sector 0 of each bank contains factory trim and boot
	 * configuration records that are hardware-permanently protected.
	 * The NVMNW returns FAILILLADDR for any erase attempt on them.
	 */
	unsigned int sectors_per_nm_bank =
		(bank->base == AM13_FLASH_BASE_NONMAIN)
		? AM13_NONMAIN_SIZE_PER_BANK / am13_info->sector_size
		: 0;

	for (unsigned int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * am13_info->sector_size;
		bank->sectors[i].size = am13_info->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected =
			(sectors_per_nm_bank && (i % sectors_per_nm_bank) == 0) ? 1 : 0;
	}

	LOG_INFO("AM13: %s flash: %u KB (%u sectors), %u bank(s), SRAM: %u KB",
		 (bank->base == AM13_FLASH_BASE_MAIN) ? "MAIN" : "NONMAIN",
		 (unsigned int)(bank->size / 1024), num_sectors,
		 am13_info->main_flash_num_banks,
		 am13_info->sram_size_kb);

	return ERROR_OK;
}

static int am13_auto_probe(struct flash_bank *bank)
{
	struct am13_flash_bank *am13_info = bank->driver_priv;

	if (am13_info->did)
		return ERROR_OK;

	return am13_probe(bank);
}

static int get_am13_info(struct flash_bank *bank,
			 struct command_invocation *cmd)
{
	struct am13_flash_bank *am13_info = bank->driver_priv;

	command_print_sameline(cmd, "AM13E23X: DID=0x%08" PRIx32
						   " TRACEID=0x%08" PRIx32
						   " banks=%u",
						   am13_info->did, am13_info->traceid,
						   am13_info->main_flash_num_banks);
	return ERROR_OK;
}

const struct flash_driver am13_flash = {
	.name = "am13",
	.flash_bank_command = am13_flash_bank_command,
	.erase = am13_erase,
	.protect = NULL,
	.write = am13_write,
	.read = default_flash_read,
	.probe = am13_probe,
	.auto_probe = am13_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.info = get_am13_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

/* SPDX-License-Identifier: GPL-2.0-or-later */

/******************************************************************************
 *
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Flash register definitions and operation prototypes for AM13E230X.
 * Derived from src/flash/nor/am13e230x.c
 *
 *****************************************************************************/

#ifndef OPENOCD_LOADERS_FLASH_AM13E230X_FLASH_H
#define OPENOCD_LOADERS_FLASH_AM13E230X_FLASH_H

#include <stdint.h>

/* Direct 32-bit register access */
#define HWREG(x) (*((volatile uint32_t *)(x)))

/*
 * Flash Controller (FCTL) registers
 * Base: 0x40042000
 * Register block offset: 0x1000
 */
#define FLASH_CONTROL_BASE          0x40042000

#define FCTL_REG_CMDEXEC            (FLASH_CONTROL_BASE + 0x1100)
#define FCTL_REG_CMDTYPE            (FLASH_CONTROL_BASE + 0x1104)
#define FCTL_REG_CMDADDR            (FLASH_CONTROL_BASE + 0x1120)
#define FCTL_REG_CMDBYTEN           (FLASH_CONTROL_BASE + 0x1124)
#define FCTL_REG_CMDDATAINDEX       (FLASH_CONTROL_BASE + 0x112C)
#define FCTL_REG_CMDDATA0           (FLASH_CONTROL_BASE + 0x1130)
#define FCTL_REG_CMDWEPROTA         (FLASH_CONTROL_BASE + 0x11D0)
#define FCTL_REG_CMDWEPROTB         (FLASH_CONTROL_BASE + 0x11D4)
#define FCTL_REG_CMDWEPROTNM        (FLASH_CONTROL_BASE + 0x1210)
#define FCTL_REG_STATCMD            (FLASH_CONTROL_BASE + 0x13D0)

/* STATCMD bits */
#define FCTL_STATCMD_CMDDONE        0x00000001
#define FCTL_STATCMD_CMDPASS        0x00000002
#define FCTL_STATCMD_CMDINPROGRESS  0x00000004

/* CMDEXEC bits */
#define FCTL_CMDEXEC_EXECUTE        0x00000001

/* CMDTYPE command field */
#define FCTL_CMDTYPE_PROGRAM        0x00000001
#define FCTL_CMDTYPE_ERASE          0x00000002
#define FCTL_CMDTYPE_CLEARSTATUS    0x00000005

/* CMDTYPE size field */
#define FCTL_CMDTYPE_SIZE_ONEWORD   0x00000000
#define FCTL_CMDTYPE_SIZE_SECTOR    0x00000040

/*
 * GSC (Global System Control) flash semaphore registers
 */
#define GSC_BASE                    0x40046000
#define GSC_REG_FPC_FLSEMREQ        (GSC_BASE + 0x1800)
#define GSC_REG_FPC_FLSEMCLR        (GSC_BASE + 0x1804)
#define GSC_REG_FPC_FLSEMSTAT       (GSC_BASE + 0x1808)

/* FLSEMSTAT bits */
#define GSC_FLSEMSTAT_DBGACC        0x00010000  /* 1 = held by debug probe */
#define GSC_FLSEMSTAT_ASSIGNED      0x80000000  /* 1 = semaphore is held */

/* Flash geometry */
#define SECTOR_SIZE                 0x800   /* 2KB */
#define FLASH_WORD_SIZE             16      /* 128-bit = 16 bytes */

/* Flash region bases */
#define FLASH_BASE_MAIN             0x00000000

/* Timeout loop count */
#define FLASH_TIMEOUT               0x00800000

/*
 * Flash operation functions
 */

/* Acquire/release GSC flash semaphore. Return 0 on success, -1 on failure. */
int flash_request_gsc_semaphore(void);
int flash_clear_gsc_semaphore(void);

/* Erase a single sector at the given address */
int flash_sector_erase(uint32_t addr);

/* Write count bytes from data to flash at addr (handles 16-byte word writes) */
int flash_write_sector(const uint8_t *data, uint32_t addr, uint32_t count);

#endif /* OPENOCD_LOADERS_FLASH_AM13E230X_FLASH_H */

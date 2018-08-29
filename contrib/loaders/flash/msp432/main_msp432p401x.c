/******************************************************************************
*
* Copyright (C) 2013-2018 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "driverlib.h"

#include "MSP432P4_FlashLibIf.h"

/* Number of erase repeats until timeout */
#define FLASH_MAX_REPEATS 5

/* Local prototypes */
void msp432_flash_init(void);
void msp432_flash_mass_erase(void);
void msp432_flash_sector_erase(void);
void msp432_flash_write(void);
void msp432_flash_continous_write(void);
void msp432_flash_exit(void);
void unlock_flash_sectors(void);
void unlock_all_flash_sectors(void);
void lock_all_flash_sectors(void);
void __cs_set_dco_frequency_range(uint32_t dco_freq);
static bool program_device(void *src, void *dest, uint32_t length);

struct backup_params {
	uint32_t BANK0_WAIT_RESTORE;
	uint32_t BANK1_WAIT_RESTORE;
	uint32_t CS_DC0_FREQ_RESTORE;
	uint8_t  VCORE_LEVEL_RESTORE;
	uint8_t  PCM_VCORE_LEVEL_RESTORE;
};

#define BACKUP_PARAMS ((struct backup_params *) 0x20000180)

/* Main with trampoline */
int main(void)
{
	/* Halt watchdog */
	MAP_WDT_A_HOLD_TIMER();

	/* Disable interrupts */
	cpu_cpsid();

	while (1) {
		switch (FLASH_LOADER->FLASH_FUNCTION) {
			case FLASH_INIT:
				FLASH_LOADER->RETURN_CODE = FLASH_BUSY;
				msp432_flash_init();
				FLASH_LOADER->FLASH_FUNCTION = 0;
				break;
			case FLASH_MASS_ERASE:
				FLASH_LOADER->RETURN_CODE = FLASH_BUSY;
				msp432_flash_mass_erase();
				FLASH_LOADER->FLASH_FUNCTION = 0;
				break;
			case FLASH_SECTOR_ERASE:
				FLASH_LOADER->RETURN_CODE = FLASH_BUSY;
				msp432_flash_sector_erase();
				FLASH_LOADER->FLASH_FUNCTION = 0;
				break;
			case FLASH_PROGRAM:
				FLASH_LOADER->RETURN_CODE = FLASH_BUSY;
				msp432_flash_write();
				FLASH_LOADER->FLASH_FUNCTION = 0;
				break;
			case FLASH_CONTINUOUS_PROGRAM:
				FLASH_LOADER->RETURN_CODE = FLASH_BUSY;
				msp432_flash_continous_write();
				FLASH_LOADER->FLASH_FUNCTION = 0;
				break;
			case FLASH_EXIT:
				FLASH_LOADER->RETURN_CODE = FLASH_BUSY;
				msp432_flash_exit();
				FLASH_LOADER->FLASH_FUNCTION = 0;
				break;
			case FLASH_NO_COMMAND:
				break;
			default:
				FLASH_LOADER->RETURN_CODE = FLASH_WRONG_COMMAND;
				break;
		}
	}
}

/* Initialize flash */
void msp432_flash_init(void)
{
	bool success = false;

	/* Point to vector table in RAM */
	SCB->VTOR = (uint32_t)0x01000000;

	/* backup system parameters */
	BACKUP_PARAMS->BANK0_WAIT_RESTORE =
		MAP_FLASH_CTL_GET_WAIT_STATE(FLASH_BANK0);
	BACKUP_PARAMS->BANK1_WAIT_RESTORE =
		MAP_FLASH_CTL_GET_WAIT_STATE(FLASH_BANK1);
	BACKUP_PARAMS->VCORE_LEVEL_RESTORE = MAP_PCM_GET_CORE_VOLTAGE_LEVEL();
	BACKUP_PARAMS->PCM_VCORE_LEVEL_RESTORE = MAP_PCM_GET_POWER_STATE();
	BACKUP_PARAMS->CS_DC0_FREQ_RESTORE = CS->CTL0 & CS_CTL0_DCORSEL_MASK;

	/* set parameters for flashing */
	success = MAP_PCM_SET_POWER_STATE(PCM_AM_LDO_VCORE0);

	/* Set Flash wait states to 2 */
	MAP_FLASH_CTL_SET_WAIT_STATE(FLASH_BANK0, 2);
	MAP_FLASH_CTL_SET_WAIT_STATE(FLASH_BANK1, 2);

	/* Set CPU speed to 24MHz */
	__cs_set_dco_frequency_range(CS_DCO_FREQUENCY_24);

	if (!success) {
		/* Indicate failed power switch */
		FLASH_LOADER->RETURN_CODE = FLASH_POWER_ERROR;
	} else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

/* Erase entire flash */
void msp432_flash_mass_erase(void)
{
	bool success = false;

	/* Allow flash writes */
	unlock_flash_sectors();

	/* Allow some mass erase repeats before timeout with error */
	int erase_repeats = FLASH_MAX_REPEATS;
	while (!success && (erase_repeats > 0)) {
		/* Mass erase with post-verify */
		success = MAP_FLASH_CTL_PERFORM_MASS_ERASE();
		erase_repeats--;
	}

	if (erase_repeats == 0)
		FLASH_LOADER->RETURN_CODE = FLASH_VERIFY_ERROR;
	else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;

	/* Block flash writes */
	lock_all_flash_sectors();
}

/* Erase one flash sector */
void msp432_flash_sector_erase(void)
{
	bool success = false;

	/* Allow flash writes */
	unlock_all_flash_sectors();

	/* Allow some sector erase repeats before timeout with error */
	int erase_repeats = FLASH_MAX_REPEATS;
	while (!success && (erase_repeats > 0)) {
		/* Sector erase with post-verify */
		success = MAP_FLASH_CTL_ERASE_SECTOR(FLASH_LOADER->DST_ADDRESS);
		erase_repeats--;
	}

	if (erase_repeats == 0)
		FLASH_LOADER->RETURN_CODE = FLASH_ERROR;
	else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;

	/* Block flash writes */
	lock_all_flash_sectors();
}

/* Write data to flash with the help of DriverLib */
void msp432_flash_write(void)
{
	bool success = false;

	/* Allow flash writes */
	unlock_all_flash_sectors();

	while (!(FLASH_LOADER->BUFFER1_STATUS_REGISTER & BUFFER_DATA_READY))
		;

	FLASH_LOADER->BUFFER1_STATUS_REGISTER |= BUFFER_ACTIVE;

	/* Program memory */
	success = program_device((uint32_t *)RAM_LOADER_BUFFER1,
		(void *)FLASH_LOADER->DST_ADDRESS, FLASH_LOADER->SRC_LENGTH);

	FLASH_LOADER->BUFFER1_STATUS_REGISTER &=
		~(BUFFER_ACTIVE | BUFFER_DATA_READY);

	/* Block flash writes */
	lock_all_flash_sectors();

	if (!success)
		FLASH_LOADER->RETURN_CODE = FLASH_ERROR;
	else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

/* Write data to flash with the help of DriverLib with auto-increment */
void msp432_flash_continous_write(void)
{
	bool buffer1_in_use = false;
	bool buffer2_in_use = false;
	uint32_t *src_address = NULL;
	bool success = false;

	uint32_t bytes_to_write = FLASH_LOADER->SRC_LENGTH;
	uint32_t write_package = 0;
	uint32_t start_addr = FLASH_LOADER->DST_ADDRESS;

	while (bytes_to_write > 0) {
		if (bytes_to_write > SRC_LENGTH_MAX) {
			write_package = SRC_LENGTH_MAX;
			bytes_to_write -= write_package;
		} else {
			write_package = bytes_to_write;
			bytes_to_write -= write_package;
		}
		unlock_all_flash_sectors();

		while (!(FLASH_LOADER->BUFFER1_STATUS_REGISTER & BUFFER_DATA_READY) &&
			!(FLASH_LOADER->BUFFER2_STATUS_REGISTER & BUFFER_DATA_READY))
			;

		if (FLASH_LOADER->BUFFER1_STATUS_REGISTER & BUFFER_DATA_READY) {
			FLASH_LOADER->BUFFER1_STATUS_REGISTER |= BUFFER_ACTIVE;
			src_address = (uint32_t *)RAM_LOADER_BUFFER1;
			buffer1_in_use = true;
		} else if (FLASH_LOADER->BUFFER2_STATUS_REGISTER & BUFFER_DATA_READY) {
			FLASH_LOADER->BUFFER2_STATUS_REGISTER |= BUFFER_ACTIVE;
			src_address = (uint32_t *)RAM_LOADER_BUFFER2;
			buffer2_in_use = true;
		}
		if (buffer1_in_use || buffer2_in_use) {
			success = program_device(src_address,
				(void *)start_addr, write_package);

			if (buffer1_in_use)
				P6->OUT &= ~BIT4; /* Program from B1 */
			else if (buffer2_in_use)
				P3->OUT &= ~BIT6; /* Program from B1 */

			start_addr += write_package;
		}
		if (buffer1_in_use) {
			FLASH_LOADER->BUFFER1_STATUS_REGISTER &=
				~(BUFFER_ACTIVE | BUFFER_DATA_READY);
			buffer1_in_use = false;
		} else if (buffer2_in_use) {
			FLASH_LOADER->BUFFER2_STATUS_REGISTER &=
				~(BUFFER_ACTIVE | BUFFER_DATA_READY);
			buffer2_in_use = false;
		}
		/* Block flash writes */
		lock_all_flash_sectors();

		if (!success) {
			FLASH_LOADER->RETURN_CODE = FLASH_ERROR;
			break;
		}
	}
	if (success)
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

/* Unlock Main/Info Flash sectors */
void unlock_flash_sectors(void)
{
	if (FLASH_LOADER->ERASE_PARAM & ERASE_MAIN) {
		MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_MAIN_MEMORY_SPACE_BANK0,
			0xFFFFFFFF);
		MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_MAIN_MEMORY_SPACE_BANK1,
			0xFFFFFFFF);
	}
	if (FLASH_LOADER->ERASE_PARAM & ERASE_INFO) {
		MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_INFO_MEMORY_SPACE_BANK0,
			FLASH_SECTOR0 | FLASH_SECTOR1);
		if (FLASH_LOADER->UNLOCK_BSL == UNLOCK_BSL_KEY)
			MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_INFO_MEMORY_SPACE_BANK1,
				FLASH_SECTOR0 | FLASH_SECTOR1);
	}
}

/* Unlock All Flash sectors */
void unlock_all_flash_sectors(void)
{
	MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_MAIN_MEMORY_SPACE_BANK0, 0xFFFFFFFF);
	MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_MAIN_MEMORY_SPACE_BANK1, 0xFFFFFFFF);
	MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_INFO_MEMORY_SPACE_BANK0,
		FLASH_SECTOR0 | FLASH_SECTOR1);
	if (FLASH_LOADER->UNLOCK_BSL == UNLOCK_BSL_KEY)
		MAP_FLASH_CTL_UNPROTECT_SECTOR(FLASH_INFO_MEMORY_SPACE_BANK1,
			FLASH_SECTOR0 | FLASH_SECTOR1);
}


/* Lock all Flash sectors */
void lock_all_flash_sectors(void)
{
	MAP_FLASH_CTL_PROTECT_SECTOR(FLASH_MAIN_MEMORY_SPACE_BANK0, 0xFFFFFFFF);
	MAP_FLASH_CTL_PROTECT_SECTOR(FLASH_MAIN_MEMORY_SPACE_BANK1, 0xFFFFFFFF);
	MAP_FLASH_CTL_PROTECT_SECTOR(FLASH_INFO_MEMORY_SPACE_BANK0,
		FLASH_SECTOR0 | FLASH_SECTOR1);
	MAP_FLASH_CTL_PROTECT_SECTOR(FLASH_INFO_MEMORY_SPACE_BANK1,
		FLASH_SECTOR0 | FLASH_SECTOR1);
}


/* Force DCO frequency range */
void __cs_set_dco_frequency_range(uint32_t dco_freq)
{
	/* Unlocking the CS Module */
	CS->KEY = CS_KEY_VAL;

	/* Resetting Tuning Parameters and Setting the frequency */
	CS->CTL0 = (CS->CTL0 & ~CS_CTL0_DCORSEL_MASK) | dco_freq;

	/* Locking the CS Module */
	CS->KEY = 0;
}

/* Exit flash programming */
void msp432_flash_exit(void)
{
	bool success = false;

	/* Restore modified registers, in reverse order */
	__cs_set_dco_frequency_range(CS_DCO_FREQUENCY_3);

	MAP_FLASH_CTL_SET_WAIT_STATE(FLASH_BANK0,
		BACKUP_PARAMS->BANK0_WAIT_RESTORE);
	MAP_FLASH_CTL_SET_WAIT_STATE(FLASH_BANK1,
		BACKUP_PARAMS->BANK1_WAIT_RESTORE);

	success = MAP_PCM_SET_POWER_STATE(BACKUP_PARAMS->PCM_VCORE_LEVEL_RESTORE);

	success &= MAP_PCM_SET_CORE_VOLTAGE_LEVEL(
		BACKUP_PARAMS->VCORE_LEVEL_RESTORE);

	__cs_set_dco_frequency_range(BACKUP_PARAMS->CS_DC0_FREQ_RESTORE);

	/* Point to vector table in Flash */
	SCB->VTOR = (uint32_t)0x00000000;

	if (!success)
		FLASH_LOADER->RETURN_CODE = FLASH_ERROR;
	else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

static bool program_device(void *src, void *dest, uint32_t length)
{
	return MAP_FLASH_CTL_PROGRAM_MEMORY(src, dest, length);
}

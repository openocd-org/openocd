/******************************************************************************
*
* Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#include "MSP432E4_FlashLibIf.h"

/* Local prototypes */
void msp432_flash_init(void);
void msp432_flash_mass_erase(void);
void msp432_flash_sector_erase(void);
void msp432_flash_write(void);
void msp432_flash_continous_write(void);
void msp432_flash_exit(void);

int main(void)
{
	/* Disable interrupts */
	__asm("  cpsid i");

	/* Halt watchdog */
	SYSCTL->RCGCWD &= ~(SYSCTL_RCGCWD_R1 + SYSCTL_RCGCWD_R0);

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
	SCB->VTOR = 0x20000000;
	FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

/* Erase entire flash */
void msp432_flash_mass_erase(void)
{
	bool success = false;

	/* Clear the flash access and error interrupts. */
	FLASH_CTRL->FCMISC = (FLASH_FCMISC_AMISC | FLASH_FCMISC_VOLTMISC |
						  FLASH_FCMISC_ERMISC | FLASH_FCMISC_PMISC);

	/* Trigger mass erase */
	FLASH_CTRL->FMC = FLASH_FMC_WRKEY | FLASH_FMC_MERASE;
	while (FLASH_CTRL->FMC & FLASH_FMC_MERASE)
		;

	/* Return an error if an access violation occurred. */
	success = !(FLASH_CTRL->FCRIS & (FLASH_FCRIS_ARIS | FLASH_FCRIS_VOLTRIS |
				FLASH_FCRIS_ERRIS));
	if (!success)
		FLASH_LOADER->RETURN_CODE = FLASH_VERIFY_ERROR;
	else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

/* Erase one flash sector */
void msp432_flash_sector_erase(void)
{
	bool success = false;

	/* Clear the flash access and error interrupts. */
	FLASH_CTRL->FCMISC = (FLASH_FCMISC_AMISC | FLASH_FCMISC_VOLTMISC |
						  FLASH_FCMISC_ERMISC | FLASH_FCMISC_PMISC);

	/* Set 16kB aligned flash page address to be erased (16kB block) */
	FLASH_CTRL->FMA = FLASH_LOADER->DST_ADDRESS;
	/* Trigger sector erase (erase flash page) */
	FLASH_CTRL->FMC = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;
	while (FLASH_CTRL->FMC & FLASH_FMC_ERASE)
		;

	/* Return an error if an access violation occurred. */
	success = !(FLASH_CTRL->FCRIS & (FLASH_FCRIS_ARIS | FLASH_FCRIS_VOLTRIS |
				FLASH_FCRIS_ERRIS));

	if (!success)
		FLASH_LOADER->RETURN_CODE = FLASH_ERROR;
	else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

/* Write data to flash */
void msp432_flash_continous_write(void)
{
	bool buffer1_in_use = false;
	bool buffer2_in_use = false;
	uint32_t *src_address = NULL;
	bool success = true;
	uint32_t i = 0;
	uint32_t address = FLASH_LOADER->DST_ADDRESS;
	uint32_t data_to_write = FLASH_LOADER->SRC_LENGTH;
	int32_t write_package = 0;

	/* Clear the flash access and error interrupts. */
	FLASH_CTRL->FCMISC = (FLASH_FCMISC_AMISC | FLASH_FCMISC_VOLTMISC |
		FLASH_FCMISC_INVDMISC | FLASH_FCMISC_PROGMISC | FLASH_FCMISC_PMISC);
	do {
		if (data_to_write > SRC_LENGTH_MAX) {
			write_package = SRC_LENGTH_MAX;
			data_to_write -= write_package;
		} else {
			write_package = data_to_write;
			data_to_write -= write_package;
		}
		while (!(FLASH_LOADER->BUFFER1_STATUS_REGISTER & BUFFER_DATA_READY) &&
			!(FLASH_LOADER->BUFFER2_STATUS_REGISTER & BUFFER_DATA_READY))
			;

		if (FLASH_LOADER->BUFFER1_STATUS_REGISTER & BUFFER_DATA_READY) {
			FLASH_LOADER->BUFFER1_STATUS_REGISTER |= BUFFER_ACTIVE;
			src_address  = (uint32_t *) RAM_LOADER_BUFFER1;
			buffer1_in_use = true;
		} else if (FLASH_LOADER->BUFFER2_STATUS_REGISTER & BUFFER_DATA_READY) {
			FLASH_LOADER->BUFFER2_STATUS_REGISTER |= BUFFER_ACTIVE;
			src_address  = (uint32_t *) RAM_LOADER_BUFFER2;
			buffer2_in_use = true;
		}

		/*
		 * The flash hardware can only write complete words to flash. If
		 * an unaligned address is passed in, we must do a read-modify-write
		 * on a word with enough bytes to align the rest of the buffer. And
		 * if less than a whole word remains at the end, we must also do a
		 * read-modify-write on a final word to finish up.
		 */
		if (0 != (address & 0x3)) {
			uint32_t head;
			uint8_t *ui8head = (uint8_t *)&head;
			uint8_t *buffer = (uint8_t *)src_address;

			/* Get starting offset for data to write (will be 1 to 3) */
			uint32_t head_offset = address & 0x03;

			/* Get the aligned address to write this first word to */
			uint32_t head_address = address & 0xfffffffc;

			/* Retrieve what is already in flash at the head address */
			head = *(uint32_t *)head_address;

			/* Substitute in the new data to write */
			while ((write_package > 0) && (head_offset < 4)) {
				ui8head[head_offset] = *buffer;
				head_offset++;
				address++;
				buffer++;
				write_package--;
			}
			src_address = (uint32_t *)buffer;

			FLASH_CTRL->FMD = head;
			FLASH_CTRL->FMA = head_address;
			FLASH_CTRL->FMC = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;

			/* Wait until the word has been programmed. */
			while (FLASH_CTRL->FMC & FLASH_FMC_WRITE)
				;

			/* Return an error if an access violation occurred. */
			success = !(FLASH_CTRL->FCRIS & (FLASH_FCRIS_ARIS |
				FLASH_FCRIS_ERIS | FLASH_FCRIS_INVDRIS | FLASH_FCRIS_PROGRIS));
		}

		/* Program a word at a time until aligned on 32-word boundary */
		while ((write_package >= 4) && ((address & 0x7f) != 0) && success) {
			FLASH_CTRL->FMD = *src_address++;
			FLASH_CTRL->FMA = address;
			FLASH_CTRL->FMC = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;

			/* Wait until the word has been programmed. */
			while (FLASH_CTRL->FMC & FLASH_FMC_WRITE)
				;

			/* Prepare for next word to write */
			write_package -= 4;
			address += 4;

			/* Return an error if an access violation occurred. */
			success = !(FLASH_CTRL->FCRIS & (FLASH_FCRIS_ARIS |
				FLASH_FCRIS_ERIS | FLASH_FCRIS_INVDRIS | FLASH_FCRIS_PROGRIS));
		}

		/* Program data in 32-word blocks */
		while ((write_package >= 32) && success) {
			/* Loop over the words in this 32-word block. */
			i = 0;
			do {
				FLASH_CTRL->FWBN[i] = *src_address++;
				write_package -= 4;
				i++;
			} while ((write_package > 0) && (i < 32));
			FLASH_CTRL->FMA = address;
			FLASH_CTRL->FMC2 = FLASH_FMC_WRKEY | FLASH_FMC2_WRBUF;

			/* Wait until the write buffer has been programmed. */
			while (FLASH_CTRL->FMC2 & FLASH_FMC2_WRBUF)
				;

			/* Increment destination address by words written */
			address += 128;

			/* Return an error if an access violation occurred. */
			success = !(FLASH_CTRL->FCRIS & (FLASH_FCRIS_ARIS |
				FLASH_FCRIS_ERIS | FLASH_FCRIS_INVDRIS | FLASH_FCRIS_PROGRIS));
		}

		/* Program a word at a time on left over data */
		while ((write_package >= 4) && success) {
			FLASH_CTRL->FMD = *src_address++;
			FLASH_CTRL->FMA = address;
			FLASH_CTRL->FMC = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;

			/* Wait until the word has been programmed. */
			while (FLASH_CTRL->FMC & FLASH_FMC_WRITE)
				;

			/* Prepare for next word to write */
			write_package -= 4;
			address += 4;

			/* Return an error if an access violation occurred. */
			success = !(FLASH_CTRL->FCRIS & (FLASH_FCRIS_ARIS |
				FLASH_FCRIS_ERIS | FLASH_FCRIS_INVDRIS | FLASH_FCRIS_PROGRIS));
		}

		if ((write_package > 0) && success) {
			uint32_t tail;
			uint8_t *ui8tail = (uint8_t *)&tail;
			uint8_t *buffer = (uint8_t *)src_address;

			/* Set starting offset for data to write */
			uint32_t tail_offset = 0;

			/* Get the address to write this last word to */
			uint32_t tail_address = address;

			/* Retrieve what is already in flash at the tail address */
			tail = *(uint32_t *)address;

			/* Substitute in the new data to write */
			while (write_package > 0) {
				ui8tail[tail_offset] = *buffer;
				tail_offset++;
				address++;
				buffer++;
				write_package--;
			}

			FLASH_CTRL->FMD = tail;
			FLASH_CTRL->FMA = tail_address;
			FLASH_CTRL->FMC = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;

			/* Wait until the word has been programmed. */
			while (FLASH_CTRL->FMC & FLASH_FMC_WRITE)
				;

			/* Return an error if an access violation occurred. */
			success = !(FLASH_CTRL->FCRIS & (FLASH_FCRIS_ARIS |
				FLASH_FCRIS_ERIS | FLASH_FCRIS_INVDRIS | FLASH_FCRIS_PROGRIS));
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
	} while (success && data_to_write);

	if (!success)
		FLASH_LOADER->RETURN_CODE = FLASH_ERROR;
	else
		FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

/* Exit flash programming */
void msp432_flash_exit(void)
{
	SCB->VTOR = 0x00000000;
	FLASH_LOADER->RETURN_CODE = FLASH_SUCCESS;
}

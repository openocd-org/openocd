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
#include "flashloader.h"
#include "flash.h"

/* Array holding erased state of the flash sectors. */
static bool g_is_erased[FLASH_MAX_SECTOR_COUNT];

extern uint8_t g_retain_buf[];

uint32_t flashloader_init(struct flash_params *params, uint8_t *buf1,
	uint8_t *buf2)
{
	/* Initialize params buffers */
	memset((void *)params, 0, 2 * sizeof(struct flash_params));
	params[0].buf_addr = (uint32_t)buf1;
	params[1].buf_addr = (uint32_t)buf2;

	/* Mark all sectors at "not erased" */
	memset(g_is_erased, false, sizeof(g_is_erased));

	return STATUS_OK;
}

uint32_t flashloader_erase_and_program(uint8_t *src, uint32_t address,
	uint32_t byte_count)
{
	if (byte_count > BUFFER_LEN)
		return STATUS_FAILED_INVALID_ARGUMENTS;

	/* Erase affected sectors */
	uint32_t status = flashloader_erase_sectors(address, byte_count);

	if (status != STATUS_OK)
		return status;

	/* Program data */
	status = flashloader_program(src, address, byte_count);

	return status;
}

uint32_t flashloader_program_with_retain(uint8_t *src, uint32_t address,
	uint32_t byte_count)
{
#if (BUFFER_LEN > FLASH_ERASE_SIZE)
#error Buffer size cannot be larger than the flash sector size!
#endif

	uint32_t first_sector_idx;
	uint32_t last_sector_idx;
	uint32_t status = STATUS_OK;
	uint32_t i;

	first_sector_idx = flashloader_address_to_sector(address);
	last_sector_idx = flashloader_address_to_sector(address + byte_count - 1);

	/* Mark all sectors as "not erased" before starting */
	memset(g_is_erased, false, sizeof(g_is_erased));

	uint32_t sec_offset = address % FLASH_ERASE_SIZE;
	uint32_t curr_count;
	uint32_t src_offset = 0;

	for (i = first_sector_idx; i <= last_sector_idx; i++) {

		/* Chop off at sector boundary if data goes into the next sector. */
		curr_count = byte_count;
		if ((address + byte_count) > ((i+1) * FLASH_ERASE_SIZE))
			curr_count -= (address + byte_count) % FLASH_ERASE_SIZE;

		/* Copy flash sector to retain buffer */
		memcpy(g_retain_buf, (void *)(i * FLASH_ERASE_SIZE), FLASH_ERASE_SIZE);

		/* Copy data buffer to retain buffer */
		memcpy(&g_retain_buf[sec_offset], &src[src_offset], curr_count);

		/* Erase and program from retain buffer */
		status = flashloader_erase_and_program(g_retain_buf,
					(i * FLASH_ERASE_SIZE), FLASH_ERASE_SIZE);
		if (status != STATUS_OK)
			return status;

		address += curr_count;
		sec_offset = address % FLASH_ERASE_SIZE;
		byte_count -= curr_count;
		src_offset += curr_count;
	}

	return status;
}

uint32_t flashloader_erase_all(void)
{
	if (flash_bank_erase(true) != FAPI_STATUS_SUCCESS)
		return STATUS_FAILED_ERASE_ALL;

	memset(g_is_erased, true, sizeof(g_is_erased));

	return STATUS_OK;
}

uint32_t flashloader_erase_sectors(uint32_t address, uint32_t byte_count)
{
	uint32_t first_sector_idx;
	uint32_t last_sector_idx;
	uint32_t status;
	uint32_t idx;

	/* Floor address to the start of the sector and convert to sector number */
	first_sector_idx = flashloader_address_to_sector(address);
	last_sector_idx = flashloader_address_to_sector(address + byte_count - 1);

	/*  Erase given sector(s) */
	for (idx = first_sector_idx; idx <= last_sector_idx; idx++) {

		/* Only erase sectors that haven't already been erased */
		if (g_is_erased[idx] == false) {
			status = flash_sector_erase(idx * FLASH_ERASE_SIZE);
			if (status != FAPI_STATUS_SUCCESS) {
				status = (STATUS_FAILED_SECTOR_ERASE |
						((idx << STATUS_EXT_INFO_S) & STATUS_EXT_INFO_M) |
						((status << STATUS_ROM_CODE_S) & STATUS_ROM_CODE_M));
				return status;
			}
			g_is_erased[idx] = true;
		}
	}

	return STATUS_OK;
}

uint32_t flashloader_program(uint8_t *src, uint32_t address,
	uint32_t byte_count)
{
	uint32_t status = flash_program(src, address, byte_count);
	if (status != FAPI_STATUS_SUCCESS) {
		status = (STATUS_FAILED_PROGRAM |
					((status << STATUS_ROM_CODE_S) & STATUS_ROM_CODE_M));
	}

	return STATUS_OK;
}

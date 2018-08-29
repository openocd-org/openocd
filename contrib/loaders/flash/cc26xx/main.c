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

/* Data buffers used by host to communicate with flashloader */

/* Flashloader parameter structure. */
__attribute__ ((section(".buffers.g_cfg")))
volatile struct flash_params g_cfg[2];
/* Data buffer 1. */
__attribute__ ((section(".buffers.g_buf1")))
uint8_t g_buf1[BUFFER_LEN];
/* Data buffer 2. */
__attribute__ ((section(".buffers.g_buf2")))
uint8_t g_buf2[BUFFER_LEN];

/* Buffer used for program with retain feature */
__attribute__ ((section(".buffers.g_retain_buf")))
uint8_t g_retain_buf[BUFFER_LEN];

uint32_t g_curr_buf; /* Current buffer used. */
uint32_t g_vims_ctl; /* Saved flash cache state. */

/******************************************************************************
*
* This function stores the current VIMS configuration before
* - disabling VIMS flash cache
* - flushing the flash line buffers.
*
* Note Not using driverlib calls because it requires using "NO_ROM" define in
*      order to work for both Cha. R1 and R2 using the same code. Manually
*      doing the steps to minimize code footprint.
*
******************************************************************************/
static void disable_flash_cache()
{
	/* 1. Make sure VIMS is not currently changing mode (VIMS:STAT register) */
	while ((HWREG(0x40034000) & 0x00000008) == 0x8)
		;

	/* Save current VIMS:CTL state */
	g_vims_ctl = HWREG(0x40034004);

	/* 2. Set VIMS mode to OFF and disable flash line buffers */
	uint32_t new_vims_ctl = g_vims_ctl | 0x33;
	HWREG(0x40034004) = new_vims_ctl;

	/* 3. Wait for VIMS to have changed mode (VIMS:STAT register) */
	while ((HWREG(0x40034000) & 0x00000008) == 0x8)
		;
}

/******************************************************************************
*
* This function restores the VIMS configuration saved off by
* disable_flash_cache().
*
* Note Not using driverlib calls because it requires using "NO_ROM" define in
*      order to work for both Cha. R1 and R2 using the same code. Manually
*      doing the steps to minimize code footprint.
*
******************************************************************************/
static void restore_cache_state()
{
	HWREG(0x40034004) = g_vims_ctl;

	/* Wait for VIMS to have changed mode (VIMS:STAT register) */
	while ((HWREG(0x40034000) & 0x00000008) == 0x8)
		;
}

/******************************************************************************
*
* CC13xx/CC26xx flashloader main function.
*
******************************************************************************/
int main(void)
{
	flashloader_init((struct flash_params *)g_cfg, g_buf1, g_buf2);

	g_curr_buf = 0; /* start with the first buffer */
	uint32_t status;

	while (1) {
		/* Wait for host to signal buffer is ready */
		while (g_cfg[g_curr_buf].full == BUFFER_EMPTY)
			;

		disable_flash_cache();

		/* Perform requested task */
		switch (g_cfg[g_curr_buf].cmd) {
			case CMD_ERASE_ALL:
				status = flashloader_erase_all();
				break;
			case CMD_PROGRAM:
				status =
					flashloader_program(
						(uint8_t *)g_cfg[g_curr_buf].buf_addr,
						g_cfg[g_curr_buf].dest, g_cfg[g_curr_buf].len);
				break;
			case CMD_ERASE_AND_PROGRAM:
				status =
					flashloader_erase_and_program(
						(uint8_t *)g_cfg[g_curr_buf].buf_addr,
						g_cfg[g_curr_buf].dest, g_cfg[g_curr_buf].len);
				break;
			case CMD_ERASE_AND_PROGRAM_WITH_RETAIN:
				status =
					flashloader_program_with_retain(
						(uint8_t *)g_cfg[g_curr_buf].buf_addr,
						g_cfg[g_curr_buf].dest, g_cfg[g_curr_buf].len);
				break;
			case CMD_ERASE_SECTORS:
				status =
					flashloader_erase_sectors(g_cfg[g_curr_buf].dest,
						g_cfg[g_curr_buf].len);
				break;
			default:
				status = STATUS_FAILED_UNKNOWN_COMMAND;
				break;
		}

		restore_cache_state();

		/* Enter infinite loop on error condition */
		if (status != STATUS_OK) {
			g_cfg[g_curr_buf].full = status;
			while (1)
				;
		}

		/* Mark current task complete, and begin looking at next buffer */
		g_cfg[g_curr_buf].full = BUFFER_EMPTY;
		g_curr_buf ^= 1;
	}
}

void _exit(int status)
{
	/* Enter infinite loop on hitting an exit condition */
	(void)status; /* Unused parameter */
	while (1)
		;
}

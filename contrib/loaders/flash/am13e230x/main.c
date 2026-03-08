// SPDX-License-Identifier: GPL-2.0-or-later

/******************************************************************************
 *
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * AM13E230X Flash Loader Overview:
 * --------------------------------
 * This is a bare-metal flash programming algorithm for the AM13E230X
 * microcontroller. OpenOCD loads the compiled binary into device SRAM
 * at 0x20000000 and starts execution. OpenOCD then streams flash data
 * into two sector-sized (2 KB) SRAM buffers via the debug probe using
 * a shared-memory handshake protocol identical to the CC26xx loader.
 *
 * The algorithm runs an infinite loop, waiting for OpenOCD to fill a buffer
 * and signal readiness. It then erases the target sector and programs the
 * buffer contents into flash. Two buffers are used in ping-pong fashion so
 * that OpenOCD can fill the next buffer while the current one is being
 * programmed.
 *
 * Shared-Memory Protocol
 * ----------------------
 * Communication uses two parameter blocks and two data buffers at fixed
 * SRAM addresses:
 *
 *   Address       Content
 *   ----------    -------
 *   0x20002000    flash_params[0]  (20 bytes)
 *   0x20002014    flash_params[1]  (20 bytes)
 *   0x20002100    buffer[0]        (2048 bytes)
 *   0x20002900    buffer[1]        (2048 bytes)
 *
 *   struct flash_params {
 *       uint32_t dest;      // Flash destination address
 *       uint32_t len;       // Number of bytes in buffer
 *       uint32_t cmd;       // Command (see below)
 *       uint32_t status;    // BUFFER_FULL=0xFFFFFFFF / BUFFER_EMPTY=0x0
 *       uint32_t buf_addr;  // Address of associated data buffer
 *   };
 *
 * Commands:
 *   0  CMD_NO_ACTION           Do nothing
 *   1  CMD_PROGRAM             Program buffer contents to flash
 *   2  CMD_ERASE_AND_PROGRAM   Erase sector, then program
 *   3  CMD_ERASE_SECTORS       Erase sectors in range [dest, dest+len)
 *
 * Sequence (per sector):
 *   1. OpenOCD writes sector data to buffer[N]
 *   2. OpenOCD writes dest, len, cmd to params[N]
 *   3. OpenOCD writes BUFFER_FULL (0xFFFFFFFF) to params[N].status
 *   4. Algorithm sees BUFFER_FULL, performs erase + program
 *   5. Algorithm writes BUFFER_EMPTY (0x0) to params[N].status
 *   6. OpenOCD polls params[N].status, sees EMPTY, proceeds to next
 *   7. Meanwhile, algorithm moves to params[N^1] (ping-pong)
 *
 * On error, the algorithm writes an error code (non-zero, non-0xFFFFFFFF)
 * to params[N].status and enters an infinite loop.
 *
 *****************************************************************************/

#include <stdint.h>
#include "flash.h"

/* Handshake values */
#define BUFFER_EMPTY    0x00000000
#define BUFFER_FULL     0xFFFFFFFF

/* Commands */
#define CMD_NO_ACTION           0
#define CMD_PROGRAM             1
#define CMD_ERASE_AND_PROGRAM   2
#define CMD_ERASE_SECTORS       3

/* Status codes written on error */
#define STATUS_OK                       0x00000000
#define STATUS_FAILED_ERASE             0x00000101
#define STATUS_FAILED_PROGRAM           0x00000102
#define STATUS_FAILED_UNKNOWN_CMD       0x00000103
#define STATUS_FAILED_SEM_ACQUIRE       0x00000104
#define STATUS_FAILED_SEM_RELEASE       0x00000105

/* Parameter block -- must match the OpenOCD driver's struct */
struct __attribute__((packed)) flash_params {
	uint32_t dest;      /* Flash destination address */
	uint32_t len;       /* Number of bytes */
	uint32_t cmd;       /* Command */
	uint32_t status;    /* Handshake: BUFFER_FULL / BUFFER_EMPTY */
	uint32_t buf_addr;  /* Address of data buffer */
};

/* Params and buffers placed at fixed addresses by the linker script */
__attribute__((section(".buffers.params")))
volatile struct flash_params g_params[2];

__attribute__((section(".buffers.buf0")))
uint8_t g_buf0[SECTOR_SIZE];

__attribute__((section(".buffers.buf1")))
uint8_t g_buf1[SECTOR_SIZE];

static int do_erase_and_program(volatile struct flash_params *p)
{
	/* Erase the target sector */
	if (flash_sector_erase(p->dest) != 0)
		return STATUS_FAILED_ERASE;

	/* Program the buffer contents */
	if (flash_write_sector((const uint8_t *)p->buf_addr, p->dest, p->len) != 0)
		return STATUS_FAILED_PROGRAM;

	return STATUS_OK;
}

static int do_program(volatile struct flash_params *p)
{
	if (flash_write_sector((const uint8_t *)p->buf_addr, p->dest, p->len) != 0)
		return STATUS_FAILED_PROGRAM;

	return STATUS_OK;
}

static int do_erase_sectors(volatile struct flash_params *p)
{
	uint32_t addr = p->dest;
	uint32_t len = p->len;

	/* Guard against dest+len wrapping past UINT32_MAX */
	if (len > (UINT32_MAX - addr))
		return STATUS_FAILED_ERASE;

	uint32_t end = addr + len;

	while (addr < end) {
		if (flash_sector_erase(addr) != 0)
			return STATUS_FAILED_ERASE;
		addr += SECTOR_SIZE;
	}

	return STATUS_OK;
}

int main(void)
{
	uint32_t curr_buf = 0;
	uint32_t status;

	/* Initialize buffer pointers so OpenOCD can read them */
	g_params[0].buf_addr = (uint32_t)g_buf0;
	g_params[1].buf_addr = (uint32_t)g_buf1;
	g_params[0].status = BUFFER_EMPTY;
	g_params[1].status = BUFFER_EMPTY;

	/* Acquire GSC flash semaphore — must be done by the CPU (not the
	 * debugger) because the flash controller checks FLSEMSTAT.DBGACC
	 * to verify the access source matches the semaphore holder. */
	if (flash_request_gsc_semaphore() != 0) {
		g_params[0].status = STATUS_FAILED_SEM_ACQUIRE;
		while (1)
			;
	}

#ifdef ALGO_DRY_RUN
	/* Dry-run mode: no flash operations, just ACK every command.
	 * Used to verify the shared-memory handshake works. */
	while (1) {
		while (g_params[curr_buf].status == BUFFER_EMPTY)
			;
		g_params[curr_buf].status = BUFFER_EMPTY;
		curr_buf ^= 1;
	}
#endif

	while (1) {
		/* Wait for OpenOCD to signal that buffer is ready */
		while (g_params[curr_buf].status == BUFFER_EMPTY)
			;

		/* Dispatch command */
		switch (g_params[curr_buf].cmd) {
		case CMD_ERASE_AND_PROGRAM:
			status = do_erase_and_program(&g_params[curr_buf]);
			break;
		case CMD_PROGRAM:
			status = do_program(&g_params[curr_buf]);
			break;
		case CMD_ERASE_SECTORS:
			status = do_erase_sectors(&g_params[curr_buf]);
			break;
		default:
			status = STATUS_FAILED_UNKNOWN_CMD;
			break;
		}

		if (status != STATUS_OK) {
			/* Report error and halt */
			g_params[curr_buf].status = status;
			if (flash_clear_gsc_semaphore() != 0)
				g_params[curr_buf].status = STATUS_FAILED_SEM_RELEASE;
			while (1)
				;
		}

		/* Signal completion */
		g_params[curr_buf].status = BUFFER_EMPTY;

		/* Swap to the other buffer */
		curr_buf ^= 1;
	}
}

void _exit(int status)
{
	(void)status;
	while (1)
		;
}

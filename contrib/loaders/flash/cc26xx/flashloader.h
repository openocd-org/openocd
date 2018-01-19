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

#ifndef OPENOCD_LOADERS_FLASH_CC26XX_FLASHLOADER_H
#define OPENOCD_LOADERS_FLASH_CC26XX_FLASHLOADER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "flash.h"

/* Number of elements in an array */
#define NELEMS(a) (sizeof(a) / sizeof(a[0]))

struct __attribute__((__packed__)) flash_params {
	uint32_t dest;     /* Destination address in flash */
	uint32_t len;      /* Number of bytes */
	uint32_t cmd;      /* Command */
	uint32_t full;     /* Handshake signal. Is buffer ready? */
	uint32_t buf_addr; /* Address of data buffer. */
};

typedef enum {
	CMD_NO_ACTION = 0,                     /* No action, default value */
	CMD_ERASE_ALL = 1,                     /* Erase all unprotected sectors */
	CMD_PROGRAM = 2,                       /* Program data */
	CMD_ERASE_AND_PROGRAM = 3,             /* Erase and program data */
	CMD_ERASE_AND_PROGRAM_WITH_RETAIN = 4, /* Erase and program, but retain */
										   /* sector data outside given range */
	CMD_ERASE_SECTORS = 5                  /* Erase unprotected sectors */
} flash_commands_t;

typedef enum {
	BUFFER_EMPTY = 0x0,      /* No data in buffer, flags last task complete */
	BUFFER_FULL = 0xFFFFFFFF /* Buffer has data, flags next task to start */
} flash_handshake_t;

#define STATUS_FLASHLOADER_STATUS_M 0x0000FFFF
#define STATUS_FLASHLOADER_STATUS_S 0
#define STATUS_ROM_CODE_M           0x00FF0000
#define STATUS_ROM_CODE_S           16
#define STATUS_EXT_INFO_M           0xFF000000
#define STATUS_EXT_INFO_S           24

typedef enum {
	STATUS_OK = 0,
	STATUS_FAILED_ERASE_ALL = 0x101,
	STATUS_FAILED_SECTOR_ERASE = 0x102,
	STATUS_FAILED_PROGRAM = 0x103,
	STATUS_FAILED_INVALID_ARGUMENTS = 0x104,
	STATUS_FAILED_UNKNOWN_COMMAND = 0x105,
} flash_status_t;

/* The buffer size used by the flashloader. The size of 1 flash sector. */
#define BUFFER_LEN FLASH_ERASE_SIZE

/*
* This function initializes the flashloader. The application must
* allocate memory for the two data buffers and the flash_params structures.
*
* params Pointer an flash_params array with 2 elements.
* buf1   Pointer to data buffer 1
* buf2   Pointer to data buffer 2
*
* Returns STATUS_OK
*
*/
extern uint32_t flashloader_init(struct flash_params *params, uint8_t *buf1,
	uint8_t *buf2);

/*
* Erase and program the necessary sectors. Data outside the given
* range will be deleted.
*
* src        Pointer to buffer containing the data.
* address    Start address in device flash
* byte_count The number of bytes to program
*
* Returns STATUS_OK on success. For status on failure:
* See flashloader_program() and flashloader_erase_sectors().
*
*/
extern uint32_t flashloader_erase_and_program(uint8_t *src, uint32_t address,
	uint32_t byte_count);

/*
* Erase and program the device sectors. Data outside the given
* data range will be kept unchanged.
*
* src        Pointer to buffer containing the data.
* address    Start address in device flash
* byte_count The number of bytes to program
*
* Returns STATUS_OK on success. For status on failure:
* See flashloader_program() and flashloader_erase_sectors().
*
*/
extern uint32_t flashloader_program_with_retain(uint8_t *src, uint32_t address,
	uint32_t byte_count);

/*
* Erases all flash sectors (that are not write-protected).
*
* Returns STATUS_OK on success.
*
*/
extern uint32_t flashloader_erase_all(void);

/*
* Erases the flash sectors affected by the given range.
*
* This function only erases sectors that are not already erased.
*
* start_addr The first address in the range.
* byte_count The number of bytes in the range.
*
* Returns STATUS_OK on success. Returns a combined status on failure:
* [31:24] The sector that failed.
* [23:16] ROM function status code. 0 means success.
* [16: 0] STATUS_FAILED_SECTOR_ERASE
*
*/
extern uint32_t flashloader_erase_sectors(uint32_t start_addr,
	uint32_t byte_count);

/*
* Program the given range.
*
* This function does not erase anything, it assumes the sectors are ready to
* be programmed.
*
* src        Pointer to buffer containing the data.
* address    Start address in device flash
* byte_count The number of bytes to program
*
* Returns STATUS_OK on success. Returns a combined status value on failure:
* [31:16] ROM function status code. 0 means success.
* [15:0 ] STATUS_FAILED_PROGRAM
*
*/
extern uint32_t flashloader_program(uint8_t *src, uint32_t address,
	uint32_t byte_count);

/*
* Convert the input address into a sector number.
*
* address The address.
*
* Returns the flash sector which the address resides in. The first sector
* is sector 0.
*
*/
static inline uint32_t flashloader_address_to_sector(uint32_t address)
	{ return (address / FLASH_ERASE_SIZE); };

#endif /* #ifndef OPENOCD_LOADERS_FLASH_CC26XX_FLASHLOADER_H */

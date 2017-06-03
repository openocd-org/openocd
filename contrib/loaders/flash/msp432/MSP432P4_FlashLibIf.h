/******************************************************************************
*
* Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef OPENOCD_LOADERS_FLASH_MSP432_MSP432P4_FLASHLIBIF_H
#define OPENOCD_LOADERS_FLASH_MSP432_MSP432P4_FLASHLIBIF_H

#include <stddef.h>
#include <stdint.h>

/* RAM loader */
static const uint32_t RAM_LOADER_START   = 0x01000000u; /* Code space */
static const uint32_t RAM_LOADER_MAIN    = 0x01000110u; /* Code space */
static const uint32_t RAM_LOADER_BUFFER1 = 0x20002000u; /* SBUS data space */
static const uint32_t RAM_LOADER_BUFFER2 = 0x20003000u; /* SBUS data space */
static const uint32_t RAM_LOADER_STACK   = 0x20002000u; /* SBUS data space */

/* Address for flash function to be executed */
static const uint32_t FLASH_FUNCTION_ADDRESS = 0x20000150u;

enum flash_command {
	FLASH_NO_COMMAND = 0,
	FLASH_MASS_ERASE = 1,
	FLASH_SECTOR_ERASE = 2,
	FLASH_PROGRAM = 4,
	FLASH_INIT = 8,
	FLASH_EXIT = 16,
	FLASH_CONTINUOUS_PROGRAM = 32
};

/* Address for algorithm program and flash buffer */
static const uint32_t DST_ADDRESS             = 0x2000015Cu;
static const uint32_t SRC_LENGTH_ADDRESS      = 0x20000160u;
static const uint32_t BUFFER1_STATUS_REGISTER = 0x20000164u;
static const uint32_t BUFFER2_STATUS_REGISTER = 0x20000168u;
static const uint32_t BUFFER_INACTIVE         = 0x00000000u;
static const uint32_t BUFFER_ACTIVE           = 0x00000001u;
static const uint32_t BUFFER_DATA_READY       = 0x00000010u;
static const size_t   SRC_LENGTH_MAX          = 4096u;

/* erase options */
static const uint32_t ERASE_PARAM_ADDRESS = 0x2000016Cu;
static const uint32_t ERASE_MAIN          = 0x00000001u;
static const uint32_t ERASE_INFO          = 0x00000002u;

/* Unlock BSL */
static const uint32_t UNLOCK_BSL_ADDRESS  = 0x20000170u;
static const uint32_t LOCK_BSL_KEY        = 0x00000000u;
static const uint32_t UNLOCK_BSL_KEY      = 0x0000000Bu;

/* Address for return code */
static const uint32_t RETURN_CODE_ADDRESS = 0x20000154u;

/* Return codes */
static const uint32_t FLASH_BUSY          = 0x00000001u;
static const uint32_t FLASH_SUCCESS       = 0x00000ACEu;
static const uint32_t FLASH_ERROR         = 0x0000DEADu;
static const uint32_t FLASH_TIMEOUT_ERROR = 0xDEAD0000u;
static const uint32_t FLASH_VERIFY_ERROR  = 0xDEADDEADu;
static const uint32_t FLASH_WRONG_COMMAND = 0x00000BADu;
static const uint32_t FLASH_POWER_ERROR   = 0x00DEAD00u;

/* Device ID address */
static const uint32_t DEVICE_ID_ADDRESS = 0x0020100Cu;
static const uint32_t PC_REGISTER       = 15u;
static const uint32_t SP_REGISTER       = 13u;

/* CS silicon and boot code revisions */
static const uint32_t SILICON_REV_ADDRESS = 0x00201010u;
static const uint32_t SILICON_REV_A       = 0x00000041u;
static const uint32_t SILICON_REV_B       = 0x00000042u;
static const uint32_t SILICON_REV_C       = 0x00000043u;
static const uint32_t SILICON_REV_D       = 0x00000044u;
static const uint32_t SILICON_REV_E       = 0x00000045u;
static const uint32_t SILICON_REV_F       = 0x00000046u;
static const uint32_t SILICON_REV_G       = 0x00000047u;
static const uint32_t SILICON_REV_H       = 0x00000048u;
static const uint32_t SILICON_REV_I       = 0x00000049u;
static const uint32_t SILICON_REV_B_WRONG = 0x00004100u;

struct flash_interface {
	volatile uint32_t FLASH_FUNCTION;
	volatile uint32_t RETURN_CODE;
	volatile uint32_t _RESERVED0;
	volatile uint32_t DST_ADDRESS;
	volatile uint32_t SRC_LENGTH;
	volatile uint32_t BUFFER1_STATUS_REGISTER;
	volatile uint32_t BUFFER2_STATUS_REGISTER;
	volatile uint32_t ERASE_PARAM;
	volatile uint32_t UNLOCK_BSL;
};

#define FLASH_LOADER_BASE ((uint32_t)0x20000150u)
#define FLASH_LOADER      ((struct flash_interface *) FLASH_LOADER_BASE)

#endif /* OPENOCD_LOADERS_FLASH_MSP432_MSP432P4_FLASHLIBIF_H */

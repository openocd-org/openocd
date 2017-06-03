/******************************************************************************
*
* Copyright (C) 2012-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef OPENOCD_LOADERS_FLASH_MSP432_MSP432P401X_H
#define OPENOCD_LOADERS_FLASH_MSP432_MSP432P401X_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define __MCU_HAS_FLCTL__ /* Module FLCTL is available */

/* Device and peripheral memory map */
#define FLASH_BASE  ((uint32_t)0x00000000)     /* Flash memory start address */
#define SRAM_BASE   ((uint32_t)0x20000000)     /* SRAM memory start address */
#define PERIPH_BASE ((uint32_t)0x40000000)     /* Peripherals start address */
#define CS_BASE     (PERIPH_BASE + 0x00010400) /* Address of module CS regs. */
#define DIO_BASE    (PERIPH_BASE + 0x00004C00) /* Address of module DIO regs. */

/* Register map for Clock Signal peripheral (CS) */
struct cs {
	volatile uint32_t KEY;  /* Key Register */
	volatile uint32_t CTL0; /* Control 0 Register */
	volatile uint32_t CTL1; /* Control 1 Register */
	volatile uint32_t CTL2; /* Control 2 Register */
	volatile uint32_t CTL3; /* Control 3 Register */
};

/* Register map for DIO port (odd interrupt) */
struct dio_port_odd_int {
	volatile uint8_t IN;  /* Port Input */
	uint8_t RESERVED0;
	volatile uint8_t OUT; /* Port Output */
};

/* Register map for DIO port (even interrupt) */
struct dio_port_even_int {
	uint8_t RESERVED0;
	volatile uint8_t IN;  /* Port Input */
	uint8_t RESERVED1;
	volatile uint8_t OUT; /* Port Output */
};

/* Peripheral declarations */
#define CS ((struct cs *) CS_BASE)
#define P3 ((struct dio_port_odd_int *) (DIO_BASE + 0x0020))
#define P6 ((struct dio_port_even_int *) (DIO_BASE + 0x0040))

/* Peripheral bit definitions */

/* DCORSEL Bit Mask */
#define CS_CTL0_DCORSEL_MASK ((uint32_t)0x00070000)
/* Nominal DCO Frequency Range (MHz): 2 to 4 */
#define CS_CTL0_DCORSEL_1 ((uint32_t)0x00010000)
/* Nominal DCO Frequency Range (MHz): 16 to 32 */
#define CS_CTL0_DCORSEL_4 ((uint32_t)0x00040000)
/* CS control key value */
#define CS_KEY_VAL ((uint32_t)0x0000695A)

/* Protects Sector 0 from program or erase */
#define FLCTL_BANK0_MAIN_WEPROT_PROT0 ((uint32_t)0x00000001)
/* Protects Sector 1 from program or erase */
#define FLCTL_BANK0_MAIN_WEPROT_PROT1 ((uint32_t)0x00000002)

#ifdef __cplusplus
}
#endif

#endif /* OPENOCD_LOADERS_FLASH_MSP432_MSP432P401X_H */

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

#ifndef OPENOCD_LOADERS_FLASH_MSP432_MSP432P411X_H
#define OPENOCD_LOADERS_FLASH_MSP432_MSP432P411X_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Available Peripherals */
#define __MCU_HAS_FLCTL_A__  /* Module FLCTL_A is available */
#define __MCU_HAS_SYSCTL_A__ /* Module SYSCTL_A is available */

/* Device and Peripheral Memory Map */
#define FLASH_BASE        ((uint32_t)0x00000000)     /* Flash memory address */
#define PERIPH_BASE       ((uint32_t)0x40000000)     /* Peripherals address */
#define CS_BASE           (PERIPH_BASE + 0x00010400) /* Address of CS regs. */
#define PCM_BASE          (PERIPH_BASE + 0x00010000) /* Address of PCM regs. */
#define RTC_C_BASE        (PERIPH_BASE + 0x00004400) /* Address of RTC_C regs */
#define TLV_BASE          ((uint32_t)0x00201000)     /* Address of TLV regs. */
#define WDT_A_BASE        (PERIPH_BASE + 0x00004800) /* Address of WDT_A regs */
#define BITBAND_PERI_BASE ((uint32_t)(0x42000000))

/*
 * Peripherals with 8-bit or 16-bit register access allow only 8-bit or
 * 16-bit bit band access, so cast to 8 bit always
 */
#define BITBAND_PERI(x, b) (*((volatile  uint8_t *) (BITBAND_PERI_BASE + \
	(((uint32_t)(uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

/* Register map for CLock Signal peripheral (CS) */
struct cs {
	volatile uint32_t KEY;  /* Key Register */
	volatile uint32_t CTL0; /* Control 0 Register */
	volatile uint32_t CTL1; /* Control 1 Register */
	volatile uint32_t CTL2; /* Control 2 Register */
	volatile uint32_t CTL3; /* Control 3 Register */
};

/* Register map for Power Control Module peripheral (PCM) */
struct pcm {
	volatile uint32_t CTL0;   /* Control 0 Register */
	volatile uint32_t CTL1;   /* Control 1 Register */
	volatile uint32_t IE;     /* Interrupt Enable Register */
	volatile uint32_t IFG;    /* Interrupt Flag Register */
	volatile uint32_t CLRIFG; /* Clear Interrupt Flag Register */
};

/* Register map for Real-Time Clock peripheral (RTC_C) */
struct rtc_c {
	volatile uint16_t CTL0;    /* RTCCTL0 Register */
	volatile uint16_t CTL13;   /* RTCCTL13 Register */
	volatile uint16_t OCAL;    /* RTCOCAL Register */
	volatile uint16_t TCMP;    /* RTCTCMP Register */
	volatile uint16_t PS0CTL;  /* RTC Prescale Timer 0 Control Register */
	volatile uint16_t PS1CTL;  /* RTC Prescale Timer 1 Control Register */
	volatile uint16_t PS;      /* Real-Time Clock Prescale Timer Register */
	volatile uint16_t IV;      /* Real-Time Clock Interrupt Vector Register */
	volatile uint16_t TIM0;    /* RTCTIM0 Register  Hexadecimal Format */
	volatile uint16_t TIM1;    /* Real-Time Clock Hour, Day of Week */
	volatile uint16_t DATE;    /* RTCDATE - Hexadecimal Format */
	volatile uint16_t YEAR;    /* RTCYEAR Register - Hexadecimal Format */
	volatile uint16_t AMINHR;  /* RTCMINHR - Hexadecimal Format */
	volatile uint16_t ADOWDAY; /* RTCADOWDAY - Hexadecimal Format */
	volatile uint16_t BIN2BCD; /* Binary-to-BCD Conversion Register */
	volatile uint16_t BCD2BIN; /* BCD-to-Binary Conversion Register */
};

/* Register map for Watchdog Timer peripheral (WDT_A) */
struct wdt_a {
	uint16_t RESERVED0[6];
	volatile uint16_t CTL; /* Watchdog Timer Control Register */
};

/* Peripheral Declarations */
#define CS    ((struct cs *) CS_BASE)
#define PCM   ((struct pcm *) PCM_BASE)
#define RTC_C ((struct rtc_c *) RTC_C_BASE)
#define WDT_A ((struct wdt_a *) WDT_A_BASE)

/* Peripheral Register Bit Definitions */

/* DCORSEL Bit Mask */
#define CS_CTL0_DCORSEL_MASK  ((uint32_t)0x00070000)
/* Nominal DCO Frequency Range (MHz): 2 to 4 */
#define CS_CTL0_DCORSEL_1     ((uint32_t)0x00010000)
/* Nominal DCO Frequency Range (MHz): 16 to 32 */
#define CS_CTL0_DCORSEL_4     ((uint32_t)0x00040000)
/* CS control key value */
#define CS_KEY_VAL            ((uint32_t)0x0000695A)

/* AMR Bit Mask */
#define PCM_CTL0_AMR_MASK     ((uint32_t)0x0000000F)
/* LPMR Bit Mask */
#define PCM_CTL0_LPMR_MASK    ((uint32_t)0x000000F0)
/* LPM3.5. Core voltage setting 0. */
#define PCM_CTL0_LPMR_10      ((uint32_t)0x000000A0)
/* LPM4.5 */
#define PCM_CTL0_LPMR_12      ((uint32_t)0x000000C0)
/* CPM Bit Offset */
#define PCM_CTL0_CPM_OFS      (8)
/* CPM Bit Mask */
#define PCM_CTL0_CPM_MASK     ((uint32_t)0x00003F00)
/* PCMKEY Bit Mask */
#define PCM_CTL0_KEY_MASK     ((uint32_t)0xFFFF0000)
/* PMR_BUSY Bit Offset */
#define PCM_CTL1_PMR_BUSY_OFS (8)

/* RTCKEY Bit Offset */
#define RTC_C_CTL0_KEY_OFS    (8)
/* RTCKEY Bit Mask */
#define RTC_C_CTL0_KEY_MASK   ((uint16_t)0xFF00)
/* RTCHOLD Bit Offset */
#define RTC_C_CTL13_HOLD_OFS  (6)
/* RTC_C Key Value for RTC_C write access */
#define RTC_C_KEY             ((uint16_t)0xA500)

/* Watchdog timer hold */
#define WDT_A_CTL_HOLD        ((uint16_t)0x0080)
/* WDT Key Value for WDT write access */
#define WDT_A_CTL_PW          ((uint16_t)0x5A00)

/* Address of BSL API table */
#define BSL_API_TABLE_ADDR    ((uint32_t)0x00202000)

#ifdef __cplusplus
}
#endif

#endif /* OPENOCD_LOADERS_FLASH_MSP432_MSP432P411X_H */

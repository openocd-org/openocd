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

#ifndef OPENOCD_LOADERS_FLASH_MSP432_DRIVERLIB_H
#define OPENOCD_LOADERS_FLASH_MSP432_DRIVERLIB_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__MSP432E4X__)
#include "msp432e4x.h"
#elif defined(__MSP432P401X__)
#include "msp432p401x.h"
#elif defined(__MSP432P411X__)
#include "msp432p411x.h"
#else
#error "Failed to match a device specific include file"
#endif

/* Structure type to access the System Control Block (SCB). */
struct SCB_Type {
	volatile uint32_t CPUID;         /* CPUID Base Register */
	volatile uint32_t ICSR;          /* Interrupt Control and State Register */
	volatile uint32_t VTOR;          /* Vector Table Offset Register */
	volatile uint32_t AIRCR;         /* Application Interrupt and Reset Control */
	volatile uint32_t SCR;           /* System Control Register */
	volatile uint32_t CCR;           /* Configuration Control Register */
	volatile uint8_t  SHP[12U];      /* System Handlers Priority Registers */
	volatile uint32_t SHCSR;         /* System Handler Control and State */
	volatile uint32_t CFSR;          /* Configurable Fault Status Register */
	volatile uint32_t HFSR;          /* HardFault Status Register */
	volatile uint32_t DFSR;          /* Debug Fault Status Register */
	volatile uint32_t MMFAR;         /* MemManage Fault Address Register */
	volatile uint32_t BFAR;          /* BusFault Address Register */
	volatile uint32_t AFSR;          /* Auxiliary Fault Status Register */
	volatile uint32_t PFR[2U];       /* Processor Feature Register */
	volatile uint32_t DFR;           /* Debug Feature Register */
	volatile uint32_t ADR;           /* Auxiliary Feature Register */
	volatile uint32_t MMFR[4U];      /* Memory Model Feature Register */
	volatile uint32_t ISAR[5U];      /* Instruction Set Attributes Register */
			 uint32_t RESERVED0[5U];
	volatile uint32_t CPACR;         /* Coprocessor Access Control Register */
};

/* SCB:SCR register bits */
#define SCB_SCR_SLEEPDEEP_POS 2U
#define SCB_SCR_SLEEPDEEP_MSK (1UL << SCB_SCR_SLEEPDEEP_POS)

/* Memory mapping of Core Hardware */
#define SCS_BASE (0xE000E000UL)         /* System Control Space Base Address */
#define SCB_BASE (SCS_BASE +  0x0D00UL) /* System Control Block Base Address */
#define SCB ((struct SCB_Type *)SCB_BASE) /* SCB configuration struct */

/* Definitions of standard bits */
#define BIT0   (uint16_t)(0x0001)
#define BIT1   (uint16_t)(0x0002)
#define BIT2   (uint16_t)(0x0004)
#define BIT3   (uint16_t)(0x0008)
#define BIT4   (uint16_t)(0x0010)
#define BIT5   (uint16_t)(0x0020)
#define BIT6   (uint16_t)(0x0040)
#define BIT7   (uint16_t)(0x0080)
#define BIT8   (uint16_t)(0x0100)
#define BIT9   (uint16_t)(0x0200)
#define BITA   (uint16_t)(0x0400)
#define BITB   (uint16_t)(0x0800)
#define BITC   (uint16_t)(0x1000)
#define BITD   (uint16_t)(0x2000)
#define BITE   (uint16_t)(0x4000)
#define BITF   (uint16_t)(0x8000)
#define BIT(x) ((uint16_t)1 << (x))

/* CPU Module prototypes */
extern uint32_t cpu_cpsid(void);
extern void cpu_wfi(void);

/* Clock Signal Module constants */
#define CS_DCO_FREQUENCY_3  CS_CTL0_DCORSEL_1
#define CS_DCO_FREQUENCY_24 CS_CTL0_DCORSEL_4

/* Power Control Module constants */
#define PCM_KEY              0x695A0000
#define PCM_AM_LDO_VCORE0    0x00
#define PCM_AM_LDO_VCORE1    0x01
#define PCM_AM_DCDC_VCORE0   0x04
#define PCM_AM_DCDC_VCORE1   0x05
#define PCM_AM_LF_VCORE0     0x08
#define PCM_AM_LF_VCORE1     0x09
#define PCM_LPM0_LDO_VCORE0  0x10
#define PCM_LPM0_LDO_VCORE1  0x11
#define PCM_LPM0_DCDC_VCORE0 0x14
#define PCM_LPM0_DCDC_VCORE1 0x15
#define PCM_LPM0_LF_VCORE0   0x18
#define PCM_LPM0_LF_VCORE1   0x19
#define PCM_LPM3             0x20
#define PCM_LPM4             0x21
#define PCM_LPM35_VCORE0     0xC0
#define PCM_LPM45            0xA0
#define PCM_VCORE0           0x00
#define PCM_VCORE1           0x01
#define PCM_VCORELPM3        0x02
#define PCM_LDO_MODE         0x00
#define PCM_DCDC_MODE        0x01
#define PCM_LF_MODE          0x02

/* Power Control Module prototypes */
extern bool pcm_set_core_voltage_level(uint_fast8_t voltage_level);
extern uint8_t pcm_get_core_voltage_level(void);
extern bool pcm_set_power_mode(uint_fast8_t power_mode);
extern uint8_t pcm_get_power_mode(void);
extern bool pcm_set_power_state(uint_fast8_t power_state);
extern uint8_t pcm_get_power_state(void);
extern bool pcm_shutdown_device(uint32_t shutdown_mode);
extern bool pcm_goto_lpm0(void);
extern bool pcm_goto_lpm3(void);
extern bool pcm_goto_lpm4(void);

/* ROM API Function Pointers */
#define ROM_API_TABLE         ((unsigned long *)0x02000800)
#define ROM_FLASH_CTL_TABLE   ((unsigned long *)(ROM_API_TABLE[7]))
#define ROM_PCM_TABLE         ((unsigned long *)(ROM_API_TABLE[13]))
#define ROM_WDT_TABLE         ((unsigned long *)(ROM_API_TABLE[25]))
#define ROM_SYS_CTL_A_TABLE   ((unsigned long *)(ROM_API_TABLE[26]))
#define ROM_FLASH_CTL_A_TABLE ((unsigned long *)(ROM_API_TABLE[27]))

#if defined(__MSP432P401X__)
#define ROM_FLASH_CTL_UNPROTECT_SECTOR \
	((bool (*)(uint_fast8_t memory_space, \
		uint32_t sector_mask))ROM_FLASH_CTL_TABLE[4])
#endif
#if defined(__MSP432P401X__)
#define ROM_FLASH_CTL_PROTECT_SECTOR \
	((bool (*)(uint_fast8_t memory_space, \
		uint32_t sector_mask))ROM_FLASH_CTL_TABLE[5])
#endif
#if defined(__MSP432P401X__)
#define ROM_FLASH_CTL_PERFORM_MASS_ERASE \
	((bool (*)(void))ROM_FLASH_CTL_TABLE[8])
#endif
#if defined(__MSP432P401X__)
#define ROM_FLASH_CTL_ERASE_SECTOR \
	((bool (*)(uint32_t addr))ROM_FLASH_CTL_TABLE[9])
#endif
#if defined(__MSP432P401X__)
#define ROM_FLASH_CTL_PROGRAM_MEMORY \
	((bool (*)(void *src, void *dest, uint32_t length))ROM_FLASH_CTL_TABLE[10])
#endif
#if defined(__MSP432P401X__)
#define ROM_FLASH_CTL_SET_WAIT_STATE \
	((void (*)(uint32_t bank, uint32_t wait_state))ROM_FLASH_CTL_TABLE[21])
#endif
#if defined(__MSP432P401X__)
#define ROM_FLASH_CTL_GET_WAIT_STATE \
	((uint32_t (*)(uint32_t bank))ROM_FLASH_CTL_TABLE[22])
#endif
#if defined(__MSP432P401X__)
#define ROM_PCM_SET_CORE_VOLTAGE_LEVEL \
	((bool (*)(uint_fast8_t voltage_level))ROM_PCM_TABLE[0])
#endif
#if defined(__MSP432P401X__)
#define ROM_PCM_GET_CORE_VOLTAGE_LEVEL \
	((uint8_t (*)(void))ROM_PCM_TABLE[1])
#endif
#if defined(__MSP432P401X__)
#define ROM_PCM_SET_POWER_STATE \
	((bool (*)(uint_fast8_t power_state))ROM_PCM_TABLE[6])
#endif
#if defined(__MSP432P401X__)
#define ROM_PCM_GET_POWER_STATE \
	((uint8_t (*)(void))ROM_PCM_TABLE[8])
#endif
#if defined(__MSP432P401X__) || defined(__MSP432P411X__)
#define ROM_WDT_A_HOLD_TIMER \
	((void (*)(void))ROM_WDT_TABLE[0])
#endif
#if defined(__MSP432P411X__)
#define ROM_SYS_CTL_A_GET_FLASH_SIZE \
	((uint_least32_t (*)(void))ROM_SYS_CTL_A_TABLE[1])
#endif
#if defined(__MSP432P411X__)
#define ROM_SYS_CTL_A_GET_INFO_FLASH_SIZE \
	((uint_least32_t (*)(void))ROM_SYS_CTL_A_TABLE[18])
#endif
#if defined(__MSP432P411X__)
#define ROM_FLASH_CTL_A_UNPROTECT_MEMORY \
	((bool (*)(uint32_t start_addr, uint32_t end_addr))ROM_FLASH_CTL_A_TABLE[4])
#endif
#if defined(__MSP432P411X__)
#define ROM_FLASH_CTL_A_PROTECT_MEMORY \
	((bool (*)(uint32_t start_addr, uint32_t end_addr))ROM_FLASH_CTL_A_TABLE[5])
#endif
#if defined(__MSP432P411X__)
#define ROM_FLASH_CTL_A_PERFORM_MASS_ERASE \
	((bool (*)(void))ROM_FLASH_CTL_A_TABLE[8])
#endif
#if defined(__MSP432P411X__)
#define ROM_FLASH_CTL_A_ERASE_SECTOR \
	((bool (*)(uint32_t addr))ROM_FLASH_CTL_A_TABLE[9])
#endif
#if defined(__MSP432P411X__)
#define ROM_FLASH_CTL_A_PROGRAM_MEMORY \
	((bool (*)(void *src, void *dest, uint32_t length)) \
		ROM_FLASH_CTL_A_TABLE[10])
#endif
#if defined(__MSP432P411X__)
#define ROM_FLASH_CTL_A_SET_WAIT_STATE \
	((void (*)(uint32_t bank, uint32_t wait_state))ROM_FLASH_CTL_A_TABLE[21])
#endif
#if defined(__MSP432P411X__)
#define ROM_FLASH_CTL_A_GET_WAIT_STATE \
	((uint32_t (*)(uint32_t bank))ROM_FLASH_CTL_A_TABLE[22])
#endif

/* Map API functions to ROM or locally built functions */
#ifdef ROM_FLASH_CTL_UNPROTECT_SECTOR
#define MAP_FLASH_CTL_UNPROTECT_SECTOR ROM_FLASH_CTL_UNPROTECT_SECTOR
#else
#define MAP_FLASH_CTL_UNPROTECT_SECTOR flash_ctl_unprotect_sector
#endif
#ifdef ROM_FLASH_CTL_PROTECT_SECTOR
#define MAP_FLASH_CTL_PROTECT_SECTOR ROM_FLASH_CTL_PROTECT_SECTOR
#else
#define MAP_FLASH_CTL_PROTECT_SECTOR flash_ctl_protect_sector
#endif
#ifdef ROM_FLASH_CTL_PERFORM_MASS_ERASE
#define MAP_FLASH_CTL_PERFORM_MASS_ERASE ROM_FLASH_CTL_PERFORM_MASS_ERASE
#else
#define MAP_FLASH_CTL_PERFORM_MASS_ERASE flash_ctl_perform_mass_erase
#endif
#ifdef ROM_FLASH_CTL_ERASE_SECTOR
#define MAP_FLASH_CTL_ERASE_SECTOR ROM_FLASH_CTL_ERASE_SECTOR
#else
#define MAP_FLASH_CTL_ERASE_SECTOR flash_ctl_erase_sector
#endif
#ifdef ROM_FLASH_CTL_PROGRAM_MEMORY
#define MAP_FLASH_CTL_PROGRAM_MEMORY ROM_FLASH_CTL_PROGRAM_MEMORY
#else
#define MAP_FLASH_CTL_PROGRAM_MEMORY flash_ctl_program_memory
#endif
#ifdef ROM_FLASH_CTL_SET_WAIT_STATE
#define MAP_FLASH_CTL_SET_WAIT_STATE ROM_FLASH_CTL_SET_WAIT_STATE
#else
#define MAP_FLASH_CTL_SET_WAIT_STATE flash_ctl_set_wait_state
#endif
#ifdef ROM_FLASH_CTL_GET_WAIT_STATE
#define MAP_FLASH_CTL_GET_WAIT_STATE ROM_FLASH_CTL_GET_WAIT_STATE
#else
#define MAP_FLASH_CTL_GET_WAIT_STATE flash_ctl_get_wait_state
#endif
#ifdef ROM_PCM_SET_CORE_VOLTAGE_LEVEL
#define MAP_PCM_SET_CORE_VOLTAGE_LEVEL ROM_PCM_SET_CORE_VOLTAGE_LEVEL
#else
#define MAP_PCM_SET_CORE_VOLTAGE_LEVEL pcm_set_core_voltage_level
#endif
#ifdef ROM_PCM_GET_CORE_VOLTAGE_LEVEL
#define MAP_PCM_GET_CORE_VOLTAGE_LEVEL ROM_PCM_GET_CORE_VOLTAGE_LEVEL
#else
#define MAP_PCM_GET_CORE_VOLTAGE_LEVEL pcm_get_core_voltage_level
#endif
#ifdef ROM_PCM_SET_POWER_STATE
#define MAP_PCM_SET_POWER_STATE ROM_PCM_SET_POWER_STATE
#else
#define MAP_PCM_SET_POWER_STATE pcm_set_power_state
#endif
#ifdef ROM_PCM_GET_POWER_STATE
#define MAP_PCM_GET_POWER_STATE ROM_PCM_GET_POWER_STATE
#else
#define MAP_PCM_GET_POWER_STATE pcm_get_power_state
#endif
#ifdef ROM_WDT_A_HOLD_TIMER
#define MAP_WDT_A_HOLD_TIMER ROM_WDT_A_HOLD_TIMER
#else
#define MAP_WDT_A_HOLD_TIMER wdt_a_hold_timer
#endif
#ifdef ROM_SYS_CTL_A_GET_FLASH_SIZE
#define MAP_SYS_CTL_A_GET_FLASH_SIZE ROM_SYS_CTL_A_GET_FLASH_SIZE
#else
#define MAP_SYS_CTL_A_GET_FLASH_SIZE sys_ctl_a_get_flash_size
#endif
#ifdef ROM_SYS_CTL_A_GET_INFO_FLASH_SIZE
#define MAP_SYS_CTL_A_GET_INFO_FLASH_SIZE ROM_SYS_CTL_A_GET_INFO_FLASH_SIZE
#else
#define MAP_SYS_CTL_A_GET_INFO_FLASH_SIZE sys_ctl_a_get_info_flash_size
#endif
#ifdef ROM_FLASH_CTL_A_UNPROTECT_MEMORY
#define MAP_FLASH_CTL_A_UNPROTECT_MEMORY ROM_FLASH_CTL_A_UNPROTECT_MEMORY
#else
#define MAP_FLASH_CTL_A_UNPROTECT_MEMORY flash_ctl_a_unprotect_memory
#endif
#ifdef ROM_FLASH_CTL_A_PROTECT_MEMORY
#define MAP_FLASH_CTL_A_PROTECT_MEMORY ROM_FLASH_CTL_A_PROTECT_MEMORY
#else
#define MAP_FLASH_CTL_A_PROTECT_MEMORY flash_ctl_a_protect_memory
#endif
#ifdef ROM_FLASH_CTL_A_PERFORM_MASS_ERASE
#define MAP_FLASH_CTL_A_PERFORM_MASS_ERASE ROM_FLASH_CTL_A_PERFORM_MASS_ERASE
#else
#define MAP_FLASH_CTL_A_PERFORM_MASS_ERASE flash_ctl_a_perform_mass_erase
#endif
#ifdef ROM_FLASH_CTL_A_ERASE_SECTOR
#define MAP_FLASH_CTL_A_ERASE_SECTOR ROM_FLASH_CTL_A_ERASE_SECTOR
#else
#define MAP_FLASH_CTL_A_ERASE_SECTOR flash_ctl_a_erase_sector
#endif
#ifdef ROM_FLASH_CTL_A_PROGRAM_MEMORY
#define MAP_FLASH_CTL_A_PROGRAM_MEMORY ROM_FLASH_CTL_A_PROGRAM_MEMORY
#else
#define MAP_FLASH_CTL_A_PROGRAM_MEMORY flash_ctl_a_program_memory
#endif
#ifdef ROM_FLASH_CTL_A_SET_WAIT_STATE
#define MAP_FLASH_CTL_A_SET_WAIT_STATE ROM_FLASH_CTL_A_SET_WAIT_STATE
#else
#define MAP_FLASH_CTL_A_SET_WAIT_STATE flash_ctl_a_set_wait_state
#endif
#ifdef ROM_FLASH_CTL_A_GET_WAIT_STATE
#define MAP_FLASH_CTL_A_GET_WAIT_STATE ROM_FLASH_CTL_A_GET_WAIT_STATE
#else
#define MAP_FLASH_CTL_A_GET_WAIT_STATE flash_ctl_a_get_wait_state
#endif

/* Real Time Clock Module prototypes */
extern void rtc_c_hold_clock(void);

/* Watchdog Timer Module prototypes */
extern void wdt_a_hold_timer(void);

#if defined(__MCU_HAS_FLCTL_A__)
#define FLASH_A_BANK0                0x00
#define FLASH_A_BANK1                0x01
#define __INFO_FLASH_A_TECH_START__  0x00200000
#define __INFO_FLASH_A_TECH_MIDDLE__ 0x00204000
#endif

#if defined(__MCU_HAS_FLCTL__)
#define FLASH_BANK0 0x00
#define FLASH_BANK1 0x01
#define FLASH_MAIN_MEMORY_SPACE_BANK0 0x01
#define FLASH_MAIN_MEMORY_SPACE_BANK1 0x02
#define FLASH_INFO_MEMORY_SPACE_BANK0 0x03
#define FLASH_INFO_MEMORY_SPACE_BANK1 0x04
#define FLASH_SECTOR0 FLCTL_BANK0_MAIN_WEPROT_PROT0
#define FLASH_SECTOR1 FLCTL_BANK0_MAIN_WEPROT_PROT1
#endif

#ifdef __cplusplus
}
#endif

#endif /* OPENOCD_LOADERS_FLASH_MSP432_DRIVERLIB_H */

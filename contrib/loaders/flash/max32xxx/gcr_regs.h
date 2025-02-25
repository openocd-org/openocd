/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2016 by Maxim Integrated                                *
 *   Copyright (C) 2025 Analog Devices, Inc.                               *
 ***************************************************************************/

#ifndef _GCR_REGS_H_
#define _GCR_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__ICCARM__)
#pragma system_include
#endif

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/*/ @cond */
/*
	If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I volatile const
#endif
#ifndef __O
#define __O volatile
#endif
#ifndef __R
#define __R volatile const
#endif
/*/ @endcond */

/* **** Definitions **** */

/**
 * @ingroup     gcr
 * @defgroup    gcr_registers GCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the GCR Peripheral Module.
 * @details Global Control Registers.
 */

/**
 * @ingroup gcr_registers
 * Structure type to access the GCR Registers.
 */
struct mxc_gcr_regs {
	__IO uint32_t scon;	 /**< <tt>\b 0x00:</tt> GCR SCON Register */
	__IO uint32_t rstr0; /**< <tt>\b 0x04:</tt> GCR RSTR0 Register */
	__IO uint32_t clkcn; /**< <tt>\b 0x08:</tt> GCR CLKCN Register */
	__IO uint32_t pm;	 /**< <tt>\b 0x0C:</tt> GCR PM Register */
	__R uint32_t rsv_0x10_0x17[2];
	__IO uint32_t pckdiv; /**< <tt>\b 0x18:</tt> GCR PCKDIV Register */
	__R uint32_t rsv_0x1c_0x23[2];
	__IO uint32_t perckcn0; /**< <tt>\b 0x24:</tt> GCR PERCKCN0 Register */
	__IO uint32_t memckcn;	/**< <tt>\b 0x28:</tt> GCR MEMCKCN Register */
	__IO uint32_t memzcn;	/**< <tt>\b 0x2C:</tt> GCR MEMZCN Register */
	__R uint32_t rsv_0x30_0x3f[4];
	__IO uint32_t sysst;	/**< <tt>\b 0x40:</tt> GCR SYSST Register */
	__IO uint32_t rstr1;	/**< <tt>\b 0x44:</tt> GCR RSTR1 Register */
	__IO uint32_t perckcn1; /**< <tt>\b 0x48:</tt> GCR PERCKCN1 Register */
	__IO uint32_t evten;	/**< <tt>\b 0x4C:</tt> GCR EVTEN Register */
	__I uint32_t revision;	/**< <tt>\b 0x50:</tt> GCR REVISION Register */
	__IO uint32_t syssie;	/**< <tt>\b 0x54:</tt> GCR SYSSIE Register */
	__R uint32_t rsv_0x58_0x63[3];
	__IO uint32_t eccerr;	/**< <tt>\b 0x64:</tt> GCR ECCERR Register */
	__IO uint32_t eccnded;	/**< <tt>\b 0x68:</tt> GCR ECCNDED Register */
	__IO uint32_t eccirqen; /**< <tt>\b 0x6C:</tt> GCR ECCIRQEN Register */
	__IO uint32_t eccerrad; /**< <tt>\b 0x70:</tt> GCR ECCERRAD Register */
};

/* Register offsets for module GCR */
/**
 * @ingroup    gcr_registers
 * @defgroup   GCR_Register_Offsets Register Offsets
 * @brief      GCR Peripheral Register Offsets from the GCR Base Peripheral Address.
 * @{
 */
#define MXC_R_GCR_SCON ((uint32_t)0x00000000UL)		/**< Offset from GCR Base Address: 0x0000 */
#define MXC_R_GCR_RSTR0 ((uint32_t)0x00000004UL)	/**< Offset from GCR Base Address: 0x0004 */
#define MXC_R_GCR_CLKCN ((uint32_t)0x00000008UL)	/**< Offset from GCR Base Address: 0x0008 */
#define MXC_R_GCR_PM ((uint32_t)0x0000000CUL)		/**< Offset from GCR Base Address: 0x000C */
#define MXC_R_GCR_PCKDIV ((uint32_t)0x00000018UL)	/**< Offset from GCR Base Address: 0x0018 */
#define MXC_R_GCR_PERCKCN0 ((uint32_t)0x00000024UL) /**< Offset from GCR Base Address: 0x0024 */
#define MXC_R_GCR_MEMCKCN ((uint32_t)0x00000028UL)	/**< Offset from GCR Base Address: 0x0028 */
#define MXC_R_GCR_MEMZCN ((uint32_t)0x0000002CUL)	/**< Offset from GCR Base Address: 0x002C */
#define MXC_R_GCR_SYSST ((uint32_t)0x00000040UL)	/**< Offset from GCR Base Address: 0x0040 */
#define MXC_R_GCR_RSTR1 ((uint32_t)0x00000044UL)	/**< Offset from GCR Base Address: 0x0044 */
#define MXC_R_GCR_PERCKCN1 ((uint32_t)0x00000048UL) /**< Offset from GCR Base Address: 0x0048 */
#define MXC_R_GCR_EVTEN ((uint32_t)0x0000004CUL)	/**< Offset from GCR Base Address: 0x004C */
#define MXC_R_GCR_REVISION ((uint32_t)0x00000050UL) /**< Offset from GCR Base Address: 0x0050 */
#define MXC_R_GCR_SYSSIE ((uint32_t)0x00000054UL)	/**< Offset from GCR Base Address: 0x0054 */
#define MXC_R_GCR_ECCERR ((uint32_t)0x00000064UL)	/**< Offset from GCR Base Address: 0x0064 */
#define MXC_R_GCR_ECCNDED ((uint32_t)0x00000068UL)	/**< Offset from GCR Base Address: 0x0068 */
#define MXC_R_GCR_ECCIRQEN ((uint32_t)0x0000006CUL) /**< Offset from GCR Base Address: 0x006C */
#define MXC_R_GCR_ECCERRAD ((uint32_t)0x00000070UL) /**< Offset from GCR Base Address: 0x0070 */
													/**@} end of group gcr_registers */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SCON GCR_SCON
 * @brief    System Control.
 * @{
 */
#define MXC_F_GCR_SCON_BSTAPEN_POS 0											 /**< SCON_BSTAPEN Position */
#define MXC_F_GCR_SCON_BSTAPEN ((uint32_t)(0x1UL << MXC_F_GCR_SCON_BSTAPEN_POS)) /**< SCON_BSTAPEN Mask */
#define MXC_V_GCR_SCON_BSTAPEN_DIS ((uint32_t)0x0UL)							 /**< SCON_BSTAPEN_DIS Value */
#define MXC_S_GCR_SCON_BSTAPEN_DIS                                                                                     \
	(MXC_V_GCR_SCON_BSTAPEN_DIS << MXC_F_GCR_SCON_BSTAPEN_POS) /**< SCON_BSTAPEN_DIS Setting */
#define MXC_V_GCR_SCON_BSTAPEN_EN ((uint32_t)0x1UL)			   /**< SCON_BSTAPEN_EN Value */
#define MXC_S_GCR_SCON_BSTAPEN_EN                                                                                      \
	(MXC_V_GCR_SCON_BSTAPEN_EN << MXC_F_GCR_SCON_BSTAPEN_POS) /**< SCON_BSTAPEN_EN Setting */

#define MXC_F_GCR_SCON_SBUSARB_POS 1											 /**< SCON_SBUSARB Position */
#define MXC_F_GCR_SCON_SBUSARB ((uint32_t)(0x3UL << MXC_F_GCR_SCON_SBUSARB_POS)) /**< SCON_SBUSARB Mask */
#define MXC_V_GCR_SCON_SBUSARB_FIX ((uint32_t)0x0UL)							 /**< SCON_SBUSARB_FIX Value */
#define MXC_S_GCR_SCON_SBUSARB_FIX                                                                                     \
	(MXC_V_GCR_SCON_SBUSARB_FIX << MXC_F_GCR_SCON_SBUSARB_POS) /**< SCON_SBUSARB_FIX Setting */
#define MXC_V_GCR_SCON_SBUSARB_ROUND ((uint32_t)0x1UL)		   /**< SCON_SBUSARB_ROUND Value */
#define MXC_S_GCR_SCON_SBUSARB_ROUND                                                                                   \
	(MXC_V_GCR_SCON_SBUSARB_ROUND << MXC_F_GCR_SCON_SBUSARB_POS) /**< SCON_SBUSARB_ROUND Setting */

#define MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS 4 /**< SCON_FLASH_PAGE_FLIP Position */
#define MXC_F_GCR_SCON_FLASH_PAGE_FLIP                                                                                 \
	((uint32_t)(0x1UL << MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS))	/**< SCON_FLASH_PAGE_FLIP Mask */
#define MXC_V_GCR_SCON_FLASH_PAGE_FLIP_NORMAL ((uint32_t)0x0UL) /**< SCON_FLASH_PAGE_FLIP_NORMAL Value */
#define MXC_S_GCR_SCON_FLASH_PAGE_FLIP_NORMAL                                                                          \
	(MXC_V_GCR_SCON_FLASH_PAGE_FLIP_NORMAL                                                                             \
	 << MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS)						 /**< SCON_FLASH_PAGE_FLIP_NORMAL Setting */
#define MXC_V_GCR_SCON_FLASH_PAGE_FLIP_SWAPPED ((uint32_t)0x1UL) /**< SCON_FLASH_PAGE_FLIP_SWAPPED Value */
#define MXC_S_GCR_SCON_FLASH_PAGE_FLIP_SWAPPED                                                                         \
	(MXC_V_GCR_SCON_FLASH_PAGE_FLIP_SWAPPED                                                                            \
	 << MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS) /**< SCON_FLASH_PAGE_FLIP_SWAPPED Setting */

#define MXC_F_GCR_SCON_CCACHE_FLUSH_POS 6 /**< SCON_CCACHE_FLUSH Position */
#define MXC_F_GCR_SCON_CCACHE_FLUSH                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_SCON_CCACHE_FLUSH_POS))	 /**< SCON_CCACHE_FLUSH Mask */
#define MXC_V_GCR_SCON_CCACHE_FLUSH_NORMAL ((uint32_t)0x0UL) /**< SCON_CCACHE_FLUSH_NORMAL Value */
#define MXC_S_GCR_SCON_CCACHE_FLUSH_NORMAL                                                                             \
	(MXC_V_GCR_SCON_CCACHE_FLUSH_NORMAL << MXC_F_GCR_SCON_CCACHE_FLUSH_POS) /**< SCON_CCACHE_FLUSH_NORMAL Setting */
#define MXC_V_GCR_SCON_CCACHE_FLUSH_FLUSH ((uint32_t)0x1UL)					/**< SCON_CCACHE_FLUSH_FLUSH Value */
#define MXC_S_GCR_SCON_CCACHE_FLUSH_FLUSH                                                                              \
	(MXC_V_GCR_SCON_CCACHE_FLUSH_FLUSH << MXC_F_GCR_SCON_CCACHE_FLUSH_POS) /**< SCON_CCACHE_FLUSH_FLUSH Setting */

#define MXC_F_GCR_SCON_CCHK_POS 13										   /**< SCON_CCHK Position */
#define MXC_F_GCR_SCON_CCHK ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CCHK_POS)) /**< SCON_CCHK Mask */
#define MXC_V_GCR_SCON_CCHK_COMPLETE ((uint32_t)0x0UL)					   /**< SCON_CCHK_COMPLETE Value */
#define MXC_S_GCR_SCON_CCHK_COMPLETE                                                                                   \
	(MXC_V_GCR_SCON_CCHK_COMPLETE << MXC_F_GCR_SCON_CCHK_POS) /**< SCON_CCHK_COMPLETE Setting */
#define MXC_V_GCR_SCON_CCHK_START ((uint32_t)0x1UL)			  /**< SCON_CCHK_START Value */
#define MXC_S_GCR_SCON_CCHK_START (MXC_V_GCR_SCON_CCHK_START << MXC_F_GCR_SCON_CCHK_POS) /**< SCON_CCHK_START Setting  \
																						  */

#define MXC_F_GCR_SCON_CHKRES_POS 15										   /**< SCON_CHKRES Position */
#define MXC_F_GCR_SCON_CHKRES ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CHKRES_POS)) /**< SCON_CHKRES Mask */
#define MXC_V_GCR_SCON_CHKRES_PASS ((uint32_t)0x0UL)						   /**< SCON_CHKRES_PASS Value */
#define MXC_S_GCR_SCON_CHKRES_PASS                                                                                     \
	(MXC_V_GCR_SCON_CHKRES_PASS << MXC_F_GCR_SCON_CHKRES_POS) /**< SCON_CHKRES_PASS Setting */
#define MXC_V_GCR_SCON_CHKRES_FAIL ((uint32_t)0x1UL)		  /**< SCON_CHKRES_FAIL Value */
#define MXC_S_GCR_SCON_CHKRES_FAIL                                                                                     \
	(MXC_V_GCR_SCON_CHKRES_FAIL << MXC_F_GCR_SCON_CHKRES_POS) /**< SCON_CHKRES_FAIL Setting */

#define MXC_F_GCR_SCON_OVR_POS 16													/**< SCON_OVR Position */
#define MXC_F_GCR_SCON_OVR ((uint32_t)(0x3UL << MXC_F_GCR_SCON_OVR_POS))			/**< SCON_OVR Mask */
#define MXC_V_GCR_SCON_OVR_0_9V ((uint32_t)0x0UL)									/**< SCON_OVR_0_9V Value */
#define MXC_S_GCR_SCON_OVR_0_9V (MXC_V_GCR_SCON_OVR_0_9V << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_0_9V Setting */
#define MXC_V_GCR_SCON_OVR_1_0V ((uint32_t)0x1UL)									/**< SCON_OVR_1_0V Value */
#define MXC_S_GCR_SCON_OVR_1_0V (MXC_V_GCR_SCON_OVR_1_0V << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_1_0V Setting */
#define MXC_V_GCR_SCON_OVR_1_1V ((uint32_t)0x2UL)									/**< SCON_OVR_1_1V Value */
#define MXC_S_GCR_SCON_OVR_1_1V (MXC_V_GCR_SCON_OVR_1_1V << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_1_1V Setting */

#define MXC_F_GCR_SCON_MEMPROT_EN_POS 20											   /**< SCON_MEMPROT_EN Position */
#define MXC_F_GCR_SCON_MEMPROT_EN ((uint32_t)(0x1UL << MXC_F_GCR_SCON_MEMPROT_EN_POS)) /**< SCON_MEMPROT_EN Mask */
#define MXC_V_GCR_SCON_MEMPROT_EN_DIS ((uint32_t)0x0UL)								   /**< SCON_MEMPROT_EN_DIS Value */
#define MXC_S_GCR_SCON_MEMPROT_EN_DIS                                                                                  \
	(MXC_V_GCR_SCON_MEMPROT_EN_DIS << MXC_F_GCR_SCON_MEMPROT_EN_POS) /**< SCON_MEMPROT_EN_DIS Setting */
#define MXC_V_GCR_SCON_MEMPROT_EN_EN ((uint32_t)0x1UL)				 /**< SCON_MEMPROT_EN_EN Value */
#define MXC_S_GCR_SCON_MEMPROT_EN_EN                                                                                   \
	(MXC_V_GCR_SCON_MEMPROT_EN_EN << MXC_F_GCR_SCON_MEMPROT_EN_POS) /**< SCON_MEMPROT_EN_EN Setting */

#define MXC_F_GCR_SCON_MEMPROT_KEYSZ_POS 21 /**< SCON_MEMPROT_KEYSZ Position */
#define MXC_F_GCR_SCON_MEMPROT_KEYSZ                                                                                   \
	((uint32_t)(0x1UL << MXC_F_GCR_SCON_MEMPROT_KEYSZ_POS)) /**< SCON_MEMPROT_KEYSZ Mask */
#define MXC_V_GCR_SCON_MEMPROT_KEYSZ_128 ((uint32_t)0x0UL)	/**< SCON_MEMPROT_KEYSZ_128 Value */
#define MXC_S_GCR_SCON_MEMPROT_KEYSZ_128                                                                               \
	(MXC_V_GCR_SCON_MEMPROT_KEYSZ_128 << MXC_F_GCR_SCON_MEMPROT_KEYSZ_POS) /**< SCON_MEMPROT_KEYSZ_128 Setting */
#define MXC_V_GCR_SCON_MEMPROT_KEYSZ_256 ((uint32_t)0x1UL)				   /**< SCON_MEMPROT_KEYSZ_256 Value */
#define MXC_S_GCR_SCON_MEMPROT_KEYSZ_256                                                                               \
	(MXC_V_GCR_SCON_MEMPROT_KEYSZ_256 << MXC_F_GCR_SCON_MEMPROT_KEYSZ_POS) /**< SCON_MEMPROT_KEYSZ_256 Setting */

/**@} end of group GCR_SCON_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_RSTR0 GCR_RSTR0
 * @brief    Reset.
 * @{
 */
#define MXC_F_GCR_RSTR0_DMA_POS 0										   /**< RSTR0_DMA Position */
#define MXC_F_GCR_RSTR0_DMA ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_DMA_POS)) /**< RSTR0_DMA Mask */
#define MXC_V_GCR_RSTR0_DMA_RESET_DONE ((uint32_t)0x0UL)				   /**< RSTR0_DMA_RESET_DONE Value */
#define MXC_S_GCR_RSTR0_DMA_RESET_DONE                                                                                 \
	(MXC_V_GCR_RSTR0_DMA_RESET_DONE << MXC_F_GCR_RSTR0_DMA_POS) /**< RSTR0_DMA_RESET_DONE Setting */
#define MXC_V_GCR_RSTR0_DMA_BUSY ((uint32_t)0x1UL)				/**< RSTR0_DMA_BUSY Value */
#define MXC_S_GCR_RSTR0_DMA_BUSY (MXC_V_GCR_RSTR0_DMA_BUSY << MXC_F_GCR_RSTR0_DMA_POS) /**< RSTR0_DMA_BUSY Setting */

#define MXC_F_GCR_RSTR0_WDT_POS 1										   /**< RSTR0_WDT Position */
#define MXC_F_GCR_RSTR0_WDT ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_WDT_POS)) /**< RSTR0_WDT Mask */

#define MXC_F_GCR_RSTR0_GPIO0_POS 2											   /**< RSTR0_GPIO0 Position */
#define MXC_F_GCR_RSTR0_GPIO0 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_GPIO0_POS)) /**< RSTR0_GPIO0 Mask */

#define MXC_F_GCR_RSTR0_GPIO1_POS 3											   /**< RSTR0_GPIO1 Position */
#define MXC_F_GCR_RSTR0_GPIO1 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_GPIO1_POS)) /**< RSTR0_GPIO1 Mask */

#define MXC_F_GCR_RSTR0_TIMER0_POS 5											 /**< RSTR0_TIMER0 Position */
#define MXC_F_GCR_RSTR0_TIMER0 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER0_POS)) /**< RSTR0_TIMER0 Mask */

#define MXC_F_GCR_RSTR0_TIMER1_POS 6											 /**< RSTR0_TIMER1 Position */
#define MXC_F_GCR_RSTR0_TIMER1 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER1_POS)) /**< RSTR0_TIMER1 Mask */

#define MXC_F_GCR_RSTR0_TIMER2_POS 7											 /**< RSTR0_TIMER2 Position */
#define MXC_F_GCR_RSTR0_TIMER2 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER2_POS)) /**< RSTR0_TIMER2 Mask */

#define MXC_F_GCR_RSTR0_TIMER3_POS 8											 /**< RSTR0_TIMER3 Position */
#define MXC_F_GCR_RSTR0_TIMER3 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER3_POS)) /**< RSTR0_TIMER3 Mask */

#define MXC_F_GCR_RSTR0_UART0_POS 11										   /**< RSTR0_UART0 Position */
#define MXC_F_GCR_RSTR0_UART0 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_UART0_POS)) /**< RSTR0_UART0 Mask */

#define MXC_F_GCR_RSTR0_SPI0_POS 13											 /**< RSTR0_SPI0 Position */
#define MXC_F_GCR_RSTR0_SPI0 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SPI0_POS)) /**< RSTR0_SPI0 Mask */

#define MXC_F_GCR_RSTR0_SPI1_POS 14											 /**< RSTR0_SPI1 Position */
#define MXC_F_GCR_RSTR0_SPI1 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SPI1_POS)) /**< RSTR0_SPI1 Mask */

#define MXC_F_GCR_RSTR0_I2C0_POS 16											 /**< RSTR0_I2C0 Position */
#define MXC_F_GCR_RSTR0_I2C0 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_I2C0_POS)) /**< RSTR0_I2C0 Mask */

#define MXC_F_GCR_RSTR0_CRYPTO_POS 18											 /**< RSTR0_CRYPTO Position */
#define MXC_F_GCR_RSTR0_CRYPTO ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_CRYPTO_POS)) /**< RSTR0_CRYPTO Mask */

#define MXC_F_GCR_RSTR0_SMPHR_POS 22										   /**< RSTR0_SMPHR Position */
#define MXC_F_GCR_RSTR0_SMPHR ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SMPHR_POS)) /**< RSTR0_SMPHR Mask */

#define MXC_F_GCR_RSTR0_TRNG_POS 24											 /**< RSTR0_TRNG Position */
#define MXC_F_GCR_RSTR0_TRNG ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TRNG_POS)) /**< RSTR0_TRNG Mask */

#define MXC_F_GCR_RSTR0_SRST_POS 29											 /**< RSTR0_SRST Position */
#define MXC_F_GCR_RSTR0_SRST ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SRST_POS)) /**< RSTR0_SRST Mask */

#define MXC_F_GCR_RSTR0_PRST_POS 30											 /**< RSTR0_PRST Position */
#define MXC_F_GCR_RSTR0_PRST ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_PRST_POS)) /**< RSTR0_PRST Mask */

#define MXC_F_GCR_RSTR0_SYSTEM_POS 31											 /**< RSTR0_SYSTEM Position */
#define MXC_F_GCR_RSTR0_SYSTEM ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SYSTEM_POS)) /**< RSTR0_SYSTEM Mask */

/**@} end of group GCR_RSTR0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_CLKCN GCR_CLKCN
 * @brief    Clock Control.
 * @{
 */
#define MXC_F_GCR_CLKCN_PSC_POS 6														 /**< CLKCN_PSC Position */
#define MXC_F_GCR_CLKCN_PSC ((uint32_t)(0x7UL << MXC_F_GCR_CLKCN_PSC_POS))				 /**< CLKCN_PSC Mask */
#define MXC_V_GCR_CLKCN_PSC_DIV1 ((uint32_t)0x0UL)										 /**< CLKCN_PSC_DIV1 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV1 (MXC_V_GCR_CLKCN_PSC_DIV1 << MXC_F_GCR_CLKCN_PSC_POS)	 /**< CLKCN_PSC_DIV1 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV2 ((uint32_t)0x1UL)										 /**< CLKCN_PSC_DIV2 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV2 (MXC_V_GCR_CLKCN_PSC_DIV2 << MXC_F_GCR_CLKCN_PSC_POS)	 /**< CLKCN_PSC_DIV2 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV4 ((uint32_t)0x2UL)										 /**< CLKCN_PSC_DIV4 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV4 (MXC_V_GCR_CLKCN_PSC_DIV4 << MXC_F_GCR_CLKCN_PSC_POS)	 /**< CLKCN_PSC_DIV4 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV8 ((uint32_t)0x3UL)										 /**< CLKCN_PSC_DIV8 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV8 (MXC_V_GCR_CLKCN_PSC_DIV8 << MXC_F_GCR_CLKCN_PSC_POS)	 /**< CLKCN_PSC_DIV8 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV16 ((uint32_t)0x4UL)										 /**< CLKCN_PSC_DIV16 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV16 (MXC_V_GCR_CLKCN_PSC_DIV16 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV16 Setting  \
																						  */
#define MXC_V_GCR_CLKCN_PSC_DIV32 ((uint32_t)0x5UL)										 /**< CLKCN_PSC_DIV32 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV32 (MXC_V_GCR_CLKCN_PSC_DIV32 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV32 Setting  \
																						  */
#define MXC_V_GCR_CLKCN_PSC_DIV64 ((uint32_t)0x6UL)										 /**< CLKCN_PSC_DIV64 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV64 (MXC_V_GCR_CLKCN_PSC_DIV64 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV64 Setting  \
																						  */
#define MXC_V_GCR_CLKCN_PSC_DIV128 ((uint32_t)0x7UL)									 /**< CLKCN_PSC_DIV128 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV128                                                                                     \
	(MXC_V_GCR_CLKCN_PSC_DIV128 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV128 Setting */

#define MXC_F_GCR_CLKCN_CLKSEL_POS 9											 /**< CLKCN_CLKSEL Position */
#define MXC_F_GCR_CLKCN_CLKSEL ((uint32_t)(0x7UL << MXC_F_GCR_CLKCN_CLKSEL_POS)) /**< CLKCN_CLKSEL Mask */
#define MXC_V_GCR_CLKCN_CLKSEL_HIRC ((uint32_t)0x0UL)							 /**< CLKCN_CLKSEL_HIRC Value */
#define MXC_S_GCR_CLKCN_CLKSEL_HIRC                                                                                    \
	(MXC_V_GCR_CLKCN_CLKSEL_HIRC << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_HIRC Setting */
#define MXC_V_GCR_CLKCN_CLKSEL_LIRC8 ((uint32_t)0x3UL)			/**< CLKCN_CLKSEL_LIRC8 Value */
#define MXC_S_GCR_CLKCN_CLKSEL_LIRC8                                                                                   \
	(MXC_V_GCR_CLKCN_CLKSEL_LIRC8 << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_LIRC8 Setting */
#define MXC_V_GCR_CLKCN_CLKSEL_HIRC8 ((uint32_t)0x5UL)			 /**< CLKCN_CLKSEL_HIRC8 Value */
#define MXC_S_GCR_CLKCN_CLKSEL_HIRC8                                                                                   \
	(MXC_V_GCR_CLKCN_CLKSEL_HIRC8 << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_HIRC8 Setting */

#define MXC_F_GCR_CLKCN_CKRDY_POS 13										   /**< CLKCN_CKRDY Position */
#define MXC_F_GCR_CLKCN_CKRDY ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_CKRDY_POS)) /**< CLKCN_CKRDY Mask */
#define MXC_V_GCR_CLKCN_CKRDY_BUSY ((uint32_t)0x0UL)						   /**< CLKCN_CKRDY_BUSY Value */
#define MXC_S_GCR_CLKCN_CKRDY_BUSY                                                                                     \
	(MXC_V_GCR_CLKCN_CKRDY_BUSY << MXC_F_GCR_CLKCN_CKRDY_POS) /**< CLKCN_CKRDY_BUSY Setting */
#define MXC_V_GCR_CLKCN_CKRDY_READY ((uint32_t)0x1UL)		  /**< CLKCN_CKRDY_READY Value */
#define MXC_S_GCR_CLKCN_CKRDY_READY                                                                                    \
	(MXC_V_GCR_CLKCN_CKRDY_READY << MXC_F_GCR_CLKCN_CKRDY_POS) /**< CLKCN_CKRDY_READY Setting */

#define MXC_F_GCR_CLKCN_HIRC_EN_POS 18											   /**< CLKCN_HIRC_EN Position */
#define MXC_F_GCR_CLKCN_HIRC_EN ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC_EN_POS)) /**< CLKCN_HIRC_EN Mask */
#define MXC_V_GCR_CLKCN_HIRC_EN_DIS ((uint32_t)0x0UL)							   /**< CLKCN_HIRC_EN_DIS Value */
#define MXC_S_GCR_CLKCN_HIRC_EN_DIS                                                                                    \
	(MXC_V_GCR_CLKCN_HIRC_EN_DIS << MXC_F_GCR_CLKCN_HIRC_EN_POS) /**< CLKCN_HIRC_EN_DIS Setting */
#define MXC_V_GCR_CLKCN_HIRC_EN_EN ((uint32_t)0x1UL)			 /**< CLKCN_HIRC_EN_EN Value */
#define MXC_S_GCR_CLKCN_HIRC_EN_EN                                                                                     \
	(MXC_V_GCR_CLKCN_HIRC_EN_EN << MXC_F_GCR_CLKCN_HIRC_EN_POS) /**< CLKCN_HIRC_EN_EN Setting */

#define MXC_F_GCR_CLKCN_HIRC8M_EN_POS 20											   /**< CLKCN_HIRC8M_EN Position */
#define MXC_F_GCR_CLKCN_HIRC8M_EN ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC8M_EN_POS)) /**< CLKCN_HIRC8M_EN Mask */

#define MXC_F_GCR_CLKCN_HIRC8M_VS_POS 21											   /**< CLKCN_HIRC8M_VS Position */
#define MXC_F_GCR_CLKCN_HIRC8M_VS ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC8M_VS_POS)) /**< CLKCN_HIRC8M_VS Mask */
#define MXC_V_GCR_CLKCN_HIRC8M_VS_VCOR ((uint32_t)0x0UL) /**< CLKCN_HIRC8M_VS_VCOR Value */
#define MXC_S_GCR_CLKCN_HIRC8M_VS_VCOR                                                                                 \
	(MXC_V_GCR_CLKCN_HIRC8M_VS_VCOR << MXC_F_GCR_CLKCN_HIRC8M_VS_POS) /**< CLKCN_HIRC8M_VS_VCOR Setting */
#define MXC_V_GCR_CLKCN_HIRC8M_VS_1V ((uint32_t)0x1UL)				  /**< CLKCN_HIRC8M_VS_1V Value */
#define MXC_S_GCR_CLKCN_HIRC8M_VS_1V                                                                                   \
	(MXC_V_GCR_CLKCN_HIRC8M_VS_1V << MXC_F_GCR_CLKCN_HIRC8M_VS_POS) /**< CLKCN_HIRC8M_VS_1V Setting */

#define MXC_F_GCR_CLKCN_HIRC_RDY_POS 26												 /**< CLKCN_HIRC_RDY Position */
#define MXC_F_GCR_CLKCN_HIRC_RDY ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC_RDY_POS)) /**< CLKCN_HIRC_RDY Mask */
#define MXC_V_GCR_CLKCN_HIRC_RDY_NOT ((uint32_t)0x0UL)								 /**< CLKCN_HIRC_RDY_NOT Value */
#define MXC_S_GCR_CLKCN_HIRC_RDY_NOT                                                                                   \
	(MXC_V_GCR_CLKCN_HIRC_RDY_NOT << MXC_F_GCR_CLKCN_HIRC_RDY_POS) /**< CLKCN_HIRC_RDY_NOT Setting */
#define MXC_V_GCR_CLKCN_HIRC_RDY_READY ((uint32_t)0x1UL)		   /**< CLKCN_HIRC_RDY_READY Value */
#define MXC_S_GCR_CLKCN_HIRC_RDY_READY                                                                                 \
	(MXC_V_GCR_CLKCN_HIRC_RDY_READY << MXC_F_GCR_CLKCN_HIRC_RDY_POS) /**< CLKCN_HIRC_RDY_READY Setting */

#define MXC_F_GCR_CLKCN_HIRC8M_RDY_POS 28 /**< CLKCN_HIRC8M_RDY Position */
#define MXC_F_GCR_CLKCN_HIRC8M_RDY ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC8M_RDY_POS)) /**< CLKCN_HIRC8M_RDY Mask */

#define MXC_F_GCR_CLKCN_LIRC8K_RDY_POS 29 /**< CLKCN_LIRC8K_RDY Position */
#define MXC_F_GCR_CLKCN_LIRC8K_RDY ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_LIRC8K_RDY_POS)) /**< CLKCN_LIRC8K_RDY Mask */

/**@} end of group GCR_CLKCN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PM GCR_PM
 * @brief    Power Management.
 * @{
 */
#define MXC_F_GCR_PM_MODE_POS 0														 /**< PM_MODE Position */
#define MXC_F_GCR_PM_MODE ((uint32_t)(0x7UL << MXC_F_GCR_PM_MODE_POS))				 /**< PM_MODE Mask */
#define MXC_V_GCR_PM_MODE_ACTIVE ((uint32_t)0x0UL)									 /**< PM_MODE_ACTIVE Value */
#define MXC_S_GCR_PM_MODE_ACTIVE (MXC_V_GCR_PM_MODE_ACTIVE << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_ACTIVE Setting */
#define MXC_V_GCR_PM_MODE_DEEPSLEEP ((uint32_t)0x2UL)								 /**< PM_MODE_DEEPSLEEP Value */
#define MXC_S_GCR_PM_MODE_DEEPSLEEP                                                                                    \
	(MXC_V_GCR_PM_MODE_DEEPSLEEP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_DEEPSLEEP Setting */
#define MXC_V_GCR_PM_MODE_SHUTDOWN ((uint32_t)0x3UL)	   /**< PM_MODE_SHUTDOWN Value */
#define MXC_S_GCR_PM_MODE_SHUTDOWN                                                                                     \
	(MXC_V_GCR_PM_MODE_SHUTDOWN << MXC_F_GCR_PM_MODE_POS)							 /**< PM_MODE_SHUTDOWN Setting */
#define MXC_V_GCR_PM_MODE_BACKUP ((uint32_t)0x4UL)									 /**< PM_MODE_BACKUP Value */
#define MXC_S_GCR_PM_MODE_BACKUP (MXC_V_GCR_PM_MODE_BACKUP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_BACKUP Setting */

#define MXC_F_GCR_PM_GPIOWKEN_POS 4											   /**< PM_GPIOWKEN Position */
#define MXC_F_GCR_PM_GPIOWKEN ((uint32_t)(0x1UL << MXC_F_GCR_PM_GPIOWKEN_POS)) /**< PM_GPIOWKEN Mask */
#define MXC_V_GCR_PM_GPIOWKEN_DIS ((uint32_t)0x0UL)							   /**< PM_GPIOWKEN_DIS Value */
#define MXC_S_GCR_PM_GPIOWKEN_DIS                                                                                      \
	(MXC_V_GCR_PM_GPIOWKEN_DIS << MXC_F_GCR_PM_GPIOWKEN_POS)							 /**< PM_GPIOWKEN_DIS Setting */
#define MXC_V_GCR_PM_GPIOWKEN_EN ((uint32_t)0x1UL)										 /**< PM_GPIOWKEN_EN Value */
#define MXC_S_GCR_PM_GPIOWKEN_EN (MXC_V_GCR_PM_GPIOWKEN_EN << MXC_F_GCR_PM_GPIOWKEN_POS) /**< PM_GPIOWKEN_EN Setting   \
																						  */

#define MXC_F_GCR_PM_HIRCPD_POS 15										   /**< PM_HIRCPD Position */
#define MXC_F_GCR_PM_HIRCPD ((uint32_t)(0x1UL << MXC_F_GCR_PM_HIRCPD_POS)) /**< PM_HIRCPD Mask */
#define MXC_V_GCR_PM_HIRCPD_ACTIVE ((uint32_t)0x0UL)					   /**< PM_HIRCPD_ACTIVE Value */
#define MXC_S_GCR_PM_HIRCPD_ACTIVE                                                                                     \
	(MXC_V_GCR_PM_HIRCPD_ACTIVE << MXC_F_GCR_PM_HIRCPD_POS) /**< PM_HIRCPD_ACTIVE Setting */
#define MXC_V_GCR_PM_HIRCPD_DEEPSLEEP ((uint32_t)0x1UL)		/**< PM_HIRCPD_DEEPSLEEP Value */
#define MXC_S_GCR_PM_HIRCPD_DEEPSLEEP                                                                                  \
	(MXC_V_GCR_PM_HIRCPD_DEEPSLEEP << MXC_F_GCR_PM_HIRCPD_POS) /**< PM_HIRCPD_DEEPSLEEP Setting */

#define MXC_F_GCR_PM_HIRC8MPD_POS 17										   /**< PM_HIRC8MPD Position */
#define MXC_F_GCR_PM_HIRC8MPD ((uint32_t)(0x1UL << MXC_F_GCR_PM_HIRC8MPD_POS)) /**< PM_HIRC8MPD Mask */

/**@} end of group GCR_PM_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCKDIV GCR_PCKDIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCKDIV_PCF_POS 0											 /**< PCKDIV_PCF Position */
#define MXC_F_GCR_PCKDIV_PCF ((uint32_t)(0x7UL << MXC_F_GCR_PCKDIV_PCF_POS)) /**< PCKDIV_PCF Mask */
#define MXC_V_GCR_PCKDIV_PCF_96MHZ ((uint32_t)0x2UL)						 /**< PCKDIV_PCF_96MHZ Value */
#define MXC_S_GCR_PCKDIV_PCF_96MHZ                                                                                     \
	(MXC_V_GCR_PCKDIV_PCF_96MHZ << MXC_F_GCR_PCKDIV_PCF_POS) /**< PCKDIV_PCF_96MHZ Setting */
#define MXC_V_GCR_PCKDIV_PCF_48MHZ ((uint32_t)0x3UL)		 /**< PCKDIV_PCF_48MHZ Value */
#define MXC_S_GCR_PCKDIV_PCF_48MHZ                                                                                     \
	(MXC_V_GCR_PCKDIV_PCF_48MHZ << MXC_F_GCR_PCKDIV_PCF_POS) /**< PCKDIV_PCF_48MHZ Setting */
#define MXC_V_GCR_PCKDIV_PCF_24MHZ ((uint32_t)0x4UL)		 /**< PCKDIV_PCF_24MHZ Value */
#define MXC_S_GCR_PCKDIV_PCF_24MHZ                                                                                     \
	(MXC_V_GCR_PCKDIV_PCF_24MHZ << MXC_F_GCR_PCKDIV_PCF_POS) /**< PCKDIV_PCF_24MHZ Setting */
#define MXC_V_GCR_PCKDIV_PCF_12MHZ ((uint32_t)0x5UL)		 /**< PCKDIV_PCF_12MHZ Value */
#define MXC_S_GCR_PCKDIV_PCF_12MHZ                                                                                     \
	(MXC_V_GCR_PCKDIV_PCF_12MHZ << MXC_F_GCR_PCKDIV_PCF_POS) /**< PCKDIV_PCF_12MHZ Setting */
#define MXC_V_GCR_PCKDIV_PCF_6MHZ ((uint32_t)0x6UL)			 /**< PCKDIV_PCF_6MHZ Value */
#define MXC_S_GCR_PCKDIV_PCF_6MHZ                                                                                      \
	(MXC_V_GCR_PCKDIV_PCF_6MHZ << MXC_F_GCR_PCKDIV_PCF_POS) /**< PCKDIV_PCF_6MHZ Setting */
#define MXC_V_GCR_PCKDIV_PCF_3MHZ ((uint32_t)0x7UL)			/**< PCKDIV_PCF_3MHZ Value */
#define MXC_S_GCR_PCKDIV_PCF_3MHZ                                                                                      \
	(MXC_V_GCR_PCKDIV_PCF_3MHZ << MXC_F_GCR_PCKDIV_PCF_POS) /**< PCKDIV_PCF_3MHZ Setting */

#define MXC_F_GCR_PCKDIV_PCFWEN_POS 3											   /**< PCKDIV_PCFWEN Position */
#define MXC_F_GCR_PCKDIV_PCFWEN ((uint32_t)(0x1UL << MXC_F_GCR_PCKDIV_PCFWEN_POS)) /**< PCKDIV_PCFWEN Mask */
#define MXC_V_GCR_PCKDIV_PCFWEN_DISABLED ((uint32_t)0x0UL)						   /**< PCKDIV_PCFWEN_DISABLED Value */
#define MXC_S_GCR_PCKDIV_PCFWEN_DISABLED                                                                               \
	(MXC_V_GCR_PCKDIV_PCFWEN_DISABLED << MXC_F_GCR_PCKDIV_PCFWEN_POS) /**< PCKDIV_PCFWEN_DISABLED Setting */
#define MXC_V_GCR_PCKDIV_PCFWEN_ENABLED ((uint32_t)0x1UL)			  /**< PCKDIV_PCFWEN_ENABLED Value */
#define MXC_S_GCR_PCKDIV_PCFWEN_ENABLED                                                                                \
	(MXC_V_GCR_PCKDIV_PCFWEN_ENABLED << MXC_F_GCR_PCKDIV_PCFWEN_POS) /**< PCKDIV_PCFWEN_ENABLED Setting */

#define MXC_F_GCR_PCKDIV_AONCD_POS 14											 /**< PCKDIV_AONCD Position */
#define MXC_F_GCR_PCKDIV_AONCD ((uint32_t)(0x3UL << MXC_F_GCR_PCKDIV_AONCD_POS)) /**< PCKDIV_AONCD Mask */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_4 ((uint32_t)0x0UL)							 /**< PCKDIV_AONCD_DIV_4 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_4                                                                                   \
	(MXC_V_GCR_PCKDIV_AONCD_DIV_4 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_4 Setting */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_8 ((uint32_t)0x1UL)			 /**< PCKDIV_AONCD_DIV_8 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_8                                                                                   \
	(MXC_V_GCR_PCKDIV_AONCD_DIV_8 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_8 Setting */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_16 ((uint32_t)0x2UL)			 /**< PCKDIV_AONCD_DIV_16 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_16                                                                                  \
	(MXC_V_GCR_PCKDIV_AONCD_DIV_16 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_16 Setting */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_32 ((uint32_t)0x3UL)			  /**< PCKDIV_AONCD_DIV_32 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_32                                                                                  \
	(MXC_V_GCR_PCKDIV_AONCD_DIV_32 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_32 Setting */

/**@} end of group GCR_PCKDIV_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PERCKCN0 GCR_PERCKCN0
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PERCKCN0_GPIO0D_POS 0												   /**< PERCKCN0_GPIO0D Position */
#define MXC_F_GCR_PERCKCN0_GPIO0D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_GPIO0D_POS)) /**< PERCKCN0_GPIO0D Mask */
#define MXC_V_GCR_PERCKCN0_GPIO0D_EN ((uint32_t)0x0UL)								   /**< PERCKCN0_GPIO0D_EN Value */
#define MXC_S_GCR_PERCKCN0_GPIO0D_EN                                                                                   \
	(MXC_V_GCR_PERCKCN0_GPIO0D_EN << MXC_F_GCR_PERCKCN0_GPIO0D_POS) /**< PERCKCN0_GPIO0D_EN Setting */
#define MXC_V_GCR_PERCKCN0_GPIO0D_DIS ((uint32_t)0x1UL)				/**< PERCKCN0_GPIO0D_DIS Value */
#define MXC_S_GCR_PERCKCN0_GPIO0D_DIS                                                                                  \
	(MXC_V_GCR_PERCKCN0_GPIO0D_DIS << MXC_F_GCR_PERCKCN0_GPIO0D_POS) /**< PERCKCN0_GPIO0D_DIS Setting */

#define MXC_F_GCR_PERCKCN0_GPIO1D_POS 1												   /**< PERCKCN0_GPIO1D Position */
#define MXC_F_GCR_PERCKCN0_GPIO1D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_GPIO1D_POS)) /**< PERCKCN0_GPIO1D Mask */

#define MXC_F_GCR_PERCKCN0_DMAD_POS 5											   /**< PERCKCN0_DMAD Position */
#define MXC_F_GCR_PERCKCN0_DMAD ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_DMAD_POS)) /**< PERCKCN0_DMAD Mask */

#define MXC_F_GCR_PERCKCN0_SPI0D_POS 6												 /**< PERCKCN0_SPI0D Position */
#define MXC_F_GCR_PERCKCN0_SPI0D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_SPI0D_POS)) /**< PERCKCN0_SPI0D Mask */

#define MXC_F_GCR_PERCKCN0_SPI1D_POS 7												 /**< PERCKCN0_SPI1D Position */
#define MXC_F_GCR_PERCKCN0_SPI1D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_SPI1D_POS)) /**< PERCKCN0_SPI1D Mask */

#define MXC_F_GCR_PERCKCN0_UART0D_POS 9												   /**< PERCKCN0_UART0D Position */
#define MXC_F_GCR_PERCKCN0_UART0D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_UART0D_POS)) /**< PERCKCN0_UART0D Mask */

#define MXC_F_GCR_PERCKCN0_I2C0D_POS 13												 /**< PERCKCN0_I2C0D Position */
#define MXC_F_GCR_PERCKCN0_I2C0D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_I2C0D_POS)) /**< PERCKCN0_I2C0D Mask */

#define MXC_F_GCR_PERCKCN0_CRYPTOD_POS 14 /**< PERCKCN0_CRYPTOD Position */
#define MXC_F_GCR_PERCKCN0_CRYPTOD ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_CRYPTOD_POS)) /**< PERCKCN0_CRYPTOD Mask */

#define MXC_F_GCR_PERCKCN0_T0D_POS 15											 /**< PERCKCN0_T0D Position */
#define MXC_F_GCR_PERCKCN0_T0D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_T0D_POS)) /**< PERCKCN0_T0D Mask */

#define MXC_F_GCR_PERCKCN0_T1D_POS 16											 /**< PERCKCN0_T1D Position */
#define MXC_F_GCR_PERCKCN0_T1D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_T1D_POS)) /**< PERCKCN0_T1D Mask */

#define MXC_F_GCR_PERCKCN0_T2D_POS 17											 /**< PERCKCN0_T2D Position */
#define MXC_F_GCR_PERCKCN0_T2D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_T2D_POS)) /**< PERCKCN0_T2D Mask */

#define MXC_F_GCR_PERCKCN0_T3D_POS 18											 /**< PERCKCN0_T3D Position */
#define MXC_F_GCR_PERCKCN0_T3D ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_T3D_POS)) /**< PERCKCN0_T3D Mask */

/**@} end of group GCR_PERCKCN0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMCKCN GCR_MEMCKCN
 * @brief    Memory Clock Control Register.
 * @{
 */
#define MXC_F_GCR_MEMCKCN_FWS_POS 0											   /**< MEMCKCN_FWS Position */
#define MXC_F_GCR_MEMCKCN_FWS ((uint32_t)(0x7UL << MXC_F_GCR_MEMCKCN_FWS_POS)) /**< MEMCKCN_FWS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM0LS_POS 16 /**< MEMCKCN_SYSRAM0LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM0LS                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM0LS_POS))	 /**< MEMCKCN_SYSRAM0LS Mask */
#define MXC_V_GCR_MEMCKCN_SYSRAM0LS_ACTIVE ((uint32_t)0x0UL) /**< MEMCKCN_SYSRAM0LS_ACTIVE Value */
#define MXC_S_GCR_MEMCKCN_SYSRAM0LS_ACTIVE                                                                             \
	(MXC_V_GCR_MEMCKCN_SYSRAM0LS_ACTIVE << MXC_F_GCR_MEMCKCN_SYSRAM0LS_POS) /**< MEMCKCN_SYSRAM0LS_ACTIVE Setting */
#define MXC_V_GCR_MEMCKCN_SYSRAM0LS_LIGHT_SLEEP ((uint32_t)0x1UL)			/**< MEMCKCN_SYSRAM0LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEMCKCN_SYSRAM0LS_LIGHT_SLEEP                                                                        \
	(MXC_V_GCR_MEMCKCN_SYSRAM0LS_LIGHT_SLEEP                                                                           \
	 << MXC_F_GCR_MEMCKCN_SYSRAM0LS_POS) /**< MEMCKCN_SYSRAM0LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEMCKCN_SYSRAM1LS_POS 17 /**< MEMCKCN_SYSRAM1LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM1LS                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM1LS_POS)) /**< MEMCKCN_SYSRAM1LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM2LS_POS 18 /**< MEMCKCN_SYSRAM2LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM2LS                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM2LS_POS)) /**< MEMCKCN_SYSRAM2LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM3LS_POS 19 /**< MEMCKCN_SYSRAM3LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM3LS                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM3LS_POS)) /**< MEMCKCN_SYSRAM3LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM4LS_POS 20 /**< MEMCKCN_SYSRAM4LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM4LS                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM4LS_POS)) /**< MEMCKCN_SYSRAM4LS Mask */

#define MXC_F_GCR_MEMCKCN_ICACHELS_POS 24 /**< MEMCKCN_ICACHELS Position */
#define MXC_F_GCR_MEMCKCN_ICACHELS ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_ICACHELS_POS)) /**< MEMCKCN_ICACHELS Mask */

#define MXC_F_GCR_MEMCKCN_ROMLS_POS 29											   /**< MEMCKCN_ROMLS Position */
#define MXC_F_GCR_MEMCKCN_ROMLS ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_ROMLS_POS)) /**< MEMCKCN_ROMLS Mask */

/**@} end of group GCR_MEMCKCN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMZCN GCR_MEMZCN
 * @brief    Memory Zeroize Control.
 * @{
 */
#define MXC_F_GCR_MEMZCN_SRAM0Z_POS 0											   /**< MEMZCN_SRAM0Z Position */
#define MXC_F_GCR_MEMZCN_SRAM0Z ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM0Z_POS)) /**< MEMZCN_SRAM0Z Mask */
#define MXC_V_GCR_MEMZCN_SRAM0Z_NOP ((uint32_t)0x0UL)							   /**< MEMZCN_SRAM0Z_NOP Value */
#define MXC_S_GCR_MEMZCN_SRAM0Z_NOP                                                                                    \
	(MXC_V_GCR_MEMZCN_SRAM0Z_NOP << MXC_F_GCR_MEMZCN_SRAM0Z_POS) /**< MEMZCN_SRAM0Z_NOP Setting */
#define MXC_V_GCR_MEMZCN_SRAM0Z_START ((uint32_t)0x1UL)			 /**< MEMZCN_SRAM0Z_START Value */
#define MXC_S_GCR_MEMZCN_SRAM0Z_START                                                                                  \
	(MXC_V_GCR_MEMZCN_SRAM0Z_START << MXC_F_GCR_MEMZCN_SRAM0Z_POS) /**< MEMZCN_SRAM0Z_START Setting */

#define MXC_F_GCR_MEMZCN_SRAM1Z_POS 1											   /**< MEMZCN_SRAM1Z Position */
#define MXC_F_GCR_MEMZCN_SRAM1Z ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM1Z_POS)) /**< MEMZCN_SRAM1Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM2Z_POS 2											   /**< MEMZCN_SRAM2Z Position */
#define MXC_F_GCR_MEMZCN_SRAM2Z ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM2Z_POS)) /**< MEMZCN_SRAM2Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM3Z_POS 3											   /**< MEMZCN_SRAM3Z Position */
#define MXC_F_GCR_MEMZCN_SRAM3Z ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM3Z_POS)) /**< MEMZCN_SRAM3Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM4Z_POS 4											   /**< MEMZCN_SRAM4Z Position */
#define MXC_F_GCR_MEMZCN_SRAM4Z ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM4Z_POS)) /**< MEMZCN_SRAM4Z Mask */

#define MXC_F_GCR_MEMZCN_ICACHEZ_POS 8												 /**< MEMZCN_ICACHEZ Position */
#define MXC_F_GCR_MEMZCN_ICACHEZ ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_ICACHEZ_POS)) /**< MEMZCN_ICACHEZ Mask */

/**@} end of group GCR_MEMZCN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSST GCR_SYSST
 * @brief    System Status Register.
 * @{
 */
#define MXC_F_GCR_SYSST_ICECLOCK_POS 0												 /**< SYSST_ICECLOCK Position */
#define MXC_F_GCR_SYSST_ICECLOCK ((uint32_t)(0x1UL << MXC_F_GCR_SYSST_ICECLOCK_POS)) /**< SYSST_ICECLOCK Mask */
#define MXC_V_GCR_SYSST_ICECLOCK_UNLOCKED ((uint32_t)0x0UL) /**< SYSST_ICECLOCK_UNLOCKED Value */
#define MXC_S_GCR_SYSST_ICECLOCK_UNLOCKED                                                                              \
	(MXC_V_GCR_SYSST_ICECLOCK_UNLOCKED << MXC_F_GCR_SYSST_ICECLOCK_POS) /**< SYSST_ICECLOCK_UNLOCKED Setting */
#define MXC_V_GCR_SYSST_ICECLOCK_LOCKED ((uint32_t)0x1UL)				/**< SYSST_ICECLOCK_LOCKED Value */
#define MXC_S_GCR_SYSST_ICECLOCK_LOCKED                                                                                \
	(MXC_V_GCR_SYSST_ICECLOCK_LOCKED << MXC_F_GCR_SYSST_ICECLOCK_POS) /**< SYSST_ICECLOCK_LOCKED Setting */

/**@} end of group GCR_SYSST_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_RSTR1 GCR_RSTR1
 * @brief    Reset 1.
 * @{
 */
#define MXC_F_GCR_RSTR1_WDT1_POS 8											 /**< RSTR1_WDT1 Position */
#define MXC_F_GCR_RSTR1_WDT1 ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_WDT1_POS)) /**< RSTR1_WDT1 Mask */
#define MXC_V_GCR_RSTR1_WDT1_RESET_DONE ((uint32_t)0x0UL)					 /**< RSTR1_WDT1_RESET_DONE Value */
#define MXC_S_GCR_RSTR1_WDT1_RESET_DONE                                                                                \
	(MXC_V_GCR_RSTR1_WDT1_RESET_DONE << MXC_F_GCR_RSTR1_WDT1_POS) /**< RSTR1_WDT1_RESET_DONE Setting */
#define MXC_V_GCR_RSTR1_WDT1_BUSY ((uint32_t)0x1UL)				  /**< RSTR1_WDT1_BUSY Value */
#define MXC_S_GCR_RSTR1_WDT1_BUSY                                                                                      \
	(MXC_V_GCR_RSTR1_WDT1_BUSY << MXC_F_GCR_RSTR1_WDT1_POS) /**< RSTR1_WDT1_BUSY Setting */

#define MXC_F_GCR_RSTR1_PUFC_POS 27											 /**< RSTR1_PUFC Position */
#define MXC_F_GCR_RSTR1_PUFC ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_PUFC_POS)) /**< RSTR1_PUFC Mask */

#define MXC_F_GCR_RSTR1_CSPIS_POS 28										   /**< RSTR1_CSPIS Position */
#define MXC_F_GCR_RSTR1_CSPIS ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_CSPIS_POS)) /**< RSTR1_CSPIS Mask */

/**@} end of group GCR_RSTR1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PERCKCN1 GCR_PERCKCN1
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PERCKCN1_TRNGD_POS 2												 /**< PERCKCN1_TRNGD Position */
#define MXC_F_GCR_PERCKCN1_TRNGD ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_TRNGD_POS)) /**< PERCKCN1_TRNGD Mask */
#define MXC_V_GCR_PERCKCN1_TRNGD_EN ((uint32_t)0x0UL)								 /**< PERCKCN1_TRNGD_EN Value */
#define MXC_S_GCR_PERCKCN1_TRNGD_EN                                                                                    \
	(MXC_V_GCR_PERCKCN1_TRNGD_EN << MXC_F_GCR_PERCKCN1_TRNGD_POS) /**< PERCKCN1_TRNGD_EN Setting */
#define MXC_V_GCR_PERCKCN1_TRNGD_DIS ((uint32_t)0x1UL)			  /**< PERCKCN1_TRNGD_DIS Value */
#define MXC_S_GCR_PERCKCN1_TRNGD_DIS                                                                                   \
	(MXC_V_GCR_PERCKCN1_TRNGD_DIS << MXC_F_GCR_PERCKCN1_TRNGD_POS) /**< PERCKCN1_TRNGD_DIS Setting */

#define MXC_F_GCR_PERCKCN1_PUFCD_POS 3												 /**< PERCKCN1_PUFCD Position */
#define MXC_F_GCR_PERCKCN1_PUFCD ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_PUFCD_POS)) /**< PERCKCN1_PUFCD Mask */

#define MXC_F_GCR_PERCKCN1_ICACHED_POS 11 /**< PERCKCN1_ICACHED Position */
#define MXC_F_GCR_PERCKCN1_ICACHED ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_ICACHED_POS)) /**< PERCKCN1_ICACHED Mask */

#define MXC_F_GCR_PERCKCN1_CSPISD_POS 30											   /**< PERCKCN1_CSPISD Position */
#define MXC_F_GCR_PERCKCN1_CSPISD ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_CSPISD_POS)) /**< PERCKCN1_CSPISD Mask */

/**@} end of group GCR_PERCKCN1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_EVTEN GCR_EVTEN
 * @brief    Event Enable Register.
 * @{
 */
#define MXC_F_GCR_EVTEN_CPU0DMAEVENT_POS 0 /**< EVTEN_CPU0DMAEVENT Position */
#define MXC_F_GCR_EVTEN_CPU0DMAEVENT                                                                                   \
	((uint32_t)(0x1UL << MXC_F_GCR_EVTEN_CPU0DMAEVENT_POS)) /**< EVTEN_CPU0DMAEVENT Mask */

#define MXC_F_GCR_EVTEN_CPU0RXEVENT_POS 1 /**< EVTEN_CPU0RXEVENT Position */
#define MXC_F_GCR_EVTEN_CPU0RXEVENT                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_EVTEN_CPU0RXEVENT_POS)) /**< EVTEN_CPU0RXEVENT Mask */

#define MXC_F_GCR_EVTEN_CPU0TXEVENT_POS 2 /**< EVTEN_CPU0TXEVENT Position */
#define MXC_F_GCR_EVTEN_CPU0TXEVENT                                                                                    \
	((uint32_t)(0x1UL << MXC_F_GCR_EVTEN_CPU0TXEVENT_POS)) /**< EVTEN_CPU0TXEVENT Mask */

/**@} end of group GCR_EVTEN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_REVISION GCR_REVISION
 * @brief    Revision Register.
 * @{
 */
#define MXC_F_GCR_REVISION_REVISION_POS 0 /**< REVISION_REVISION Position */
#define MXC_F_GCR_REVISION_REVISION                                                                                    \
	((uint32_t)(0xFFFFUL << MXC_F_GCR_REVISION_REVISION_POS)) /**< REVISION_REVISION Mask */

/**@} end of group GCR_REVISION_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSSIE GCR_SYSSIE
 * @brief    System Status Interrupt Enable Register.
 * @{
 */
#define MXC_F_GCR_SYSSIE_ICEULIE_POS 0												 /**< SYSSIE_ICEULIE Position */
#define MXC_F_GCR_SYSSIE_ICEULIE ((uint32_t)(0x1UL << MXC_F_GCR_SYSSIE_ICEULIE_POS)) /**< SYSSIE_ICEULIE Mask */
#define MXC_V_GCR_SYSSIE_ICEULIE_DIS ((uint32_t)0x0UL)								 /**< SYSSIE_ICEULIE_DIS Value */
#define MXC_S_GCR_SYSSIE_ICEULIE_DIS                                                                                   \
	(MXC_V_GCR_SYSSIE_ICEULIE_DIS << MXC_F_GCR_SYSSIE_ICEULIE_POS) /**< SYSSIE_ICEULIE_DIS Setting */
#define MXC_V_GCR_SYSSIE_ICEULIE_EN ((uint32_t)0x1UL)			   /**< SYSSIE_ICEULIE_EN Value */
#define MXC_S_GCR_SYSSIE_ICEULIE_EN                                                                                    \
	(MXC_V_GCR_SYSSIE_ICEULIE_EN << MXC_F_GCR_SYSSIE_ICEULIE_POS) /**< SYSSIE_ICEULIE_EN Setting */

/**@} end of group GCR_SYSSIE_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCERR GCR_ECCERR
 * @brief    ECC Error Register
 * @{
 */
#define MXC_F_GCR_ECCERR_SYSRAM0ECCERR_POS 0 /**< ECCERR_SYSRAM0ECCERR Position */
#define MXC_F_GCR_ECCERR_SYSRAM0ECCERR                                                                                 \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_SYSRAM0ECCERR_POS)) /**< ECCERR_SYSRAM0ECCERR Mask */

#define MXC_F_GCR_ECCERR_SYSRAM1ECCERR_POS 1 /**< ECCERR_SYSRAM1ECCERR Position */
#define MXC_F_GCR_ECCERR_SYSRAM1ECCERR                                                                                 \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_SYSRAM1ECCERR_POS)) /**< ECCERR_SYSRAM1ECCERR Mask */

#define MXC_F_GCR_ECCERR_SYSRAM2ECCERR_POS 2 /**< ECCERR_SYSRAM2ECCERR Position */
#define MXC_F_GCR_ECCERR_SYSRAM2ECCERR                                                                                 \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_SYSRAM2ECCERR_POS)) /**< ECCERR_SYSRAM2ECCERR Mask */

#define MXC_F_GCR_ECCERR_SYSRAM3ECCERR_POS 3 /**< ECCERR_SYSRAM3ECCERR Position */
#define MXC_F_GCR_ECCERR_SYSRAM3ECCERR                                                                                 \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_SYSRAM3ECCERR_POS)) /**< ECCERR_SYSRAM3ECCERR Mask */

#define MXC_F_GCR_ECCERR_SYSRAM4ECCERR_POS 4 /**< ECCERR_SYSRAM4ECCERR Position */
#define MXC_F_GCR_ECCERR_SYSRAM4ECCERR                                                                                 \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_SYSRAM4ECCERR_POS)) /**< ECCERR_SYSRAM4ECCERR Mask */

/**@} end of group GCR_ECCERR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCNDED GCR_ECCNDED
 * @brief    ECC Not Double Error Detect Register
 * @{
 */
#define MXC_F_GCR_ECCNDED_SYSRAM0ECCNDED_POS 0 /**< ECCNDED_SYSRAM0ECCNDED Position */
#define MXC_F_GCR_ECCNDED_SYSRAM0ECCNDED                                                                               \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCNDED_SYSRAM0ECCNDED_POS)) /**< ECCNDED_SYSRAM0ECCNDED Mask */

#define MXC_F_GCR_ECCNDED_SYSRAM1ECCNDED_POS 1 /**< ECCNDED_SYSRAM1ECCNDED Position */
#define MXC_F_GCR_ECCNDED_SYSRAM1ECCNDED                                                                               \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCNDED_SYSRAM1ECCNDED_POS)) /**< ECCNDED_SYSRAM1ECCNDED Mask */

#define MXC_F_GCR_ECCNDED_SYSRAM2ECCNDED_POS 2 /**< ECCNDED_SYSRAM2ECCNDED Position */
#define MXC_F_GCR_ECCNDED_SYSRAM2ECCNDED                                                                               \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCNDED_SYSRAM2ECCNDED_POS)) /**< ECCNDED_SYSRAM2ECCNDED Mask */

#define MXC_F_GCR_ECCNDED_SYSRAM3ECCNDED_POS 3 /**< ECCNDED_SYSRAM3ECCNDED Position */
#define MXC_F_GCR_ECCNDED_SYSRAM3ECCNDED                                                                               \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCNDED_SYSRAM3ECCNDED_POS)) /**< ECCNDED_SYSRAM3ECCNDED Mask */

#define MXC_F_GCR_ECCNDED_SYSRAM4ECCNDED_POS 4 /**< ECCNDED_SYSRAM4ECCNDED Position */
#define MXC_F_GCR_ECCNDED_SYSRAM4ECCNDED                                                                               \
	((uint32_t)(0x1UL << MXC_F_GCR_ECCNDED_SYSRAM4ECCNDED_POS)) /**< ECCNDED_SYSRAM4ECCNDED Mask */

/**@} end of group GCR_ECCNDED_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCIRQEN GCR_ECCIRQEN
 * @brief    ECC IRQ Enable Register
 * @{
 */
#define MXC_F_GCR_ECCIRQEN_ECCERAD_POS 0 /**< ECCIRQEN_ECCERAD Position */
#define MXC_F_GCR_ECCIRQEN_ECCERAD                                                                                     \
	((uint32_t)(0x7FFFFFFFUL << MXC_F_GCR_ECCIRQEN_ECCERAD_POS)) /**< ECCIRQEN_ECCERAD Mask */

/**@} end of group GCR_ECCIRQEN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCERRAD GCR_ECCERRAD
 * @brief    ECC Error Address Register
 * @{
 */
#define MXC_F_GCR_ECCERRAD_ECCERAD_POS 0 /**< ECCERRAD_ECCERAD Position */
#define MXC_F_GCR_ECCERRAD_ECCERAD                                                                                     \
	((uint32_t)(0x7FFFFFFFUL << MXC_F_GCR_ECCERRAD_ECCERAD_POS)) /**< ECCERRAD_ECCERAD Mask */

/**@} end of group GCR_ECCERRAD_Register */

#ifdef __cplusplus
}
#endif

#endif /* _GCR_REGS_H_ */

/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2016 by Maxim Integrated                                *
 *   Copyright (C) 2025-2026 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef _CTB_REGS_H_
#define _CTB_REGS_H_

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
 * @ingroup     ctb
 * @defgroup    ctb_registers CTB_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the CTB Peripheral Module.
 * @details The Cryptographic Toolbox is a combination of cryptographic engines and a secure cryptographic accelerator
 * (SCA) used to provide advanced cryptographic security.
 */

/**
 * @ingroup ctb_registers
 * Structure type to access the CTB Registers.
 */
struct mxc_ctb_regs {
	__IO uint32_t crypto_ctrl; /**< <tt>\b 0x00:</tt> CTB CRYPTO_CTRL Register */
	__IO uint32_t cipher_ctrl; /**< <tt>\b 0x04:</tt> CTB CIPHER_CTRL Register */
	__IO uint32_t hash_ctrl;   /**< <tt>\b 0x08:</tt> CTB HASH_CTRL Register */
	__IO uint32_t crc_ctrl;	   /**< <tt>\b 0x0C:</tt> CTB CRC_CTRL Register */
	__IO uint32_t dma_src;	   /**< <tt>\b 0x10:</tt> CTB DMA_SRC Register */
	__IO uint32_t dma_dest;	   /**< <tt>\b 0x14:</tt> CTB DMA_DEST Register */
	__IO uint32_t dma_cnt;	   /**< <tt>\b 0x18:</tt> CTB DMA_CNT Register */
	__R uint32_t rsv_0x1c;
	__O uint32_t crypto_din[4];	 /**< <tt>\b 0x20:</tt> CTB CRYPTO_DIN Register */
	__I uint32_t crypto_dout[4]; /**< <tt>\b 0x30:</tt> CTB CRYPTO_DOUT Register */
	__IO uint32_t crc_poly;		 /**< <tt>\b 0x40:</tt> CTB CRC_POLY Register */
	__IO uint32_t crc_val;		 /**< <tt>\b 0x44:</tt> CTB CRC_VAL Register */
	__R uint32_t rsv_0x48;
	__IO uint32_t ham_ecc;		   /**< <tt>\b 0x4C:</tt> CTB HAM_ECC Register */
	__IO uint32_t cipher_init[4];  /**< <tt>\b 0x50:</tt> CTB CIPHER_INIT Register */
	__O uint32_t cipher_key[8];	   /**< <tt>\b 0x60:</tt> CTB CIPHER_KEY Register */
	__IO uint32_t hash_digest[16]; /**< <tt>\b 0x80:</tt> CTB HASH_DIGEST Register */
	__IO uint32_t hash_msg_sz[4];  /**< <tt>\b 0xC0:</tt> CTB HASH_MSG_SZ Register */
	__IO uint32_t aad_length_0;	   /**< <tt>\b 0xD0:</tt> CTB AAD_LENGTH_0 Register */
	__IO uint32_t aad_length_1;	   /**< <tt>\b 0xD4:</tt> CTB AAD_LENGTH_1 Register */
	__IO uint32_t pld_length_0;	   /**< <tt>\b 0xD8:</tt> CTB PLD_LENGTH_0 Register */
	__IO uint32_t pld_length_1;	   /**< <tt>\b 0xDC:</tt> CTB PLD_LENGTH_1 Register */
	__IO uint32_t tagmic[4];	   /**< <tt>\b 0xE0:</tt> CTB TAGMIC Register */
	__R uint32_t rsv_0xf0_0xff[4];
	__IO uint32_t sca_ctrl0;		/**< <tt>\b 0x100:</tt> CTB SCA_CTRL0 Register */
	__IO uint32_t sca_ctrl1;		/**< <tt>\b 0x104:</tt> CTB SCA_CTRL1 Register */
	__IO uint32_t sca_stat;			/**< <tt>\b 0x108:</tt> CTB SCA_STAT Register */
	__IO uint32_t sca_ppx_addr;		/**< <tt>\b 0x10C:</tt> CTB SCA_PPX_ADDR Register */
	__IO uint32_t sca_ppy_addr;		/**< <tt>\b 0x110:</tt> CTB SCA_PPY_ADDR Register */
	__IO uint32_t sca_ppz_addr;		/**< <tt>\b 0x114:</tt> CTB SCA_PPZ_ADDR Register */
	__IO uint32_t sca_pqx_addr;		/**< <tt>\b 0x118:</tt> CTB SCA_PQX_ADDR Register */
	__IO uint32_t sca_pqy_addr;		/**< <tt>\b 0x11C:</tt> CTB SCA_PQY_ADDR Register */
	__IO uint32_t sca_pqz_addr;		/**< <tt>\b 0x120:</tt> CTB SCA_PQZ_ADDR Register */
	__IO uint32_t sca_rdsa_addr;	/**< <tt>\b 0x124:</tt> CTB SCA_RDSA_ADDR Register */
	__IO uint32_t sca_res_addr;		/**< <tt>\b 0x128:</tt> CTB SCA_RES_ADDR Register */
	__IO uint32_t sca_op_buff_addr; /**< <tt>\b 0x12C:</tt> CTB SCA_OP_BUFF_ADDR Register */
	__IO uint32_t sca_moddata;		/**< <tt>\b 0x130:</tt> CTB SCA_MODDATA Register */
};

/* Register offsets for module CTB */
/**
 * @ingroup    ctb_registers
 * @defgroup   CTB_Register_Offsets Register Offsets
 * @brief      CTB Peripheral Register Offsets from the CTB Base Peripheral Address.
 * @{
 */
#define MXC_R_CTB_CRYPTO_CTRL ((uint32_t)0x00000000UL)		/**< Offset from CTB Base Address: 0x0000 */
#define MXC_R_CTB_CIPHER_CTRL ((uint32_t)0x00000004UL)		/**< Offset from CTB Base Address: 0x0004 */
#define MXC_R_CTB_HASH_CTRL ((uint32_t)0x00000008UL)		/**< Offset from CTB Base Address: 0x0008 */
#define MXC_R_CTB_CRC_CTRL ((uint32_t)0x0000000CUL)			/**< Offset from CTB Base Address: 0x000C */
#define MXC_R_CTB_DMA_SRC ((uint32_t)0x00000010UL)			/**< Offset from CTB Base Address: 0x0010 */
#define MXC_R_CTB_DMA_DEST ((uint32_t)0x00000014UL)			/**< Offset from CTB Base Address: 0x0014 */
#define MXC_R_CTB_DMA_CNT ((uint32_t)0x00000018UL)			/**< Offset from CTB Base Address: 0x0018 */
#define MXC_R_CTB_CRYPTO_DIN ((uint32_t)0x00000020UL)		/**< Offset from CTB Base Address: 0x0020 */
#define MXC_R_CTB_CRYPTO_DOUT ((uint32_t)0x00000030UL)		/**< Offset from CTB Base Address: 0x0030 */
#define MXC_R_CTB_CRC_POLY ((uint32_t)0x00000040UL)			/**< Offset from CTB Base Address: 0x0040 */
#define MXC_R_CTB_CRC_VAL ((uint32_t)0x00000044UL)			/**< Offset from CTB Base Address: 0x0044 */
#define MXC_R_CTB_HAM_ECC ((uint32_t)0x0000004CUL)			/**< Offset from CTB Base Address: 0x004C */
#define MXC_R_CTB_CIPHER_INIT ((uint32_t)0x00000050UL)		/**< Offset from CTB Base Address: 0x0050 */
#define MXC_R_CTB_CIPHER_KEY ((uint32_t)0x00000060UL)		/**< Offset from CTB Base Address: 0x0060 */
#define MXC_R_CTB_HASH_DIGEST ((uint32_t)0x00000080UL)		/**< Offset from CTB Base Address: 0x0080 */
#define MXC_R_CTB_HASH_MSG_SZ ((uint32_t)0x000000C0UL)		/**< Offset from CTB Base Address: 0x00C0 */
#define MXC_R_CTB_AAD_LENGTH_0 ((uint32_t)0x000000D0UL)		/**< Offset from CTB Base Address: 0x00D0 */
#define MXC_R_CTB_AAD_LENGTH_1 ((uint32_t)0x000000D4UL)		/**< Offset from CTB Base Address: 0x00D4 */
#define MXC_R_CTB_PLD_LENGTH_0 ((uint32_t)0x000000D8UL)		/**< Offset from CTB Base Address: 0x00D8 */
#define MXC_R_CTB_PLD_LENGTH_1 ((uint32_t)0x000000DCUL)		/**< Offset from CTB Base Address: 0x00DC */
#define MXC_R_CTB_TAGMIC ((uint32_t)0x000000E0UL)			/**< Offset from CTB Base Address: 0x00E0 */
#define MXC_R_CTB_SCA_CTRL0 ((uint32_t)0x00000100UL)		/**< Offset from CTB Base Address: 0x0100 */
#define MXC_R_CTB_SCA_CTRL1 ((uint32_t)0x00000104UL)		/**< Offset from CTB Base Address: 0x0104 */
#define MXC_R_CTB_SCA_STAT ((uint32_t)0x00000108UL)			/**< Offset from CTB Base Address: 0x0108 */
#define MXC_R_CTB_SCA_PPX_ADDR ((uint32_t)0x0000010CUL)		/**< Offset from CTB Base Address: 0x010C */
#define MXC_R_CTB_SCA_PPY_ADDR ((uint32_t)0x00000110UL)		/**< Offset from CTB Base Address: 0x0110 */
#define MXC_R_CTB_SCA_PPZ_ADDR ((uint32_t)0x00000114UL)		/**< Offset from CTB Base Address: 0x0114 */
#define MXC_R_CTB_SCA_PQX_ADDR ((uint32_t)0x00000118UL)		/**< Offset from CTB Base Address: 0x0118 */
#define MXC_R_CTB_SCA_PQY_ADDR ((uint32_t)0x0000011CUL)		/**< Offset from CTB Base Address: 0x011C */
#define MXC_R_CTB_SCA_PQZ_ADDR ((uint32_t)0x00000120UL)		/**< Offset from CTB Base Address: 0x0120 */
#define MXC_R_CTB_SCA_RDSA_ADDR ((uint32_t)0x00000124UL)	/**< Offset from CTB Base Address: 0x0124 */
#define MXC_R_CTB_SCA_RES_ADDR ((uint32_t)0x00000128UL)		/**< Offset from CTB Base Address: 0x0128 */
#define MXC_R_CTB_SCA_OP_BUFF_ADDR ((uint32_t)0x0000012CUL) /**< Offset from CTB Base Address: 0x012C */
#define MXC_R_CTB_SCA_MODDATA ((uint32_t)0x00000130UL)		/**< Offset from CTB Base Address: 0x0130 */
															/**@} end of group ctb_registers */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CRYPTO_CTRL CTB_CRYPTO_CTRL
 * @brief    Crypto Control Register.
 * @{
 */
#define MXC_F_CTB_CRYPTO_CTRL_RST_POS 0												   /**< CRYPTO_CTRL_RST Position */
#define MXC_F_CTB_CRYPTO_CTRL_RST ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_RST_POS)) /**< CRYPTO_CTRL_RST Mask */

#define MXC_F_CTB_CRYPTO_CTRL_INTR_POS 1 /**< CRYPTO_CTRL_INTR Position */
#define MXC_F_CTB_CRYPTO_CTRL_INTR ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_INTR_POS)) /**< CRYPTO_CTRL_INTR Mask */

#define MXC_F_CTB_CRYPTO_CTRL_SRC_POS 2												   /**< CRYPTO_CTRL_SRC Position */
#define MXC_F_CTB_CRYPTO_CTRL_SRC ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_SRC_POS)) /**< CRYPTO_CTRL_SRC Mask */

#define MXC_F_CTB_CRYPTO_CTRL_BSO_POS 4												   /**< CRYPTO_CTRL_BSO Position */
#define MXC_F_CTB_CRYPTO_CTRL_BSO ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_BSO_POS)) /**< CRYPTO_CTRL_BSO Mask */

#define MXC_F_CTB_CRYPTO_CTRL_BSI_POS 5												   /**< CRYPTO_CTRL_BSI Position */
#define MXC_F_CTB_CRYPTO_CTRL_BSI ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_BSI_POS)) /**< CRYPTO_CTRL_BSI Mask */

#define MXC_F_CTB_CRYPTO_CTRL_WAIT_EN_POS 6 /**< CRYPTO_CTRL_WAIT_EN Position */
#define MXC_F_CTB_CRYPTO_CTRL_WAIT_EN                                                                                  \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_WAIT_EN_POS)) /**< CRYPTO_CTRL_WAIT_EN Mask */

#define MXC_F_CTB_CRYPTO_CTRL_WAIT_POL_POS 7 /**< CRYPTO_CTRL_WAIT_POL Position */
#define MXC_F_CTB_CRYPTO_CTRL_WAIT_POL                                                                                 \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_WAIT_POL_POS)) /**< CRYPTO_CTRL_WAIT_POL Mask */

#define MXC_F_CTB_CRYPTO_CTRL_WRSRC_POS 8 /**< CRYPTO_CTRL_WRSRC Position */
#define MXC_F_CTB_CRYPTO_CTRL_WRSRC                                                                                    \
	((uint32_t)(0x3UL << MXC_F_CTB_CRYPTO_CTRL_WRSRC_POS)) /**< CRYPTO_CTRL_WRSRC Mask */
#define MXC_V_CTB_CRYPTO_CTRL_WRSRC_NONE ((uint32_t)0x0UL) /**< CRYPTO_CTRL_WRSRC_NONE Value */
#define MXC_S_CTB_CRYPTO_CTRL_WRSRC_NONE                                                                               \
	(MXC_V_CTB_CRYPTO_CTRL_WRSRC_NONE << MXC_F_CTB_CRYPTO_CTRL_WRSRC_POS) /**< CRYPTO_CTRL_WRSRC_NONE Setting */
#define MXC_V_CTB_CRYPTO_CTRL_WRSRC_CIPHEROUTPUT ((uint32_t)0x1UL)		  /**< CRYPTO_CTRL_WRSRC_CIPHEROUTPUT Value */
#define MXC_S_CTB_CRYPTO_CTRL_WRSRC_CIPHEROUTPUT                                                                       \
	(MXC_V_CTB_CRYPTO_CTRL_WRSRC_CIPHEROUTPUT                                                                          \
	 << MXC_F_CTB_CRYPTO_CTRL_WRSRC_POS)					   /**< CRYPTO_CTRL_WRSRC_CIPHEROUTPUT Setting */
#define MXC_V_CTB_CRYPTO_CTRL_WRSRC_READFIFO ((uint32_t)0x2UL) /**< CRYPTO_CTRL_WRSRC_READFIFO Value */
#define MXC_S_CTB_CRYPTO_CTRL_WRSRC_READFIFO                                                                           \
	(MXC_V_CTB_CRYPTO_CTRL_WRSRC_READFIFO << MXC_F_CTB_CRYPTO_CTRL_WRSRC_POS) /**< CRYPTO_CTRL_WRSRC_READFIFO Setting  \
																			   */

#define MXC_F_CTB_CRYPTO_CTRL_RDSRC_POS 10 /**< CRYPTO_CTRL_RDSRC Position */
#define MXC_F_CTB_CRYPTO_CTRL_RDSRC                                                                                    \
	((uint32_t)(0x3UL << MXC_F_CTB_CRYPTO_CTRL_RDSRC_POS))		  /**< CRYPTO_CTRL_RDSRC Mask */
#define MXC_V_CTB_CRYPTO_CTRL_RDSRC_DMADISABLED ((uint32_t)0x0UL) /**< CRYPTO_CTRL_RDSRC_DMADISABLED Value */
#define MXC_S_CTB_CRYPTO_CTRL_RDSRC_DMADISABLED                                                                        \
	(MXC_V_CTB_CRYPTO_CTRL_RDSRC_DMADISABLED                                                                           \
	 << MXC_F_CTB_CRYPTO_CTRL_RDSRC_POS)					   /**< CRYPTO_CTRL_RDSRC_DMADISABLED Setting */
#define MXC_V_CTB_CRYPTO_CTRL_RDSRC_DMAORAPB ((uint32_t)0x1UL) /**< CRYPTO_CTRL_RDSRC_DMAORAPB Value */
#define MXC_S_CTB_CRYPTO_CTRL_RDSRC_DMAORAPB                                                                           \
	(MXC_V_CTB_CRYPTO_CTRL_RDSRC_DMAORAPB << MXC_F_CTB_CRYPTO_CTRL_RDSRC_POS) /**< CRYPTO_CTRL_RDSRC_DMAORAPB Setting  \
																			   */
#define MXC_V_CTB_CRYPTO_CTRL_RDSRC_RNG ((uint32_t)0x2UL)					  /**< CRYPTO_CTRL_RDSRC_RNG Value */
#define MXC_S_CTB_CRYPTO_CTRL_RDSRC_RNG                                                                                \
	(MXC_V_CTB_CRYPTO_CTRL_RDSRC_RNG << MXC_F_CTB_CRYPTO_CTRL_RDSRC_POS) /**< CRYPTO_CTRL_RDSRC_RNG Setting */

#define MXC_F_CTB_CRYPTO_CTRL_FLAG_MODE_POS 14 /**< CRYPTO_CTRL_FLAG_MODE Position */
#define MXC_F_CTB_CRYPTO_CTRL_FLAG_MODE                                                                                \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_FLAG_MODE_POS)) /**< CRYPTO_CTRL_FLAG_MODE Mask */

#define MXC_F_CTB_CRYPTO_CTRL_DMADNEMSK_POS 15 /**< CRYPTO_CTRL_DMADNEMSK Position */
#define MXC_F_CTB_CRYPTO_CTRL_DMADNEMSK                                                                                \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_DMADNEMSK_POS)) /**< CRYPTO_CTRL_DMADNEMSK Mask */

#define MXC_F_CTB_CRYPTO_CTRL_DMA_DONE_POS 24 /**< CRYPTO_CTRL_DMA_DONE Position */
#define MXC_F_CTB_CRYPTO_CTRL_DMA_DONE                                                                                 \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_DMA_DONE_POS)) /**< CRYPTO_CTRL_DMA_DONE Mask */

#define MXC_F_CTB_CRYPTO_CTRL_GLS_DONE_POS 25 /**< CRYPTO_CTRL_GLS_DONE Position */
#define MXC_F_CTB_CRYPTO_CTRL_GLS_DONE                                                                                 \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_GLS_DONE_POS)) /**< CRYPTO_CTRL_GLS_DONE Mask */

#define MXC_F_CTB_CRYPTO_CTRL_HSH_DONE_POS 26 /**< CRYPTO_CTRL_HSH_DONE Position */
#define MXC_F_CTB_CRYPTO_CTRL_HSH_DONE                                                                                 \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_HSH_DONE_POS)) /**< CRYPTO_CTRL_HSH_DONE Mask */

#define MXC_F_CTB_CRYPTO_CTRL_CPH_DONE_POS 27 /**< CRYPTO_CTRL_CPH_DONE Position */
#define MXC_F_CTB_CRYPTO_CTRL_CPH_DONE                                                                                 \
	((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_CPH_DONE_POS)) /**< CRYPTO_CTRL_CPH_DONE Mask */

#define MXC_F_CTB_CRYPTO_CTRL_ERR_POS 29											   /**< CRYPTO_CTRL_ERR Position */
#define MXC_F_CTB_CRYPTO_CTRL_ERR ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_ERR_POS)) /**< CRYPTO_CTRL_ERR Mask */

#define MXC_F_CTB_CRYPTO_CTRL_RDY_POS 30											   /**< CRYPTO_CTRL_RDY Position */
#define MXC_F_CTB_CRYPTO_CTRL_RDY ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_RDY_POS)) /**< CRYPTO_CTRL_RDY Mask */

#define MXC_F_CTB_CRYPTO_CTRL_DONE_POS 31 /**< CRYPTO_CTRL_DONE Position */
#define MXC_F_CTB_CRYPTO_CTRL_DONE ((uint32_t)(0x1UL << MXC_F_CTB_CRYPTO_CTRL_DONE_POS)) /**< CRYPTO_CTRL_DONE Mask */

/**@} end of group CTB_CRYPTO_CTRL_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CIPHER_CTRL CTB_CIPHER_CTRL
 * @brief    Cipher Control Register.
 * @{
 */
#define MXC_F_CTB_CIPHER_CTRL_ENC_POS 0												   /**< CIPHER_CTRL_ENC Position */
#define MXC_F_CTB_CIPHER_CTRL_ENC ((uint32_t)(0x1UL << MXC_F_CTB_CIPHER_CTRL_ENC_POS)) /**< CIPHER_CTRL_ENC Mask */

#define MXC_F_CTB_CIPHER_CTRL_KEY_POS 1												   /**< CIPHER_CTRL_KEY Position */
#define MXC_F_CTB_CIPHER_CTRL_KEY ((uint32_t)(0x1UL << MXC_F_CTB_CIPHER_CTRL_KEY_POS)) /**< CIPHER_CTRL_KEY Mask */

#define MXC_F_CTB_CIPHER_CTRL_SRC_POS 2												   /**< CIPHER_CTRL_SRC Position */
#define MXC_F_CTB_CIPHER_CTRL_SRC ((uint32_t)(0x3UL << MXC_F_CTB_CIPHER_CTRL_SRC_POS)) /**< CIPHER_CTRL_SRC Mask */
#define MXC_V_CTB_CIPHER_CTRL_SRC_CIPHERKEY ((uint32_t)0x0UL) /**< CIPHER_CTRL_SRC_CIPHERKEY Value */
#define MXC_S_CTB_CIPHER_CTRL_SRC_CIPHERKEY                                                                            \
	(MXC_V_CTB_CIPHER_CTRL_SRC_CIPHERKEY << MXC_F_CTB_CIPHER_CTRL_SRC_POS) /**< CIPHER_CTRL_SRC_CIPHERKEY Setting */
#define MXC_V_CTB_CIPHER_CTRL_SRC_REGFILE ((uint32_t)0x2UL)				   /**< CIPHER_CTRL_SRC_REGFILE Value */
#define MXC_S_CTB_CIPHER_CTRL_SRC_REGFILE                                                                              \
	(MXC_V_CTB_CIPHER_CTRL_SRC_REGFILE << MXC_F_CTB_CIPHER_CTRL_SRC_POS) /**< CIPHER_CTRL_SRC_REGFILE Setting */
#define MXC_V_CTB_CIPHER_CTRL_SRC_QSPIKEY_REGFILE ((uint32_t)0x3UL)		 /**< CIPHER_CTRL_SRC_QSPIKEY_REGFILE Value */
#define MXC_S_CTB_CIPHER_CTRL_SRC_QSPIKEY_REGFILE                                                                      \
	(MXC_V_CTB_CIPHER_CTRL_SRC_QSPIKEY_REGFILE                                                                         \
	 << MXC_F_CTB_CIPHER_CTRL_SRC_POS) /**< CIPHER_CTRL_SRC_QSPIKEY_REGFILE Setting */

#define MXC_F_CTB_CIPHER_CTRL_CIPHER_POS 4 /**< CIPHER_CTRL_CIPHER Position */
#define MXC_F_CTB_CIPHER_CTRL_CIPHER                                                                                   \
	((uint32_t)(0x7UL << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS)) /**< CIPHER_CTRL_CIPHER Mask */
#define MXC_V_CTB_CIPHER_CTRL_CIPHER_DIS ((uint32_t)0x0UL)	/**< CIPHER_CTRL_CIPHER_DIS Value */
#define MXC_S_CTB_CIPHER_CTRL_CIPHER_DIS                                                                               \
	(MXC_V_CTB_CIPHER_CTRL_CIPHER_DIS << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_DIS Setting */
#define MXC_V_CTB_CIPHER_CTRL_CIPHER_AES128 ((uint32_t)0x1UL)			   /**< CIPHER_CTRL_CIPHER_AES128 Value */
#define MXC_S_CTB_CIPHER_CTRL_CIPHER_AES128                                                                            \
	(MXC_V_CTB_CIPHER_CTRL_CIPHER_AES128 << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_AES128 Setting   \
																			   */
#define MXC_V_CTB_CIPHER_CTRL_CIPHER_AES192 ((uint32_t)0x2UL)				  /**< CIPHER_CTRL_CIPHER_AES192 Value */
#define MXC_S_CTB_CIPHER_CTRL_CIPHER_AES192                                                                            \
	(MXC_V_CTB_CIPHER_CTRL_CIPHER_AES192 << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_AES192 Setting   \
																			   */
#define MXC_V_CTB_CIPHER_CTRL_CIPHER_AES256 ((uint32_t)0x3UL)				  /**< CIPHER_CTRL_CIPHER_AES256 Value */
#define MXC_S_CTB_CIPHER_CTRL_CIPHER_AES256                                                                            \
	(MXC_V_CTB_CIPHER_CTRL_CIPHER_AES256 << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_AES256 Setting   \
																			   */
#define MXC_V_CTB_CIPHER_CTRL_CIPHER_DES ((uint32_t)0x4UL)					  /**< CIPHER_CTRL_CIPHER_DES Value */
#define MXC_S_CTB_CIPHER_CTRL_CIPHER_DES                                                                               \
	(MXC_V_CTB_CIPHER_CTRL_CIPHER_DES << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_DES Setting */
#define MXC_V_CTB_CIPHER_CTRL_CIPHER_TDES ((uint32_t)0x5UL)				   /**< CIPHER_CTRL_CIPHER_TDES Value */
#define MXC_S_CTB_CIPHER_CTRL_CIPHER_TDES                                                                              \
	(MXC_V_CTB_CIPHER_CTRL_CIPHER_TDES << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_TDES Setting */

#define MXC_F_CTB_CIPHER_CTRL_MODE_POS 8 /**< CIPHER_CTRL_MODE Position */
#define MXC_F_CTB_CIPHER_CTRL_MODE ((uint32_t)(0x7UL << MXC_F_CTB_CIPHER_CTRL_MODE_POS)) /**< CIPHER_CTRL_MODE Mask */
#define MXC_V_CTB_CIPHER_CTRL_MODE_ECB ((uint32_t)0x0UL) /**< CIPHER_CTRL_MODE_ECB Value */
#define MXC_S_CTB_CIPHER_CTRL_MODE_ECB                                                                                 \
	(MXC_V_CTB_CIPHER_CTRL_MODE_ECB << MXC_F_CTB_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_ECB Setting */
#define MXC_V_CTB_CIPHER_CTRL_MODE_CBC ((uint32_t)0x1UL)			   /**< CIPHER_CTRL_MODE_CBC Value */
#define MXC_S_CTB_CIPHER_CTRL_MODE_CBC                                                                                 \
	(MXC_V_CTB_CIPHER_CTRL_MODE_CBC << MXC_F_CTB_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_CBC Setting */
#define MXC_V_CTB_CIPHER_CTRL_MODE_CFB ((uint32_t)0x2UL)			   /**< CIPHER_CTRL_MODE_CFB Value */
#define MXC_S_CTB_CIPHER_CTRL_MODE_CFB                                                                                 \
	(MXC_V_CTB_CIPHER_CTRL_MODE_CFB << MXC_F_CTB_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_CFB Setting */
#define MXC_V_CTB_CIPHER_CTRL_MODE_OFB ((uint32_t)0x3UL)			   /**< CIPHER_CTRL_MODE_OFB Value */
#define MXC_S_CTB_CIPHER_CTRL_MODE_OFB                                                                                 \
	(MXC_V_CTB_CIPHER_CTRL_MODE_OFB << MXC_F_CTB_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_OFB Setting */
#define MXC_V_CTB_CIPHER_CTRL_MODE_CTR ((uint32_t)0x4UL)			   /**< CIPHER_CTRL_MODE_CTR Value */
#define MXC_S_CTB_CIPHER_CTRL_MODE_CTR                                                                                 \
	(MXC_V_CTB_CIPHER_CTRL_MODE_CTR << MXC_F_CTB_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_CTR Setting */
#define MXC_V_CTB_CIPHER_CTRL_MODE_GCM ((uint32_t)0x5UL)			   /**< CIPHER_CTRL_MODE_GCM Value */
#define MXC_S_CTB_CIPHER_CTRL_MODE_GCM                                                                                 \
	(MXC_V_CTB_CIPHER_CTRL_MODE_GCM << MXC_F_CTB_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_GCM Setting */
#define MXC_V_CTB_CIPHER_CTRL_MODE_CCM ((uint32_t)0x6UL)			   /**< CIPHER_CTRL_MODE_CCM Value */
#define MXC_S_CTB_CIPHER_CTRL_MODE_CCM                                                                                 \
	(MXC_V_CTB_CIPHER_CTRL_MODE_CCM << MXC_F_CTB_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_CCM Setting */

#define MXC_F_CTB_CIPHER_CTRL_HVC_POS 11											   /**< CIPHER_CTRL_HVC Position */
#define MXC_F_CTB_CIPHER_CTRL_HVC ((uint32_t)(0x1UL << MXC_F_CTB_CIPHER_CTRL_HVC_POS)) /**< CIPHER_CTRL_HVC Mask */

#define MXC_F_CTB_CIPHER_CTRL_DTYPE_POS 12 /**< CIPHER_CTRL_DTYPE Position */
#define MXC_F_CTB_CIPHER_CTRL_DTYPE                                                                                    \
	((uint32_t)(0x1UL << MXC_F_CTB_CIPHER_CTRL_DTYPE_POS)) /**< CIPHER_CTRL_DTYPE Mask */

#define MXC_F_CTB_CIPHER_CTRL_CCMM_POS 13 /**< CIPHER_CTRL_CCMM Position */
#define MXC_F_CTB_CIPHER_CTRL_CCMM ((uint32_t)(0x7UL << MXC_F_CTB_CIPHER_CTRL_CCMM_POS)) /**< CIPHER_CTRL_CCMM Mask */

#define MXC_F_CTB_CIPHER_CTRL_CCML_POS 16 /**< CIPHER_CTRL_CCML Position */
#define MXC_F_CTB_CIPHER_CTRL_CCML ((uint32_t)(0x7UL << MXC_F_CTB_CIPHER_CTRL_CCML_POS)) /**< CIPHER_CTRL_CCML Mask */

/**@} end of group CTB_CIPHER_CTRL_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_HASH_CTRL CTB_HASH_CTRL
 * @brief    HASH Control Register.
 * @{
 */
#define MXC_F_CTB_HASH_CTRL_INIT_POS 0												 /**< HASH_CTRL_INIT Position */
#define MXC_F_CTB_HASH_CTRL_INIT ((uint32_t)(0x1UL << MXC_F_CTB_HASH_CTRL_INIT_POS)) /**< HASH_CTRL_INIT Mask */

#define MXC_F_CTB_HASH_CTRL_XOR_POS 1											   /**< HASH_CTRL_XOR Position */
#define MXC_F_CTB_HASH_CTRL_XOR ((uint32_t)(0x1UL << MXC_F_CTB_HASH_CTRL_XOR_POS)) /**< HASH_CTRL_XOR Mask */

#define MXC_F_CTB_HASH_CTRL_HASH_POS 2												 /**< HASH_CTRL_HASH Position */
#define MXC_F_CTB_HASH_CTRL_HASH ((uint32_t)(0x7UL << MXC_F_CTB_HASH_CTRL_HASH_POS)) /**< HASH_CTRL_HASH Mask */
#define MXC_V_CTB_HASH_CTRL_HASH_DIS ((uint32_t)0x0UL)								 /**< HASH_CTRL_HASH_DIS Value */
#define MXC_S_CTB_HASH_CTRL_HASH_DIS                                                                                   \
	(MXC_V_CTB_HASH_CTRL_HASH_DIS << MXC_F_CTB_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_DIS Setting */
#define MXC_V_CTB_HASH_CTRL_HASH_SHA1 ((uint32_t)0x1UL)			   /**< HASH_CTRL_HASH_SHA1 Value */
#define MXC_S_CTB_HASH_CTRL_HASH_SHA1                                                                                  \
	(MXC_V_CTB_HASH_CTRL_HASH_SHA1 << MXC_F_CTB_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA1 Setting */
#define MXC_V_CTB_HASH_CTRL_HASH_SHA224 ((uint32_t)0x2UL)			/**< HASH_CTRL_HASH_SHA224 Value */
#define MXC_S_CTB_HASH_CTRL_HASH_SHA224                                                                                \
	(MXC_V_CTB_HASH_CTRL_HASH_SHA224 << MXC_F_CTB_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA224 Setting */
#define MXC_V_CTB_HASH_CTRL_HASH_SHA256 ((uint32_t)0x3UL)			  /**< HASH_CTRL_HASH_SHA256 Value */
#define MXC_S_CTB_HASH_CTRL_HASH_SHA256                                                                                \
	(MXC_V_CTB_HASH_CTRL_HASH_SHA256 << MXC_F_CTB_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA256 Setting */
#define MXC_V_CTB_HASH_CTRL_HASH_SHA384 ((uint32_t)0x4UL)			  /**< HASH_CTRL_HASH_SHA384 Value */
#define MXC_S_CTB_HASH_CTRL_HASH_SHA384                                                                                \
	(MXC_V_CTB_HASH_CTRL_HASH_SHA384 << MXC_F_CTB_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA384 Setting */
#define MXC_V_CTB_HASH_CTRL_HASH_SHA512 ((uint32_t)0x5UL)			  /**< HASH_CTRL_HASH_SHA512 Value */
#define MXC_S_CTB_HASH_CTRL_HASH_SHA512                                                                                \
	(MXC_V_CTB_HASH_CTRL_HASH_SHA512 << MXC_F_CTB_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA512 Setting */

#define MXC_F_CTB_HASH_CTRL_LAST_POS 5												 /**< HASH_CTRL_LAST Position */
#define MXC_F_CTB_HASH_CTRL_LAST ((uint32_t)(0x1UL << MXC_F_CTB_HASH_CTRL_LAST_POS)) /**< HASH_CTRL_LAST Mask */

/**@} end of group CTB_HASH_CTRL_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CRC_CTRL CTB_CRC_CTRL
 * @brief    CRC Control Register.
 * @{
 */
#define MXC_F_CTB_CRC_CTRL_CRC_POS 0											 /**< CRC_CTRL_CRC Position */
#define MXC_F_CTB_CRC_CTRL_CRC ((uint32_t)(0x1UL << MXC_F_CTB_CRC_CTRL_CRC_POS)) /**< CRC_CTRL_CRC Mask */

#define MXC_F_CTB_CRC_CTRL_MSB_POS 1											 /**< CRC_CTRL_MSB Position */
#define MXC_F_CTB_CRC_CTRL_MSB ((uint32_t)(0x1UL << MXC_F_CTB_CRC_CTRL_MSB_POS)) /**< CRC_CTRL_MSB Mask */

#define MXC_F_CTB_CRC_CTRL_PRNG_POS 2											   /**< CRC_CTRL_PRNG Position */
#define MXC_F_CTB_CRC_CTRL_PRNG ((uint32_t)(0x1UL << MXC_F_CTB_CRC_CTRL_PRNG_POS)) /**< CRC_CTRL_PRNG Mask */

#define MXC_F_CTB_CRC_CTRL_ENT_POS 3											 /**< CRC_CTRL_ENT Position */
#define MXC_F_CTB_CRC_CTRL_ENT ((uint32_t)(0x1UL << MXC_F_CTB_CRC_CTRL_ENT_POS)) /**< CRC_CTRL_ENT Mask */

#define MXC_F_CTB_CRC_CTRL_HAM_POS 4											 /**< CRC_CTRL_HAM Position */
#define MXC_F_CTB_CRC_CTRL_HAM ((uint32_t)(0x1UL << MXC_F_CTB_CRC_CTRL_HAM_POS)) /**< CRC_CTRL_HAM Mask */

#define MXC_F_CTB_CRC_CTRL_HRST_POS 5											   /**< CRC_CTRL_HRST Position */
#define MXC_F_CTB_CRC_CTRL_HRST ((uint32_t)(0x1UL << MXC_F_CTB_CRC_CTRL_HRST_POS)) /**< CRC_CTRL_HRST Mask */

/**@} end of group CTB_CRC_CTRL_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_DMA_SRC CTB_DMA_SRC
 * @brief    Crypto DMA Source Address.
 * @{
 */
#define MXC_F_CTB_DMA_SRC_ADDR_POS 0													/**< DMA_SRC_ADDR Position */
#define MXC_F_CTB_DMA_SRC_ADDR ((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_DMA_SRC_ADDR_POS)) /**< DMA_SRC_ADDR Mask */

/**@} end of group CTB_DMA_SRC_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_DMA_DEST CTB_DMA_DEST
 * @brief    Crypto DMA Destination Address.
 * @{
 */
#define MXC_F_CTB_DMA_DEST_ADDR_POS 0													  /**< DMA_DEST_ADDR Position */
#define MXC_F_CTB_DMA_DEST_ADDR ((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_DMA_DEST_ADDR_POS)) /**< DMA_DEST_ADDR Mask */

/**@} end of group CTB_DMA_DEST_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_DMA_CNT CTB_DMA_CNT
 * @brief    Crypto DMA Byte Count.
 * @{
 */
#define MXC_F_CTB_DMA_CNT_COUNT_POS 0													  /**< DMA_CNT_COUNT Position */
#define MXC_F_CTB_DMA_CNT_COUNT ((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_DMA_CNT_COUNT_POS)) /**< DMA_CNT_COUNT Mask */

/**@} end of group CTB_DMA_CNT_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CRYPTO_DIN CTB_CRYPTO_DIN
 * @brief    Crypto Data Input. Data input can be written to this register instead of using
 *           the DMA. This register writes to the FIFO. This register occupies four
 *           successive words to allow the use of multi-store instructions. Words can be
 *           written to any location, they will be placed in the FIFO in the order they are
 *           written. The endian swap input control bit affects this register.
 * @{
 */
#define MXC_F_CTB_CRYPTO_DIN_DATA_POS 0 /**< CRYPTO_DIN_DATA Position */
#define MXC_F_CTB_CRYPTO_DIN_DATA                                                                                      \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_CRYPTO_DIN_DATA_POS)) /**< CRYPTO_DIN_DATA Mask */

/**@} end of group CTB_CRYPTO_DIN_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CRYPTO_DOUT CTB_CRYPTO_DOUT
 * @brief    Crypto Data Output. Resulting data from cipher calculation. Data is placed in
 *           the lower words of these four registers depending on the algorithm. For block
 *           cipher modes, this register holds the result of most recent encryption or
 *           decryption operation. These registers are affected by the endian swap bits.
 * @{
 */
#define MXC_F_CTB_CRYPTO_DOUT_DATA_POS 0 /**< CRYPTO_DOUT_DATA Position */
#define MXC_F_CTB_CRYPTO_DOUT_DATA                                                                                     \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_CRYPTO_DOUT_DATA_POS)) /**< CRYPTO_DOUT_DATA Mask */

/**@} end of group CTB_CRYPTO_DOUT_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CRC_POLY CTB_CRC_POLY
 * @brief    CRC Polynomial. The polynomial to be used for Galois Field calculations (CRC or
 *           LFSR) should be written to this register. This register is affected by the MSB
 *           control bit.
 * @{
 */
#define MXC_F_CTB_CRC_POLY_DATA_POS 0													  /**< CRC_POLY_DATA Position */
#define MXC_F_CTB_CRC_POLY_DATA ((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_CRC_POLY_DATA_POS)) /**< CRC_POLY_DATA Mask */

/**@} end of group CTB_CRC_POLY_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CRC_VAL CTB_CRC_VAL
 * @brief    CRC Value. This is the state for the Galois Field. This register holds the
 *           result of a CRC calculation or the current state of the LFSR. This register is
 *           affected by the MSB control bit.
 * @{
 */
#define MXC_F_CTB_CRC_VAL_VAL_POS 0													  /**< CRC_VAL_VAL Position */
#define MXC_F_CTB_CRC_VAL_VAL ((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_CRC_VAL_VAL_POS)) /**< CRC_VAL_VAL Mask */

/**@} end of group CTB_CRC_VAL_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_HAM_ECC CTB_HAM_ECC
 * @brief    Hamming ECC Register.
 * @{
 */
#define MXC_F_CTB_HAM_ECC_ECC_POS 0												  /**< HAM_ECC_ECC Position */
#define MXC_F_CTB_HAM_ECC_ECC ((uint32_t)(0xFFFFUL << MXC_F_CTB_HAM_ECC_ECC_POS)) /**< HAM_ECC_ECC Mask */

#define MXC_F_CTB_HAM_ECC_PAR_POS 16										   /**< HAM_ECC_PAR Position */
#define MXC_F_CTB_HAM_ECC_PAR ((uint32_t)(0x1UL << MXC_F_CTB_HAM_ECC_PAR_POS)) /**< HAM_ECC_PAR Mask */

/**@} end of group CTB_HAM_ECC_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CIPHER_INIT CTB_CIPHER_INIT
 * @brief    Initial Vector. For block cipher operations that use CBC, CFB, OFB, or CNTR
 *           modes, this register holds the initial value. This register is updated with each
 *           encryption or decryption operation. This register is affected by the endian swap
 *           bits.
 * @{
 */
#define MXC_F_CTB_CIPHER_INIT_IVEC_POS 0 /**< CIPHER_INIT_IVEC Position */
#define MXC_F_CTB_CIPHER_INIT_IVEC                                                                                     \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_CIPHER_INIT_IVEC_POS)) /**< CIPHER_INIT_IVEC Mask */

/**@} end of group CTB_CIPHER_INIT_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_CIPHER_KEY CTB_CIPHER_KEY
 * @brief    Cipher Key.  This register holds the key used for block cipher operations. The
 *           lower words are used for block ciphers that use shorter key lengths. This
 *           register is affected by the endian swap input control bits.
 * @{
 */
#define MXC_F_CTB_CIPHER_KEY_KEY_POS 0 /**< CIPHER_KEY_KEY Position */
#define MXC_F_CTB_CIPHER_KEY_KEY ((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_CIPHER_KEY_KEY_POS)) /**< CIPHER_KEY_KEY Mask   \
																							 */

/**@} end of group CTB_CIPHER_KEY_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_HASH_DIGEST CTB_HASH_DIGEST
 * @brief    This register holds the calculated hash value. This register is affected by the
 *           endian swap bits.
 * @{
 */
#define MXC_F_CTB_HASH_DIGEST_HASH_POS 0 /**< HASH_DIGEST_HASH Position */
#define MXC_F_CTB_HASH_DIGEST_HASH                                                                                     \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_HASH_DIGEST_HASH_POS)) /**< HASH_DIGEST_HASH Mask */

/**@} end of group CTB_HASH_DIGEST_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_HASH_MSG_SZ CTB_HASH_MSG_SZ
 * @brief    Message Size. This register holds the lowest 32-bit of message size in bytes.
 * @{
 */
#define MXC_F_CTB_HASH_MSG_SZ_MSGSZ_POS 0 /**< HASH_MSG_SZ_MSGSZ Position */
#define MXC_F_CTB_HASH_MSG_SZ_MSGSZ                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_HASH_MSG_SZ_MSGSZ_POS)) /**< HASH_MSG_SZ_MSGSZ Mask */

/**@} end of group CTB_HASH_MSG_SZ_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_AAD_LENGTH_0 CTB_AAD_LENGTH_0
 * @brief    .AAD Length Register 0.
 * @{
 */
#define MXC_F_CTB_AAD_LENGTH_0_LENGTH_POS 0 /**< AAD_LENGTH_0_LENGTH Position */
#define MXC_F_CTB_AAD_LENGTH_0_LENGTH                                                                                  \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_AAD_LENGTH_0_LENGTH_POS)) /**< AAD_LENGTH_0_LENGTH Mask */

/**@} end of group CTB_AAD_LENGTH_0_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_AAD_LENGTH_1 CTB_AAD_LENGTH_1
 * @brief    .AAD Length Register 1.
 * @{
 */
#define MXC_F_CTB_AAD_LENGTH_1_LENGTH_POS 0 /**< AAD_LENGTH_1_LENGTH Position */
#define MXC_F_CTB_AAD_LENGTH_1_LENGTH                                                                                  \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_AAD_LENGTH_1_LENGTH_POS)) /**< AAD_LENGTH_1_LENGTH Mask */

/**@} end of group CTB_AAD_LENGTH_1_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_PLD_LENGTH_0 CTB_PLD_LENGTH_0
 * @brief    .PLD Length Register 0.
 * @{
 */
#define MXC_F_CTB_PLD_LENGTH_0_LENGTH_POS 0 /**< PLD_LENGTH_0_LENGTH Position */
#define MXC_F_CTB_PLD_LENGTH_0_LENGTH                                                                                  \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_PLD_LENGTH_0_LENGTH_POS)) /**< PLD_LENGTH_0_LENGTH Mask */

/**@} end of group CTB_PLD_LENGTH_0_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_PLD_LENGTH_1 CTB_PLD_LENGTH_1
 * @brief    .LENGTH.
 * @{
 */
#define MXC_F_CTB_PLD_LENGTH_1_LENGTH_POS 0 /**< PLD_LENGTH_1_LENGTH Position */
#define MXC_F_CTB_PLD_LENGTH_1_LENGTH                                                                                  \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_PLD_LENGTH_1_LENGTH_POS)) /**< PLD_LENGTH_1_LENGTH Mask */

/**@} end of group CTB_PLD_LENGTH_1_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_TAGMIC CTB_TAGMIC
 * @brief    TAG/MIC Registers.
 * @{
 */
#define MXC_F_CTB_TAGMIC_LENGTH_POS 0													  /**< TAGMIC_LENGTH Position */
#define MXC_F_CTB_TAGMIC_LENGTH ((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_TAGMIC_LENGTH_POS)) /**< TAGMIC_LENGTH Mask */

/**@} end of group CTB_TAGMIC_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_CTRL0 CTB_SCA_CTRL0
 * @brief    SCA Control 0 Register.
 * @{
 */
#define MXC_F_CTB_SCA_CTRL0_STC_POS 0											   /**< SCA_CTRL0_STC Position */
#define MXC_F_CTB_SCA_CTRL0_STC ((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL0_STC_POS)) /**< SCA_CTRL0_STC Mask */

#define MXC_F_CTB_SCA_CTRL0_SCAIE_POS 1												   /**< SCA_CTRL0_SCAIE Position */
#define MXC_F_CTB_SCA_CTRL0_SCAIE ((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL0_SCAIE_POS)) /**< SCA_CTRL0_SCAIE Mask */

#define MXC_F_CTB_SCA_CTRL0_ABORT_POS 2												   /**< SCA_CTRL0_ABORT Position */
#define MXC_F_CTB_SCA_CTRL0_ABORT ((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL0_ABORT_POS)) /**< SCA_CTRL0_ABORT Mask */

#define MXC_F_CTB_SCA_CTRL0_ERMEM_POS 4												   /**< SCA_CTRL0_ERMEM Position */
#define MXC_F_CTB_SCA_CTRL0_ERMEM ((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL0_ERMEM_POS)) /**< SCA_CTRL0_ERMEM Mask */

#define MXC_F_CTB_SCA_CTRL0_MANPARAM_POS 5 /**< SCA_CTRL0_MANPARAM Position */
#define MXC_F_CTB_SCA_CTRL0_MANPARAM                                                                                   \
	((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL0_MANPARAM_POS)) /**< SCA_CTRL0_MANPARAM Mask */

#define MXC_F_CTB_SCA_CTRL0_HWKEY_POS 6												   /**< SCA_CTRL0_HWKEY Position */
#define MXC_F_CTB_SCA_CTRL0_HWKEY ((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL0_HWKEY_POS)) /**< SCA_CTRL0_HWKEY Mask */

#define MXC_F_CTB_SCA_CTRL0_OPCODE_POS 8 /**< SCA_CTRL0_OPCODE Position */
#define MXC_F_CTB_SCA_CTRL0_OPCODE ((uint32_t)(0x1FUL << MXC_F_CTB_SCA_CTRL0_OPCODE_POS)) /**< SCA_CTRL0_OPCODE Mask   \
																						   */

#define MXC_F_CTB_SCA_CTRL0_MODADDR_POS 16 /**< SCA_CTRL0_MODADDR Position */
#define MXC_F_CTB_SCA_CTRL0_MODADDR                                                                                    \
	((uint32_t)(0x1FUL << MXC_F_CTB_SCA_CTRL0_MODADDR_POS)) /**< SCA_CTRL0_MODADDR Mask */

#define MXC_F_CTB_SCA_CTRL0_ECCSIZE_POS 24 /**< SCA_CTRL0_ECCSIZE Position */
#define MXC_F_CTB_SCA_CTRL0_ECCSIZE                                                                                    \
	((uint32_t)(0x3UL << MXC_F_CTB_SCA_CTRL0_ECCSIZE_POS)) /**< SCA_CTRL0_ECCSIZE Mask */

/**@} end of group CTB_SCA_CTRL0_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_CTRL1 CTB_SCA_CTRL1
 * @brief    SCA Advanced Control Register.
 * @{
 */
#define MXC_F_CTB_SCA_CTRL1_MAN_POS 0											   /**< SCA_CTRL1_MAN Position */
#define MXC_F_CTB_SCA_CTRL1_MAN ((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL1_MAN_POS)) /**< SCA_CTRL1_MAN Mask */

#define MXC_F_CTB_SCA_CTRL1_AUTOCARRY_POS 1 /**< SCA_CTRL1_AUTOCARRY Position */
#define MXC_F_CTB_SCA_CTRL1_AUTOCARRY                                                                                  \
	((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL1_AUTOCARRY_POS)) /**< SCA_CTRL1_AUTOCARRY Mask */

#define MXC_F_CTB_SCA_CTRL1_PLUSONE_POS 2 /**< SCA_CTRL1_PLUSONE Position */
#define MXC_F_CTB_SCA_CTRL1_PLUSONE                                                                                    \
	((uint32_t)(0x1UL << MXC_F_CTB_SCA_CTRL1_PLUSONE_POS)) /**< SCA_CTRL1_PLUSONE Mask */

#define MXC_F_CTB_SCA_CTRL1_RESSELECT_POS 3 /**< SCA_CTRL1_RESSELECT Position */
#define MXC_F_CTB_SCA_CTRL1_RESSELECT                                                                                  \
	((uint32_t)(0x3UL << MXC_F_CTB_SCA_CTRL1_RESSELECT_POS)) /**< SCA_CTRL1_RESSELECT Mask */

#define MXC_F_CTB_SCA_CTRL1_CARRYPOS_POS 8 /**< SCA_CTRL1_CARRYPOS Position */
#define MXC_F_CTB_SCA_CTRL1_CARRYPOS                                                                                   \
	((uint32_t)(0x3FFUL << MXC_F_CTB_SCA_CTRL1_CARRYPOS_POS)) /**< SCA_CTRL1_CARRYPOS Mask */

/**@} end of group CTB_SCA_CTRL1_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_STAT CTB_SCA_STAT
 * @brief    SCA Status Register.
 * @{
 */
#define MXC_F_CTB_SCA_STAT_BUSY_POS 0											   /**< SCA_STAT_BUSY Position */
#define MXC_F_CTB_SCA_STAT_BUSY ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_BUSY_POS)) /**< SCA_STAT_BUSY Mask */

#define MXC_F_CTB_SCA_STAT_SCAIF_POS 1												 /**< SCA_STAT_SCAIF Position */
#define MXC_F_CTB_SCA_STAT_SCAIF ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_SCAIF_POS)) /**< SCA_STAT_SCAIF Mask */

#define MXC_F_CTB_SCA_STAT_PVF1_POS 2											   /**< SCA_STAT_PVF1 Position */
#define MXC_F_CTB_SCA_STAT_PVF1 ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_PVF1_POS)) /**< SCA_STAT_PVF1 Mask */

#define MXC_F_CTB_SCA_STAT_PVF2_POS 3											   /**< SCA_STAT_PVF2 Position */
#define MXC_F_CTB_SCA_STAT_PVF2 ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_PVF2_POS)) /**< SCA_STAT_PVF2 Mask */

#define MXC_F_CTB_SCA_STAT_FSMERR_POS 4												   /**< SCA_STAT_FSMERR Position */
#define MXC_F_CTB_SCA_STAT_FSMERR ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_FSMERR_POS)) /**< SCA_STAT_FSMERR Mask */

#define MXC_F_CTB_SCA_STAT_COMPERR_POS 5 /**< SCA_STAT_COMPERR Position */
#define MXC_F_CTB_SCA_STAT_COMPERR ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_COMPERR_POS)) /**< SCA_STAT_COMPERR Mask */

#define MXC_F_CTB_SCA_STAT_MEMERR_POS 6												   /**< SCA_STAT_MEMERR Position */
#define MXC_F_CTB_SCA_STAT_MEMERR ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_MEMERR_POS)) /**< SCA_STAT_MEMERR Mask */

#define MXC_F_CTB_SCA_STAT_CARRY_POS 8												 /**< SCA_STAT_CARRY Position */
#define MXC_F_CTB_SCA_STAT_CARRY ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_CARRY_POS)) /**< SCA_STAT_CARRY Mask */

#define MXC_F_CTB_SCA_STAT_GTE2I2_POS 9												   /**< SCA_STAT_GTE2I2 Position */
#define MXC_F_CTB_SCA_STAT_GTE2I2 ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_GTE2I2_POS)) /**< SCA_STAT_GTE2I2 Mask */

#define MXC_F_CTB_SCA_STAT_ALUNEG1_POS 10 /**< SCA_STAT_ALUNEG1 Position */
#define MXC_F_CTB_SCA_STAT_ALUNEG1 ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_ALUNEG1_POS)) /**< SCA_STAT_ALUNEG1 Mask */

#define MXC_F_CTB_SCA_STAT_ALUNEG2_POS 11 /**< SCA_STAT_ALUNEG2 Position */
#define MXC_F_CTB_SCA_STAT_ALUNEG2 ((uint32_t)(0x1UL << MXC_F_CTB_SCA_STAT_ALUNEG2_POS)) /**< SCA_STAT_ALUNEG2 Mask */

/**@} end of group CTB_SCA_STAT_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_PPX_ADDR CTB_SCA_PPX_ADDR
 * @brief    PPX Coordinate Data Pointer Register.
 * @{
 */
#define MXC_F_CTB_SCA_PPX_ADDR_ADDR_POS 0 /**< SCA_PPX_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_PPX_ADDR_ADDR                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_PPX_ADDR_ADDR_POS)) /**< SCA_PPX_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_PPX_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_PPY_ADDR CTB_SCA_PPY_ADDR
 * @brief    PPY Coordinate Data Pointer Register.
 * @{
 */
#define MXC_F_CTB_SCA_PPY_ADDR_ADDR_POS 0 /**< SCA_PPY_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_PPY_ADDR_ADDR                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_PPY_ADDR_ADDR_POS)) /**< SCA_PPY_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_PPY_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_PPZ_ADDR CTB_SCA_PPZ_ADDR
 * @brief    PPZ Coordinate Data Pointer Register.
 * @{
 */
#define MXC_F_CTB_SCA_PPZ_ADDR_ADDR_POS 0 /**< SCA_PPZ_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_PPZ_ADDR_ADDR                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_PPZ_ADDR_ADDR_POS)) /**< SCA_PPZ_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_PPZ_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_PQX_ADDR CTB_SCA_PQX_ADDR
 * @brief    PQX Coordinate Data Pointer Register.
 * @{
 */
#define MXC_F_CTB_SCA_PQX_ADDR_ADDR_POS 0 /**< SCA_PQX_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_PQX_ADDR_ADDR                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_PQX_ADDR_ADDR_POS)) /**< SCA_PQX_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_PQX_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_PQY_ADDR CTB_SCA_PQY_ADDR
 * @brief    PQY Coordinate Data Pointer Register.
 * @{
 */
#define MXC_F_CTB_SCA_PQY_ADDR_ADDR_POS 0 /**< SCA_PQY_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_PQY_ADDR_ADDR                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_PQY_ADDR_ADDR_POS)) /**< SCA_PQY_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_PQY_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_PQZ_ADDR CTB_SCA_PQZ_ADDR
 * @brief    PQZ Coordinate Data Pointer Register.
 * @{
 */
#define MXC_F_CTB_SCA_PQZ_ADDR_ADDR_POS 0 /**< SCA_PQZ_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_PQZ_ADDR_ADDR                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_PQZ_ADDR_ADDR_POS)) /**< SCA_PQZ_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_PQZ_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_RDSA_ADDR CTB_SCA_RDSA_ADDR
 * @brief    SCA RDSA Address Register.
 * @{
 */
#define MXC_F_CTB_SCA_RDSA_ADDR_ADDR_POS 0 /**< SCA_RDSA_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_RDSA_ADDR_ADDR                                                                                   \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_RDSA_ADDR_ADDR_POS)) /**< SCA_RDSA_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_RDSA_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_RES_ADDR CTB_SCA_RES_ADDR
 * @brief    SCA Result Address Register.
 * @{
 */
#define MXC_F_CTB_SCA_RES_ADDR_ADDR_POS 0 /**< SCA_RES_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_RES_ADDR_ADDR                                                                                    \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_RES_ADDR_ADDR_POS)) /**< SCA_RES_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_RES_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_OP_BUFF_ADDR CTB_SCA_OP_BUFF_ADDR
 * @brief    SCA Operation Buffer Address Register.
 * @{
 */
#define MXC_F_CTB_SCA_OP_BUFF_ADDR_ADDR_POS 0 /**< SCA_OP_BUFF_ADDR_ADDR Position */
#define MXC_F_CTB_SCA_OP_BUFF_ADDR_ADDR                                                                                \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_OP_BUFF_ADDR_ADDR_POS)) /**< SCA_OP_BUFF_ADDR_ADDR Mask */

/**@} end of group CTB_SCA_OP_BUFF_ADDR_Register */

/**
 * @ingroup  ctb_registers
 * @defgroup CTB_SCA_MODDATA CTB_SCA_MODDATA
 * @brief    SCA Modulo Data Input Register.
 * @{
 */
#define MXC_F_CTB_SCA_MODDATA_DATA_POS 0 /**< SCA_MODDATA_DATA Position */
#define MXC_F_CTB_SCA_MODDATA_DATA                                                                                     \
	((uint32_t)(0xFFFFFFFFUL << MXC_F_CTB_SCA_MODDATA_DATA_POS)) /**< SCA_MODDATA_DATA Mask */

/**@} end of group CTB_SCA_MODDATA_Register */

#ifdef __cplusplus
}
#endif

#endif /* _CTB_REGS_H_ */

/*
 * Copyright (C) 2005-2006 by egnite Software GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */


#ifndef samregsH
#define samregsH


/*
 * Register definitions below copied from NutOS
 */

#define DBGU_BASE       0xFFFFF200      /*!< \brief DBGU base address. */

#define DBGU_CIDR_OFF           0x00000040      /*!< \brief DBGU chip ID register offset. */
#define DBGU_CIDR   (DBGU_BASE + DBGU_CIDR_OFF) /*!< \brief DBGU chip ID register. */


#define MC_BASE         0xFFFFFF00      /*!< \brief Memory controller base. */

#define MC_FMR_OFF              0x00000060      /*!< \brief MC flash mode register offset. */
#define MC_FMR      (MC_BASE + MC_FMR_OFF)      /*!< \brief MC flash mode register address. */
#define MC_FRDY                 0x00000001      /*!< \brief Flash ready. */
#define MC_LOCKE                0x00000004      /*!< \brief Lock error. */
#define MC_PROGE                0x00000008      /*!< \brief Programming error. */
#define MC_NEBP                 0x00000080      /*!< \brief No erase before programming. */
#define MC_FWS_MASK             0x00000300      /*!< \brief Flash wait state mask. */
#define MC_FWS_1R2W             0x00000000      /*!< \brief 1 cycle for read, 2 for write operations. */
#define MC_FWS_2R3W             0x00000100      /*!< \brief 2 cycles for read, 3 for write operations. */
#define MC_FWS_3R4W             0x00000200      /*!< \brief 3 cycles for read, 4 for write operations. */
#define MC_FWS_4R4W             0x00000300      /*!< \brief 4 cycles for read and write operations. */
#define MC_FMCN_MASK            0x00FF0000      /*!< \brief Flash microsecond cycle number mask. */

#define MC_FCR_OFF              0x00000064      /*!< \brief MC flash command register offset. */
#define MC_FCR      (MC_BASE + MC_FCR_OFF)      /*!< \brief MC flash command register address. */
#define MC_FCMD_MASK            0x0000000F      /*!< \brief Flash command mask. */
#define MC_FCMD_NOP             0x00000000      /*!< \brief No command. */
#define MC_FCMD_WP              0x00000001      /*!< \brief Write page. */
#define MC_FCMD_SLB             0x00000002      /*!< \brief Set lock bit. */
#define MC_FCMD_WPL             0x00000003      /*!< \brief Write page and lock. */
#define MC_FCMD_CLB             0x00000004      /*!< \brief Clear lock bit. */
#define MC_FCMD_EA              0x00000008      /*!< \brief Erase all. */
#define MC_FCMD_SGPB            0x0000000B      /*!< \brief Set general purpose NVM bit. */
#define MC_FCMD_CGPB            0x0000000D      /*!< \brief Clear general purpose NVM bit. */
#define MC_FCMD_SSB             0x0000000F      /*!< \brief Set security bit. */
#define MC_PAGEN_MASK           0x0003FF00      /*!< \brief Page number mask. */
#define MC_KEY                  0x5A000000      /*!< \brief Writing protect key. */

#define MC_FSR_OFF              0x00000068      /*!< \brief MC flash status register offset. */
#define MC_FSR      (MC_BASE + MC_FSR_OFF)      /*!< \brief MC flash status register address. */
#define MC_SECURITY             0x00000010      /*!< \brief Security bit status. */


#endif

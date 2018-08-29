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

#ifndef OPENOCD_LOADERS_FLASH_CC26XX_HW_REGS_H
#define OPENOCD_LOADERS_FLASH_CC26XX_HW_REGS_H

/******************************************************************************
*
* Macros for direct hardware access.
*
* If using these macros the programmer should be aware of any limitations to
* the address accessed i.e. if it supports word and/or byte access.
*
******************************************************************************/
/* Word (32 bit) access to address x */
/* Read example  : my32BitVar = HWREG(base_addr + offset) ; */
/* Write example : HWREG(base_addr + offset) = my32BitVar ; */
#define HWREG(x) (*((volatile unsigned long *)(x)))

/* Half word (16 bit) access to address x */
/* Read example  : my16BitVar = HWREGH(base_addr + offset) ; */
/* Write example : HWREGH(base_addr + offset) = my16BitVar ; */
#define HWREGH(x) (*((volatile unsigned short *)(x)))

/* Byte (8 bit) access to address x */
/* Read example  : my8BitVar = HWREGB(base_addr + offset) ; */
/* Write example : HWREGB(base_addr + offset) = my8BitVar ; */
#define HWREGB(x) (*((volatile unsigned char *)(x)))

/******************************************************************************
*
* Macro for access to bit-band supported addresses via the bit-band region.
*
* Macro calculates the corresponding address to access in the bit-band region
* based on the actual address of the memory/register and the bit number.
*
* Do NOT use this macro to access the bit-band region directly!
*
******************************************************************************/
/* Bit-band access to address x bit number b using word access (32 bit) */
#define HWREGBITW(x, b) \
	HWREG(((unsigned long)(x) & 0xF0000000) | 0x02000000 | \
		(((unsigned long)(x) & 0x000FFFFF) << 5) | ((b) << 2))

/******************************************************************************
*
* Memory mapped components base address definitions
*
******************************************************************************/
#define FLASH_BASE                                                  0x40030000
#define FLASH_CFG_BASE                                              0x50000000
#define AON_PMCTL_BASE                                              0x40090000

/******************************************************************************
*
* This section defines the register offsets of FLASH component
*
******************************************************************************/

/* FMC and Efuse Status */
#define FLASH_O_STAT                                                0x0000001C

/* Configuration */
#define FLASH_O_CFG                                                 0x00000024

/* Flash Size Configuration */
#define FLASH_O_FLASH_SIZE                                          0x0000002C

/* Firmware Lock */
#define FLASH_O_FWLOCK                                              0x0000003C

/* Firmware Flags */
#define FLASH_O_FWFLAG                                              0x00000040

/* FMC Read Control */
#define FLASH_O_FRDCTL                                              0x00002000

/* FMC Bank Protection */
#define FLASH_O_FBPROT                                              0x00002030

/* FMC Bank Sector Enable */
#define FLASH_O_FBSE                                                0x00002034

/* FMC Module Access Control */
#define FLASH_O_FMAC                                                0x00002050

/* FMC Module Status */
#define FLASH_O_FMSTAT                                              0x00002054

/* FMC Flash Lock */
#define FLASH_O_FLOCK                                               0x00002064

/* FMC VREADCT Trim */
#define FLASH_O_FVREADCT                                            0x00002080

/* FMC VHVCT1 Trim */
#define FLASH_O_FVHVCT1                                             0x00002084

/* FMC VHVCT2 Trim */
#define FLASH_O_FVHVCT2                                             0x00002088

/* FMC VNVCT Trim */
#define FLASH_O_FVNVCT                                              0x00002090

/* FMC VSL_P Trim */
#define FLASH_O_FVSLP                                               0x00002094

/* FMC VWLCT Trim */
#define FLASH_O_FVWLCT                                              0x00002098

/* FMC Sequential Pump Information */
#define FLASH_O_FSEQPMP                                             0x000020A8

/* FMC FSM Command */
#define FLASH_O_FSM_CMD                                             0x0000220C

/* FMC FSM Program/Erase Operation Setup */
#define FLASH_O_FSM_PE_OSU                                          0x00002210

/* FMC FSM Voltage Status Setup */
#define FLASH_O_FSM_VSTAT                                           0x00002214

/* FMC FSM Program/Erase Verify Setup */
#define FLASH_O_FSM_PE_VSU                                          0x00002218

/* FMC FSM Compare Verify Setup */
#define FLASH_O_FSM_CMP_VSU                                         0x0000221C

/* FMC FSM EXECUTEZ to Valid Data */
#define FLASH_O_FSM_EX_VAL                                          0x00002220

/* FMC FSM Read Mode Hold */
#define FLASH_O_FSM_RD_H                                            0x00002224

/* FMC FSM Program Hold */
#define FLASH_O_FSM_P_OH                                            0x00002228

/* FMC FSM Erase Operation Hold */
#define FLASH_O_FSM_ERA_OH                                          0x0000222C

/* FMC FSM Program/Erase Verify Hold */
#define FLASH_O_FSM_PE_VH                                           0x00002234

/* FMC FSM Program Pulse Width */
#define FLASH_O_FSM_PRG_PW                                          0x00002240

/* FMC FSM Erase Pulse Width */
#define FLASH_O_FSM_ERA_PW                                          0x00002244

/* FMC FSM Maximum Programming Pulses */
#define FLASH_O_FSM_PRG_PUL                                         0x00002268

/* FMC FSM Maximum Erase Pulses */
#define FLASH_O_FSM_ERA_PUL                                         0x0000226C

/* FMC FSM EC Step Size */
#define FLASH_O_FSM_STEP_SIZE                                       0x00002270

/* FMC FSM EC Step Height */
#define FLASH_O_FSM_EC_STEP_HEIGHT                                  0x00002278

/* FMC FSM_ST_MACHINE */
#define FLASH_O_FSM_ST_MACHINE                                      0x0000227C

/* FMC FSM Register Write Enable */
#define FLASH_O_FSM_WR_ENA                                          0x00002288

/* FMC FSM Command Execute */
#define FLASH_O_FSM_EXECUTE                                         0x000022B4

/* FMC FSM Sector Erased 1 */
#define FLASH_O_FSM_SECTOR1                                         0x000022C0

/* FMC FSM Sector Erased  2 */
#define FLASH_O_FSM_SECTOR2                                         0x000022C4

/* FMC Flash Bank 0 Starting Address */
#define FLASH_O_FCFG_B0_START                                       0x00002410

/* FMC Flash Bank 0 Sector Size 0 */
#define FLASH_O_FCFG_B0_SSIZE0                                      0x00002430

/******************************************************************************
*
* Register: FLASH_O_STAT
*
******************************************************************************/
/* Field:     [2] SAMHOLD_DIS
*
* Status indicator of flash sample and hold sequencing logic. This bit will go
* to 1 some delay after CFG.DIS_IDLE is set to 1.
* 0: Not disabled
* 1: Sample and hold disabled and stable */
#define FLASH_STAT_SAMHOLD_DIS                                      0x00000004

/* Field:     [1] BUSY
*
* Fast version of the FMC FMSTAT.BUSY bit.
* This flag is valid immediately after the operation setting it (FMSTAT.BUSY
* is delayed some cycles)
* 0 : Not busy
* 1 : Busy */
#define FLASH_STAT_BUSY                                             0x00000002

/******************************************************************************
*
* Register: FLASH_O_CFG
*
******************************************************************************/
/* Field:     [8] STANDBY_MODE_SEL
*
* [Configured by boot firmware]
* STANDBY mode selection control. This bit, in conjunction with
* STANDBY_PW_SEL, determine which 1 of 4 sub-modes is selected for control of
* the behavior and timing of the STANDBY input to the pump.
*
* 0 : Legacy PG1 behavior is selected when STANDBY_PW_SEL = 00. This is
* referred to as sub-mode 1. When STANDBY_PW_SEL != 00, then sub-mode 2
* behavior is selected. STANDBY will be glitchy in these modes.
* 1 : STANDBY pulse-width counter modes selected. In these two modes (referred
* to as sub-mode 3 and sub-mode 4), the low time pulse width of the STANDBY
* signal to the pump, is controlled by a programmable timer. STANDBY will not
* be glitchy in these modes. */
#define FLASH_CFG_STANDBY_MODE_SEL_M                                0x00000100
#define FLASH_CFG_STANDBY_MODE_SEL_S                                         8

/* Field:   [7:6] STANDBY_PW_SEL
*
* [Configured by boot firmware]
* STANDBY pulse width counter selection control. These bits, in conjunction
* with STANDBY_MODE_SEL, determine which 1 of 4 sub-modes is selected for
* control of the behavior and timing of the STANDBY input to the pump.
*
* 00 : Legacy PG1 behavior is selected when STANDBY_MODE_SEL=0. Sub-mode 4 is
* selected when STANDBY_MODE_SEL=1. In sub-mode 4, STANDBY will be low for at
* least 9 pump clock cycles.
* 01 : Sub-mode 2 or 3 is selected, and STANDBY will be low for at least 9
* pump clock cycles.
* 10: Sub-mode 2 or 3 is selected, and STANDBY will be low for at least 5 pump
* clock cycles.
* 11: Sub-mode 2 or 3 is selected, and STANDBY will be low for at least 13
* pump clock cycles. */
#define FLASH_CFG_STANDBY_PW_SEL_M                                  0x000000C0
#define FLASH_CFG_STANDBY_PW_SEL_S                                           6

/* Field:     [1] DIS_STANDBY
*
* [Configured by boot firmware]
* Disable standby functionality in read idle state */
#define FLASH_CFG_DIS_STANDBY                                       0x00000002
#define FLASH_CFG_DIS_STANDBY_BITN                                           1
#define FLASH_CFG_DIS_STANDBY_M                                     0x00000002

/* Field:     [0] DIS_IDLE
*
* [Configured by boot firmware]
* Disable sample and hold functionality in read idle state */
#define FLASH_CFG_DIS_IDLE                                          0x00000001
#define FLASH_CFG_DIS_IDLE_M                                        0x00000001
#define FLASH_CFG_DIS_IDLE_S                                                 0

/******************************************************************************
*
* Register: FLASH_O_FLASH_SIZE
*
******************************************************************************/
/* Field:   [7:0] SECTORS
*
* [Configured by boot firmware]
* Flash size. The number of flash sectors in the configured device. Read
* access to sectors equal to this number or higher will result in an error.
* The CCFG area is the sector (SECTORS - 1) Writing to this register is
* disabled by the CFG.CONFIGURED bit. */
#define FLASH_FLASH_SIZE_SECTORS_M                                  0x000000FF
#define FLASH_FLASH_SIZE_SECTORS_S                                           0

/******************************************************************************
*
* Register: FLASH_O_FRDCTL
*
******************************************************************************/
/* Field:  [11:8] RWAIT
*
* [Configured by boot firmware]
* FMC Wait State. This field determines the FLCLK period during FMC controlled
*  flash accesses:
* - During power up/ power down / low power mode
* - During FSM operations like program, erase
* - During software interface mode (see  FLOCK , FBSTROBES registers)
* FLCLK_period = HCLK_period X (RWAIT + 1),
* FSM state machine operations are usually twice this amount. This value
* should never be set less than 2. */
#define FLASH_FRDCTL_RWAIT_M                                        0x00000F00
#define FLASH_FRDCTL_RWAIT_S                                                 8

/******************************************************************************
*
* Register: FLASH_O_FBPROT
*
******************************************************************************/
/* Field:     [0] PROTL1DIS
*
* Level 1 Protection Disable bit. Setting this bit disables protection from
* writing to the FBAC.OTPPROTDIS bits as well as the Sector Enable registers
* FBSE for all banks. Clearing this bit enables protection and disables write
* access to the FBAC.OTPPROTDIS register bits and FBSE register. */
#define FLASH_FBPROT_PROTL1DIS                                      0x00000001

/******************************************************************************
*
* Register: FLASH_O_FMSTAT
*
******************************************************************************/
/* Field:     [4] CSTAT
*
* Command Status. Once the FSM starts any failure will set this bit. When set,
* this bit informs the host that the program, erase, or validate sector
* command failed and the command was stopped. This bit is cleared by the
* Clear_Status command. For some errors, this will be the only indication of
* an FSM error because the cause does not fall within the other error bit
* types. */
#define FLASH_FMSTAT_CSTAT                                          0x00000010

/******************************************************************************
*
* Register: FLASH_O_FVREADCT
*
******************************************************************************/
/* Field:   [3:0] VREADCT
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of
* wordline power supply for read mode. */
#define FLASH_FVREADCT_VREADCT_M                                    0x0000000F
#define FLASH_FVREADCT_VREADCT_S                                             0

/******************************************************************************
*
* Register: FLASH_O_FVHVCT1
*
******************************************************************************/
/* Field: [23:20] TRIM13_E
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of high
* voltage supply input during erase operation. */
#define FLASH_FVHVCT1_TRIM13_E_M                                    0x00F00000
#define FLASH_FVHVCT1_TRIM13_E_S                                            20

/* Field: [19:16] VHVCT_E
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of high
* voltage supply input during erase operation. */
#define FLASH_FVHVCT1_VHVCT_E_M                                     0x000F0000
#define FLASH_FVHVCT1_VHVCT_E_S                                             16

/* Field:   [7:4] TRIM13_PV
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of high
* voltage supply input during program verify operation. */
#define FLASH_FVHVCT1_TRIM13_PV_M                                   0x000000F0
#define FLASH_FVHVCT1_TRIM13_PV_S                                            4

/* Field:   [3:0] VHVCT_PV
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of high
* voltage supply input during program verify operation. */
#define FLASH_FVHVCT1_VHVCT_PV_M                                    0x0000000F
#define FLASH_FVHVCT1_VHVCT_PV_S                                             0

/******************************************************************************
*
* Register: FLASH_O_FVHVCT2
*
******************************************************************************/
/* Field: [23:20] TRIM13_P
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of high
* voltage supply input during program operation. */
#define FLASH_FVHVCT2_TRIM13_P_M                                    0x00F00000
#define FLASH_FVHVCT2_TRIM13_P_S                                            20

/* Field: [19:16] VHVCT_P
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of high
* voltage supply input during program operation. */
#define FLASH_FVHVCT2_VHVCT_P_M                                     0x000F0000
#define FLASH_FVHVCT2_VHVCT_P_S                                             16

/******************************************************************************
*
* Register: FLASH_O_FVNVCT
*
******************************************************************************/
/* Field:  [12:8] VCG2P5CT
*
* [Configured by boot firmware]
* These bits control the voltage level for the VCG 2.5 CT pump voltage. */
#define FLASH_FVNVCT_VCG2P5CT_M                                     0x00001F00
#define FLASH_FVNVCT_VCG2P5CT_S                                              8

/******************************************************************************
*
* Register: FLASH_O_FVSLP
*
******************************************************************************/
/* Field: [15:12] VSL_P
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of high
* current power input during program operation. */
#define FLASH_FVSLP_VSL_P_M                                         0x0000F000
#define FLASH_FVSLP_VSL_P_S                                                 12

/******************************************************************************
*
* Register: FLASH_O_FVWLCT
*
******************************************************************************/
/* Field:   [4:0] VWLCT_P
*
* [Configured by boot firmware]
* These bits control the voltage level for the specified pump voltage of
* wordline power supply during programming operations. */
#define FLASH_FVWLCT_VWLCT_P_M                                      0x0000001F
#define FLASH_FVWLCT_VWLCT_P_S                                               0

/******************************************************************************
*
* Register: FLASH_O_FSEQPMP
*
******************************************************************************/
/* Field: [21:20] TRIM_1P7
*
* [Configured by boot firmware]
* This register goes directly to the pump's TRIM_1P7 port pins. */
#define FLASH_FSEQPMP_TRIM_1P7_M                                    0x00300000
#define FLASH_FSEQPMP_TRIM_1P7_S                                            20

/* Field: [14:12] VIN_AT_X
*
* This register controls to the pump's VIN_AT_XPX port pins with the following
* encoding;
*
* If VIN_BY_PASS=0 then pump VIN_AT_XPX is equal to VIN_AT_XIN input ports
* from the BATMON logic after clocking through synchronizers and the sequence
* checker FSM logic contained in the flash wrapper.
*
* If VIN_BY_PASS=1 and VIN_AT_X=???
*
* 0: then all pump VIN_AT_XPX signals are 0.
* 1: then pump VIN_AT_1P7 is set.
* 2: then pump VIN_AT_2P1 is also set.
* 3: then pump VIN_AT_2P4 is also set.
* 4-7: then pump VIN_AT_3P0 is also set (ie all VIN_AT_XPX signals are 1). */
#define FLASH_FSEQPMP_VIN_AT_X_M                                    0x00007000
#define FLASH_FSEQPMP_VIN_AT_X_S                                            12

/* Field:     [8] VIN_BY_PASS
*
* [Configured by boot firmware]
*
* When this bit is a zero, the pump's VIN_AT_XPX ports comes from the FMC
* input port VIN_AT_XIN.
*
* When this bit is a one, the pump's VIN_AT_XPX ports comes from the VIN_AT_X
* bits in 14:12. */
#define FLASH_FSEQPMP_VIN_BY_PASS                                   0x00000100
#define FLASH_FSEQPMP_VIN_BY_PASS_M                                 0x00000100

/******************************************************************************
*
* Register: FLASH_O_FSM_PE_OSU
*
******************************************************************************/
/* Field:  [15:8] PGM_OSU
*
* [Configured by boot firmware]
* Program Operation Setup time. This determines the flash clocks from the mode
* change to program, to the start of the program pulse. */
#define FLASH_FSM_PE_OSU_PGM_OSU_M                                  0x0000FF00
#define FLASH_FSM_PE_OSU_PGM_OSU_S                                           8

/* Field:   [7:0] ERA_OSU
*
* [Configured by boot firmware]
* Erase Operation Setup time. This determines the flash clocks from the mode
* change to erase, to the start of the erase pulse. */
#define FLASH_FSM_PE_OSU_ERA_OSU_M                                  0x000000FF
#define FLASH_FSM_PE_OSU_ERA_OSU_S                                           0

/******************************************************************************
*
* Register: FLASH_O_FSM_VSTAT
*
******************************************************************************/
/* Field: [15:12] VSTAT_CNT
*
* [Configured by boot firmware]
* Voltage Status Count. Gives the number of consecutive HCLK pulses that must
* be out of range before a voltage-out-of-range status error is given in
* FMSTAT.VOLSTAT. One pulse in range will reset the counter. This is mainly a
* glitch filter on the voltage status pump signal. */
#define FLASH_FSM_VSTAT_VSTAT_CNT_M                                 0x0000F000
#define FLASH_FSM_VSTAT_VSTAT_CNT_S                                         12

/******************************************************************************
*
* Register: FLASH_O_FSM_PE_VSU
*
******************************************************************************/
/* Field:  [15:8] PGM_VSU
*
* [Configured by boot firmware]
* Program Verify Setup time. This determines the flash clocks from the mode
* change to program verify, to the change of address and the beginning of the
* address setup time. */
#define FLASH_FSM_PE_VSU_PGM_VSU_M                                  0x0000FF00
#define FLASH_FSM_PE_VSU_PGM_VSU_S                                           8

/* Field:   [7:0] ERA_VSU
*
* [Configured by boot firmware]
* Erase Verify Setup time. This determines the flash clocks from the mode
* change to erase verify, to the change of address and the beginning of the
* address setup time. */
#define FLASH_FSM_PE_VSU_ERA_VSU_M                                  0x000000FF
#define FLASH_FSM_PE_VSU_ERA_VSU_S                                           0

/******************************************************************************
*
* Register: FLASH_O_FSM_CMP_VSU
*
******************************************************************************/
/* Field: [15:12] ADD_EXZ
*
* [Configured by boot firmware]
* Address to EXECUTEZ low setup time. This determines the flash clocks from
* the row address change to the time EXECUTEZ goes low. All operations use
* this value. */
#define FLASH_FSM_CMP_VSU_ADD_EXZ_M                                 0x0000F000
#define FLASH_FSM_CMP_VSU_ADD_EXZ_S                                         12

/******************************************************************************
*
* Register: FLASH_O_FSM_EX_VAL
*
******************************************************************************/
/* Field:  [15:8] REP_VSU
*
* [Configured by boot firmware]
* Repeat Verify action setup. If a program or erase operation advances to the
* program_verify or erase_verify then this special shorter mode transition
* time will be used in place of FSM_PE_VSU.PGM_VSU or FSM_PE_VSU.ERA_VSU
* times. */
#define FLASH_FSM_EX_VAL_REP_VSU_M                                  0x0000FF00
#define FLASH_FSM_EX_VAL_REP_VSU_S                                           8

/* Field:   [7:0] EXE_VALD
*
* [Configured by boot firmware]
* EXECUTEZ low to valid Data. Determines the number of Flash clock cycles from
* EXECUTEZ going low to the time the verify data can be read in the program
* verify mode. Erase and compact verify is always a constant value which is
* currently set at one flash clock. This value must be greater than 0. */
#define FLASH_FSM_EX_VAL_EXE_VALD_M                                 0x000000FF
#define FLASH_FSM_EX_VAL_EXE_VALD_S                                          0

/******************************************************************************
*
* Register: FLASH_O_FSM_RD_H
*
******************************************************************************/
/* Field:   [7:0] RD_H
*
* [Configured by boot firmware]
* Read mode hold. This determines the number of flash clocks from the start of
* the Read mode at the end of the operations until the FSM clears the
* FMSTAT.BUSY. Writing a zero to this register will result in a value of 1.
* The reset value of this register is 0x3Ah before FMC version 3.0.10.0 and
* 0x5Ah after this version. */
#define FLASH_FSM_RD_H_RD_H_M                                       0x000000FF
#define FLASH_FSM_RD_H_RD_H_S                                                0

/******************************************************************************
*
* Register: FLASH_O_FSM_P_OH
*
******************************************************************************/
/* Field:  [15:8] PGM_OH
*
* [Configured by boot firmware]
* EXECUTEZ high to mode change. This value determines the flash clocks from
* the EXECUTEZ going high at the end of a program operation to the time the
* mode can change. This value must be greater than or equal to one. */
#define FLASH_FSM_P_OH_PGM_OH_M                                     0x0000FF00
#define FLASH_FSM_P_OH_PGM_OH_S                                              8

/******************************************************************************
*
* Register: FLASH_O_FSM_ERA_OH
*
******************************************************************************/
/* Field:  [15:0] ERA_OH
*
* [Configured by boot firmware]
* EXECUTEZ high to mode change. Determines the flash clocks from EXECUTEZ
* going high at the end of an erase operation to the time the mode can change.
* If a bank erase is happening, then this is the time to when the TEZ and TCR
* values for bank erase are released. The mode changes 10 flash clocks after
* they are released. This value must be greater than or equal to one. */
#define FLASH_FSM_ERA_OH_ERA_OH_M                                   0x0000FFFF
#define FLASH_FSM_ERA_OH_ERA_OH_S                                            0

/******************************************************************************
*
* Register: FLASH_O_FSM_PE_VH
*
******************************************************************************/
/* Field:  [15:8] PGM_VH
*
* [Configured by boot firmware]
* Program Verify Hold. This register determines the flash clocks from EXECUTEZ
* going high after a program verify to a mode change. This value must be
* greater than or equal to one */
#define FLASH_FSM_PE_VH_PGM_VH_M                                    0x0000FF00
#define FLASH_FSM_PE_VH_PGM_VH_S                                             8

/******************************************************************************
*
* Register: FLASH_O_FSM_PRG_PW
*
******************************************************************************/
/* Field:  [15:0] PROG_PUL_WIDTH
*
* [Configured by boot firmware]
* Program Pulse width.This register gives the number of flash clocks that the
* EXECUTEZ signal is low in a program operation. */
#define FLASH_FSM_PRG_PW_PROG_PUL_WIDTH_M                           0x0000FFFF
#define FLASH_FSM_PRG_PW_PROG_PUL_WIDTH_S                                    0

/******************************************************************************
*
* Register: FLASH_O_FSM_ERA_PW
*
******************************************************************************/
/* Field:  [31:0] FSM_ERA_PW
*
* [Configured by boot firmware]
* Erase Pulse width. This register gives the number flash clocks that the
* EXECUTEZ signal is low in an erase operation. */
#define FLASH_FSM_ERA_PW_FSM_ERA_PW_M                               0xFFFFFFFF
#define FLASH_FSM_ERA_PW_FSM_ERA_PW_S                                        0

/******************************************************************************
*
* Register: FLASH_O_FSM_PRG_PUL
*
******************************************************************************/
/* Field: [19:16] BEG_EC_LEVEL
*
* [Configured by boot firmware]
* Beginning level for VHVCT. This determines the beginning level for VHVCT
* that is used during erase modes. The pump voltage control registers supply
* the other values that do not change during FSM operations. The reset value
* is the same as FVHVCT1.VHVCT_E. */
#define FLASH_FSM_PRG_PUL_BEG_EC_LEVEL_M                            0x000F0000
#define FLASH_FSM_PRG_PUL_BEG_EC_LEVEL_S                                    16

/* Field:  [11:0] MAX_PRG_PUL
*
* [Configured by boot firmware]
* Maximum Programming Pulses. This register contains the maximum number of
* programming pulses allowed at one address. If it takes any more than this
* amount during a programming operation then the FSM will exit with an error
* and with the program violation, FMSTAT.PGV set, and the general error set,
* FMSTAT.CSTAT. Setting FSM_ST_MACHINE.OVERRIDE to 0 will allow more than this
* maximum value to occur without an error. During pre-conditioning for an
* erase operation the FSM programs all the bits to zero. If the maximum number
* of programming pulses is reached for an address, the FSM will continue with
* the next address and set the FMSTAT.PCV and the general error FMSTAT.CSTAT.
* If the FSM_ST_MACHINE.PREC_STOP_EN is set then the FSM will stop with errors
* when more than the maximum number of pulses is needed. The
* FSM_ST_MACHINE.OVERRIDE bit will take priority over the
* FSM_ST_MACHINE.PREC_STOP_EN and continue doing pulses without setting the
* error bits. Suspend operations will count a pulse if the program operation
* began no matter how long the pulse lasted before is was suspended. Frequent
* suspend or auto-suspend operations could result in max_pulse count error. */
#define FLASH_FSM_PRG_PUL_MAX_PRG_PUL_M                             0x00000FFF
#define FLASH_FSM_PRG_PUL_MAX_PRG_PUL_S                                      0

/******************************************************************************
*
* Register: FLASH_O_FSM_ERA_PUL
*
******************************************************************************/
/* Field: [19:16] MAX_EC_LEVEL
*
* [Configured by boot firmware]
* Maximum VHVCT Level. This determines the maximum level for VHVCT that is
* used during erase modes. The FSM will stop advancing VHVCT once it counts up
* to the MAX_EC_LEVEL level from the beginning level. The MAX_EC_LEVEL +
* FSM_EC_STEP_HEIGHT.EC_STEP_HEIGHT must be less than 0x200. The reset value
* is the same as FVHVCT1.VHVCT_E. */
#define FLASH_FSM_ERA_PUL_MAX_EC_LEVEL_M                            0x000F0000
#define FLASH_FSM_ERA_PUL_MAX_EC_LEVEL_S                                    16

/* Field:  [11:0] MAX_ERA_PUL
*
* [Configured by boot firmware]
* Maximum Erase Pulses. This register contains the maximum number of erase
* pulses allowed at one address. If it takes any more than this amount the FSM
* will exit with an error and with both the FMSTAT.EV and FMSTAT.CSTAT bits
* set. Setting FSM_ST_MACHINE.OVERRIDE to 1 will allow more than this maximum
* value to occur without an error. Suspend operations will count a pulse if
* the erase operation began no matter how long the pulse lasted before is was
* suspended. Frequent suspend or auto-suspend operations could result in
* max_pulse count error. */
#define FLASH_FSM_ERA_PUL_MAX_ERA_PUL_M                             0x00000FFF
#define FLASH_FSM_ERA_PUL_MAX_ERA_PUL_S                                      0

/******************************************************************************
*
* Register: FLASH_O_FSM_STEP_SIZE
*
******************************************************************************/
/* Field: [24:16] EC_STEP_SIZE
*
* [Configured by boot firmware]
* VHVCT Step Size. This is the number of erase pulses that must be completed
* for each level before the FSM increments the FSM_PUL_CNTR.CUR_EC_LEVEL to
* the next higher level. Actual erase pulses per level equals (EC_STEP_SIZE
* +1). The stepping is only needed for the VHVCT voltage. */
#define FLASH_FSM_STEP_SIZE_EC_STEP_SIZE_M                          0x01FF0000
#define FLASH_FSM_STEP_SIZE_EC_STEP_SIZE_S                                  16

/******************************************************************************
*
* Register: FLASH_O_FSM_EC_STEP_HEIGHT
*
******************************************************************************/
/* Field:   [3:0] EC_STEP_HEIGHT
*
* [Configured by boot firmware]
* Height of each EC step. This is the number of counts that the
* FSM_PUL_CNTR.CUR_EC_LEVEL will increment when going to a new level. Actual
* count size equals (EC_STEP_HEIGHT + 1). The stepping applies only to the
* VHVCT voltage. If adding the height to the FSM_PUL_CNTR.CUR_EC_LEVEL results
* in a value higher than the FSM_ERA_PUL.MAX_EC_LEVEL then the
* FSM_PUL_CNTR.CUR_EC_LEVEL will be lowered to the MAX LEVEL before it is used
* in the next erase pulse. */
#define FLASH_FSM_EC_STEP_HEIGHT_EC_STEP_HEIGHT_M                   0x0000000F

/******************************************************************************
*
* Register: FLASH_O_FSM_ST_MACHINE
*
******************************************************************************/
/* Field:    [23] DO_PRECOND
*
* [Configured by boot firmware]
* Do preconditioning. When this bit is a one, the FSM will precondition the
* sector or bank before doing an erase operation. When zero, the FSM will just
* begin with the erase verify and skip the preconditioning. */
#define FLASH_FSM_ST_MACHINE_DO_PRECOND                             0x00800000
#define FLASH_FSM_ST_MACHINE_DO_PRECOND_M                           0x00800000
#define FLASH_FSM_ST_MACHINE_DO_PRECOND_S                                   23

/* Field:    [14] ONE_TIME_GOOD
*
* [Configured by boot firmware]
* One Time Good function. If this bit is a one then the 'One Time Good'
* function is enabled for all program operations. This includes operations
* inside the erase functions and other functions. When zero, this function is
* disabled for all modes. When doing the One Time Good function, the FSM will
* attempt to program a location with data. If a desired zero bit reads back
* from the flash one time as good then that bit is blocked from writing a zero
* to the flash array again for this address. When the address changes, all
* bits are unblocked. This prevents a bit from reading 0 in one programming
* pulse and then 1 in the next programming pulse. On the second time the bit
* would get a programming pulse even though it read 0 in an earlier read. If
* this bit is a zero then the zero bits will be masked for each program verify
* operation. It is recommended for this bit to be set to 1. */
#define FLASH_FSM_ST_MACHINE_ONE_TIME_GOOD                          0x00004000

/******************************************************************************
*
* Register: FLASH_O_FCFG_B0_SSIZE0
*
******************************************************************************/
/* Field:   [3:0] B0_SECT_SIZE
*
* Size of sectors in Bank 0. Common sector size for all sectors in the bank in
* 1K bytes multiples.
* 0x0: 0K bytes
* 0x1: 1K bytes(FLES)
* 0x2: 2K bytes
* 0x4: 4K bytes (FLEE)
* ...
* 0xF: 15K bytes */
#define FLASH_FCFG_B0_SSIZE0_B0_SECT_SIZE_M                         0x0000000F
#define FLASH_FCFG_B0_SSIZE0_B0_SECT_SIZE_S                                  0

/******************************************************************************
*
* This section defines the register offsets of FCFG1 component
*
******************************************************************************/

/* Flash Erase and Program Setup Time */
#define FCFG1_O_FLASH_E_P                                           0x00000170

/* Flash Compaction, Execute, Program and Read */
#define FCFG1_O_FLASH_C_E_P_R                                       0x00000174

/* Flash Program, Read, and Program Verify */
#define FCFG1_O_FLASH_P_R_PV                                        0x00000178

/* Flash Erase Hold and Sequence */
#define FCFG1_O_FLASH_EH_SEQ                                        0x0000017C

/* Flash VHV Erase */
#define FCFG1_O_FLASH_VHV_E                                         0x00000180

/* Flash Program Pulse */
#define FCFG1_O_FLASH_PP                                            0x00000184

/* Flash Program and Erase Pulse */
#define FCFG1_O_FLASH_PROG_EP                                       0x00000188

/* Flash Erase Pulse Width */
#define FCFG1_O_FLASH_ERA_PW                                        0x0000018C

/* Flash VHV */
#define FCFG1_O_FLASH_VHV                                           0x00000190

/* Flash VHV Program Verify */
#define FCFG1_O_FLASH_VHV_PV                                        0x00000194

/* Flash Voltages */
#define FCFG1_O_FLASH_V                                             0x00000198

/* Flash OTP Data 3 */
#define FCFG1_O_FLASH_OTP_DATA3                                     0x000002B0

/* Flash OTP Data 4 */
#define FCFG1_O_FLASH_OTP_DATA4                                     0x00000308

/******************************************************************************
*
* Register: FCFG1_O_FLASH_E_P
*
******************************************************************************/
/* Field: [31:24] PSU
*
* Program setup time in cycles. Value will be written to
* FLASH:FSM_PE_OSU.PGM_OSU by the flash device driver when an erase/program
* operation is initiated. */
#define FCFG1_FLASH_E_P_PSU_M                                       0xFF000000
#define FCFG1_FLASH_E_P_PSU_S                                               24

/* Field: [23:16] ESU
*
* Erase setup time in cycles. Value will be written to
* FLASH:FSM_PE_OSU.ERA_OSU by the flash device driver when an erase/program
* operation is initiated. */
#define FCFG1_FLASH_E_P_ESU_M                                       0x00FF0000
#define FCFG1_FLASH_E_P_ESU_S                                               16

/* Field:  [15:8] PVSU
*
* Program verify setup time in cycles. Value will be written to
* FLASH:FSM_PE_VSU.PGM_VSU by the flash device driver when an erase/program
* operation is initiated. */
#define FCFG1_FLASH_E_P_PVSU_M                                      0x0000FF00
#define FCFG1_FLASH_E_P_PVSU_S                                               8

/* Field:   [7:0] EVSU
*
* Erase verify setup time in cycles. Value will be written to
* FLASH:FSM_PE_VSU.ERA_VSU by the flash device driver when an erase/program
* operation is initiated. */
#define FCFG1_FLASH_E_P_EVSU_M                                      0x000000FF
#define FCFG1_FLASH_E_P_EVSU_S                                               0

/******************************************************************************
*
* Register: FCFG1_O_FLASH_C_E_P_R
*
******************************************************************************/
/* Field: [31:24] RVSU
*
* Repeat verify setup time in cycles. Used for repeated verifies during
* program and erase. Value will be written to FLASH:FSM_EX_VAL.REP_VSU by the
* flash device driver when an erase/program operation is initiated. */
#define FCFG1_FLASH_C_E_P_R_RVSU_M                                  0xFF000000
#define FCFG1_FLASH_C_E_P_R_RVSU_S                                          24

/* Field: [23:16] PV_ACCESS
*
* Program verify EXECUTEZ-&#62;data valid time in half-microseconds. Value
* will be converted to number of FCLK cycles by by flash device driver and the
* converted value is written to FLASH:FSM_EX_VAL.EXE_VALD when an
* erase/program operation is initiated. */
#define FCFG1_FLASH_C_E_P_R_PV_ACCESS_M                             0x00FF0000
#define FCFG1_FLASH_C_E_P_R_PV_ACCESS_S                                     16

/* Field: [15:12] A_EXEZ_SETUP
*
* Address-&#62;EXECUTEZ setup time in cycles. Value will be written to
* FLASH:FSM_CMP_VSU.ADD_EXZ by the flash device driver when an erase/program
* operation is initiated. */
#define FCFG1_FLASH_C_E_P_R_A_EXEZ_SETUP_M                          0x0000F000
#define FCFG1_FLASH_C_E_P_R_A_EXEZ_SETUP_S                                  12

/******************************************************************************
*
* Register: FCFG1_O_FLASH_P_R_PV
*
******************************************************************************/
/* Field: [31:24] PH
*
* Program hold time in half-microseconds after SAFELV goes high. Value will be
* converted to number of FCLK cycles by the flash device driver and the
* converted value is written to FLASH:FSM_P_OH.PGM_OH when an erase/program
* operation is initiated. */
#define FCFG1_FLASH_P_R_PV_PH_M                                     0xFF000000
#define FCFG1_FLASH_P_R_PV_PH_S                                             24

/* Field: [23:16] RH
*
* Read hold/mode transition time in cycles. Value will be written to the RD_H
* field bits[7:0] of the FSM_RD_H register in the flash module by the flash
* device driver when an erase/program operation is initiated. */
#define FCFG1_FLASH_P_R_PV_RH_M                                     0x00FF0000
#define FCFG1_FLASH_P_R_PV_RH_S                                             16

/* Field:  [15:8] PVH
*
* Program verify hold time in half-microseconds after SAFELV goes high. Value
* will be converted to number of FCLK cycles by the flash device driver and
* the converted value is written to FLASH:FSM_PE_VH.PGM_VH when an
* erase/program operation is initiated. */
#define FCFG1_FLASH_P_R_PV_PVH_M                                    0x0000FF00
#define FCFG1_FLASH_P_R_PV_PVH_S                                             8

/******************************************************************************
*
* Register: FCFG1_O_FLASH_EH_SEQ
*
******************************************************************************/
/* Field: [31:24] EH
*
* Erase hold time in half-microseconds after SAFELV goes high. Value will be
* converted to number of FCLK cycles by the flash device driver and the
* converted value is written to FLASH:FSM_ERA_OH.ERA_OH when an erase/program
* operation is initiated. */
#define FCFG1_FLASH_EH_SEQ_EH_M                                     0xFF000000
#define FCFG1_FLASH_EH_SEQ_EH_S                                             24

/* Field: [15:12] VSTAT
*
* Max number of HCLK cycles allowed for pump brown-out. Value will be written
* to FLASH:FSM_VSTAT.VSTAT_CNT when an erase/program operation is initiated. */
#define FCFG1_FLASH_EH_SEQ_VSTAT_M                                  0x0000F000
#define FCFG1_FLASH_EH_SEQ_VSTAT_S                                          12

/******************************************************************************
*
* Register: FCFG1_O_FLASH_VHV_E
*
******************************************************************************/
/* Field: [31:16] VHV_E_START
*
* Starting VHV-Erase CT for stairstep erase. Value will be written to
* FLASH:FSM_PRG_PUL.BEG_EC_LEVEL when erase/program operation is initiated. */
#define FCFG1_FLASH_VHV_E_VHV_E_START_M                             0xFFFF0000
#define FCFG1_FLASH_VHV_E_VHV_E_START_S                                     16

/* Field:  [15:0] VHV_E_STEP_HIGHT
*
* Number of VHV CTs to step after each erase pulse (up to the max). The actual
* FMC register value should be one less than this since the FMC starts
* counting from zero. Value will be written to
* FLASH:FSM_EC_STEP_HEIGHT.EC_STEP_HEIGHT when an erase/program operation is
* initiated. */
#define FCFG1_FLASH_VHV_E_VHV_E_STEP_HIGHT_M                        0x0000FFFF
#define FCFG1_FLASH_VHV_E_VHV_E_STEP_HIGHT_S                                 0

/******************************************************************************
*
* Register: FCFG1_O_FLASH_PP
*
******************************************************************************/
/* Field:  [15:0] MAX_PP
*
* Max program pulse limit per program operation. Value will be written to
* FLASH:FSM_PRG_PUL.MAX_PRG_PUL when an erase/program operation is initiated. */
#define FCFG1_FLASH_PP_MAX_PP_M                                     0x0000FFFF
#define FCFG1_FLASH_PP_MAX_PP_S                                              0

/******************************************************************************
*
* Register: FCFG1_O_FLASH_PROG_EP
*
******************************************************************************/
/* Field: [31:16] MAX_EP
*
* Max erase pulse limit per erase operation. Value will be written to
* FLASH:FSM_ERA_PUL.MAX_ERA_PUL when an erase/program operation is initiated. */
#define FCFG1_FLASH_PROG_EP_MAX_EP_M                                0xFFFF0000
#define FCFG1_FLASH_PROG_EP_MAX_EP_S                                        16

/* Field:  [15:0] PROGRAM_PW
*
* Program pulse width in half-microseconds. Value will be converted to number
* of FCLK cycles by the flash device driver and the converted value is written
* to FLASH:FSM_PRG_PW.PROG_PUL_WIDTH when a erase/program operation is
* initiated. */
#define FCFG1_FLASH_PROG_EP_PROGRAM_PW_M                            0x0000FFFF
#define FCFG1_FLASH_PROG_EP_PROGRAM_PW_S                                     0

/******************************************************************************
*
* Register: FCFG1_O_FLASH_ERA_PW
*
******************************************************************************/
/* Field:  [31:0] ERASE_PW
*
* Erase pulse width in half-microseconds. Value will be converted to number of
* FCLK cycles by the flash device driver and the converted value is written to
* FLASH:FSM_ERA_PW.FSM_ERA_PW when a erase/program operation is initiated. */
#define FCFG1_FLASH_ERA_PW_ERASE_PW_M                               0xFFFFFFFF
#define FCFG1_FLASH_ERA_PW_ERASE_PW_S                                        0

/******************************************************************************
*
* Register: FCFG1_O_FLASH_VHV
*
******************************************************************************/
/* Field: [27:24] TRIM13_P
*
* Value will be written to FLASH:FVHVCT2.TRIM13_P by the flash device driver
* when an erase/program operation is initiated. */
#define FCFG1_FLASH_VHV_TRIM13_P_M                                  0x0F000000
#define FCFG1_FLASH_VHV_TRIM13_P_S                                          24

/* Field: [19:16] VHV_P
*
* Value will be written to FLASH:FVHVCT2.VHVCT_P by the flash device driver
* when an erase/program operation is initiated. */
#define FCFG1_FLASH_VHV_VHV_P_M                                     0x000F0000
#define FCFG1_FLASH_VHV_VHV_P_S                                             16

/* Field:  [11:8] TRIM13_E
*
* Value will be written to FLASH:FVHVCT1.TRIM13_E by the flash device driver
* when an erase/program operation is initiated. */
#define FCFG1_FLASH_VHV_TRIM13_E_M                                  0x00000F00
#define FCFG1_FLASH_VHV_TRIM13_E_S                                           8

/* Field:   [3:0] VHV_E
*
* Value will be written to FLASH:FVHVCT1.VHVCT_E by the flash device driver
* when an erase/program operation is initiated */
#define FCFG1_FLASH_VHV_VHV_E_M                                     0x0000000F
#define FCFG1_FLASH_VHV_VHV_E_S                                              0

/******************************************************************************
*
* Register: FCFG1_O_FLASH_VHV_PV
*
******************************************************************************/
/* Field: [27:24] TRIM13_PV
*
* Value will be written to FLASH:FVHVCT1.TRIM13_PV by the flash device driver
* when an erase/program operation is initiated. */
#define FCFG1_FLASH_VHV_PV_TRIM13_PV_M                              0x0F000000
#define FCFG1_FLASH_VHV_PV_TRIM13_PV_S                                      24

/* Field: [19:16] VHV_PV
*
* Value will be written to FLASH:FVHVCT1.VHVCT_PV by the flash device driver
* when an erase/program operation is initiated. */
#define FCFG1_FLASH_VHV_PV_VHV_PV_M                                 0x000F0000
#define FCFG1_FLASH_VHV_PV_VHV_PV_S                                         16

/* Field:  [15:8] VCG2P5
*
* Control gate voltage during read, read margin, and erase verify. Value will
* be written to FLASH:FVNVCT.VCG2P5CT by the flash device driver when an
* erase/program operation is initiated. */
#define FCFG1_FLASH_VHV_PV_VCG2P5_M                                 0x0000FF00
#define FCFG1_FLASH_VHV_PV_VCG2P5_S                                          8

/******************************************************************************
*
* Register: FCFG1_O_FLASH_V
*
******************************************************************************/
/* Field: [31:24] VSL_P
*
* Sourceline voltage applied to the selected block during programming. Value
* will be written to FLASH:FVSLP.VSL_P by the flash device driver when an
* erase/program operation is initiated. */
#define FCFG1_FLASH_V_VSL_P_M                                       0xFF000000
#define FCFG1_FLASH_V_VSL_P_S                                               24

/* Field: [23:16] VWL_P
*
* Wordline voltage applied to the selected half-row during programming. Value
* will be written to  FLASH:FVWLCT.VWLCT_P by the flash device driver when an
* erase/program operation is initiated. */
#define FCFG1_FLASH_V_VWL_P_M                                       0x00FF0000
#define FCFG1_FLASH_V_VWL_P_S                                               16

/* Field:  [15:8] V_READ
*
* Wordline voltage applied to the selected block during reads and verifies.
* Value will be written to FLASH:FVREADCT.VREADCT by the flash device driver
* when an erase/program operation is initiated. */
#define FCFG1_FLASH_V_V_READ_M                                      0x0000FF00
#define FCFG1_FLASH_V_V_READ_S                                               8

/******************************************************************************
*
* Register: FCFG1_O_FLASH_OTP_DATA3
*
******************************************************************************/
/* Field: [31:23] EC_STEP_SIZE
*
* Value will be written to FLASH:FSM_STEP_SIZE.EC_STEP_SIZE by the flash
* device driver when a erase/program operation is initiated. */
#define FCFG1_FLASH_OTP_DATA3_EC_STEP_SIZE_M                        0xFF800000
#define FCFG1_FLASH_OTP_DATA3_EC_STEP_SIZE_S                                23

/* Field:    [22] DO_PRECOND
*
* Value will be written to FLASH:FSM_ST_MACHINE.DO_PRECOND by the flash device
* driver when a erase/program operation is initiated.
*
* Note that during a Total Erase operation the flash bank will always be
* erased with Precondition enabled independent of the value of this FCFG1 bit
* field. */
#define FCFG1_FLASH_OTP_DATA3_DO_PRECOND_M                          0x00400000
#define FCFG1_FLASH_OTP_DATA3_DO_PRECOND_S                                  22

/* Field: [21:18] MAX_EC_LEVEL
*
* Value will be written to FLASH:FSM_ERA_PUL.MAX_EC_LEVEL by the flash device
* driver when a erase/program operation is initiated. */
#define FCFG1_FLASH_OTP_DATA3_MAX_EC_LEVEL_M                        0x003C0000
#define FCFG1_FLASH_OTP_DATA3_MAX_EC_LEVEL_S                                18

/* Field: [17:16] TRIM_1P7
*
* Value will be written to FLASH:FSEQPMP.TRIM_1P7 by the flash device driver
* when a erase/program operation is initiated. */
#define FCFG1_FLASH_OTP_DATA3_TRIM_1P7_M                            0x00030000
#define FCFG1_FLASH_OTP_DATA3_TRIM_1P7_S                                    16

/******************************************************************************
*
* Register: FCFG1_O_FLASH_OTP_DATA4
*
******************************************************************************/
/* Field:    [31] STANDBY_MODE_SEL_INT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.STANDBY_MODE_SEL by flash device driver FW when a flash write
* operation is initiated. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_WRT_M            0x80000000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_WRT_S                    31

/* Field: [30:29] STANDBY_PW_SEL_INT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.STANDBY_PW_SEL by flash device driver FW when a flash write
* operation is initiated. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_WRT_M              0x60000000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_WRT_S                      29

/* Field:    [28] DIS_STANDBY_INT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.DIS_STANDBY by flash device driver FW when a flash write operation
* is initiated. */
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_WRT_M                 0x10000000

/* Field:    [27] DIS_IDLE_INT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.DIS_IDLE by flash device driver FW when a flash write operation is
* initiated. */
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_WRT_M                    0x08000000
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_WRT_S                            27

/* Field: [26:24] VIN_AT_X_INT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:FSEQPMP.VIN_AT_X by flash device driver FW when a flash write
* operation is initiated. */
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_WRT_M                    0x07000000
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_WRT_S                            24

/* Field:    [23] STANDBY_MODE_SEL_EXT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.STANDBY_MODE_SEL by flash device driver FW when a flash write
* operation is initiated. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_WRT_M            0x00800000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_WRT_S                    23

/* Field: [22:21] STANDBY_PW_SEL_EXT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.STANDBY_PW_SEL by flash device driver FW when a flash write
* operation is initiated. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_WRT_M              0x00600000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_WRT_S                      21

/* Field:    [20] DIS_STANDBY_EXT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.DIS_STANDBY by flash device driver FW when a flash write operation
* is initiated. */
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_WRT_M                 0x00100000

/* Field:    [19] DIS_IDLE_EXT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.DIS_IDLE by flash device driver FW when a flash write operation is
* initiated. */
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_WRT_M                    0x00080000
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_WRT_S                            19

/* Field: [18:16] VIN_AT_X_EXT_WRT
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:FSEQPMP.VIN_AT_X by flash device driver FW when a flash write
* operation is initiated. */
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_WRT_M                    0x00070000
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_WRT_S                            16

/* Field:    [15] STANDBY_MODE_SEL_INT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.STANDBY_MODE_SEL both by boot FW while in safezone, and by flash
* device driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_RD_M             0x00008000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_RD_S                     15

/* Field: [14:13] STANDBY_PW_SEL_INT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.STANDBY_PW_SEL both by boot FW while in safezone, and by flash
* device driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_RD_M               0x00006000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_RD_S                       13

/* Field:    [12] DIS_STANDBY_INT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.DIS_STANDBY both by boot FW while in safezone, and by flash device
* driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_RD_M                  0x00001000

/* Field:    [11] DIS_IDLE_INT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:CFG.DIS_IDLE both by boot FW while in safezone, and by flash device
* driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_RD_M                     0x00000800
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_RD_S                             11

/* Field:  [10:8] VIN_AT_X_INT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 0, this value will be written to
* FLASH:FSEQPMP.VIN_AT_X both by boot FW while in safezone, and by flash
* device driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_RD_M                     0x00000700
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_RD_S                              8

/* Field:     [7] STANDBY_MODE_SEL_EXT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.STANDBY_MODE_SEL both by boot FW while in safezone, and by flash
* device driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_RD_M             0x00000080
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_RD_S                      7

/* Field:   [6:5] STANDBY_PW_SEL_EXT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.STANDBY_PW_SEL both by boot FW while in safezone, and by flash
* device driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_RD_M               0x00000060
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_RD_S                        5

/* Field:     [4] DIS_STANDBY_EXT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.DIS_STANDBY both by boot FW while in safezone, and by flash device
* driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_RD_M                  0x00000010

/* Field:     [3] DIS_IDLE_EXT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:CFG.DIS_IDLE both by boot FW while in safezone, and by flash device
* driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_RD_M                     0x00000008
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_RD_S                              3

/* Field:   [2:0] VIN_AT_X_EXT_RD
*
* If AON_PMCTL:PWRCTL.EXT_REG_MODE = 1, this value will be written to
* FLASH:FSEQPMP.VIN_AT_X both by boot FW while in safezone, and by flash
* device driver FW after completion of a flash write operation. */
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_RD_M                     0x00000007
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_RD_S                              0

/******************************************************************************
*
* This section defines the register offsets of AON_PMCTL component
*
******************************************************************************/

/* Power Management Control */
#if defined(DEVICE_CC26X2)
/* Agama (CC26x2) specific definition */
#define AON_PMCTL_O_PWRCTL                                          0x00000010
#elif defined(DEVICE_CC26X0)
/* Chameleon (CC26x0) specific definition */
#define AON_PMCTL_O_PWRCTL                                          0x00000000
#endif

/* Field:     [1] EXT_REG_MODE
*
* Status of source for VDDRsupply:
*
* 0: DCDC or GLDO are generating VDDR
* 1: DCDC and GLDO are bypassed and an external regulator supplies VDDR */
#define AON_PMCTL_PWRCTL_EXT_REG_MODE                               0x00000002

#endif /* #ifndef OPENOCD_LOADERS_FLASH_CC26XX_HW_REGS_H */

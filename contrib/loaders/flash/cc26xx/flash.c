/******************************************************************************
*
* Copyright (C) 2016-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdint.h>
#include <stdbool.h>
#include "flash.h"

/******************************************************************************
*
* Defines for accesses to the security control in the customer configuration
* area in flash top sector.
*
******************************************************************************/
#define CCFG_OFFSET_SECURITY CCFG_O_BL_CONFIG
#define CCFG_SIZE_SECURITY   0x00000014

/******************************************************************************
*
* Default values for security control in customer configuration area in flash
* top sector.
*
******************************************************************************/
const uint8_t g_ccfg_default_sec[] = {
	0xFF, 0xFF, 0xFF, 0xC5,
	0xFF, 0xFF, 0xFF, 0xFF,
	0xC5, 0xFF, 0xFF, 0xFF,
	0xC5, 0xC5, 0xC5, 0xFF,
	0xC5, 0xC5, 0xC5, 0xFF
};

typedef uint32_t (*flash_prg_pntr_t) (uint8_t *, uint32_t, uint32_t);
typedef uint32_t (*flash_sector_erase_pntr_t) (uint32_t);

/******************************************************************************
*
* Function prototypes for static functions
*
******************************************************************************/
static void issue_fsm_command(flash_state_command_t command);
static void enable_sectors_for_write(void);
static uint32_t scale_cycle_values(uint32_t specified_timing,
	uint32_t scale_value);
static void set_write_mode(void);
static void trim_for_write(void);
static void set_read_mode(void);

/******************************************************************************
*
* Erase a flash sector
*
******************************************************************************/
uint32_t flash_sector_erase(uint32_t sector_address)
{
	uint32_t error_return;
	flash_sector_erase_pntr_t func_pntr;

	/* Call ROM function */
	func_pntr = (uint32_t (*)(uint32_t))(ROM_API_FLASH_TABLE[5]);
	error_return = func_pntr(sector_address);

	/* Enable standby because ROM function might have disabled it */
	HWREGBITW(FLASH_BASE + FLASH_O_CFG, FLASH_CFG_DIS_STANDBY_BITN) = 0;

	/* Return status of operation. */
	return error_return;
}

/******************************************************************************
*
* Erase all unprotected sectors in the flash main bank
*
******************************************************************************/
uint32_t flash_bank_erase(bool force_precondition)
{
	uint32_t error_return;
	uint32_t sector_address;
	uint32_t reg_val;

	/* Enable all sectors for erase. */
	enable_sectors_for_write();

	/* Clear the Status register. */
	issue_fsm_command(FAPI_CLEAR_STATUS);

	/* Enable erase of all sectors and enable precondition if required. */
	reg_val = HWREG(FLASH_BASE + FLASH_O_FSM_ST_MACHINE);
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_ENABLE;
	HWREG(FLASH_BASE + FLASH_O_FSM_SECTOR1) = 0x00000000;
	HWREG(FLASH_BASE + FLASH_O_FSM_SECTOR2) = 0x00000000;
	if (force_precondition)
		HWREG(FLASH_BASE + FLASH_O_FSM_ST_MACHINE) |=
			FLASH_FSM_ST_MACHINE_DO_PRECOND;
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_DISABLE;

	/* Issue the bank erase command to the FSM. */
	issue_fsm_command(FAPI_ERASE_BANK);

	/* Wait for erase to finish. */
	while (flash_check_fsm_for_ready() == FAPI_STATUS_FSM_BUSY)
		;

	/* Update status. */
	error_return = flash_check_fsm_for_error();

	/* Disable sectors for erase. */
	flash_disable_sectors_for_write();

	/* Set configured precondition mode since it may have been forced on. */
	if (!(reg_val & FLASH_FSM_ST_MACHINE_DO_PRECOND)) {
		HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_ENABLE;
		HWREG(FLASH_BASE + FLASH_O_FSM_ST_MACHINE) &=
			~FLASH_FSM_ST_MACHINE_DO_PRECOND;
		HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_DISABLE;
	}

	/* Program security data to default values in the customer configuration */
	/* area within the flash top sector if erase was successful. */
	if (error_return == FAPI_STATUS_SUCCESS) {
		sector_address = FLASHMEM_BASE + flash_size_get() -
							flash_sector_size_get();
		error_return = flash_program((uint8_t *)g_ccfg_default_sec,
							(sector_address + CCFG_OFFSET_SECURITY),
							CCFG_SIZE_SECURITY);
	}

	/* Return status of operation. */
	return error_return;
}

/******************************************************************************
*
* Programs unprotected main bank flash sectors
*
******************************************************************************/
uint32_t flash_program(uint8_t *data_buffer, uint32_t address, uint32_t count)
{
	uint32_t error_return;
	flash_prg_pntr_t func_pntr;

	/* Call ROM function */
	func_pntr = (uint32_t (*)(uint8_t *, uint32_t, uint32_t))
					(ROM_API_FLASH_TABLE[6]);
	error_return = func_pntr(data_buffer, address, count);

	/* Enable standby because ROM function might have disabled it */
	HWREGBITW(FLASH_BASE + FLASH_O_CFG, FLASH_CFG_DIS_STANDBY_BITN) = 0;

	/* Return status of operation. */
	return error_return;
}

/******************************************************************************
*
* Disables all sectors for erase and programming on the active bank
*
******************************************************************************/
void flash_disable_sectors_for_write(void)
{
	/* Configure flash back to read mode */
	set_read_mode();

	/* Disable Level 1 Protection. */
	HWREG(FLASH_BASE + FLASH_O_FBPROT) = FLASH_FBPROT_PROTL1DIS;

	/* Disable all sectors for erase and programming. */
	HWREG(FLASH_BASE + FLASH_O_FBSE) = 0x0000;

	/* Enable Level 1 Protection. */
	HWREG(FLASH_BASE + FLASH_O_FBPROT) = 0;

	/* Protect sectors from sector erase. */
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_ENABLE;
	HWREG(FLASH_BASE + FLASH_O_FSM_SECTOR1) = 0xFFFFFFFF;
	HWREG(FLASH_BASE + FLASH_O_FSM_SECTOR2) = 0xFFFFFFFF;
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_DISABLE;
}

/******************************************************************************
*
* Issues a command to the Flash State Machine.
*
******************************************************************************/
static void issue_fsm_command(flash_state_command_t command)
{
	/* Enable write to FSM register. */
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_ENABLE;

	/* Issue FSM command. */
	HWREG(FLASH_BASE + FLASH_O_FSM_CMD) = command;

	/* Start command execute. */
	HWREG(FLASH_BASE + FLASH_O_FSM_EXECUTE) = FLASH_CMD_EXEC;

	/* Disable write to FSM register. */
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_DISABLE;
}

/******************************************************************************
*
* Enables all sectors for erase and programming on the active bank.
*
* This function disables the idle reading power reduction mode, selects the
* flash bank and enables all sectors for erase and programming on the active
* bank.
* Sectors may be protected from programming depending on the value of the
* FLASH_O_FSM_BSLPx registers.
* Sectors may be protected from erase depending on the value of the
* FLASH_O_FSM_BSLEx registers. Additional sector erase protection is set by
* the FLASH_O_FSM_SECTOR1 register.
*
******************************************************************************/
static void enable_sectors_for_write(void)
{
	/* Trim flash module for program/erase operation. */
	trim_for_write();

	/* Configure flash to write mode */
	set_write_mode();

	/* Select flash bank. */
	HWREG(FLASH_BASE + FLASH_O_FMAC) = 0x00;

	/* Disable Level 1 Protection. */
	HWREG(FLASH_BASE + FLASH_O_FBPROT) = FLASH_FBPROT_PROTL1DIS;

	/* Enable all sectors for erase and programming. */
	HWREG(FLASH_BASE + FLASH_O_FBSE) = 0xFFFF;

	/* Enable Level 1 Protection */
	HWREG(FLASH_BASE + FLASH_O_FBPROT) = 0;
}

/******************************************************************************
*
* Trims the Flash Bank and Flash Pump for program/erase functionality
*
* This trimming will make it possible to perform erase and program operations
* of the flash. Trim values are loaded from factory configuration area
* (referred to as FCGF1). The trimming done by this function is valid until
* reset of the flash module.
*
* Some registers shall be written with a value that is a number of FCLK
* cycles. The trim values controlling these registers have a value of
* number of half us. FCLK = SysClk / ((RWAIT+1) x 2).
*
******************************************************************************/
static void trim_for_write(void)
{
	uint32_t value;
	uint32_t temp_val;
	uint32_t fclk_scale;
	uint32_t rwait;

	/* Return if flash is already trimmed for program/erase operations. */
	if (HWREG(FLASH_BASE + FLASH_O_FWFLAG) & FW_WRT_TRIMMED)
		return;

	/* Configure the FSM registers */

	/* Enable access to the FSM registers. */
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_ENABLE;

	/* Determine the scaling value to be used on timing related trim values. */
	/* The value is based on the flash module clock frequency and RWAIT */
	rwait = (HWREG(FLASH_BASE + FLASH_O_FRDCTL) &
				FLASH_FRDCTL_RWAIT_M) >> FLASH_FRDCTL_RWAIT_S;
	fclk_scale = (16 * FLASH_MODULE_CLK_FREQ) / (rwait + 1);

	/* Configure Program pulse width bits 15:0. */
	/* (FCFG1 offset 0x188 bits 15:0). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_PROG_EP) &
				FCFG1_FLASH_PROG_EP_PROGRAM_PW_M) >>
				FCFG1_FLASH_PROG_EP_PROGRAM_PW_S;

	value = scale_cycle_values(value, fclk_scale);

	HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PW) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PW) &
		~FLASH_FSM_PRG_PW_PROG_PUL_WIDTH_M) |
		((value << FLASH_FSM_PRG_PW_PROG_PUL_WIDTH_S) &
		FLASH_FSM_PRG_PW_PROG_PUL_WIDTH_M);

	/* Configure Erase pulse width bits 31:0. */
	/* (FCFG1 offset 0x18C bits 31:0). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_ERA_PW) &
				FCFG1_FLASH_ERA_PW_ERASE_PW_M) >>
				FCFG1_FLASH_ERA_PW_ERASE_PW_S;

	value = scale_cycle_values(value, fclk_scale);

	HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PW) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PW) &
		~FLASH_FSM_ERA_PW_FSM_ERA_PW_M) |
		((value << FLASH_FSM_ERA_PW_FSM_ERA_PW_S) &
		FLASH_FSM_ERA_PW_FSM_ERA_PW_M);

	/* Configure no of flash clock cycles from EXECUTEZ going low to the the
	   verify data can be read in the program verify mode bits 7:0. */
	/* (FCFG1 offset 0x174 bits 23:16). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_C_E_P_R) &
				FCFG1_FLASH_C_E_P_R_PV_ACCESS_M) >>
				FCFG1_FLASH_C_E_P_R_PV_ACCESS_S;

	value = scale_cycle_values(value, fclk_scale);

	HWREG(FLASH_BASE + FLASH_O_FSM_EX_VAL) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_EX_VAL) &
		~FLASH_FSM_EX_VAL_EXE_VALD_M) |
		((value << FLASH_FSM_EX_VAL_EXE_VALD_S) &
		FLASH_FSM_EX_VAL_EXE_VALD_M);

	/* Configure the number of flash clocks from the start of the Read mode at
	   the end of the operations until the FSM clears the BUSY bit in FMSTAT. */
	/* (FCFG1 offset 0x178 bits 23:16). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_P_R_PV) &
				FCFG1_FLASH_P_R_PV_RH_M) >>
				FCFG1_FLASH_P_R_PV_RH_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_RD_H) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_RD_H) &
		~FLASH_FSM_RD_H_RD_H_M) |
		((value << FLASH_FSM_RD_H_RD_H_S) &
		FLASH_FSM_RD_H_RD_H_M);

	/* Configure Program hold time */
	/* (FCFG1 offset 0x178 bits 31:24). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_P_R_PV) &
				FCFG1_FLASH_P_R_PV_PH_M) >>
				FCFG1_FLASH_P_R_PV_PH_S;

	value = scale_cycle_values(value, fclk_scale);

	HWREG(FLASH_BASE + FLASH_O_FSM_P_OH) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_P_OH) &
		~FLASH_FSM_P_OH_PGM_OH_M) |
		((value << FLASH_FSM_P_OH_PGM_OH_S) &
		FLASH_FSM_P_OH_PGM_OH_M);

	/* Configure Erase hold time */
	/* (FCFG1 offset 0x17C bits 31:24). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_EH_SEQ) &
				FCFG1_FLASH_EH_SEQ_EH_M) >>
				FCFG1_FLASH_EH_SEQ_EH_S;

	value = scale_cycle_values(value, fclk_scale);

	HWREG(FLASH_BASE + FLASH_O_FSM_ERA_OH) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_ERA_OH) &
		~FLASH_FSM_ERA_OH_ERA_OH_M) |
		((value << FLASH_FSM_ERA_OH_ERA_OH_S) &
		FLASH_FSM_ERA_OH_ERA_OH_M);

	/* Configure Program verify row switch time */
	/* (FCFG1 offset0x178 bits 15:8). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_P_R_PV) &
				FCFG1_FLASH_P_R_PV_PVH_M) >>
				FCFG1_FLASH_P_R_PV_PVH_S;

	value = scale_cycle_values(value, fclk_scale);

	HWREG(FLASH_BASE + FLASH_O_FSM_PE_VH) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PE_VH) &
		~FLASH_FSM_PE_VH_PGM_VH_M) |
		((value << FLASH_FSM_PE_VH_PGM_VH_S) &
		FLASH_FSM_PE_VH_PGM_VH_M);

	/* Configure Program Operation Setup time */
	/* (FCFG1 offset 0x170 bits 31:24). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_E_P) &
				FCFG1_FLASH_E_P_PSU_M) >>
				FCFG1_FLASH_E_P_PSU_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_PE_OSU) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PE_OSU) &
		~FLASH_FSM_PE_OSU_PGM_OSU_M) |
		((value << FLASH_FSM_PE_OSU_PGM_OSU_S) &
		FLASH_FSM_PE_OSU_PGM_OSU_M);

	/* Configure Erase Operation Setup time */
	/* (FCGF1 offset 0x170 bits 23:16). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_E_P) &
				FCFG1_FLASH_E_P_ESU_M) >>
				FCFG1_FLASH_E_P_ESU_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_PE_OSU) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PE_OSU) &
		~FLASH_FSM_PE_OSU_ERA_OSU_M) |
		((value << FLASH_FSM_PE_OSU_ERA_OSU_S) &
		FLASH_FSM_PE_OSU_ERA_OSU_M);

	/* Confgure Program Verify Setup time */
	/* (FCFG1 offset 0x170 bits 15:8). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_E_P) &
				FCFG1_FLASH_E_P_PVSU_M) >>
				FCFG1_FLASH_E_P_PVSU_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_PE_VSU) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PE_VSU) &
		~FLASH_FSM_PE_VSU_PGM_VSU_M) |
		((value << FLASH_FSM_PE_VSU_PGM_VSU_S) &
		FLASH_FSM_PE_VSU_PGM_VSU_M);

	/* Configure Erase Verify Setup time */
	/* (FCFG1 offset 0x170 bits 7:0). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_E_P) &
				FCFG1_FLASH_E_P_EVSU_M) >>
				FCFG1_FLASH_E_P_EVSU_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_PE_VSU) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PE_VSU) &
		~FLASH_FSM_PE_VSU_ERA_VSU_M) |
		((value << FLASH_FSM_PE_VSU_ERA_VSU_S) &
		FLASH_FSM_PE_VSU_ERA_VSU_M);

	/* Configure Addr to EXECUTEZ low setup time */
	/* (FCFG1 offset 0x174 bits 15:12). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_C_E_P_R) &
				FCFG1_FLASH_C_E_P_R_A_EXEZ_SETUP_M) >>
				FCFG1_FLASH_C_E_P_R_A_EXEZ_SETUP_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_CMP_VSU) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_CMP_VSU) &
		~FLASH_FSM_CMP_VSU_ADD_EXZ_M) |
		((value << FLASH_FSM_CMP_VSU_ADD_EXZ_S) &
		FLASH_FSM_CMP_VSU_ADD_EXZ_M);

	/* Configure Voltage Status Count */
	/* (FCFG1 offset 0x17C bits 15:12). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_EH_SEQ) &
				FCFG1_FLASH_EH_SEQ_VSTAT_M) >>
				FCFG1_FLASH_EH_SEQ_VSTAT_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_VSTAT) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_VSTAT) &
		~FLASH_FSM_VSTAT_VSTAT_CNT_M) |
		((value << FLASH_FSM_VSTAT_VSTAT_CNT_S) &
		FLASH_FSM_VSTAT_VSTAT_CNT_M);

	/* Configure Repeat Verify action setup */
	/* (FCFG1 offset 0x174 bits 31:24). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_C_E_P_R) &
				FCFG1_FLASH_C_E_P_R_RVSU_M) >>
				FCFG1_FLASH_C_E_P_R_RVSU_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_EX_VAL) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_EX_VAL) &
		~FLASH_FSM_EX_VAL_REP_VSU_M) |
		((value << FLASH_FSM_EX_VAL_REP_VSU_S) &
		FLASH_FSM_EX_VAL_REP_VSU_M);

	/* Configure Maximum Programming Pulses */
	/* (FCFG1 offset 0x184 bits 15:0). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_PP) &
				FCFG1_FLASH_PP_MAX_PP_M) >>
				FCFG1_FLASH_PP_MAX_PP_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PUL) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PUL) &
		~FLASH_FSM_PRG_PUL_MAX_PRG_PUL_M) |
		((value << FLASH_FSM_PRG_PUL_MAX_PRG_PUL_S) &
		FLASH_FSM_PRG_PUL_MAX_PRG_PUL_M);

	/* Configure Beginning level for VHVCT used during erase modes */
	/* (FCFG1 offset 0x180 bits 31:16). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_VHV_E) &
				FCFG1_FLASH_VHV_E_VHV_E_START_M) >>
				FCFG1_FLASH_VHV_E_VHV_E_START_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PUL) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PUL) &
		~FLASH_FSM_PRG_PUL_BEG_EC_LEVEL_M) |
		((value << FLASH_FSM_PRG_PUL_BEG_EC_LEVEL_S) &
		FLASH_FSM_PRG_PUL_BEG_EC_LEVEL_M);

	/* Configure Maximum EC Level */
	/* (FCFG1 offset 0x2B0 bits 21:18). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA3) &
				FCFG1_FLASH_OTP_DATA3_MAX_EC_LEVEL_M) >>
				FCFG1_FLASH_OTP_DATA3_MAX_EC_LEVEL_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PUL) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PUL) &
		~FLASH_FSM_ERA_PUL_MAX_EC_LEVEL_M) |
		((value << FLASH_FSM_ERA_PUL_MAX_EC_LEVEL_S) &
		FLASH_FSM_ERA_PUL_MAX_EC_LEVEL_M);

	/* Configure Maximum Erase Pulses */
	/* (FCFG1 offset 0x188 bits 31:16). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_PROG_EP) &
				FCFG1_FLASH_PROG_EP_MAX_EP_M) >>
				FCFG1_FLASH_PROG_EP_MAX_EP_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PUL) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PUL) &
		~FLASH_FSM_ERA_PUL_MAX_ERA_PUL_M) |
		((value << FLASH_FSM_ERA_PUL_MAX_ERA_PUL_S) &
		FLASH_FSM_ERA_PUL_MAX_ERA_PUL_M);

	/* Configure the VHVCT Step Size. This is the number of erase pulses that
	   must be completed for each level before the FSM increments the
	   CUR_EC_LEVEL to the next higher level. Actual erase pulses per level
	   equals (EC_STEP_SIZE +1). The stepping is only needed for the VHVCT
	   voltage. */
	/* (FCFG1 offset 0x2B0 bits 31:23). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA3) &
				FCFG1_FLASH_OTP_DATA3_EC_STEP_SIZE_M) >>
				FCFG1_FLASH_OTP_DATA3_EC_STEP_SIZE_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_STEP_SIZE) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_STEP_SIZE) &
		~FLASH_FSM_STEP_SIZE_EC_STEP_SIZE_M) |
		((value << FLASH_FSM_STEP_SIZE_EC_STEP_SIZE_S) &
		FLASH_FSM_STEP_SIZE_EC_STEP_SIZE_M);

	/* Configure the hight of each EC step. This is the number of counts that
	   the CUR_EC_LEVEL will increment when going to a new level. Actual count
	   size equals (EC_STEP_HEIGHT + 1). The stepping applies only to the VHVCT
	   voltage.
	   The read trim value is decremented by 1 before written to the register
	   since actual counts equals (register value + 1). */
	/* (FCFG1 offset 0x180 bits 15:0). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_VHV_E) &
				FCFG1_FLASH_VHV_E_VHV_E_STEP_HIGHT_M) >>
				FCFG1_FLASH_VHV_E_VHV_E_STEP_HIGHT_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_EC_STEP_HEIGHT) = ((value - 1) &
		FLASH_FSM_EC_STEP_HEIGHT_EC_STEP_HEIGHT_M);

	/* Configure Precondition used in erase operations */
	/* (FCFG1 offset 0x2B0 bit 22). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA3) &
				FCFG1_FLASH_OTP_DATA3_DO_PRECOND_M) >>
				FCFG1_FLASH_OTP_DATA3_DO_PRECOND_S;

	HWREG(FLASH_BASE + FLASH_O_FSM_ST_MACHINE) =
		(HWREG(FLASH_BASE + FLASH_O_FSM_ST_MACHINE) &
		~FLASH_FSM_ST_MACHINE_DO_PRECOND_M) |
		((value << FLASH_FSM_ST_MACHINE_DO_PRECOND_S) &
		FLASH_FSM_ST_MACHINE_DO_PRECOND_M);

	/* Enable the recommended Good Time function. */
	HWREG(FLASH_BASE + FLASH_O_FSM_ST_MACHINE) |=
		FLASH_FSM_ST_MACHINE_ONE_TIME_GOOD;

	/* Disable write access to FSM registers. */
	HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA) = FSM_REG_WRT_DISABLE;

	/* Configure the voltage registers */

	/* Unlock voltage registers (0x2080 - 0x2098). */
	HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0xAAAA;

	/* Configure voltage level for the specified pump voltage of high
	   voltage supply input during erase operation VHVCT_E and TRIM13_E */
	/* (FCFG1 offset 0x190 bits[3:0] and bits[11:8]). */
	temp_val = HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_VHV);

	value = ((temp_val & FCFG1_FLASH_VHV_TRIM13_E_M)>>
				FCFG1_FLASH_VHV_TRIM13_E_S) << FLASH_FVHVCT1_TRIM13_E_S;
	value |= ((temp_val & FCFG1_FLASH_VHV_VHV_E_M)>>
				FCFG1_FLASH_VHV_VHV_E_S) << FLASH_FVHVCT1_VHVCT_E_S;

	HWREG(FLASH_BASE + FLASH_O_FVHVCT1) = (HWREG(FLASH_BASE + FLASH_O_FVHVCT1) &
		~(FLASH_FVHVCT1_TRIM13_E_M | FLASH_FVHVCT1_VHVCT_E_M)) | value;

	/* Configure voltage level for the specified pump voltage of high voltage
	   supply input during program verify operation VHVCT_PV and TRIM13_PV */
	/* (OTP offset 0x194 bits[19:16] and bits[27:24]). */
	temp_val = HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_VHV_PV);

	value = ((temp_val & FCFG1_FLASH_VHV_PV_TRIM13_PV_M) >>
				FCFG1_FLASH_VHV_PV_TRIM13_PV_S) << FLASH_FVHVCT1_TRIM13_PV_S;
	value |= ((temp_val & FCFG1_FLASH_VHV_PV_VHV_PV_M) >>
				FCFG1_FLASH_VHV_PV_VHV_PV_S) << FLASH_FVHVCT1_VHVCT_PV_S;

	HWREG(FLASH_BASE + FLASH_O_FVHVCT1) = (HWREG(FLASH_BASE + FLASH_O_FVHVCT1) &
		~(FLASH_FVHVCT1_TRIM13_PV_M | FLASH_FVHVCT1_VHVCT_PV_M)) | value;

	/* Configure voltage level for the specified pump voltage of high voltage
	   supply input during program operation VHVCT_P and TRIM13_P */
	/* (FCFG1 offset 0x190 bits[19:16] and bits[27:24]). */
	temp_val = HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_VHV);

	value = ((temp_val & FCFG1_FLASH_VHV_TRIM13_P_M) >>
				FCFG1_FLASH_VHV_TRIM13_P_S) << FLASH_FVHVCT2_TRIM13_P_S;
	value |= ((temp_val & FCFG1_FLASH_VHV_VHV_P_M) >>
				FCFG1_FLASH_VHV_VHV_P_S) << FLASH_FVHVCT2_VHVCT_P_S;

	HWREG(FLASH_BASE + FLASH_O_FVHVCT2) = (HWREG(FLASH_BASE + FLASH_O_FVHVCT2) &
		~(FLASH_FVHVCT2_TRIM13_P_M | FLASH_FVHVCT2_VHVCT_P_M)) | value;

	/* Configure voltage level for the specified pump voltage of wordline power
	   supply for read mode */
	/* (FCFG1 offset 0x198 Bits 15:8). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_V) &
				FCFG1_FLASH_V_V_READ_M) >> FCFG1_FLASH_V_V_READ_S;

	HWREG(FLASH_BASE + FLASH_O_FVREADCT) =
		(HWREG(FLASH_BASE + FLASH_O_FVREADCT) &
		~FLASH_FVREADCT_VREADCT_M) |
		((value << FLASH_FVREADCT_VREADCT_S) &
		FLASH_FVREADCT_VREADCT_M);

	/* Configure the voltage level for the VCG 2.5 CT pump voltage */
	/* (FCFG1 offset 0x194 bits 15:8). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_VHV_PV) &
				FCFG1_FLASH_VHV_PV_VCG2P5_M) >>
				FCFG1_FLASH_VHV_PV_VCG2P5_S;

	HWREG(FLASH_BASE + FLASH_O_FVNVCT) =
		(HWREG(FLASH_BASE + FLASH_O_FVNVCT) &
		~FLASH_FVNVCT_VCG2P5CT_M) |
		((value << FLASH_FVNVCT_VCG2P5CT_S) &
		FLASH_FVNVCT_VCG2P5CT_M);

	/* Configure the voltage level for the specified pump voltage of high
	   current power input during program operation */
	/* (FCFG1 offset 0x198 bits 31:24). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_V) &
				FCFG1_FLASH_V_VSL_P_M) >>
				FCFG1_FLASH_V_VSL_P_S;

	HWREG(FLASH_BASE + FLASH_O_FVSLP) =
		(HWREG(FLASH_BASE + FLASH_O_FVSLP) &
		~FLASH_FVSLP_VSL_P_M) |
		((value << FLASH_FVSLP_VSL_P_S) &
		FLASH_FVSLP_VSL_P_M);

	/* Configure the voltage level for the specified pump voltage of wordline
	   power supply during programming operations */
	/* (OTP offset 0x198 bits 23:16). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_V) &
				FCFG1_FLASH_V_VWL_P_M) >>
				FCFG1_FLASH_V_VWL_P_S;

	HWREG(FLASH_BASE + FLASH_O_FVWLCT) =
		(HWREG(FLASH_BASE + FLASH_O_FVWLCT) &
		~FLASH_FVWLCT_VWLCT_P_M) |
		((value << FLASH_FVWLCT_VWLCT_P_S) &
		FLASH_FVWLCT_VWLCT_P_M);

	/* Configure the pump's TRIM_1P7 port pins. */
	/* (FCFG1 offset 0x2B0 bits 17:16). */
	value = (HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA3) &
				FCFG1_FLASH_OTP_DATA3_TRIM_1P7_M) >>
				FCFG1_FLASH_OTP_DATA3_TRIM_1P7_S;

	HWREG(FLASH_BASE + FLASH_O_FSEQPMP) =
		(HWREG(FLASH_BASE + FLASH_O_FSEQPMP) &
		~FLASH_FSEQPMP_TRIM_1P7_M) |
		((value << FLASH_FSEQPMP_TRIM_1P7_S) &
		FLASH_FSEQPMP_TRIM_1P7_M);

	/* Lock the voltage registers. */
	HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0x55AA;

	/* Set trimmed flag. */
	HWREG(FLASH_BASE + FLASH_O_FWLOCK) = 5;
	HWREG(FLASH_BASE + FLASH_O_FWFLAG) |= FW_WRT_TRIMMED;
	HWREG(FLASH_BASE + FLASH_O_FWLOCK) = 0;
}

/******************************************************************************
*
* Used to scale the TI OTP values based on the FClk scaling value.
*
******************************************************************************/
static uint32_t scale_cycle_values(uint32_t specified_timing,
	uint32_t scale_value)
{
	uint32_t scaled_value = (specified_timing * scale_value) >> 6;
	return scaled_value;
}

/******************************************************************************
*
* Used to set flash in read mode.
*
* Flash is configured with values loaded from OTP dependent on the current
* regulator mode.
*
******************************************************************************/
static void set_read_mode(void)
{
	uint32_t trim_value;
	uint32_t value;

	/* Configure the STANDBY_MODE_SEL, STANDBY_PW_SEL, DIS_STANDBY, DIS_IDLE,
	   VIN_AT_X and VIN_BY_PASS for read mode */
	if (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_PWRCTL) &
		AON_PMCTL_PWRCTL_EXT_REG_MODE) {

		/* Select trim values for external regulator mode:
		   Configure STANDBY_MODE_SEL (OTP offset 0x308 bit 7)
		   Configure STANDBY_PW_SEL   (OTP offset 0x308 bit 6:5)
		   Must be done while the register bit field CONFIG.DIS_STANDBY = 1 */
		HWREG(FLASH_BASE + FLASH_O_CFG) |= FLASH_CFG_DIS_STANDBY;

		trim_value =
			HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA4);

		value = ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_RD_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_RD_S) <<
					FLASH_CFG_STANDBY_MODE_SEL_S;

		value |= ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_RD_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_RD_S) <<
					FLASH_CFG_STANDBY_PW_SEL_S;

		/* Configure DIS_STANDBY (OTP offset 0x308 bit 4).
		   Configure DIS_IDLE    (OTP offset 0x308 bit 3). */
		value |= ((trim_value &
					(FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_RD_M |
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_RD_M)) >>
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_RD_S) <<
					FLASH_CFG_DIS_IDLE_S;

		HWREG(FLASH_BASE + FLASH_O_CFG) = (HWREG(FLASH_BASE + FLASH_O_CFG) &
			~(FLASH_CFG_STANDBY_MODE_SEL_M | FLASH_CFG_STANDBY_PW_SEL_M |
			FLASH_CFG_DIS_STANDBY_M | FLASH_CFG_DIS_IDLE_M)) | value;

		/* Check if sample and hold functionality is disabled. */
		if (HWREG(FLASH_BASE + FLASH_O_CFG) & FLASH_CFG_DIS_IDLE) {
			/* Wait for disabled sample and hold functionality to be stable. */
			while (!(HWREG(FLASH_BASE+FLASH_O_STAT) & FLASH_STAT_SAMHOLD_DIS))
				;
		}

		/* Configure VIN_AT_X (OTP offset 0x308 bits 2:0) */
		value = ((trim_value &
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_RD_M) >>
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_RD_S) <<
					FLASH_FSEQPMP_VIN_AT_X_S;

		/* Configure VIN_BY_PASS which is dependent on the VIN_AT_X value.
		   If VIN_AT_X = 7 then VIN_BY_PASS should be 0 otherwise
		   VIN_BY_PASS should be 1 */
		if (((value & FLASH_FSEQPMP_VIN_AT_X_M) >>
			FLASH_FSEQPMP_VIN_AT_X_S) != 0x7)
			value |= FLASH_FSEQPMP_VIN_BY_PASS;

		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0xAAAA;
		HWREG(FLASH_BASE + FLASH_O_FSEQPMP) =
			(HWREG(FLASH_BASE + FLASH_O_FSEQPMP) &
			~(FLASH_FSEQPMP_VIN_BY_PASS_M |
			FLASH_FSEQPMP_VIN_AT_X_M)) | value;
		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0x55AA;
	} else {

		/* Select trim values for internal regulator mode:
		   Configure STANDBY_MODE_SEL (OTP offset 0x308 bit 15)
		   COnfigure STANDBY_PW_SEL   (OTP offset 0x308 bit 14:13)
		   Must be done while the register bit field CONFIG.DIS_STANDBY = 1 */
		HWREG(FLASH_BASE + FLASH_O_CFG) |= FLASH_CFG_DIS_STANDBY;

		trim_value =
			HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA4);

		value = ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_RD_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_RD_S) <<
					FLASH_CFG_STANDBY_MODE_SEL_S;

		value |= ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_RD_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_RD_S) <<
					FLASH_CFG_STANDBY_PW_SEL_S;

		/* Configure DIS_STANDBY (OTP offset 0x308 bit 12).
		   Configure DIS_IDLE    (OTP offset 0x308 bit 11). */
		value |= ((trim_value &
					(FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_RD_M |
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_RD_M)) >>
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_RD_S) <<
					FLASH_CFG_DIS_IDLE_S;

		HWREG(FLASH_BASE + FLASH_O_CFG) = (HWREG(FLASH_BASE + FLASH_O_CFG) &
			~(FLASH_CFG_STANDBY_MODE_SEL_M | FLASH_CFG_STANDBY_PW_SEL_M |
			FLASH_CFG_DIS_STANDBY_M | FLASH_CFG_DIS_IDLE_M)) | value;

		/* Check if sample and hold functionality is disabled. */
		if (HWREG(FLASH_BASE + FLASH_O_CFG) & FLASH_CFG_DIS_IDLE) {
			/* Wait for disabled sample and hold functionality to be stable. */
			while (!(HWREG(FLASH_BASE + FLASH_O_STAT) & FLASH_STAT_SAMHOLD_DIS))
				;
		}

		/* Configure VIN_AT_X (OTP offset 0x308 bits 10:8) */
		value = (((trim_value &
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_RD_M) >>
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_RD_S) <<
					FLASH_FSEQPMP_VIN_AT_X_S);

		/* Configure VIN_BY_PASS which is dependent on the VIN_AT_X value.
		   If VIN_AT_X = 7 then VIN_BY_PASS should be 0 otherwise
		   VIN_BY_PASS should be 1 */
		if (((value & FLASH_FSEQPMP_VIN_AT_X_M) >>
			FLASH_FSEQPMP_VIN_AT_X_S) != 0x7)
			value |= FLASH_FSEQPMP_VIN_BY_PASS;

		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0xAAAA;
		HWREG(FLASH_BASE + FLASH_O_FSEQPMP) =
			(HWREG(FLASH_BASE + FLASH_O_FSEQPMP) &
			~(FLASH_FSEQPMP_VIN_BY_PASS_M |
			FLASH_FSEQPMP_VIN_AT_X_M)) | value;
		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0x55AA;
	}
}

/******************************************************************************
*
* Used to set flash in write mode.
*
* Flash is configured with values loaded from OTP dependent on the current
* regulator mode.
*
******************************************************************************/
static void set_write_mode(void)
{
	uint32_t trim_value;
	uint32_t value;

	/* Configure the STANDBY_MODE_SEL, STANDBY_PW_SEL, DIS_STANDBY, DIS_IDLE,
	   VIN_AT_X and VIN_BY_PASS for program/erase mode */
	if (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_PWRCTL) &
		AON_PMCTL_PWRCTL_EXT_REG_MODE) {

		/* Select trim values for external regulator mode:
		   Configure STANDBY_MODE_SEL (OTP offset 0x308 bit 23)
		   Configure STANDBY_PW_SEL   (OTP offset 0x308 bit 22:21)
		   Must be done while the register bit field CONFIG.DIS_STANDBY = 1 */
		HWREG(FLASH_BASE + FLASH_O_CFG) |= FLASH_CFG_DIS_STANDBY;

		trim_value =
			HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA4);

		value = ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_WRT_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_WRT_S) <<
					FLASH_CFG_STANDBY_MODE_SEL_S;

		value |= ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_WRT_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_WRT_S) <<
					FLASH_CFG_STANDBY_PW_SEL_S;

		/* Configure DIS_STANDBY (OTP offset 0x308 bit 20).
		   Configure DIS_IDLE    (OTP offset 0x308 bit 19). */
		value |= ((trim_value &
					(FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_WRT_M |
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_WRT_M)) >>
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_WRT_S) <<
					FLASH_CFG_DIS_IDLE_S;

		HWREG(FLASH_BASE + FLASH_O_CFG) = (HWREG(FLASH_BASE + FLASH_O_CFG) &
			~(FLASH_CFG_STANDBY_MODE_SEL_M | FLASH_CFG_STANDBY_PW_SEL_M |
			FLASH_CFG_DIS_STANDBY_M | FLASH_CFG_DIS_IDLE_M)) | value;

		/* Check if sample and hold functionality is disabled. */
		if (HWREG(FLASH_BASE + FLASH_O_CFG) & FLASH_CFG_DIS_IDLE) {
			/* Wait for disabled sample and hold functionality to be stable. */
			while (!(HWREG(FLASH_BASE + FLASH_O_STAT) & FLASH_STAT_SAMHOLD_DIS))
				;
		}

		/* Configure VIN_AT_X (OTP offset 0x308 bits 18:16) */
		value = ((trim_value &
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_WRT_M) >>
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_WRT_S) <<
					FLASH_FSEQPMP_VIN_AT_X_S;

		/* Configure VIN_BY_PASS which is dependent on the VIN_AT_X value.
		   If VIN_AT_X = 7 then VIN_BY_PASS should be 0 otherwise
		   VIN_BY_PASS should be 1 */
		if (((value & FLASH_FSEQPMP_VIN_AT_X_M) >>
			FLASH_FSEQPMP_VIN_AT_X_S) != 0x7)
			value |= FLASH_FSEQPMP_VIN_BY_PASS;

		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0xAAAA;
		HWREG(FLASH_BASE + FLASH_O_FSEQPMP) =
			(HWREG(FLASH_BASE + FLASH_O_FSEQPMP) &
			~(FLASH_FSEQPMP_VIN_BY_PASS_M |
			FLASH_FSEQPMP_VIN_AT_X_M)) | value;
		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0x55AA;
	} else {
		/* Select trim values for internal regulator mode:
		   Configure STANDBY_MODE_SEL (OTP offset 0x308 bit 31)
		   COnfigure STANDBY_PW_SEL   (OTP offset 0x308 bit 30:29)
		   Must be done while the register bit field CONFIG.DIS_STANDBY = 1 */
		HWREG(FLASH_BASE + FLASH_O_CFG) |= FLASH_CFG_DIS_STANDBY;

		trim_value =
			HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_FLASH_OTP_DATA4);

		value = ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_WRT_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_WRT_S) <<
					FLASH_CFG_STANDBY_MODE_SEL_S;

		value |= ((trim_value &
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_WRT_M) >>
					FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_WRT_S) <<
					FLASH_CFG_STANDBY_PW_SEL_S;

		/* Configure DIS_STANDBY (OTP offset 0x308 bit 28).
		   Configure DIS_IDLE    (OTP offset 0x308 bit 27). */
		value |= ((trim_value &
					(FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_WRT_M |
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_WRT_M)) >>
					FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_WRT_S) <<
					FLASH_CFG_DIS_IDLE_S;

		HWREG(FLASH_BASE + FLASH_O_CFG) = (HWREG(FLASH_BASE + FLASH_O_CFG) &
			~(FLASH_CFG_STANDBY_MODE_SEL_M | FLASH_CFG_STANDBY_PW_SEL_M |
			FLASH_CFG_DIS_STANDBY_M | FLASH_CFG_DIS_IDLE_M)) | value;

		/* Check if sample and hold functionality is disabled. */
		if (HWREG(FLASH_BASE + FLASH_O_CFG) & FLASH_CFG_DIS_IDLE) {
			/* Wait for disabled sample and hold functionality to be stable. */
			while (!(HWREG(FLASH_BASE + FLASH_O_STAT) & FLASH_STAT_SAMHOLD_DIS))
				;
		}

		/* Configure VIN_AT_X (OTP offset 0x308 bits 26:24) */
		value = ((trim_value &
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_WRT_M) >>
					FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_WRT_S) <<
					FLASH_FSEQPMP_VIN_AT_X_S;

		/* Configure VIN_BY_PASS which is dependent on the VIN_AT_X value.
		   If VIN_AT_X = 7 then VIN_BY_PASS should be 0 otherwise
		   VIN_BY_PASS should be 1 */
		if (((value & FLASH_FSEQPMP_VIN_AT_X_M) >>
			FLASH_FSEQPMP_VIN_AT_X_S) != 0x7)
			value |= FLASH_FSEQPMP_VIN_BY_PASS;

		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0xAAAA;
		HWREG(FLASH_BASE + FLASH_O_FSEQPMP) =
			(HWREG(FLASH_BASE + FLASH_O_FSEQPMP) &
			~(FLASH_FSEQPMP_VIN_BY_PASS_M |
			FLASH_FSEQPMP_VIN_AT_X_M)) | value;
		HWREG(FLASH_BASE + FLASH_O_FLOCK) = 0x55AA;
	}
}

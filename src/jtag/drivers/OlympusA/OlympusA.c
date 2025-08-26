// SPDX-License-Identifier: GPL-2.0-or-later

/*
Copyright (C) 2025 Marvell
Simplified version of JTAG Control build in to Zeus in the Olympus Driver
Executing SVF commands
Initially followed Based on usb bluster source code.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

//Enable for debugging:
//#define MM_LOG_USER
//#define MM_WINDOWS_MESSAGE_BOX

// project specific includes
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include <helper/replacements.h>
#include "OlympusA.h"

// system includes
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

//#include <chain.h>

#ifndef _WIN32
# include <unistd.h>
#else
# include <windows.h>
#endif

#include "InphiOlympus.h"
#include "OlympusErrors.h"
#include "Zeus-1Interface.h"
#include "GaiaInterface.h"

#define SUPPORTED_OLYMPUS_DLL		((2025 << 16) | 5) //2025.5
#define JTAG_BACK_TO_IDLE

typedef struct {
	HANDLE lDeviceHandle;
	DWORD MbID;
	DWORD DbID;
	DWORD ChipID;
	DWORD SerialNumber;
	DWORD PortNumber;	//For Apollo

	LONG Update;

	LONG JTAG_DIS_GPIO;
	LONG JTAG_DIS_GPIO_PORT;
	LONG TRST_N_GPIO;
	LONG TRST_N_GPIO_PORT;
	LONG TMS_GPIO;
	LONG TDI_GPIO;
	LONG TDO_GPIO;
	LONG PHYADR_GPIO[4];
	LONG GPIO_PORT;
	LONG TCK_InterfaceNumber;

}olympusajtag_controller_t;

olympusajtag_controller_t olympusajtag_controller;

static int olympusa_init(void)
{
	DWORD Error;
	CHAR ErrorText[MAX_PATH];
	WCHAR wc_Name[MAX_PATH];

	HANDLE DeviceHandle[MAX_MNT_NUMBER * MAX_DEVICE_TYPE_NUMBER];
	DWORD TotalDeviceNumber = 0;
	DWORD SerialNumber[MAX_MNT_NUMBER * MAX_DEVICE_TYPE_NUMBER];
	DWORD MbID[MAX_MNT_NUMBER * MAX_DEVICE_TYPE_NUMBER];
	DWORD DbID[MAX_MNT_NUMBER * MAX_DEVICE_TYPE_NUMBER];
	DWORD ChipID[MAX_MNT_NUMBER * MAX_DEVICE_TYPE_NUMBER];
	BOOL Update[MAX_MNT_NUMBER * MAX_DEVICE_TYPE_NUMBER];

	for(int i = 0; i < MAX_MNT_NUMBER * MAX_DEVICE_TYPE_NUMBER; i++)
		{
		Error = MNT_OpenDevice(MNT_ALL_DEVICES_ID, i, &DeviceHandle[i]);
		if(Error != OLYMPUS_SUCCESS && Error != OLYMPUS_ALREADY_OPEN)
			{
			TotalDeviceNumber = i;
			break;
			}
		}

	if(TotalDeviceNumber == 0)
		{
		LOG_ERROR("OlympusA: Device is not found. Please check Power and Cable.");
		#ifdef MM_WINDOWS_MESSAGE_BOX
			MessageBox(NULL, "Could not find any Zeus boards", "Olympus Error", MB_OK | MB_ICONERROR);
		#endif
		return ERROR_JTAG_DEVICE_ERROR;
		}

	//Let's check Version
	OlympusVersion Version;
	Version.Type = User;
	Error = MNT_GetVersions(DeviceHandle[0], 1, &Version, NULL);
	if(Error != OLYMPUS_SUCCESS)
		{
		LOG_ERROR("OlympusA: MNT_GetVersions Error: 0x%lX", Error);
		return ERROR_JTAG_DEVICE_ERROR;
		}

	if(((Version.Year << 16) | Version.MajorN) < SUPPORTED_OLYMPUS_DLL)
		{
		sprintf_s(ErrorText, MAX_PATH, "OlympusA: InphiOlympus DLL version %ld.%ld is not supported", Version.Year, Version.MajorN);
		LOG_ERROR("%s", ErrorText);
		#ifdef MM_WINDOWS_MESSAGE_BOX
			MessageBox(NULL, ErrorText, "Olympus Error", MB_OK | MB_ICONERROR);
		#endif
		for(DWORD i = 0; i < TotalDeviceNumber; i++)
			MNT_CloseDevice(DeviceHandle[i]);
		return ERROR_JTAG_DEVICE_ERROR;
		}

	for(DWORD i = 0; i < TotalDeviceNumber; i++)
		{
		OlympusDeviceState DeviceState;
		ZeroMemory(&DeviceState, sizeof(OlympusDeviceState));
		DeviceState.Size = sizeof(OlympusDeviceState);
		Error = MNT_GetDeviceState(DeviceHandle[i], &DeviceState);
		//Check if it is running. If so, do not reinitialize
		if(DeviceState.PowerBIAS == 0.)	//Change to RunningMode later
			Update[i] = 1;
		else
			Update[i] = 0;

		OlympusHWFunction lConfig;
		ZeroMemory(&lConfig, sizeof(OlympusHWFunction));
		lConfig.Size = sizeof(OlympusHWFunction);
		lConfig.Mode = MNT_SKIP_ATHENA_POWER;	// 5;
		Error = MNT_LoadDevice(DeviceHandle[i], &lConfig);
		if(Error != OLYMPUS_SUCCESS)
			{
			LOG_ERROR("OlympusA: MNT_LoadDevice Error: 0x%lX", Error);
			return ERROR_JTAG_DEVICE_ERROR;
			}

		//Check if it is running. If so, do not reinitialize
		if(Update[i])
			{
			Error = MNT_ConfigDevice(DeviceHandle[i], NULL);	// OlympusDeviceConfig* DeviceConfig)
			if(Error != OLYMPUS_SUCCESS)
				{
				LOG_ERROR("OlympusA: MNT_ConfigDevice Error: 0x%lX", Error);
				return ERROR_JTAG_DEVICE_ERROR;
				}

			//Enable Power from EEPROM Settings. Change it later
			Error = MNT_ControlSequence(DeviceHandle[i], 1, NULL);	// ControlSequences);
			if(Error != OLYMPUS_SUCCESS)
				{
				LOG_ERROR("OlympusA: MNT_ControlSequence Error: 0x%lX", Error);
				return ERROR_JTAG_DEVICE_ERROR;
				}
			}

		OlympusDeviceInfo DeviceInfo;
		ZeroMemory(&DeviceInfo, sizeof(OlympusDeviceInfo));
		DeviceInfo.Size = sizeof(OlympusDeviceInfo);
		Error = MNT_GetDeviceInfo(DeviceHandle[i], &DeviceInfo);
		if(Error != OLYMPUS_SUCCESS)
			{
			LOG_ERROR("OlympusA: MNT_GetDeviceInfo Error: 0x%lX", Error);
			return ERROR_JTAG_DEVICE_ERROR;
			}

		SerialNumber[i] = DeviceInfo.MBSerialNumber;
		MbID[i] = DeviceInfo.MBDeviceType;
		DbID[i] = DeviceInfo.DBDeviceType[0];
		ChipID[i] = DeviceInfo.DBChipIDs[0] & 0xFF;
		}

	DWORD FindFlag = 0;
	//Now Analyze SerialNumber
	olympusajtag_controller_t *ctrl = &olympusajtag_controller;

	if(TotalDeviceNumber == 1)
		{
		ctrl->lDeviceHandle = DeviceHandle[0];
		ctrl->SerialNumber = SerialNumber[0];
		ctrl->MbID = MbID[0];
		ctrl->DbID = DbID[0];
		ctrl->ChipID = ChipID[0];
		ctrl->Update = Update[0];
		FindFlag = 1;
		}
	else if(TotalDeviceNumber > 1)
		{
		for(DWORD i = 0; i < TotalDeviceNumber; i++)
			{
			if(ctrl->SerialNumber < MAX_MNT_NUMBER)
				{
				if(ctrl->SerialNumber == i)
					{
					ctrl->lDeviceHandle = DeviceHandle[i];
					ctrl->SerialNumber = SerialNumber[i];
					ctrl->MbID = MbID[i];
					ctrl->DbID = DbID[i];
					ctrl->ChipID = ChipID[i];
					ctrl->Update = Update[i];
					FindFlag = 1;
					}
				}
			else if(ctrl->SerialNumber == SerialNumber[i])
				{
				ctrl->lDeviceHandle = DeviceHandle[i];
				ctrl->MbID = MbID[i];
				ctrl->DbID = DbID[i];
				ctrl->ChipID = ChipID[i];
				ctrl->Update = Update[i];
				FindFlag = 1;
				}
			else
				{
				MNT_CloseDevice(DeviceHandle[i]);
				}
			}
		}
	else
		{
		LOG_ERROR("OlympusA: TotalDeviceNumber Software Error");
		return ERROR_JTAG_DEVICE_ERROR;
		}

	if(FindFlag == 0)
		{
		sprintf_s(ErrorText, MAX_PATH, "OlympusA: Could not find specified Zeus S/N: %ld", ctrl->SerialNumber);
		LOG_ERROR("%s", ErrorText);
		#ifdef MM_WINDOWS_MESSAGE_BOX
			MessageBox(NULL, ErrorText, "Olympus Error", MB_OK | MB_ICONERROR);
		#endif
		return ERROR_JTAG_DEVICE_ERROR;
		}

	//Find our GPIOs: TRST_N, TCK, TMS, TDI, TDO
	ctrl->JTAG_DIS_GPIO = -1;
	ctrl->JTAG_DIS_GPIO_PORT = 0;	//May be different port?
	ctrl->TRST_N_GPIO = -1;
	ctrl->TRST_N_GPIO_PORT = 0;
	ctrl->TMS_GPIO = -1;
	ctrl->TDI_GPIO = -1;
	ctrl->TDO_GPIO = -1;

	ctrl->GPIO_PORT = 0;
	ctrl->TCK_InterfaceNumber = 2;

	//Search GPIO Locations
	for(DWORD i = 0; i < 32; i++)
		{
		Error = MNT_GetOneGPIOInfo(ctrl->lDeviceHandle, 0, i, NULL, NULL, wc_Name);
		if(Error != OLYMPUS_SUCCESS)
			{
			LOG_ERROR("OlympusA: MNT_GetOneGPIOInfo Error: 0x%lX", Error);
			return ERROR_JTAG_DEVICE_ERROR;
			}
		else if(_wcsicmp(wc_Name, L"TRST_N") == 0)
			ctrl->TRST_N_GPIO = i;
		else if(_wcsicmp(wc_Name, L"JTAG_DIS") == 0)
			ctrl->JTAG_DIS_GPIO = i;
		else if(_wcsicmp(wc_Name, L"JTAGFPGA") == 0)
			ctrl->JTAG_DIS_GPIO = i;
		else if(_wcsicmp(wc_Name, L"TMS") == 0)
			ctrl->TMS_GPIO = i;
		else if(_wcsicmp(wc_Name, L"TDI") == 0)
			ctrl->TDI_GPIO = i;
		else if(_wcsicmp(wc_Name, L"TDO") == 0)
			ctrl->TDO_GPIO = i;
		else if(_wcsicmp(wc_Name, L"PHYADR0") == 0)
			ctrl->PHYADR_GPIO[0] = i;
		else if(_wcsicmp(wc_Name, L"PHYADR1") == 0)
			ctrl->PHYADR_GPIO[1] = i;
		else if(_wcsicmp(wc_Name, L"PHYADR2") == 0)
			ctrl->PHYADR_GPIO[2] = i;
		else if(_wcsicmp(wc_Name, L"PHYADR3") == 0)
			ctrl->PHYADR_GPIO[3] = i;
		}

	if(ctrl->JTAG_DIS_GPIO < 0)
		{
		for(DWORD i = 0; i < 8; i++)
			{
			Error = MNT_GetOneGPIOInfo(ctrl->lDeviceHandle, 1, i, NULL, NULL, wc_Name);
			if(Error != OLYMPUS_SUCCESS)
				{
				LOG_ERROR("OlympusA: MNT_GetOneGPIOInfo Error: 0x%lX", Error);
				return ERROR_JTAG_DEVICE_ERROR;
				}
			else if(_wcsicmp(wc_Name, L"JTAG_DIS") == 0)
				{
				ctrl->JTAG_DIS_GPIO = i;
				ctrl->JTAG_DIS_GPIO_PORT = 1;
				}
			}
		}

	if(ctrl->TRST_N_GPIO < 0)
		{
		for(DWORD i = 0; i < 8; i++)
			{
			Error = MNT_GetOneGPIOInfo(ctrl->lDeviceHandle, 1, i, NULL, NULL, wc_Name);
			if(Error != OLYMPUS_SUCCESS)
				{
				LOG_ERROR("OlympusA: MNT_GetOneGPIOInfo Error: 0x%lX", Error);
				return ERROR_JTAG_DEVICE_ERROR;
				}
			else if(_wcsicmp(wc_Name, L"TRST_N") == 0)
				{
				ctrl->TRST_N_GPIO = i;
				ctrl->TRST_N_GPIO_PORT = 1;
				}
			}
		}

	#ifdef MM_LOG_USER
		LOG_USER("M: JTAG PORT: %ld, JTAG Bit: %ld", ctrl->JTAG_DIS_GPIO_PORT, ctrl->JTAG_DIS_GPIO);
	#endif

	Error = MNT_SetOneGPIODirection(ctrl->lDeviceHandle, ctrl->JTAG_DIS_GPIO_PORT, ctrl->JTAG_DIS_GPIO, 1);
	Error = MNT_SetOneGPIO(ctrl->lDeviceHandle, ctrl->JTAG_DIS_GPIO_PORT, ctrl->JTAG_DIS_GPIO, 1);

	Error = MNT_SetOneGPIODirection(ctrl->lDeviceHandle, ctrl->GPIO_PORT, ctrl->TMS_GPIO, 1);
	Error = MNT_SetOneGPIODirection(ctrl->lDeviceHandle, ctrl->GPIO_PORT, ctrl->TDI_GPIO, 1);
	Error = MNT_SetOneGPIODirection(ctrl->lDeviceHandle, ctrl->GPIO_PORT, ctrl->TDO_GPIO, 0);

	Error = MNT_SetOneGPIO(ctrl->lDeviceHandle, ctrl->TRST_N_GPIO_PORT, ctrl->TRST_N_GPIO, 1);
	Error = MNT_SetOneGPIODirection(ctrl->lDeviceHandle, ctrl->TRST_N_GPIO_PORT, ctrl->TRST_N_GPIO, 1);

	OlympusInterfaceInfo InterfaceInfo;
	InterfaceInfo.Size = sizeof(OlympusInterfaceInfo);
	InterfaceInfo.InterfaceNumber = 2;	//Update for Apollo. Add new parameter
	InterfaceInfo.ClkMhz = 13.;

	InterfaceInfo.InterfaceProtocol = SIF_JTAGSVF;	//14;
	InterfaceInfo.InterfaceClockType = SIF_CLK_NORM;

	Error = MNT_ConfigureInterface(ctrl->lDeviceHandle, 1, &InterfaceInfo);
	if(Error != OLYMPUS_SUCCESS)
		{
		LOG_ERROR("OlympusA: MNT_ConfigureInterface Error: 0x%lX", Error);
		return ERROR_JTAG_DEVICE_ERROR;
		}

	if(ctrl->Update)
		sprintf_s(ErrorText, MAX_PATH, "JTAG is connected to Zeus S/N: %ld. Board was initialized with setting from configuration EEPROM", ctrl->SerialNumber);
	else
		sprintf_s(ErrorText, MAX_PATH, "JTAG is connected to Zeus S/N: %ld.", ctrl->SerialNumber);

	#ifdef MM_LOG_USER
		LOG_USER("M: %s", ErrorText);
	#endif

	return ERROR_OK;
}

static int olympusa_quit(void)
{
	//STUB. Temp
	MNT_CloseDevice(olympusajtag_controller.lDeviceHandle);

	return ERROR_OK;
}

// toggle TRST_N (or SET?). Use SVF Command
static int olympusa_reset(int trst, int srst)
{
	DWORD Error;
	uint32_t SVFCommand;
	DWORD dwTemp;

	//Bit [7:6] - Control TRST_N: 01 - Set 1, 10 - Set 0, 11 - TOGGLE
	if(trst == 1)
		SVFCommand = 0x80;
	else
		SVFCommand = 0x40;

	//OVERWITE: DO TOGGLE
	//SVFCommand = 0xC0;
	Error = MNT_Write(	olympusajtag_controller.lDeviceHandle,
						2,
						SVFCommand,
						1,
						&dwTemp);

	if(Error != OLYMPUS_SUCCESS)
		{
		LOG_ERROR("OlympusA: MNT_Write Error: 0x%lX", Error);
		return ERROR_JTAG_DEVICE_ERROR;
		}

	//Actual TOGGLE
	if(trst == 1)
		MNT_SetOneGPIO(olympusajtag_controller.lDeviceHandle, olympusajtag_controller.TRST_N_GPIO_PORT, olympusajtag_controller.TRST_N_GPIO, 0);
	else
		MNT_SetOneGPIO(olympusajtag_controller.lDeviceHandle, olympusajtag_controller.TRST_N_GPIO_PORT, olympusajtag_controller.TRST_N_GPIO, 1);

	#ifdef MM_LOG_USER
		LOG_USER("M: OlympusA: Set TRST_N: %d", trst);
	#endif
	LOG_DEBUG("OlympusA: Set TRST_N: %d", trst);

	return ERROR_OK;
}

static int olympusa_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd;
	int bit_count;
	int scan_buffer_length;
	enum scan_type type;
	int jtag_error = ERROR_OK;
	uint8_t *buffer;
	DWORD Error = 0;
	uint32_t SVFCommand;
	DWORD dwTemp;

	for (cmd = cmd_queue; jtag_error == ERROR_OK && cmd; cmd = cmd->next)
		{
		switch (cmd->type)
			{
			case JTAG_RESET:
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_RESET: %d - %d", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				#endif
				olympusa_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
				//RUNTEST. Expect to move states
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_RUNTEST %i cycles, end in %s", cmd->cmd.runtest->num_cycles, tap_state_name(cmd->cmd.runtest->end_state));
				#endif
				LOG_DEBUG_IO("runtest %i cycles, end in %s", cmd->cmd.runtest->num_cycles, tap_state_name(cmd->cmd.runtest->end_state));

				if(cmd->cmd.runtest->end_state == TAP_IRPAUSE)
					SVFCommand = 0x28;
				else if(cmd->cmd.runtest->end_state == TAP_DRPAUSE)
					SVFCommand = 0x18;
				else if(cmd->cmd.runtest->end_state == TAP_RESET)
					SVFCommand = 0x38;
				else if(cmd->cmd.runtest->end_state == TAP_IDLE)
					SVFCommand = 0x08;
				else
					{
					#ifdef MM_LOG_USER
						LOG_USER("M: INVALID STATE %s", tap_state_name(cmd->cmd.runtest->end_state));
					#endif
					break;
					}

				SVFCommand = 0x4 | (cmd->cmd.runtest->num_cycles << 16);
				Error = MNT_Write(	olympusajtag_controller.lDeviceHandle,
									2,
									SVFCommand,
									1,
									&dwTemp);
				if(Error != OLYMPUS_SUCCESS)
					{
					LOG_ERROR("OlympusA: MNT_Write Error: 0x%lX", Error);
					jtag_error = ERROR_JTAG_DEVICE_ERROR;
					}
				break;
			case JTAG_STABLECLOCKS:
				//STABLECLOCKS. For now same as RUNTEST. Do not Move states
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_STABLECLOCKS num_cycles: %d", cmd->cmd.stableclocks->num_cycles);
				#endif
				SVFCommand = 0x4 | (cmd->cmd.runtest->num_cycles << 16);
				Error = MNT_Write(	olympusajtag_controller.lDeviceHandle,
									2,
									SVFCommand,
									1,
									&dwTemp);
				if(Error != OLYMPUS_SUCCESS)
					{
					LOG_ERROR("OlympusA: MNT_Write Error: 0x%lX", Error);
					jtag_error = ERROR_JTAG_DEVICE_ERROR;
					}
				break;
			case JTAG_TLR_RESET:
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_TLR_RESET  statemove end in %s", tap_state_name(cmd->cmd.statemove->end_state));
				#endif
				//GOTO Reset State
				SVFCommand = 0x8 | 0x30;
				Error = MNT_Write(	olympusajtag_controller.lDeviceHandle,
									2,
									SVFCommand,
									1,
									&dwTemp);
				if(Error != OLYMPUS_SUCCESS)
					{
					LOG_ERROR("OlympusA: MNT_Write Error: 0x%lX", Error);
					jtag_error = ERROR_JTAG_DEVICE_ERROR;
					}
				break;
			case JTAG_PATHMOVE:
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_PATHMOVE: %i states, end in %s", cmd->cmd.pathmove->num_states,
																	tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
					for(unsigned int i = 0; i < cmd->cmd.pathmove->num_states; i++)
						{
						LOG_USER("M:         STATE %i - %s", cmd->cmd.pathmove->num_states,
														tap_state_name(cmd->cmd.pathmove->path[i]));
						}
				#endif
				//GOTO 00 - IDLE, 01 - DRPAUSE, 10 - IRPAUSE, 11 - RESET

				SVFCommand = 0;
				if(cmd->cmd.statemove->end_state == TAP_IRPAUSE)
					SVFCommand = 0x28;
				else if(cmd->cmd.statemove->end_state == TAP_DRPAUSE)
					SVFCommand = 0x18;
				else if(cmd->cmd.statemove->end_state == TAP_RESET)
					SVFCommand = 0x38;
				else if(cmd->cmd.statemove->end_state == TAP_IDLE)
					SVFCommand = 0x08;
				else
					{
					#ifdef MM_LOG_USER
						LOG_USER("M: INVALID STATE %s", tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
					#endif
					break;
					}

				Error = MNT_Write(	olympusajtag_controller.lDeviceHandle,
									2,
									SVFCommand,
									1,
									&dwTemp);
				if(Error != OLYMPUS_SUCCESS)
					{
					LOG_ERROR("OlympusA: MNT_Write Error: 0x%lX", Error);
					jtag_error = ERROR_JTAG_DEVICE_ERROR;
					}
				break;
			case JTAG_TMS:
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_TMS !!! NOT IMPLEMENTED !!! num_bits: %d", cmd->cmd.tms->num_bits);
				#endif
				LOG_ERROR("OlympusA: JTAG_TMS !!! NOT IMPLEMENTED !!! num_bits: %d", cmd->cmd.tms->num_bits);
				jtag_error = ERROR_FAIL;
				break;
			case JTAG_SLEEP:
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_SLEEP: %d us", cmd->cmd.sleep->us);
				#endif
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			case JTAG_SCAN:
				#ifdef MM_LOG_USER
					LOG_USER("M: JTAG_SCAN - %s scan end in %s", (cmd->cmd.scan->ir_scan) ? "IR" : "DR", tap_state_name(cmd->cmd.scan->end_state));
				#endif
				LOG_DEBUG_IO("%s scan end in %s",  (cmd->cmd.scan->ir_scan) ? "IR" : "DR", tap_state_name(cmd->cmd.scan->end_state));

				bit_count = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				//syncbb_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if((cmd->cmd.scan->ir_scan) == 0)
					SVFCommand = 0x1;
				else
					SVFCommand = 0x2;

				//Validate END State. If IR_SCAN -> IRSHIFT (DO NOT GO TO DRSHIFT)
				/*
				if (!((!ir_scan && (tap_get_state() == TAP_DRSHIFT)) || (ir_scan && (tap_get_state() == TAP_IRSHIFT))))
				*/

				// STOP AT SPECIFIED STATE:

				if(cmd->cmd.scan->end_state == TAP_IRPAUSE)
					SVFCommand |= 0x28;
				else if(cmd->cmd.scan->end_state == TAP_DRPAUSE)
					SVFCommand |= 0x18;
				else if(cmd->cmd.scan->end_state == TAP_RESET)
					SVFCommand |= 0x38;
				else if(cmd->cmd.scan->end_state == TAP_IDLE)
					;
				else
					LOG_USER("OlympusA: INVALID STATE %s", tap_state_name(cmd->cmd.scan->end_state));

				scan_buffer_length = DIV_ROUND_UP(bit_count, 32);

				SVFCommand |= (bit_count << 16);

				#ifdef MM_LOG_USER
					LOG_USER("M: ... SVFCommand 0x%X; bit_count %d; Scan Type: %d; Data 0: 0x%lX", SVFCommand, bit_count, type, (DWORD)buffer[0]);
				#endif

				//if(type == SCAN_OUT)
				Error = MNT_Write(	olympusajtag_controller.lDeviceHandle,
									2,
									SVFCommand,
									scan_buffer_length,			//Size in DWORDs
									(DWORD*)buffer);

				if(Error != OLYMPUS_SUCCESS)
					{
					LOG_ERROR("OlympusA:  MNT_Write Error: 0x%lX", Error);
					jtag_error = ERROR_JTAG_DEVICE_ERROR;
					}

				if(type == SCAN_IN || type == SCAN_IO)
					{
					#ifdef MM_LOG_USER
						LOG_USER("M: ... Start Read scan_buffer_length: %d", scan_buffer_length);
					#endif

					Error = MNT_Read(	olympusajtag_controller.lDeviceHandle,
										0x100,
										ZEUS_PACKET_READ_MEM_ADDR,
										scan_buffer_length,
										(DWORD*)buffer);

					if(Error != OLYMPUS_SUCCESS)
						{
						LOG_ERROR("OlympusA: MNT_Read Error: 0x%lX", Error);
						jtag_error = ERROR_JTAG_DEVICE_ERROR;
						}

					#ifdef MM_LOG_USER
						LOG_USER("M: ...0. 0x%lX ... 1. 0x%lX", (DWORD)buffer[0], (DWORD)buffer[4]);
					#endif
					}

				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					{
					#ifdef MM_LOG_USER
						LOG_USER("M: jtag_read_buffer failed");
					#endif
					jtag_error = ERROR_JTAG_QUEUE_FAILED;
					}
				free(buffer);
				#ifdef MM_LOG_USER
					LOG_USER("M: ... JTAG_SCAN DONE");
				#endif
				break;
			default:
				#ifdef MM_LOG_USER
					LOG_USER("M BUG: unknown JTAG command type 0x%X", cmd->type);
				#endif
				LOG_ERROR("BUG: unknown JTAG command type 0x%X", cmd->type);
				jtag_error = ERROR_FAIL;
			}
	}

	return jtag_error;
}

//Set Speed in kHz
static int olympusa_speed(int speed)
{
	DWORD Error;

	if (speed > 26000)
		{
#ifdef MM_LOG_USER
		LOG_USER("M: OlympusA: Maximum speed is 26 MHz (trying to set: %d)", speed);
#endif
		LOG_ERROR("OlympusA: Maximum speed is 26 MHz (trying to set: %d)", speed);
		return ERROR_FAIL;
		}

	OlympusInterfaceInfo InterfaceInfo;
	InterfaceInfo.Size = sizeof(OlympusInterfaceInfo);
	InterfaceInfo.InterfaceNumber = 2;

	InterfaceInfo.ClkMhz = ((double)speed + 0.5) / 1000.;

	InterfaceInfo.InterfaceProtocol = SIF_JTAGSVF;// SIF_MDIO;
	InterfaceInfo.InterfaceClockType = SIF_CLK_NORM;

	Error = MNT_ConfigureInterface(olympusajtag_controller.lDeviceHandle, 1, &InterfaceInfo);
	if(Error != OLYMPUS_SUCCESS)
		{
		LOG_ERROR("OlympusA: MNT_ConfigureInterface Error: 0x%lX", Error);
		return ERROR_JTAG_DEVICE_ERROR;
		}

#ifdef MM_LOG_USER
	LOG_USER("M: olympusa_speed: speed set %d -- %f (internal MHz)", speed, InterfaceInfo.ClkMhz);
#endif

	return ERROR_OK;
}

//Calculate what would be real JTAG SPEED based on KHZ
static int olympusa_khz(int khz, int *jtag_speed)
{
	if (khz == 0)
		{
		LOG_DEBUG("OlympusA: RCLK not supported");
		return ERROR_FAIL;
		}

	double ClkMhz;
	LONG divider;
	DWORD counter;

	ClkMhz = (double)khz / 1000;

	//#define MNT_CLOCK_MHz	(104.)
	if(ClkMhz > (MNT_CLOCK_MHz / 4.))
		ClkMhz = (MNT_CLOCK_MHz / 4.);

	divider = (LONG)(MNT_CLOCK_MHz / (ClkMhz + 0.0000000001));
	if(divider < 0)
		divider = 0;

	counter = 0;
	if(ClkMhz < 50.)
		while(divider >>= 1) counter++;

	//Convert clock to real clock
	*jtag_speed = ((MNT_CLOCK_MHz / 2.) / ((LONGLONG)1 << counter)) * 1000;

#ifdef MM_LOG_USER
	LOG_USER("M: olympusa_khz: khz %d -- jtag_speed %d kHz", khz, *jtag_speed);
	LOG_USER("M: ClkMhz: %f; divider: %ld; counter: %ld", ClkMhz, divider, counter);
#endif

	return ERROR_OK;
}

//Convert Speed to KHz(1 to 1)
static int olympusa_speed_div(int speed, int *khz)
{
	*khz = speed;
#ifdef MM_LOG_USER
	LOG_USER("M: olympusa_speed_div: speed %d -- khz %d", speed, *khz);
#endif
	return ERROR_OK;
}

COMMAND_HANDLER(connect_command)
{
	uint16_t SerialNumber;
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], SerialNumber);
	LOG_USER("OlympusA: Request to work with Zeus number %d", SerialNumber);
	if(olympusajtag_controller.SerialNumber != (DWORD)SerialNumber)
		{
		//Select New Board
		LOG_USER("OlympusA: Switching from Zeus number %ld to number %d ", olympusajtag_controller.SerialNumber, SerialNumber);

		if(olympusajtag_controller.lDeviceHandle != NULL)
			MNT_CloseDevice(olympusajtag_controller.lDeviceHandle);

		ZeroMemory(&olympusajtag_controller, sizeof(olympusajtag_controller_t));
		olympusajtag_controller.SerialNumber = (DWORD)SerialNumber;

		return olympusa_init();
		}

	return ERROR_OK;
}

COMMAND_HANDLER(port_command)
{
	uint16_t PortNumber;
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], PortNumber);
	LOG_USER("OlympusA: Request new Port Number %d", PortNumber);
	if(olympusajtag_controller.PortNumber != (DWORD)PortNumber)
		{
		//Select New PortNumber
		LOG_USER("OlympusA: Switching Port number %ld to number %d ", olympusajtag_controller.PortNumber, PortNumber);

		//TODO: add port configuring command

		olympusajtag_controller.PortNumber = (DWORD)PortNumber;
		}

	return ERROR_OK;
}

static const struct command_registration olympusa_subcommand_handlers[] = {
	{
		.name = "connect",
		.handler = &connect_command,
		.mode = COMMAND_CONFIG,
		.help = "connect to the specified zeus board (by serial number or by sequence number)",
		.usage = "connect",
	},
	{
		.name = "port",
		.handler = &port_command,
		.mode = COMMAND_CONFIG,
		.help = "select JTAG port number",
		.usage = "port",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration olympusa_command_handlers[] = {
	{
		.name = "olympusa",
		.mode = COMMAND_ANY,
		.help = "perform olympus management",
		.chain = olympusa_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface olympusa_interface = {
	//We do not support DEBUG_CAP_TMS_SEQ() .supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = olympusa_execute_queue,
};

struct adapter_driver olympusa_adapter_driver = {
	.name = "olympusa",
	.transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG,
	.commands	= olympusa_command_handlers,

	.init		= olympusa_init,
	.quit		= olympusa_quit,
	.reset		= olympusa_reset,
	.speed		= olympusa_speed,
	.khz		= olympusa_khz,
	.speed_div	= olympusa_speed_div,

	.jtag_ops	= &olympusa_interface,
};

// Copyright 2014 INPHI
// Errors definitions

#ifndef __OlympusErrors_h_
#define __OlympusErrors_h_

#define	OLYMPUS_SUCCESS						0

//Error Types for External Error Handling function
#define MNT_NO_ERROR	0
#define MNT_WARNING		1
#define MNT_ERROR		2
#define MNT_CRITICAL	3

#define MNT_LEVEL_LOW			0
#define MNT_LEVEL_DRIVER		1
#define MNT_LEVEL_USER			2
#define MNT_LEVEL_NO_HARDWARE	3
#define MNT_LEVEL_PACKET		4

#define	OLYMPUS_SYSTEM_ERROR			0xE0001000
#define	OLYMPUS_LIBRARY_ERROR			0xE0011000

//Memory Problems
#define	OLYMPUS_MEMORY_ALLOC			0x80400000		
#define	OLYMPUS_MEMORY_ERROR			0x80410000		
#define	OLYMPUS_MEMORY_FREE				0x80420000		
#define	OLYMPUS_MEMORY_EXIST			0x80430000		

//Device Problems
#define	OLYMPUS_DEVICEISNOTSUPPORTED	0xC7101000	
#define	OLYMPUS_KERNEL_ACCESS			0x91101000	
#define	OLYMPUS_DEVICENOTFOUND			0x91111000	

#define	OLYMPUS_SUBDEVICEISNOTSUPPORTED	0xC7D01000	

#define	OLYMPUS_DEVICEISNOTINITIALIZED	0xC7D11000

//Zamboni
#define	ZAMBONI_DEVICENOTFOUND			0xC7111000	

//Driver/Software problem
#define	OLYMPUS_OPEN_ERROR				0x80101000
#define	OLYMPUS_ALREADY_OPEN			0x80111000
#define	OLYMPUS_NOT_OPEN				0x80121000

#define	OLYMPUS_CLOSE_ERROR				0x80201000

#define	OLYMPUS_WRONG_MODE				0x80061000

#define	OLYMPUS_LOAD_ERROR				0x871E1000

#define	OLYMPUS_PARAMETER_ERROR			0x80700000
#define	OLYMPUS_PARAMETER_SIZE_ERROR	0x80A00000

#define	OLYMPUS_POWER_NUMBER_ERROR		0x80710000
#define	OLYMPUS_VOLTAGE_TOO_HIGH_ERROR	0x80720000
#define	OLYMPUS_VOLTAGE_TOO_LOW_ERROR	0x80730000
#define	OLYMPUS_VOLTAGE_SENSE_LOW_ERROR	0x80740000

#define	OLYMPUS_OBSOLETE_FUNCTION		0x80010000

//Version problems
#define	OLYMPUS_WRONG_USER_VERSION		0x80001000
#define	OLYMPUS_WRONG_KERNEL_VERSION	0x81001000
#define	OLYMPUS_WRONG_FIRMWARE_VERSION	0x82001000
#define	OLYMPUS_WRONG_FPGA_VERSION		0x83001000
#define	OLYMPUS_WRONG_EEPROM			0x88001000

//File Problems
#define	OLYMPUS_FILE_OPEN_ERROR			0x86E00000
#define	OLYMPUS_FILE_CLOSE_ERROR		0x86E10000
#define	OLYMPUS_FILE_READ_ERROR			0x86E20000
#define	OLYMPUS_FILE_WRITE_ERROR		0x86E30000
#define	OLYMPUS_FILE_CHECKSUM_ERROR		0x86E40000

//FX3
#define	OLYMPUS_DEVICE_IS_BUSY			0x82601000
#define	OLYMPUS_I2C_READ_ERROR			0x82D01000
#define	OLYMPUS_FX3_I2C_SPEED			0x82B01000

#define	OLYMPUS_PACKET_IS_EMPTY_WARNING	0x8A502000
#define	OLYMPUS_PACKET_IN_PROCESS_WARNING	0x8A602000
#define	OLYMPUS_PACKET_FAIL_WARNING		0x8AD02000
#define	OLYMPUS_PACKET_OUT_OF_MEMORY	0x8AA01000

#define	OLYMPUS_NOT_INITIALIZED			0x80D01000

#define	Error_TimerIsRunning			0x8CD01000
#define	Error_TimerIsDead				0x8CD11000
#define	Error_TimerIsWeak				0x8CD21000

#define	Warning_NotAvailable			0x80672000
#define	OLYMPUS_TIMEOUT					0x80301000

#define	OLYMPUS_DEVICE_RETURN_NAK		0x84BD1000

#define	Error_OpenRegistry				0x8D101000
#define	Warning_OpenRegistry			0x8D102000
#define	Error_WriteRegistry				0x8DC01000
#define	Error_ReadRegistry				0x8DB01000
#define	Error_ParamRegistry				0x8D701000	
#define	Error_CloseRegistry				0x8D201000

#define	OLYMPUS_CONFIG_ERROR			0x89001000

#define	Error_InputMode					0x80611000
#define	Error_OutputMode				0x80621000
#define	OLYMPUS_WRONG_DIRECTION			0x80631000
#define	Error_ChannelNumber				0x80641000
#define	Error_SamplingRate				0x80651000
#define	Error_StartOffset				0x80661000

#define	Error_Software					0x8FF01000

#define	Error_Function_Mask				0xFFFFF000

//MATLAB ERRORs
#define	Error_Matlab_FunctionNotImplemented		0xA0001300
#define	Error_Matlab_MemoryAllocation			0xA0401300
#define	Error_Matlab_NeedParameters_2			0xA0701300
#define	Error_Matlab_NeedParameters_3			0xA0711300
#define	Error_Matlab_NeedParameters_4			0xA0721300
#define	Error_Matlab_NeedReturnValue			0xA0731300
#define	Error_Matlab_NeedStructure				0xA0741300
#define	Error_Matlab_WrongSize					0xA0601300

#endif


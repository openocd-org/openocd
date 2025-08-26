#ifndef _ZEUS_INTERFACE_H_
#define _ZEUS_INTERFACE_H_

//Cypress GPIO Pins:

#define ZEUS_CTRL1				22	//CSI_B
#define ZEUS_CTRL2				17	//RDWR_B
#define ZEUS_CTRL3				23	//PUDC_B
#define ZEUS_CTRL4				26	//EMCCLK

#define ZEUS_CCLK				27

#define ZEUS_STATUS3			20
#define ZEUS_STATUS4			21

#define ZEUS_WP_DB				51

#define ZEUS_WP_MB				50

#define ZEUS_PROG_B				25
#define ZEUS_INIT_B				28
#define ZEUS_DONE				29

#define ZEUS_ERROR_LED			45

#define ZEUS_DB0				52
#define ZEUS_DB1				57

//USB Commands via Control Pipe
#define ZEUS_CMD_INTERNAL 		0xA0

#define ZEUS_CMD_JMP_BOOT 		0x90
#define ZEUS_CMD_SW_USB2 		0x91
#define ZEUS_CMD_SW_USB3 		0x92
#define ZEUS_CMD_RST			0x93
#define ZEUS_CMD_GET_BOOR_VER	0x94
#define ZEUS_CMD_RD_FX3_DIRECT	0x95
#define ZEUS_CMD_ENABLE_WP 		0x99

#define ZEUS_CMD_VERSION 		0xB0
#define ZEUS_CMD_RESET	  		0xB1
#define ZEUS_CMD_PROG_FPGA  	0xB2

#define ZEUS_CMD_EEPROM_READ	0xB3	// 16 Bit Address + Data
#define ZEUS_CMD_EEPROM_WRITE	0xB4	// 16 Bit Address + Data

#define ZEUS_CMD_I2C_READ	    0xB5	// Read One Byte (Address + Data)
#define ZEUS_CMD_I2C_WRITE	    0xB6	// Write One Byte (Address + Data)
#define ZEUS_CMD_I2C_WRITE_JANUS	0xB7	// Set Janus Port + Write One Byte (Address + Data)
#define ZEUS_CMD_I2C_READ_JANUS		0xB8	// Set Janus Port + Read One Byte (Address + Data)

#define ZEUS_CMD_GPIO_CTRL_GET	0xB9
#define ZEUS_CMD_GPIO_CTRL_SET	0xBA

#define ZEUS_CMD_GET_DONE		0xBB

#define ZEUS_GET_USB_SPEED		0xBC

#define ZEUS_CMD_GPIO_DB_GET	0xBD	//2 bits to DB
#define ZEUS_CMD_GPIO_DB_SET	0xBE	//2 bits to DB
#define ZEUS_CMD_GPIO_DB_CFG	0xBF	//2 bits to DB Configure

#define ZEUS_CMD_FAN_CONTROL	0xC0	//Measure GPIO_DB0, Set Threshold, Enable Loop
#define ZEUS_CMD_DB_ID_GET		0xC1	//Get DB Type
#define ZEUS_CMD_API_CTRL		0xC2
#define ZEUS_CMD_MDIO_WRITE		0xC3
#define ZEUS_CMD_MDIO_READ		0xC4
#define ZEUS_CMD_API_TEST0		0xC5	//Write
#define ZEUS_CMD_API_TEST1		0xC6	//Read
#define ZEUS_CMD_API_TEST2		0xC7	//Write
#define ZEUS_CMD_API_TEST3		0xC8	//Read

#define ZEUS_CMD_EEPROM_READ_ID			0xD0	//Read Device Types and Serial Numbers
#define ZEUS_CMD_EEPROM_FORCE_READ_ID	0xD1	//Force reading Serial Numbers

#define ZEUS_CMD_READ_TIME				0xD2

#define ZEUS_CMD_I2C_SET_PREAMBLE		0xD3
#define ZEUS_CMD_I2C_GET_PREAMBLE		0xD4
#define ZEUS_CMD_I2C_FULL_READ			0xD5
#define ZEUS_CMD_I2C_FULL_WRITE			0xD6
#define ZEUS_CMD_I2C_LAST_ERROR			0xD7
#define ZEUS_CMD_I2C_SPEED				0xD8

#define ZEUS_CMD_UART_SEND				0xD9
#define ZEUS_CMD_UART_RECEIVE			0xDA

#define ZEUS_CMD_EEPROM_READ_JANUS		0xDB	// Switch Port + 16 Bit Address + Data
#define ZEUS_CMD_EEPROM_WRITE_JANUS		0xDC	// Switch Port + 16 Bit Address + Data

#define ZEUS_CMD_I2C_FULL_READ_JANUS	0xDD	// Switch Port + Read
#define ZEUS_CMD_I2C_FULL_WRITE_JANUS	0xDE	// Switch Port + Write

#define ZEUS_CMD_GET_LAST_TEMPERATURE	0xDF	// Get Last Temperature from TMP411 / TMP461
/*
#define ZEUS_CMD_PID_INIT				0xE0
#define ZEUS_CMD_PID_GETCOEF			0xE1
#define ZEUS_CMD_PID_SETLIMITS			0xE2
#define ZEUS_CMD_PID_GETLIMITS			0xE3
#define ZEUS_CMD_PID_GETRESULT			0xE4
#define ZEUS_CMD_PID_SETTARGET			0xE5
*/

//Zamboni Functions
#define ZEUS_CMD_ZAMBONI_SET_CONFIG        	0xE0
#define ZEUS_CMD_ZAMBONI_GET_CONFIG        	0xE1
#define ZEUS_CMD_ZAMBONI_TEMPERATURE		0xE2
#define ZEUS_CMD_ZAMBONI_GET_STATUS			0xE3
#define ZEUS_CMD_ZAMBONI_ENABLE         	0xE4
#define ZEUS_CMD_ZAMBONI_RMW_GPIOS         	0xE5
#define ZEUS_CMD_ZAMBONI_PELTIER_DIRECTION  0xE6

//I2C Info. Please use only 400 kHz !!!

#define ZEUS_MB_ADDR				0xA0	    //EEPROM 24AA128/24LC128			400 kHz
#define ZEUS_DB_ADDR				0xA2	    //EEPROM 24AA128/24LC128			400 kHz
#define ZEUS_INTERPOSER_ADDR		0xA4	    //EEPROM 24AA128/24LC128			400 kHz
#define ZAMBONI_ADDR				0xAC	    //EEPROM 24AA128/24LC128			400 kHz

#define ZEUS_I2C_GPIO_ADDR			0x40	    //0x20 << 1	SX1502 GPIO Extender	400 kHz
#define ZEUS_I2C_GPIO_DAT_ADDR		0x00
#define ZEUS_I2C_GPIO_DIR_ADDR		0x01

#define ZEUS_I2C_CURR_ADDR			0xE0	    //0x70 << 1	XRP7620 400 kHz

#define ATLAS_I2C_GPIO_ADDR			0x40	    //0x20 << 1	PCA9575 GPIO Extender	400 kHz

#define ATLAS_I2C_GPIO_EXT_ADDR0	0x5C	//0x2E << 1
#define ATLAS_I2C_GPIO_EXT_ADDR1	0x5A	//0x2D << 1		-- This is main for Atlas-2
#define ATLAS_I2C_GPIO_EXT_ADDR2	0x5E	//0x2F << 1

#define TUNDRA_1_NUMBER_OF_EXTENDERS	5
#define TUNDRA_I2C_GPIO_EXT_ADDR1	0x44	//0x22 << 1 -- GPIO Extender Address 0x2
#define TUNDRA_I2C_GPIO_EXT_ADDR2	0x46	//0x23 << 1 -- GPIO Extender Address 0x3
#define TUNDRA_I2C_GPIO_EXT_ADDR3	0x48	//0x24 << 1 -- GPIO Extender Address 0x4
#define TUNDRA_I2C_GPIO_EXT_ADDR4	0x4A	//0x25 << 1 -- GPIO Extender Address 0x5
#define TUNDRA_I2C_GPIO_EXT_ADDR5	0x4C

#define URANUS7_I2C_GPIO_EXT_U10	0x50	//PCA9575 GPIO Extender			400 kHz
#define URANUS7_I2C_GPIO_EXT_U11	0x54
#define URANUS7_I2C_GPIO_EXT_U12	0x52
#define URANUS7_I2C_GPIO_EXT_U13	0x56
#define URANUS7_I2C_GPIO_EXT_U14	0x44
#define URANUS7_I2C_GPIO_EXT_U15	0x4A
#define URANUS7_I2C_GPIO_EXT_U16	0x46
#define URANUS7_I2C_GPIO_EXT_U17	0x48

#define ZAMBONI_I2C_GPIO_EXT_ADDR	0x58    //Zamboni

#define LM96063_ADDR				0x98	//1001 100R. This is the same as TMP411A_ADDR !!!	100 kHz
#define AMC6821_ADDR_00				0x30	//0011 000R											100 kHz

//For Sanket:
#define MAX77812EWB					0x62    //DC-DC
#define ADS1100                     0x92    //One channel ADC

#define DS4432_ADDR					0x90	//1001000R - NOT USED	400 kHz
#define DS4424_ADDR_00				0x20	//A1_A0_10_000R			400 kHz
#define AD5593R_ADDR_1              0x22    //AD5593R (ZAMBONI)     400 kHz
    #define AD5593R_CONFIG_MODE     0 << 4
        #define AD5593R_ADC_SEQ_REG         0x0002
        #define AD5593R_GP_CTRL_REG         0x0003
        #define AD5593R_ADC_CFG_REG         0x0004
        #define AD5593R_DAC_CFG_REG         0x0005
        #define AD5593R_PULLDOWN_CFG_REG    0x0006
        #define AD5593R_LDAC_MODE_REG       0x0007
        #define AD5593R_GPIO_CFG_REG        0x0008
        #define AD5593R_GPIO_OUTPUT_REG     0x0009
        #define AD5593R_GPIO_INPUT_REG      0x000A
        #define AD5593R_PD_REF_REG          0x000B
        #define AD5593R_GPIO_OPENDRAIN_CFG_REG      0x000C
        #define AD5593R_TS_CFG_REG          0x000D
        #define AD5593R_SW_RESET_REG        0x000F

    #define AD5593R_DAC_WR           1 << 4

        #define AD5593R_VPELTIER_CH     0
        #define AD5593R_CUR_LIMIT_CH    1
        #define AD5593R_VFAN0_CH        2
        #define AD5593R_VFAN1_CH        3

    #define AD5593R_ADC_RD           4 << 4
    #define AD5593R_DAC_RD           5 << 4
    #define AD5593R_GPIO_RD          6 << 4
    #define AD5593R_REG_RD           7 << 4

#define SHT4_ADDR_A                 0x88    //Humidity Sensor (ZAMBONI)
#define SHT4_ADDR_B                 0x8A    //Humidity Sensor
#define SHT4_ADDR_C                 0x8C    //Humidity Sensor

#define TMP411A_ADDR_100			0x98	//(0x4C << 1) 1001 100R				400 kHz (MAIN)
#define TMP411A_ADDR_011			0x96	//(0x4B << 1) 1001 011R				400 kHz (Secondary)

//#define TMP461_ADDR_0				0x90	//(0x48 << 1) 1001 000R				400 kHz
//-- RESERVED for ADS1100 --- #define TMP461_ADDR_1				0x92	//(0x49 << 1) 1001 001R				400 kHz
//#define TMP461_ADDR_2				0x94	//(0x4A << 1) 1001 010R				400 kHz
//#define TMP461_ADDR_3				0x96	//(0x4B << 1) 1001 011R				400 kHz
//#define TMP461_ADDR_4				0x98	//(0x4C << 1) 1001 100R				400 kHz
#define TMP461_ADDR_5				0x9A	//(0x4D << 1) 1001 101R (ZAMBONI)	400 kHz
#define TMP461_ADDR_6				0x9C	//(0x4E << 1) 1001 110R (ZAMBONI)	400 kHz
#define TMP461_ADDR_7				0x9E	//(0x4F << 1) 1001 111R				400 kHz
//#define TMP461_ADDR_8				0xA0	//(0x50 << 1) 1010 000R				400 kHz

#define MCP9600_ADDR_0				0xC0	//1100 000R	(Ceres & ZAMBONI)	    100 kHz			!!!
#define MCP9600_ADDR_7				0xCE	//1100 111R	(ZAMBONI)	            100 kHz			!!!

#define JANUS_I2C_SWITCH_ADDR		0xE4	//0x72 << 1
#define REGRESSION_I2C_GPIO_ADDR	0x42	//0x21 << 1
#define JANUS_I2C_GRETA_ADDR		0x4E	//0x27 << 1

#define APOLLO_I2C_DAC_AD5697R_ADDR	0x18	//0x0C << 1

#define TUNDRA_PCIE_CLK_ADDR_0		0xD0	//1101000
#define TUNDRA_PCIE_CLK_ADDR_1		0xD4	//1101010

#define TUNDRA_PCIE_CLK_MAX_BYTES	20

//Humidity Sensor Commands
#define HS_SOFT_RST_CMD					0x94
#define HS_GETSERIALNUMBER_CMD			0x89
#define HS_MEAS_HP_CMD					0xFD
#define HS_MEAS_MP_CMD					0xF6
#define HS_MEAS_LP_CMD					0xE0
#define HS_HEAT_200_1S_MEAS_LP_CMD		0x39
#define HS_HEAT_200_100MS_MEAS_LP_CMD	0x32
#define HS_HEAT_110_1S_MEAS_LP_CMD		0x2F
#define HS_HEAT_110_100MS_MEAS_LP_CMD	0x24
#define HS_HEAT_20_1S_MEAS_LP_CMD		0x1E
#define HS_HEAT_20_100MS_MEAS_LP_CMD	0x15

//FPGA Commands

//FPGA Memory Map
#define ZEUS_DEVICEID_REG_ADDR  0x0
#define ZEUS_VERSIONID_REG_ADDR 0x1
#define ZEUS_COMMAND_REG_ADDR   0x2
//Bit 0 - Zeus-2 - SWAP ADC0/ADC1						Janus: o_DetMuxR			Apollo: VSENSE
//Bit 1 - Zeus-3-Limited								Janus: o_TcxoEn				Apollo: GPIO is GPIO in SIF MODE
//Bit 2 - Force MDIO Optimization to be always ON		Janus: o_ExtTcxoEn
//Bit 3 - Enable MDIO Optimization
//Bit 4 - Enable DB Signals / Enable Marvell Sif on GPIO2 / Temp for use with Uranus-9
//Bit 5 - Clock optimization
//Bit 6 - USB2
//Bit 7 -												Janus: SI_RESET_N 
//Bit 8 -												Janus: PWR_CYCLE 
//Bit 9  - Zamboni Mode
//Bit 10 - Enable Zamboni

#define ZEUS_STATUS_REG_ADDR    0x3
//Bit 0 - DB is Present
//Bit 1 - Power sequence is busy 
//Bit 2 - Lock Status (Enable, Low, Lock)
//Bit 3 - i_AverageRst
//Bit 4 - i_AtlasAverageRst

//Bit 8 - Trigger 0
//Bit 9 - Trigger 1

//Bit 10 - Zamboni is Present

//Test Points and LEDs
#define ZEUS_TP1_REG_ADDR       0x4
#define ZEUS_TP2_REG_ADDR       0x5
#define ZEUS_TP3_REG_ADDR       0x6
#define ZEUS_TP4_REG_ADDR       0x7

#define ZEUS_LED1_REG_ADDR      0x8
#define ZEUS_LED2_REG_ADDR      0x9
#define ZEUS_LED3_REG_ADDR      0xA
#define ZEUS_LED4_REG_ADDR      0xB

#define ZEUS_GPIO_DIR0_ADDR     0xC
#define ZEUS_GPIO_DAT0_ADDR     0xD

#define ZEUS_GPIO_R_DIR_ADDR    0xE
#define ZEUS_GPIO_R_DAT_ADDR    0xF		// o_BiaStartIn for Tundra
#define TRISTATE_SPI_REG_ADDR   0xF		// For JANUS

//#define ZEUS_GPIO_DAT1_ADDR     0xF

//Power Manager. DACs. ADCs

///////////////////////////////////////////////////////////////////

#define ZEUS_PWR0_CTRL_REG      0x0010   //Bit 0 - Start (AutoClear), Bit 1 - Start from START_VAL, Bit 2, 3 - Select Sequence, Bit 31 - Running  
#define ZEUS_PWR1_CTRL_REG      0x0011
#define ZEUS_PWR2_CTRL_REG      0x0012
#define ZEUS_PWR3_CTRL_REG      0x0013
#define ZEUS_PWR4_CTRL_REG      0x0014
#define ZEUS_PWR5_CTRL_REG      0x0015

#define ZEUS_PWR6_CTRL_REG		0x0016   //Atlas DAC0 PWR8/9
#define ZEUS_PWR7_CTRL_REG		0x0017   //Atlas DAC2 PWR7
#define ZEUS_PWR8_CTRL_REG		0x0018   //Atlas DAC3 PWR6
#define ZEUS_PWR9_CTRL_REG		0x0019   //Atlas DAC4 PWR5
#define ZEUS_PWR10_CTRL_REG		0x001A   //Atlas DAC5 PWR4
#define ZEUS_PWR11_CTRL_REG		0x001B   //Atlas DAC6 VAUX0
#define ZEUS_PWR12_CTRL_REG		0x001C   //Atlas DAC7 VAUX1
#define ZEUS_PWR13_CTRL_REG		0x001D   //Atlas DAC8 Optional SW

//reserved 0x1E
//reserved 0x1F

#define ZEUS_PWR0_START_VAL     0x0020
#define ZEUS_PWR0_STOP_VAL      0x0021
#define ZEUS_PWR0_STEP_VAL      0x0022
#define ZEUS_PWR0_DELAY_VAL     0x0023
#define ZEUS_PWR0_INIT_DLY_VAL	0x0024
#define ZEUS_PWR0_TRIG_VAL		0x0025

#define ZEUS_PWR1_START_VAL     0x0026
#define ZEUS_PWR1_STOP_VAL      0x0027
#define ZEUS_PWR1_STEP_VAL      0x0028
#define ZEUS_PWR1_DELAY_VAL     0x0029
#define ZEUS_PWR1_INIT_DLY_VAL	0x002A
#define ZEUS_PWR1_TRIG_VAL		0x002B

#define ZEUS_PWR2_START_VAL     0x002C
#define ZEUS_PWR2_STOP_VAL      0x002D
#define ZEUS_PWR2_STEP_VAL      0x002E
#define ZEUS_PWR2_DELAY_VAL     0x002F
#define ZEUS_PWR2_INIT_DLY_VAL	0x0030
#define ZEUS_PWR2_TRIG_VAL		0x0031

#define ZEUS_PWR3_START_VAL     0x0032
#define ZEUS_PWR3_STOP_VAL      0x0033
#define ZEUS_PWR3_STEP_VAL      0x0034
#define ZEUS_PWR3_DELAY_VAL     0x0035
#define ZEUS_PWR3_INIT_DLY_VAL	0x0036
#define ZEUS_PWR3_TRIG_VAL		0x0037

#define ZEUS_PWR4_START_VAL     0x0038
#define ZEUS_PWR4_STOP_VAL      0x0039
#define ZEUS_PWR4_STEP_VAL      0x003A
#define ZEUS_PWR4_DELAY_VAL     0x003B
#define ZEUS_PWR4_INIT_DLY_VAL	0x003C
#define ZEUS_PWR4_TRIG_VAL		0x003D

//FOR DAC DB:
#define ZEUS_PWR5_START_VAL     0x003E
#define ZEUS_PWR5_STOP_VAL      0x003F
#define ZEUS_PWR5_STEP_VAL      0x0040
#define ZEUS_PWR5_DELAY_VAL     0x0041
#define ZEUS_PWR5_INIT_DLY_VAL	0x0042
#define ZEUS_PWR5_TRIG_VAL		0x0043

//FOR ATLAS-1. PWR-0/1 VDD (PWR8/9) DAC0
#define PWR6_START_VAL			0x0044
#define PWR6_STOP_VAL		    0x0045
#define PWR6_STEP_VAL		    0x0046
#define PWR6_DELAY_VAL			0x0047
#define PWR6_INIT_DLY_VAL		0x0048
#define PWR6_TRIG_VAL			0x0049

//FOR ATLAS-1. PWR-2 (PWR7) DAC2
#define PWR7_START_VAL			0x004A
#define PWR7_STOP_VAL			0x004B
#define PWR7_STEP_VAL			0x004C
#define PWR7_DELAY_VAL			0x004D
#define PWR7_INIT_DLY_VAL		0x004E
#define PWR7_TRIG_VAL			0x004F

//FOR ATLAS-1. PWR-3 (PWR6) DAC3
#define PWR8_START_VAL			0x0050
#define PWR8_STOP_VAL			0x0051
#define PWR8_STEP_VAL			0x0052
#define PWR8_DELAY_VAL			0x0053
#define PWR8_INIT_DLY_VAL		0x0054
#define PWR8_TRIG_VAL			0x0055

//FOR ATLAS-1. PWR-4 (PWR5) DAC4
#define PWR9_START_VAL			0x0056
#define PWR9_STOP_VAL			0x0057
#define PWR9_STEP_VAL			0x0058
#define PWR9_DELAY_VAL			0x0059
#define PWR9_INIT_DLY_VAL		0x005A
#define PWR9_TRIG_VAL			0x005B

//FOR ATLAS-1. PWR-5 (PWR5) DAC5
#define PWR10_START_VAL			0x005C
#define PWR10_STOP_VAL			0x005D
#define PWR10_STEP_VAL			0x005E
#define PWR10_DELAY_VAL			0x005F
#define PWR10_INIT_DLY_VAL		0x0060
#define PWR10_TRIG_VAL			0x0061

//FOR ATLAS-1. PWR-6 (AUX0) DAC6
#define PWR11_START_VAL			0x0062
#define PWR11_STOP_VAL			0x0063
#define PWR11_STEP_VAL			0x0064
#define PWR11_DELAY_VAL			0x0065
#define PWR11_INIT_DLY_VAL		0x0066
#define PWR11_TRIG_VAL			0x0067

//FOR ATLAS-1. PWR-7 (AUX1) DAC7
#define PWR12_START_VAL			0x0068
#define PWR12_STOP_VAL			0x0069
#define PWR12_STEP_VAL			0x006A
#define PWR12_DELAY_VAL			0x006B
#define PWR12_INIT_DLY_VAL		0x006C
#define PWR12_TRIG_VAL			0x006D

//FOR ATLAS-1. Optional SW DAC8
#define PWR13_START_VAL			0x006E
#define PWR13_STOP_VAL			0x006F
#define PWR13_STEP_VAL			0x0070
#define PWR13_DELAY_VAL			0x0071
#define PWR13_INIT_DLY_VAL		0x0072
#define PWR13_TRIG_VAL			0x0073

//Total number for PWR Manager registers 5 + 5 * (4 * 4) + 1 = 86
#define ZEUS_PWR_MEM_SIZE       52	//ZEUS_PWR5_TRIG_VAL - ZEUS_PWR0_CTRL_REG + 1		//52	
#define ZEUS_ATLAS_PWR_MEM_SIZE 100	//ZEUS_PWR5_TRIG_VAL - ZEUS_PWR0_CTRL_REG + 1		//52	
#define ZEUS_PWR_MEM_GROUP_SIZE 6	//ZEUS_PWR0_TRIG_VAL - ZEUS_PWR0_START_VAL + 1		//6

//-----------------------------------

#define ZEUS_PM_CTRL_REG        0x0075
//Bit 0 - En DB, 
//Bit 1 - overwrite reset1, 
//Bit 2 - overwrite reset1 value, 
//Bit 3 - overwrite reset2, 

//Bit 4 - overwrite reset2 value
//Bit 5 - Reset DACs
//Bit 6 - overwrite reset1 tristate 
//Bit 7 - overwrite reset2 tristate 

//Bit 8 - Enable DAC to DB
//Bit 9 - ERIS-2x Mode
//Bit 10 - ATLAS Mode
//Bit 11 - Overwrite Atlas Discharge

//Bit 12 - Atlas Load Enable to GPIO24 
//Bit 13 - Atlas Discharge Value 0
//Bit 14 - Atlas Discharge Value 1
//Bit 15 - Atlas Discharge Value 2

//Bit 16 - Atlas Discharge Value 3
//Bit 17 - Atlas Discharge Value 3
//Bit 18 - Pontus-1 remap ADCs			
//Bit 19 - VDD0 On DB for ERIS-4 Mode	/ o_DbPowerOff for Tundra

//Bit 20 - reset 1 polarity (0 - active low)
//Bit 21 - reset 2 polarity (0 - active low)
//Bit 22 - Atlas-2 indicator
//Bit 23 - Discharge on R1   //ERIS-4 Mode

//Bit 24 - overwrite reset3, 
//Bit 25 - overwrite reset3 value, 
//Bit 26 - overwrite reset4, 
//Bit 27 - overwrite reset4 value
	 
//Bit 28 - overwrite reset3 tristate 
//Bit 29 - overwrite reset4 tristate 
//Bit 30 -  
//Bit 31 -  

//For Apollo
//Bit 24 - o2_VccioEn[0]
//Bit 25 - o2_VccioEn[1]
//Bit 26 - o2_I2CEn[0]     -> Active LOW - Not Available on Apollo-1B
//Bit 27 - o2_I2CEn[1]     -> Active LOW

#define ZEUS_PM_CTRL_DB_EN				0x00000001
#define ZEUS_PM_CTRL_RST1_EN			0x00000002
#define ZEUS_PM_CTRL_RST1_VAL			0x00000004
#define ZEUS_PM_CTRL_RST2_EN			0x00000008

#define ZEUS_PM_CTRL_RST2_VAL			0x00000010
#define ZEUS_PM_CTRL_RST_DAC			0x00000020
#define ZEUS_PM_CTRL_RST1_TRI			0x00000040
#define ZEUS_PM_CTRL_RST2_TRI			0x00000080

#define ZEUS_PM_CTRL_DB_DAC_EN_TRI		0x00000100
#define ZEUS_PM_CTRL_ERIS2				0x00000200
#define ZEUS_PM_CTRL_KILL_TEMP_EN		0x00000400
#define ZEUS_PM_CTRL_ATLAS_DISCH_EN		0x00000800

#define ZEUS_PM_CTRL_LOAD_EN			0x00001000
#define ZEUS_PM_CTRL_ATLAS_DISCHARGE0	0x00002000
#define ZEUS_PM_CTRL_ATLAS_DISCHARGE1	0x00004000
#define ZEUS_PM_CTRL_ATLAS_DISCHARGE2	0x00008000

#define ZEUS_PM_CTRL_ATLAS_DISCHARGE3	0x00010000
#define ZEUS_PM_CTRL_ATLAS_DISCHARGE4	0x00020000
#define ZEUS_PM_CTRL_PONTUS_REMAP		0x00040000
#define ZEUS_PM_CTRL_VDD0_ON_DB			0x00080000

#define ZEUS_PM_ATLAS_2_INDICATOR		0x00400000
#define ZEUS_PM_CTRL_DISCHARGE_ON_R1	0x00800000

#define ZEUS_PM_CTRL_RST3_EN			0x01000000
#define ZEUS_PM_CTRL_RST3_VAL			0x02000000
#define ZEUS_PM_CTRL_RST4_EN			0x04000000
#define ZEUS_PM_CTRL_RST4_VAL			0x08000000
#define ZEUS_PM_CTRL_RST3_TRI			0x10000000
#define ZEUS_PM_CTRL_RST4_TRI			0x20000000

#define ZEUS_PM_APOLLO_EN_PWR0          0x01000000
#define ZEUS_PM_APOLLO_EN_PWR1          0x02000000
#define ZEUS_PM_APOLLO_EN_I2C0          0x04000000  //Not on Apollo-1B
#define ZEUS_PM_APOLLO_EN_I2C1          0x08000000

#define ZEUS_PM_APOLLO_PULL_UP_SDATA0   0x10000000
#define ZEUS_PM_APOLLO_PULL_UP_SCLK0    0x20000000
#define ZEUS_PM_APOLLO_PULL_UP_SDATA1   0x40000000
#define ZEUS_PM_APOLLO_PULL_UP_SCLK1    0x80000000

#define ZEUS_CLK_CTRL_REG		0x0076   
//Bit 0 - OscEn, 
//Bit 1 - Clk_5v_En, 
//Bit 2 - Clk_Ext2_EN[0] 
//Bit 3 - Clk_Ext2_EN[1] 
//Bit 4,5 - Disable ADC Clock
//Bit 6 - Disable ATLAS ADC Clock

#define ZEUS_TP_INVERT_REG		0x0077
//#define ZEUS_ADDR_STEP_REG		0x0078

#define ZEUS_ADC_AVERAGE_REG	0x0079

//#define ZEUS_MDIO_DIV_REG		0x0070

#define ZEUS_MDIO0_DIV_REG      0x007A
#define ZEUS_MDIO1_DIV_REG      0x007B
#define ZEUS_MDIO2_DIV_REG      0x007C
#define ZEUS_MDIO3_DIV_REG      0x007D
#define ZEUS_MDIO4_DIV_REG      0x007E

//#define ZEUS_MDIO_CTRL_REG		0x0071
//Bit 0 - en continues clock MDIO0
//Bit 1 - en continues clock MDIO1
//Bit 2 - en continues clock MDIO2
//Bit 3 - en continues clock MDIO3

//Bit 4 - tristate clock MDC0
//Bit 5 - tristate clock MDC1
//Bit 6 - tristate clock MDC2
//Bit 7 - tristate clock MDC3

//Bit 8 - enable I2C mode port 0
//Bit 9 - enable I2C mode port 1
//Bit 10 - enable I2C mode port 2
//Bit 11 - enable I2C mode port 3

//Bit 12 - enable SPI mode port 0
//Bit 13 - enable SPI mode port 1
//Bit 14 - enable SPI mode port 2
//Bit 15 - enable SPI mode port 3

//Bit 16 - en continues clock MDIO4
//Bit 17 - tristate clock MDC4
//Bit 18 - enable I2C mode port 4
//Bit 19 - enable SPI mode port 4

//Bit 20 - toggle clock MDC4
//Bit 21
//Bit 22
//Bit 23

//Bit 24
//Bit 25
//Bit 26
//Bit 27

//Bit 28 - toggle clock MDC0
//Bit 29 - toggle clock MDC1
//Bit 30 - toggle clock MDC2
//Bit 31 - toggle clock MDC3

//#define ZEUS_GPIO_R_DIR_ADDR    0x0072
//#define ZEUS_GPIO_R_DAT_ADDR    0x0073

// 8/15/2019 #define ZEUS_SPI_READ_DATA		0x00080
#define ZEUS_GPIO_DIR1_ADDR     0x0080
#define ZEUS_GPIO_DAT1_ADDR     0x0081
    
#define ZEUS_GPIO_0_OD_ADDR		0x0082
#define ZEUS_GPIO_R_OD_ADDR		0x0083

#define PORT0_CTRL_REG			0x0084
#define PORT1_CTRL_REG			0x0085
#define PORT2_CTRL_REG			0x0086
#define PORT3_CTRL_REG			0x0087
#define PORT4_CTRL_REG			0x0088

//Bits 3:0
#define SERIAL_PORT_MDIO		        0
#define SERIAL_PORT_I2C			        1
#define SERIAL_PORT_JTAG		        2
#define SERIAL_PORT_IJTAG     	        3
#define SERIAL_PORT_SPI_EEPROM	        4
#define SERIAL_PORT_I2C_H		        5
#define SERIAL_PORT_MRVL_SIF            6
#define SERIAL_PORT_GPIO_OVERWRITE_MODE 7
#define SERIAL_PORT_SPI_SPARTA_MODE     8
#define SERIAL_PORT_SPI_H_MODE          9
#define SERIAL_PORT_MARVELL_SIF_OLD     10
#define SERIAL_PORT_SPI_SILAB           11
#define SERIAL_PORT_SIF_MDIO22_A        12
#define SERIAL_PORT_SIF_MDIO22_B        13
#define SERIAL_PORT_JTAG_SVF	        14
#define SERIAL_PORT_DISABLE		        0xF

//Bits 5:4
#define SERIAL_PORT_CONT_CLK	        0x1
#define SERIAL_PORT_TRI_CLK		        0x2
//Bit 6
#define DISABLE_I2C_ABORT		        0x40        //USED for INDIRECT MDIO Mode

//Bit 7 - 32 bit data for Marvell SIF
#define MARVELL_SIF_32_DATA			    1
#define MARVELL_SIF_32_DATA_MASK	    0x00000080

//Bit 7 - 32 bit data for Marvell SIF
//`define MARVELL_SIF_32_DATA 1'b1

//Bit 13:8 - Number of Address Bits for Marvell SIF - 6 bit - up to 64
#define MARVELL_SIF_32_ADDRESS_MASK	    0x00003F00
//Bit 15:14 - Step Size for address increment 2 bits shift by 1, 2, 4, 8
#define MARVELL_SIF_STEP_SIZE_MASK	    0x0000C000
// Bit 16
#define SERIAL_CLK_READ_MODE    	    0x00010000
// Bit 17
#define SERIAL_DELAY_READ_MODE    	    0x00020000
// Bit 20, 21, 22 -> Select Index for SIF Mode Control

//Bit 30 -  Set Clock
//Bit 31 -  Toggle Clock
#define SERIAL_SET_CLK			        0x40000000
#define SERIAL_TOGGLE_CLK		        0x80000000

#define PORT_STATUS0			0x0089
#define PORT_STATUS1			0x008A

#define PORT_STATUS2			0x008B  //SVF Port 0
#define PORT_STATUS3			0x008C  //SVF Port 1

//#define PORT4_STATUS			0x008D

#define SIF_AUX_CTRL    		0x008D
//Bit 0 -  Value for BANK_SEL (PhyAddress 0)
//Bit 1 -  Overwrite BANK_SEL
//Bit 2 -  Value for SCAN_TEST_EN (PhyAddress 1)
//Bit 3 -  Overwrite SCAN_TEST_EN
//Bit 4 -  Value for UTMI_RESET (PhyAddress 2)
//Bit 5 -  Overwrite BANK_SEL

//#define 	0x0079
//#define 	0x007A
#define GPIO_0_CNFG_23_16_ADDR	0x008E
#define GPIO_0_CNFG_31_24_ADDR	0x008F

///////////////////////////////////////////////////////////////////
#define PWR0_MAX_REG		0x0090 
#define PWR0_MIN_REG		0x0091
#define PWR0_OVER_REG		0x0092 
#define PWR0_UNDER_REG		0x0093 
#define PWR0_UP_THRES_REG	0x0094 
#define PWR0_DN_THRES_REG	0x0095 
#define PWR0_DIS_THRES_REG	0x0096 

#define PWR1_MAX_REG		0x0098 
#define PWR1_MIN_REG		0x0099
#define PWR1_OVER_REG		0x009A 
#define PWR1_UNDER_REG		0x009B 
#define PWR1_UP_THRES_REG	0x009C 
#define PWR1_DN_THRES_REG	0x009D 
#define PWR1_DIS_THRES_REG	0x009E 

#define PWR2_MAX_REG		0x00A0 
#define PWR2_MIN_REG		0x00A1
#define PWR2_OVER_REG		0x00A2 
#define PWR2_UNDER_REG		0x00A3 
#define PWR2_UP_THRES_REG	0x00A4 
#define PWR2_DN_THRES_REG	0x00A5 
#define PWR2_DIS_THRES_REG	0x00A6 

#define PWR3_MAX_REG		0x00A8 
#define PWR3_MIN_REG		0x00A9
#define PWR3_OVER_REG		0x00AA 
#define PWR3_UNDER_REG		0x00AB 
#define PWR3_UP_THRES_REG	0x00AC 
#define PWR3_DN_THRES_REG	0x00AD 
#define PWR3_DIS_THRES_REG	0x00AE 
//Atlas
#define PWR4_MAX_REG		0x00B0 
#define PWR4_MIN_REG		0x00B1
#define PWR4_OVER_REG		0x00B2 
#define PWR4_UNDER_REG		0x00B3 
#define PWR4_UP_THRES_REG	0x00B4 
#define PWR4_DN_THRES_REG	0x00B5 
#define PWR4_DIS_THRES_REG	0x00B6 

#define PWR5_MAX_REG		0x00B8 
#define PWR5_MIN_REG		0x00B9
#define PWR5_OVER_REG		0x00BA 
#define PWR5_UNDER_REG		0x00BB 
#define PWR5_UP_THRES_REG	0x00BC 
#define PWR5_DN_THRES_REG	0x00BD 
#define PWR5_DIS_THRES_REG	0x00AE 

#define PWR6_MAX_REG		0x00C0 
#define PWR6_MIN_REG		0x00C1
#define PWR6_OVER_REG		0x00C2 
#define PWR6_UNDER_REG		0x00C3 
#define PWR6_UP_THRES_REG	0x00C4 
#define PWR6_DN_THRES_REG	0x00C5 
#define PWR6_DIS_THRES_REG	0x00C6 

#define PWR7_MAX_REG		0x00C8 
#define PWR7_MIN_REG		0x00C9
#define PWR7_OVER_REG		0x00CA 
#define PWR7_UNDER_REG		0x00CB 
#define PWR7_UP_THRES_REG	0x00CC 
#define PWR7_DN_THRES_REG	0x00CD 
#define PWR7_DIS_THRES_REG	0x00CE 

#define PWR8_MAX_REG		0x00D0 
#define PWR8_MIN_REG		0x00D1
#define PWR8_OVER_REG		0x00D2 
#define PWR8_UNDER_REG		0x00D3 
#define PWR8_UP_THRES_REG	0x00D4 
#define PWR8_DN_THRES_REG	0x00D5 
#define PWR8_DIS_THRES_REG	0x00D6 

//#define PWR9_MAX_REG		0x00D8 
//#define PWR9_MIN_REG		0x00D9
//#define PWR9_OVER_REG		0x00DA 
//#define PWR9_UNDER_REG		0x00DB 
//#define PWR9_UP_THRES_REG	0x00DC 
//#define PWR9_DN_THRES_REG	0x00DD 
//#define PWR9_DIS_THRES_REG	0x00DE

#define PWR_LED_CTRL_REG	0x00DF 
//bit 0: red_led_enable;
//bit 1: red_led_value;

#define ZEUS_PWR_LIMITS_SIZE	32 //8 * 4
#define ATLAS_PWR_LIMITS_SIZE	40 //8 * 5

///////////////////////////////////////////////////////////////////
//Read Only:
#define ZEUS_ADC0_REG_ADDR     0x00E0
#define ZEUS_ADC1_REG_ADDR     0x00E1
#define ZEUS_ADC2_REG_ADDR     0x00E2
#define ZEUS_ADC3_REG_ADDR     0x00E3
#define ZEUS_ADC4_REG_ADDR     0x00E4
#define ZEUS_ADC5_REG_ADDR     0x00E5
#define ZEUS_ADC6_REG_ADDR     0x00E6
#define ZEUS_ADC7_REG_ADDR     0x00E7

#define ZEUS_ADC8_REG_ADDR     0x00E8
#define ZEUS_ADC9_REG_ADDR     0x00E9
#define ZEUS_ADC10_REG_ADDR    0x00EA
#define ZEUS_ADC11_REG_ADDR    0x00EB
#define ZEUS_ADC12_REG_ADDR    0x00EC
#define ZEUS_ADC13_REG_ADDR    0x00ED
#define ZEUS_ADC14_REG_ADDR    0x00EE
#define ZEUS_ADC15_REG_ADDR    0x00EF

#define ATLAS_ADC0_REG_ADDR    0x00F0
#define ATLAS_ADC1_REG_ADDR    0x00F1
#define ATLAS_ADC2_REG_ADDR    0x00F2
#define ATLAS_ADC3_REG_ADDR    0x00F3
#define ATLAS_ADC4_REG_ADDR    0x00F4
#define ATLAS_ADC5_REG_ADDR    0x00F5
#define ATLAS_ADC6_REG_ADDR    0x00F6
#define ATLAS_ADC7_REG_ADDR    0x00F7

#define ATLAS_ADC8_REG_ADDR    0x00F8
#define ATLAS_ADC9_REG_ADDR    0x00F9
#define ATLAS_ADC10_REG_ADDR   0x00FA
#define ATLAS_ADC11_REG_ADDR   0x00FB
#define ATLAS_ADC12_REG_ADDR   0x00FC
#define ATLAS_ADC13_REG_ADDR   0x00FD
#define ATLAS_ADC14_REG_ADDR   0x00FE
#define ATLAS_ADC15_REG_ADDR   0x00FF

///////////////////////////////////////////////////////////////////
// THIS SECTION RESERVED FOR KERNEL

//4/4/2016 NOT USED #define ZEUS_PACKET_CTRL_REG	0x0100
//Bit 0 - Reset Memory
//Bit 1 - Enable Write to Memory
#define ZEUS_PACKET_MEM_LEN_REG	0x101  //Read Only

#define ZEUS_POLL_TIMEOUT_REG	0x102

///////////////////////////////////////////////////////////////////

#define ZEUS_POLL_LOCAL_TIMEOUT_REG	0x103

///////////////////////////////////////////////////////////////////

//DAC Direct Access

#define ZEUS_DAC0_REG_ADDR		    0x0200
#define ZEUS_DAC1_REG_ADDR		    0x0201
#define ZEUS_DAC2_REG_ADDR		    0x0202
#define ZEUS_DAC3_REG_ADDR		    0x0203
#define ZEUS_DAC4_REG_ADDR		    0x0204
#define ZEUS_DAC5_REG_ADDR		    0x0205
#define ZEUS_DAC6_REG_ADDR		    0x0206
#define ZEUS_DAC7_REG_ADDR		    0x0207
#define ZEUS_DAC8_REG_ADDR		    0x0208
#define ZEUS_DAC9_REG_ADDR		    0x0209
#define ZEUS_DAC10_REG_ADDR		    0x020A
#define ZEUS_DAC11_REG_ADDR		    0x020B

#define ATLAS_DAC0_REG_ADDR			0x020C
#define ATLAS_DAC2_REG_ADDR			0x020D
#define ATLAS_DAC3_REG_ADDR			0x020E
#define ATLAS_DAC4_REG_ADDR			0x020F
#define ATLAS_DAC5_REG_ADDR			0x0210
#define ATLAS_DAC6_REG_ADDR			0x0211
#define ATLAS_DAC7_REG_ADDR			0x0212
#define ATLAS_DAC8_REG_ADDR			0x0213

#define ZEUS_DAC_DIRECT0_REG_ADDR	0x0214 //32 bit: Start + 7 bits + 24 Bit Control
#define ZEUS_DAC_DIRECT1_REG_ADDR	0x0215 //32 bit: Start + 7 bits + 24 Bit Control
#define ZEUS_DAC_DIRECT2_REG_ADDR	0x0216 //32 bit: Start + 7 bits + 24 Bit Control
#define ZEUS_DAC_DIRECT3_REG_ADDR	0x0217 //32 bit: Start + 7 bits + 24 Bit Control

//DAC Functional Naming
#define ZEUS_PWR0_REG_ADDR			0x0203
#define ZEUS_SW0_REG_ADDR			0x0202

#define ZEUS_PWR1_REG_ADDR			0x0201
#define ZEUS_SW1_REG_ADDR			0x0200

#define ZEUS_PWR2_REG_ADDR			0x0208
#define ZEUS_SW2_REG_ADDR			0x020B

#define ZEUS_PWR3_REG_ADDR			0x0209
#define ZEUS_SW3_REG_ADDR			0x020A

#define ZEUS_PWR4_REG_ADDR			0x0204
#define ZEUS_SW4_REG_ADDR			0x0205

#define ZEUS_BIAS_REG_ADDR			0x0206
#define ZEUS_DACDB_REG_ADDR			0x0207

#define ZEUS_PWR0_TARGET_REG_ADDR	0x0220
#define ZEUS_PWR1_TARGET_REG_ADDR	0x0221
#define ZEUS_PWR2_TARGET_REG_ADDR	0x0222
#define ZEUS_PWR3_TARGET_REG_ADDR	0x0223
#define ZEUS_PWR4_TARGET_REG_ADDR	0x0224	//VDDMIO
#define ZEUS_PWR5_TARGET_REG_ADDR	0x0225	//DAC DB

#define ZEUS_PWR6_TARGET_REG_ADDR	0x0226	//VDD
#define ZEUS_PWR7_TARGET_REG_ADDR	0x0227	//1
#define ZEUS_PWR8_TARGET_REG_ADDR	0x0228	//2
#define ZEUS_PWR9_TARGET_REG_ADDR	0x0229	//3
#define ZEUS_PWR10_TARGET_REG_ADDR	0x022A	//4
#define ZEUS_PWR11_TARGET_REG_ADDR	0x022B	//VAUX0
#define ZEUS_PWR12_TARGET_REG_ADDR	0x022C	//VAUX2
#define ZEUS_PWR13_TARGET_REG_ADDR	0x022D	//Reserved SW

#define ZEUS_AUTO_ADJ_ENABLE		0x0230	//MSW - Adapt and Stop, LSW - Enable
#define ZEUS_AUTO_ADJ_REDIRECT		0x0231
#define ZEUS_AUTO_ADJ_BANDWIDTH		0x0232
#define ZEUS_AUTO_ADJ_LOCK_BANDWIDTH	0x0233

#define ZEUS_AUTO_ADJ_PWR0_LIMIT	0x0234
#define ZEUS_AUTO_ADJ_PWR1_LIMIT	0x0235
#define ZEUS_AUTO_ADJ_PWR2_LIMIT	0x0236
#define ZEUS_AUTO_ADJ_PWR3_LIMIT	0x0237
#define ZEUS_AUTO_ADJ_PWR4_LIMIT	0x0238
#define ZEUS_AUTO_ADJ_PWR5_LIMIT	0x0239
#define ZEUS_AUTO_ADJ_PWR6_LIMIT	0x023A
#define ZEUS_AUTO_ADJ_PWR7_LIMIT	0x023B
#define ZEUS_AUTO_ADJ_PWR8_LIMIT	0x023C
#define ZEUS_AUTO_ADJ_PWR9_LIMIT	0x023D
#define ZEUS_AUTO_ADJ_PWR10_LIMIT	0x023E
#define ZEUS_AUTO_ADJ_PWR11_LIMIT	0x023F
#define ZEUS_AUTO_ADJ_PWR12_LIMIT	0x0240
#define ZEUS_AUTO_ADJ_PWR13_LIMIT	0x0241

#define ZEUS_ADC0_TARGET_REG_ADDR	0x0242
#define ZEUS_ADC1_TARGET_REG_ADDR   0x0243
#define ZEUS_ADC2_TARGET_REG_ADDR   0x0244
#define ZEUS_ADC3_TARGET_REG_ADDR   0x0245

#define ZEUS_ADC4_TARGET_REG_ADDR   0x0246
#define ZEUS_ADC5_TARGET_REG_ADDR   0x0247
#define ZEUS_ADC6_TARGET_REG_ADDR   0x0248
#define ZEUS_ADC7_TARGET_REG_ADDR   0x0249
#define ZEUS_ADC8_TARGET_REG_ADDR   0x024A

#define ZEUS_AUTO_ADJ_STATUS		0x024B
#define ATLAS_AUTO_ADJ_STATUS		0x024C

#define ZEUS_AUTO_ADJ_ADC0_LIMIT	0x0250
#define ZEUS_AUTO_ADJ_ADC1_LIMIT	0x0251
#define ZEUS_AUTO_ADJ_ADC2_LIMIT	0x0252
#define ZEUS_AUTO_ADJ_ADC3_LIMIT	0x0253

#define ZEUS_AUTO_ADJ_ADC4_LIMIT	0x0254
#define ZEUS_AUTO_ADJ_ADC5_LIMIT	0x0255
#define ZEUS_AUTO_ADJ_ADC6_LIMIT	0x0256
#define ZEUS_AUTO_ADJ_ADC7_LIMIT	0x0257
#define ZEUS_AUTO_ADJ_ADC8_LIMIT	0x0258
///////////////////////////////////////////////////////////////////

#define SPI0_STATUS                 0x0260
#define SPI1_STATUS                 0x0261
#define SPI2_STATUS                 0x0262
#define SPI3_STATUS                 0x0263
#define SPI4_STATUS                 0x0264

///////////////////////////////////////////////////////////////////
// SIF Configuration. Move MARVELL_SIF_32_DATA, MARVELL_SIF_32_ADDRESS_MASK, MARVELL_SIF_STEP_SIZE_MASK from PORTx_CTRL here

#define SIF_CONFIG_MEMORY           0x0270

#define SIF_PORT0_INDEX0            0x0270  // 5 ADDR SIZE, 1 DATA SIZE, 2 STEP, 3 PRE, 5 WR WAIT, 5 RD WAIT, 5 WR POST, 5 RD POST = 31 - 1 Register
        #define SIF_32_ADDRESS_MASK	            0x0000001F
        #define SIF_32_DATA_MASK	            0x00000020
        #define SIF_STEP_SIZE_MASK	            0x000000C0
        #define SIF_PRE_BITS_MASK               0x00000700
        #define SIF_WR_WAIT_BITS_MASK           0x0000F800
        #define SIF_RD_WAIT_BITS_MASK           0x001F0000
        #define SIF_WR_POST_BITS_MASK           0x03E00000
        #define SIF_RD_POST_BITS_MASK	        0x7C000000

        #define SIF_32_ADDRESS_SHIFT            0
        #define SIF_32_DATA_SHIFT               5
        #define SIF_STEP_SIZE_SHIFT     	    6
        #define SIF_PRE_BITS_SHIFT              8
        #define SIF_WR_WAIT_BITS_SHIFT          11
        #define SIF_RD_WAIT_BITS_SHIFT          16
        #define SIF_WR_POST_BITS_SHIFT          21
        #define SIF_RD_POST_BITS_SHIFT          26
#define SIF_PORT0_INDEX1            0x0271
#define SIF_PORT0_INDEX2            0x0272
#define SIF_PORT0_INDEX3            0x0273
#define SIF_PORT0_INDEX4            0x0274
#define SIF_PORT0_INDEX5            0x0275
#define SIF_PORT0_INDEX6            0x0276
#define SIF_PORT0_INDEX7            0x0277

#define SIF_PORT1_INDEX0            0x0278
//..
#define SIF_PORT1_INDEX7            0x027F
#define SIF_PORT2_INDEX0            0x0280
#define SIF_PORT3_INDEX0            0x0288
#define SIF_PORT4_INDEX0            0x0290
#define SIF_PORT5_INDEX0            0x0298

///////////////////////////////////////////////////////////////////

#define ZEUS_FPGA_REG_ADDR			0x0300
#define ZEUS_CLK_DB_REG				0x0301

///////////////////////////////////////////////////////////////////
//Timer
#define ZEUS_TIMER_REG_0			0x0310
#define ZEUS_TIMER_REG_1			0x0311

///////////////////////////////////////////////////////////////////
//TL10
#define MNT_MAX_NUMBER_TL10_CONFIG  2
#define PORT0_TL10_CMD_ADDR			0x0320
#define PORT0_TL10_DATA_MASK		0x0321
#define PORT1_TL10_CMD_ADDR			0x0322
#define PORT1_TL10_DATA_MASK		0x0323
#define PORT2_TL10_CMD_ADDR			0x0324
#define PORT2_TL10_DATA_MASK		0x0325
#define PORT3_TL10_CMD_ADDR			0x0326
#define PORT3_TL10_DATA_MASK		0x0327
#define PORT4_TL10_CMD_ADDR			0x0328
#define PORT4_TL10_DATA_MASK		0x0329

///////////////////////////////////////////////////////////////////
//SiLAB Copy of Numerator differences
#define SILAB_CHIP0_COPY_NUM0		0x0330
#define SILAB_CHIP0_COPY_NUM1		0x0331
#define SILAB_CHIP0_COPY_NUM2		0x0332
#define SILAB_CHIP0_COPY_NUM3		0x0333
#define SILAB_CHIP0_COPY_NUM4		0x0334

#define SILAB_CHIP1_COPY_NUM0		0x0340
#define SILAB_CHIP1_COPY_NUM1		0x0341
#define SILAB_CHIP1_COPY_NUM2		0x0342
#define SILAB_CHIP1_COPY_NUM3		0x0343
#define SILAB_CHIP1_COPY_NUM4		0x0344

///////////////////////////////////////////////////////////////////
//XADC FOR APOLLO
#define APOLLO_XADC0_REG_ADDR  0x0400
#define APOLLO_XADC31_REG_ADDR 0x041F

///////////////////////////////////////////////////////////////////
// IJTAG Configuration
#ifndef IJTAG_MAXIMUM_NUMBER_OF_IPS
#define IJTAG_MAXIMUM_NUMBER_OF_IPS                 64      //7/1/2023: Decrease from 128 
#define IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS          32      //7/1/2023: Increase from 16
//#define IJTAG_DR_LENGTH_NUMBER_OF_ITEMS             4       //16 items * 8 = 128 bits / 32 = 4 registers 
#define IJTAG_DR_LENGTH_NUMBER_OF_ITEMS             8       //32 items * 8 = 128 bits / 32 = 8 registers 

#define IJTAG_MAXIMUM_DR_NUMBER_OF_BITS             256
#define IJTAG_MAXIMUM_DESELECT_NUMBER_OF_BITS       250
#define IJTAG_MAXIMUM_DR_NUMBER_OF_DWORDS           8

#define IJTAG_MAXIMUM_NUMBER_OF_BITS                128     //For POST or PRE
#define IJTAG_POST_PRE_NUMBER_OF_ITEMS              8       //4 post + 4 pre
#define IJTAG_POST_or_PRE_NUMBER_OF_ITEMS           4       //4 post / 4 pre

#define IJTAG_DR_DATA_NUMBER_OF_ITEMS               32      // 
#define IJTAG_MAXIMUM_IR_LENGTH                     32
#endif
#define IJTAG_NUMBER_OF_TRST_N_CYCLES               8

#define IJTAG_CURRENT_CNFG_PORT0                    0x76FB   // 31:16 - Port 0 File ID  
                                                             // 7:0   - Port 0 Active IP
#define IJTAG_CURRENT_CNFG_PORT1                    0x76FC   // 31:16 - Port 1 File ID  
                                                             // 7:0  - Port 0 Active IP

#define IJTAG_SELECT_TRST_N_IR_LENGTH_PORT1_PORT0   0x76FD  // 31:24  - Port 1 Time for TRST_N. Number of TRST_N cycles * 2 (Low + High) in IJTAG CLK
                                                            // 23:16  - Port 0 Time for TRST_N
                                                            // 12:8   - PORT 1 IR Command Length - 5 bits -> Up to 32 bits IR
                                                            // 4:0    - PORT 0 IR Command Length - 5 bits -> Up to 32 bits IR
                                                                 
#define IJTAG_SELECT_IR_DATA_PORT0                  0x76FE  // IR Data -> up to 32 bits
#define IJTAG_SELECT_IR_DATA_PORT1                  0x76FF  // IR Data -> up to 32 bits

///////////////////////////////////////////////////////////////////
//POST, PRE LENGTH, DR_NUMBER. 128 Register x 2 ports = 256 registers
#define IJTAG_PRE_POST_LENGTH_DR_NUMBER_IP0_PORT0   0x7700   // Number of DRs for IP0  
//...  
#define IJTAG_PRE_POST_LENGTH_DR_NUMBER_IP127_PORT0 0x777F  

#define IJTAG_PRE_POST_LENGTH_DR_NUMBER_IP0_PORT1   0x7780   // Number of DRs for IP0  
//...  
#define IJTAG_PRE_POST_LENGTH_DR_NUMBER_IP127_PORT1 0x77FF  
                                                                
///////////////////////////////////////////////////////////////////
//IP0 POST PRE Info
#define IJTAG_IP0_POST_DATA0    0x7800  //128 bits. Need 4 32 bit registers.
#define IJTAG_IP0_POST_DATA1    0x7801
#define IJTAG_IP0_POST_DATA2    0x7802
#define IJTAG_IP0_POST_DATA3    0x7803

#define IJTAG_IP0_PRE_DATA0     0x7804  //128 bits. Need 4 32 bit registers.
#define IJTAG_IP0_PRE_DATA1     0x7805
#define IJTAG_IP0_PRE_DATA2     0x7806
#define IJTAG_IP0_PRE_DATA3     0x7807
//...
#define IJTAG_IP127_PRE_DATA3   0x7BFF

//For Port 0: 0x7800 - 0x7BFF
#define IJTAG_IP_POST_PRE_PORT0 0x7800
//For Port 1: 0x7C00 - 0x7FFF
#define IJTAG_IP_POST_PRE_PORT1 0x7C00

////////////////////////////////////////////////////////////////////////
//IP0 DR Length. 512 Registers x2 ports - 1K Registers 
//The middle names are not correct, since we increased to 32 DRs, but decreased number of IPs to 64
#define IJTAG_SELECT_IP0_DR3_0_LENGTH       0x8000  //8 Bit Length for each DR. 4 DRs Total 
#define IJTAG_SELECT_IP0_DR7_4_LENGTH       0x8001 
#define IJTAG_SELECT_IP0_DR11_8_LENGTH      0x8002 
#define IJTAG_SELECT_IP0_DR15_12_LENGTH     0x8003 
//.....
#define IJTAG_SELECT_IP127_DR3_0_LENGTH     0x81FC  //8 Bit Length for each DR. 4 DRs Total 
#define IJTAG_SELECT_IP127_DR7_4_LENGTH     0x81FD 
#define IJTAG_SELECT_IP127_DR11_8_LENGTH    0x81FE 
#define IJTAG_SELECT_IP127_DR15_12_LENGTH   0x81FF 

//For Port 0: 0x8000 - 0x81FF
#define IJTAG_SELECT_IP_DR_LENGTH_PORT0     0x8000  //512 Registers -> Total 64 (IP) * 32 (DR) / 4 (4 DR Length per register)
//For Port 1: 0x8200 - 0x83FF
#define IJTAG_SELECT_IP_DR_LENGTH_PORT1     0x8200

////////////////////////////////////////////////////////////////////////
//0x8400 - 0x8FFF - Free 3K Addresses
#define IJTAG_PRAM_IP_DR_BIT_NUMBER_PORT0   0x9000 // 64 Registers to specify bit number for each IP to set 1
#define IJTAG_PRAM_IP_DR_BIT_NUMBER_PORT1   0x9040
#define IJTAG_DESELECT_IP_DR_LENGTH_PORT0   0x9080 // 64 Registers 8 bit each -> maximum 256 bits for data
#define IJTAG_DESELECT_IP_DR_LENGTH_PORT1   0x90C0
//0x9100 - 0x91FF - Free 256 Addresses

#define IJTAG_DESELECT_IP_DR_DATA_PORT0     0x9200 // 512 Registers -> 64 (IPs) * 256 Bits / 32 bits per Register
#define IJTAG_DESELECT_IP_DR_DATA_PORT1     0x9400
//0x9600 - 0x9FFF - Free 2560 Addresses
 
////////////////////////////////////////////////////////////////////////
//IP0 DR Data. 32 registers x 128 = 4K * 2 ports = 8K **** X2 7/14/2023                                                         
//The middle names are not correct, since we increased to 32 DRs, but decreased number of IPs to 64
#define IJTAG_SELECT_IP0_DR_DATA        0xA000  //32 registers for one IP 
//...
#define IJTAG_SELECT_IP0_DR15_DATA      0xA01F 
//...
#define IJTAG_SELECT_IP127_DR15_DATA0   0xAFFF

//For Port 0: 0xA000 - 0xBFFF: 8K
#define IJTAG_SELECT_IP_DR_DATA_PORT0   0xA000
//For Port 1: 0xC000 - 0xDFFF: 8K
#define IJTAG_SELECT_IP_DR_DATA_PORT1   0xC000

////////////////////////////////////////////////////////////////////////
//0xE000 - 0xEFFF - Free 1K Addresses
//0xF000 - 0xFFFF - Free 1K Addresses
 
///////////////////////////////////////////////////////////////////

#define ZEUS_ADC_TEST_MEM_ADDR		0x80000000
#define ZEUS_PACKET_READ_MEM_ADDR	0x40000000

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

#define SLOW_ADDR_MASK			0xFFFF0000

#define SI_ADDR_SPACE			0x00010000
#define SI_ADDR_SPACE1			0x00020000

//#define ZEUS_JTAG_ADDR_SPACE    0x00020000
//#define ZEUS_JTAG_DIRECT_ADDR   0x00020000

//8/15/2019 Removed #define SPI_EEPROM_SPACE		0x00040000
//#define SIF4_SPACE				0x00060000

#define DPR_ADDR_SPACE0_BIT     0x00040000
#define DPR_ADDR_SPACE1_BIT     0x00080000

//MDIO

//MDIO Address:
//3 bits + 5 bits + 5 bits + 16 bit
/*
#define ZEUS_MDIO0_START_ADDR   0x20000000
#define ZEUS_MDIO1_START_ADDR   0x24000000
#define ZEUS_MDIO2_START_ADDR   0x28000000
#define ZEUS_MDIO3_START_ADDR   0x2C000000

#define ZEUS_MDIO4_START_ADDR   0x30000000

#define ZEUS_MDIO_ADDR_SPACE    0x20000000
*/
#define ZEUS_MAX_PORT_NUMBER	5

#define JANUS_SPI_ADDR_SPACE	0x02000000

#define JANUS_SPI0_START_ADDR	0x02000000
#define JANUS_SPI1_START_ADDR	0x02400000
#define JANUS_SPI2_START_ADDR	0x02800000
#define JANUS_SPI3_START_ADDR	0x02C00000
#define JANUS_SPI4_START_ADDR	0x03000000


#endif


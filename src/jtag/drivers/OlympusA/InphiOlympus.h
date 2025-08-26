// Copyright (c) 2014 - 2021 INPHI 
// Copyright (c) 2021 - 2024 Marvell
// Olympus Public header.

#ifndef __InphiOlympus__h_
#define __InphiOlympus__h_

#ifdef __cplusplus
extern "C" {  // for export C function in C++ source code
#endif

#if !defined(Q_OS_MAC) && !defined(Q_OS_LINUX)
#include <windows.h>
#else
#   define MAX_PATH 260
typedef wchar_t WCHAR;
typedef void *HANDLE;
typedef HANDLE *PHANDLE;
typedef unsigned long DWORD;
typedef void* LPVOID;
typedef int BOOL;
typedef unsigned short UINT16;
typedef unsigned char  UINT8;
typedef void* PVOID;
typedef long LONG;
typedef unsigned long ULONG;
typedef unsigned long long __int64;
#endif

#define MAX_MNT_NUMBER			16  //Note by SCM: Max number of Zeus boards on one computer
#define MAX_DEVICE_TYPE_NUMBER	5	//Soft, GaiaPlus, Zeus-1,...

#define	MNT_ALL_DEVICES_ID		-1

//List of MBs
#define MNT_SOFT_ID		0	//1
#define GAIAPLUS_ID		1	
#define ZEUS1_ID		2	
#define ZEUS2_ID		3	//2
#define ZEUS3_ID		4	//3
#define ZEUS4_ID		5
#define ZEUS5_ID		6
#define FX3_ID			9	//4

#define HERA1_ID 		10		
#define HERA2_ID 		11	//5	//Use it for Janus
#define HERA3_ID 		12	

#define HERCULES1_ID 	0x20
#define HERCULES2_ID 	0x21
#define HERCULES3_ID 	0x22

#define APOLLO1_ID 		0x30	//6
#define APOLLO2_ID 		0x31
#define APOLLO3_ID 		0x33

#define ZEUS_F_ID	 		0x4D420002
#define KAR_F_ID	 		0x4D420012
#define JANUS1_F_ID 		0x4A410001
#define APOLLO1_F_ID 		0x41500001

//List of Regression
#define PHANXIPEAK_ID		0x0000	//Spica
#define TWINPEAK_ID			0x0001	//2 Porrima chips
#define SNOWPEAK_ID			0x0002	//Alcor
#define MONTEGRAPPA_ID		0x0003	//Vega 15x15
#define LANGPEAK_ID			0x0004	//Polaris2
#define PUTAPEAK_ID			0x0005	//Lynx
#define BROTHERSPEAK_ID		0x0006	//Atlas Chip
#define MERUPEAK_ID			0x0007	//STC-56 (Syrma Test Chip)
#define DIAMONDPEAK_ID		0x0008	//Odyssey
#define SILVERPEAK_ID		0x0009	//Expedition

#define MONZA_ID			0x000A	//Monza. Should we add new category? Voyager / STC05R4

#define CASTLEPEAK_ID		0x000B	//Blazer / STC03R2

#define BROOKLYN_ID			0x1001
#define RIALTO_ID			0x1002
#define MANHATTAN_ID		0x1003
#define HELIX_ID			0x1004

#define MEKONG_ID			0x2001
#define YUKON_ID			0x2002

//List of DBs
#define URANUS_2_ID		0x0001	//CDR Test Board
#define URANUS_3_ID		0x0002	//"LoopBakck" Test Board. No Chips
#define URANUS_4_ID		0x0010	//"FPGA" Test Board. No Chips
#define URANUS_5_ID		0x0011	//Break-out board
#define URANUS_6_ID		0x0012	//External Power Plug in board for Atlas
#define URANUS_7_ID		0x0013	//Atlas Calibration board
#define URANUS_8_ID		0x0014	//Zeus Calibration board
#define URANUS_9_ID		0x0015	//Cisco Break-out board
#define URANUS_9_GENERIC_ID		0x0016	//Generic Uranus-9
#define URANUS_10_ID	0x0017	//Eris-4 Test Board
#define URANUS_11_ID	0x0018	//
#define URANUS_12_ID	0x0019	//

#define ATHENA_0_ID		0x0000
#define ATHENA_1_ID		0x0003	//LPCDR 10x10 solder down + socket
#define ATHENA_2_ID		0x0004	//LPCDR 10x10 one solder down chip
#define ATHENA_3_ID		0x0005	//LPCDR 6x6 socket + solder down (10x10)
#define ATHENA_4_ID		0x0006	//LPCDR 6x6 one solder down chip
#define ATHENA_5_ID		0x0007	//LPCDR 10x10 2 solder down chip
#define ATHENA_6_ID		0x0008	//LPCDR 6x6 2 solder down chip
#define ATHENA_7_ID		0x0009	//LPCDR 10x10 one solder down chip (ARDENT Connector)
#define ATHENA_8_ID		0x000A	//LPCDR 6x6 one solder down chip (ARDENT Connector)
#define ATHENA_9_ID		0x000B	//LPCDR 10x10 solder down + socket (Copy of Athena 1 - modify for different temperature head)
#define ATHENA_10_ID	0x000C	//LPCDR 6x6 socket + solder down (10x10) (Copy of Athena 3 - modify for different temperature head)
#define ATHENA_11_ID	0x000D	//LPCDR x1 socket
#define ATHENA_12_ID	0x000E	//Reserved
#define ATHENA_13_ID	0x000F	//Reserved
#define ATHENA_14_ID	0x0020	//Reserved
#define ATHENA_15_ID	0x0021	//Reserved

#define ARES_0_ID		0x0100	//PAM4 Socket board
#define ARES_1_ID		0x0101	//PAM4 Socket board
#define ARES_1B_ID		0x0101	//PAM4 Socket board. Same ID as ARES-1 Different PCB Revision
#define ARES_2_ID		0x0102	//PAM4 Solder
#define ARES_2B_ID		0x0102	//PAM4 Solder. Same ID as ARES-2 Different PCB Revision
#define ARES_2C_ID		0x0102	//PAM4 Solder. Same ID as ARES-2B Different PCB Revision

#define ERIS_0_ID		0x0200
#define ERIS_1_ID		0x0201	//Power Supply Interposer board.
#define ERIS_2_ID		0x0202	//Power Supply Interposer board. for Caltrop Only.
#define ERIS_2B_ID		0x0202	//Power Supply Interposer board. for Caltrop Only.
#define ERIS_2C_ID		0x0202	//Power Supply Interposer board. for Caltrop Only.
#define ERIS_3_ID		0x0203	//Power Supply Interposer board. for Cronus Only.
#define ERIS_4_ID		0x0204	//Power Supply Interposer board.

#define THEMIS_0_ID		0x0300	//
#define THEMIS_1_ID		0x0301	//Sumatra-C board
#define THEMIS_2_ID		0x0302	//Sumatra-C board

#define HYPERION_0_ID	0x0400	//
#define HYPERION_1_ID	0x0401	//2 LP-CDR chips 10x10 + QSFP28 
#define HYPERION_2_ID	0x0402	//

#define CETO_0_ID		0x0500	//
#define CETO_1_ID		0x0501	//GB2-F 16x16
#define CETO_2_ID		0x0502	//GB2-F 19x19

#define HYDRA_0_ID		0x0600	//
#define HYDRA_1_ID		0x0601	//Sumatra-D

#define NOTUS_0_ID		0x0700
#define NOTUS_1_ID		0x0701	//NOTUS-1

#define PLEIADES_0_ID	0x0800
#define PLEIADES_1_ID	0x0801	//Pleiades-A	
#define PLEIADES_2_ID	0x0802	//Pleiades-B
#define PLEIADES_3_ID	0x0803	//Pleiades-C

#define CALTROP_0_ID	0x0900
#define CALTROP_1_ID	0x0901	//Caltrop
#define CALTROP_2_ID	0x0902	//Caltrop-2
#define CALTROP_3_ID	0x0903	//Caltrop-3
#define CALTROP_4_ID	0x0904	//Caltrop-4
#define CALTROP_5_ID	0x0905	//Caltrop-5

#define ADONIS_0_ID		0x0A00
#define ADONIS_1_ID		0x0A01	//Adonis

#define ATLAS_0_ID		0x0B00
#define ATLAS_1_ID		0x0B01	//Atlas
#define ATLAS_2_ID		0x0B02	
#define ATLAS_3_ID		0x0B03	

#define PONTUS_0_ID		0x0C00
#define PONTUS_1_ID		0x0C01	//Pontus

#define HELIOS_0_ID		0x0D00
#define HELIOS_1_ID		0x0D01	//Helios

#define CRONUS_0_ID		0x0E00
#define CRONUS_1_ID		0x0E01	//Cronus
#define CRONUS_2_ID		0x0E02
#define CRONUS_3_ID		0x0E03
#define CRONUS_4_ID		0x0E04

#define SANTACLARA_0_ID		0x0F00
#define SANTACLARA_1_ID		0x0F01	//Santa Clara

#define SANJOSE_0_ID		0x1000
#define SANJOSE_1_ID		0x1001	//Santa Jose

#define ANTEVORTA_0_ID		0x1100
#define ANTEVORTA_1_ID		0x1101	//Antevorta
#define ANTEVORTA_2_ID		0x1102
#define ANTEVORTA_3_ID		0x1103

#define TAURUS_0_ID			0x1200
#define TAURUS_1_ID			0x1201	//	Taurus-1
#define TAURUS_2_ID			0x1202	//	Taurus-2

#define DRACO_0_ID			0x1300	//	DRACO
#define DRACO_1_ID			0x1301	//	DRACO-1

#define CASCADE_0_ID		0x1400
#define CASCADE_1_ID		0x1401	//	CASCADE-1

#define GAIA_0_ID			0x1500	//GAIA-0
#define GAIA_1_ID			0x1501	//GAIA-1
#define GAIA_3_ID			0x1503	//GAIA-3

#define ARTEMIS_0_ID		0x1600
#define ARTEMIS_1_ID		0x1601	//	ARTEMIS-1
#define ARTEMIS_2_ID		0x1602	//	ARTEMIS-2
#define ARTEMIS_3_ID		0x1603	//	ARTEMIS-3
#define ARTEMIS_4_ID		0x1604	//	ARTEMIS-4

#define LEO_0_ID			0x1700	//	LEO
#define LEO_1_ID			0x1701	//	LEO-1

#define LUNAR_0_ID			0x1800
#define LUNAR_1_ID			0x1801	//	LUNAR-1
#define LUNAR_2_ID			0x1802

#define DEMETER_0_ID		0x1900
#define DEMETER_1_ID		0x1901	//	DEMETER-1

#define CERES_0_ID			0x1A00
#define CERES_1_ID			0x1A01	//	CERES-1
#define CERES_3_ID			0x1A03	//	CERES-3
#define CERES_6_ID			0x1A06	//	CERES-6 - Spica 5n
#define CERES_7_ID			0x1A07	//	CERES-7 - Spica 5n

#define MYRRA_0_ID			0x1B00
#define MYRRA_1_ID			0x1B01	//	MYRRA-1

#define ZEPHYROS_0_ID		0x1C00
#define ZEPHYROS_1_ID		0x1C01	//	ZEPHYROS-1

#define CRATER_0_ID			0x1D00
#define CRATER_1_ID			0x1D01	//	CRATER-1
#define CRATER_2_ID			0x1D02

#define SOPHIA_0_ID			0x1E00
#define SOPHIA_1_ID			0x1E01	//	SOPHIA-1
#define SOPHIA_2_ID			0x1E02	//	SOPHIA-2 + RAPTOR

#define BRUNELLO_0_ID		0x1F00
#define BRUNELLO_1_ID		0x1F01	//	Brunello

#define HECTOR_0_ID			0x2000
#define HECTOR_1_ID			0x2001	//	Hector

#define PROMETHEUS_0_ID		0x2100
#define PROMETHEUS_1_ID		0x2101	//	Atlas in package
#define PROMETHEUS_2_ID		0x2102	//	Atlas Bare Die
#define PROMETHEUS_3_ID		0x2103	//	Atlas 

#define RAPTOR_0_ID			0x2200
#define RAPTOR_2_ID			0x2202	//	Raptor-2

#define ODYSSEY_0_ID		0x2300	//	Odyssey Family
#define ODYSSEY_1_ID		0x2301	//	Odyssey-1
#define ODYSSEY_2_ID		0x2302	//	Odyssey-2
#define ODYSSEY_3_ID		0x2303	//	Odyssey-3
#define ODYSSEY_4_ID		0x2304	//	Odyssey-4
#define ODYSSEY_5_ID		0x2305	//	Odyssey-5

#define TUNDRA_0_ID			0x2400
#define TUNDRA_1_ID			0x2401	//	Tundra-1
#define TUNDRA_2_ID			0x2402	//	Tundra-2
#define TUNDRA_3_ID			0x2403	//	Tundra-3

#define SPRINTER_0_ID		0x2500	//	Sprinter Family

#define HESTIA_0_ID			0x2600	//	Hestia Family for Nova

#define EXPEDITION_0_ID		0x2700	//	Expedition

#define VOYAGER_0_ID		0x2800	//	Voyager (STC 5nm R4 2D)
#define VOYAGER_1_ID		0x2801	//	STC05R4A
#define VOYAGER_2_ID		0x2802	//	STC05R4C
#define VOYAGER_3_ID		0x2803	//	STC05R4B1
#define VOYAGER_4_ID		0x2804	//	STC05R4B2
#define VOYAGER_5_ID		0x2805	//	STC05R4B1
#define VOYAGER_6_ID		0x2806	//	STC05R4B2

#define ANDROMEDA_0_ID		0x2900	//	Perseus-5n 800G AB/CD 17x17mm

#define BLAZER_0_ID			0x2A00	//	Blazer
#define BLAZER_1_ID			0x2A01	//
#define BLAZER_2_ID			0x2A02	//
#define BLAZER_3_ID			0x2A03	//
#define BLAZER_4_ID			0x2A04	//
#define BLAZER_5_ID			0x2A05	//

#define TRITON_0_ID			0x2B00	//	Triton for Aquila  

#define THALLO_0_ID			0x2C00	//	Leo-1 chip

#define ARIYA_0_ID			0x2D00	//	STC3R3
#define ARIYA_1_ID			0x2D01	//	
#define ARIYA_2_ID			0x2D02	//	
#define ARIYA_3_ID			0x2D03	//	
#define ARIYA_4_ID			0x2D04	//	
#define ARIYA_5_ID			0x2D05	//	
#define ARIYA_6_ID			0x2D06	//	
#define ARIYA_7_ID			0x2D07	//	


//From Steve M.
//List of breakout projects (using Uranus 5 or Apollo) TODO: Delete, replace with check for
//chip id = EMPTY_CHIP_ID and customer = JUNIPER_PAM_ID
#define BREAKOUT_PAM4_MDIO_0    (0x1)

//List Of Chips
#define EMPTY_CHIP_ID		0x0		//NO CHIP
#define CDR_ID				0x1		//CDR
#define LPCDR_10x10_ID		0x2		//LP-CDR 10x10 
#define LPCDR_6x6_ID		0x3		//LP-CDR 6x6
#define GB2_F_16x16_ID		0x4		//GB2-F 16x16
#define GB2_F_19x19_ID		0x5		//GB2-F 19x19
#define PAM4_ID				0x6		//PAM4
#define SUMATRA_C_ID		0x7		//Sumatra-C
#define SUMATRA_D_ID		0x8		//Sumatra-D
#define HYPERION_ID			0x9		//Hyperion
#define LPCDR_x1_ID			0xA		//LP-CDR x1
#define PAM4A1_ID			0xB		//PAM4 A1 IN015025
#define GB1_16x16_ID		0xC		//GB1 16x16
#define GB1_19x19_ID		0xD		//GB1 19x19
#define PAM4B0_ID			0xE		//PAM4 B0
#define LPCDR_A1_10x10_ID	0xF		//LP-CDR A1
#define LPCDR_A1_6x6_ID		0x10	//LP-CDR A1
#define LPCDR_B0_10x10_ID	0x11	//LP-CDR B0
#define LPCDR_B0_6x6_ID		0x12	//LP-CDR B0
#define LPCDR_A2_10x10_ID	0x13	//LP-CDR A2
#define LPCDR_A2_6x6_ID		0x14	//LP-CDR A2
#define PAM4A2_ID			0x15	//PAM4 A2
#define LPCDR_A3_10x10_ID	0x16	//LP-CDR A3
#define LPCDR_A3_6x6_ID		0x17	//LP-CDR A3
#define PAM4X4_B0_ID		0x18	//PAM 400G MCM
#define POLARIS_A0_4x4R_ID	0x19	//Polaris A0 4x4 Re-timer (Adonis)
#define PAM4B1_ID			0x1A	//PAM4 B1
#define PAM4X4_B1_ID		0x1B	//PAM 400G MCM B1 (Caltrop)
#define POLARIS_LC_A0_ID				0x1C	//Polaris A0 LC Package for Cisco
#define POLARIS_400G_CFP8_GB_A0_ID		0x1D	//Polaris A0 400G CFP8 GB (Pontus)
#define POLARIS_OSFP_BOTTOM_RT_A0_ID	0x1E	//Polaris A0 OSFP BOTTOM SIDE RETIMER (Helios-1)
#define VEGA_A0_ID						0x1F	//VEGA (Cronus-1)
#define POLARIS_A0_4x4R_TOP_ID			0x20	//Polaris A0 4x4 Re-timer Top Side (Adonis)
#define POLARIS_QSFP_DD_TOP_RT_A0_ID	0x21	//Polaris A0 OSFP TOP SIDE RETIMER (Helios-2)
#define LPCDR_B1_10x10_ID	0x22	//LP-CDR B1
#define LPCDR_B1_6x6_ID		0x23	//LP-CDR B1

#define POLARIS_SFP56_A0_ID				0x24	//Polaris A0 SFP56 (Adonis-3)

#define POLARIS_B0_4x4R_ID				0x25	//Polaris B0 4x4 Re-timer (Adonis-1)
#define POLARIS_B0_4x4R_TOP_ID			0x26	//Polaris B0 4x4 Re-timer Top Side (Adonis-2)
#define POLARIS_400G_CFP8_GB_B0_ID		0x27	//Polaris B0 400G CFP8 GB (Pontus)
#define POLARIS_LC_B0_ID				0x28	//Polaris B0 LC Package for Cisco
#define POLARIS_OSFP_BOTTOM_RT_B0_ID	0x29	//Polaris B0 OSFP BOTTOM SIDE RETIMER (Helios-1)
#define POLARIS_QSFP_DD_TOP_RT_B0_ID	0x2A	//Polaris B0 OSFP TOP SIDE RETIMER (Helios-2)
#define POLARIS_SFP56_B0_ID				0x2B	//Polaris B0 SFP56 (Adonis-3)

#define PORRIMA_400G_GB_A0_ID			0x2C	//Porrima A0 (Antevorta-1)

#define VEGA_400G_GB_A0_ID				0x2D	//Dual VEGA (Cronus-2)
#define VEGA_CDR5L_A0_ID				0x2E	//VEGA 23x23 (Cronus-3)

#define PORRIMA_100G_A0_ID				0x2F	//Porrima 8x10 (Antevorta-2)

#define VEGA_B0_ID						0x30	//VEGA (Cronus-1)
#define VEGA_400G_GB_B0_ID				0x31	//Dual VEGA (Cronus-2)
#define VEGA_CDR5L_B0_ID				0x32	//VEGA 23x23 (Cronus-3)

#define PORRIMA_400G_GB_A1_ID			0x33	//Porrima (Antevorta-1)
#define PORRIMA_100G_A1_ID				0x34	//Porrima 8x10 (Antevorta-2)

#define PORRIMA_DR4_EML_SBT_ID			0x35	//Porrima 12x13 DR 400G EML (Antevorta-3)

#define POLARIS_SFP56_DD_A0_ID			0x36	//Polaris A0 SFP56_DD 6x7 (Adonis-4)

#define PORRIMA_DR_400G_ID				0x37	//Porrima 10x13 DR (Antevorta-1)

#define PORRIMA_DR_100G_EML_SBT_ID		0x38	//Porrima 10.5x10.5 DR EML SBT (Antevorta-4)

#define STC_56_ID						0x39	//STC-56 pin compatible with Adonis-2 (Polaris Top Side 8x10) (Artemis-1)

#define POLARIS_8x10_GB_TOP_B0_ID		0x3A	//Polaris B0 8x10 package 4:2 GB Top Side (Adonis-2)

#define PORRIMA_DR4_EML_DBT_ID			0x3B	//Porrima 12x13 DR 400G EML (Antevorta-3 / 3B)

#define PORRIMA_DR_100G_EML_DBT_ID		0x3C	//Porrima 10.5x10.5 DR EML DBT (Antevorta-4)

#define SKYWALKER_TVA0_ID				0x3D	//Skywalker (Lunar-1, 2)

#define CTC_112_ID						0x3E	//CTC-112 (Artemis-2)

#define CTC2_ID							0x3F	//Dual CTC-112 (Artemis-3)
#define VEGA_LF_ID						0x40	//Vega LF 23x23 (Cronus-4)
#define SPICA_800G_TOP_ID				0x41	//Spica 800G NON-EML Top Side (Ceres-2)
#define SPICA_800G_BOT_ID				0x42	//Spica 800G NON-EML Bottom Side (Ceres-2)
#define SPICA_800G_EML_TOP_ID			0x43	//Spica 800G EML Top Side (Ceres-2)
#define SPICA_800G_EML_BOT_ID			0x44	//Spica 800G EML Bottom Side (Ceres-2)
#define STC_B0_56_ID					0x45	//STC-56 (Artemis-1)
#define PORRIMA_GEN3_DR4_EML_DBT_ID		0x46	//Porrima-2 12x13 DR 400G EML (Antevorta-3 / 3B)
#define SPICA_800G_SIPHO_BTM_ID			0x47	//Spica 800G SiPho Bottom Side 14x14 (Ceres-3 / PhanXiPeak-1)

#define SPICA_800G_SIPHO_TOP_ID			0x48	//SPICA 800G TOP SIPHO 15x14 (Ceres-1)

#define POLARIS_GEN2_400G_STANDARD_ID	0x49	//Polaris Gen2 400G Standard Driver (Myrra-1)
#define ALCOR_A0_8x7_SIPHO_ID			0x4A	//Alcor (Zephyros-1)
#define MIRA_A0_E1_14x14_ID				0x4B	//Mira / Skywalker (Ceres-3)
#define STC_C0_56_ID					0x4C	//STC-56 (Artemis-1)
#define CTC_A1_112_ID					0x4D	//CTC-112 A1 (Artemis-2)
#define ALCOR_A0_8x7_EML_ID				0x4E	//Alcor (Zephyros-1)
#define PORRIMA_GEN3_400G_ID			0x4F	//Porrima GEN 3 (Antevorta-1)
#define CTC_B0_112_ID					0x50	//CTC-112 B0 (Artemis-2)
#define CTC_LP_112_ID					0x51	//CTC-112 LP (Artemis-2)

#define BRUNELLO_CHIP_ID				0x52	//Brunello connected to Sophia
#define SATC_A0_56_ID					0x53	//SATC (Artemis-1 / MeruPeak-1)
#define PLR2_A0_8x10_ID					0x54	//PLR2 "Mira" (Adonis-2C, LangPeak-1)

#define CTC3_ID							0x55	//CTC-112 (Artemis-4)
#define LYNX_A0_ID						0x56	//Lynx (Hector-1, PuTaPeak-1)

#define SPICAPLUS_800G_EML_BOTTOM_ID	0x57	// Ceres-2
#define SPICAPLUS_800G_EML_OIM_BOTTOM_ID	0x58	// Ceres-2
#define SPICAPLUS_400G_EML_BOTTOM_ID	0x59	// Ceres-2
#define SPICAPLUS_800G_EML_TOP_ID		0x5A	// Ceres-2

#define SPICAPLUS_800G_SIPHO_BOTTOM_ID	0x5B	// Ceres-3 / PhanXiPeak-1
#define SPICAPLUS_800G_SIPHO_OIM_BOTTOM_ID	0x5C	// Ceres-3 / PhanXiPeak-1
#define SPICAPLUS_800G_STD_BOTTOM_ID	0x5D	// Ceres-3 / PhanXiPeak-1
#define SPICAPLUS_400G_STD_BOTTOM_ID	0x5E	// Ceres-3 / PhanXiPeak-1

#define PORRIMA_GEN4_400G_STD_TOP_ID	0x5F	// Antevorta-1 / TwinPeak-1
#define PORRIMA_GEN4_400G_SIPHO_TOP_ID	0x60	// Antevorta-1 / TwinPeak-1
#define PORRIMA_GEN4_400G_EML_TOP_ID	0x61	// Antevorta-3

#define PORRIMA_GEN4_400G_EML_TOP_ID	0x61	// Antevorta-3

#define ATLAS_DIE_EQ_A0					0x62	// Prometheus-2. First Bare Die
#define ATLAS_DIE_IQ_A0					0x63	// Prometheus-2. Second Bare Die

#define STC05R1_A0						0x64	// Sophia-2 + Raptor

#define ATLAS_IQ_A0						0x65	// Prometheus-1. Assembly .1.2
#define ATLAS_EQ_A0						0x66	// Prometheus-1. Assembly .3 
#define ATLAS_IQ_EQ_A0					0x67	// Prometheus-1. Assembly .1.1
#define ATLAS_WLB_A0					0x68	// Prometheus-3. Assembly .1

#define SPICAPLUS_400G_EML_TOP_ID		0x69	// Ceres-4
#define SPICAPLUS_400G_SIPHO_TOP_ID		0x6A	// Ceres-5

#define STC05R3_A0						0x6B	// Odyssey - DiamondPeak
#define STC05R3_2DIES_A0				0x6C	// Odyssey - DiamondPeak

#define SPICAPLUS_800G_SIPHO_TOP_ID		0x6D	// SPICA PLUS 800G TOP SIPHO 15x14 (Ceres-1)

#define STC05R3_COWOS_2P5D_A0			0x6E	// STC05 R3 CoWoS 2.5D
#define STC05R3_A1						0x6F	// Odyssey - DiamondPeak
#define STC05R3_2DIES_A1				0x70	// Odyssey - DiamondPeak
#define STC05R3_COWOS_2P5D_A1			0x71	// STC05 R3 CoWoS 2.5D

#define SPICA5N_SIPHO_TOP				0x72	// Spica 5N SiPho Top Ceres-6
#define SPICA5N_STD_TOP					0x73	// Spica 5N STD Top Ceres-6
#define SPICA5N_EML_TOP					0x74	// Spica 5N EML Top Ceres-7

#define ARCTIC_SCM_A0					0x75	// Arctic - Sprinter-1 

#define STC03C1_A0						0x76	// Expedition-1, SilverPeak-1

#define NOVA_800G_SIPHO					0x77	// Nova SiPho	Hestia-1
#define NOVA_800G_STD					0x78	// Nova STD		Hestia-1
#define NOVA_800G_EML					0x79	// Nova EML		Hestia-2

#define LYNX_200G_B0					0x7A	// Hector-1
#define LYNX_400G_B0					0x7B	// Hector-1
#define LYNX_800G_B0					0x7C	// Hector-1

#define VEGA_10GC_B0_ID					0x7D	// VEGA (Cronus-1)

#define REDUX_A0_ID						0x7E	// Redux (Sprinter-2)

#define STC03C3_A0 						0x7F	// Expedition-2

#define STC05R4A						0x80	// Voyager-1
#define STC05R4B1						0x81	// Voyager-3, Voyager-5
#define STC05R4C						0x82	// Voyager-2, Monza-1, Monza-2

#define PERSEUS_400G_SIPHO_TIA_A		0x83	// Andromeda-1
#define PERSEUS_400G_VCSEL_TIA_C		0x84	// Andromeda-1 or Andromeda-2

#define PERSEUS_400G_VCSEL_B			0x85	// Andromeda-2
#define PERSEUS_400G_SIPHO_D			0x86	// Andromeda-2 or Andromeda-3

#define NOVA_B0_800G_SIPHO				0x87	// Nova B0 SiPho	Hestia-1
#define NOVA_B0_800G_STD				0x88	// Nova B0 STD		Hestia-1
#define NOVA_B0_800G_EML				0x89	// Nova B0 EML		Hestia-2

#define NOVA2_A0_800G_SIPHO				0x8A	// Nova 2 A0 SiPho	Hestia-5
#define NOVA2_A0_800G_STD				0x8B	// Nova 2 A0 STD	Hestia-5
#define NOVA2_A0_800G_EML				0x8C	// Nova 2 A0 EML	Hestia-5

#define STC03R2A						0x8D	// Blazer-1 - 64G, XSR
#define STC03R2C						0x8E	// Blazer-2 - 112G
#define STC03R2B1						0x8F	// Blazer-3 - 3nm-3nm
#define STC03R2B2						0x90	// Blazer-4 - 5nm-3nm
#define STC03R2B3						0x91	// Blazer-5 - 3nm-5nm

#define LEO1							0x92	// Thallo-1
#define LEO2							0x93	//

#define STC05R4B2						0x94	// Voyager-4,  Voyager-6

#define STC03R1_KUDO_1P0				0x95	// Expedition-1

#define NUMBER_OF_SUPPORTED_CHIPS		0x96

#define NOTCORNER			0x0
#define SLOWSLOW			0x1
#define SLOWFAST			0x2
#define FASTSLOW			0x3
#define FASTFAST			0x4
#define TYPCORNER			0x5
#define V1CORNER			0x6
#define V2CORNER			0x7

#define NUMBER_OF_CORNER	0x8

//Chip Types
#define UNKOWN_CHIP			0x0
#define LPCDR_CHIP			0x1
#define PAM4_CHIP			0x2
#define POLARIS_A_CHIP		0x3
#define VEGA_CHIP			0x4
#define PORRIMA_A_CHIP		0x5
#define POLARIS_B_CHIP		0x6
#define PORRIMA_B_CHIP		0x7
#define SYRMA_A_CHIP		0x8				//SYRMA_B_CHIP SAME ID
#define SUN_A_CHIP			0x9
#define CAPELLA_A_CHIP		0xA
#define CANOPUS_A_CHIP		0xB
#define SPICA_A_CHIP		0xC
#define MIRA_A_CHIP			0xD

#define MAX_NUMBER_OF_CHIP_TYPE	0xE

//iJTAF boards

#define IJTAG_MODEL_ID		0x0000
#define IJTAG_ARCTIC_ID		0x0001
#define IJTAG_REDUX_ID		0x0002
#define IJTAG_TERMINUS_ID	0x0003
#define IJTAG_CAYMAN_ID		0x0004
#define IJTAG_HAWKOWL_ID	0x0005

#define NUMBER_OF_IJTAG_TYPE	0x0006

//Power Type Names
#define POWERTYPE_DIRECT_ID	0x00
#define POWERTYPE_GPIO_ID	0x01
#define POWERTYPE_PULLUP_ID	0x02
#define POWERTYPE_CLOCK_ID	0x03
#define POWERTYPE_DC_DC_ID	0x04
#define POWERTYPE_AVS_ID	0x05
#define POWERTYPE_FAN_ID	0x06
#define POWERTYPE_FIXED_ID	0x07
#define POWERTYPE_NO_CONNECT_ID	0x08
#define POWERTYPE_UNDEFINED_ID	0x09
#define POWERTYPE_SHARED_ID	0x0A

#define NUMBER_OF_POWER_TYP	0x0B


//List of Customers
/*
#define INTERNAL_ID			0x000004B4		// Internal board
#define CUSTOMER_ID			0x00010001		// "General" Customer
#define HUAWEI_ID			0x00010002		// 
#define CISCO_ID			0x00010003		// 
#define FINISTAR_ID			0x00010004		// 
#define MICROSOFT_ID		0x00010005		// 
#define NOKIA_ID			0x00010006		// 
#define JUNIPER_ID			0x00010007		// 

#define JUNIPER_PAM_ID		0x00020001		// 
*/

#define INTERNAL_ID			0x00
#define ADMIN_ID			0x01
#define SEAGATE_ID			0x02
#define WD_ID				0x03
#define FUJITSU_ID			0x04
#define HGST_ID				0x05
#define TOSHIBA_ID			0x06
#define SAMSUNG_ID			0x07
#define INTEL_ID			0x08
#define CISCO_ID			0x09
#define SOFTWARE_0_ID		0x0A
#define SOFTWARE_1_ID		0x0B
#define SOFTWARE_2_ID		0x0C
#define SOFTWARE_3_ID		0x0D
#define SOFTWARE_4_ID		0x0E
#define SOFTWARE_5_ID		0x0F
#define MICRON_ID			0x10
#define INTERNAL_CIP_ID		0x11
#define XYRATEX_ID			0x12
#define LITEON_ID			0x13
#define SUNDISK_ID			0x14
#define SMART_ID			0x15
#define HUAWEI_ID			0x16
#define TEKTRONIX_ID		0x17
#define ZTE_ID				0x18
#define SAE_ID				0x19
#define ERICSSON_ID			0x1A
#define KIOXIA_ID			0x1B
#define CENTEC_ID			0x1C
#define JUNIPER_ID			0x1D
#define CRUISE_ID			0x1E
#define NETFORWARD_ID		0x1F
#define PALOALTONETWORKS_ID	0x20
#define IPBU_ID				0x21
#define GOOGLE_ID			0x22

#define NUMBER_OF_CUSTOMERS	0x23

//List of FabHouses
#define GENERAL_FABHOUSE		0		// Unknown
#define GORILLA_FABHOUSE		1
#define TTM_ANAHEIM_FABHOUSE	2
#define TTM_SAN_JOSE_FABHOUSE	3
#define ISU_PETASYS_FABHOUSE	4
#define OPERATIONS_FABHOUSE		5
#define SANMINA_FABHOUSE		6
#define SAMMIT_FABHOUSE			7
#define RandDTECH_FABHOUSE		8
#define PHASE3_FABHOUSE			9
#define SINGAPORE_FABHOUSE		10
#define SCC_FABHOUSE			11
#define RandDAltanova_FABHOUSE	12
#define TTM_TORONTO_FABHOUSE	13
#define TTM_HUIYANG_FABHOUSE	14

#define NUMBER_OF_FABS			15

#define GENERAL_ASSYSHOP		0		// Unknown
#define CBA_ASSYSHOP			1
#define GREEN_ASSYSHOP			2
#define PACTRON_ASSYSHOP		3
#define IN_HOUSE_ASSYSHOP		4
#define OPERATIONS_ASSYSHOP		5
#define GORILLA_ASSYSHOP		6
#define RandDTECH_ASSYSHOP		7
#define ABC_ASSYSHOP			8
#define ROCKET_ASSYSHOP			9
#define SUNNYTECH_ASSYSHOP		10
#define ALPHA_ASSYSHOP			11
#define SCC_ASSYSHOP			12
#define INFINITI_ASSYSHOP		13

#define NUMBER_OF_ASSY			14

#define GENERIC_ASSEMBLY_TYPE	0x10
#define SOCKET_ASSEMBLY_TYPE	0x11
#define SOLDER_ASSEMBLY_TYPE	0x12
#define SOFTWARE_ASSEMBLY_TYPE	0x13

#define MNT_MAX_NUMBER_INTERFACES		5	// note by SCM: number of comm paths (MDIO buses) per connector
#define MNT_MAX_NUMBER_CONFIG_INDEXES	8
#define MNT_MAX_NUMBER_CLOCKS			2
#define MNT_NUMBER_GPIO_PORTS			4
#define MNT_NUMBER_GPIO_PORTS_X			6

#define MNT_REGRESSION_MAX_NUMBER		5
#define MNT_MAX_BIAS					2

#define MNT_MAX_NUMBER_OF_CHIP_INTERFACES		2	// Currently Maximum number of MDIO buses connected to real chips
#define MNT_MAX_PHY_NUMBER						32	// PHY Addresses

#define MB_ZEUS_CFG_POWER_SUPPLY_NUMBER		4
#define MB_ZEUS_MAX_POWER_SUPPLY_NUMBER		6
#define MB_ZEUS_ATLAS_CFG_POWER_SUPPLY_NUMBER		4 + 5	//9		Update Version 10.0
#define MB_ZEUS_ATLAS_MAX_POWER_SUPPLY_NUMBER		6 + 7	//13	Update Version 10.0
#define MB_MAX_POWER_SUPPLY_NUMBER			16				//Support up to 2 BIAs

#define MB_MAX_ADC_TP					2
#define MNT_MAX_NUMBER_OF_CHIPS_OLD		16
#define MNT_MAX_NUMBER_OF_CHIPS			32
#define MNT_MAX_CHIP_DESCRIPTION_LEN	256

#define MNT_MAX_PACKET_SIZE			16384 * 4 * 16	// This is Temporarily limitation. Add option to configure from MNT_Config function
#define MNT_MAX_SEQUENCE_READ_SIZE	16384		// FPGA Limitation. May be increased.

#if !defined(Q_OS_MAC)
#define MNT			__declspec(dllexport)
#define JAN			__declspec(dllexport)
#else
#define MNT
#define JAN
#endif

//For OlympusHWFunction.Mode
#define MNT_DO_NOT_RELOAD	0
#define MNT_FORCE_RELOAD	1
#define MNT_UPDATE_DEFAULT	2
#define MNT_SKIP_DEFAULT	3
#define MNT_SKIP_ATHENA_POWER	5
#define MNT_UPDATE_DB_VALUE	6
#define MNT_ONLY_FIRMWARE	7
#define MNT_KILL_FPGA		8
#define MNT_JAN_CHAIN		9
#define MNT_ONLY_FIRMWARE_J	10
#define MNT_ONLY_FIRMWARE_A	11

#define MNT_DAC_MAX_VOLTAGE		5.0
#define MNT_DAC_RESOLUTION		16383.
#define MNT_ATLAS_DAC_MAX_VOLTAGE		4.096
#define MNT_ATLAS_DAC_RESOLUTION		65535.
#define MNT_LUNAR_DAC_MAX_VOLTAGE		2.5

#define MNT_MB_DAC_NUMBER	12 

#define MNT_ADC_MAX_VOLTAGE		4.096
#define MNT_ADC_RESOLUTION		16383.
#define MNT_ATLAS_ADC_RESOLUTION		65535.

#define MNT_ZAMBONI_DAC_ADC_MAX_VOLTAGE		2.5
#define MNT_ZAMBONI_DAC_ADC_RESOLUTION		4096

#define MNT_CLOCK_MHz	(104.)
#define MNT_CLOCK_NS	(1000. / 104.)
#define MNT_DAC_CLK		(1000. / 52.)	//19.23


#ifndef OlympusVersion
	enum MNT_VersionType { User, Kernel, Firmware1, FPGA1, FirmwareBoot, JSON};
	typedef struct
		{
		enum MNT_VersionType Type;
/*OLD
		int Major;
		int Minor;
*/
/*
		unsigned int MajorN	: 16;
		unsigned int Year	: 16;		//Used to be Major
		unsigned int MinorN;
*/
		int Year;		//Used to be Major
		unsigned int MajorN	: 16;
		unsigned int MinorN : 16;

		char Description[MAX_PATH];
		char Date[MAX_PATH];
		}
	OlympusVersion;
#endif

typedef struct
	{
	unsigned long Size;


	}
OlympusGlobalConfig;

//Specification for Hardware Configuration
typedef struct
	{
	unsigned long Size;
	long Mode;					
								//Mode 10 - Only Firmware Janus
								//Mode 9 - Janus Chain
								//Mode 8 - Kill FPGA
								//Mode 7 - Only Firmware
								//Mode 6 - Update DB Values
								//Mode 5 - Do not apply powers for Athena
								//Mode 3 - Do not reload, if it is loaded. Do not Update default settings
								//Mode 2 - Update Default settings only;
								//Mode 1 - Always Reload. Update Default settings;
								//Mode 0 - Do not reload, if it is loaded. 

	wchar_t* Config_File;		//NULL - Use default, -1 - do not load . NOT USED !!!
	wchar_t* Firmware_File;		//NULL - Use default, -1 - do not load
	wchar_t* FPGA_File;			//NULL - Use default, -1 - do not load, (scm) or full path to fpga file
	unsigned long SizeOfSpecialFunction;
	void* SpecialFunction;		//If SizeOfSpecialFunction == 1, use SpecialFunction as FPGA_ID 

	unsigned long Reserved;
	unsigned long id;
	}
OlympusHWFunction;

//Specification for SI Configuration
typedef struct
	{
	unsigned long Size;
	long TableEntry;					
	wchar_t* ConfigFile;			//NULL - Use default, -1 - do not load
	wchar_t* SiLabFile;				//NULL - Use default, -1 - do not load

	unsigned long  PartID_Rev;		//Use this Part ID + Revision for check
	unsigned long  DCOMode;			//This is per File

	char CustomID[4];

	//Specific Modes per Channel. 3 channels in Zeus (used), 5 Total, 12 in Janus
	BOOL Enable[12];
	unsigned long OutputMode[12];
	double OutputAmplitude[12];

	unsigned long  Reserved0;
	unsigned long  Reserved1;
	unsigned long  Reserved2;
	unsigned long  Reserved3;
	}
OlympusSIFunction;

typedef struct
	{
	unsigned long  Size;
	unsigned long  PowerSupplyNumber;
	unsigned long  CalibrationCurrentLOAD;
	unsigned long  Reserved0;
	unsigned long  Reserved1;
	unsigned long  Reserved2;

	double DacOffset;
	double DacGain;

	double AdcOffset;
	double AdcGain;

	unsigned long  C_RevCode;
	unsigned long  C_CalOffset;

	unsigned long  C_CalPoint[6];

	unsigned long  B_RevCode;
	unsigned long  B_CalOffset;

	unsigned long  B_CalPoint[6];
	}
PowerCalibration;

typedef struct
	{
	unsigned long Size;
	unsigned long PowerSupplyNumber;
	WCHAR SupplyDescription[MAX_PATH];
	double TargetVoltage;
	double MaximumVoltage;
	double MinimumVoltage;
	double SWTarget;
	double CurrentLOAD;
	double StepSize;				//NOT USED
	double RampTime;
	double CurrentLimit;
	double OverVoltage;
	double UnderVoltage;
	double PowerUpThreshold;
	double PowerDownThreshold;
	double DischargeThreshold;
	double CurrentVDACValue;
	double CurrentSWValue;
	double SWGain;
	unsigned long TriggerSource;
	double DelayAfterTrigger;
	unsigned long TriggerDestination;
	unsigned long SharedAdaptSense;	//Bit 0 == 1 - Indicate sharing with previous supply
									//Bit 1 == 1 - Adapt is Enabled
									//Bit 2 == 1 - Sense is Present
									//Bit 15..8 == Power Type
	double RampDown;
	double DelayDown;
	}
PowerSupplyInfo;

typedef struct
	{
	unsigned long Size;
	unsigned long PowerSupplyNumber;

    double InitialVoltage;	//Specify negative value to use "Current Voltage" as start point
	double FinalVoltage;

	double RampTime;	//ms

	unsigned long TriggerSource;
	double DelayAfterTriggerMS;
	}
PowerSupplyConfig;

typedef struct
	{
	unsigned long Size;
	unsigned long StartIn		: 1;
	unsigned long SignalIn		: 1;
	unsigned long Status		: 1;
	unsigned long Error			: 1;

	unsigned long Reserved		: 26;
	unsigned long AutoClear		: 1;
	unsigned long ReadCommand	: 1;
	}
BiaControl;

typedef struct
	{
	unsigned long Size;
	unsigned long MBDeviceType;
	unsigned long MBDeviceNumber;
	unsigned long MBSerialNumber;
	unsigned long MBCustomerID;
	unsigned long MBASSYPCBRevision;
	unsigned long MBProductionDate;

	unsigned long MBFabHouse;		//New 06/13/2017 + 1
	unsigned long MBAssyHouse;		//New 06/13/2017 + 2

	unsigned long DBDeviceType[MNT_REGRESSION_MAX_NUMBER];
	unsigned long DBSerialNumber[MNT_REGRESSION_MAX_NUMBER];
	unsigned long DBCustomerID[MNT_REGRESSION_MAX_NUMBER];
	unsigned long DBASSYPCBRevision[MNT_REGRESSION_MAX_NUMBER];
	unsigned long DBProductionDate[MNT_REGRESSION_MAX_NUMBER];
	unsigned long DBHDMIClockMode[MNT_REGRESSION_MAX_NUMBER];
	unsigned long DBAssemblyEx;

	unsigned long DBFabHouse[MNT_REGRESSION_MAX_NUMBER];		//New 06/13/2017 + 3
	unsigned long DBAssyHouse[MNT_REGRESSION_MAX_NUMBER];		//New 06/13/2017 + 4

	unsigned long InterposerDeviceType;
	unsigned long InterposerSerialNumber;
	unsigned long InterposerCustomerID;
	unsigned long InterposerASSYPCBRevision;
	unsigned long InterposerProductionDate;

	unsigned long InterposerFabHouse;		//New 06/13/2017 + 5
	unsigned long InterposerAssyHouse;		//New 06/13/2017 + 6

	wchar_t MBName[MNT_MAX_CHIP_DESCRIPTION_LEN];
	wchar_t DBName[MNT_REGRESSION_MAX_NUMBER][MNT_MAX_CHIP_DESCRIPTION_LEN];
	wchar_t InterposerName[MNT_MAX_CHIP_DESCRIPTION_LEN];

	unsigned long  DBChipIDs[MNT_REGRESSION_MAX_NUMBER];
	unsigned long  DBSlowFast[MNT_REGRESSION_MAX_NUMBER];

	unsigned long  NumberOfChips[MNT_REGRESSION_MAX_NUMBER];
	//STUB: Lets' have number of chips for each interface in the next revision
	wchar_t ChipDescription[MNT_REGRESSION_MAX_NUMBER][MNT_MAX_NUMBER_OF_CHIPS][MNT_MAX_CHIP_DESCRIPTION_LEN];
	unsigned long  InterfaceNumber[MNT_REGRESSION_MAX_NUMBER][MNT_MAX_NUMBER_OF_CHIPS];
	unsigned long  InterfaceAddr[MNT_REGRESSION_MAX_NUMBER][MNT_MAX_NUMBER_OF_CHIPS];

	unsigned long  NumberOfInterfaces;					//Maximum 5 for ZEUS / Janus
	unsigned long  InterfaceType[MNT_MAX_NUMBER_INTERFACES];
	double MaximumSpeed[MNT_MAX_NUMBER_INTERFACES];
	unsigned long  InterfaceClockType[MNT_MAX_NUMBER_INTERFACES];
	double CurrentSpeed[MNT_MAX_NUMBER_INTERFACES];
	
	unsigned long  ClockSource[MNT_MAX_NUMBER_CLOCKS];	//Disable, Enable, External
	unsigned long  ClockFreq[MNT_REGRESSION_MAX_NUMBER][MNT_MAX_NUMBER_CLOCKS];
	unsigned long  ClockAmplitude[MNT_REGRESSION_MAX_NUMBER][MNT_MAX_NUMBER_CLOCKS];
	unsigned long  ClockType[MNT_MAX_NUMBER_CLOCKS];	//Differential or Single Ended

	unsigned long NumberOfPowerSupplies[MNT_REGRESSION_MAX_NUMBER];
	unsigned long CurrentSpiBiaPort;
	HANDLE BiaHandles[MNT_REGRESSION_MAX_NUMBER];
	double BIAS;
	PowerSupplyInfo Power[MNT_REGRESSION_MAX_NUMBER][MB_MAX_POWER_SUPPLY_NUMBER];

	unsigned long LoadedFunction;
	unsigned long SoftKey;

	unsigned long USBSpeed;

	wchar_t PathForOlympus[MAX_PATH - 32];
	}
OlympusDeviceInfo;

typedef struct
	{
	unsigned long  Size;
	unsigned long  CommandStatus;
	unsigned long  RunningMode;
	unsigned long  Overflow;
	unsigned long  ClockMode;

	unsigned long  PowerCode[5];
	unsigned long  PowerCodeBeforeCorrection[5];
	unsigned long  SWPowerCode[5];

	unsigned long  PowerBIASCode;
	unsigned long  PowerDBCode;

	unsigned long  CurrentLoadCode[4];

	double PowerBeforeCorrection[5];
	double Power[5];
	double SWPower[5];

	double PowerBIAS;
	double PowerDB;

	double CurrentLoad[4];

	double TotalSeconds;
	double RunSeconds;
	}
OlympusDeviceState;

typedef struct
	{
	unsigned long  Size;
	unsigned long  CommandStatus;

	unsigned long  PowerCode[8];
	unsigned long  PowerCodeBeforeCorrection[8];

	unsigned long  CurrentLoadCode[8];

	double PowerBeforeCorrection[8];
	double Power[8];

	double CurrentLoad[8];
	}
OlympusDeviceStateExtraPowers;

typedef struct
	{
	unsigned long	Size;
	PowerSupplyInfo Power0;
	PowerSupplyInfo Power1;
	PowerSupplyInfo Power2;
	PowerSupplyInfo Power3;

	PowerSupplyInfo Power4;

	double BIAS;	//Read Only

	unsigned long	Gpio_Direction[MNT_NUMBER_GPIO_PORTS];
	unsigned long	Gpio_Value[MNT_NUMBER_GPIO_PORTS];
	unsigned long	Gpio_OpenDrain[MNT_NUMBER_GPIO_PORTS];

	unsigned long	InterfaceClockType[MNT_MAX_NUMBER_INTERFACES];
	double			InterfaceClockSpeed[MNT_MAX_NUMBER_INTERFACES];

	unsigned long	ClockSource[MNT_MAX_NUMBER_CLOCKS];	//Disable, Enable, External
	unsigned long	ClockFreq[MNT_MAX_NUMBER_CLOCKS];
	unsigned long	ClockAmplitude[MNT_MAX_NUMBER_CLOCKS];
	unsigned long	ClockType[MNT_MAX_NUMBER_CLOCKS];	//Differential Single Ended
	}
OlympusDeviceConfig;

typedef struct
	{
	unsigned long Size;

	PowerSupplyInfo Power[8];		//8 Power supplies on Atlas. Only 7 are used
	}
OlympusDeviceConfigExtraPowers;

typedef struct
	{
	unsigned long Size;

	float Ki;									//to hardware as long
	float Kp_m;									//to hardware as long
	float rdac;
	float rtop;
	float rbot;
	float gain;
	float offset;
	long LimitLower;							//to hardware. DAC Limit Lower for Max voltage
	long LimitUpper;							//to hardware. DAC Limit Upper for Min voltage

	float ActiveTemperatureGain;				//not used yet
	float ActiveTemperatureOffset;				//not used yet

	long SoakTimeCounterPIDLoops;				//to hardware. Maximum 255 * 0.5 sec 
	long TemperatureLockRange0PXX;				//to hardware. Maximum +/-2.56 Degree
	long ActiveTemperatureSensor;				//to hardware. 0 (On Board), 1 (On Chip), 2 (Bottom Plate), 3 (Top Plate)
	long TimeoutValueToLock;					//to hardware. Maximum 725 * 0.5 sec
	
	long WatchDogTimeMins;						//to hardware. Maximum 65535 Mins

	long LEDLimitUpper;							//to hardware. Maximum 0 to 255. LED configuration for HOT limits
	long LEDLimitLower;							//to hardware. Maximum 0 to 255. LED configuration for COLD limits

	long HumidityThresholdTemperature;			//to hardware. Maximum 2.56%
	long HumidityThresholdValue0PX;				//to hardware. Maximum 2.56%

	float const_K_term;							//May be Calculated -> to hardware as float
	float const_X_term;							//May be Calculated -> to hardware as float

	long EnableErrorCheck;
	long dwReserved1;
	long dwReserved2;
	}
OlympusZamboniConfig;

typedef struct
	{
	unsigned long Size;

	float Temperature[4];	//0 (On Board), 1 (On Chip), 2 (Bottom Plate), 3 (Top Plate)
	float Adc[4];			//24VA, 24VB, Voltage, Current
	float Humidity[2];		//Humidity sensors for zamboni[0] and daughter board[1]
	float TargetTemperature;//User SetTemperature
	unsigned long CountdownLockCounter;	//Soak time
	unsigned long CountdownTimeoutCounter; //Timeout
	unsigned long GPIO;
	unsigned long Errors;	//Bit 0 - TMP461 is Open
							//Bit 1 - Bottom Sensor is Open
							//Bit 2 - Bottom Sensor is Short
							//Bit 3 - Top Sensor is Open
							//Bit 4 - Top Sensor is Short
							//Bit 5 - Fan is not running
							//Bit 6 - Pump is not running
							//Bit 7 - I2C Error							
							//Bit 8 - WatchDogTimeout
							//Bit 9 - Not Initialized or Cable not Connected

	unsigned long Status;	//Bit 0 - Target Temperature is In Range
							//Bit 1 - Target Temperature is Locked (after soak time)
							//Bit 2 - Timeout Lock
							//Bit 3 - Loop is Running
							//Bit 4 - Calculation is Disabled
							//Bit 5 - Data Valid

	float RoomTemperature;  //Data from Humidity/temp sensor on Zamboni
	unsigned long Reserved[4];
	}
OlympusZamboniStatus;

//From Steve M.:  Scoped enums not supported before C++11
#ifdef __cplusplus
	#if __cplusplus <= 199711L
	enum OlympusInterfaceProtocol { SIF_MDIO,		//0
									SIF_I2C,		//1
									SIF_JTAG,		//2
									SIF_IJTAG,		//3 
									SIF_EEPROM_SPI, //4
									SIF_I2C_H,		//5
									SIF_MRVL,		//6
									SIF_OVERWRITE,	//7
									SIF_SPI_SPARTA,	//8
									SIF_SPI_H,		//9
									SIF_MARVELL_OLD,//10
									SPI_SILAB,		//11
									SIF_MDIO22_A,	//12
									SIF_MDIO22_B,	//13
									SIF_JTAGSVF,	//14
									SIF_DISABLE = 0xF,	//15 
									SIF_UNKNOWN = -1 };
	enum OlympusClockType { SIF_CLK_NORM, SIF_CLK_CONT, SIF_CLK_TRI, SIF_CLK_OVERWRITE, SIF_CLK_UNKNOWN = -1 };
	enum OlympusBoardType { ZEUS, JANUS, APOLLO, SPARTA};
	#else
	enum OlympusInterfaceProtocol : long {	SIF_MDIO, 
											SIF_I2C, 
											SIF_JTAG, 
											SIF_IJTAG, 
											SIF_EEPROM_SPI, 
											SIF_I2C_H, 
											SIF_MRVL, 
											SIF_OVERWRITE, 
											SIF_SPI_SPARTA, 
											SIF_SPI_H, 
											SIF_MARVELL_OLD, 
											SPI_SILAB, 
											SIF_MDIO22_A, 
											SIF_MDIO22_B, 
											SIF_JTAGSVF, 
											SIF_DISABLE = 0xF, 
											SIF_UNKNOWN = -1 };
	enum OlympusClockType : long { SIF_CLK_NORM, SIF_CLK_CONT, SIF_CLK_TRI, SIF_CLK_OVERWRITE, SIF_CLK_UNKNOWN = -1 };
	enum OlympusBoardType : long { ZEUS, JANUS, APOLLO, SPARTA};
	#endif
#else
enum OlympusInterfaceProtocol { SIF_MDIO,		//0
	SIF_I2C,		//1
	SIF_JTAG,		//2
	SIF_IJTAG,		//3 
	SIF_EEPROM_SPI, //4
	SIF_I2C_H,		//5
	SIF_MRVL,		//6
	SIF_OVERWRITE,	//7
	SIF_SPI_SPARTA,	//8
	SIF_SPI_H,		//9
	SIF_MARVELL_OLD,//10
	SPI_SILAB,		//11
	SIF_MDIO22_A,	//12
	SIF_MDIO22_B,	//13
	SIF_JTAGSVF,	//14
	SIF_DISABLE = 0xF,	//15 
	SIF_UNKNOWN = -1 };
enum OlympusClockType { SIF_CLK_NORM, SIF_CLK_CONT, SIF_CLK_TRI, SIF_CLK_OVERWRITE, SIF_CLK_UNKNOWN = -1 };
enum OlympusBoardType { ZEUS, JANUS, APOLLO, SPARTA};
#endif

typedef struct
	{
	unsigned long				Size;
	unsigned long				InterfaceNumber;
	enum OlympusInterfaceProtocol	InterfaceProtocol;
	enum OlympusClockType			InterfaceClockType;
	double						ClkMhz;
	}
OlympusInterfaceInfo;

// 	   This structure will be OBSOLETE in 2023.6 !!!
typedef struct
	{
	unsigned long				Size;
	unsigned long				InterfaceNumber;
	enum OlympusInterfaceProtocol	InterfaceProtocol;
	enum OlympusClockType			InterfaceClockType;
	double						ClkMhz;
	long						NumberOfAddressBits;
	long						NumberOfDataBits;		//Redirect TRST_N for iJTAG
	long						AddressIncrementSize;
	long						DisableI2CAbort;
	long						ReadClockMode;			//Bit 0 - invert read clock, Bit 1 - delay MDIO read
	}
OlympusInterfaceInfoEx;

typedef struct
	{
	unsigned long				Size;
	unsigned long				InterfaceNumber;
	unsigned long				NumberOfAddressBits[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				NumberOfDataBits[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				AddressIncrementSize[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				PreBits[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				WrWaitBits[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				RdWaitBits[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				WrPostBits[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				RdPostBits[MNT_MAX_NUMBER_CONFIG_INDEXES];

	unsigned long				Reserved0[MNT_MAX_NUMBER_CONFIG_INDEXES];
	unsigned long				Reserved1;
	unsigned long				Reserved2;
	unsigned long				Reserved3;
	unsigned long				Reserved4;
	}
OlympusInterfaceInfoSif;

typedef struct
	{
	unsigned short				Size;
	unsigned short				IndirectMode;	//1 – STC05R2, 2 – TL10, otherwise normal
	unsigned short				AccessMode;		//0 -> Normal; 1: Same Address with Set Address; 2: Same Address, No Set Address; 3: Set Address ONLY
	unsigned short				Flag32Bit;		//32Bit Mode for Regular MDIO (not used in raptor)
	unsigned short				Broadcast;		//For TL10
	unsigned short				PollReturnZero;
	unsigned short				PollDoNotStop;
	unsigned short				Reserved[1];
	}
OlympusInterfaceAccessOptions;

#define MAX_NUMBER_CONTROL_SEQUENCES	8

typedef struct
	{
	unsigned long	Size;

	unsigned long	UpdateGpio[MNT_NUMBER_GPIO_PORTS];
	unsigned long	Gpio_Direction[MNT_NUMBER_GPIO_PORTS];
	unsigned long	Gpio_Value[MNT_NUMBER_GPIO_PORTS];

	long			PowerSequence;
	long			ResetN[2];

	double			Delay_in_ns;
	unsigned long	NextState;
	}
OlympusControlSequence;

typedef struct
	{
	unsigned long	Size;

	unsigned long	SerialNumber;
	unsigned long	CustomerID;
	unsigned char	PCBPlace;
	unsigned char	PCBRevision;
	unsigned char	AssemblyPlace;
	unsigned char	AssemblyRevision;
	unsigned long	CalibrationDate;
	unsigned long	HSSerialNumber;

	float			fPeltierGain;
	float			fPeltierOffs;
	float			fFan0Gain;
	float			fFan0Offs;
	float			fFan1Gain;
	float			fFan1Offs;
	float			fCurrentLimitGain;
	float			fCurrentLimitOffs;
	float			f24VAGain;
	float			f24VAOffs;
	float			f24VBGain;
	float			f24VBOffs;
	float			fPeltierVoltageGain;
	float			fPeltierVoltageOffs;
	float			fPeltierCurrentGain;
	float			fPeltierCurrentOffs;

	unsigned long	dwPeltierRTOP;
	unsigned long	dwPeltierRBOT;
	unsigned long	dwPeltierRDAC;

	unsigned long	dwFanRTOP;
	unsigned long	dwFanRBOT;
	unsigned long	dwFanRDAC;

	unsigned long	ReservedGO[5];

	unsigned long	Plates;
	unsigned long	Cooling;
	unsigned long	TargetBoardID;
	unsigned long	TargetChipID;
	unsigned long	ReservedT[4];
	}
OlympusZamboniInfo;

//External Error Handling Function
typedef DWORD(*TypdefErrorHandling)(DWORD, wchar_t*, DWORD, DWORD);
extern DWORD DefaultErrorHandlingFunction(DWORD ErrorCode, WCHAR* FunctionName, DWORD Severity, DWORD FunctionLevel);

//Global configuration for the Driver. Currently does not do anything
MNT DWORD MNT_Config(void* GlobalConfig);

//Return number of devices connected to the computer. If DeviceType == "-1", function returns total number of all connected devices
MNT DWORD MNT_GetDevices(DWORD DeviceType, PDWORD NumberOfDevices);
//software may retrieve device handle (if it lost). Or check if device is opened 
MNT DWORD MNT_GetHandle(DWORD DeviceType, DWORD DeviceNumber, PHANDLE DeviceHandle);
//software may retrieve the device typw and number by supplying handle 
MNT DWORD MNT_GetDeviceType(HANDLE DeviceHandle, PDWORD DeviceType, PDWORD DeviceNumber);

//May function to open device and get the HANDLE, 
MNT DWORD MNT_OpenDevice(DWORD DeviceType, DWORD DeviceNumber, PHANDLE DeviceHandle);
MNT DWORD MNT_CloseDevice(HANDLE DeviceHandle);

//This function loads hardware. If sHFFunction == NULL -> Load default configuration and it will not reload, if already loaded
MNT DWORD MNT_LoadDevice(HANDLE DeviceHandle, OlympusHWFunction* sHWFunction);  // scm: pass structure to force reload.

//Return useful information.
MNT DWORD MNT_GetDeviceInfo(HANDLE DeviceHandle, OlympusDeviceInfo* sDeviceInfo);

//This is subset of MNT_GetDeviceInfo function to get versions only. Currently we have 5 versions: DLL, Kernel, Firmware, FPGA, Firmware Bootloader
MNT DWORD MNT_GetVersions(HANDLE DeviceHandle, DWORD NumberOfVersions, OlympusVersion *Versions, void *extrainfo);
//This is subset of MNT_GetDeviceInfo function to get MB and DB serial numbers. if software does not care about particular serial number, pass NULL
MNT DWORD MNT_GetSerialNumbers(HANDLE DeviceHandle, DWORD* MBSerialNumber, DWORD* DBSerialNumber);

//Idea behind softkey -> synchronize multiple software. 
MNT DWORD MNT_SetSoftKey(HANDLE DeviceHandle, DWORD SoftKey);
MNT DWORD MNT_GetSoftKey(HANDLE DeviceHandle, DWORD* SoftKey);

//DeviceState may be used by different software or by the same software after re-starting it,
MNT DWORD MNT_GetDeviceState(HANDLE DeviceHandle, OlympusDeviceState* DeviceState);
//Info about Extra Powers on Atlas
MNT DWORD MNT_GetDeviceStateExtraPowers(HANDLE DeviceHandle, OlympusDeviceStateExtraPowers* ExtraPowers);
//Currently not implemented.
MNT DWORD MNT_SetDeviceState(HANDLE DeviceHandle, OlympusDeviceState* param);

//FX3 or FPGA timers may be used as a precise timer. Better resolution compare to Windows. 
//Options: 0 - fpga, 1 - fx3, 2 - os, 3 - reset (for FPGA)
MNT DWORD MNT_GetTime(HANDLE DeviceHandle, DWORD option, double* Seconds);

//Basic configuration of MB. Currently may be used to overwrite default thresholds for Power Supplies. Use SIZE MSB Bit as indicator to get values back
MNT DWORD MNT_ConfigDevice(HANDLE DeviceHandle, OlympusDeviceConfig* DeviceConfig);
//Configure extra powers on Atlas. Use SIZE MSB Bit as indicator to get values back
MNT DWORD MNT_ConfigDeviceExtraPowers(HANDLE DeviceHandle, OlympusDeviceConfigExtraPowers* ExtraPowers);

//Set Power Supplies Info. Use SIZE MSB Bit as indicator to get values back
MNT DWORD MNT_ConfigDevicePowers(HANDLE DeviceHandle, DWORD NumberOfPower, PowerSupplyInfo PowerInfo[MB_MAX_POWER_SUPPLY_NUMBER]);

//Automatic Control of Sequences. Usually use for Power Up Sequence
MNT DWORD MNT_ControlSequence(HANDLE DeviceHandle, DWORD NumberOfSequences, OlympusControlSequence ControlSequences[MAX_NUMBER_CONTROL_SEQUENCES]);

//Supply Error code to get the description
MNT DWORD MNT_GetDetailedError(DWORD ErrorCode, WCHAR* Text, DWORD MaxCharacters);
MNT DWORD MNT_AnalyzeError(DWORD ErrorCode, WCHAR* Text, DWORD MaxCharacters);

//Obsolete Functions: DO NOT USE:
MNT DWORD MNT_SetInterfaceClock(HANDLE DeviceHandle, double ClkMhz, LONG ContinuousTristateClock);
//NULL pointer -> do not read
//Obsolete Functions: DO NOT USE:
MNT DWORD MNT_GetInterfaceClock(HANDLE DeviceHandle, double* ClkMhz, LONG* ContinuousTristateClock);

//Configure Individual Interface Functions
MNT DWORD MNT_ConfigureInterface(HANDLE DeviceHandle, DWORD NumberOfInterfaces, OlympusInterfaceInfo* InterfaceInfo);
// 	   This function will be OBSOLETE in 2023.6 !!!
MNT DWORD MNT_ConfigureInterfaceEx(HANDLE DeviceHandle, DWORD NumberOfInterfaces, OlympusInterfaceInfoEx* InterfaceInfoEx);
MNT DWORD MNT_ConfigureInterfaceSif(HANDLE DeviceHandle, DWORD NumberOfInterfaces, BOOL ReadFlag, OlympusInterfaceInfoSif* InterfaceInfoSif, WCHAR DefaultInterfaceDescription[MNT_MAX_NUMBER_INTERFACES][MNT_MAX_NUMBER_CONFIG_INDEXES][MNT_MAX_CHIP_DESCRIPTION_LEN]);
//Update Interface Configuration. To be used in Sequence
MNT DWORD MNT_UpdateInterface(HANDLE DeviceHandle, OlympusInterfaceInfoEx* Interface);
//Configure MDIO For TL10 
MNT DWORD MNT_ConfigureInterfaceTL10(HANDLE DeviceHandle, DWORD InterfaceNumber, 
													BOOL ReadFlag, 
													WORD* CmdReg,
													WORD* AddrReg,
													WORD* DataReg,
													WORD* MaskReg);
//Toggle specified clock
MNT DWORD MNT_ToggleInterfaceClock(HANDLE DeviceHandle, DWORD InterfaceNumber);

//For Testing only Specify Size == 0 -> Execute Single Write/Read one MDIO Register. Do not Set Address
//Later FPGA would automatically do that with correct size == 1
//For Testing only Specify Size == 0xFFFFFFFFF -> Set Address only. Do not execute write/read
// OBSOLETE. DO NOT USE.
MNT DWORD MNT_MdioWrite(HANDLE DeviceHandle, DWORD Bus, DWORD PHYADDR, DWORD DEVTYPE, DWORD Address, DWORD Size, DWORD* Data);
// OBSOLETE. DO NOT USE.
MNT DWORD MNT_MdioRead(HANDLE DeviceHandle, DWORD Bus, DWORD PHYADDR, DWORD DEVTYPE, DWORD Address, DWORD Size, DWORD* Data);
// OBSOLETE. DO NOT USE.
MNT DWORD MNT_MdioRMW(HANDLE DeviceHandle, DWORD Bus, DWORD PHYADDR, DWORD DEVTYPE, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask);
// OBSOLETE. DO NOT USE.
MNT DWORD MNT_MdioPoll(	HANDLE DeviceHandle, 
						DWORD Bus, 
						DWORD PHYADDR, 
						DWORD DEVTYPE, 
						DWORD Address, 
						DWORD Size,
						DWORD* Data, 
						DWORD* Mask, 
						DWORD* Timeout,
						BOOL* QuietMode);

// OBSOLETE. DO NOT USE.
MNT DWORD MNT_MdioAccess(	HANDLE DeviceHandle,
							DWORD Size,
							DWORD *Cmd,	//Write, Read, RMW, Wait, Delay, NOP
							DWORD *Bus,
							DWORD *PHYADDR, 
							DWORD *DEVTYPE, 
							DWORD *Address,
							DWORD *Data,
							DWORD *Mask);

//For Native 32 bit access: Bus & 0x30000 = 0x10000 For Native 8 bit access: Bus & 0x30000 = 0x20000
MNT DWORD MNT_Write(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data);
MNT DWORD MNT_WriteEx(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, OlympusInterfaceInfoEx* Interface);
MNT DWORD MNT_WriteX(HANDLE DeviceHandle, LONG Bus, SHORT HiAddrOrSIFIndex, LONG PhyAdr, DWORD Address, DWORD Size, DWORD* Data, OlympusInterfaceAccessOptions* Options);

MNT DWORD MNT_Read(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data);
MNT DWORD MNT_ReadEx(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, OlympusInterfaceInfoEx* Interface);
MNT DWORD MNT_ReadMaskData(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, DWORD* Shift);
MNT DWORD MNT_ReadMaskDataEx(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, DWORD* Shift, OlympusInterfaceInfoEx* Interface);
//Combine MNT_Read and MNT_ReadMaskData
MNT DWORD MNT_ReadX(HANDLE DeviceHandle, LONG Bus, SHORT HiAddrOrSIFIndex, LONG DevType, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, DWORD* Shift, OlympusInterfaceAccessOptions* Options);

MNT DWORD MNT_RMW(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask);
MNT DWORD MNT_RMWEx(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, OlympusInterfaceInfoEx* Interface);
MNT DWORD MNT_RMWX(HANDLE DeviceHandle, LONG Bus, SHORT HiAddrOrSIFIndex, LONG PhyAdr, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, OlympusInterfaceAccessOptions* Options);

//Mode bits in the Bus field: Bit 18: Return 0 on fail, Bit 19: Do Not stop on fail
MNT DWORD MNT_Poll(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, DWORD* Timeout, BOOL* QuietMode);
MNT DWORD MNT_PollEx(HANDLE DeviceHandle, DWORD Bus, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, DWORD* Timeout, BOOL* QuietMode, OlympusInterfaceInfoEx* Interface);
MNT DWORD MNT_PollX(HANDLE DeviceHandle, LONG Bus, SHORT HiAddrOrSIFIndex, LONG PhyAdr, DWORD Address, DWORD Size, DWORD* Data, DWORD* Mask, DWORD* Timeout, BOOL* QuietMode, OlympusInterfaceAccessOptions* Options);
//Move to Config Interface MNT DWORD MNT_AddressStep(HANDLE DeviceHandle, DWORD* Step);

//Delay command makes sense for sequences only. Otherwise actual delay is larger then specified. Precision is 0.010 us. Minimum delay is 100 ns. (if inside packet).
MNT DWORD MNT_Delay(HANDLE DeviceHandle, double Delay_uS);

//Read and clear the status: 0 - no errors, number indicates the transaction cycle, which aborted I2C transaction
MNT DWORD MNT_ReadClearStatus(HANDLE DeviceHandle, DWORD* Data);

//Read extra data form SPI or JTAG commands
MNT DWORD MNT_ReadExtra(HANDLE DeviceHandle, DWORD Address, DWORD Size, DWORD* Data);

//Set Global Timeout value. -1 - read back
MNT DWORD MNT_PollTimeout(HANDLE DeviceHandle, double* TimeoutUS);

//Power supply need a minimum load to function correctly. By default DLL sets it to 2mA. Specify Current in mA
MNT DWORD MNT_SetCurrentLoad(HANDLE DeviceHandle, double* Current);
//For status see: XRP7620 document on K drive
MNT DWORD MNT_GetCurrentLoad(HANDLE DeviceHandle, BYTE* Status, double* Current);

//Main function to control Power. Data: PowerSupplyConfig. If Data == NULL, do not load. If Execute == -1, do not execute, if == 0, abort, 0x10000 -> First BIA, 0x20000 -> Second BIA
//This is obsolete function. Use MNT_PowerControlEx instead
MNT DWORD MNT_PowerControl(HANDLE DeviceHandle, DWORD NumberOfSupplies, PowerSupplyConfig* Data, DWORD Execute);

//Wait for ADC Average with timeout (us)
//This is obsolete function. Use MNT_WaitForAdcAverageEx instead
MNT DWORD MNT_WaitForAdcAverage(HANDLE DeviceHandle, DWORD TimeoutUS);
//Check if Power Control is DONE. If DONE == NULL, do not return DONE, if DetailedDone != NULL, return DONE for each power supply Bit 0 - Power Supply 0, etc;
//11/12/2015 Add Lock Info DetailedDone: 4 bit - DAC is updating, 4-bit low, 4 bit lock, 4 bit DAC in range, 4 bit ADC in range, 4 bit 0, 4 bit adjust enable
//This is obsolete function. Use MNT_PowerControlDoneEx instead
MNT DWORD MNT_PowerControlDone(HANDLE DeviceHandle, BOOL* Done, DWORD* DetailedDone);
//Configure Voltage Adapt Circuit, Specify NULL pointer - do not update, Specify value "-1" read back. Status always read backs. 
//Enable for each channel: 0 (disable), 1 (enable), or 2 (auto lock). Zeus Only
//Status: Bit 3:0 - ADC is not low. Bit 7:4 - Adjust Enabled (DAC is in range). Bit 11:8 - Lock, Bit 15:12 - Dac is low
//This is obsolete function. Use MNT_PowerAdaptConfigEx instead
MNT DWORD MNT_PowerAdaptConfig(HANDLE DeviceHandle, LONG* Bandwidth, LONG* LockBandwidth,
															LONG AdjustLimitPercent[MB_MAX_POWER_SUPPLY_NUMBER],
															LONG Enable[MB_MAX_POWER_SUPPLY_NUMBER], LONG* Status);
//Zeus + Atlas Configuration
//This is obsolete function. Use MNT_PowerAdaptConfigZAEx instead
MNT DWORD MNT_PowerAdaptConfigZA(HANDLE DeviceHandle, LONG* Bandwidth, LONG* LockBandwidth,
															LONG AdjustLimitPercent[MB_ZEUS_ATLAS_CFG_POWER_SUPPLY_NUMBER],
															LONG Enable[MB_ZEUS_ATLAS_CFG_POWER_SUPPLY_NUMBER], LONGLONG* Status);

//Get Actual DAC Values if Code == NULL, do not return codes. There are 12 DACs for Zeus + 8 for Atlas
MNT DWORD MNT_GetDacValues(HANDLE DeviceHandle, DWORD* Codes);

//THIS FUNCTION IS Obsolete DO NOT USE IT. Use MNT_SetPowerEx
//Temporally function until MNT_PowerControl is done. Specify Voltage in Volts
MNT DWORD MNT_SetPower(HANDLE DeviceHandle, DWORD Channel, double Voltage);

//THIS FUNCTION IS Obsolete DO NOT USE IT. Use MNT_SetPowerSoftRampEx
//Temporally function until MNT_PowerControl is done. Specify Voltage in Volts. If StartVoltage < 0 start from Current Value. 
//Restriction: maximum TimeinMs - 3 ms. Note be carefully power up different power supplies in case they are connected on DB.
MNT DWORD MNT_SetPowerSoftRamp(HANDLE DeviceHandle, DWORD Channel, double StartVoltage, double EndVoltage, double TimeInMs);

//Read ADC. there are 16 ADC on the board (See Zeus schematics Page 6)
//This is obsolete function. Use MNT_ReadVoltageEx instead
MNT DWORD MNT_ReadVoltage(HANDLE DeviceHandle, DWORD Channel, double* Voltage);
//Voltages must be an array of 16 doubles for Zeus and 32 for Atlas
//This is obsolete function. Use MNT_ReadAllVoltagesEx instead
MNT DWORD MNT_ReadAllVoltages(HANDLE DeviceHandle, double* Voltages);
//If Current == NULL -> do not return, if Power == NULL-> do not return Otherwise supply placeholder for 4 channels
// Current in A. Power in mW: 4 Powers Maximum: Zeus + Atlas: 9 doubles
//This is obsolete function. Use MNT_ReadAllCurrentPowerEx instead
MNT DWORD MNT_ReadAllCurrentPower(HANDLE DeviceHandle, double* Current, double* Power);
//This is obsolete function. Use MNT_ReadAllVoltageCurrentPowerEx instead
MNT DWORD MNT_ReadAllVoltageCurrentPower(HANDLE DeviceHandle, double* Voltage, double* Current, double* Power);
//This is obsolete function. Use MNT_GetTargetVoltagesEx instead
MNT DWORD MNT_GetTargetVoltages(HANDLE DeviceHandle, DWORD NumberOfVoltages, double* VoltageBeforeCorrection);
//This is obsolete function. Use MNT_GetBaseVoltagesEx instead
MNT DWORD MNT_GetBaseVoltages(HANDLE DeviceHandle, double* SetVoltages, double* SenseVoltages, double* Currents, double* Powers);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MNT DWORD MNT_GetTargetVoltagesEx(HANDLE DeviceHandle, LONG JPort, DWORD NumberOfVoltages, double* VoltageBeforeCorrection);
MNT DWORD MNT_GetBaseVoltagesEx(HANDLE DeviceHandle, LONG JPort, double* SetVoltages, double* SenseVoltages, double* Currents, double* Powers);
MNT DWORD MNT_ConfigDevicePowersEx(HANDLE DeviceHandle, LONG JPort, DWORD NumberOfPower, PowerSupplyInfo PowerInfo[MB_MAX_POWER_SUPPLY_NUMBER]);
MNT DWORD MNT_ReadAllVoltagesEx(HANDLE DeviceHandle, LONG JPort, double* Voltages);
MNT DWORD MNT_ADCControlEx(HANDLE DeviceHandle, LONG JPort, LONG* Average, LONG* Disable0, LONG* Disable1);
MNT DWORD MNT_ReadAllCurrentPowerEx(HANDLE DeviceHandle, LONG JPort, double* Current, double* Power);
MNT DWORD MNT_ReadAllVoltageCurrentPowerEx(HANDLE DeviceHandle, LONG JPort, double* Voltage, double* Current, double* Power);
MNT DWORD MNT_PowerControlEx(HANDLE DeviceHandle, LONG JPort, DWORD NumberOfSupplies, PowerSupplyConfig* Data, DWORD Execute);
MNT DWORD MNT_WaitForAdcAverageEx(HANDLE DeviceHandle, LONG JPort, DWORD TimeoutUS);
MNT DWORD MNT_PowerControlDoneEx(HANDLE DeviceHandle, LONG JPort, BOOL* Done, DWORD* DetailedDone);
MNT DWORD MNT_PowerAdaptConfigEx(HANDLE DeviceHandle, LONG JPort, LONG* Bandwidth, LONG* LockBandwidth,
															LONG AdjustLimitPercent[MB_MAX_POWER_SUPPLY_NUMBER],
															LONG Enable[MB_MAX_POWER_SUPPLY_NUMBER], LONG* Status);
MNT DWORD MNT_PowerAdaptConfigZAEx(HANDLE DeviceHandle, LONG JPort, LONG* Bandwidth, LONG* LockBandwidth,
															LONG AdjustLimitPercent[MB_ZEUS_ATLAS_CFG_POWER_SUPPLY_NUMBER],
															LONG Enable[MB_ZEUS_ATLAS_CFG_POWER_SUPPLY_NUMBER], LONGLONG* Status);
MNT DWORD MNT_SetPowerEx(HANDLE DeviceHandle, LONG JPort, DWORD Channel, double Voltage);
MNT DWORD MNT_SetPowerSoftRampEx(HANDLE DeviceHandle, LONG JPort, DWORD Channel, double StartVoltage, double EndVoltage, double TimeInMs);
MNT DWORD MNT_ReadVoltageEx(HANDLE DeviceHandle, LONG JPort, DWORD Channel, double* Voltage);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ADC Control. Advanced users only! Specify "-1" to read back values. NULL - ignore it.
MNT DWORD MNT_ADCControl(HANDLE DeviceHandle, LONG* Average, LONG* Disable0, LONG* Disable1);

//DAC Control. Advanced users only! Specify "-1" to read back values. NULL - ignore it.
MNT DWORD MNT_DACControl(HANDLE DeviceHandle, DWORD DACNumber, LONG* DACValue);

//If CalibrationData == NULL ->Restore Default. (-1) -> Eris-4
MNT DWORD MNT_ForceCalibrationValues(HANDLE DeviceHandle, DWORD  Channel, PowerCalibration* CalibrationData);
MNT DWORD MNT_GetCalibrationValues(HANDLE DeviceHandle, DWORD  Channel, PowerCalibration* CalibrationData);

//Overwrite Reset Lines to DB. Specify -1 -> Keep the old value; 2 - Tristate
MNT DWORD MNT_GetResetsInfo(HANDLE DeviceHandle, DWORD Port, BOOL Polarity[4], WCHAR wcNames[4][MAX_PATH]);
MNT DWORD MNT_ForceReset(HANDLE DeviceHandle, LONG Reset1, LONG Reset2);
MNT DWORD MNT_ForceResetPlus(HANDLE DeviceHandle, LONG Port, LONG Reset1, LONG Reset2);
//This function allow to set and read data
MNT DWORD MNT_ForceResetEx(HANDLE DeviceHandle, LONG* Reset1, LONG* Reset2);
MNT DWORD MNT_ForceResetPlusEx(HANDLE DeviceHandle, LONG Port, LONG* Reset1, LONG* Reset2);
MNT DWORD MNT_ForceResetKarEx(HANDLE DeviceHandle, LONG Port, LONG Reset[4]);
//Control DB signals. Tristate all data lines and/or Kill power lines.  Specify -1 -> Keep the old value;
//EnableDBSignals: Bit 0 - Signals Enable, Bit 1 - DAC Enable, Bit 2 - Reset0 Tristate, Bit 3 - Reset1 Tristate; ForcePowerReset -> Kill DAC / Power
//For Apollo: Bit 0: PWR0 En, Bit 1: PWR1 En, Bit 2: I2C0 En, Bit 3: I2C1 En
MNT DWORD MNT_ControlDB(HANDLE DeviceHandle, LONG EnableDBSignals, LONG EnablePowerReset);
//This function allow to set and read data
//EnableDBSignals: Bit 0 - Signals Enable, Bit 1 - DAC Enable, Bit 2 - Reset0 Tristate, Bit 3 - Reset1 Tristate; Bit 4 - Power on DB On (Eris). Off (Tundra);  ForcePowerReset -> Kill DAC / Power
//For Apollo: Bit 0: PWR0 En, Bit 1: PWR1 En, Bit 2: I2C0 En, Bit 3: I2C1 En
MNT DWORD MNT_ControlDBEx(HANDLE DeviceHandle, LONG* EnableDBSignals, LONG* EnablePowerReset);

//Contol Bia Signals
MNT DWORD MNT_ControlBia(HANDLE DeviceHandle, BiaControl* BiaControl);

//Control All signals from MB to DB. Use "-1" - as do not change
//There are 4 ports. Main Port == 0 32 signals, See schematics for the particular DB.
//if Direction == 1 -> MB-to-DB if Direction == 0 -> DB-to-MB
MNT DWORD MNT_SetGPIOs(HANDLE DeviceHandle, DWORD Port, DWORD Direction, DWORD Value);
MNT DWORD MNT_GetGPIOs(HANDLE DeviceHandle, DWORD Port, DWORD* Direction, DWORD* Value);
MNT DWORD MNT_GetGPIOsPlus(HANDLE DeviceHandle, DWORD Port, DWORD* Direction, DWORD* Value, WCHAR wcNames[32][MAX_PATH]);
MNT DWORD MNT_GetGPIOsDefaultInfo(HANDLE DeviceHandle, DWORD Port, DWORD* Direction, DWORD* Value, DWORD* OpenDrain, DWORD* Hidden, WCHAR wcNames[32][MAX_PATH]);

//Subset of previous function. Set only one signal direction, not whole bus. BitPosition - Signal Number (0..31). 
MNT DWORD MNT_SetOneGPIODirection(HANDLE DeviceHandle, DWORD Port, DWORD BitPosition, BOOL Direction);
MNT DWORD MNT_GetOneGPIODirection(HANDLE DeviceHandle, DWORD Port, DWORD BitPosition, BOOL* Direction);

//Subset of previous function. Set only one signal, not whole bus. BitPosition - Signal Number (0..31). 
//If Port Direction set as input SetOneGPIO does not change the signal, returns the error. Value 0, 1 or 2 (open drain)
MNT DWORD MNT_SetOneGPIO(HANDLE DeviceHandle, DWORD Port, DWORD BitPosition, int Value);
MNT DWORD MNT_GetOneGPIO(HANDLE DeviceHandle, DWORD Port, DWORD BitPosition, BOOL* Value);

//Return Info: Type: 0 (Input), 1 - (Output), 2 - Open Drain; Value - 0 or 1
MNT DWORD MNT_GetOneGPIOInfo(HANDLE DeviceHandle, DWORD Port, DWORD BitPosition, DWORD* Type, DWORD* Value, WCHAR Name[MAX_PATH]);
MNT DWORD MNT_GetOneGPIODefaultInfo(HANDLE DeviceHandle, DWORD Port, DWORD BitPosition, DWORD* Type, DWORD* Value, WCHAR Name[MAX_PATH]);

//Configure optional GPIO mode to route TP signal. Valid for GPIO Port 0 ONLY. Bits 16..31. Mux == -1: do not change, return Value. Valid Values: 0..8
MNT DWORD MNT_ConfigGPIOAux(HANDLE DeviceHandle, DWORD Port, DWORD BitPosition, LONG* Mux);

//Connect one of 4 TP and 4 LED to specified signal inside FPGA. (TpLedNumber - 0..7)
//For list of available signals. See ZeusTPList.h. 
MNT DWORD MNT_SetTpLed(HANDLE DeviceHandle, DWORD TpLedNumber, DWORD Value);
MNT DWORD MNT_GetTpLed(HANDLE DeviceHandle, DWORD TpLedNumber, DWORD* Value);

//Invert TpLed signals (8 bits)
MNT DWORD MNT_ConfigTpLed(HANDLE DeviceHandle, DWORD Invert);

//Set Clock Mux (see Zeus schematic page 14)
MNT DWORD MNT_SetClockMux(HANDLE DeviceHandle, BYTE Mux);
MNT DWORD MNT_GetClockMux(HANDLE DeviceHandle, BYTE* Mux);

//Pass (DWORD) -1 to keep the old value
//OscEn - Enable TCXO (Page 12), Clk5Ven - Enable 5V for single-to-diff converter (Page 13), Clk2Ext2_xEn - Enable converters (Page 13)
MNT DWORD MNT_SetClockCtrl(HANDLE DeviceHandle, DWORD OscEn, DWORD Clk5VEn, DWORD ClkExt2_0En, DWORD ClkExt2_1En);
//Pass NULL if you don't need a value back
MNT DWORD MNT_GetClockCtrl(HANDLE DeviceHandle, DWORD* OscEn, DWORD* Clk5VEn, DWORD* ClkExt2_0En, DWORD* ClkExt2_1En);

//Communicate with SI chip (Zeus schematic page 12) via SPI interface. 
MNT DWORD MNT_SiWrite(HANDLE DeviceHandle, DWORD Address, DWORD Size, DWORD* Data);
MNT DWORD MNT_SiWriteEx(HANDLE DeviceHandle, DWORD Chip, DWORD Address, DWORD Size, DWORD* Data);
MNT DWORD MNT_SiRead(HANDLE DeviceHandle, DWORD Address, DWORD Size, DWORD* Data);
MNT DWORD MNT_SiReadEx(HANDLE DeviceHandle, DWORD Chip, DWORD Address, DWORD Size, DWORD* Data);
//Subset of MNT_SiWrite to write one register (no need to supply a pointer)
MNT DWORD MNT_SiSingleWrite(HANDLE DeviceHandle, DWORD Address, DWORD Data);
MNT DWORD MNT_SiSingleWriteEx(HANDLE DeviceHandle, DWORD Chip, DWORD Address, DWORD Data);

//Advanced functions to communicate with Si chip
MNT DWORD MNT_SiInit(HANDLE DeviceHandle, DWORD Chip);
MNT DWORD MNT_SiConfigureInput(HANDLE DeviceHandle, DWORD ChipID, DWORD InputNumber);
MNT DWORD MNT_SiConfigureExtOutput(HANDLE DeviceHandle, LONG* Mux);
//MNT DWORD MNT_SiSavePPMOffsets(HANDLE DeviceHandle, DWORD ChipID);
MNT DWORD MNT_SiCreatePPMOffset(HANDLE DeviceHandle, DWORD ChipID, DWORD N_Number, double PPMOffset, BOOL SaveFlag);
MNT DWORD MNT_SiGetPPMOffset(HANDLE DeviceHandle, DWORD ChipID, DWORD N_Number, double* PPMOffset);

MNT DWORD MNT_SiGetStatus(HANDLE DeviceHandle, DWORD* LOL);
MNT DWORD MNT_SiGetStatusEx(HANDLE DeviceHandle, DWORD Chip, DWORD* LOL);
MNT DWORD MNT_SiConfigureOutputEx(HANDLE DeviceHandle, DWORD Chip, DWORD NumberOfOutputs, DWORD* ChannelNumber, double* AmplitudeInmV, DWORD * Enable);
MNT DWORD MNT_SiConfigureOutputFormat(HANDLE DeviceHandle, DWORD ChipID, DWORD NumberOfOutputs, DWORD* ChannelNumber, 
															LONG* OutputFormat, LONG* CmosDrv, LONG* InvertPositive, LONG* InvertNegative);

MNT DWORD MNT_SiGetPartNumber(HANDLE DeviceHandle, DWORD ChipID, DWORD PartID[11], char* Name);
MNT DWORD MNT_SiGetDetaildStatus(HANDLE DeviceHandle, DWORD Chip, LONG* LOSXAXB, LONG* XAXB_ERR, LONG LOS[2], LONG OOF[2], LONG* LOL);

MNT DWORD MNT_SiLabCalculateSetClock(HANDLE, DWORD, double*, LONG, double* oFvco, double* oN0_DIV, ULONGLONG* oM_NUM, DWORD* oM_DEN, ULONGLONG* oN0_NUM, DWORD* oN0_DEN);
MNT DWORD MNT_SiLab5341_PreSetClock(HANDLE, DWORD, DWORD);
MNT DWORD MNT_SiLab5395_PreSetClock(HANDLE, DWORD, DWORD);

MNT DWORD MNT_SiLoadFromFileBuffer(HANDLE DeviceHandle, DWORD ChipID, DWORD ActualPartID_Rev, void* FileName, CHAR* DataBuffer);
MNT DWORD MNT_SiLoad(HANDLE DeviceHandle, DWORD ChipID, OlympusSIFunction* sSIFunction);
MNT DWORD MNT_SiGetMNTInfo(HANDLE DeviceHandle, OlympusSIFunction* sSIFunction, LONG* Size, CHAR* Result);
MNT DWORD MNT_SiGetDesignID(HANDLE DeviceHandle, DWORD ChipID, WCHAR DesignID[8]);

MNT DWORD MNT_PcieClockWrite(HANDLE DeviceHandle, DWORD Address, DWORD Size, BYTE* Data);
MNT DWORD MNT_PcieClockRead(HANDLE DeviceHandle, DWORD Address, DWORD Size, BYTE* Data);
MNT DWORD MNT_PcieClockGetPartNumber(HANDLE DeviceHandle, DWORD PartID[4], char* Name);
//Only supports 2 outputs for now. Part 9FGL02;
// Set or Get OutputEnable / OutputType: 0: Low / Low, 1: HiZ / HiZ, 2: Hi  / Low, 3: Low / Hi
MNT DWORD MNT_PcieOutputEnable(HANDLE DeviceHandle, LONG OutputEnable[8], LONG* OutputType);
MNT DWORD MNT_PcieAmplitudeCode(HANDLE DeviceHandle, LONG* Amplitude);
MNT DWORD MNT_PcieAmplitudeValue(HANDLE DeviceHandle, double* AmplitudeValue);
MNT DWORD MNT_PcieSpreadSpectrum(HANDLE DeviceHandle, LONG* SpreadSpectrum, LONG* SpreadSpectrumLatched);
MNT DWORD MNT_PcieSlew(HANDLE DeviceHandle, LONG Slew[8]);
MNT DWORD MNT_PcieRef(HANDLE DeviceHandle, LONG* Slew, LONG* PowerDown, LONG* RefOe);
MNT DWORD MNT_PciePullUpDown(HANDLE DeviceHandle, LONG PullUp[8], LONG PullDown[8]);
MNT DWORD MNT_PcieImpedanceCode(HANDLE DeviceHandle, LONG Impedance[8]);
MNT DWORD MNT_PcieImpedanceValue(HANDLE DeviceHandle, double ImpedanceValue[8]);

//Write SPI EEPROM
MNT DWORD MNT_EepromWrite(HANDLE DeviceHandle, DWORD Address, DWORD Size, DWORD* Data);
//Read SPI EEPROM
MNT DWORD MNT_EepromRead(HANDLE DeviceHandle, DWORD Address, DWORD Size, DWORD* Data);
//Access second half of EEPROM for all boards except Odyssey
MNT DWORD MNT_WriteCalValues(HANDLE DeviceHandle, DWORD Offset, DWORD SizeInBytes, BYTE* Data);
MNT DWORD MNT_ReadCalValues(HANDLE DeviceHandle, DWORD Offset, DWORD SizeInBytes, BYTE* Data);
//Access Last 32 bytes of EEPROM (backwards) to Strore Chip Info 
MNT DWORD MNT_WriteChipInfo(HANDLE DeviceHandle, BYTE Port, DWORD SizeInBytes, CHAR* Data);
MNT DWORD MNT_ReadChipInfo(HANDLE DeviceHandle, BYTE Port, DWORD SizeInBytes, CHAR* Data);
//Not Supported Yet
MNT DWORD MNT_WriteBoardDescription(HANDLE DeviceHandle, BYTE Port, DWORD Size, WCHAR* Data);
MNT DWORD MNT_ReadBoardDescription(HANDLE DeviceHandle, BYTE Port, DWORD* Size, WCHAR* Data);
//Update Product Code in the EEPROM
MNT DWORD MNT_ProductCode(HANDLE DeviceHandle, LONG* NewAssyField);
//Read Temperature Correction from the EEPROM
MNT DWORD MNT_ReadEEPROMTemperatureCorrection(HANDLE DeviceHandle, LONG GainCode[4], LONG OffsCode[4]);
//Write Temperature Correction to the EEPROM
MNT DWORD MNT_WriteEEPROMTemperatureCorrection(HANDLE DeviceHandle, LONG GainCode[4], LONG OffsCode[4]);

// I2C Commands
//08h 1 0 0 0 CFG0 read / write Configuration port0 register
//09h 1 0 0 1 CFG1 read / write Configuration port1 register
//0Ah 1 0 1 0 OUT0 read / write Output port0 register
//0Bh 1 0 1 1 OUT1 read / write Output port1 register
MNT DWORD MNT_WriteI2C(HANDLE DeviceHandle, BYTE DeviceAddress, BYTE Command, BYTE Value);
//00h 0 0 0 0 IN0 read only Input port 0 register
//01h 0 0 0 1 IN1 read only Input port 1 register
MNT DWORD MNT_ReadI2C(HANDLE DeviceHandle, BYTE DeviceAddress, BYTE Command, BYTE* Value);

MNT DWORD MNT_WriteI2CEx(HANDLE DeviceHandle, WORD DeviceAddress, WORD Command, BYTE* Value, DWORD Size, DWORD Mode);
MNT DWORD MNT_ReadI2CEx(HANDLE DeviceHandle, WORD DeviceAddress, WORD Command, BYTE* Value, DWORD Size, DWORD Mode);

MNT DWORD MNT_WriteI2CJanus(HANDLE DeviceHandle, BYTE Port, BYTE DeviceAddress, BYTE Command, BYTE Value);
MNT DWORD MNT_ReadI2CJanus(HANDLE DeviceHandle, BYTE Port, BYTE DeviceAddress, BYTE Command, BYTE* Value);

MNT DWORD MNT_WriteI2CStreamJanus(HANDLE DeviceHandle, BYTE Port, BYTE DeviceAddress, WORD Command, BYTE Size, BYTE* Value);
MNT DWORD MNT_ReadI2CStreamJanus(HANDLE DeviceHandle, BYTE Port, BYTE DeviceAddress, WORD Command, BYTE Size, BYTE* Value);

//Set FX3 I2C Speed in Hz (100000 - 100kHz, 400000 - 400 kHz)
//Please keep the default speed at 400 kHz. if set to 100kHz, please return to 400 kHz after access to slow device is done
MNT DWORD MNT_I2CSpeed(HANDLE DeviceHandle, DWORD SpeedInHz);

//Tachometer reading
MNT DWORD MNT_Tachometer(HANDLE DeviceHandle, double* tick, double* frequency, double* rpm);

//Temperature Reading. TMP411A, MCP9600 Thermocouple 
MNT DWORD MNT_Temperature(HANDLE DeviceHandle, DWORD SensorID, double* Temperature, double Resolution);
//Temperature set thresholds and additional configuration. Not implemented yet.
MNT DWORD MNT_TemperatureConfig(HANDLE DeviceHandle, DWORD NumberOfSensors, LONG* Limits, void* Configuration);
MNT DWORD MNT_TemperatureStatus(HANDLE DeviceHandle, DWORD SensorID, BYTE JanusI2CPort, LONG* Status);
//if Gain < -256 -> Return Data
MNT DWORD MNT_TemperatureCorrection(HANDLE DeviceHandle, DWORD SensorNumber, LONG* Gain,  LONG* Offset);

//Packet Commands:

//Create sequence. Recorder based. 
//Redirect MDIO commands to memory buffer
MNT DWORD MNT_RecordSequence(HANDLE DeviceHandle);
//Execute recorded sequence. Should use MNT_PlaySequenceWithresult instead
MNT DWORD MNT_PlaySequence(HANDLE DeviceHandle, DWORD Execute);
//Execute Recorded Sequence and return result. Execute == 0 -> do not run. Get Result Only, 2 -> Do not run if empty
MNT DWORD MNT_PlaySequenceWithResult(HANDLE DeviceHandle, DWORD* ReadBuffer, DWORD* BufferSize);
//if Size != 0 read Number of Values to read. if Size == 0 - return full buffer if Data != NULL. BufferSize may be NULL
//In the future this may be used to check result of POLL command
MNT DWORD MNT_ReadSequenceResult(HANDLE DeviceHandle, DWORD Size, DWORD* ReadBuffer, DWORD* BufferSize);

//Obsolete function
MNT DWORD MNT_ExecutePacket(HANDLE DeviceHandle, DWORD SizeOut, DWORD* DataOut, DWORD SizeIn, DWORD* DataIn);
//Advance User commands:
MNT DWORD MNT_ExecutePacketEx(HANDLE DeviceHandle, DWORD SizeOut, DWORD* DataOut, DWORD* SizeIn, DWORD* DataIn);

//MNT_ResetSequence is obsolete. DO NOT USE IT. 
MNT DWORD MNT_SetSequenceEntry(HANDLE DeviceHandle, DWORD Bus, DWORD Command, DWORD Address, DWORD Mask, DWORD Data);
MNT DWORD MNT_GetSequenceSize(HANDLE DeviceHandle,  DWORD* CommandSize, DWORD* ExpectedResultSize);

//IJTAG Related Function:
//Read Currently Active Configuration and ID for each Port
MNT DWORD MNT_ReadActiveIJTAGConfiguration(HANDLE DeviceHandle, LONG ID[2], DWORD ActiveIP[2]);

#ifndef IJTAG_MAXIMUM_NUMBER_OF_IPS
#define IJTAG_MAXIMUM_NUMBER_OF_IPS                 64      //7/1/2023: Decrease from 128 
#define IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS          32      //7/1/2023: Increase from 16
//#define IJTAG_DR_LENGTH_NUMBER_OF_ITEMS             4       //16 items * 8 = 128 bits / 32 = 4 registers 
#define IJTAG_DR_LENGTH_NUMBER_OF_ITEMS             8       //32 items * 8 = 128 bits / 32 = 8 registers 

#define IJTAG_MAXIMUM_DR_NUMBER_OF_BITS             256
#define IJTAG_MAXIMUM_DESELECT_NUMBER_OF_BITS       256
#define IJTAG_MAXIMUM_DR_NUMBER_OF_DWORDS           8

#define IJTAG_MAXIMUM_NUMBER_OF_BITS                128     //For POST or PRE
#define IJTAG_POST_PRE_NUMBER_OF_ITEMS              8       //4 post + 4 pre
#define IJTAG_POST_or_PRE_NUMBER_OF_ITEMS           4       //4 post / 4 pre

#define IJTAG_DR_DATA_NUMBER_OF_ITEMS               32      // 
#define IJTAG_MAXIMUM_IR_LENGTH                     32
#endif

MNT DWORD MNT_ParseIJTAGFile(	HANDLE DeviceHandle, 
								void* JsonFileName,					//Input File Name (CHAR or WCHAR)
								DWORD IDtoLoad,

								LONG *ID,							//Output ID Length
								OlympusVersion* JsonVersion,

								DWORD* TRST_N_NumberOfCycles,
								DWORD* IRLength,			//Output IR Length
								DWORD* IRData,				//Output IR Data

								DWORD* NumberOfIPs,			//Output Number of IPs
								DWORD NumberOfDRs[IJTAG_MAXIMUM_NUMBER_OF_IPS],		//Output Number of DRs for each IP
								DWORD DRLength[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS],	//Output DR Length for each DR in each IP
								DWORD DRData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS][IJTAG_MAXIMUM_DR_NUMBER_OF_DWORDS],	//Output DR Data. Maximum 8 DWORDs for each DR in each IP

								DWORD DeselectLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],		//Number of Bits for 64 IPs
								DWORD DeselectData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_DR_NUMBER_OF_DWORDS],		//De-select Data - 8 Registers per IP

								DWORD PRAMBitPosition[IJTAG_MAXIMUM_NUMBER_OF_IPS],	//PRAM Select Bit Position for each IP

								DWORD PostLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],		//Output Post Length for each IP
								DWORD PostData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_POST_or_PRE_NUMBER_OF_ITEMS],		//Output Post Data. Maximum 4 DWORDs for each IP
								DWORD PreLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],		//Output Post Length for each IP
								DWORD PreData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_POST_or_PRE_NUMBER_OF_ITEMS]);		//Output Pre Data. Maximum 4 DWORDs for each IP

MNT DWORD MNT_ConfigureIJTAG(	HANDLE DeviceHandle, 
										DWORD Port,
										DWORD NumberOfIPs,

										LONG ID,
										DWORD TRST_N_NumberOfCycles,
										DWORD IRLength,
										DWORD IRData,
										DWORD NumberOfDRs[IJTAG_MAXIMUM_NUMBER_OF_IPS],
										DWORD DRLength[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS],
										DWORD DRData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS][IJTAG_MAXIMUM_DR_NUMBER_OF_DWORDS],
										DWORD  DeselectLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],
										DWORD  DeselectData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_DR_NUMBER_OF_DWORDS],
										DWORD PRAMBitPosition[IJTAG_MAXIMUM_NUMBER_OF_IPS],

										DWORD PostLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],
										DWORD PostData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_POST_or_PRE_NUMBER_OF_ITEMS],
										DWORD PreLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],
										DWORD PreData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_POST_or_PRE_NUMBER_OF_ITEMS]);

MNT DWORD MNT_ParseAndConfigureIJTAG(	HANDLE DeviceHandle, 
										void* JsonFileName,
										LONG IDtoLoad,
										OlympusVersion* JsonVersion,
										DWORD Port);

//Read ID and All Configuration Data.
MNT DWORD MNT_ReadIJTAGConfiguration(	HANDLE DeviceHandle, 
										DWORD Port,
										DWORD NumberOfIPs,

										LONG* ID, 
										DWORD* ActiveIP, 
										DWORD* TRST_N_NumberOfCycles,
										DWORD *IRLength,
										DWORD *IRData,
										DWORD NumberOfDRs[IJTAG_MAXIMUM_NUMBER_OF_IPS],
										DWORD DRLength[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS],
										DWORD DRData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_MAXIMUM_NUMBER_OF_SELECT_DRS][IJTAG_MAXIMUM_DR_NUMBER_OF_DWORDS],
										DWORD PostLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],
										DWORD PostData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_POST_or_PRE_NUMBER_OF_ITEMS],
										DWORD PreLength[IJTAG_MAXIMUM_NUMBER_OF_IPS],
										DWORD PreData[IJTAG_MAXIMUM_NUMBER_OF_IPS][IJTAG_POST_or_PRE_NUMBER_OF_ITEMS]
									);

//Logging
MNT DWORD MNT_ConfigureErrorHandling(PVOID ErrorHandlingFunction, DWORD Mode);

//Configuration (JSON)
MNT DWORD MNT_CreateJSON(HANDLE DeviceHandle, DWORD JPort, DWORD Option, void* FileName);
MNT DWORD  MNT_JSONVersion(HANDLE DeviceHandle, DWORD  JPort, DWORD * Version, DWORD  *Date, DWORD * JsonType);
//For Apollo ONLY. Obsolete. Do not use
MNT DWORD MNT_LoadJSON(HANDLE DeviceHandle, DWORD Port, void* FileName);
MNT DWORD MNT_LoadJSONEx(HANDLE DeviceHandle, DWORD Port, DWORD BoardID, void* FileName);

//Janus
JAN DWORD JAN_CurrentSPIPort(HANDLE DeviceHandle, LONG* Port);	//Virtual
//JAN DWORD JAN_CurrentI2CPort(HANDLE DeviceHandle, char* Port);	//Actual
JAN DWORD JAN_ControlModes(	HANDLE DeviceHandle, 
							LONG* DetMuxR, 
							LONG* TCXOEn, 
							LONG* ExtTCXOEn,
							LONG* SiResetN,
							LONG* PwrCycle);

//Regression Board
JAN DWORD JAN_ChainDetect(HANDLE DeviceHandle, LONG ChainR, BYTE* LastDevice);
JAN DWORD JAN_ChainTemperature(HANDLE DeviceHandle, LONG ChainR, double Temperature[31]);
//If Board Address < 0 -> All Boards. If Limit < -128 -> Do not set
JAN DWORD JAN_ChainSetLimits(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, double LowLimit[2], double HighLimit[2]);
//-1, or NULL -> Do not change
JAN DWORD JAN_ChainSetModes(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, 
								SHORT Rate, SHORT OS, SHORT SD, SHORT FH[2], SHORT FL[2], SHORT T1[2], SHORT POL[2]);
JAN DWORD JAN_ChainLED(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, BOOL Led);
JAN DWORD JAN_ChainDecodeDeviceName(HANDLE MyDevice, LONG Option, LONG DeviceID, LONG AssyPCBVersion, LONG SerialNumber, WCHAR* DeviceName);
//Low level
JAN DWORD JAN_ChainClear(HANDLE DeviceHandle, LONG ChainR, LONG Reset, LONG Alert1, LONG Alert2);
//This function writes EEPROM. Do not use it often.
JAN DWORD JAN_ChainAddressInitialize(HANDLE DeviceHandle, LONG ChainR, BYTE Result[32]);
JAN DWORD JAN_ChainWriteEEPROM(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, WORD EEPROM[8]);
JAN DWORD JAN_ChainReadEEPROM(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, WORD EEPROM[8]);
JAN DWORD JAN_ChainReadEEPROMIDsOnly(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, WORD EEPROM[5]);
//If Board Address < 0 -> Write Register to All Boards
JAN DWORD JAN_ChainWriteRegister(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, LONG RegisterAddress, WORD Register);
//If Board Address < 0 -> Read Register to All Boards
JAN DWORD JAN_ChainReadRegister(HANDLE DeviceHandle, LONG ChainR, LONG BoardAddress, LONG RegisterAddress, WORD* Register);


JAN DWORD JAN_UARTSend(HANDLE DeviceHandle, DWORD Size, BYTE* Data);
JAN DWORD JAN_UARTReceive(HANDLE DeviceHandle, DWORD Size, BYTE* Data);

JAN DWORD JAN_ReadInternalVoltages(HANDLE DeviceHandle, double* Voltages);	//9 max

//PID Controller, not finished
/*
MNT DWORD MNT_PIDInit(HANDLE DeviceHandle,	float PCoef, float ICoef, float DCoef,
													float Target, float CurrentValue,
													float LowLimit, float HighLimit);
MNT DWORD MNT_PIDSetTarget(HANDLE DeviceHandle, float Target);
MNT DWORD MNT_PIDGetCoef(HANDLE DeviceHandle,	float* PCoef,  float* ICoef, float* DCoef,
														float* PCoefOriginal,  float* ICoefOriginal, float* DCoefOriginal);
MNT DWORD MNT_PIDSetLimits(	HANDLE DeviceHandle, float LowLimit, float HighLimit);
MNT DWORD MNT_PIDGetLimits(	HANDLE DeviceHandle, float* LowLimit, float* HighLimit);
MNT DWORD MNT_PIDGetResult(	HANDLE DeviceHandle, float CurrentValue, float* Control);
*/

//Zamboni Humidity
MNT DWORD HS_GetHWSerialNumber(HANDLE DeviceHandle, BYTE JanusI2CPort, DWORD* SerialNumber);
MNT DWORD HS_SoftReset(HANDLE DeviceHandle, BYTE JanusI2CPort);
MNT DWORD HS_Measure(HANDLE DeviceHandle, BYTE JanusI2CPort, BYTE Command, DWORD DelayInMS, double* RelativeHumidity, double* Temperature);
//Zamboni Thermocouple
MNT DWORD TC_GetDeviceID(HANDLE DeviceHandle, BYTE JanusI2CPort, WORD DeviceNumber, WORD* DeviceID);
MNT DWORD TC_GetTemperature(HANDLE DeviceHandle, BYTE JanusI2CPort, WORD DeviceNumber, double Resolution, double* Temperature);
MNT DWORD TC_GetEMFStatus(HANDLE DeviceHandle, BYTE JanusI2CPort, WORD DeviceNumber, BOOL* StatusBit4);
MNT DWORD TC_Configure(HANDLE DeviceHandle, BYTE JanusI2CPort, WORD DeviceNumber,
							SHORT* TC_Type, SHORT* Filter, 
							SHORT* SensorResolution, SHORT* ADCResolution);
MNT DWORD TC_WriteRegister(HANDLE DeviceHandle, BYTE JanusI2CPort, WORD DeviceNumber, WORD Address, WORD SizeInBytes, DWORD Data);
MNT DWORD TC_GetAllRegisters(HANDLE DeviceHandle, BYTE JanusI2CPort, WORD DeviceNumber, BYTE Result[31]);
//Zamboni ADC / DAC
MNT DWORD AD_WriteRegister(HANDLE DeviceHandle, BYTE JanusI2CPort, BYTE Address, BYTE SubAddress, WORD Data);
MNT DWORD AD_ReadRegister(HANDLE DeviceHandle, BYTE JanusI2CPort, BYTE Address, BYTE SubAddress, WORD* Data);
MNT DWORD AD_ZamboniReadVoltages(HANDLE DeviceHandle, BYTE JanusI2CPort, double Resolution, double Voltages[4]);
MNT DWORD AD_ZamboniReadCodes(HANDLE DeviceHandle, BYTE JanusI2CPort, DWORD Codes[4]);
MNT DWORD AD_ZamboniPeltierVoltage(HANDLE DeviceHandle, BYTE JanusI2CPort, double Resolution, double* Voltage);
MNT DWORD AD_ZamboniPeltierCode(HANDLE DeviceHandle, BYTE JanusI2CPort, LONG* Code);
MNT DWORD AD_ZamboniCurrentLimit(HANDLE DeviceHandle, BYTE JanusI2CPort, double Resolution, double* Limit);
MNT DWORD AD_ZamboniFanVoltage(HANDLE DeviceHandle, BYTE JanusI2CPort, DWORD FanNumber, double Resolution, double* Voltage);

//Zamboni GPIO
MNT DWORD ZamboniGPIO(HANDLE DeviceHandle, BYTE JanusI2CPort, BOOL SecondPort, LONG* Direction, LONG* Value);
MNT DWORD ZamboniGPIORMW(HANDLE DeviceHandle, BYTE JanusI2CPort, BOOL SecondPort, BYTE Mask, BYTE Value);

MNT DWORD ZamboniPeltierDirection(HANDLE DeviceHandle, BYTE JanusI2CPort, CHAR* ToHot);

//Zamboni Global
MNT DWORD ZamboniInfo(HANDLE DeviceHandle, BYTE JanusI2CPort, BOOL* Mode, BOOL* Enabled, BOOL* Present, OlympusZamboniInfo* zInfo);
MNT DWORD ZamboniInit(HANDLE DeviceHandle, BYTE JanusI2CPort);

MNT DWORD ZamboniGetConfig(HANDLE DeviceHandle, BYTE JanusI2CPort, OlympusZamboniConfig* Config);
MNT DWORD ZamboniSetConfig(HANDLE DeviceHandle, BYTE JanusI2CPort, OlympusZamboniConfig* Config);
MNT DWORD ZamboniEnable(HANDLE DeviceHandle, BYTE JanusI2CPort, LONG *Enable);
MNT DWORD ZamboniTemperature(HANDLE DeviceHandle, BYTE JanusI2CPort, float SetTemperature, float* CurrentTemperature, float Resolution, WORD SlewRateDegMin);
MNT DWORD ZamboniGetStatus(HANDLE DeviceHandle, BYTE JanusI2CPort, OlympusZamboniStatus* ZamboniStatus);

#ifdef __cplusplus
	}
#endif

#endif

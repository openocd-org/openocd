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

#ifndef OPENOCD_LOADERS_FLASH_MSP432_MSP432E4X_H
#define OPENOCD_LOADERS_FLASH_MSP432_MSP432E4X_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register map for FLASH_CTRL peripheral (FLASH_CTRL) */
struct flash_ctrl {
	volatile uint32_t FMA;       /* Flash Memory Address */
	volatile uint32_t FMD;       /* Flash Memory Data */
	volatile uint32_t FMC;       /* Flash Memory Control */
	volatile uint32_t FCRIS;     /* Flash Controller Raw Interrupt Status */
	volatile uint32_t FCIM;      /* Flash Controller Interrupt Mask */
	volatile uint32_t FCMISC;    /* Flash Cont. Masked Int. Status and Clear */
	volatile uint32_t RESERVED0[2];
	volatile uint32_t FMC2;      /* Flash Memory Control 2 */
	volatile uint32_t RESERVED1[3];
	volatile uint32_t FWBVAL;    /* Flash Write Buffer Valid */
	volatile uint32_t RESERVED2[2];
	volatile uint32_t FLPEKEY;   /* Flash Program/Erase Key */
	volatile uint32_t RESERVED3[48];
	volatile uint32_t FWBN[32];  /* Flash Write Buffer n */
};

/* Register map for SYSCTL peripheral (SYSCTL) */
struct sys_ctrl {
	volatile uint32_t DID0;      /* Device Identification 0 */
	volatile uint32_t DID1;      /* Device Identification 1 */
	volatile uint32_t RESERVED0[12];
	volatile uint32_t PTBOCTL;   /* Power-Temp Brown Out Control */
	volatile uint32_t RESERVED1[5];
	volatile uint32_t RIS;       /* Raw Interrupt Status */
	volatile uint32_t IMC;       /* Interrupt Mask Control */
	volatile uint32_t MISC;      /* Masked Interrupt Status and Clear */
	volatile uint32_t RESC;      /* Reset Cause */
	volatile uint32_t PWRTC;     /* Power-Temperature Cause */
	volatile uint32_t NMIC;      /* NMI Cause Register */
	volatile uint32_t RESERVED2[5];
	volatile uint32_t MOSCCTL;   /* Main Oscillator Control */
	volatile uint32_t RESERVED3[12];
	volatile uint32_t RSCLKCFG;  /* Run and Sleep Mode Configuration Register */
	volatile uint32_t RESERVED4[3];
	volatile uint32_t MEMTIM0;   /* Memory Timing Register 0 for Main Flash */
	volatile uint32_t RESERVED5[29];
	volatile uint32_t ALTCLKCFG; /* Alternate Clock Configuration */
	volatile uint32_t RESERVED6[2];
	union {
		volatile uint32_t DSLPCLKCFG; /* Deep Sleep Clock Configuration */
		volatile uint32_t DSCLKCFG;   /* Deep Sleep Clock Register */
	};
	volatile uint32_t DIVSCLK;   /* Divisor and Source Clock Configuration */
	volatile uint32_t SYSPROP;   /* System Properties */
	volatile uint32_t PIOSCCAL;  /* Precision Internal Oscillator Calibration */
	volatile uint32_t PIOSCSTAT; /* Precision Internal Oscillator Statistics */
	volatile uint32_t RESERVED7[2];
	volatile uint32_t PLLFREQ0;  /* PLL Frequency 0 */
	volatile uint32_t PLLFREQ1;  /* PLL Frequency 1 */
	volatile uint32_t PLLSTAT;   /* PLL Status */
	volatile uint32_t RESERVED8[7];
	volatile uint32_t SLPPWRCFG; /* Sleep Power Configuration */
	volatile uint32_t DSLPPWRCFG; /* Deep-Sleep Power Configuration */
	volatile uint32_t RESERVED9[4];
	volatile uint32_t NVMSTAT;   /* Non-Volatile Memory Information */
	volatile uint32_t RESERVED10[4];
	volatile uint32_t LDOSPCTL;  /* LDO Sleep Power Control */
	volatile uint32_t RESERVED11;
	volatile uint32_t LDODPCTL;  /* LDO Deep-Sleep Power Control */
	volatile uint32_t RESERVED12[6];
	volatile uint32_t RESBEHAVCTL; /* Reset Behavior Control Register */
	volatile uint32_t RESERVED13[6];
	volatile uint32_t HSSR;      /* Hardware System Service Request */
	volatile uint32_t RESERVED14[34];
	volatile uint32_t USBPDS;    /* USB Power Domain Status */
	volatile uint32_t USBMPC;    /* USB Memory Power Control */
	volatile uint32_t EMACPDS;   /* Ethernet MAC Power Domain Status */
	volatile uint32_t EMACMPC;   /* Ethernet MAC Memory Power Control */
	volatile uint32_t RESERVED15;
	volatile uint32_t LCDMPC;    /* LCD Memory Power Control */
	volatile uint32_t RESERVED16[26];
	volatile uint32_t PPWD;      /* Watchdog Timer Peripheral Present */
	volatile uint32_t PPTIMER;   /* General-Purpose Timer Peripheral Present */
	volatile uint32_t PPGPIO;    /* General-Purpose I/O Peripheral Present */
	volatile uint32_t PPDMA;     /* Micro DMA Peripheral Present */
	volatile uint32_t PPEPI;     /* EPI Peripheral Present */
	volatile uint32_t PPHIB;     /* Hibernation Peripheral Present */
	volatile uint32_t PPUART;    /* UART Peripheral Present */
	volatile uint32_t PPSSI;     /* Synchronous Serial Inter. Periph. Present */
	volatile uint32_t PPI2C;     /* Inter-Integrated Circuit Periph. Present */
	volatile uint32_t RESERVED17;
	volatile uint32_t PPUSB;     /* Universal Serial Bus Peripheral Present */
	volatile uint32_t RESERVED18;
	volatile uint32_t PPEPHY;    /* Ethernet PHY Peripheral Present */
	volatile uint32_t PPCAN;     /* Controller Area Network Periph. Present */
	volatile uint32_t PPADC;     /* Analog-to-Dig. Converter Periph. Present */
	volatile uint32_t PPACMP;    /* Analog Comparator Peripheral Present */
	volatile uint32_t PPPWM;     /* Pulse Width Modulator Peripheral Present */
	volatile uint32_t PPQEI;     /* Quadrature Encoder Inter. Periph. Present */
	volatile uint32_t RESERVED19[4];
	volatile uint32_t PPEEPROM;  /* EEPROM Peripheral Present */
	volatile uint32_t RESERVED20[6];
	volatile uint32_t PPCCM;     /* CRC/Cryptographic Modules Periph. Present */
	volatile uint32_t RESERVED21[6];
	volatile uint32_t PPLCD;     /* LCD Peripheral Present */
	volatile uint32_t RESERVED22;
	volatile uint32_t PPOWIRE;   /* 1-Wire Peripheral Present */
	volatile uint32_t PPEMAC;    /* Ethernet MAC Peripheral Present */
	volatile uint32_t RESERVED23[88];
	volatile uint32_t SRWD;      /* Watchdog Timer Software Reset */
	volatile uint32_t SRTIMER;   /* General-Purpose Timer Software Reset */
	volatile uint32_t SRGPIO;    /* General-Purpose I/O Software Reset */
	volatile uint32_t SRDMA;     /* Micro Direct Memory Access Software Reset */
	volatile uint32_t SREPI;     /* EPI Software Reset */
	volatile uint32_t SRHIB;     /* Hibernation Software Reset */
	volatile uint32_t SRUART;    /* UART Software Reset */
	volatile uint32_t SRSSI;     /* Synchronous Serial Inter. Software Reset */
	volatile uint32_t SRI2C;     /* Inter-Integrated Circuit Software Reset */
	volatile uint32_t RESERVED24;
	volatile uint32_t SRUSB;     /* Universal Serial Bus Software Reset */
	volatile uint32_t RESERVED25;
	volatile uint32_t SREPHY;    /* Ethernet PHY Software Reset */
	volatile uint32_t SRCAN;     /* Controller Area Network Software Reset */
	volatile uint32_t SRADC;     /* Analog-to-Dig. Converter Software Reset */
	volatile uint32_t SRACMP;    /* Analog Comparator Software Reset */
	volatile uint32_t SRPWM;     /* Pulse Width Modulator Software Reset */
	volatile uint32_t SRQEI;     /* Quadrature Encoder Inter. Software Reset */
	volatile uint32_t RESERVED26[4];
	volatile uint32_t SREEPROM;  /* EEPROM Software Reset */
	volatile uint32_t RESERVED27[6];
	volatile uint32_t SRCCM;     /* CRC/Cryptographic Modules Software Reset */
	volatile uint32_t RESERVED28[6];
	volatile uint32_t SRLCD;     /* LCD Controller Software Reset */
	volatile uint32_t RESERVED29;
	volatile uint32_t SROWIRE;   /* 1-Wire Software Reset */
	volatile uint32_t SREMAC;    /* Ethernet MAC Software Reset */
	volatile uint32_t RESERVED30[24];
	volatile uint32_t RCGCWD;    /* Watchdog Run Mode Clock Gating Control */
};

/* Peripheral Memory Map */
#define FLASH_CTRL_BASE 0x400FD000UL
#define SYSCTL_BASE     0x400FE000UL

/* Peripheral Declarations */
#define FLASH_CTRL ((struct flash_ctrl *) FLASH_CTRL_BASE)
#define SYSCTL     ((struct sys_ctrl *) SYSCTL_BASE)

/* The following are defines for the bit fields in the FLASH_FMC register. */
#define FLASH_FMC_WRKEY  0xA4420000 /* FLASH write key */
#define FLASH_FMC_COMT   0x00000008 /* Commit Register Value */
#define FLASH_FMC_MERASE 0x00000004 /* Mass Erase Flash Memory */
#define FLASH_FMC_ERASE  0x00000002 /* Erase a Page of Flash Memory */
#define FLASH_FMC_WRITE  0x00000001 /* Write a Word into Flash Memory */

/* The following are defines for the bit fields in the FLASH_FCRIS register. */
#define FLASH_FCRIS_PROGRIS 0x00002000 /* Program Verify Raw Interrupt Status */
#define FLASH_FCRIS_ERRIS   0x00000800 /* Erase Verify Raw Interrupt Status */
#define FLASH_FCRIS_INVDRIS 0x00000400 /* Invalid Data Raw Interrupt Status */
#define FLASH_FCRIS_VOLTRIS 0x00000200 /* Pump Voltage Raw Interrupt Status */
#define FLASH_FCRIS_ERIS    0x00000004 /* EEPROM Raw Interrupt Status */
#define FLASH_FCRIS_PRIS    0x00000002 /* Programming Raw Interrupt Status */
#define FLASH_FCRIS_ARIS    0x00000001 /* Access Raw Interrupt Status */

/* The following are defines for the bit fields in the FLASH_FCIM register. */
#define FLASH_FCIM_PROGMASK 0x00002000 /* PROGVER Interrupt Mask */
#define FLASH_FCIM_ERMASK   0x00000800 /* ERVER Interrupt Mask */
#define FLASH_FCIM_INVDMASK 0x00000400 /* Invalid Data Interrupt Mask */
#define FLASH_FCIM_VOLTMASK 0x00000200 /* VOLT Interrupt Mask */
#define FLASH_FCIM_EMASK    0x00000004 /* EEPROM Interrupt Mask */
#define FLASH_FCIM_PMASK    0x00000002 /* Programming Interrupt Mask */
#define FLASH_FCIM_AMASK    0x00000001 /* Access Interrupt Mask */

/* The following are defines for the bit fields in the FLASH_FCMISC register. */
#define FLASH_FCMISC_PROGMISC 0x00002000 /* PROGVER Interrupt Status/Clear */
#define FLASH_FCMISC_ERMISC   0x00000800 /* ERVER Interrupt Status/Clear */
#define FLASH_FCMISC_INVDMISC 0x00000400 /* Invalid Data Int. Status/Clear */
#define FLASH_FCMISC_VOLTMISC 0x00000200 /* VOLT Interrupt Status/Clear */
#define FLASH_FCMISC_EMISC    0x00000004 /* EEPROM Interrupt Status/Clear */
#define FLASH_FCMISC_PMISC    0x00000002 /* Programming Int. Status/Clear */
#define FLASH_FCMISC_AMISC    0x00000001 /* Access Interrupt Status/Clear */

/* The following are defines for the bit fields in the FLASH_FMC2 register. */
#define FLASH_FMC2_WRBUF 0x00000001 /* Buffered Flash Memory Write */

/* The following are defines for the bit fields in the SYSCTL_RCGCWD reg. */
#define SYSCTL_RCGCWD_R1 0x00000002 /* Watchdog 1 Run Mode Clock Gating Cont. */
#define SYSCTL_RCGCWD_R0 0x00000001 /* Watchdog 0 Run Mode Clock Gating Cont. */

#ifdef __cplusplus
}
#endif

#endif  /* OPENOCD_LOADERS_FLASH_MSP432_MSP432E4X_H */

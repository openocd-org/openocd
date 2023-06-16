/* SPDX-License-Identifier: GPL-2.0-or-later */
/****************************************************************************
    File : reg_ezusb.h                                                      *
    Contents : FX2 microcontroller registers file for NanoXplore            *
    USB-JTAG ANGIE adapter hardware.                                        *
    Based on openULINK project code by: Martin Schmoelzer.                  *
    Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.              *
    <aboudjelida@nanoxplore.com>                                            *
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#ifndef REG_EZUSB_H
#define REG_EZUSB_H

/**
 * @file
 * All information in this file was taken from the EZ-USB FX2 Technical
 * Reference Manual, Cypress Semiconductor, 3901 North First Street
 * San Jose, CA 95134 (www.cypress.com).
 *
 * The EZ-USB Technical Reference Manual is called "EZ-USB FX2 TRM" hereafter.
 */

/* Compiler-specific definitions of SBIT, SFR, SFRX, ... macros */
#include <mcs51/compiler.h>

/* Bit vectors */
#define bmbit0      0x01
#define bmbit1      0x02
#define bmbit2      0x04
#define bmbit3      0x08
#define bmbit4      0x10
#define bmbit5      0x20
#define bmbit6      0x40
#define bmbit7      0x80

/**************************************************************************
 ************************ Special Function Registers **********************
 ***************************************************************************/
SFR(IOA,            0x80);
SBIT(IOA0,          0x80, 0);
SBIT(IOA1,          0x80, 1);
SBIT(IOA2,          0x80, 2);
SBIT(IOA3,          0x80, 3);
SBIT(IOA4,          0x80, 4);
SBIT(IOA5,          0x80, 5);
SBIT(IOA6,          0x80, 6);
SBIT(IOA7,          0x80, 7);

SFR(SP,             0x81);
SFR(DPL0,           0x82);
SFR(DPH0,           0x83);
SFR(DPL1,           0x84);
SFR(DPL2,           0x85);

SFR(DPS,            0x86);
#define SEL         bmbit0
/* Bit 1 read-only, always reads '0' */
/* Bit 2 read-only, always reads '0' */
/* Bit 3 read-only, always reads '0' */
/* Bit 4 read-only, always reads '0' */
/* Bit 5 read-only, always reads '0' */
/* Bit 6 read-only, always reads '0' */
/* Bit 7 read-only, always reads '0' */

SFR(PCON,           0x87);
#define IDLE        bmbit0
#define STOP        bmbit1
#define GF0         bmbit2
#define GF1         bmbit3
/* Bit 4 read-only, always reads '1' */
/* Bit 5 read-only, always reads '1' */
/* Bit 6 unused */
#define SMOD0     bmbit7

SFR(TCON,           0x88);
SBIT(IT0,           0x88, 0);
SBIT(IE0,           0x88, 1);
SBIT(IT1,           0x88, 2);
SBIT(IE1,           0x88, 3);
SBIT(TR0,           0x88, 4);
SBIT(TF0,           0x88, 5);
SBIT(TR1,           0x88, 6);
SBIT(TF1,           0x88, 7);

SFR(TMOD,           0x89);
SFR(TL0,            0x8A);
SFR(TL1,            0x8B);
SFR(TH0,            0x8C);
SFR(TH1,            0x8D);

SFR(CKCON,          0x8E);
#define MD0         bmbit0
#define MD1         bmbit1
#define MD2         bmbit2
#define T0M         bmbit3
#define T1M         bmbit4
#define T2M         bmbit5
/* Bit 6 unused */
/* Bit 7 unused */

SFR(SPC_FNC,        0x8F);
#define BMWRS       bmbit0
/* Bit 1 read-only, always reads '0' */
/* Bit 2 read-only, always reads '0' */
/* Bit 3 read-only, always reads '0' */
/* Bit 4 read-only, always reads '0' */
/* Bit 5 read-only, always reads '0' */
/* Bit 6 read-only, always reads '0' */
/* Bit 7 read-only, always reads '0' */

SFR(IOB,            0x90);
SBIT(IOB0,          0x90, 0);
SBIT(IOB1,          0x90, 1);
SBIT(IOB2,          0x90, 2);
SBIT(IOB3,          0x90, 3);
SBIT(IOB4,          0x90, 4);
SBIT(IOB5,          0x90, 5);
SBIT(IOB6,          0x90, 6);
SBIT(IOB7,          0x90, 7);

SFR(EXIF,           0x91);
SBIT(USBINT,        0x91, 4);
SBIT(I2CINT,        0x91, 5);
SBIT(IE4,           0x91, 6);
SBIT(IE5,           0x91, 7);

SFR(MPAGE,          0x92);
SFR(SCON0,          0x98);
SBIT(RI,            0x98, 0);
SBIT(TI,            0x98, 1);
SBIT(RB8,           0x98, 2);
SBIT(TB8,           0x98, 3);
SBIT(REN,           0x98, 4);
SBIT(SM2,           0x98, 5);
SBIT(SM1,           0x98, 6);
SBIT(SM0,           0x98, 7);

SFR(SBUF0,          0x99);
SFR(AUTOPTRH1,      0x9A);
SFR(AUTOPTRL1,      0x9B);
SFR(AUTOPTRH2,      0x9D);
SFR(AUTOPTRL2,      0x9E);

#define AUTOPTR1H AUTOPTRH1 /* for backwards compatibility with examples */
#define AUTOPTR1L AUTOPTRL1 /* for backwards compatibility with examples */
#define APTR1H AUTOPTRH1 /* for backwards compatibility with examples */
#define APTR1L AUTOPTRL1 /* for backwards compatibility with examples */

SFR(IOC,            0xA0);
SBIT(IOC0,          0xA0, 0);
SBIT(IOC1,          0xA0, 1);
SBIT(IOC2,          0xA0, 2);
SBIT(IOC3,          0xA0, 3);
SBIT(IOC4,          0xA0, 4);
SBIT(IOC5,          0xA0, 5);
SBIT(IOC6,          0xA0, 6);
SBIT(IOC7,          0xA0, 7);

SFR(INT2CLR,        0xA1);
SFR(INT4CLR,        0xA2);
SFR(IE,             0xA8);
SBIT(EX0,           0xA8, 0);
SBIT(ET0,           0xA8, 1);
SBIT(EX1,           0xA8, 2);
SBIT(ET1,           0xA8, 3);
SBIT(ES0,           0xA8, 4);
SBIT(ET2,           0xA8, 5);
SBIT(ES1,           0xA8, 6);
SBIT(EA,            0xA8, 7);

SFR(EP2468STAT,     0xAA);
#define EP8F        bmbit7
#define EP8E        bmbit6
#define EP6F        bmbit5
#define EP6E        bmbit4
#define EP4F        bmbit3
#define EP4E        bmbit2
#define EP2F        bmbit1
#define EP2E        bmbit0

SFR(EP24FIFOFLGS,   0xAB);
SFR(EP68FIFOFLGS,   0xAC);
SFR(AUTOPTRSETUP,   0xAF);
SFR(IOD,            0xB0);
SBIT(IOD0,          0xB0, 0);
SBIT(IOD1,          0xB0, 1);
SBIT(IOD2,          0xB0, 2);
SBIT(IOD3,          0xB0, 3);
SBIT(IOD4,          0xB0, 4);
SBIT(IOD5,          0xB0, 5);
SBIT(IOD6,          0xB0, 6);
SBIT(IOD7,          0xB0, 7);

SFR(IOE,            0xB1);
SFR(OEA,            0xB2);
SFR(OEB,            0xB3);
SFR(OEC,            0xB4);
SFR(OED,            0xB5);
SFR(OEE,            0xB6);

SFR(IP,             0xB8);
SBIT(PX0,           0xB8, 0);
SBIT(PT0,           0xB8, 1);
SBIT(PX1,           0xB8, 2);
SBIT(PT1,           0xB8, 3);
SBIT(PS0,           0xB8, 4);
SBIT(PT2,           0xB8, 5);
SBIT(PS1,           0xB8, 6);
/* Bit 7 read-only, always reads '1' */

SFR(EP01STAT,       0xBA);
SFR(GPIFTRIG,       0xBB);
#define BMGPIFDONE  bmbit7
#define BMGPIFREAD  bmbit2
#define GPIF_EP2    0
#define GPIF_EP4    1
#define GPIF_EP6    2
#define GPIF_EP8    3

SFR(GPIFSGLDATH,    0xBD);
SFR(GPIFSGLDATLX,   0xBE);
SFR(GPIFSGLDATLNOX, 0xBF);

SFR(SCON1,          0xC0);
SBIT(RI_1,          0xC0, 0);
SBIT(TI_1,          0xC0, 1);
SBIT(RB8_1,         0xC0, 2);
SBIT(TB8_1,         0xC0, 3);
SBIT(REN_1,         0xC0, 4);
SBIT(SM2_1,         0xC0, 5);
SBIT(SM1_1,         0xC0, 6);
SBIT(SM0_1,         0xC0, 7);

SFR(SBUF1,          0xC1);
SFR(T2CON,          0xC8);
SBIT(CPRL2,         0xC8, 0);
SBIT(C_T2,          0xC8, 1);
SBIT(TR2,           0xC8, 2);
SBIT(EXEN2,         0xC8, 3);
SBIT(TCLK,          0xC8, 4);
SBIT(RCLK,          0xC8, 5);
SBIT(EXF2,          0xC8, 6);
SBIT(TF2,           0xC8, 7);

SFR(RCAP2L,         0xCA);
SFR(RCAP2H,         0xCB);
SFR(TL2,            0xCC);
SFR(TH2,            0xCD);
SFR(PSW,            0xD0);
SBIT(P,             0xD0, 0);
SBIT(F1,            0xD0, 1);
SBIT(OV,            0xD0, 2);
SBIT(RS0,           0xD0, 3);
SBIT(RS1,           0xD0, 4);
SBIT(F0,            0xD0, 5);
SBIT(AC,            0xD0, 6);
SBIT(CY,            0xD0, 7);

SFR(EICON,          0xD8);
/* Bit 0 read-only, always reads '0' */
/* Bit 1 read-only, always reads '0' */
/* Bit 2 read-only, always reads '0' */
SBIT(INT6,          0xD8, 3);
SBIT(RESI,          0xD8, 4);
SBIT(ERESI,         0xD8, 5);
/* Bit 6 read-only, always reads '1' */
SBIT(SMOD1,         0xD8, 7);

SFR(ACC,            0xE0);
SFR(EIE,            0xE8);
SBIT(EUSB,          0xE8, 0);
SBIT(EI2C,          0xE8, 1);
SBIT(EX4,           0xE8, 2);
SBIT(EX5,           0xE8, 3);
SBIT(EWDI,          0xE8, 4);
/* Bit 5 read-only, always reads '1' */
/* Bit 6 read-only, always reads '1' */
/* Bit 7 read-only, always reads '1' */

SFR(B,              0xF0);
SFR(EIP,            0xF8);
SBIT(PUSB,          0xF8, 0);
SBIT(PI2C,          0xF8, 1);
SBIT(PX4,           0xF8, 2);
SBIT(PX5,           0xF8, 3);
SBIT(PX6,           0xF8, 4);
/* Bit 5 read-only, always reads '1' */
/* Bit 6 read-only, always reads '1' */
/* Bit 7 read-only, always reads '1' */

/**************************************************************************
 ***************************** XDATA Registers ****************************
 ***************************************************************************/

SFRX(GPIF_WAVE_DATA,         0xE400);
SFRX(RES_WAVEDATA_END,       0xE480);

/* General Configuration */
SFRX(CPUCS,         0xE600);
#define RES8051     bmbit0
#define CLKOE       bmbit1
#define BMCLKINV    bmbit2
#define bmclkspd0   bmbit3
#define bmclkspd1   bmbit4
#define bmclkspd   (bmbit4 | bmbit3)
#define BMPRTCSTB   bmbit5

/* PCON register */
#define BMSMOD0     bmbit7

SFRX(IFCONFIG,        0xE601);
#define BMIFCLKSRC    bmbit7
#define BM3048MHZ     bmbit6
#define BMIFCLKOE     bmbit5
#define BMIFCLKPOL    bmbit4
#define BMASYNC       bmbit3
#define BMGSTATE      bmbit2
#define BMIFCFG1      bmbit1
#define BMIFCFG0      bmbit0
#define BMIFCFGMASK   (BMIFCFG0 | BMIFCFG1)
#define BMIFGPIF      BMIFCFG1

SFRX(PINFLAGSAB,      0xE602);
SFRX(PINFLAGSCD,      0xE603);
SFRX(FIFORESET,       0xE604);
#define BMNAKALL      bmbit7

SFRX(BREAKPT,         0xE605);
#define BMBREAK       bmbit3
#define BMBPPULSE     bmbit2
#define BMBPEN        bmbit1

SFRX(BPADDRH,         0xE606);
SFRX(BPADDRL,         0xE607);
SFRX(UART230,         0xE608);
SFRX(FIFOPINPOLAR,    0xE609);
SFRX(REVID,           0xE60A);
SFRX(REVCTL,          0xE60B);
#define BMNOAUTOARM    bmbit1
#define BMSKIPCOMMIT   bmbit0

/* Endpoint Configuration */
SFRX(EP1OUTCFG,       0xE610);
SFRX(EP1INCFG,        0xE611);
SFRX(EP2CFG,          0xE612);
SFRX(EP4CFG,          0xE613);
SFRX(EP6CFG,          0xE614);
SFRX(EP8CFG,          0xE615);
SFRX(EP2FIFOCFG,      0xE618);
SFRX(EP4FIFOCFG,      0xE619);
SFRX(EP6FIFOCFG,      0xE61A);
SFRX(EP8FIFOCFG,      0xE61B);
#define BMINFM        bmbit6
#define BMOEP         bmbit5
#define BMAUTOOUT     bmbit4
#define BMAUTOIN      bmbit3
#define BMZEROLENIN   bmbit2
#define BMWORDWIDE    bmbit0

SFRX(EP2AUTOINLENH,   0xE620);
SFRX(EP2AUTOINLENL,   0xE621);
SFRX(EP4AUTOINLENH,   0xE622);
SFRX(EP4AUTOINLENL,   0xE623);
SFRX(EP6AUTOINLENH,   0xE612);
SFRX(EP6AUTOINLENL,   0xE613);
SFRX(EP8AUTOINLENH,   0xE614);
SFRX(EP8AUTOINLENL,   0xE615);
SFRX(EP2FIFOPFH,      0xE630);
SFRX(EP2FIFOPFL,      0xE631);
SFRX(EP4FIFOPFH,      0xE632);
SFRX(EP4FIFOPFL,      0xE633);
SFRX(EP6FIFOPFH,      0xE634);
SFRX(EP6FIFOPFL,      0xE635);
SFRX(EP8FIFOPFH,      0xE636);
SFRX(EP8FIFOPFL,      0xE637);
SFRX(EP2ISOINPKTS,    0xE640);
SFRX(EP4ISOINPKTS,    0xE641);
SFRX(EP6ISOINPKTS,    0xE642);
SFRX(EP8ISOINPKTS,    0xE643);
SFRX(INPKTEND,        0xE648);
SFRX(OUTPKTEND,       0xE649);

/* Interrupts */
SFRX(EP2FIFOIE,       0xE650);
SFRX(EP2FIFOIRQ,      0xE651);
SFRX(EP4FIFOIE,       0xE652);
SFRX(EP4FIFOIRQ,      0xE653);
SFRX(EP6FIFOIE,       0xE654);
SFRX(EP6FIFOIRQ,      0xE655);
SFRX(EP8FIFOIE,       0xE656);
SFRX(EP8FIFOIRQ,      0xE657);
SFRX(IBNIE,           0xE658);
SFRX(IBNIRQ,          0xE659);
#define EP0IBN        bmbit0
#define EP1IBN        bmbit1
#define EP2IBN        bmbit2
#define EP4IBN        bmbit3
#define EP6IBN        bmbit4
#define EP8IBN        bmbit5

SFRX(NAKIE,           0xE65A);
SFRX(NAKIRQ,          0xE65B);
#define EP8PING       bmbit7
#define EP6PING       bmbit6
#define EP4PING       bmbit5
#define EP2PING       bmbit4
#define EP1PING       bmbit3
#define EP0PING       bmbit2
#define IBN           bmbit0

SFRX(USBIEN,          0xE65C);
SFRX(USBIRQ,          0xE65D);
#define SUDAVI        bmbit0
#define SOFI          bmbit1
#define SUTOKI        bmbit2
#define SUSPI         bmbit3
#define URESI         bmbit4
#define HSGRANT       bmbit5
#define EP0ACK        bmbit6

SFRX(EPIE,            0xE65E);
SFRX(EPIRQ,           0xE65F);
SFRX(GPIFIE,          0xE660);
SFRX(GPIFIRQ,         0xE661);
SFRX(USBERRIE,        0xE662);
SFRX(USBERRIRQ,       0xE663);
SFRX(ERRCNTLIM,       0xE664);
SFRX(CLRERRCNT,       0xE665);
SFRX(INT2IVEC,        0xE666);
#define I2V0          bmbit2
#define I2V1          bmbit3
#define I2V2          bmbit4
#define I2V3          bmbit5
#define I2V4          bmbit6

SFRX(INT4IVEC,        0xE667);
SFRX(INTSETUP,        0xE668);
#define AV4EN         bmbit0
#define INT4IN        bmbit1
#define AV2EN         bmbit3

/* Input/Output */
SFRX(PORTACFG,        0xE670);
#define BMINT0        bmbit0
#define BMINT1        bmbit1
#define BMFLAGD       bmbit7

SFRX(PORTCCFG,        0xE671);
#define BMGPIFA0      bmbit0
#define BMGPIFA1      bmbit1
#define BMGPIFA2      bmbit2
#define BMGPIFA3      bmbit3
#define BMGPIFA4      bmbit4
#define BMGPIFA5      bmbit5
#define BMGPIFA6      bmbit6
#define BMGPIFA7      bmbit7

SFRX(PORTECFG,        0xE672);
#define BMT0OUT       bmbit0
#define BMT1OUT       bmbit1
#define BMT2OUT       bmbit2
#define BMRXD0OUT     bmbit3
#define BMRXD1OUT     bmbit4
#define BMINT6        bmbit5
#define BMT2EX        bmbit6
#define BMGPIFA8      bmbit7

SFRX(I2CS,            0xE678);
#define BMDONE        bmbit0
#define BMACK         bmbit1
#define BMBERR        bmbit2
#define BMID         (bmbit4 | bmbit3)
#define BMLASTRD      bmbit5
#define BMSTOP        bmbit6
#define BMSTART       bmbit7

SFRX(I2DAT,           0xE679);
SFRX(I2CTL,           0xE67A);
#define BMSTOPIE      bmbit1
#define BM400KHZ      bmbit0

SFRX(XAUTODAT1,       0xE67B);
SFRX(XAUTODAT2,       0xE67C);
#define EXTAUTODAT1   XAUTODAT1
#define EXTAUTODAT2   XAUTODAT2

/* USB Control */
SFRX(USBCS,           0xE680);
#define SIGRSUME      bmbit0
#define RENUM         bmbit1
#define NOSYNSOF      bmbit2
#define DISCON        bmbit3
#define HSM           bmbit7

SFRX(SUSPEND,         0xE681);
SFRX(WAKEUPCS,        0xE682);
#define BMWU2         bmbit7
#define BMWU          bmbit6
#define BMWU2POL      bmbit5
#define BMWUPOL       bmbit4
#define BMDPEN        bmbit2
#define BMWU2EN       bmbit1
#define BMWUEN        bmbit0

SFRX(TOGCTL,           0xE683);
#define BMTOGCTLEPMASK bmbit3 | bmbit2 | bmbit1 | bmbit0
#define BMRESETTOGGLE  bmbit5
#define BMSETTOGGLE    bmbit6
#define BMQUERYTOGGLE  bmbit7

SFRX(USBFRAMEH,        0xE684);
SFRX(USBFRAMEL,        0xE685);
SFRX(MICROFRAME,       0xE686);
SFRX(FNADDR,           0xE687);

/* Endpoints */
SFRX(EP0BCH,           0xE68A);
SFRX(EP0BCL,           0xE68B);
SFRX(EP1OUTBC,         0xE68D);
SFRX(EP1INBC,          0xE68F);
SFRX(EP2BCH,           0xE690);
SFRX(EP2BCL,           0xE691);
SFRX(EP4BCH,           0xE694);
SFRX(EP4BCL,           0xE695);
SFRX(EP6BCH,           0xE698);
SFRX(EP6BCL,           0xE699);
SFRX(EP8BCH,           0xE69C);
SFRX(EP8BCL,           0xE69D);
SFRX(EP0CS,            0xE6A0);
#define HSNAK          bmbit7

SFRX(EP1INCS,          0xE6A2);
SFRX(EP1OUTCS,         0xE6A1);
#define EPSTALL        bmbit0
#define EPBSY          bmbit1

SFRX(EP2CS,            0xE6A3);
SFRX(EP4CS,            0xE6A4);
SFRX(EP6CS,            0xE6A5);
SFRX(EP8CS,            0xE6A6);
#define BMEPEMPTY      bmbit2
#define BMEPFULL       bmbit3
#define BMNPAK        (bmbit6 | bmbit5 | bmbit4)

SFRX(EP2FIFOFLGS,      0xE6A7);
SFRX(EP4FIFOFLGS,      0xE6A8);
SFRX(EP6FIFOFLGS,      0xE6A9);
SFRX(EP8FIFOFLGS,      0xE6AA);
SFRX(EP2FIFOBCH,       0xE6AB);
SFRX(EP2FIFOBCL,       0xE6AC);
SFRX(EP4FIFOBCH,       0xE6AD);
SFRX(EP4FIFOBCL,       0xE6AE);
SFRX(EP6FIFOBCH,       0xE6AF);
SFRX(EP6FIFOBCL,       0xE6B0);
SFRX(EP8FIFOBCH,       0xE6B1);
SFRX(EP8FIFOBCL,       0xE6B2);
SFRX(SUDPTRH,          0xE6B3);
SFRX(SUDPTRL,          0xE6B4);

SFRX(SUDPTRCTL,        0xE6B5);
#define BMSDPAUTO      bmbit0

SFRX(SETUPDAT[8],      0xE6B8);

/* GPIF */
SFRX(GPIFWFSELECT,     0xE6C0);
SFRX(GPIFIDLECS,       0xE6C1);
SFRX(GPIFIDLECTL,      0xE6C2);
SFRX(GPIFCTLCFG,       0xE6C3);
SFRX(GPIFADRH,         0xE6C4);
SFRX(GPIFADRL,         0xE6C5);
SFRX(GPIFTCB3,         0xE6CE);
SFRX(GPIFTCB2,         0xE6CF);
SFRX(GPIFTCB1,         0xE6D0);
SFRX(GPIFTCB0,         0xE6D1);

#define EP2GPIFTCH     GPIFTCB1   /* these are here for backwards compatibility */
#define EP2GPIFTCL     GPIFTCB0
#define EP4GPIFTCH     GPIFTCB1   /* these are here for backwards compatibility */
#define EP4GPIFTCL     GPIFTCB0
#define EP6GPIFTCH     GPIFTCB1   /* these are here for backwards compatibility */
#define EP6GPIFTCL     GPIFTCB0
#define EP8GPIFTCH     GPIFTCB1   /* these are here for backwards compatibility */
#define EP8GPIFTCL     GPIFTCB0

SFRX(EP2GPIFFLGSEL,    0xE6D2);
SFRX(EP2GPIFPFSTOP,    0xE6D3);
SFRX(EP2GPIFTRIG,      0xE6D4);
SFRX(EP4GPIFFLGSEL,    0xE6DA);
SFRX(EP4GPIFPFSTOP,    0xE6DB);
SFRX(EP4GPIFTRIG,      0xE6DC);
SFRX(EP6GPIFFLGSEL,    0xE6E2);
SFRX(EP6GPIFPFSTOP,    0xE6E3);
SFRX(EP6GPIFTRIG,      0xE6E4);
SFRX(EP8GPIFFLGSEL,    0xE6EA);
SFRX(EP8GPIFPFSTOP,    0xE6EB);
SFRX(EP8GPIFTRIG,      0xE6EC);
SFRX(XGPIFSGLDATH,     0xE6F0);
SFRX(XGPIFSGLDATLX,    0xE6F1);
SFRX(XGPIFSGLDATLNOX,  0xE6F2);
SFRX(GPIFREADYCFG,     0xE6F3);
SFRX(GPIFREADYSTAT,    0xE6F4);
SFRX(GPIFABORT,        0xE6F5);

// UDMA
SFRX(FLOWSTATE,        0xE6C6);
SFRX(FLOWLOGIC,        0xE6C7);
SFRX(FLOWEQ0CTL,       0xE6C8);
SFRX(FLOWEQ1CTL,       0xE6C9);
SFRX(FLOWHOLDOFF,      0xE6CA);
SFRX(FLOWSTB,          0xE6CB);
SFRX(FLOWSTBEDGE,      0xE6CC);
SFRX(FLOWSTBHPERIOD,   0xE6CD);
SFRX(GPIFHOLDAMOUNT,   0xE60C);
SFRX(UDMACRCH,         0xE67D);
SFRX(UDMACRCL,         0xE67E);
SFRX(UDMACRCQUAL,      0xE67F);

/* Debug/Test
 * The following registers are for Cypress's internal testing purposes only.
 * These registers are not documented in the datasheet or the Technical Reference
 * Manual as they were not designed for end user application usage
 */
SFRX(DBUG,             0xE6F8);
SFRX(TESTCFG,          0xE6F9);
SFRX(USBTEST,          0xE6FA);
SFRX(CT1,              0xE6FB);
SFRX(CT2,              0xE6FC);
SFRX(CT3,              0xE6FD);
SFRX(CT4,              0xE6FE);

/* Endpoint Buffers */
SFRX(EP0BUF[64],        0xE740);
SFRX(EP1INBUF[64],      0xE7C0);
SFRX(EP1OUTBUF[64],     0xE780);
SFRX(EP2FIFOBUF[512],   0xF000);
SFRX(EP4FIFOBUF[512],   0xF400);
SFRX(EP6FIFOBUF[512],   0xF800);
SFRX(EP8FIFOBUF[512],   0xFC00);

/* Error Correction Code (ECC) Registers (FX2LP/FX1 only) */
SFRX(ECCCFG,            0xE628);
SFRX(ECCRESET,          0xE629);
SFRX(ECC1B0,            0xE62A);
SFRX(ECC1B1,            0xE62B);
SFRX(ECC1B2,            0xE62C);
SFRX(ECC2B0,            0xE62D);
SFRX(ECC2B1,            0xE62E);
SFRX(ECC2B2,            0xE62F);

/* Feature Registers  (FX2LP/FX1 only) */
SFRX(GPCR2,         0xE50D);
#define BMFULLSPEEDONLY    bmbit4

#endif

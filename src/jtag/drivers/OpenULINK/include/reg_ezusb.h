/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef REG_EZUSB_H
#define REG_EZUSB_H

/**
 * @file
 * All information in this file was taken from the EZ-USB Technical
 * Reference Manual, Cypress Semiconductor, 3901 North First Street
 * San Jose, CA 95134 (www.cypress.com).
 *
 * The EZ-USB Technical Reference Manual is called "EZ-USB TRM" hereafter.
 *
 * The following bit name  definitions differ from those in the EZ-USB TRM:
 * - All lowercase characters in the EZ-USB TRM bit names have been converted
 *   to capitals (e. g. "WakeSRC" converted to "WAKESRC").
 * - CPUCS:  8051RES is named "RES8051".
 * - ISOCTL: Two MBZ ("Must Be Zero") bits are named "MBZ0" and "MBZ1".
 * - I2CS: STOP and START bits are preceded by "I2C_"
 * - INxCS, OUTxCS: the busy and stall bits are named "EPBSY" and "EPSTALL".
 * - TOGCTL: EZ-USB TRM bit names are preceded by "TOG_".
 */

/* Compiler-specific definitions of SBIT, SFR, SFRX, ... macros */
#include <mcs51/compiler.h>

/* Bit vectors */
#define bmBit0      0x01
#define bmBit1      0x02
#define bmBit2      0x04
#define bmBit3      0x08
#define bmBit4      0x10
#define bmBit5      0x20
#define bmBit6      0x40
#define bmBit7      0x80

/**************************************************************************
 ************************ Special Function Registers **********************
 ***************************************************************************/

/* See EZ-USB TRM, pp. A-9 - A-10 */

SFR(SP,             0x81);
SFR(DPL0,           0x82);
SFR(DPH0,           0x83);
SFR(DPL1,           0x84);
SFR(DPL2,           0x85);

SFR(DPS,            0x86);
#define SEL         bmBit0
/* Bit 1 read-only, always reads '0' */
/* Bit 2 read-only, always reads '0' */
/* Bit 3 read-only, always reads '0' */
/* Bit 4 read-only, always reads '0' */
/* Bit 5 read-only, always reads '0' */
/* Bit 6 read-only, always reads '0' */
/* Bit 7 read-only, always reads '0' */

SFR(PCON,           0x87);
#define IDLE        bmBit0
#define STOP        bmBit1
#define GF0         bmBit2
#define GF1         bmBit3
/* Bit 4 read-only, always reads '1' */
/* Bit 5 read-only, always reads '1' */
/* Bit 6 unused */
#define SMOD0     bmBit7

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
/* Some bits in this register share the same name in the EZ-USB TRM. Therefore,
 * we add a '0'/'1' to distinguish them */
#define M00         bmBit0
#define M01         bmBit1
#define CT0         bmBit2
#define GATE0       bmBit3
#define M10         bmBit4
#define M11         bmBit5
#define CT1         bmBit6
#define GATE1       bmBit7

SFR(TL0,            0x8A);
SFR(TL1,            0x8B);
SFR(TH0,            0x8C);
SFR(TH1,            0x8D);

SFR(CKCON,          0x8E);
#define MD0         bmBit0
#define MD1         bmBit1
#define MD2         bmBit2
#define T0M         bmBit3
#define T1M         bmBit4
#define T2M         bmBit5
/* Bit 6 unused */
/* Bit 7 unused */

SFR(SPC_FNC,        0x8D);
#define bmWRS       bmBit0
/* Bit 1 read-only, always reads '0' */
/* Bit 2 read-only, always reads '0' */
/* Bit 3 read-only, always reads '0' */
/* Bit 4 read-only, always reads '0' */
/* Bit 5 read-only, always reads '0' */
/* Bit 6 read-only, always reads '0' */
/* Bit 7 read-only, always reads '0' */

SFR(EXIF,           0x91);
/* Bit 0 read-only, always reads '0' */
/* Bit 1 read-only, always reads '0' */
/* Bit 2 read-only, always reads '0' */
/* Bit 3 read-only, always reads '1' */
#define USBINT      bmBit4
#define I2CINT      bmBit5
#define IE4         bmBit6
#define IE5         bmBit7

/* Definition of the _XPAGE register, according to SDCC Compiler User Guide,
 * Version 3.0.1, Chapter 4, p. 61. Also see EZ-USB TRM, p. 2-4. */
SFR(MPAGE,          0x92);
SFR(_XPAGE,         0x92);

SFR(SCON0,          0x98);
SBIT(RI_0,          0x98, 0);
SBIT(TI_0,          0x98, 1);
SBIT(RB8_0,         0x98, 2);
SBIT(TB8_0,         0x98, 3);
SBIT(REN_0,         0x98, 4);
SBIT(SM2_0,         0x98, 5);
SBIT(SM1_0,         0x98, 6);
SBIT(SM0_0,         0x98, 7);

SFR(SBUF0,          0x99);

SFR(IE,             0xA8);
SBIT(EX0,           0xA8, 0);
SBIT(ET0,           0xA8, 1);
SBIT(EX1,           0xA8, 2);
SBIT(ET1,           0xA8, 3);
SBIT(ES0,           0xA8, 4);
SBIT(ET2,           0xA8, 5);
SBIT(ES1,           0xA8, 6);
SBIT(EA,            0xA8, 7);

SFR(IP,             0xB8);
SBIT(PX0,           0xB8, 0);
SBIT(PT0,           0xB8, 1);
SBIT(PX1,           0xB8, 2);
SBIT(PT1,           0xB8, 3);
SBIT(PS0,           0xB8, 4);
SBIT(PT2,           0xB8, 5);
SBIT(PS1,           0xB8, 6);
/* Bit 7 read-only, always reads '1' */

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
SBIT(CT2,           0xC8, 1);
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

/************************ Endpoint 0-7 Data Buffers ************************/
SFRX(OUT7BUF[64],   0x7B40);
SFRX(IN7BUF[64],    0x7B80);
SFRX(OUT6BUF[64],   0x7BC0);
SFRX(IN6BUF[64],    0x7C00);
SFRX(OUT5BUF[64],   0x7C40);
SFRX(IN5BUF[64],    0x7C80);
SFRX(OUT4BUF[64],   0x7CC0);
SFRX(IN4BUF[64],    0x7D00);
SFRX(OUT3BUF[64],   0x7D40);
SFRX(IN3BUF[64],    0x7D80);
SFRX(OUT2BUF[64],   0x7DC0);
SFRX(IN2BUF[64],    0x7E00);
SFRX(OUT1BUF[64],   0x7E40);
SFRX(IN1BUF[64],    0x7E80);
SFRX(OUT0BUF[64],   0x7EC0);
SFRX(IN0BUF[64],    0x7F00);
/* 0x7F40 - 0x7F5F reserved */

/**************************** Isochronous Data *****************************/
SFRX(OUT8DATA,      0x7F60);
SFRX(OUT9DATA,      0x7F61);
SFRX(OUT10DATA,     0x7F62);
SFRX(OUT11DATA,     0x7F63);
SFRX(OUT12DATA,     0x7F64);
SFRX(OUT13DATA,     0x7F65);
SFRX(OUT14DATA,     0x7F66);
SFRX(OUT15DATA,     0x7F67);

SFRX(IN8DATA,       0x7F68);
SFRX(IN9DATA,       0x7F69);
SFRX(IN10DATA,      0x7F6A);
SFRX(IN11DATA,      0x7F6B);
SFRX(IN12DATA,      0x7F6C);
SFRX(IN13DATA,      0x7F6D);
SFRX(IN14DATA,      0x7F6E);
SFRX(IN15DATA,      0x7F6F);

/************************* Isochronous Byte Counts *************************/
SFRX(OUT8BCH,       0x7F70);
SFRX(OUT8BCL,       0x7F71);
SFRX(OUT9BCH,       0x7F72);
SFRX(OUT9BCL,       0x7F73);
SFRX(OUT10BCH,      0x7F74);
SFRX(OUT10BCL,      0x7F75);
SFRX(OUT11BCH,      0x7F76);
SFRX(OUT11BCL,      0x7F77);
SFRX(OUT12BCH,      0x7F78);
SFRX(OUT12BCL,      0x7F79);
SFRX(OUT13BCH,      0x7F7A);
SFRX(OUT13BCL,      0x7F7B);
SFRX(OUT14BCH,      0x7F7C);
SFRX(OUT14BCL,      0x7F7D);
SFRX(OUT15BCH,      0x7F7E);
SFRX(OUT16BCL,      0x7F7F);

/****************************** CPU Registers ******************************/
SFRX(CPUCS,         0x7F92);
#define RES8051     bmBit0
#define CLK24OE     bmBit1
/* Bit 2 read-only, always reads '0' */
/* Bit 3 read-only, always reads '0' */
/* Bits 4...7: Chip Revision */

SFRX(PORTACFG,      0x7F93);
#define T0OUT       bmBit0
#define T1OUT       bmBit1
#define OE          bmBit2
#define CS          bmBit3
#define FWR         bmBit4
#define FRD         bmBit5
#define RXD0OUT     bmBit6
#define RXD1OUT     bmBit7

SFRX(PORTBCFG,      0x7F94);
#define T2          bmBit0
#define T2EX        bmBit1
#define RXD1        bmBit2
#define TXD1        bmBit3
#define INT4        bmBit4
#define INT5        bmBit5
#define INT6        bmBit6
#define T2OUT       bmBit7

SFRX(PORTCCFG,      0x7F95);
#define RXD0        bmBit0
#define TXD0        bmBit1
#define INT0        bmBit2
#define INT1        bmBit3
#define T0          bmBit4
#define T1          bmBit5
#define WR          bmBit6
#define RD          bmBit7

/*********************** Input-Output Port Registers ***********************/
SFRX(OUTA,          0x7F96);
#define OUTA0       bmBit0
#define OUTA1       bmBit1
#define OUTA2       bmBit2
#define OUTA3       bmBit3
#define OUTA4       bmBit4
#define OUTA5       bmBit5
#define OUTA6       bmBit6
#define OUTA7       bmBit7

SFRX(OUTB,          0x7F97);
#define OUTB0       bmBit0
#define OUTB1       bmBit1
#define OUTB2       bmBit2
#define OUTB3       bmBit3
#define OUTB4       bmBit4
#define OUTB5       bmBit5
#define OUTB6       bmBit6
#define OUTB7       bmBit7

SFRX(OUTC,          0x7F98);
#define OUTC0       bmBit0
#define OUTC1       bmBit1
#define OUTC2       bmBit2
#define OUTC3       bmBit3
#define OUTC4       bmBit4
#define OUTC5       bmBit5
#define OUTC6       bmBit6
#define OUTC7       bmBit7

SFRX(PINSA,         0x7F99);
#define PINA0       bmBit0
#define PINA1       bmBit1
#define PINA2       bmBit2
#define PINA3       bmBit3
#define PINA4       bmBit4
#define PINA5       bmBit5
#define PINA6       bmBit6
#define PINA7       bmBit7

SFRX(PINSB,         0x7F9A);
#define PINB0       bmBit0
#define PINB1       bmBit1
#define PINB2       bmBit2
#define PINB3       bmBit3
#define PINB4       bmBit4
#define PINB5       bmBit5
#define PINB6       bmBit6
#define PINB7       bmBit7

SFRX(PINSC,         0x7F9B);
#define PINC0       bmBit0
#define PINC1       bmBit1
#define PINC2       bmBit2
#define PINC3       bmBit3
#define PINC4       bmBit4
#define PINC5       bmBit5
#define PINC6      bmBit6
#define PINC7       bmBit7

SFRX(OEA,           0x7F9C);
#define OEA0        bmBit0
#define OEA1        bmBit1
#define OEA2        bmBit2
#define OEA3        bmBit3
#define OEA4        bmBit4
#define OEA5        bmBit5
#define OEA6        bmBit6
#define OEA7        bmBit7

SFRX(OEB,           0x7F9D);
#define OEB0        bmBit0
#define OEB1        bmBit1
#define OEB2        bmBit2
#define OEB3        bmBit3
#define OEB4        bmBit4
#define OEB5        bmBit5
#define OEB6        bmBit6
#define OEB7        bmBit7

SFRX(OEC,           0x7F9E);
#define OEC0        bmBit0
#define OEC1        bmBit1
#define OEC2        bmBit2
#define OEC3        bmBit3
#define OEC4        bmBit4
#define OEC5        bmBit5
#define OEC6        bmBit6
#define OEC7        bmBit7

/* 0x7F9F reserved */

/****************** Isochronous Control/Status Registers *******************/
SFRX(ISOERR,        0x7FA0);
#define ISO8ERR     bmBit0
#define ISO9ERR     bmBit1
#define ISO10ERR    bmBit2
#define ISO11ERR    bmBit3
#define ISO12ERR    bmBit4
#define ISO13ERR    bmBit5
#define ISO14ERR    bmBit6
#define ISO15ERR    bmBit7

SFRX(ISOCTL,        0x7FA1);
#define ISODISAB    bmBit0
#define MBZ0        bmBit1
#define MBZ1        bmBit2
#define PPSTAT      bmBit3
/* Bit 4 unused */
/* Bit 5 unused */
/* Bit 6 unused */
/* Bit 7 unused */

SFRX(ZBCOUT,        0x7FA2);
#define EP8         bmBit0
#define EP9         bmBit1
#define EP10        bmBit2
#define EP11        bmBit3
#define EP12        bmBit4
#define EP13        bmBit5
#define EP14        bmBit6
#define EP15        bmBit7

/* 0x7FA3 reserved */
/* 0x7FA4 reserved */

/****************************** I2C Registers ******************************/
SFRX(I2CS,          0x7FA5);
#define DONE        bmBit0
#define ACK         bmBit1
#define BERR        bmBit2
#define ID0         bmBit3
#define ID1         bmBit4
#define LASTRD      bmBit5
#define I2C_STOP    bmBit6
#define I2C_START   bmBit7

SFRX(I2DAT,         0x7FA6);
/* 0x7FA7 reserved */

/******************************* Interrupts ********************************/
SFRX(IVEC,          0x7FA8);
/* Bit 0 read-only, always reads '0' */
/* Bit 1 read-only, always reads '0' */
#define IV0         bmBit2
#define IV1         bmBit3
#define IV2         bmBit4
#define IV3         bmBit5
#define IV4         bmBit6
/* Bit 7 read-only, always reads '0' */

SFRX(IN07IRQ,       0x7FA9);
#define IN0IR       bmBit0
#define IN1IR       bmBit1
#define IN2IR       bmBit2
#define IN3IR       bmBit3
#define IN4IR       bmBit4
#define IN5IR       bmBit5
#define IN6IR       bmBit6
#define IN7IR       bmBit7

SFRX(OUT07IRQ,      0x7FAA);
#define OUT0IR      bmBit0
#define OUT1IR      bmBit1
#define OUT2IR      bmBit2
#define OUT3IR      bmBit3
#define OUT4IR      bmBit4
#define OUT5IR      bmBit5
#define OUT6IR      bmBit6
#define OUT7IR      bmBit7

SFRX(USBIRQ,        0x7FAB);
#define SUDAVIR     bmBit0
#define SOFIR       bmBit1
#define SUTOKIR     bmBit2
#define SUSPIR      bmBit3
#define URESIR      bmBit4
/* Bit 5 unused */
/* Bit 6 unused */
/* Bit 7 unused */

SFRX(IN07IEN,       0x7FAC);
#define IN0IEN      bmBit0
#define IN1IEN      bmBit1
#define IN2IEN      bmBit2
#define IN3IEN      bmBit3
#define IN4IEN      bmBit4
#define IN5IEN      bmBit5
#define IN6IEN      bmBit6
#define IN7IEN      bmBit7

SFRX(OUT07IEN,      0x7FAD);
#define OUT0IEN     bmBit0
#define OUT1IEN     bmBit1
#define OUT2IEN     bmBit2
#define OUT3IEN     bmBit3
#define OUT4IEN     bmBit4
#define OUT5IEN     bmBit5
#define OUT6IEN     bmBit6
#define OUT7IEN     bmBit7

SFRX(USBIEN,        0x7FAE);
#define SUDAVIE     bmBit0
#define SOFIE       bmBit1
#define SUTOKIE     bmBit2
#define SUSPIE      bmBit3
#define URESIE      bmBit4
/* Bit 5 unused */
/* Bit 6 unused */
/* Bit 7 unused */

SFRX(USBBAV,        0x7FAF);
#define AVEN        bmBit0
#define BPEN        bmBit1
#define BPPULSE     bmBit2
#define BREAK       bmBit3
/* Bit 4 unused */
/* Bit 5 unused */
/* Bit 6 unused */
/* Bit 7 unused */

/* 0x7FB0 reserved */
/* 0x7FB1 reserved */
SFRX(BPADDRH,       0x7FB2);
SFRX(BPADDRL,       0x7FB3);

/****************************** Endpoints 0-7 ******************************/
SFRX(EP0CS,         0x7FB4);
#define EP0STALL    bmBit0
#define HSNAK       bmBit1
#define IN0BSY      bmBit2
#define OUT0BSY     bmBit3
/* Bit 4 unused */
/* Bit 5 unused */
/* Bit 6 unused */
/* Bit 7 unused */

SFRX(IN0BC,         0x7FB5);
SFRX(IN1CS,         0x7FB6);
SFRX(IN1BC,         0x7FB7);
SFRX(IN2CS,         0x7FB8);
SFRX(IN2BC,         0x7FB9);
SFRX(IN3CS,         0x7FBA);
SFRX(IN3BC,         0x7FBB);
SFRX(IN4CS,         0x7FBC);
SFRX(IN4BC,         0x7FBD);
SFRX(IN5CS,         0x7FBE);
SFRX(IN5BC,         0x7FBF);
SFRX(IN6CS,         0x7FC0);
SFRX(IN6BC,         0x7FC1);
SFRX(IN7CS,         0x7FC2);
SFRX(IN7BC,         0x7FC3);
/* 0x7FC4 reserved */
SFRX(OUT0BC,        0x7FC5);
SFRX(OUT1CS,        0x7FC6);
SFRX(OUT1BC,        0x7FC7);
SFRX(OUT2CS,        0x7FC8);
SFRX(OUT2BC,        0x7FC9);
SFRX(OUT3CS,        0x7FCA);
SFRX(OUT3BC,        0x7FCB);
SFRX(OUT4CS,        0x7FCC);
SFRX(OUT4BC,        0x7FCD);
SFRX(OUT5CS,        0x7FCE);
SFRX(OUT5BC,        0x7FCF);
SFRX(OUT6CS,        0x7FD0);
SFRX(OUT6BC,        0x7FD1);
SFRX(OUT7CS,        0x7FD2);
SFRX(OUT7BC,        0x7FD3);

/* The INxSTALL, OUTxSTALL, INxBSY and OUTxBSY bits are the same for all
 * INxCS/OUTxCS registers. For better readability, we define them only once */
#define EPSTALL     bmBit0
#define EPBSY       bmBit1

/************************** Global USB Registers ***************************/
SFRX(SUDPTRH,       0x7FD4);
SFRX(SUDPTRL,       0x7FD5);

SFRX(USBCS,         0x7FD6);
#define SIGRSUME    bmBit0
#define RENUM       bmBit1
#define DISCOE      bmBit2
#define DISCON      bmBit3
/* Bit 4 unused */
/* Bit 5 unused */
/* Bit 6 unused */
#define WAKESRC     bmBit7

SFRX(TOGCTL,        0x7FD7);
#define TOG_EP0     bmBit0
#define TOG_EP1     bmBit1
#define TOG_EP2     bmBit2
/* Bit 3 is read-only, always reads '0' */
#define TOG_IO      bmBit4
#define TOG_R       bmBit5
#define TOG_S       bmBit6
#define TOG_Q       bmBit7

SFRX(USBFRAMEL,     0x7FD8);
SFRX(USBFRAMEH,     0x7FD9);
/* 0x7FDA reserved */
SFRX(FNADDR,        0x7FDB);
/* 0x7FDC reserved */

SFRX(USBPAIR,       0x7FDD);
#define PR2IN       bmBit0
#define PR4IN       bmBit1
#define PR6IN       bmBit2
#define PR2OUT      bmBit3
#define PR4OUT      bmBit4
#define PR6OUT      bmBit5
/* Bit 6 unused */
#define ISOSEND0    bmBit7

SFRX(IN07VAL,       0x7FDE);
/* Bit 0 is read-only, always reads '1' */
#define IN1VAL      bmBit1
#define IN2VAL      bmBit2
#define IN3VAL      bmBit3
#define IN4VAL      bmBit4
#define IN5VAL      bmBit5
#define IN6VAL      bmBit6
#define IN7VAL      bmBit7

SFRX(OUT07VAL,      0x7FDF);
/* Bit 0 is read-only, always reads '1' */
#define OUT1VAL     bmBit1
#define OUT2VAL     bmBit2
#define OUT3VAL     bmBit3
#define OUT4VAL     bmBit4
#define OUT5VAL     bmBit5
#define OUT6VAL     bmBit6
#define OUT7VAL     bmBit7

SFRX(INISOVAL,      0x7FE0);
#define IN8VAL      bmBit0
#define IN9VAL      bmBit1
#define IN10VAL     bmBit2
#define IN11VAL     bmBit3
#define IN12VAL     bmBit4
#define IN13VAL     bmBit5
#define IN14VAL     bmBit6
#define IN15VAL     bmBit7

SFRX(OUTISOVAL,     0x7FE1);
#define OUT8VAL     bmBit0
#define OUT9VAL     bmBit1
#define OUT10VAL    bmBit2
#define OUT11VAL    bmBit3
#define OUT12VAL    bmBit4
#define OUT13VAL    bmBit5
#define OUT14VAL    bmBit6
#define OUT15VAL    bmBit7

SFRX(FASTXFR,       0x7FE2);
#define WMOD0       bmBit0
#define WMOD1       bmBit1
#define WPOL        bmBit2
#define RMOD0       bmBit3
#define RMOD1       bmBit4
#define RPOL        bmBit5
#define FBLK        bmBit6
#define FISO        bmBit7

SFRX(AUTOPTRH,      0x7FE3);
SFRX(AUTOPTRL,      0x7FE4);
SFRX(AUTODATA,      0x7FE5);
/* 0x7FE6 reserved */
/* 0x7FE7 reserved */

/******************************* Setup Data ********************************/
SFRX(SETUPDAT[8],   0x7FE8);

/************************* Isochronous FIFO sizes **************************/
SFRX(OUT8ADDR,      0x7FF0);
SFRX(OUT9ADDR,      0x7FF1);
SFRX(OUT10ADDR,     0x7FF2);
SFRX(OUT11ADDR,     0x7FF3);
SFRX(OUT12ADDR,     0x7FF4);
SFRX(OUT13ADDR,     0x7FF5);
SFRX(OUT14ADDR,     0x7FF6);
SFRX(OUT15ADDR,     0x7FF7);

SFRX(IN8ADDR,       0x7FF8);
SFRX(IN9ADDR,       0x7FF9);
SFRX(IN10ADDR,      0x7FFA);
SFRX(IN11ADDR,      0x7FFB);
SFRX(IN12ADDR,      0x7FFC);
SFRX(IN13ADDR,      0x7FFD);
SFRX(IN14ADDR,      0x7FFE);
SFRX(IN15ADDR,      0x7FFF);

#endif

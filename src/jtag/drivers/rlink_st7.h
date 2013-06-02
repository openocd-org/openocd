/***************************************************************************
 *   Copyright (C) 2008 Lou Deluxe                                         *
 *   lou.openocd012@fixit.nospammail.net                                   *
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

#define ST7_FOSC		(12 * 1000000)

/* This is not a complete enumeration of ST7 registers, but it is sufficient for this interface driver. */

#define ST7_PADR		(0x0000)
#define ST7_PADDR		(ST7_PADR + 1)
#define ST7_PAOR		(ST7_PADR + 2)
#define ST7_PBDR		(0x0003)
#define ST7_PBDDR		(ST7_PBDR + 1)
#define ST7_PCDR		(0x0006)
#define ST7_PCDDR		(ST7_PCDR + 1)
#define ST7_PCOR		(ST7_PCDR + 2)
#define ST7_PDDR		(0x0009)
#define ST7_PDDDR		(ST7_PDDR + 1)
#define ST7_PDOR		(ST7_PDDR + 2)
#define ST7_PEDR		(0x000c)
#define ST7_PEDDR		(ST7_PEDR + 1)
#define ST7_PEOR		(ST7_PEDR + 2)
#define ST7_PFDR		(0x000f)
#define ST7_PFDDR		(ST7_PFDR + 1)

#define ST7_ADCDR		(0x0012)
#define ST7_ADCCSR		(ST7_ADCDR + 1)

#define ST7_EP2TXR		(0x003e)
#define ST7_EP2TXR_STAT_TX0	(1 << 0)
#define ST7_EP2TXR_STAT_TX1	(1 << 1)
#define ST7_EP2TXR_STAT_DISABLED	(0)
#define ST7_EP2TXR_STAT_STALL	(ST7_EP2TXR_STAT_TX0)
#define ST7_EP2TXR_STAT_VALID	(ST7_EP2TXR_STAT_TX1 | ST7_EP2TXR_STAT_TX0)
#define ST7_EP2TXR_STAT_NAK	(ST7_EP2TXR_STAT_TX1)
#define ST7_EP2TXR_DTOG_TX	(1 << 2)
#define ST7_EP2TXR_CTR_TX	(1 << 3)

#define ST7_USB_BUF_EP0OUT	(0x1550)
#define ST7_USB_BUF_EP0IN	(0x1560)
#define ST7_USB_BUF_EP1OUT	(0x1570)
#define ST7_USB_BUF_EP1IN	(0x1580)
#define ST7_USB_BUF_EP2UODI	(0x1590)
#define ST7_USB_BUF_EP2UIDO	(0x1650)

#define ST7_PA0			(1 << 0)
#define ST7_PA1			(1 << 1)
#define ST7_PA2			(1 << 2)
#define ST7_PA3			(1 << 3)
#define ST7_PA4			(1 << 4)
#define ST7_PA5			(1 << 5)
#define ST7_PA6			(1 << 6)
#define ST7_PA7			(1 << 7)

#define ST7_PB0			(1 << 0)
#define ST7_PB1			(1 << 1)
#define ST7_PB2			(1 << 2)
#define ST7_PB3			(1 << 3)
#define ST7_PB4			(1 << 4)
#define ST7_PB5			(1 << 5)
#define ST7_PB6			(1 << 6)
#define ST7_PB7			(1 << 7)

#define ST7_PC0			(1 << 0)
#define ST7_PC1			(1 << 1)
#define ST7_PC2			(1 << 2)
#define ST7_PC3			(1 << 3)
#define ST7_PC4			(1 << 4)
#define ST7_PC5			(1 << 5)
#define ST7_PC6			(1 << 6)
#define ST7_PC7			(1 << 7)

#define ST7_PD0			(1 << 0)
#define ST7_PD1			(1 << 1)
#define ST7_PD2			(1 << 2)
#define ST7_PD3			(1 << 3)
#define ST7_PD4			(1 << 4)
#define ST7_PD5			(1 << 5)
#define ST7_PD6			(1 << 6)
#define ST7_PD7			(1 << 7)

#define ST7_PE0			(1 << 0)
#define ST7_PE1			(1 << 1)
#define ST7_PE2			(1 << 2)
#define ST7_PE3			(1 << 3)
#define ST7_PE4			(1 << 4)
#define ST7_PE5			(1 << 5)
#define ST7_PE6			(1 << 6)
#define ST7_PE7			(1 << 7)

#define ST7_PF0			(1 << 0)
#define ST7_PF1			(1 << 1)
#define ST7_PF2			(1 << 2)
#define ST7_PF3			(1 << 3)
#define ST7_PF4			(1 << 4)
#define ST7_PF5			(1 << 5)
#define ST7_PF6			(1 << 6)
#define ST7_PF7			(1 << 7)

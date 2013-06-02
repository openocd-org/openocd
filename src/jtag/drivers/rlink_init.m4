m4_divert(`-1')
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

m4_undefine(`CTRL_MPEG_L')
m4_undefine(`CTRL_CARD_L')

m4_ifelse(SHIFTER_PRESCALER, 1, `
	m4_define(`CTRL_MPEG_L', `m4_eval(`0x8 | 0x0')')
')
m4_ifelse(SHIFTER_PRESCALER, 2, `
	m4_define(`CTRL_MPEG_L', `m4_eval(`0x8 | 0x2')')
	m4_define(`CTRL_CARD_L', `m4_eval(`0x8 | 0x1')')
')
m4_ifelse(SHIFTER_PRESCALER, 8, `
	m4_define(`CTRL_MPEG_L', `m4_eval(`0x8 | 0x3')')
')
m4_ifelse(SHIFTER_PRESCALER, 11, `
	m4_define(`CTRL_MPEG_L', `m4_eval(`0x8 | 0x4')')
')
m4_ifelse(SHIFTER_PRESCALER, 64, `
	m4_define(`CTRL_MPEG_L', `m4_eval(`0x8 | 0x7')')
')

m4_ifdef(`CTRL_MPEG_L',,`
	m4_errprint(`SHIFTER_PRESCALER was not defined with a supported value
')	m4_m4exit(`1')
')

m4_divert(`0')m4_dnl

init:
	A.H = 0

	A.L = 0

	DR_MPEG = A	; TDI and TCK start out low
	DR_CARD = A	; TMS starts out low

	A.L = 0x6

	CTRL_FCI = A	; MPEG and CARD driven by FCI
	DDR_MPEG = A	; TDI and TCK are outputs

	A.L = 0x1

	X = A		; X == 1
	DDR_CARD = A	; TMS is output

	A.L = CTRL_MPEG_L
	CTRL_MPEG = A
m4_ifdef(`CTRL_CARD_L',
`	A.L = 'CTRL_CARD_L`
')m4_dnl
	CTRL_CARD = A

	STATUS STOP

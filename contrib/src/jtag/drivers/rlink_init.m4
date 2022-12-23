# SPDX-License-Identifier: GPL-2.0-or-later

#
# Copyright (C) 2008 Lou Deluxe
# lou.openocd012@fixit.nospammail.net
#

m4_divert(`-1')

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

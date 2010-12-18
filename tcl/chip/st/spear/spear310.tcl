# Specific init scripts for ST SPEAr310 system on chip
# http://www.st.com/spear
#
# Date:      2010-09-23
# Author:    Antonio Borneo <borneo.antonio@gmail.com>


proc sp310_init {} {
	mww 0xfca80034 0x0000ffff	;# enable all RAS clocks
	mww 0xfca80040 0x00000000	;# remove all RAS resets
	mww 0xb4000008 0x00002ff4	;# RAS function enable

	mww 0xfca8013c 0x2f7bc210	;# plgpio_pad_drv
	mww 0xfca80140 0x017bdef6
}

proc sp310_emi_init {} {
	# set EMI pad strength
	mmw 0xfca80134 0x0e000000 0x00000000
	mmw 0xfca80138 0x0e739ce7 0x00000000
	mmw 0xfca8013c 0x00039ce7 0x00000000

	# set safe EMI timing as in BootROM
	#mww 0x4f000000 0x0000000f	;# tAP_0_reg
	#mww 0x4f000004 0x00000000	;# tSDP_0_reg
	#mww 0x4f000008 0x000000ff	;# tDPw_0_reg
	#mww 0x4f00000c 0x00000111	;# tDPr_0_reg
	#mww 0x4f000010 0x00000002	;# tDCS_0_reg

	# set fast EMI timing as in Linux
	mww 0x4f000000 0x00000010	;# tAP_0_reg
	mww 0x4f000004 0x00000005	;# tSDP_0_reg
	mww 0x4f000008 0x0000000a	;# tDPw_0_reg
	mww 0x4f00000c 0x0000000a	;# tDPr_0_reg
	mww 0x4f000010 0x00000005	;# tDCS_0_re

	# 32bit wide, 8/16/32bit access
	mww 0x4f000014 0x0000000e	;# control_0_reg
	mww 0x4f000094 0x0000003f	;# ack_reg
}

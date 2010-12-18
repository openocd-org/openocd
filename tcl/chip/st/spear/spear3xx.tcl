# Generic init scripts for all ST SPEAr3xx family
# http://www.st.com/spear
#
# Date:      2010-09-23
# Author:    Antonio Borneo <borneo.antonio@gmail.com>


# Initialize internal clock
# Default:
# - Crystal =  24 MHz
# - PLL1    = 332 MHz
# - PLL2    = 332 MHz
# - CPU_CLK = 332 MHz
# - DDR_CLK = 332 MHz async
# - HCLK    = 166 MHz
# - PCLK    =  83 MHz
proc sp3xx_clock_default {} {
	mww 0xfca00000 0x00000002	;# set sysclk slow
	mww 0xfca00014 0x0ffffff8	;# set pll timeout to minimum (100us ?!?)

	# DDRCORE disable to change frequency
	set val [expr ([mrw 0xfca8002c] & ~0x20000000) | 0x40000000]
	mww 0xfca8002c $val
	mww 0xfca8002c $val ;# Yes, write twice!

	# programming PLL1
	mww 0xfca8000c 0xa600010c	;# M=166 P=1 N=12
	mww 0xfca80008 0x00001c0a	;# power down
	mww 0xfca80008 0x00001c0e	;# enable
	mww 0xfca80008 0x00001c06	;# strobe
	mww 0xfca80008 0x00001c0e
	while { [expr [mrw 0xfca80008] & 0x01] == 0x00 } { sleep 1 }

	# programming PLL2
	mww 0xfca80018 0xa600010c	;# M=166, P=1, N=12
	mww 0xfca80014 0x00001c0a	;# power down
	mww 0xfca80014 0x00001c0e	;# enable
	mww 0xfca80014 0x00001c06	;# strobe
	mww 0xfca80014 0x00001c0e
	while { [expr [mrw 0xfca80014] & 0x01] == 0x00 } { sleep 1 }

	mww 0xfca80028 0x00000082	;# enable plltimeen
	mww 0xfca80024 0x00000511	;# set hclkdiv="/2" & pclkdiv="/2"

	mww 0xfca00000 0x00000004	;# setting SYSCTL to NORMAL mode
	while { [expr [mrw 0xfca00000] & 0x20] != 0x20 } { sleep 1 }

	# Select source of DDR clock
	#mmw 0xfca80020 0x10000000 0x70000000 ;# PLL1
	mmw 0xfca80020 0x30000000 0x70000000 ;# PLL2

	# DDRCORE enable after change frequency
	mmw 0xfca8002c 0x20000000 0x00000000
}

proc sp3xx_common_init {} {
	mww 0xfca8002c 0xfffffff8	;# enable clock of all peripherals
	mww 0xfca80038 0x00000000	;# remove reset of all peripherals

	mww 0xfca800e4 0x78000008	;# COMP1V8_REG
	mww 0xfca800ec 0x78000008	;# COMP3V3_REG

	mww 0xfca80050 0x00000001	;# Enable clk mem port 1

	mww 0xfc000000 0x10000f5f	;# init SMI and set HW mode
	mww 0xfc000000 0x00000f5f

	# Initialize Bus Interconnection Matrix
	# All ports Round-Robin and lowest priority
	mww 0xfca8007c 0x80000007
	mww 0xfca80080 0x80000007
	mww 0xfca80084 0x80000007
	mww 0xfca80088 0x80000007
	mww 0xfca8008c 0x80000007
	mww 0xfca80090 0x80000007
	mww 0xfca80094 0x80000007
	mww 0xfca80098 0x80000007
	mww 0xfca8009c 0x80000007
}

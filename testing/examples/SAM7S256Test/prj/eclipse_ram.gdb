target remote localhost:3333
monitor reset
monitor sleep 500
monitor poll
monitor soft_reset_halt
monitor arm7_9 sw_bkpts enable

# WDT_MR, disable watchdog 
monitor mww 0xFFFFFD44 0x00008000

# RSTC_MR, enable user reset
monitor mww 0xfffffd08 0xa5000001

# CKGR_MOR
monitor mww 0xFFFFFC20 0x00000601
monitor sleep 10

# CKGR_PLLR
monitor mww 0xFFFFFC2C 0x00481c0e
monitor sleep 10

# PMC_MCKR
monitor mww 0xFFFFFC30 0x00000007
monitor sleep 10

# PMC_IER
monitor mww 0xFFFFFF60 0x00480100
monitor sleep 100

load
break main
continue

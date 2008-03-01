target remote localhost:3333
monitor reset
monitor sleep 500
monitor poll
monitor soft_reset_halt
monitor arm7_9 sw_bkpts enable
monitor mww 0xFFE00000 0x1000213D
monitor mww 0xFFE00004 0x20003E3D
monitor mww 0xFFE00020 0x00000001
monitor mdw 0xFFE00000 1
monitor mdw 0xFFE00004 1
load
break main
continue

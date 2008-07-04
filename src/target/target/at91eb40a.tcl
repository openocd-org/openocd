#Script for AT91EB40a

#Atmel ties SRST & TRST together, at which point it makes
#no sense to use TRST, but use TMS instead.
#
#The annoying thing with tying SRST & TRST together is that
#there is no way to halt the CPU *before and during* the
#SRST reset, which means that the CPU will run a number
#of cycles before it can be halted(as much as milliseconds).
openocd {reset_config srst_only srst_pulls_trst}
 
#jtag scan chain
#format L IRC IRCM IDCODE (Length, IR Capture, IR Capture Mask, IDCODE)
openocd {jtag_device 4 0x1 0xf 0xe}

#target configuration
openocd {target arm7tdmi little 0 arm7tdmi-s_r4}

# speed up memory downloads
openocd {arm7 fast_memory_access enable}
openocd {arm7_9 dcc_downloads enable}

# OpenOCD does not have a flash driver for for AT91FR40162S 
openocd {target_script 0 reset event/at91eb40a_reset.script}

#flash driver
openocd {flash bank ecosflash 0x01000000 0x200000 2 2 0 ecos/at91eb40a.elf}

# required for usable performance. Used for lots of
# other things than flash programming.
openocd {working_area 0 0x00000000 0x20000 nobackup}

#force hardware values - we're running out of flash more
#often than not. The user can disable this in his
#subsequent config script.
openocd {arm7_9 force_hw_bkpts enable}

set reset_count 0

proc target_reset_0 {} {
	global reset_count
	# Reset script for AT91EB40a
	openocd {reg cpsr 0x000000D3} 		
	openocd {mww 0xFFE00020 0x1}
	openocd {mww 0xFFE00024 0x00000000}  
	openocd {mww 0xFFE00000 0x01002539} 
	openocd {mww 0xFFFFF124 0xFFFFFFFF}  
	openocd {mww 0xffff0010 0x100}
	openocd {mww 0xffff0034 0x100}
	set reset_count [expr $reset_count+1]
	puts "Testing reset $reset_count !"
}

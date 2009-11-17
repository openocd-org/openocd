# Defines basic Tcl procs for OpenOCD JTAG module

# Executed during "init". Can be overridden
# by board/target/... scripts
proc jtag_init {} {
	if {[catch {jtag arp_init} err]!=0} {
		# try resetting additionally
		init_reset startup
	}
}

#########

# TODO: power_restore and power_dropout are currently neither
# documented nor supported except on ZY1000.

proc power_restore {} {
	puts "Sensed power restore."
	reset init
}

add_help_text power_restore "Overridable procedure run when power restore is detected. Runs 'reset init' by default."

proc power_dropout {} {
	puts "Sensed power dropout."
}

#########

# TODO: srst_deasserted and srst_asserted are currently neither
# documented nor supported except on ZY1000.

proc srst_deasserted {} {
	puts "Sensed nSRST deasserted."
	reset init
}
add_help_text srst_deasserted "Overridable procedure run when srst deassert is detected. Runs 'reset init' by default."

proc srst_asserted {} {
	puts "Sensed nSRST asserted."
}

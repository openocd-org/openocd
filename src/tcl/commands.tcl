
# Production command
# FIX!!! need to figure out how to feed back relevant output
# from e.g. "flash banks" command...
proc board_produce {filename serialnumber} {
	openocd "reset init"
	openocd "flash write_image erase $filename [flash] bin"]]
	openocd "verify_image $filename [flash] bin"]]
	echo "Successfully ran production procedure"
}

proc board_test {} {
	echo "Production test not implemented"
}

# Show flash in human readable form
# This is an example of a human readable form of a low level fn
proc flash_banks_pretty {} { 
	set i 0 	
	set result ""
	foreach {a} [flash_banks] {
		if {$i > 0} {
			set result "$result\n"
		}
		set result [format "$result#%d: %s at 0x%08x, size 0x%08x, buswidth %d, chipwidth %d" $i [lindex $a 0] [lindex $a 1] [lindex $a 2] [lindex $a 3] [lindex $a 4]]
		set i [expr $i+1]	
	}	
	return $result
}

# We need to explicitly redirect this to the OpenOCD command
# as Tcl defines the exit proc
proc exit {} {
	openocd_throw exit
}

# If a fn is unknown to Tcl, we try to execute it as an OpenOCD command
proc unknown {args} {

	# This is uglier than it needs to be since the "flash banks" is really
	# a single command. For now only "flash banks" has been converted from
	# C to Tcl as an example, but if we do decide to go down this path, then
	# some more generic scheme will be put in place here.
	#
	# Help texts need a makeover. There needs to be help texts for
	# tcl procs + perhaps some work w.r.t. making the help command
	# format things prettier.
	if {[string compare [lindex $args 0] flash]==0 && [string compare [lindex $args 1] banks]==0} {
		return [flash_banks_pretty]
	}  

	# We print out as we run the command
	if {[string length $args]>0} {
		openocd_throw "$args"
	}
	# The primary return value have been set by "openocd" above,
	# so we need to clear it, lest we print out the output from
	# the command twice.
	return ""
}

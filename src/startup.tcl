#
# Defines basic Tcl procs that must be there for
# OpenOCD to work.
#
# Embedded into OpenOCD executable
#


# Help text list. A list of command + help text pairs.
#
# Commands can be more than one word and they are stored
# as "flash banks" "help text x x x"

global ocd_helptext
set ocd_helptext {}

proc add_help_text {cmd cmd_help} {
	global ocd_helptext
	lappend ocd_helptext [list $cmd $cmd_help]
}

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

# We have currently converted only "flash banks" to tcl.
proc flash args {
	if {[string compare [lindex $args 0] banks]==0} {
		return [flash_banks_pretty]
	}
	openocd_throw "flash $args"
}

#Print help text for a command. Word wrap
#help text that is too wide inside column.
proc help {args} {
	global ocd_helptext
	set cmd $args
	foreach a [lsort $ocd_helptext] {
		if {[string length $cmd]==0||[string first $cmd $a]!=-1||[string first $cmd [lindex $a 1]]!=-1} {
			set w 50
			set cmdname [lindex $a 0]
			set h [lindex $a 1]
			set n 0
			while 1 {
				if {$n > [string length $h]} {break}
				
				set next_a [expr $n+$w]
				if {[string length $h]>$n+$w} {
					set xxxx [string range $h $n [expr $n+$w]]
					for {set lastpos [expr [string length $xxxx]-1]} {$lastpos>=0&&[string compare [string range $xxxx $lastpos $lastpos] " "]!=0} {set lastpos [expr $lastpos-1]} {
					}
					#set next_a -1
					if {$lastpos!=-1} {
						set next_a [expr $lastpos+$n+1]
					}
				}
				
				
				puts [format "%-25s %s" $cmdname [string range $h $n [expr $next_a-1]] ]
				set cmdname ""
				set n [expr $next_a]
			}
		}
	}
}

add_help_text help "Tcl implementation of help command"


# If a fn is unknown to Tcl, we try to execute it as an OpenOCD command
proc unknown {args} {
	if {[string length $args]>0} {
		set cmd ""
		# We need to add back quotes for arguments w/space
		# for args without space, we can add quotes anyway
		foreach {a} $args {
			set cmd "$cmd \"$a\""
		}
		openocd_throw $cmd
	}
	# openocd_throw outputs while running and also sets the
	# primary return value to the output of the command
	#
	# The primary return value have been set by "openocd" above,
	# so we need to clear it, lest we print out the output from
	# the command twice.
	return ""
}

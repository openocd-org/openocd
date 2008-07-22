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

proc add_help_text {cmd cmd_help} {
	global ocd_helptext
	lappend ocd_helptext [list $cmd $cmd_help]
}

proc get_help_text {} {
	global ocd_helptext
	return $ocd_helptext
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
proc flash_banks {} { 
	set i 0 	
	set result ""
	foreach {a} [ocd_flash_banks] {
		if {$i > 0} {
			set result "$result\n"
		}
		set result [format "$result#%d: %s at 0x%08x, size 0x%08x, buswidth %d, chipwidth %d" $i $a(name) $a(base) $a(size) $a(bus_width) $a(chip_width)]
		set i [expr $i+1]	
	}	
	return $result
}

# We need to explicitly redirect this to the OpenOCD command
# as Tcl defines the exit proc
proc exit {} {
	ocd_throw exit
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
#
# We also support two level commands. "flash banks" is translated to
# flash_banks
proc unknown {args} {
	# do the name mangling from "flash banks" to "flash_banks"
	if {[llength $args]>=2} {
		set cmd_name "[lindex $args 0]_[lindex $args 1]"
		# Fix?? add a check here if this is a command?
		# we'll strip away args until we fail anyway...
		return [eval "$cmd_name [lrange $args 2 end]"]
	}
	# This really is an unknown command.
	return -code error "Unknown command: $args"
}


proc target_script {target_num eventname scriptname} {
	if {[string compare $eventname reset]==0} {
		set eventname post_reset
	}

	# This is the script we invoke
	proc "target_[set target_num]_[set eventname]" {} "script $scriptname" 
	
}

add_help_text target_script "<target#> <event=reset/pre_reset/post_halt/pre_resume/gdb_program_config> <script_file>"


# Try flipping / and \ to find file if the filename does not
# match the precise spelling
proc find {filename} {
	if {[catch {ocd_find $filename} t]==0} {
		return $t
	}  
	if {[catch {ocd_find [string map {\ /} $filename} t]==0} {
		return $t
	}  
	if {[catch {ocd_find [string map {/ \\} $filename} t]==0} {
		return $t
	}  
	# make sure error message matches original input string
	return -code error "Can't find $filename"
}
add_help_text find "<file> - print full path to file according to OpenOCD search rules"

# Run script
proc script {filename} {
	source [find $filename]
}

#proc daemon_reset {} {
#	puts "Daemon reset is obsolete. Use -c init -c \"reset halt\" at end of openocd command line instead");
#}

add_help_text script "<filename> - filename of OpenOCD script (tcl) to run"



# Handle GDB 'R' packet. Can be overriden by configuration script,
# but it's not something one would expect target scripts to do
# normally
proc ocd_gdb_restart {target_num} {
	# Fix!!! we're resetting all targets here! Really we should reset only
	# one target
	reset halt
}
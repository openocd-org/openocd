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

proc new_target_name { } {
	return [target number [expr [target count] - 1 ]]
}

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

# If RCLK is not supported, use fallback_speed_khz
proc jtag_rclk {fallback_speed_khz} {
	if {[catch {jtag_khz 0}]!=0} {
		jtag_khz $fallback_speed_khz
	}
}

add_help_text jtag_rclk "fallback_speed_khz - set JTAG speed to RCLK or use fallback speed"

proc ocd_process_reset { MODE } {

	# If this target must be halted...
	set halt -1
	if { 0 == [string compare $MODE halt] } {
		set halt 1
	}
	if { 0 == [string compare $MODE init] } {
		set halt 1;
	}
	if { 0 == [string compare $MODE run ] } {
		set halt 0;
	}
	if { $halt < 0 } {
		return -error "Invalid mode: $MODE, must be one of: halt, init, or run";
	}

	foreach t [ target names ] {
		# New event script.
		$t invoke-event reset-start
	}

	# Init the tap controller.
	jtag arp_init-reset

	# Examine all targets.
	foreach t [ target names ] {
		$t arp_examine
	}

	# Let the C code know we are asserting reset.
	foreach t [ target names ] {
		$t invoke-event reset-assert-pre
		# C code needs to know if we expect to 'halt'
		$t arp_reset assert $halt
		$t invoke-event reset-assert-post
	}

	# Now de-assert reset.
	foreach t [ target names ] {
		$t invoke-event reset-deassert-pre
		# Again, de-assert code needs to know..
		$t arp_reset deassert $halt
		$t invoke-event reset-deassert-post
	}

	# Pass 1 - Now try to halt.
	if { $halt } {
		foreach t [target names] {
	
			# Wait upto 1 second for target to halt.  Why 1sec? Cause
			# the JTAG tap reset signal might be hooked to a slow
			# resistor/capacitor circuit - and it might take a while
			# to charge
			
			# Catch, but ignore any errors.
			catch { $t arp_waitstate halted 1000 }
			
			# Did we succeed?
			set s [$t curstate]
			
			if { 0 != [string compare $s "halted" ] } {
				return -error [format "TARGET: %s - Not halted" $t]
			}
		}
	}

	#Pass 2 - if needed "init"
	if { 0 == [string compare init $MODE] } {
		foreach t [target names] {
			set err [catch "$t arp_waitstate halted 5000"]
			# Did it halt?
			if { $err == 0 } {
				$t invoke-event reset-init		
			}
		}
	}

	foreach t [ target names ] {
		$t invoke-event reset-end
	}
}

# stubs for targets scripts that do not have production procedure
proc production_info {} {
	return "Imagine an explanation here..."
}
add_help_text production_info "Displays information on production procedure for target script. Implement this procedure in target script."

proc production {firmwarefile serialnumber} {
	puts "Imagine production procedure running successfully. Programmed $firmwarefile with serial number $serialnumber"
}

add_help_text production "<serialnumber> - Runs production procedure. Throws exception if procedure failed. Prints progress messages. Implement this procedure in the target script."

proc production_test {} {
	puts "Imagine nifty test procedure having run to completion here."
}
add_help_text production "Runs test procedure. Throws exception if procedure failed. Prints progress messages. Implement in target script."

add_help_text cpu "<name> - prints out target options and a comment on CPU which matches name"

# A list of names of CPU and options required
set ocd_cpu_list {
	{
		name IXP42x 
		options {xscale -variant IXP42x} 
		comment {IXP42x cpu}
	}
	{
		name arm7 
		options {arm7tdmi -variant arm7tdmi} 
		comment {vanilla ARM7}
	}
}

# Invoked from Tcl code
proc ocd_cpu {args} {
	set name $args
	set result ""
	global ocd_cpu_list
	foreach a [lsort $ocd_cpu_list] {
		if {[string length $args]==0||[string first [string toupper $name] [string toupper "$a(name)$a(options)$a(comment)"]]!=-1} {
			lappend result $a 
		}
	}
	return $result
}

proc cpu {args} {
    #     0123456789012345678901234567890123456789012345678901234567890123456789
	puts "CPU                 Options                                 Comment"
	foreach a [lsort [ocd_cpu $args]] {
		puts [format "%-20s%-40s%s" $a(name) $a(options) $a(comment)]
	}
}

proc power_restore {} {
	puts "Sensed power restore."
	reset init
}

add_help_text power_restore "Overridable procedure run when power restore is detected. Runs 'reset init' by default."

proc power_dropout {} {
	puts "Sensed power dropout."
}

proc srst_deasserted {} {
	puts "Sensed nSRST deasserted."
	reset init
}
add_help_text srst_deasserted "Overridable procedure run when srst deassert is detected. Runs 'reset init' by default."

proc srst_asserted {} {
	puts "Sensed nSRST asserted."
}

# catch any exceptions, capture output and return output
proc capture_catch {a} {
	catch {
		capture {uplevel $a}
	} result
	return $result 
}

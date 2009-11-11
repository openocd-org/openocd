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
		if {[string length $cmd] == 0 || \
			[string first $cmd $a] != -1 || \
			[string first $cmd [lindex $a 1]] != -1} \
		{
			set w 50
			set cmdname [lindex $a 0]
			set h [lindex $a 1]
			set n 0
			while 1 {
				if {$n > [string length $h]} {break}

				set next_a [expr $n + $w]
				if {[string length $h] > $n + $w} \
				{
					set xxxx [string range $h $n [expr $n + $w]]
					for {set lastpos [expr [string length $xxxx] - 1]} \
						{$lastpos >= 0 && [string compare \
							[string range $xxxx $lastpos $lastpos] " "] != 0} \
						{set lastpos [expr $lastpos - 1]} \
					{
					}
					#set next_a -1
					if {$lastpos != -1} {
						set next_a [expr $lastpos + $n + 1]
					}
				}

				puts [format "%-25s %s" $cmdname \
						[string range $h $n [expr $next_a-1]] ]
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
		if {[catch {info body $cmd_name}]==0} {
		    # the command exists, try it...
			return [eval "$cmd_name [lrange $args 2 end]"]
		}
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

add_help_text script "<filename> - filename of OpenOCD script (tcl) to run"

# Handle GDB 'R' packet. Can be overriden by configuration script,
# but it's not something one would expect target scripts to do
# normally
proc ocd_gdb_restart {target_id} {
	# Fix!!! we're resetting all targets here! Really we should reset only
	# one target
	reset halt
}


# This reset logic may be overridden by board/target/... scripts as needed
# to provide a reset that, if possible, is close to a power-up reset.
#
# Exit requirements include:  (a) JTAG must be working, (b) the scan
# chain was validated with "jtag arp_init" (or equivalent), (c) nothing
# stays in reset.  No TAP-specific scans were performed.  It's OK if
# some targets haven't been reset yet; they may need TAP-specific scans.
#
# The "mode" values include:  halt, init, run (from "reset" command);
# startup (at OpenOCD server startup, when JTAG may not yet work); and
# potentially more (for reset types like cold, warm, etc)
proc init_reset { mode } {
	jtag arp_init-reset
}


global in_process_reset
set in_process_reset 0

# Catch reset recursion
proc ocd_process_reset { MODE } {
	global in_process_reset
	if {$in_process_reset} {
		set in_process_reset 0
		return -code error "'reset' can not be invoked recursively"
	}

	set in_process_reset 1
	set success [expr [catch {ocd_process_reset_inner $MODE} result]==0]
	set in_process_reset 0

	if {$success} {
		return $result
	} else {
		return -code error $result
	}
}

proc ocd_process_reset_inner { MODE } {
	set targets [target names]

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

	# Target event handlers *might* change which TAPs are enabled
	# or disabled, so we fire all of them.  But don't issue any
	# target "arp_*" commands, which may issue JTAG transactions,
	# unless we know the underlying TAP is active.
	#
	# NOTE:  ARP == "Advanced Reset Process" ... "advanced" is
	# relative to a previous restrictive scheme

	foreach t $targets {
		# New event script.
		$t invoke-event reset-start
	}

	# Use TRST or TMS/TCK operations to reset all the tap controllers.
	# TAP reset events get reported; they might enable some taps.
	init_reset $MODE

	# Examine all targets on enabled taps.
	foreach t $targets {
		if {[jtag tapisenabled [$t cget -chain-position]]} {
			$t arp_examine
		}
	}

	# Assert SRST, and report the pre/post events.
	# Note:  no target sees SRST before "pre" or after "post".
	foreach t $targets {
		$t invoke-event reset-assert-pre
	}
	foreach t $targets {
		# C code needs to know if we expect to 'halt'
		if {[jtag tapisenabled [$t cget -chain-position]]} {
			$t arp_reset assert $halt
		}
	}
	foreach t $targets {
		$t invoke-event reset-assert-post
	}

	# Now de-assert SRST, and report the pre/post events.
	# Note:  no target sees !SRST before "pre" or after "post".
	foreach t $targets {
		$t invoke-event reset-deassert-pre
	}
	foreach t $targets {
		# Again, de-assert code needs to know if we 'halt'
		if {[jtag tapisenabled [$t cget -chain-position]]} {
			$t arp_reset deassert $halt
		}
	}
	foreach t $targets {
		$t invoke-event reset-deassert-post
	}

	# Pass 1 - Now wait for any halt (requested as part of reset
	# assert/deassert) to happen.  Ideally it takes effect without
	# first executing any instructions.
	if { $halt } {
		foreach t $targets {
			if {[jtag tapisenabled [$t cget -chain-position]] == 0} {
				continue
			}

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
		foreach t $targets {
			if {[jtag tapisenabled [$t cget -chain-position]] == 0} {
				continue
			}

			set err [catch "$t arp_waitstate halted 5000"]
			# Did it halt?
			if { $err == 0 } {
				$t invoke-event reset-init
			}
		}
	}

	foreach t $targets {
		$t invoke-event reset-end
	}
}

#########

# REVISIT power_restore, power_dropout, srst_deasserted, srst_asserted
# are currently neither documented nor supported except on ZY1000.

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

#########

# catch any exceptions, capture output and return output
proc capture_catch {a} {
	catch {
		capture {uplevel $a}
	} result
	return $result
}


# Executed during "init". Can be overridden
# by board/target/... scripts
proc jtag_init {} {
	if {[catch {jtag arp_init} err]!=0} {
		# try resetting additionally
		init_reset startup
	}
}

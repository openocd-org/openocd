# SPDX-License-Identifier: GPL-2.0-or-later

# Defines basic Tcl procs for OpenOCD server modules

# Handle GDB 'R' packet. Can be overridden by configuration script,
# but it's not something one would expect target scripts to do
# normally
proc ocd_gdb_restart {target_id} {
	# Fix!!! we're resetting all targets here! Really we should reset only
	# one target
	reset halt
}

lappend _telnet_autocomplete_skip prevent_cps
lappend _telnet_autocomplete_skip POST
lappend _telnet_autocomplete_skip Host:
proc prevent_cps {} {
	echo "Possible SECURITY ATTACK detected."
	echo "It looks like somebody is sending POST or Host: commands to OpenOCD."
	echo "This is likely due to an attacker attempting to use Cross Protocol Scripting"
	echo "to compromise your OpenOCD instance. Connection aborted."
	exit
}

proc POST {args} { prevent_cps }
proc Host: {args} { prevent_cps }

# list of commands we don't want to appear in autocomplete
lappend _telnet_autocomplete_skip _telnet_autocomplete_helper

# helper for telnet autocomplete
proc _telnet_autocomplete_helper pattern {
	set cmds [info commands $pattern]

	# skip matches in variable '_telnet_autocomplete_skip'
	foreach skip $::_telnet_autocomplete_skip {
		foreach n [lsearch -all -regexp $cmds "^$skip\$"] {
			set cmds [lreplace $cmds $n $n]
		}
	}

	return [lsort $cmds]
}

lappend _telnet_autocomplete_skip "gdb_sync"
proc "gdb_sync" {} {
	echo "DEPRECATED! use 'gdb sync', not 'gdb_sync'"
	eval gdb sync
}

lappend _telnet_autocomplete_skip "gdb_port"
proc "gdb_port" {args} {
	echo "DEPRECATED! use 'gdb port', not 'gdb_port'"
	eval gdb port $args
}

lappend _telnet_autocomplete_skip "gdb_memory_map"
proc "gdb_memory_map" {state} {
	echo "DEPRECATED! use 'gdb memory_map', not 'gdb_memory_map'"
	eval gdb memory_map $state
}

lappend _telnet_autocomplete_skip "gdb_flash_program"
proc "gdb_flash_program" {state} {
	echo "DEPRECATED! use 'gdb flash_program', not 'gdb_flash_program'"
	eval gdb flash_program $state
}

lappend _telnet_autocomplete_skip "gdb_report_data_abort"
proc "gdb_report_data_abort" {state} {
	echo "DEPRECATED! use 'gdb report_data_abort', not 'gdb_report_data_abort'"
	eval gdb report_data_abort $state
}

lappend _telnet_autocomplete_skip "gdb_report_register_access_error"
proc "gdb_report_register_access_error" {state} {
	echo "DEPRECATED! use 'gdb report_register_access_error', not 'gdb_report_register_access_error'"
	eval gdb report_register_access_error $state
}

lappend _telnet_autocomplete_skip "gdb_breakpoint_override"
proc "gdb_breakpoint_override" {override} {
	echo "DEPRECATED! use 'gdb breakpoint_override', not 'gdb_breakpoint_override'"
	eval gdb breakpoint_override $override
}

lappend _telnet_autocomplete_skip "gdb_target_description"
proc "gdb_target_description" {state} {
	echo "DEPRECATED! use 'gdb target_description', not 'gdb_target_description'"
	eval gdb target_description $state
}

lappend _telnet_autocomplete_skip "gdb_save_tdesc"
proc "gdb_save_tdesc" {} {
	echo "DEPRECATED! use 'gdb save_tdesc', not 'gdb_save_tdesc'"
	eval gdb save_tdesc
}

lappend _telnet_autocomplete_skip "tcl_port"
proc "tcl_port" {args} {
	echo "DEPRECATED! use 'tcl port' not 'tcl_port'"
	eval tcl port $args
}

lappend _telnet_autocomplete_skip "tcl_notifications"
proc "tcl_notifications" {state} {
	echo "DEPRECATED! use 'tcl notifications' not 'tcl_notifications'"
	eval tcl notifications $state
}

lappend _telnet_autocomplete_skip "tcl_trace"
proc "tcl_trace" {state} {
	echo "DEPRECATED! use 'tcl trace' not 'tcl_trace'"
	eval tcl trace $state
}

lappend _telnet_autocomplete_skip "telnet_port"
proc "telnet_port" {args} {
	echo "DEPRECATED! use 'telnet port', not 'telnet_port'"
	eval telnet port $args
}

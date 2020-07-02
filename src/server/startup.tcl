# Defines basic Tcl procs for OpenOCD server modules

# Handle GDB 'R' packet. Can be overridden by configuration script,
# but it's not something one would expect target scripts to do
# normally
proc ocd_gdb_restart {target_id} {
	# Fix!!! we're resetting all targets here! Really we should reset only
	# one target
	reset halt
}

proc prevent_cps {} {
	echo "Possible SECURITY ATTACK detected."
	echo "It looks like somebody is sending POST or Host: commands to OpenOCD."
	echo "This is likely due to an attacker attempting to use Cross Protocol Scripting"
	echo "to compromise your OpenOCD instance. Connection aborted."
	exit
}

proc POST {args} { prevent_cps }
proc Host: {args} { prevent_cps }

# Defines basic Tcl procs for OpenOCD flash module

# Show flash in human readable form
# This is an example of a human readable form of a low level fn
proc flash_banks {} {
	set i 0
	set result ""
	foreach {a} [ocd_flash banks] {
		if {$i > 0} {
			set result "$result\n"
		}
		set result [format "$result#%d: %s at 0x%08x, size 0x%08x, buswidth %d, chipwidth %d" $i $a(name) $a(base) $a(size) $a(bus_width) $a(chip_width)]
		set i [expr $i+1]
	}
	return $result
}

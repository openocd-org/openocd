# SPDX-License-Identifier: GPL-2.0-or-later

# Defines basic Tcl procs that must exist for OpenOCD scripts to work.
#
# Embedded into OpenOCD executable
#

# Try flipping / and \ to find file if the filename does not
# match the precise spelling
# lappend _telnet_autocomplete_skip _find_internal
proc _find_internal {filename} {
	if {[catch {ocd_find $filename} t]==0} {
		return $t
	}
	if {[catch {ocd_find [string map {\\ /} $filename]} t]==0} {
		return $t
	}
	if {[catch {ocd_find [string map {/ \\} $filename]} t]==0} {
		return $t
	}
	# make sure error message matches original input string
	return -code error "Can't find $filename"
}

proc find {filename} {
	if {[catch {_find_internal $filename} t]==0} {
		return $t
	}

	# Check in vendor specific folder:

	# - path/to/a/certain/vendor_config_file
	# - path/to/a/certain/vendor-config_file
	# replaced with
	# - path/to/a/certain/vendor/config_file
	regsub {([/\\])([^/\\_-]*)[_-]([^/\\]*$)} $filename "\\1\\2\\1\\3" f
	if {[catch {_find_internal $f} t]==0} {
		echo "WARNING: '$filename' is deprecated, use '$f' instead"
		return $t
	}

	foreach vendor {nordic ti sifive st} {
		# - path/to/a/certain/config_file
		# replaced with
		# - path/to/a/certain/${vendor}/config_file
		regsub {([/\\])([^/\\]*$)} $filename "\\1$vendor\\1\\2" f
		if {[catch {_find_internal $f} t]==0} {
			echo "WARNING: '$filename' is deprecated, use '$f' instead"
			return $t
		}
	}

	# at last, check for explicit renaming
	if {[catch {
		source [_find_internal file_renaming.cfg]
		set unixname [string map {\\ /} $filename]
		regsub {^(.*/|)((board|chip|cpld|cpu|fpga|interface|target|test|tools)/.*.cfg$)} $unixname {{\1} {\2}} split
		set newname [lindex $split 0][dict get $_file_renaming [lindex $split 1]]
		_find_internal $newname
	} t]==0} {
		echo "WARNING: '$filename' is deprecated, use '$newname' instead"
		return $t
	}

	return -code error "Can't find $filename"
}
add_usage_text find "<file>"
add_help_text find "print full path to file according to OpenOCD search rules"

# Find and run a script
proc script {filename} {
	uplevel #0 [list source [find $filename]]
}
add_help_text script "filename of OpenOCD script (tcl) to run"
add_usage_text script "<file>"

# Run a list of post-init commands
# Each command should be added with 'lappend post_init_commands command'
lappend _telnet_autocomplete_skip _run_post_init_commands
proc _run_post_init_commands {} {
	if {[info exists ::post_init_commands]} {
		foreach cmd $::post_init_commands {
			eval $cmd
		}
	}
}

# Run a list of pre-shutdown commands
# Each command should be added with 'lappend pre_shutdown_commands command'
lappend _telnet_autocomplete_skip _run_pre_shutdown_commands
proc _run_pre_shutdown_commands {} {
	if {[info exists ::pre_shutdown_commands]} {
		foreach cmd $::pre_shutdown_commands {
			eval $cmd
		}
	}
}

#########

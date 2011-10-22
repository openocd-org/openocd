# Defines basic Tcl procs that must exist for OpenOCD scripts to work.
#
# Embedded into OpenOCD executable
#


# We need to explicitly redirect this to the OpenOCD command
# as Tcl defines the exit proc
proc exit {} {
	ocd_throw exit
}

# All commands are registered with an 'ocd_' prefix, while the "real"
# command is a wrapper that calls this function.  Its primary purpose is
# to discard 'handler' command output,
proc ocd_bouncer {name args} {
	set cmd [format "ocd_%s" $name]
	set type [eval ocd_command type $cmd $args]
	if {$type == "native"} {
		return [eval $cmd $args]
	} else {if {$type == "simple"} {
		if {[catch {eval $cmd $args}] == 0} {
			return ""
		} else {
			# 'classic' commands output error message as part of progress output
			set errmsg ""
		}
	} else {if {$type == "group"} {
		catch {eval ocd_usage $name $args}
		set errmsg [format "%s: command requires more arguments" \
			[concat $name " " $args]]
	} else {
		set errmsg [format "Unknown command type: %s" $type]
	}}}
	return -code error $errmsg
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
add_usage_text find "<file>"
add_help_text find "print full path to file according to OpenOCD search rules"

# Find and run a script
proc script {filename} {
	uplevel #0 [list source [find $filename]]
}
add_help_text script "filename of OpenOCD script (tcl) to run"
add_usage_text script "<file>"

#########


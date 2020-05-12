# Defines basic Tcl procs that must exist for OpenOCD scripts to work.
#
# Embedded into OpenOCD executable
#

# handle commands help and usage
namespace eval openocd_help {
	namespace export help usage add_help_text del_help_text add_usage_text del_usage_text
	variable help_array {}
	variable usage_array {}
	variable line_width 75

	proc add_help_text {command_name helptext_string} {
		variable help_array

		set help_array($command_name) ${helptext_string}
		return {}
	}

	proc del_help_text {command_name} {
		variable help_array

		unset -nocomplain help_array($command_name)
		return {}
	}

	proc add_usage_text {command_name usagetext_string} {
		variable usage_array

		set usage_array($command_name) ${usagetext_string}
		return {}
	}

	proc del_usage_text {command_name} {
		variable usage_array

		unset -nocomplain usage_array($command_name)
		return {}
	}

	proc search_token {token} {
		variable help_array
		variable usage_array
		set result {}

		if {${token} eq ""} {
			return [lsort -unique [concat [array names help_array] [array names usage_array]]]
		}

		foreach key [array names help_array] {
			if {[string first ${token} ${key}] != -1} {lappend result ${key}; continue}
			if {[string first ${token} $help_array(${key})] != -1} {lappend result ${key}}
		}
		foreach key [array names usage_array] {
			if {[string first ${token} ${key}] != -1} {lappend result ${key}; continue}
			if {[string first ${token} $usage_array(${key})] != -1} {lappend result ${key}}
		}

		return [lsort -unique ${result}]
	}

	# Wrap "text" to fit in lines of "line_length" characters.
	# Add "first_line_indent" spaces in front of first line and
	# "left_indent" spaces in front of the remaining lines.
	# Wrap only happens at space character.
	# Then echo the result.

	proc wrap_echo {first_line_indent left_indent text} {
		variable line_width

		# Find where to split 1st line
		set text [string repeat " " $first_line_indent]${text}
		regexp -indices ".{1,$line_width} " ${text}\x20 position
		set position [lindex $position 1]

		set splitter \n[string repeat " " $left_indent]\\1
		set new_width [expr $line_width - $left_indent]
		# Split all lines
		echo [regsub -all -start $position " *(.{1,$new_width})( +|$)" ${text} $splitter]
	}

	proc help {args} {
		variable help_array
		variable usage_array

		foreach cmd [search_token [eval concat $args]] {
			set subcmds_n [regexp -all " " $cmd]
			set usage_indent_1 [expr 2 * $subcmds_n]
			set usage_indent_2 [expr $usage_indent_1 + 10]
			set help_indent [expr $usage_indent_1 + 6]

			if {[info exists usage_array($cmd)] && $usage_array($cmd) ne ""} {
				wrap_echo $usage_indent_1 $usage_indent_2 ${cmd}\x20$usage_array(${cmd})
			} else {
				wrap_echo $usage_indent_1 $usage_indent_2 $cmd
			}

			set help_text ""
			if {[info exists help_array($cmd)] && $help_array($cmd) ne ""} {
				set help_text $help_array($cmd)
			}
			switch [eval command mode $cmd] {
				"any"    {set help_text [concat ${help_text} (command valid any time)]}
				"config" {set help_text [concat ${help_text} (configuration command)]}
				default  {}
			}
			if {$help_text ne ""} {
				wrap_echo $help_indent $help_indent ${help_text}
			}
		}
	}

	proc usage {args} {
		variable usage_array

		foreach cmd [search_token [eval concat $args]] {
			set subcmds_n [regexp -all " " $cmd]
			set usage_indent_1 [expr 2 * $subcmds_n]
			set usage_indent_2 [expr $usage_indent_1 + 10]

			if {[info exists usage_array($cmd)] && $usage_array($cmd) ne ""} {
				wrap_echo $usage_indent_1 $usage_indent_2 "$cmd $usage_array($cmd)"
			} else {
				wrap_echo $usage_indent_1 $usage_indent_2 $cmd
			}
		}
	}
}
namespace import openocd_help::help openocd_help::usage openocd_help::add_* openocd_help::del_*

add_help_text help "Show full command help; command can be multiple tokens."
add_usage_text help {[command_name]}

add_help_text usage "Show basic command usage; command can be multiple tokens."
add_usage_text usage {[command_name]}

add_help_text add_help_text "Add new command help text; Command can be multiple tokens."
add_usage_text add_help_text "command_name helptext_string"

add_help_text del_help_text "Delete a command help text; Command can be multiple tokens."
add_usage_text del_help_text "command_name"

add_help_text add_usage_text "Add new command usage text; command can be multiple tokens."
add_usage_text add_usage_text "command_name usage_string"

add_help_text del_usage_text "Delete a command usage text; command can be multiple tokens."
add_usage_text del_usage_text "command_name"

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

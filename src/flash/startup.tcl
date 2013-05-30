# Defines basic Tcl procs for OpenOCD flash module

#
# program utility proc
# usage: program filename
# optional args: verify, reset and address
#

proc program {filename args} {

	foreach arg $args {
		if {[string equal $arg "verify"]} {
			set verify 1
		} elseif {[string equal $arg "reset"]} {
			set reset 1
		} else {
			set address $arg
		}
	}

	# make sure init is called
	if {[catch {init}] != 0} {
		echo "** OpenOCD init Failed **"
		shutdown
		return
	}

	# reset target and call any init scripts
	if {[catch {reset init}] != 0} {
		echo "** Unable to reset target **"
		shutdown
		return
	}

	# start programming phase
	echo "** Programming Started **"
	if {[info exists address]} {
		set flash_args "$filename $address"
	} else {
		set flash_args "$filename"
	}

	if {[catch {eval flash write_image erase $flash_args}] == 0} {
		echo "** Programming Finished **"
		if {[info exists verify]} {
			# verify phase
			echo "** Verify Started **"
			if {[catch {eval verify_image $flash_args}] == 0} {
				echo "** Verified OK **"
			} else {
				echo "** Verify Failed **"
			}
		}

		if {[info exists reset]} {
			# reset target if requested
			# also disable target polling, we are shutting down anyway
			poll off
			echo "** Resetting Target **"
			reset run
		}
	} else {
		echo "** Programming Failed **"
	}

	# shutdown OpenOCD
	shutdown
}

add_help_text program "write an image to flash, address is only required for binary images. verify, reset are optional"
add_usage_text program "<filename> \[address\] \[verify\] \[reset\]"

# stm32f0x uses the same flash driver as the stm32f1x
# this alias enables the use of either name.
proc stm32f0x args {
	eval stm32f1x $args
}

# stm32f3x uses the same flash driver as the stm32f1x
# this alias enables the use of either name.
proc stm32f3x args {
	eval stm32f1x $args
}

# stm32f4x uses the same flash driver as the stm32f2x
# this alias enables the use of either name.
proc stm32f4x args {
	eval stm32f2x $args
}

# ease migration to updated flash driver
proc stm32x args {
	echo "DEPRECATED! use 'stm32f1x $args' not 'stm32x $args'"
	eval stm32f1x $args
}

proc stm32f2xxx args {
	echo "DEPRECATED! use 'stm32f2x $args' not 'stm32f2xxx $args'"
	eval stm32f2x $args
}

# SPDX-License-Identifier: GPL-2.0-or-later

namespace eval testing_helpers {

	proc test_failure message {
		echo $message
		shutdown error
	}

	proc check_for_error {expctd_code msg_ptrn script} {
		set code [catch {uplevel 1 $script} msg]
		if {!$code} {
			test_failure \
				"'$script' finished successfully. \
				Was expecting an error."
		}
		if {$expctd_code ne "" && $code != $expctd_code} {
			test_failure \
				"'$script' returned unexpected error code $code. \
				Was expecting $expctd_code. Error message: '$msg'"
		}
		if {$msg_ptrn ne "" && ![regexp $msg_ptrn $msg]} {
			test_failure \
				"'$script' returned unexpected error message '$msg'. \
				Was expecting '$msg_ptrn'. Error code: $code"
		}
	}

	proc check_generic_error script {
		tailcall check_for_error 1 {} $script
	}

	proc check_invalid_arg script {
		tailcall check_for_error -603 {} $script
	}

	proc check_syntax_err script {
		tailcall check_for_error -601 {} $script
	}

	proc check_overflow_err script {
		tailcall check_for_error -604 {} $script
	}

	proc check_underflow_err script {
		tailcall check_for_error -605 {} $script
	}

	proc check_matches {pattern script} {
		set result [uplevel $script]
		if {[regexp $pattern $result]} {return}
		test_failure \
			"'$script' produced unexpected result '$result'. \
			Was expecting '$pattern'."
	}

	namespace export check_generic_error check_invalid_arg check_syntax_err \
		check_overflow_err check_underflow_err check_matches
}

namespace eval jtag_dummy_testing {

	namespace import testing_helpers::*

	variable ntaps

	proc setup {arg_ntaps} {
		variable ntaps $arg_ntaps
		adapter driver dummy
		proc ::jtag_init {} {}
		for {set i 0} {$i < $ntaps} {incr i} {
			jtag newtap "tap$i" tap -irlen 1
		}
		dummy update_context
		dummy_set_default_handlers

		namespace eval ::testing_helpers {
			variable old_check_for_error [list [info args check_for_error] [info body check_for_error]]

			proc check_for_error {expctd_code msg_ptrn script} {
				variable old_check_for_error
				dummy_expect_no_operations
				uplevel 1 "apply {$old_check_for_error} {$expctd_code} {$msg_ptrn} {$script}"
				dummy_set_default_handlers
			}
		}
	}

	proc dummy_set_default_handlers {} {
		namespace eval ::dummy {
			proc state_transition {state tdi} {}
			proc get_tdo {} {return {tdo: 0}}
		}
	}

	proc dummy_expect_no_operations {} {
		namespace eval ::dummy {
			proc state_transition {state tdi} {
				error "Expecting no JTAG transitions."
			}
			proc get_tdo {} {
				error "Expecting reads of TDO."
			}
		}
	}

	namespace export setup
}

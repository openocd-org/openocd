# SPDX-License-Identifier: GPL-2.0-or-later

namespace eval testing_helpers {

	proc test_failure message {
		echo $message
		shutdown error
	}

	proc check_for_error {expctd_code msg_ptrn script} {
		set code [catch {uplevel $script} msg]
		set expanded_script [uplevel subst \"$script\"]
		if {!$code} {
			test_failure \
				"'$expanded_script' finished successfully. \
				Was expecting an error."
		}
		if {$expctd_code ne "" && $code != $expctd_code} {
			test_failure \
				"'$expanded_script' returned unexpected error code $code. \
				Was expecting $expctd_code. Error message: '$msg'"
		}
		if {$msg_ptrn ne "" && ![regexp -- $msg_ptrn $msg]} {
			test_failure \
				"'$expanded_script' returned unexpected error message '$msg'. \
				Was expecting '$msg_ptrn'. Error code: $code"
		}
	}

	proc check_error_matches {pattern script} {
		tailcall check_for_error {} $pattern $script
	}

	proc check_syntax_err script {
		tailcall check_for_error 1 {} $script
	}

	proc check_matches {pattern script} {
		set result [uplevel $script]
		if {[regexp $pattern $result]} {return}
		test_failure \
			"'$script' produced unexpected result '$result'. \
			Was expecting '$pattern'."
	}

	namespace export check_error_matches check_syntax_err check_matches
}

namespace eval configure_testing {

	variable target_idx 0

	proc unique_tgt_name {} {
		variable target_idx
		incr target_idx
		return test_target$target_idx
	}

	proc target_create_first_args {} {
		return "target create [unique_tgt_name] testee"
	}

	proc simple_configure_options {} {
		return {
			-work-area-virt 0
			-work-area-phys 0
			-work-area-size 1
			-work-area-backup 0
			-endian little
			-coreid 1
			-chain-position tap.cpu
			-dbgbase 0
			-rtos hwthread
			-gdb-port 0
			-gdb-max-connections 1
		}
	}

	namespace export target_create_first_args simple_configure_options
}

# SPDX-License-Identifier: GPL-2.0-or-later

# Description:
#  Measure the CPU clock frequency of an ARM Cortex-M based device.
#
# Return:
#  The CPU clock frequency in Hz. A negative value indicates that the loop
#  counter was saturated.
#
# Note:
#  You may need to adapt the number of cycles for your device.
#
add_help_text cortex_m_test_cpu_speed "Measure the CPU clock frequency of an ARM Cortex-M based device"
add_usage_text cortex_m_test_cpu_speed {address [timeout [cycles_per_loop]]}
proc cortex_m_test_cpu_speed { address { timeout 200 } { cycles_per_loop 4 } } {
	set loop_counter_start 0xffffffff

	halt

	# Backup registers and memory.
	set backup_regs [get_reg -force {pc r0 xPSR}]
	set backup_mem [read_memory $address 16 3]

	# We place the following code at the given address to measure the
	# CPU clock frequency:
	#
	# 3801: subs r0, #1
	# d1fd: bne #-2
	# e7fe: b #-4
	write_memory $address 16 {0x3801 0xd1fd 0xe7fe}

	set_reg "pc $address r0 $loop_counter_start"
	resume
	sleep $timeout
	halt

	# Get the loop counter value from register r0.
	set loop_counter_end [dict values [get_reg r0]]
	set loop_counter_diff [expr {$loop_counter_start - $loop_counter_end}]

	# Restore registers and memory.
	set_reg $backup_regs
	write_memory $address 16 $backup_mem

	if { [expr {$loop_counter_end == 0}] } {
		return -1
	}

	return [expr {double($loop_counter_diff) * $cycles_per_loop / $timeout * 1000}]
}

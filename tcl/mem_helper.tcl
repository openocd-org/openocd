# SPDX-License-Identifier: GPL-2.0-or-later

# Helper for common memory read/modify/write procedures

# mrw: "memory read word", returns value of $reg
proc mrw {reg} {
	return [read_memory $reg 32 1]
}

add_usage_text mrw "address"
add_help_text mrw "Returns value of word in memory."

# mrh: "memory read halfword", returns value of $reg
proc mrh {reg} {
	return [read_memory $reg 16 1]
}

add_usage_text mrh "address"
add_help_text mrh "Returns value of halfword in memory."

# mrb: "memory read byte", returns value of $reg
proc mrb {reg} {
	return [read_memory $reg 8 1]
}

add_usage_text mrb "address"
add_help_text mrb "Returns value of byte in memory."

# mmw: "memory modify word", updates value of $reg
#       $reg <== ((value & ~$clearbits) | $setbits)
proc mmw {reg setbits clearbits} {
	set old [mrw $reg]
	set new [expr {($old & ~$clearbits) | $setbits}]
	mww $reg $new
}

add_usage_text mmw "address setbits clearbits"
add_help_text mmw "Modify word in memory. new_val = (old_val & ~clearbits) | setbits;"

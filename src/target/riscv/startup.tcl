# SPDX-License-Identifier: GPL-2.0-or-later

lappend _telnet_autocomplete_skip "riscv set_enable_virtual"
proc {riscv set_enable_virtual} on_off {
	echo {DEPRECATED! use 'riscv virt2phys_mode' not 'riscv set_enable_virtual'}
	foreach t [target names] {
		if {[$t cget -type] ne "riscv"} { continue }
		switch -- [$t riscv virt2phys_mode] {
			off -
			hw {
				switch -- $on_off {
					on  {$t riscv virt2phys_mode hw}
					off {$t riscv virt2phys_mode off}
				}
			}
			sw {
				if {$on_off eq "on"} {
					error {Can't enable virtual while translation mode is SW}
				}
			}
		}
	}
	return {}
}

lappend _telnet_autocomplete_skip "riscv set_enable_virt2phys"
proc {riscv set_enable_virt2phys} on_off {
	echo {DEPRECATED! use 'riscv virt2phys_mode' not 'riscv set_enable_virt2phys'}
	foreach t [target names] {
		if {[$t cget -type] ne "riscv"} { continue }
		switch -- [riscv virt2phys_mode] {
			off -
			sw {
				switch -- $on_off {
					on  {riscv virt2phys_mode sw}
					off {riscv virt2phys_mode off}
				}
			}
			hw {
				if {$on_off eq "on"} {
					error {Can't enable virt2phys while translation mode is HW}
				}
			}
		}
	}
	return {}
}

foreach mode {m s u} {
	lappend _telnet_autocomplete_skip "riscv set_ebreak$mode"
}

proc riscv {cmd args} {
	tailcall "riscv $cmd" {*}$args
}

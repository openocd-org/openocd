# SPDX-License-Identifier: GPL-2.0-or-later

# Utilities to simplify watchpoint management in certain scenarios
#
# Copyright (c) 2023-2025, Syntacore LLC

proc sc_lib_watchpoints_stash {} {
  if {[[target current] curstate] ne "halted"} {
    error "could not stash watchpoints, since [target current] is not halted"
  }
  global __SYNTACORE_WATCHPOINT_STASH
  if {[info exists __SYNTACORE_WATCHPOINT_STASH]} {
    if {[llength $__SYNTACORE_WATCHPOINT_STASH] > 0} {
      error "could not stash watchpoints, since existing stash is not empty"
    }
  }
  set __SYNTACORE_WATCHPOINT_STASH [list]
  set wp_list [wp]
  foreach wp_record [split $wp_list "\n"] {
    set wp_record [string trim $wp_record]
    if { $wp_record eq "" } {
      continue
    }
    # Output example:
    # address: 0x00000000, len: 0x00000004, r/w/a: 2, value: 0x00000000, mask: 0xffffffffffffffff
    if {![regexp {address: +([^ ,]+), len: +([^ ,]+), r/w/a: +([^ ,]+), value: +([^ ,]+), mask: +([^ ,]+)} $wp_record \
      m addr len type value mask]} {
      error "could not parse watchpoint description: $wp_record"
    }
    lappend __SYNTACORE_WATCHPOINT_STASH \
      [list $addr $len $type $value $mask]
    rwp $addr
  }
  echo "[llength $__SYNTACORE_WATCHPOINT_STASH] watchpoints stashed!"
}

proc sc_lib_watchpoints_stash_drop {} {
  global __SYNTACORE_WATCHPOINT_STASH
  set __SYNTACORE_WATCHPOINT_STASH [list]
}

proc sc_lib_watchpoints_restore {} {
  if {[[target current] curstate] ne "halted"} {
    error "could not restore watchpoints, since [target current] is not halted"
  }
  global __SYNTACORE_WATCHPOINT_STASH
  if {![info exists __SYNTACORE_WATCHPOINT_STASH]} {
    set __SYNTACORE_WATCHPOINT_STASH [list]
  }

  if {[string trim [string cat [split [wp] "\n"]]] != ""} {
    error "unable to restore watchpoints stash, since there are pending watchpoints"
  }

  foreach wp_record $__SYNTACORE_WATCHPOINT_STASH {
    set addr [lindex $wp_record 0]
    set len [lindex $wp_record 1]
    set type [lindex $wp_record 2]
    if { $type == 2 } {
      set type a
    } elseif { $type == 1 } {
      set type w
    } elseif { $type == 0 } {
      set type r
    }
    set value [lindex $wp_record 3]
    set mask [lindex $wp_record 4]
    echo "restoring watchpoint: wp $addr $len $type $value $mask"
    wp $addr $len $type $value $mask
  }
  sc_lib_watchpoints_stash_drop
}

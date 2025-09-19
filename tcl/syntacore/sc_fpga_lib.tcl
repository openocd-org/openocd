# SPDX-License-Identifier: GPL-2.0-or-later

# A collection of reference scripts that are useful for basic fpga tasks
#
# Copyright (c) 2023-2025, Syntacore LLC
#
# If you experience difficulties with these scripts or would like to introduce
# a new functionality - feel free to contact OpenOCD maintainer at Syntacore.

# Known Problems/Bugs:
#
# - sc_fpga_halt_all/sc_fpga_resume_all are not bullet-proof. The current
#   implementation assumes that only one SMP group can exist and that all
#   targets are part of it.
#   The problem is that it is possible to have several SMP groups or to have a
#   target that does not belong to any. Currently, OpenOCD does not expose
#   functionality to identify which target belongs to which group. It only
#   allows us to query if a target belongs to some group. So these functions
#   are not safe to use in such contexts. That being said, we do hope that
#   such contexts are seldom (well, never :), to be more precise) used and most
#   users don't need to worry about that.

namespace eval _SC_INTERNALS {
    variable FPGA_LIB_VERBOSE_MODE 1
    proc sc_fpga_fname {} {
        return [lindex [info level -2] 0]
    }

    proc sc_lib_print { msg } {
        variable FPGA_LIB_VERBOSE_MODE
        if {${FPGA_LIB_VERBOSE_MODE}} {
          echo "[sc_fpga_fname]: $msg"
        }
    }

    proc fill_gprs_with_zero {} {
        for { set reg_idx 1 } { $reg_idx < 32 } { incr reg_idx } {
            reg $reg_idx 0
        }
    }

    proc sc_lib_do_resume_impl { addr } {
        if { $addr eq ""} {
            resume
        } else {
            resume $addr
        }
    }

    proc sc_lib_do_resume { addr } {
        # TODO: currently, TCL-interpreter used by OpenOCD does not clear
        # pending exception info, so for now we don't use exceptions here
        # set log [capture \
        #    { set code [ catch { sc_fpga_internal_do_resume_impl $addr } ex ] }]
        # if {$code != 0 } {
        #   return -code error "could not resume target! code: $code, log: $log, ex: $ex"
        # }
        sc_lib_do_resume_impl $addr
    }

    proc sc_lib_run_impl { file_path file_type load_address entry_point mode { primary_target ""} } {
        sc_fpga_halt_all
        if { $primary_target eq "" } {
            set primary_target [lindex [target names] 0]
            sc_lib_print "$primary_target derived as primary target"
        }
        sc_lib_print "switching active target to $primary_target"
        targets $primary_target
        set load_image_args [list $file_path]
        if { $load_address ne "" } {
            lappend load_image_args $load_address
            lappend load_image_args $file_type
        }
        sc_lib_print "load_image $load_image_args"
        set load_result [string trim [load_image {*}$load_image_args]]
        sc_lib_print "$load_result"

        if { $entry_point eq "" && $load_address ne "" } {
            sc_lib_print \
                "no entry point was specified, using load address ($load_address) as one"
            set entry_point $load_address
        }

        if { $mode eq "all" } {
            sc_fpga_resume_all $entry_point
        } else {
            resume $entry_point
        }
        return $entry_point
    }

    proc apply_for_each_target { function_name args } {
        set current_target [target current]
        foreach t [target names] {
            targets $t
            if {[llength $args] == 0} {
                $function_name
            } else {
                $function_name {*}$args
            }
        }
        targets $current_target
    }

    proc sc_lib_read_reg_hex {reg_name} {
        set raw_reg_value [string trim [reg $reg_name]]
        sc_lib_print "$raw_reg_value"
        return [string trim [lindex [split $raw_reg_value :] 1]]
    }

    proc sc_lib_write_reg { reg_name value } {
        return [string trim [reg $reg_name $value]]
    }

    proc sc_lib_require_halted {} {
        poll
        set current_state [[target current] curstate]
        if {$current_state ne "halted"} {
            error "current state $current_state for [target current] is not halted"
        }
    }

    proc sc_lib_experimental_reset_pmu_subsystem { pmu_ctrs_max } {
        set k_inhibit_all 0xffffffff
        # should re-program mcountinhibit/mcycle/minstret
        sc_lib_print "[sc_lib_write_reg mcountinhibit $k_inhibit_all]"
        sc_lib_print "[sc_lib_write_reg mcycle 0]"
        sc_lib_print "[sc_lib_write_reg minstret 0]"
        # drop mhpmevent selectors and mhpmcounter
        for { set ev_idx 0 } { $ev_idx < $pmu_ctrs_max } { incr ev_idx } {
            set EventSelector [expr {3 + $ev_idx}]
            sc_lib_print "[sc_lib_write_reg mhpmevent${EventSelector} 0]"
            sc_lib_print "[sc_lib_write_reg mhpmcounter${EventSelector} 0]"
        }
    }

    proc sc_lib_experimental_enable_pmu_counters { selectors_list
                                                   pmu_ctrs_max
                                                   en_cy
                                                   en_ir
                                                   pmu_ctrs } {
        set pmu_ctrs [lsort -unique $pmu_ctrs]
        if {[llength pmu_ctrs] > $pmu_ctrs_max} {
            error "too many PMU counters requested was requested"
        }
        set inhibit_value 0xffffffff
        if { $en_cy != 0 } {
            set inhibit_value [expr { $inhibit_value ^ 1 }]
        }
        if { $en_ir != 0 } {
            set inhibit_value [expr { $inhibit_value ^ 4 }]
        }
        set pmu_counter_idx 0
        foreach pmu_event $pmu_ctrs {
            set event_selector [dict get $selectors_list $pmu_event]
            # Set OF flag
            set mhpmevent_val [expr { $event_selector | (1 << 63)}]
            set selector_reg "mhpmevent[expr {3 + $pmu_counter_idx}]"
            set mhpmevent_hex [format "0x%016x" $mhpmevent_val]
            set inhibit_value [expr { $inhibit_value ^ (1 << ($pmu_counter_idx + 3))}]
            sc_lib_print "$pmu_event - [sc_lib_write_reg $selector_reg $mhpmevent_hex]"
            incr pmu_counter_idx
        }
        sc_lib_print "[sc_lib_write_reg mcountinhibit [format "0x%08x" $inhibit_value]]"
    }

    ## aarsize - data size: 2 (32 bits), 3 (64 bits), 4 (128 bits)
    proc sc_lib_get_aarsize {xlen} {
        if {$xlen == 32} {
            return 2
        }
        if {$xlen == 64} {
            return 3
        }
        return -code error "invalid MXL value: $xlen"
    }

    proc sc_lib_riscv_encode_abstarct_command {regno write transfer aarsize cmdtype} {
        set cmd [expr {($regno & 0xFFFF) |
            ($write << 16) |
            ($transfer << 17) |
            ($aarsize << 20) |
            ($cmdtype << 24)}]
        return $cmd
    }

    proc sc_lib_require_halted_and_check_csr {csr_num} {
        sc_lib_require_halted
        if {$csr_num > 4069 || $csr_num < 0} {
            return -code error "$csr_num is not a valid CSR number"
        }
    }

    variable FPGA_LIB_BUSY_DURATION 3

    proc sc_lib_riscv_csr_impl {csr_num value write xlen} {
        set aarsize [sc_lib_get_aarsize $xlen]
        set COMMAND_ADDR 0x17
        riscv dmi_write $COMMAND_ADDR [sc_lib_riscv_encode_abstarct_command $csr_num \
            $write 1 $aarsize 0]

        set ABSTRACTCS_ADDR 0x16
        set abstractcs [riscv dmi_read $ABSTRACTCS_ADDR]
        set start_time [clock seconds]

        while {$abstractcs & 0x1000 != 0} {
            if {([clock seconds] - $start_time) >= [sc_fpga_get_busy_duration]} {
                return -code error "Busy bit set after duration time"
            }
            set abstractcs [riscv dmi_read $ABSTRACTCS_ADDR]
        }

        set cmderr [expr {($abstractcs & 0x700) >> 8}]
        if {$cmderr != 0} {
            riscv dmi_write $ABSTRACTCS_ADDR 0x700
            return -code error "problem with abstract command execution, \
                    error code : $cmderr"
        }
    }
}

proc sc_fpga_ctrl_silence {} {
    set ::_SC_INTERNALS::FPGA_LIB_VERBOSE_MODE 0
    echo "sc_fpga_lib silent mode activated"
}
proc sc_fpga_ctrl_verbose {} {
    set ::_SC_INTERNALS::FPGA_LIB_VERBOSE_MODE 1
    echo "sc_fpga_lib verbose mode activated"
}

proc sc_fpga_read_reg {reg_name} {
    set hex_value [_SC_INTERNALS::sc_lib_read_reg_hex $reg_name]
    set result [format %u $hex_value]
    _SC_INTERNALS::sc_lib_print "$reg_name: $result"
    return $result
}

proc sc_fpga_write_reg { reg_name value } {
    set result [_SC_INTERNALS::sc_lib_write_reg $reg_name $value]
    _SC_INTERNALS::sc_lib_print "$result"
    return $result
}

proc sc_fpga_halt_all {} {
    # TODO: take into account cases when several SMP groups are present or
    # there are targets that does not belong to any SMP group
    if {[string trim [smp]] eq "off"} {
        _SC_INTERNALS::apply_for_each_target halt
    } else {
        halt
    }
    _SC_INTERNALS::sc_lib_print "all targets halted"
}

proc sc_fpga_resume_all { { addr "" } } {
    # TODO: take into account cases when several SMP groups are present or
    # there are targets that does not belong to any SMP group
    if {[string trim [smp]] eq "off"} {
        _SC_INTERNALS::apply_for_each_target _SC_INTERNALS::sc_lib_do_resume $addr
    } else {
        _SC_INTERNALS::sc_lib_do_resume $addr
    }
    if { $addr eq "" } {
        _SC_INTERNALS::sc_lib_print "all targets resumed"
    } else {
        _SC_INTERNALS::sc_lib_print "all targets resumed at $addr"
    }
}

proc sc_fpga_step_all { { addr "" } } {
    if { $addr eq "" } {
        _SC_INTERNALS::apply_for_each_target step
        _SC_INTERNALS::sc_lib_print "done stepping all harts"
    } else {
        _SC_INTERNALS::apply_for_each_target step $addr
        _SC_INTERNALS::sc_lib_print "done stepping all harts at [format 0x%08x $addr]"
    }
}

proc sc_fpga_zero_regs {} {
    _SC_INTERNALS::apply_for_each_target _SC_INTERNALS::fill_gprs_with_zero
    _SC_INTERNALS::sc_lib_print "all general-purpose registers are zero-out"
}

proc sc_fpga_setup_semihosting { {what ""} } {
    # TODO: not the best design... may be changed in future
    riscv set_ebreakm on
    riscv set_ebreaks on
    riscv set_ebreaku on

    _SC_INTERNALS::sc_lib_print "NOTE: dcsr.ebreak{m,s,u} are set to ON"

    if {$what eq "" || $what eq "enable" } {
        return [arm semihosting enable]
    } elseif {$what eq "stop" || $what eq "disable" } {
        return [arm semihosting disable]
    } else {
        return -code error "unknown operation $what passed to semihosting setup"
    }
}

proc sc_fpga_find_target_by_hartid { hartid } {
    set current_target [target current]
    foreach t [target names] {
        targets $t
        if {[catch { _SC_INTERNALS::sc_lib_read_reg_hex mhartid } mhartid]} {
            targets $current_target
            return -code error "could not read mhartid from $t ($mhartid)"
        }
        set decimal_val [expr $mhartid]
        if { $decimal_val == $hartid } {
            _SC_INTERNALS::sc_lib_print "$t has mhartid of $hartid"
            targets $current_target
            return $t
        }
    }
    targets $current_target
    return -code error "could not find target with mhartid = $hartid"
}

proc sc_fpga_run_elf { file_path entry_point { tgt ""} } {
    set entry_point \
        [_SC_INTERNALS::sc_lib_run_impl $file_path "" "" $entry_point one $tgt]
    _SC_INTERNALS::sc_lib_print \
        "started execution of $file_path with $entry_point as entry point"
}

proc sc_fpga_run_elf_smp { file_path entry_point { tgt ""} } {
    set entry_point \
        [_SC_INTERNALS::sc_lib_run_impl $file_path "" "" $entry_point all $tgt]
    _SC_INTERNALS::sc_lib_print \
        "started execution of $file_path with $entry_point as entry point"
}

proc sc_fpga_run_bin { file_path load_address {entry_point ""} { tgt ""} } {
    set entry_point \
        [_SC_INTERNALS::sc_lib_run_impl $file_path bin $load_address $entry_point one $tgt]
    _SC_INTERNALS::sc_lib_print \
        "started execution of $file_path with $entry_point as entry point"
}

proc sc_fpga_run_bin_smp { file_path load_address {entry_point ""} { tgt ""} } {
    set entry_point \
        [_SC_INTERNALS::sc_lib_run_impl $file_path bin $load_address $entry_point all $tgt]
    _SC_INTERNALS::sc_lib_print \
        "started execution of $file_path with $entry_point as entry point"
}

proc sc_fpga_info {} {
    return [join \
        [list \
            "Version: [version]" \
            "Number of harts: [llength [target names]]" \
            "SMP status: [string trim [smp]]" \
        ] "\n"]
}

proc sc_fpga_get_busy_duration {} {
    return ${::_SC_INTERNALS::FPGA_LIB_BUSY_DURATION}
}

proc sc_fpga_set_busy_duration {value} {
    set ::_SC_INTERNALS::FPGA_LIB_BUSY_DURATION $value
}

proc sc_fpga_riscv_csr_read {csr_num {xlen 64}} {
    _SC_INTERNALS::sc_lib_require_halted_and_check_csr $csr_num

    _SC_INTERNALS::sc_lib_riscv_csr_impl $csr_num 0 0 $xlen
    set DATA_0 0x04
    set DATA_1 0x05
    if {$xlen == 32} {
        return [format 0x%.8x [expr {[riscv dmi_read $DATA_0]}]]
    }
    if {$xlen == 64} {
        return [format 0x%.16x [expr {[riscv dmi_read $DATA_0] | [riscv dmi_read $DATA_1]<<32}]]
    }
    return -code error "unsupported architecture size"
}

proc sc_fpga_riscv_csr_write {csr_num value {xlen 64}} {
    _SC_INTERNALS::sc_lib_require_halted_and_check_csr $csr_num

    set DATA_0 0x04
    set DATA_1 0x05
    if {$xlen == 32} {
        riscv dmi_write $DATA_0 $value
    }
    if {$xlen == 64} {
        riscv dmi_write $DATA_0 [expr {$value & 0xFFFFFFFF}]
        riscv dmi_write $DATA_1 [expr {$value >> 32}]
    }

    _SC_INTERNALS::sc_lib_riscv_csr_impl $csr_num $value 1 $xlen
}

## @return value of a counter corresponding to the specified event
## @param[in] "context" object returned by sc_experimental_pmu_setup
## @param[in] name of PMU event
##
## NOTE/TODO: DO NOT use this function on 32-bit targets
proc sc_experimental_pmu_get { ctx pmu_ctr } {
    _SC_INTERNALS::sc_lib_require_halted
    if {$pmu_ctr eq "CY"} {
        return [sc_fpga_read_reg mcycle]
    }
    if {$pmu_ctr eq "TIME"} {
        return [sc_fpga_read_reg time]
    }
    if {$pmu_ctr eq "IR"} {
        return [sc_fpga_read_reg minstret]
    }
    set pmu_ctr_idx [lsearch -nocase $ctx $pmu_ctr]
    if {$pmu_ctr_idx == -1} {
        error "could not find $pmu_ctr in the pmu context"
    }
    set reg_idx [expr { 3 + $pmu_ctr_idx } ]
    set counter_reg_name mhpmcounter${reg_idx}
    _SC_INTERNALS::sc_lib_print "reading $counter_reg_name as $pmu_ctr counter"
    set result [sc_fpga_read_reg $counter_reg_name]
    _SC_INTERNALS::sc_lib_print "done"
    return $result
}

## @return "context" object. This object should be passed to
##       sc_experimental_pmu_get, to read PMU counter
## @param[in] dictionary representing supported PMU counters and corresponding
##       selectors.
## @param[in] maximum number of PMU counters implemented by target
##
## NOTE/TODO: DO NOT use this function on 32-bit targets
## TODO: we may want to implement S/U/M-mode filtering
proc sc_experimental_pmu_setup { pmu_selectors pmu_ctrs_max args } {
    set pmu_ctrs_max [expr { $pmu_ctrs_max + 0}]
    if {$pmu_ctrs_max > 32} {
        error "maximum number of PMU registers should not be greater than 32 ($pmu_ctrs_max)!"
    }
    set cy_enable 0
    set ir_enable 0
    set pmu_ctrs [list]
    set counters $args
    foreach counter $counters {
        if {[dict exists $pmu_selectors $counter]} {
            lappend pmu_ctrs $counter
            continue
        }
        if { $counter eq "CY" } {
            set cy_enable 1
            continue
        }
        if {$counter eq "IR" } {
            set ir_enable 1
            continue
        }
        error "unsupported PMU counter: $counter"
    }
    set pmu_ctrs [lsort -unique $pmu_ctrs]
    if {[llength $pmu_ctrs] > $pmu_ctrs_max} {
        error "too many PMU counters requested was requested (max is $pmu_ctrs_max)"
    }
    _SC_INTERNALS::sc_lib_require_halted
    _SC_INTERNALS::sc_lib_experimental_reset_pmu_subsystem $pmu_ctrs_max
    _SC_INTERNALS::sc_lib_experimental_enable_pmu_counters \
        $pmu_selectors $pmu_ctrs_max $cy_enable $ir_enable $pmu_ctrs
    _SC_INTERNALS::sc_lib_print "PMU counters { $pmu_ctrs } (CY: $cy_enable, IR: $ir_enable) configured!"
    return $pmu_ctrs
}

proc sc_experimental_sync_resume args {
    if {[llength $args] != 0} {
        set targets $args
    } else {
        set targets [target names]
    }

    set taps {}
    foreach target $targets {
        lappend taps [$target cget -chain-position]
    }

    if {[llength [lsort -unique $taps]] != [llength $taps]} {
        error "Only the case of one TAP per target is supported."
    }

    set op_len 2
    set op_write 2
    set data_len 32
    # set "dmactive" and "resumereq"
    set dmcontrol_val [expr { 1 | 1 << 30 }]
    set data_hex 0x[format %x $dmcontrol_val]
    set dmcontrol_addr 0x10
    set addr_hex 0x[format %x $dmcontrol_addr]

    set ir_list {}
    set dr_list {}
    foreach tap $taps target $targets {
        array set rvi [$target riscv info]
        set addr_len [lindex [array get rvi dm.abits] 1]
        lappend ir_list 0x11
        lappend dr_list \
            ${op_len}:${op_write},${data_len}:${data_hex},$addr_len:${addr_hex}
    }
    # FIXME: RISC-V targets update "ebreak*" by halting the hart on poll.
    poll disable
    jtag execute scan {*}$taps -ir {*}$ir_list -dr {*}$dr_list
}

echo "--- LOADED SC FPGA LIBRARY ---"

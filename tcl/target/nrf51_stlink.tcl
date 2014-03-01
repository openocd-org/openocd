#
# script for Nordic nRF51 series, a CORTEX-M0 chip
#

source [find target/swj-dp.tcl]

if { [info exists CHIPNAME] } {
   set _CHIPNAME $CHIPNAME
} else {
   set _CHIPNAME nrf51
}

if { [info exists ENDIAN] } {
   set _ENDIAN $ENDIAN
} else {
   set _ENDIAN little
}

# Work-area is a space in RAM used for flash programming
# By default use 2kB
if { [info exists WORKAREASIZE] } {
   set _WORKAREASIZE $WORKAREASIZE
} else {
   set _WORKAREASIZE 0x800
}

if { [info exists CPUTAPID] } {
   set _CPUTAPID $CPUTAPID
} else {
   set _CPUTAPID 0x0bb11477
}

if { [info exists TRANSPORT] } {
   set _TRANSPORT $TRANSPORT
   if { $TRANSPORT == "hla_jtag" } {
      if { [info exists CPUTAPID] == 0 } {
         # jtag requires us to use the jtag tap id
         set _CPUTAPID 0x3ba00477
      }
   }
} else {
   set _TRANSPORT hla_swd
}

# add deprecated transport name check
if { $_TRANSPORT == "stlink_swd" } {
	set _TRANSPORT "hla_swd"
	echo "DEPRECATED! use 'hla_swd' transport not 'stlink_swd'"
}

if { $_TRANSPORT == "stlink_jtag" } {
	set _TRANSPORT "hla_jtag"
	echo "DEPRECATED! use 'hla_jtag' transport not 'stlink_jtag'"
}
# end deprecated checks

transport select $_TRANSPORT
hla newtap $_CHIPNAME cpu -expected-id $_CPUTAPID

set _TARGETNAME $_CHIPNAME.cpu
target create $_TARGETNAME hla_target -chain-position $_TARGETNAME

$_TARGETNAME configure -work-area-phys 0x20000000 -work-area-size $_WORKAREASIZE -work-area-backup 0

# The chip supports standard ARM/Cortex-M0 SYSRESETREQ signal, so for
# non-"hla" targets it would be useful to have the following in the config.
# cortex_m reset_config sysresetreq

flash bank $_CHIPNAME.flash nrf51 0x00000000 0 1 1 $_TARGETNAME
flash bank $_CHIPNAME.uicr nrf51 0x10001000 0 1 1 $_TARGETNAME

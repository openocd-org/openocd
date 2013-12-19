# ARM Debug Interface V5 (ADI_V5) utility
# ... Mostly for SWJ-DP (not SW-DP or JTAG-DP, since
# SW-DP and JTAG-DP targets don't need to switch based
# on which transport is active.
#
# declare a JTAG or SWD Debug Access Point (DAP)
# based on the transport in use with this session.
# You can't access JTAG ops when SWD is active, etc.

# params are currently what "jtag newtap" uses
# because OpenOCD internals are still strongly biased
# to JTAG ....  but for SWD, "irlen" etc are ignored,
# and the internals work differently

# for now, ignore non-JTAG and non-SWD transports
# (e.g. initial flash programming via SPI or UART)

# split out "chip" and "tag" so we can someday handle
# them more uniformly irlen too...)

global using_jtag
set using_jtag 1

proc swj_newdap {chip tag args} {
 global using_jtag
 set tran [transport select]
 if [string equal $tran "jtag"] { eval jtag newtap $chip $tag $args; set using_jtag 1 }
 if [string equal $tran "swd"] { eval swd newdap $chip $tag $args; set using_jtag 0 }
 if [string equal $tran "cmsis-dap"] { eval cmsis-dap newdap $chip $tag $args; set using_jtag 0 }
}

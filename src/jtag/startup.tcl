# Defines basic Tcl procs for OpenOCD JTAG module

# Executed during "init". Can be overridden
# by board/target/... scripts
proc jtag_init {} {
	if {[catch {jtag arp_init} err]!=0} {
		# try resetting additionally
		init_reset startup
	}
}

# This reset logic may be overridden by board/target/... scripts as needed
# to provide a reset that, if possible, is close to a power-up reset.
#
# Exit requirements include:  (a) JTAG must be working, (b) the scan
# chain was validated with "jtag arp_init" (or equivalent), (c) nothing
# stays in reset.  No TAP-specific scans were performed.  It's OK if
# some targets haven't been reset yet; they may need TAP-specific scans.
#
# The "mode" values include:  halt, init, run (from "reset" command);
# startup (at OpenOCD server startup, when JTAG may not yet work); and
# potentially more (for reset types like cold, warm, etc)
proc init_reset { mode } {
	if {[using_jtag]} {
		jtag arp_init-reset
	}
}

#########

# TODO: power_restore and power_dropout are currently neither
# documented nor supported.

proc power_restore {} {
	echo "Sensed power restore, running reset init and halting GDB."
	reset init

	# Halt GDB so user can deal with a detected power restore.
	#
	# After GDB is halted, then output is no longer forwarded
	# to the GDB console.
	set targets [target names]
	foreach t $targets {
		# New event script.
		$t invoke-event arp_halt_gdb
	}
}

add_help_text power_restore "Overridable procedure run when power restore is detected. Runs 'reset init' by default."

proc power_dropout {} {
	echo "Sensed power dropout."
}

#########

# TODO: srst_deasserted and srst_asserted are currently neither
# documented nor supported.

proc srst_deasserted {} {
	echo "Sensed nSRST deasserted, running reset init and halting GDB."
	reset init

	# Halt GDB so user can deal with a detected reset.
	#
	# After GDB is halted, then output is no longer forwarded
	# to the GDB console.
	set targets [target names]
	foreach t $targets {
		# New event script.
		$t invoke-event arp_halt_gdb
	}
}

add_help_text srst_deasserted "Overridable procedure run when srst deassert is detected. Runs 'reset init' by default."

proc srst_asserted {} {
	echo "Sensed nSRST asserted."
}

# measure actual JTAG clock
proc measure_clk {} {
	set start_time [ms];
	set iterations 10000000;
	runtest $iterations;
	set speed [expr "$iterations.0 / ([ms] - $start_time)"]
	echo "Running at more than $speed kHz";
}

add_help_text measure_clk "Runs a test to measure the JTAG clk. Useful with RCLK / RTCK."

proc default_to_jtag { f args } {
	set current_transport [transport select]
	if {[using_jtag]} {
		eval $f $args
	} {
		error "session transport is \"$current_transport\" but your config requires JTAG"
	}
}

proc jtag args {
	eval default_to_jtag jtag $args
}

proc jtag_rclk args {
	eval default_to_jtag jtag_rclk $args
}

proc jtag_ntrst_delay args {
	eval default_to_jtag jtag_ntrst_delay $args
}

proc jtag_ntrst_assert_width args {
	eval default_to_jtag jtag_ntrst_assert_width $args
}

# BEGIN MIGRATION AIDS ...  these adapter operations originally had
# JTAG-specific names despite the fact that the operations were not
# specific to JTAG, or otherwise had troublesome/misleading names.
#
# FIXME phase these aids out after some releases
#
lappend _telnet_autocomplete_skip jtag_reset
proc jtag_reset args {
	echo "DEPRECATED! use 'adapter \[de\]assert' not 'jtag_reset'"
	switch $args {
		"0 0"
			{eval adapter deassert trst deassert srst}
		"0 1"
			{eval adapter deassert trst assert srst}
		"1 0"
			{eval adapter assert trst deassert srst}
		"1 1"
			{eval adapter assert trst assert srst}
		default
			{return -code 1 -level 1 "jtag_reset: syntax error"}
	}
}

lappend _telnet_autocomplete_skip adapter_khz
proc adapter_khz args {
	echo "DEPRECATED! use 'adapter speed' not 'adapter_khz'"
	eval adapter speed $args
}

lappend _telnet_autocomplete_skip adapter_name
proc adapter_name args {
	echo "DEPRECATED! use 'adapter name' not 'adapter_name'"
	eval adapter name $args
}

lappend _telnet_autocomplete_skip adapter_nsrst_delay
proc adapter_nsrst_delay args {
	echo "DEPRECATED! use 'adapter srst delay' not 'adapter_nsrst_delay'"
	eval adapter srst delay $args
}

lappend _telnet_autocomplete_skip adapter_nsrst_assert_width
proc adapter_nsrst_assert_width args {
	echo "DEPRECATED! use 'adapter srst pulse_width' not 'adapter_nsrst_assert_width'"
	eval adapter srst pulse_width $args
}

lappend _telnet_autocomplete_skip interface
proc interface args {
	echo "DEPRECATED! use 'adapter driver' not 'interface'"
	eval adapter driver $args
}

lappend _telnet_autocomplete_skip interface_transports
proc  interface_transports args {
	echo "DEPRECATED! use 'adapter transports' not 'interface_transports'"
	eval adapter transports $args
}

lappend _telnet_autocomplete_skip interface_list
proc  interface_list args {
	echo "DEPRECATED! use 'adapter list' not 'interface_list'"
	eval adapter list $args
}

lappend _telnet_autocomplete_skip ftdi_location
proc ftdi_location args {
	echo "DEPRECATED! use 'adapter usb location' not 'ftdi_location'"
	eval adapter usb location $args
}

lappend _telnet_autocomplete_skip xds110_serial
proc xds110_serial args {
	echo "DEPRECATED! use 'adapter serial' not 'xds110_serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip xds110_supply_voltage
proc xds110_supply_voltage args {
	echo "DEPRECATED! use 'xds110 supply' not 'xds110_supply_voltage'"
	eval xds110 supply $args
}

proc hla {cmd args} {
        tailcall "hla $cmd" {*}$args
}

lappend _telnet_autocomplete_skip "hla newtap"
proc "hla newtap" {args} {
	echo "DEPRECATED! use 'swj_newdap' not 'hla newtap'"
	eval swj_newdap $args
}

lappend _telnet_autocomplete_skip ftdi_device_desc
proc ftdi_device_desc args {
	echo "DEPRECATED! use 'ftdi device_desc' not 'ftdi_device_desc'"
	eval ftdi device_desc $args
}

lappend _telnet_autocomplete_skip ftdi_serial
proc ftdi_serial args {
	echo "DEPRECATED! use 'adapter serial' not 'ftdi_serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip ftdi_channel
proc ftdi_channel args {
	echo "DEPRECATED! use 'ftdi channel' not 'ftdi_channel'"
	eval ftdi channel $args
}

lappend _telnet_autocomplete_skip ftdi_layout_init
proc ftdi_layout_init args {
	echo "DEPRECATED! use 'ftdi layout_init' not 'ftdi_layout_init'"
	eval ftdi layout_init $args
}

lappend _telnet_autocomplete_skip ftdi_layout_signal
proc ftdi_layout_signal args {
	echo "DEPRECATED! use 'ftdi layout_signal' not 'ftdi_layout_signal'"
	eval ftdi layout_signal $args
}

lappend _telnet_autocomplete_skip ftdi_set_signal
proc ftdi_set_signal args {
	echo "DEPRECATED! use 'ftdi set_signal' not 'ftdi_set_signal'"
	eval ftdi set_signal $args
}

lappend _telnet_autocomplete_skip ftdi_get_signal
proc ftdi_get_signal args {
	echo "DEPRECATED! use 'ftdi get_signal' not 'ftdi_get_signal'"
	eval ftdi get_signal $args
}

lappend _telnet_autocomplete_skip ftdi_vid_pid
proc ftdi_vid_pid args {
	echo "DEPRECATED! use 'ftdi vid_pid' not 'ftdi_vid_pid'"
	eval ftdi vid_pid $args
}

lappend _telnet_autocomplete_skip ftdi_tdo_sample_edge
proc ftdi_tdo_sample_edge args {
	echo "DEPRECATED! use 'ftdi tdo_sample_edge' not 'ftdi_tdo_sample_edge'"
	eval ftdi tdo_sample_edge $args
}

lappend _telnet_autocomplete_skip remote_bitbang_host
proc remote_bitbang_host args {
	echo "DEPRECATED! use 'remote_bitbang host' not 'remote_bitbang_host'"
	eval remote_bitbang host $args
}

lappend _telnet_autocomplete_skip remote_bitbang_port
proc remote_bitbang_port args {
	echo "DEPRECATED! use 'remote_bitbang port' not 'remote_bitbang_port'"
	eval remote_bitbang port $args
}

lappend _telnet_autocomplete_skip openjtag_device_desc
proc openjtag_device_desc args {
	echo "DEPRECATED! use 'openjtag device_desc' not 'openjtag_device_desc'"
	eval openjtag device_desc $args
}

lappend _telnet_autocomplete_skip openjtag_variant
proc openjtag_variant args {
	echo "DEPRECATED! use 'openjtag variant' not 'openjtag_variant'"
	eval openjtag variant $args
}

lappend _telnet_autocomplete_skip parport_port
proc parport_port args {
	echo "DEPRECATED! use 'parport port' not 'parport_port'"
	eval parport port $args
}

lappend _telnet_autocomplete_skip parport_cable
proc parport_cable args {
	echo "DEPRECATED! use 'parport cable' not 'parport_cable'"
	eval parport cable $args
}

lappend _telnet_autocomplete_skip parport_write_on_exit
proc parport_write_on_exit args {
	echo "DEPRECATED! use 'parport write_on_exit' not 'parport_write_on_exit'"
	eval parport write_on_exit $args
}

lappend _telnet_autocomplete_skip parport_toggling_time
proc parport_toggling_time args {
	echo "DEPRECATED! use 'parport toggling_time' not 'parport_toggling_time'"
	eval parport toggling_time $args
}

lappend _telnet_autocomplete_skip jtag_dpi_set_port
proc jtag_dpi_set_port args {
	echo "DEPRECATED! use 'jtag_dpi set_port' not 'jtag_dpi_set_port'"
	eval jtag_dpi set_port $args
}

lappend _telnet_autocomplete_skip jtag_dpi_set_address
proc jtag_dpi_set_address args {
	echo "DEPRECATED! use 'jtag_dpi set_address' not 'jtag_dpi_set_address'"
	eval jtag_dpi set_address $args
}

lappend _telnet_autocomplete_skip jtag_vpi_set_port
proc jtag_vpi_set_port args {
	echo "DEPRECATED! use 'jtag_vpi set_port' not 'jtag_vpi_set_port'"
	eval jtag_vpi set_port $args
}

lappend _telnet_autocomplete_skip jtag_vpi_set_address
proc jtag_vpi_set_address args {
	echo "DEPRECATED! use 'jtag_vpi set_address' not 'jtag_vpi_set_address'"
	eval jtag_vpi set_address $args
}

lappend _telnet_autocomplete_skip jtag_vpi_stop_sim_on_exit
proc jtag_vpi_stop_sim_on_exit args {
	echo "DEPRECATED! use 'jtag_vpi stop_sim_on_exit' not 'jtag_vpi_stop_sim_on_exit'"
	eval jtag_vpi stop_sim_on_exit $args
}

lappend _telnet_autocomplete_skip presto_serial
proc presto_serial args {
	echo "DEPRECATED! use 'presto serial' not 'presto_serial'"
	eval presto serial $args
}

lappend _telnet_autocomplete_skip xlnx_pcie_xvc_config
proc xlnx_pcie_xvc_config args {
	echo "DEPRECATED! use 'xlnx_pcie_xvc config' not 'xlnx_pcie_xvc_config'"
	eval xlnx_pcie_xvc config $args
}

lappend _telnet_autocomplete_skip ulink_download_firmware
proc ulink_download_firmware args {
	echo "DEPRECATED! use 'ulink download_firmware' not 'ulink_download_firmware'"
	eval ulink download_firmware $args
}

lappend _telnet_autocomplete_skip vsllink_usb_vid
proc vsllink_usb_vid args {
	echo "DEPRECATED! use 'vsllink usb_vid' not 'vsllink_usb_vid'"
	eval vsllink usb_vid $args
}

lappend _telnet_autocomplete_skip vsllink_usb_pid
proc vsllink_usb_pid args {
	echo "DEPRECATED! use 'vsllink usb_pid' not 'vsllink_usb_pid'"
	eval vsllink usb_pid $args
}

lappend _telnet_autocomplete_skip vsllink_usb_serial
proc vsllink_usb_serial args {
	echo "DEPRECATED! use 'adapter serial' not 'vsllink_usb_serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip vsllink_usb_bulkin
proc vsllink_usb_bulkin args {
	echo "DEPRECATED! use 'vsllink usb_bulkin' not 'vsllink_usb_bulkin'"
	eval vsllink usb_bulkin $args
}

lappend _telnet_autocomplete_skip vsllink_usb_bulkout
proc vsllink_usb_bulkout args {
	echo "DEPRECATED! use 'vsllink usb_bulkout' not 'vsllink_usb_bulkout'"
	eval vsllink usb_bulkout $args
}

lappend _telnet_autocomplete_skip vsllink_usb_interface
proc vsllink_usb_interface args {
	echo "DEPRECATED! use 'vsllink usb_interface' not 'vsllink_usb_interface'"
	eval vsllink usb_interface $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_jtag_nums
proc bcm2835gpio_jtag_nums args {
	echo "DEPRECATED! use 'bcm2835gpio jtag_nums' not 'bcm2835gpio_jtag_nums'"
	eval bcm2835gpio jtag_nums $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_tck_num
proc bcm2835gpio_tck_num args {
	echo "DEPRECATED! use 'bcm2835gpio tck_num' not 'bcm2835gpio_tck_num'"
	eval bcm2835gpio tck_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_tms_num
proc bcm2835gpio_tms_num args {
	echo "DEPRECATED! use 'bcm2835gpio tms_num' not 'bcm2835gpio_tms_num'"
	eval bcm2835gpio tms_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_tdo_num
proc bcm2835gpio_tdo_num args {
	echo "DEPRECATED! use 'bcm2835gpio tdo_num' not 'bcm2835gpio_tdo_num'"
	eval bcm2835gpio tdo_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_tdi_num
proc bcm2835gpio_tdi_num args {
	echo "DEPRECATED! use 'bcm2835gpio tdi_num' not 'bcm2835gpio_tdi_num'"
	eval bcm2835gpio tdi_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_swd_nums
proc bcm2835gpio_swd_nums args {
	echo "DEPRECATED! use 'bcm2835gpio swd_nums' not 'bcm2835gpio_swd_nums'"
	eval bcm2835gpio swd_nums $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_swclk_num
proc bcm2835gpio_swclk_num args {
	echo "DEPRECATED! use 'bcm2835gpio swclk_num' not 'bcm2835gpio_swclk_num'"
	eval bcm2835gpio swclk_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_swdio_num
proc bcm2835gpio_swdio_num args {
	echo "DEPRECATED! use 'bcm2835gpio swdio_num' not 'bcm2835gpio_swdio_num'"
	eval bcm2835gpio swdio_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_swdio_dir_num
proc bcm2835gpio_swdio_dir_num args {
	echo "DEPRECATED! use 'bcm2835gpio swdio_dir_num' not 'bcm2835gpio_swdio_dir_num'"
	eval bcm2835gpio swdio_dir_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_srst_num
proc bcm2835gpio_srst_num args {
	echo "DEPRECATED! use 'bcm2835gpio srst_num' not 'bcm2835gpio_srst_num'"
	eval bcm2835gpio srst_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_trst_num
proc bcm2835gpio_trst_num args {
	echo "DEPRECATED! use 'bcm2835gpio trst_num' not 'bcm2835gpio_trst_num'"
	eval bcm2835gpio trst_num $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_speed_coeffs
proc bcm2835gpio_speed_coeffs args {
	echo "DEPRECATED! use 'bcm2835gpio speed_coeffs' not 'bcm2835gpio_speed_coeffs'"
	eval bcm2835gpio speed_coeffs $args
}

lappend _telnet_autocomplete_skip bcm2835gpio_peripheral_base
proc bcm2835gpio_peripheral_base args {
	echo "DEPRECATED! use 'bcm2835gpio peripheral_base' not 'bcm2835gpio_peripheral_base'"
	eval bcm2835gpio peripheral_base $args
}

lappend _telnet_autocomplete_skip linuxgpiod_jtag_nums
proc linuxgpiod_jtag_nums args {
	echo "DEPRECATED! use 'linuxgpiod jtag_nums' not 'linuxgpiod_jtag_nums'"
	eval linuxgpiod jtag_nums $args
}

lappend _telnet_autocomplete_skip linuxgpiod_tck_num
proc linuxgpiod_tck_num args {
	echo "DEPRECATED! use 'linuxgpiod tck_num' not 'linuxgpiod_tck_num'"
	eval linuxgpiod tck_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_tms_num
proc linuxgpiod_tms_num args {
	echo "DEPRECATED! use 'linuxgpiod tms_num' not 'linuxgpiod_tms_num'"
	eval linuxgpiod tms_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_tdo_num
proc linuxgpiod_tdo_num args {
	echo "DEPRECATED! use 'linuxgpiod tdo_num' not 'linuxgpiod_tdo_num'"
	eval linuxgpiod tdo_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_tdi_num
proc linuxgpiod_tdi_num args {
	echo "DEPRECATED! use 'linuxgpiod tdi_num' not 'linuxgpiod_tdi_num'"
	eval linuxgpiod tdi_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_srst_num
proc linuxgpiod_srst_num args {
	echo "DEPRECATED! use 'linuxgpiod srst_num' not 'linuxgpiod_srst_num'"
	eval linuxgpiod srst_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_trst_num
proc linuxgpiod_trst_num args {
	echo "DEPRECATED! use 'linuxgpiod trst_num' not 'linuxgpiod_trst_num'"
	eval linuxgpiod trst_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_swd_nums
proc linuxgpiod_swd_nums args {
	echo "DEPRECATED! use 'linuxgpiod swd_nums' not 'linuxgpiod_swd_nums'"
	eval linuxgpiod swd_nums $args
}

lappend _telnet_autocomplete_skip linuxgpiod_swclk_num
proc linuxgpiod_swclk_num args {
	echo "DEPRECATED! use 'linuxgpiod swclk_num' not 'linuxgpiod_swclk_num'"
	eval linuxgpiod swclk_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_swdio_num
proc linuxgpiod_swdio_num args {
	echo "DEPRECATED! use 'linuxgpiod swdio_num' not 'linuxgpiod_swdio_num'"
	eval linuxgpiod swdio_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_led_num
proc linuxgpiod_led_num args {
	echo "DEPRECATED! use 'linuxgpiod led_num' not 'linuxgpiod_led_num'"
	eval linuxgpiod led_num $args
}

lappend _telnet_autocomplete_skip linuxgpiod_gpiochip
proc linuxgpiod_gpiochip args {
	echo "DEPRECATED! use 'linuxgpiod gpiochip' not 'linuxgpiod_gpiochip'"
	eval linuxgpiod gpiochip $args
}

lappend _telnet_autocomplete_skip sysfsgpio_jtag_nums
proc sysfsgpio_jtag_nums args {
	echo "DEPRECATED! use 'sysfsgpio jtag_nums' not 'sysfsgpio_jtag_nums'"
	eval sysfsgpio jtag_nums $args
}

lappend _telnet_autocomplete_skip sysfsgpio_tck_num
proc sysfsgpio_tck_num args {
	echo "DEPRECATED! use 'sysfsgpio tck_num' not 'sysfsgpio_tck_num'"
	eval sysfsgpio tck_num $args
}

lappend _telnet_autocomplete_skip sysfsgpio_tms_num
proc sysfsgpio_tms_num args {
	echo "DEPRECATED! use 'sysfsgpio tms_num' not 'sysfsgpio_tms_num'"
	eval sysfsgpio tms_num $args
}

lappend _telnet_autocomplete_skip sysfsgpio_tdo_num
proc sysfsgpio_tdo_num args {
	echo "DEPRECATED! use 'sysfsgpio tdo_num' not 'sysfsgpio_tdo_num'"
	eval sysfsgpio tdo_num $args
}

lappend _telnet_autocomplete_skip sysfsgpio_tdi_num
proc sysfsgpio_tdi_num args {
	echo "DEPRECATED! use 'sysfsgpio tdi_num' not 'sysfsgpio_tdi_num'"
	eval sysfsgpio tdi_num $args
}

lappend _telnet_autocomplete_skip sysfsgpio_srst_num
proc sysfsgpio_srst_num args {
	echo "DEPRECATED! use 'sysfsgpio srst_num' not 'sysfsgpio_srst_num'"
	eval sysfsgpio srst_num $args
}

lappend _telnet_autocomplete_skip sysfsgpio_trst_num
proc sysfsgpio_trst_num args {
	echo "DEPRECATED! use 'sysfsgpio trst_num' not 'sysfsgpio_trst_num'"
	eval sysfsgpio trst_num $args
}

lappend _telnet_autocomplete_skip sysfsgpio_swd_nums
proc sysfsgpio_swd_nums args {
	echo "DEPRECATED! use 'sysfsgpio swd_nums' not 'sysfsgpio_swd_nums'"
	eval sysfsgpio swd_nums $args
}

lappend _telnet_autocomplete_skip sysfsgpio_swclk_num
proc sysfsgpio_swclk_num args {
	echo "DEPRECATED! use 'sysfsgpio swclk_num' not 'sysfsgpio_swclk_num'"
	eval sysfsgpio swclk_num $args
}

lappend _telnet_autocomplete_skip sysfsgpio_swdio_num
proc sysfsgpio_swdio_num args {
	echo "DEPRECATED! use 'sysfsgpio swdio_num' not 'sysfsgpio_swdio_num'"
	eval sysfsgpio swdio_num $args
}

lappend _telnet_autocomplete_skip buspirate_adc
proc buspirate_adc args {
	echo "DEPRECATED! use 'buspirate adc' not 'buspirate_adc'"
	eval buspirate adc $args
}

lappend _telnet_autocomplete_skip buspirate_vreg
proc buspirate_vreg args {
	echo "DEPRECATED! use 'buspirate vreg' not 'buspirate_vreg'"
	eval buspirate vreg $args
}

lappend _telnet_autocomplete_skip buspirate_pullup
proc buspirate_pullup args {
	echo "DEPRECATED! use 'buspirate pullup' not 'buspirate_pullup'"
	eval buspirate pullup $args
}

lappend _telnet_autocomplete_skip buspirate_led
proc buspirate_led args {
	echo "DEPRECATED! use 'buspirate led' not 'buspirate_led'"
	eval buspirate led $args
}

lappend _telnet_autocomplete_skip buspirate_speed
proc buspirate_speed args {
	echo "DEPRECATED! use 'buspirate speed' not 'buspirate_speed'"
	eval buspirate speed $args
}

lappend _telnet_autocomplete_skip buspirate_mode
proc buspirate_mode args {
	echo "DEPRECATED! use 'buspirate mode' not 'buspirate_mode'"
	eval buspirate mode $args
}

lappend _telnet_autocomplete_skip buspirate_port
proc buspirate_port args {
	echo "DEPRECATED! use 'buspirate port' not 'buspirate_port'"
	eval buspirate port $args
}

lappend _telnet_autocomplete_skip usb_blaster_device_desc
proc usb_blaster_device_desc args {
	echo "DEPRECATED! use 'usb_blaster device_desc' not 'usb_blaster_device_desc'"
	eval usb_blaster device_desc $args
}

lappend _telnet_autocomplete_skip usb_blaster_vid_pid
proc usb_blaster_vid_pid args {
	echo "DEPRECATED! use 'usb_blaster vid_pid' not 'usb_blaster_vid_pid'"
	eval usb_blaster vid_pid $args
}

lappend _telnet_autocomplete_skip usb_blaster_lowlevel_driver
proc usb_blaster_lowlevel_driver args {
	echo "DEPRECATED! use 'usb_blaster lowlevel_driver' not 'usb_blaster_lowlevel_driver'"
	eval usb_blaster lowlevel_driver $args
}

lappend _telnet_autocomplete_skip usb_blaster_pin
proc usb_blaster_pin args {
	echo "DEPRECATED! use 'usb_blaster pin' not 'usb_blaster_pin'"
	eval usb_blaster pin $args
}

lappend _telnet_autocomplete_skip usb_blaster_firmware
proc usb_blaster_firmware args {
	echo "DEPRECATED! use 'usb_blaster firmware' not 'usb_blaster_firmware'"
	eval usb_blaster firmware $args
}

lappend _telnet_autocomplete_skip ft232r_serial_desc
proc ft232r_serial_desc args {
	echo "DEPRECATED! use 'adapter serial_desc' not 'ft232r_serial_desc'"
	eval adapter serial_desc $args
}

lappend _telnet_autocomplete_skip ft232r_vid_pid
proc ft232r_vid_pid args {
	echo "DEPRECATED! use 'ft232r vid_pid' not 'ft232r_vid_pid'"
	eval ft232r vid_pid $args
}

lappend _telnet_autocomplete_skip ft232r_jtag_nums
proc ft232r_jtag_nums args {
	echo "DEPRECATED! use 'ft232r jtag_nums' not 'ft232r_jtag_nums'"
	eval ft232r jtag_nums $args
}

lappend _telnet_autocomplete_skip ft232r_tck_num
proc ft232r_tck_num args {
	echo "DEPRECATED! use 'ft232r tck_num' not 'ft232r_tck_num'"
	eval ft232r tck_num $args
}

lappend _telnet_autocomplete_skip ft232r_tms_num
proc ft232r_tms_num args {
	echo "DEPRECATED! use 'ft232r tms_num' not 'ft232r_tms_num'"
	eval ft232r tms_num $args
}

lappend _telnet_autocomplete_skip ft232r_tdo_num
proc ft232r_tdo_num args {
	echo "DEPRECATED! use 'ft232r tdo_num' not 'ft232r_tdo_num'"
	eval ft232r tdo_num $args
}

lappend _telnet_autocomplete_skip ft232r_tdi_num
proc ft232r_tdi_num args {
	echo "DEPRECATED! use 'ft232r tdi_num' not 'ft232r_tdi_num'"
	eval ft232r tdi_num $args
}

lappend _telnet_autocomplete_skip ft232r_srst_num
proc ft232r_srst_num args {
	echo "DEPRECATED! use 'ft232r srst_num' not 'ft232r_srst_num'"
	eval ft232r srst_num $args
}

lappend _telnet_autocomplete_skip ft232r_trst_num
proc ft232r_trst_num args {
	echo "DEPRECATED! use 'ft232r trst_num' not 'ft232r_trst_num'"
	eval ft232r trst_num $args
}

lappend _telnet_autocomplete_skip ft232r_restore_serial
proc ft232r_restore_serial args {
	echo "DEPRECATED! use 'ft232r restore_serial' not 'ft232r_restore_serial'"
	eval ft232r restore_serial $args
}

lappend _telnet_autocomplete_skip "aice serial"
proc "aice serial" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'aice serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip cmsis_dap_serial
proc cmsis_dap_serial args {
	echo "DEPRECATED! use 'adapter serial' not 'cmsis_dap_serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip "ft232r serial_desc"
proc "ft232r serial_desc" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'ft232r serial_desc'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip "ftdi serial"
proc "ftdi serial" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'ftdi serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip hla_serial
proc hla_serial args {
	echo "DEPRECATED! use 'adapter serial' not 'hla_serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip "jlink serial"
proc "jlink serial" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'jlink serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip kitprog_serial
proc kitprog_serial args {
	echo "DEPRECATED! use 'adapter serial' not 'kitprog_serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip "presto serial"
proc "presto serial" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'presto serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip "st-link serial"
proc "st-link serial" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'st-link serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip "vsllink usb_serial"
proc "vsllink usb_serial" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'vsllink usb_serial'"
	eval adapter serial $args
}

lappend _telnet_autocomplete_skip "xds110 serial"
proc "xds110 serial" {args} {
	echo "DEPRECATED! use 'adapter serial' not 'xds110 serial'"
	eval adapter serial $args
}

# END MIGRATION AIDS

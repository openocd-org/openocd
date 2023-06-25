# SPDX-License-Identifier: GPL-2.0-or-later

# Copyright (c) 2025, Syntacore LLC

if BUILD_USB_RESET
bin_PROGRAMS = usb_reset
usb_reset_SOURCES = debug_adapter_reset.c
endif

bin_SCRIPTS = openocd_run.exp

cfgdir = $(pkgdatadir)/scripts/syntacore
cfg_DATA = syntacore_ide_csr_cloak.cfg

syntacore_ide_csr_cloak.cfg: ide_cloak.tcl
	$(abspath $<) $@

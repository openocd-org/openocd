
if OOCD_TRACE
OOCD_TRACE_FILES = oocd_trace.c
else
OOCD_TRACE_FILES =
endif

INCLUDES = -I$(top_srcdir)/src/gdb -I$(top_srcdir)/src/helper 	-I$(top_srcdir)/src/jtag -I$(top_srcdir)/src/xsvf $(all_includes)
METASOURCES = AUTO
AM_CPPFLAGS = -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
noinst_LIBRARIES = libtarget.a
libtarget_a_SOURCES = target.c register.c breakpoints.c armv4_5.c embeddedice.c etm.c arm7tdmi.c arm9tdmi.c \
	arm_jtag.c arm7_9_common.c algorithm.c arm920t.c arm720t.c armv4_5_mmu.c armv4_5_cache.c arm_disassembler.c \
	arm966e.c arm926ejs.c feroceon.c etb.c xscale.c arm_simulator.c image.c armv7m.c cortex_m3.c cortex_swjdp.c \
	etm_dummy.c $(OOCD_TRACE_FILES) target_request.c trace.c arm11.c arm11_dbgtap.c
noinst_HEADERS = target.h trace.h register.h armv4_5.h embeddedice.h etm.h arm7tdmi.h arm9tdmi.h \
	arm_jtag.h arm7_9_common.h arm920t.h arm720t.h armv4_5_mmu.h armv4_5_cache.h breakpoints.h algorithm.h \
	arm_disassembler.h arm966e.h arm926ejs.h etb.h xscale.h arm_simulator.h image.h armv7m.h cortex_m3.h cortex_swjdp.h \
	etm_dummy.h oocd_trace.h target_request.h trace.h arm11.h

nobase_dist_pkglib_DATA = xscale/debug_handler.bin event/at91eb40a_reset.script target/at91eb40a.cfg \
	event/at91r40008_reset.script event/sam7s256_reset.script event/sam7x256_reset.script \
	target/at91r40008.cfg target/lpc2148.cfg target/lpc2294.cfg target/sam7s256.cfg \
	target/sam7x256.cfg target/str710.cfg target/str912.cfg target/nslu2.cfg target/pxa255_sst.cfg \
	target/pxa255.cfg  target/zy1000.cfg event/zy1000_reset.script event/at91sam9260_reset.script target/at91sam9260.cfg \
	target/wi-9c.cfg event/wi-9c_reset.script event/pxa255_reset.script target/stm32.cfg  target/xba_revA3.cfg event/xba_revA3.script \
	ecos/at91eb40a.elf target/lm3s6965.cfg interface/parport.cfg \
	event/omap5912_reset.script interface/jtagkey-tiny.cfg interface/jtagkey.cfg interface/str9-comstick.cfg \
	target/epc9301.cfg target/ipx42x.cfg target/lpc2129.cfg target/netx500.cfg \
	target/omap5912.cfg target/pxa270.cfg target/str750.cfg target/str9comstick.cfg \
	target/str730.cfg target/stm32stick.cfg event/str912_reset.script event/str710_program.script \
	target/lm3s811.cfg interface/luminary.cfg interface/luminary-lm3s811.cfg interface/stm32-stick.cfg \
	interface/calao-usb-a9260-c01.cfg interface/calao-usb-a9260-c02.cfg \
	interface/calao-usb-a9260.cfg target/at91sam9260minimal.cfg  event/lpc2148_reset.script \
	interface/chameleon.cfg interface/at91rm9200.cfg interface/jlink.cfg interface/arm-usb-ocd.cfg \
	interface/signalyzer.cfg event/eir-sam7se512_reset.script target/eir-sam7se512.cfg \
	event/hammer_reset.script interface/flyswatter.cfg target/hammer.cfg


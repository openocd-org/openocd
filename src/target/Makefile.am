
if OOCD_TRACE
OOCD_TRACE_FILES = oocd_trace.c
else
OOCD_TRACE_FILES =
endif

AM_CPPFLAGS = \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/jtag \
	-I$(top_srcdir)/src/xsvf

BIN2C		= $(top_builddir)/src/helper/bin2char$(EXEEXT_FOR_BUILD)
DEBUG_HANDLER	= $(top_srcdir)/src/target/xscale/debug_handler.bin

xscale_debug.h: $(BIN2C) $(DEBUG_HANDLER)
	$(BIN2C) < $(DEBUG_HANDLER) xscale_debug_handler > xscale_debug.h

METASOURCES = AUTO
noinst_LTLIBRARIES = libtarget.la
libtarget_la_SOURCES = \
	target.c \
	register.c \
	breakpoints.c \
	armv4_5.c \
	embeddedice.c \
	etm.c \
	arm7tdmi.c \
	arm9tdmi.c \
	arm_jtag.c \
	arm7_9_common.c \
	algorithm.c \
	arm920t.c \
	arm720t.c \
	armv4_5_mmu.c \
	armv4_5_cache.c \
	arm_disassembler.c \
	arm966e.c \
	arm926ejs.c \
	fa526.c \
	feroceon.c \
	etb.c \
	xscale.c \
	arm_simulator.c \
	image.c \
	armv7m.c \
	armv7a.c \
	cortex_m3.c \
	cortex_a8.c \
	arm_adi_v5.c \
	etm_dummy.c \
	$(OOCD_TRACE_FILES) \
	target_request.c \
	trace.c \
	arm11.c \
	arm11_dbgtap.c \
	mips32.c \
	mips_m4k.c \
	mips32_pracc.c \
	mips32_dmaacc.c \
	mips_ejtag.c \
	avrt.c

BUILT_SOURCES = \
	xscale_debug.h

noinst_HEADERS = \
	target.h \
	target_type.h \
	trace.h \
	register.h \
	armv4_5.h \
	embeddedice.h \
	etm.h \
	arm7tdmi.h \
	arm9tdmi.h \
	arm_jtag.h \
	arm7_9_common.h \
	arm920t.h \
	arm720t.h \
	armv4_5_mmu.h \
	armv4_5_cache.h \
	breakpoints.h \
	algorithm.h \
	arm_disassembler.h \
	arm966e.h \
	arm926ejs.h \
	etb.h \
	xscale.h \
	xscale_debug.h \
	arm_simulator.h \
	image.h \
	armv7m.h \
	armv7a.h \
	cortex_m3.h \
	cortex_a8.h \
	arm_adi_v5.h \
	etm_dummy.h \
	oocd_trace.h \
	target_request.h \
	trace.h \
	arm11.h \
	mips32.h \
	mips_m4k.h \
	mips_ejtag.h \
	mips32_pracc.h \
	mips32_dmaacc.h \
	avrt.h

nobase_dist_pkglib_DATA =
nobase_dist_pkglib_DATA += ecos/at91eb40a.elf

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

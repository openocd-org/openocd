
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
	etm_dummy.c $(OOCD_TRACE_FILES) target_request.c trace.c arm11.c arm11_dbgtap.c mips32.c mips_m4k.c \
	mips32_pracc.c mips32_dmaacc.c mips_ejtag.c
noinst_HEADERS = target.h trace.h register.h armv4_5.h embeddedice.h etm.h arm7tdmi.h arm9tdmi.h \
	arm_jtag.h arm7_9_common.h arm920t.h arm720t.h armv4_5_mmu.h armv4_5_cache.h breakpoints.h algorithm.h \
	arm_disassembler.h arm966e.h arm926ejs.h etb.h xscale.h arm_simulator.h image.h armv7m.h cortex_m3.h cortex_swjdp.h \
	etm_dummy.h oocd_trace.h target_request.h trace.h arm11.h mips32.h mips_m4k.h mips_ejtag.h mips32_pracc.h mips32_dmaacc.h

nobase_dist_pkglib_DATA =
nobase_dist_pkglib_DATA += xscale/debug_handler.bin 
nobase_dist_pkglib_DATA += ecos/at91eb40a.elf
# Various chip targets
nobase_dist_pkglib_DATA += $(wildcard $(srcdir)/target/*.cfg)
# Various jtag interfaces
nobase_dist_pkglib_DATA += $(wildcard $(srcdir)/interface/*.cfg)
# Various preconfigured boards
nobase_dist_pkglib_DATA += $(wildcard $(srcdir)/board/*.cfg)

# test files
nobase_dist_pkglib_DATA += $(wildcard $(srcdir)/test/*.cfg)

MAINTAINERCLEANFILES = Makefile.in

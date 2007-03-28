INCLUDES = -I$(top_srcdir)/src/gdb -I$(top_srcdir)/src/helper 	-I$(top_srcdir)/src/jtag -I$(top_srcdir)/src/xsvf $(all_includes)
METASOURCES = AUTO
noinst_LIBRARIES = libtarget.a
libtarget_a_SOURCES = target.c register.c breakpoints.c armv4_5.c embeddedice.c etm.c arm7tdmi.c arm9tdmi.c \
	arm_jtag.c arm7_9_common.c algorithm.c arm920t.c arm720t.c armv4_5_mmu.c armv4_5_cache.c arm_disassembler.c \
	arm966e.c arm926ejs.c etb.c xscale.c arm_simulator.c
noinst_HEADERS = target.h register.h armv4_5.h embeddedice.h etm.h arm7tdmi.h arm9tdmi.h \
		arm_jtag.h arm7_9_common.h arm920t.h arm720t.h armv4_5_mmu.h armv4_5_cache.h breakpoints.h algorithm.h \
		arm_disassembler.h arm966e.h arm926ejs.h etb.h xscale.h arm_simulator.h

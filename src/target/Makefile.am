include $(top_srcdir)/common.mk

if OOCD_TRACE
OOCD_TRACE_FILES = oocd_trace.c
else
OOCD_TRACE_FILES =
endif

SUBDIRS = openrisc
libtarget_la_LIBADD = $(top_builddir)/src/target/openrisc/libopenrisc.la

BIN2C		= $(top_builddir)/src/helper/bin2char$(EXEEXT_FOR_BUILD)

DEBUG_HANDLER	= $(srcdir)/xscale/debug_handler.bin
EXTRA_DIST = \
	startup.tcl \
	$(wildcard $(srcdir)/xscale/*)

DEBUG_HEADER	= xscale_debug.h
BUILT_SOURCES = $(DEBUG_HEADER)
CLEANFILES = $(DEBUG_HEADER)

$(DEBUG_HEADER): $(BIN2C) $(DEBUG_HANDLER)
	$(BIN2C) < $(DEBUG_HANDLER) xscale_debug_handler > xscale_debug.h

METASOURCES = AUTO
noinst_LTLIBRARIES = libtarget.la
libtarget_la_SOURCES = \
	$(TARGET_CORE_SRC) \
	$(ARM_DEBUG_SRC) \
	$(ARMV4_5_SRC) \
	$(ARMV6_SRC) \
	$(ARMV7_SRC) \
	$(ARM_MISC_SRC) \
	$(AVR32_SRC) \
	$(MIPS32_SRC) \
	$(NDS32_SRC) \
	$(INTEL_IA32_SRC) \
	avrt.c \
	dsp563xx.c \
	dsp563xx_once.c \
	dsp5680xx.c \
	hla_target.c

TARGET_CORE_SRC = \
	algorithm.c \
	register.c \
	image.c \
	breakpoints.c \
	target.c \
	target_request.c \
	testee.c \
	smp.c

ARMV4_5_SRC = \
	armv4_5.c \
	armv4_5_mmu.c \
	armv4_5_cache.c \
	$(ARM7_9_SRC)

ARM7_9_SRC = \
	arm7_9_common.c \
	arm7tdmi.c \
	arm720t.c \
	arm9tdmi.c \
	arm920t.c \
	arm966e.c \
	arm946e.c \
	arm926ejs.c \
	feroceon.c

ARM_MISC_SRC = \
	fa526.c \
	xscale.c

ARMV6_SRC = \
	arm11.c \
	arm11_dbgtap.c

ARMV7_SRC = \
	armv7m.c \
	cortex_m.c \
	armv7a.c \
	cortex_a.c

ARM_DEBUG_SRC = \
	arm_dpm.c \
	arm_jtag.c \
	arm_disassembler.c \
	arm_simulator.c \
	arm_semihosting.c \
	arm_adi_v5.c \
	adi_v5_jtag.c \
	adi_v5_swd.c \
	adi_v5_cmsis_dap.c \
	embeddedice.c \
	trace.c \
	etb.c \
	etm.c \
	$(OOCD_TRACE_FILES) \
	etm_dummy.c

AVR32_SRC = \
	avr32_ap7k.c \
	avr32_jtag.c \
	avr32_mem.c \
	avr32_regs.c

MIPS32_SRC = \
	mips32.c \
	mips_m4k.c \
	mips32_pracc.c \
	mips32_dmaacc.c \
	mips_ejtag.c

NDS32_SRC = \
	nds32.c \
	nds32_reg.c \
	nds32_cmd.c \
	nds32_disassembler.c \
	nds32_tlb.c \
	nds32_v2.c \
	nds32_v3_common.c \
	nds32_v3.c \
	nds32_v3m.c \
	nds32_aice.c

INTEL_IA32_SRC = \
	quark_x10xx.c \
	lakemont.c \
	x86_32_common.c

noinst_HEADERS = \
	algorithm.h \
	arm.h \
	arm_dpm.h \
	arm_jtag.h \
	arm_adi_v5.h \
	arm_disassembler.h \
	arm_opcodes.h \
	arm_simulator.h \
	arm_semihosting.h \
	arm7_9_common.h \
	arm7tdmi.h \
	arm720t.h \
	arm9tdmi.h \
	arm920t.h \
	arm926ejs.h \
	arm966e.h \
	arm946e.h \
	arm11.h \
	arm11_dbgtap.h \
	armv4_5.h \
	armv4_5_mmu.h \
	armv4_5_cache.h \
	armv7a.h \
	armv7m.h \
	avrt.h \
	dsp563xx.h \
	dsp563xx_once.h \
	dsp5680xx.h \
	breakpoints.h \
	cortex_m.h \
	cortex_a.h \
	embeddedice.h \
	etb.h \
	etm.h \
	etm_dummy.h \
	image.h \
	mips32.h \
	mips_m4k.h \
	mips_ejtag.h \
	mips32_pracc.h \
	mips32_dmaacc.h \
	oocd_trace.h \
	register.h \
	target.h \
	target_type.h \
	trace.h \
	target_request.h \
	trace.h \
	xscale.h \
	smp.h \
	avr32_ap7k.h \
	avr32_jtag.h \
	avr32_mem.h \
	avr32_regs.h \
	nds32.h \
	nds32_cmd.h \
	nds32_disassembler.h \
	nds32_edm.h \
	nds32_insn.h \
	nds32_reg.h \
	nds32_tlb.h \
	nds32_v2.h \
	nds32_v3_common.h \
	nds32_v3.h \
	nds32_v3m.h \
	nds32_aice.h \
	lakemont.h \
	x86_32_common.h

ocddatadir = $(pkglibdir)
nobase_dist_ocddata_DATA =

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

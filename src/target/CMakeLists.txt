# SPDX-License-Identifier: GPL-2.0-or-later

add_library(target STATIC)

# TARGET_CORE_SRC
target_sources(target PRIVATE
	algorithm.c
	register.c
	image.c
	breakpoints.c
	target.c
	target_request.c
	testee.c
	semihosting_common.c
	smp.c
	rtt.c
)

# ARM_DEBUG_SRC
target_sources(target PRIVATE
	arm_dpm.c
	arm_jtag.c
	arm_disassembler.c
	arm_simulator.c
	arm_semihosting.c
	arm_adi_v5.c
	arm_dap.c
	armv7a_cache.c
	armv7a_cache_l2x.c
	adi_v5_dapdirect.c
	adi_v5_jtag.c
	adi_v5_swd.c
	embeddedice.c
	trace.c
	etb.c
	etm.c
	etm_dummy.c
	arm_tpiu_swo.c
	arm_cti.c
)

# ARMV4_5_SRC
target_sources(target PRIVATE
	armv4_5.c
	armv4_5_mmu.c
	armv4_5_cache.c
	# ARM7_9_SRC
	arm7_9_common.c
	arm7tdmi.c
	arm720t.c
	arm9tdmi.c
	arm920t.c
	arm966e.c
	arm946e.c
	arm926ejs.c
	feroceon.c
)

# ARMV6_SRC
target_sources(target PRIVATE
	arm11.c
	arm11_dbgtap.c
)

# ARMV7_SRC
target_sources(target PRIVATE
	armv7m.c
	armv7m_trace.c
	cortex_m.c
	armv7a.c
	armv7a_mmu.c
	cortex_a.c
	ls1_sap.c
	mem_ap.c
)

# ARMV8_SRC
target_sources(target PRIVATE
	armv8_dpm.c
	armv8_opcodes.c
	aarch64.c
	a64_disassembler.c
	armv8.c
	armv8_cache.c
)

# ARM_MISC_SRC
target_sources(target PRIVATE
	fa526.c
	xscale.c
)

# AVR32_SRC
target_sources(target PRIVATE
	avr32_ap7k.c
	avr32_jtag.c
	avr32_mem.c
	avr32_regs.c
)

# MIPS32_SRC
target_sources(target PRIVATE
	mips32.c
	mips_m4k.c
	mips32_pracc.c
	mips32_dmaacc.c
	mips_ejtag.c
)

# STM8_SRC
target_sources(target PRIVATE
	stm8.c
)

# INTEL_IA32_SRC
target_sources(target PRIVATE
	quark_x10xx.c
	quark_d20xx.c
	lakemont.c
	x86_32_common.c
)

# ESIRISC_SRC
target_sources(target PRIVATE
	esirisc.c
	esirisc_jtag.c
	esirisc_trace.c
)

# ARC_SRC
target_sources(target PRIVATE
	arc.c
	arc_cmd.c
	arc_jtag.c
	arc_mem.c
)

target_sources(target PRIVATE
	avrt.c
	dsp563xx.c
	dsp563xx_once.c
	dsp5680xx.c
	hla_target.c
)

# MIPS64_SRC
target_sources(target PRIVATE
	mips64.c
	mips32_pracc.c
	mips64_pracc.c
	mips_mips64.c
	trace.c
	mips_ejtag.c
)

target_sources(target PRIVATE
	algorithm.h
	arm.h
	arm_coresight.h
	arm_dpm.h
	arm_jtag.h
	arm_adi_v5.h
	armv7a_cache.h
	armv7a_cache_l2x.h
	armv7a_mmu.h
	arm_disassembler.h
	a64_disassembler.h
	arm_opcodes.h
	arm_simulator.h
	arm_semihosting.h
	arm7_9_common.h
	arm7tdmi.h
	arm720t.h
	arm9tdmi.h
	arm920t.h
	arm926ejs.h
	arm966e.h
	arm946e.h
	arm11.h
	arm11_dbgtap.h
	armv4_5.h
	armv4_5_mmu.h
	armv4_5_cache.h
	armv7a.h
	armv7m.h
	armv7m_trace.h
	armv8.h
	armv8_dpm.h
	armv8_opcodes.h
	armv8_cache.h
	avrt.h
	dsp563xx.h
	dsp563xx_once.h
	dsp5680xx.h
	breakpoints.h
	cortex_m.h
	cortex_a.h
	aarch64.h
	embeddedice.h
	etb.h
	etm.h
	etm_dummy.h
	arm_tpiu_swo.h
	image.h
	mips32.h
	mips64.h
	mips_cpu.h
	mips_m4k.h
	mips_mips64.h
	mips_ejtag.h
	mips32_pracc.h
	mips32_dmaacc.h
	mips64_pracc.h
	register.h
	target.h
	target_type.h
	trace.h
	target_request.h
	trace.h
	xscale.h
	smp.h
	avr32_ap7k.h
	avr32_jtag.h
	avr32_mem.h
	avr32_regs.h
	semihosting_common.h
	stm8.h
	lakemont.h
	x86_32_common.h
	arm_cti.h
	esirisc.h
	esirisc_jtag.h
	esirisc_regs.h
	esirisc_trace.h
	arc.h
	arc_cmd.h
	arc_jtag.h
	arc_mem.h
	rtt.h
)

if(HAVE_CAPSTONE)
	target_include_directories(target PRIVATE ${CAPSTONE_INCLUDE_DIRS})
	target_link_libraries(target PRIVATE ${CAPSTONE_LINK_LIBRARIES})
endif()

# sub targets of openocd-target library
list(APPEND target_targets
	espressif
	xtensa
	riscv
	openrisc
)

foreach(target ${target_targets})
	add_subdirectory(${target})
	target_compile_options(${target} PRIVATE ${OPENOCD_COMMON_COMPILER_FLAGS})
	target_include_directories(${target} PRIVATE $<TARGET_PROPERTY:openocd,INTERFACE_INCLUDE_DIRECTORIES>)
	target_include_directories(${target} PRIVATE $<TARGET_PROPERTY:openocd_main,INTERFACE_INCLUDE_DIRECTORIES>)
endforeach()

target_link_libraries(target PRIVATE ${target_targets})
# circular dependency
target_link_libraries(target PRIVATE rtos rtt)

set_property(TARGET tcl_scripts APPEND PROPERTY STARTUP_TCL_SRCS ${CMAKE_CURRENT_LIST_DIR}/startup.tcl)

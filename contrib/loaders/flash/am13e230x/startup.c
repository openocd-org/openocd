// SPDX-License-Identifier: GPL-2.0-or-later

/******************************************************************************
 *
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Startup code for AM13E230X flash loader algorithm.
 * OpenOCD jumps directly to entry() after loading the binary into SRAM.
 *
 *****************************************************************************/

#include <stdint.h>

extern int main(void);

/* Stack: 100 words = 400 bytes (matches cc26xx) */
__attribute__((section(".stack"), used))
static uint32_t stack[100];

/* BSS boundaries from linker */
extern uint32_t _bss;
extern uint32_t _ebss;
/* Stack top from linker */
extern uint32_t _estack;

/*
 * Entry point. OpenOCD sets PC here after loading the binary.
 */
__attribute__((section(".entry"), naked))
void entry(void)
{
	/* Switch to privileged Thread mode using MSP.
	 * The previous application (e.g. Zephyr) may have set
	 * CONTROL.nPRIV=1 (unprivileged). Peripheral access
	 * requires privileged mode. */
	__asm("		mov		r0, #0");
	__asm("		msr		CONTROL, r0");
	__asm("		isb");

	/* Initialize stack pointer to top of .stack section */
	__asm("		ldr		sp, =_estack");

	/* Zero fill the bss segment */
	__asm("		ldr     r0, =_bss\n"
		  "		ldr     r1, =_ebss\n"
		  "		mov     r2, #0\n"
		  "		.thumb_func\n"
		  "	zero_loop:\n"
		  "		cmp		r0, r1\n"
		  "		it		lt\n"
		  "		strlt	r2, [r0], #4\n"
		  "		blt		zero_loop");

	/* Call the application's entry point */
	__asm("		bl		main");

	/* If we ever return, infinite loop */
	__asm("	halt_loop:\n"
		  "		b		halt_loop");
}

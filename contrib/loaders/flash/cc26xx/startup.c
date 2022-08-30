// SPDX-License-Identifier: BSD-3-Clause

/******************************************************************************
*
* Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
*
******************************************************************************/

#include <stdint.h>

/******************************************************************************
*
* The entry point for the application startup code.
*
******************************************************************************/
extern int main(void);

/******************************************************************************
*
* Reserve space for the system stack.
*
******************************************************************************/
__attribute__ ((section(".stack")))
static uint32_t stack[100];
const uint32_t stack_pntr = (uint32_t)stack + sizeof(stack);

/******************************************************************************
*
* The following are constructs created by the linker indicating where
* the "bss" and "ebss" segments reside in memory.
*
******************************************************************************/
extern uint32_t _bss;
extern uint32_t _ebss;

/******************************************************************************
*
* This is the entry point that handles setting the stack within the allowed
* workspace, initializing the .bss segment, and then jumping to main.
*
******************************************************************************/
__attribute__ ((section(".entry")))
void entry(void)
{
	/* Workaround for ITT instructions. */
	__asm("		NOP");
	__asm("		NOP");
	__asm("		NOP");
	__asm("		NOP");

	/* Initialize stack pointer */
	__asm("		ldr		sp, =stack_pntr");

	/* Zero fill the bss segment. */
	__asm("		ldr     r0, =_bss\n"
		  "		ldr     r1, =_ebss\n"
		  "		mov     r2, #0\n"
		  "		.thumb_func\n"
		  "	zero_loop:\n"
		  "		cmp		r0, r1\n"
		  "		it		lt\n"
		  "		strlt	r2, [r0], #4\n"
		  "		blt		zero_loop");

	/* Call the application's entry point. */
	main();

	/* If we ever return, enter an infinite loop */
	while (1)
		;
}

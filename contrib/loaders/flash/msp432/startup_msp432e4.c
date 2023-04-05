// SPDX-License-Identifier: BSD-3-Clause

/******************************************************************************
*
* Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
*
******************************************************************************/

#include <stdint.h>

/* Entry point for the application. */
extern int main();

/* Reserve space for the system stack. */
extern uint32_t __stack_top;

typedef void(*pFunc)(void);

/* Interrupt handler prototypes */
void default_handler(void);
void reset_handler(void);

/*
 * The vector table.  Note that the proper constructs must be placed on this to
 * ensure that it ends up at physical address 0x0000.0000 or at the start of
 * the program if located at a start address other than 0.
 */
void (* const intr_vectors[])(void) __attribute__((section(".intvecs"))) = {
	(pFunc)&__stack_top, /* The initial stack pointer */
	reset_handler,       /* The reset handler         */
	default_handler,     /* The NMI handler           */
	default_handler,     /* The hard fault handler    */
	default_handler,     /* The MPU fault handler     */
	default_handler,     /* The bus fault handler     */
	default_handler,     /* The usage fault handler   */
	0,                   /* Reserved                  */
	0,                   /* Reserved                  */
	0,                   /* Reserved                  */
	0,                   /* Reserved                  */
	default_handler,     /* SVCall handler            */
	default_handler,     /* Debug monitor handler     */
	0,                   /* Reserved                  */
	default_handler,     /* The PendSV handler        */
	default_handler      /* The SysTick handler       */
};

/*
 * The following are constructs created by the linker, indicating where
 * the "data" and "bss" segments reside in memory.  The initializers for
 * the "data" segment resides immediately following the "text" segment.
 */
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

/*
 * This is the code that gets called when the processor first starts execution
 * following a reset event.  Only the absolutely necessary set is performed,
 * after which the application supplied entry() routine is called.  Any fancy
 * actions (such as making decisions based on the reset cause register, and
 * resetting the bits in that register) are left solely in the hands of the
 * application.
 */
__attribute__((section(".reset"))) __attribute__((naked))
void reset_handler(void)
{
	/* Set stack pointer */
	__asm("    MOVW.W  r0, #0x1700\n"
		  "    MOVT.W  r0, #0x2000\n"
		  "    mov     sp, r0\n");

	/* Zero fill the bss segment. */
	__asm("    ldr     r0, =__bss_start__\n"
		  "    ldr     r1, =__bss_end__\n"
		  "    mov     r2, #0\n"
		  "    .thumb_func\n"
		  "zero_loop:\n"
		  "    cmp     r0, r1\n"
		  "    it      lt\n"
		  "    strlt   r2, [r0], #4\n"
		  "    blt     zero_loop");

	/* Call the application's entry point. */
	main();
}

/*
 * This is the code that gets called when the processor receives an unexpected
 * interrupt.  This simply enters an infinite loop, preserving the system state
 * for examination by a debugger.
 */
void default_handler(void)
{
	/* Enter an infinite loop. */
	while (1)
		;
}

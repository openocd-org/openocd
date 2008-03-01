/***********************************************************************************
*	Copyright 2005 Anglia Design
*	This demo code and associated components are provided as is and has no warranty,
*	implied or otherwise.  You are free to use/modify any of the provided
*	code at your own risk in your applications with the expressed limitation
*	of liability (see below)
* 
*	LIMITATION OF LIABILITY:   ANGLIA OR ANGLIA DESIGNS SHALL NOT BE LIABLE FOR ANY
*	LOSS OF PROFITS, LOSS OF USE, LOSS OF DATA, INTERRUPTION OF BUSINESS, NOR FOR
*	INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES OF ANY KIND WHETHER UNDER
*	THIS AGREEMENT OR OTHERWISE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*
*	Author	: Spencer Oliver
*	Web     	: www.anglia-designs.com
*
*     mifi, 22.01.2008, small changes by the init of the C++ eabi constructors. 
*                       Here I have replaced the eabi init by the normal init.
*                       Thanks to Spen for the startup code. 
***********************************************************************************/

/**** Startup Code (executed after Reset) ****/

/* Frequency values kHz */
/* set to suit target hardware */

	.equ	FOSC,			25000
	
/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs */

	.equ	Mode_USR,		0x10
	.equ	Mode_FIQ,		0x11
	.equ	Mode_IRQ,		0x12
	.equ	Mode_SVC,		0x13
	.equ	Mode_ABT,		0x17
	.equ	Mode_UND,		0x1B
	.equ	Mode_SYS,		0x1F			/* available on ARM Arch 4 and later */

	.equ	I_Bit,			0x80			/* when I bit is set, IRQ is disabled */
	.equ	F_Bit,			0x40			/* when F bit is set, FIQ is disabled */

	.equ	SRAM32,			0x00
	.equ	SRAM64,			0x08
	.equ	SRAM96,			0x10

/* --- System memory locations */

	.equ	SCRO_AHB_UMB,		0x5C002034		/* System configuration register 0 (unbuffered) */

	.equ	FMI_BASE_UMB,		0x54000000		/* Flash FMI base address (unbuffered) */
	.equ	BBSR_off_addr,		0x00
	.equ	NBBSR_off_addr,		0x04
	.equ	BBADR_off_addr,		0x0C
	.equ	NBBADR_off_addr,	0x10
	.equ	CR_off_addr,		0x18

.ifndef LIBUFF
	.equ	LIBUFF, 0
.endif

/* Startup Code must be linked first at Address at which it expects to run. */

	.text
	.arm
	.section .init, "ax"
	
	.global _start
	.global _Main_Crystal

/* After remap this will be our reset handler */

_start:
		LDR     pc, =NextInst
NextInst:

		NOP		/* Wait for OSC stabilization */
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP

/* Enable buffered mode */

.if LIBUFF
		MRC		p15, 0, r0, c1, c0, 0		/* Read CP15 register 1 into r0 */
		ORR		r0, r0, #0x8				/* Enable Write Buffer on AHB */
		MCR		p15, 0, r0, c1, c0, 0 		/* Write CP15 register 1 */
.endif

/* Remap Flash Bank 0 at address 0x0 and Bank 1 at address 0x80000, */
/* when the bank 0 is the boot bank, then enable the Bank 1. */

		LDR		r0, =FMI_BASE_UMB
		LDR		r1, =0x4					/* configure 512KB Boot bank 0 */
		STR		r1, [r0, #BBSR_off_addr]

		LDR		r1, =0x2					/* configure 32KB Non Boot bank 1 */
		STR		r1, [r0, #NBBSR_off_addr]

		LDR		r1, =(0x00000000 >> 2)		/* Boot Bank Base Address */
		STR		r1, [r0, #BBADR_off_addr]

		LDR		r1, =(0x00080000 >> 2)		/* Non Boot Bank Base Address */
		STR		r1, [r0, #NBBADR_off_addr]

		LDR		r1, =0x18					/* Flash Banks 0 1 enabled */
		STR		r1, [r0, #CR_off_addr]
		
/* Enable 96K RAM */

		LDR		r0, =SCRO_AHB_UMB
#		LDR		r1, =0x0196				/* prefetch disabled, default enabled */
		LDR		r1, =0x0187|SRAM96
		STR		r1, [r0]

/* Set bits 17-18 (Instruction/Data TCM order) of the */
/* Core Configuration Control Register */
 
		MOV		r0, #0x60000
		MCR		p15, 0x1, r0, c15, c1, 0
  
/* Setup Stack for each mode */

/* Enter Abort Mode and set its Stack Pointer */

		MSR		cpsr_c, #Mode_ABT|I_Bit|F_Bit
		LDR		sp, =__stack_abt_end__

/* Enter Undefined Instruction Mode and set its Stack Pointer */

		MSR		cpsr_c, #Mode_UND|I_Bit|F_Bit
		LDR		sp, =__stack_und_end__

/* Enter Supervisor Mode and set its Stack Pointer */

		MSR		cpsr_c, #Mode_SVC|I_Bit|F_Bit
		LDR		sp, =__stack_svc_end__

/* Enter FIQ Mode and set its Stack Pointer */

		MSR		cpsr_c, #Mode_FIQ|I_Bit|F_Bit
		LDR		sp, =__stack_fiq_end__

/* Enter IRQ Mode and set its Stack Pointer */

		MSR		cpsr_c, #Mode_IRQ|I_Bit|F_Bit
		LDR		sp, =__stack_irq_end__

/* Enter System/User Mode and set its Stack Pointer */

		MSR		cpsr_c, #Mode_SYS
		LDR		sp, =__stack_end__

/* Setup a default Stack Limit (when compiled with "-mapcs-stack-check") */

		LDR		sl, =__bss_end__

/* Relocate .data section (Copy from ROM to RAM) */

		LDR		r1, =_etext
		LDR		r2, =__data_start
		LDR		r3, =_edata
LoopRel:
		CMP		r2, r3
		LDRLO	r0, [r1], #4
		STRLO	r0, [r2], #4
		BLO		LoopRel

/* Clear .bss section (Zero init) */

		MOV		r0, #0
		LDR		r1, =__bss_start__
		LDR		r2, =__bss_end__
LoopZI:
		CMP		r1, r2
		STRLO	r0, [r1], #4
		BLO		LoopZI
  		
/* Call C++ constructors */

		LDR		r0, =__ctors_start__
		LDR 		r1, =__ctors_end__
ctor_loop:
		CMP 		r0, r1
		BEQ 		ctor_end
		LDR 		r2, [r0], #4
		STMFD 	sp!, {r0-r1}
		BLX		r2
		LDMFD		sp!, {r0-r1}
		B 		ctor_loop
ctor_end:

/* Need to set up standard file handles */
/* Only used under simulator, normally overide syscall.c */

#		BL		initialise_monitor_handles

/* if we use debug version of str9lib this will call the init function */
		
		BL		libdebug
libdebug:		

/* Enter the C code, use B instruction so as to never return */
/* use BL main if you want to use c++ destructors below */

		B		main

/* Return from main, loop forever. */

#exit_loop:
#		B		exit_loop
	
/* Fosc values, used by libstr9 */

_Main_Crystal:	.long	FOSC

	.weak libdebug
	
	.end

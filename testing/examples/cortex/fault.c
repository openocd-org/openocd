/*
 * COMPILE:  arm-none-eabi-gcc -mthumb -march=armv7-m ...
 *	... plus, provide at least a default exception vector table.
 *
 * RUN:  this is best run from SRAM.  It starts at main() then triggers
 * a fault before more than a handful of instructions have executed.
 * Run each test case in two modes:
 *
 * (1)	Faults caught on the Cortex-M3.  Default handlers are usually
 *	loop-to-self NOPs, so a debugger won't notice faults until they
 *	halt the core and examine xSPR and other registers.
 *
 *	To verify the fault triggered, issue "halt" from OpenOCD; you
 *	should be told about the fault and (some of) its details.
 *	Then it's time to run the next test.
 *
 *	NOTE however that "reset" will restart everything; verify that
 *	case by observing your reset handler doing its normal work.
 *
 * (2)	Faults intercepted by OpenOCD "vector_catch ..." commands.
 *
 *	OpenOCD should tell you about the fault, and show the same
 *	details, without your "halt" command.
 *
 * Someday, a fancy version of this code could provide a vector table and
 * fault handlers which use semihosting (when that works on Cortex-M3) to
 * report what happened, again without needing a "halt" command.
 */


/* These symbols match the OpenOCD "cortex_m vector_catch" bit names. */
enum vc_case {
	hard_err,
	int_err,
	bus_err,
	state_err,
	chk_err,
	nocp_err,
	mm_err,
	reset,
};

/* REVISIT come up with a way to avoid recompiling, maybe:
 *  - write it in RAM before starting
 *  - compiled-in BKPT, manual patch of r0, then resume
 *  - ...
 */

#ifndef VC_ID
#warning "no VC_ID ... using reset"
#define VC_ID reset
#endif

int main(void) __attribute__ ((externally_visible, noreturn));

/*
 * Trigger various Cortex-M3 faults to verify that OpenOCD behaves OK
 * in terms of its vector_catch handling.
 *
 * Fault handling should be left entirely up to the application code
 * UNLESS a "vector_catch" command tells OpenOCD to intercept a fault.
 *
 * See ARMv7-M architecure spec table B1-9 for the list of faults and
 * their mappings to the vector catch bits.
 */
int main(void)
{
	/* One test case for each vector catch bit.  We're not doing
	 * hardware testing; so it doesn't matter when some DEMCR bits
	 * could apply in multiple ways.
	 */
	switch (VC_ID) {

	/* "cortex_m vector_catch hard_err" */
	case hard_err:
		/* FORCED - Fault escalation */

		/* FIXME code this */
		break;

	/* "cortex_m vector_catch int_err" */
	case int_err:
		/* STKERR -- Exception stack BusFault */

		/* FIXME code this */
		break;

	/* "cortex_m vector_catch bus_err" */
	case bus_err:
		/* PRECISERR -- precise data bus read
		 * Here we assume a Cortex-M3 with 512 MBytes SRAM is very
		 * unlikely, so the last SRAM byte isn't a valid address.
		 */
		__asm__ volatile(
			"mov r0, #0x3fffffff\n"
			"ldrb r0, [r0]\n"
			);
		break;

	/* "cortex_m vector_catch state_err" */
	case state_err:
		/* UNDEFINSTR -- architectural undefined instruction */
		__asm__ volatile(".hword 0xde00");
		break;

	/* "cortex_m vector_catch chk_err" */
	case chk_err:
		/* UNALIGNED ldm */
		__asm__ volatile(
			"mov r0, #1\n"
			"ldm r0, {r1, r2}\n"
			);
		break;

	/* "cortex_m vector_catch nocp_err" */
	case nocp_err:
		/* NOCP ... Cortex-M3 has no coprocessors (like CP14 DCC),
		 * but these instructions are allowed by ARMv7-M.
		 */
		__asm__ volatile("mrc p14, 0, r0, c0, c5, 0");
		break;

	/* "cortex_m vector_catch mm_err" */
	case mm_err:
		/* IACCVIOL -- instruction fetch from an XN region */
		__asm__ volatile(
			"mov r0, #0xe0000000\n"
			"mov pc, r0\n"
			);
		break;

	/* "cortex_m vector_catch reset" */
	case reset:
		__asm__ volatile(
			/* r1 = SYSRESETREQ */
			"mov r1, #0x0004\n"
			/* r1 |= VECTKEY */
			"movt r1, #0x05fa\n"
			/* r0 = &AIRCR */
			"mov r0, #0xed00\n"
			"add r0, #0xc\n"
			"movt r0, #0xe000\n"
			/* AIRCR = ... */
			"str r1, [r0, #0]\n"
			);
		break;
	}

	/* don't return */
	while (1)
		continue;
}

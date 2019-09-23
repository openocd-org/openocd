#if __riscv_xlen == 64
# define LREG ld
# define SREG sd
# define REGBYTES 8
#else
# define LREG lw
# define SREG sw
# define REGBYTES 4
#endif

		.section .text.entry
		.global _start
_start:
		/* Spell out `la sp, stack_end` because otherwise the assembler won't
		generate position-independent code. */
1:	    auipc sp,     %pcrel_hi(stack_end) # load stack_end(hi)
	    addi  sp, sp, %pcrel_lo(1b)  # load stack_end(lo)
		jal 	flash_fespi
		ebreak

		.section .data
		.balign REGBYTES
stack:
		.fill	16, REGBYTES, 0x8675309
stack_end:

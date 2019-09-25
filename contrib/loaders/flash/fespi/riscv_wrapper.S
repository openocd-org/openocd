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
		lla		sp, stack_end
		jal 	flash_fespi
		ebreak

		.section .data
		.balign REGBYTES
stack:
		.fill	16, REGBYTES, 0x8675309
stack_end:

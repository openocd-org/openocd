/*
	Jump table for flash driver
	
	Registers in ARM callling convention is to place args in registers 
	starting at r0.
	
	So for:
	
	void foo(int a, int b, int c).
	
	a=r0
	b=r1
	c=r2
	
	
*/
	.global _stack_base
	.global _stack_start
	.global _workarea
	.global _start
	.global _start_bss_clear
_start:
	// offset=0
	// int erase(void *address, int len)
	ldr     sp,=_stack_start                
	bl erase
	nop // Stop CPU here using hw breakpoint
	
	// offset=0xc
	// int program(void *buffer, void *address, int len)	
	ldr     sp,=_stack_start              
	bl program
	nop // Stop CPU here using hw breakpoint
	
	// offset=0x18
	ldr     r0,=_workarea                
	nop // Stop CPU here using hw breakpoint
	
	// offset=0x20
	// int init() - returns error message if the flash chip can't be detected	
	ldr     sp,=_stack_start              
	bl init
	nop // Stop CPU here using hw breakpoint

    .section ".bss"
    .balign 4
_stack_base:
        .rept 4096
        .byte 0
        .endr
_stack_start:
    .balign 4
_workarea:
        .rept 8192
        .byte 0
        .endr
	// NB!!! we clear bss while the stack is in use, so we start BSS clearing here !!! :-)
_start_bss_clear:

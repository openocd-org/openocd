	.section	.rodata

	.align		4
	.global		 xscale_debug_handler
xscale_debug_handler:
	.incbin		"xscale/debug_handler.bin"
	.size		xscale_debug_handler, . - xscale_debug_handler

	.align		4
	.global		 xscale_debug_handler_size
xscale_debug_handler_size:
	.word		. - xscale_debug_handler
	.size		xscale_debug_handler_size, 4

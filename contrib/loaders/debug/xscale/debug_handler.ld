/* identify the Entry Point  */
ENTRY(reset_handler)

/* specify the mini-ICache memory areas  */
MEMORY
{
	mini_icache_0 (x)  				: ORIGIN = 0x0, LENGTH = 1024	/* first part of mini icache (sets 0-31) */
	mini_icache_1 (x)  				: ORIGIN = 0x400, LENGTH = 1024	/* second part of mini icache (sets 0-31) */
}

/* now define the output sections  */
SECTIONS
{
	.part1 :
	{
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		*(.part1)
	} >mini_icache_0

	.part2 :
	{
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		LONG(0)
		*(.part2)
		FILL(0x0)
	} >mini_icache_1

	/DISCARD/ :
	{
		*(.text)
		*(.glue_7)
		*(.glue_7t)
		*(.data)
		*(.bss)
	}
}

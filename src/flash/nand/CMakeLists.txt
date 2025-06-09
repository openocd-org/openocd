# SPDX-License-Identifier: GPL-2.0-or-later

add_library(nand STATIC)

target_sources(nand PRIVATE
	ecc.c
	ecc_kw.c
	core.c
	fileio.c
	tcl.c
	arm_io.c
	nonce.c
	davinci.c
	lpc3180.c
	lpc32xx.c
	mxc.c
	mx3.c
	orion.c
	s3c24xx.c
	s3c2410.c
	s3c2412.c
	s3c2440.c
	s3c2443.c
	s3c6400.c
	at91sam9.c 
	nuc910.c
	driver.c
	arm_io.h
	core.h
	driver.h
	fileio.h
	imp.h
	lpc3180.h
	lpc32xx.h
	mxc.h
	mx3.h
	s3c24xx.h
	s3c24xx_regs.h
	nuc910.h
)

# circular dependency
target_link_libraries(nand PRIVATE openocdflash)
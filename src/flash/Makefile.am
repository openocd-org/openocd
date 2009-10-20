AM_CPPFLAGS = \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/jtag \
	-I$(top_srcdir)/src/target

METASOURCES = AUTO
noinst_LTLIBRARIES = libflash.la
libflash_la_SOURCES = \
	arm_nandio.c \
	flash.c \
	lpc2000.c \
	lpc288x.c \
	lpc2900.c \
	cfi.c \
	non_cfi.c \
	at91sam7.c \
	at91sam3.c \
	davinci_nand.c \
	str7x.c \
	str9x.c \
	aduc702x.c \
	nand.c \
	nand_ecc.c \
	nand_ecc_kw.c \
	lpc3180_nand_controller.c \
	stellaris.c \
	str9xpec.c \
	stm32x.c \
	tms470.c \
	ecos.c \
	orion_nand.c \
	s3c24xx_nand.c \
	s3c2410_nand.c \
	s3c2412_nand.c \
	s3c2440_nand.c \
	s3c2443_nand.c \
	ocl.c \
	mflash.c \
	pic32mx.c \
	avrf.c \
	faux.c \
	mx3_nand.c

noinst_HEADERS = \
	arm_nandio.h \
	flash.h \
	lpc2000.h \
	lpc288x.h \
	lpc2900.h \
	cfi.h \
	non_cfi.h \
	at91sam7.h \
	at91sam3.h \
	str7x.h \
	str9x.h \
	nand.h \
	lpc3180_nand_controller.h \
	stellaris.h \
	str9xpec.h \
	stm32x.h \
	tms470.h \
	s3c24xx_nand.h \
	s3c24xx_regs_nand.h \
	mflash.h \
	ocl.h \
	pic32mx.h \
	avrf.h \
	mx3_nand.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

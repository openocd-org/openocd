AM_CPPFLAGS = \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/jtag \
	-I$(top_srcdir)/src/target

METASOURCES = AUTO
noinst_LTLIBRARIES = libflash.la
libflash_la_SOURCES = \
	$(FLASH_SRCS) \
	$(NAND_SRCS) \
	mflash.c

FLASH_SRCS = \
	common.c \
	cfi.c \
	non_cfi.c \
	faux.c \
	$(FLASH_DEVICES_SRCS) \
	flash.c

FLASH_DEVICES_SRCS = \
	aduc702x.c \
	at91sam3.c \
	at91sam7.c \
	avrf.c \
	ecos.c \
	lpc2000.c \
	lpc288x.c \
	lpc2900.c \
	ocl.c \
	pic32mx.c \
	stellaris.c \
	stm32x.c \
	str7x.c \
	str9x.c \
	str9xpec.c \
	tms470.c

NAND_SRCS = \
	arm_nandio.c \
	nand_ecc.c \
	nand_ecc_kw.c \
	$(NAND_DEVICES_SRCS) \
	nand.c

NAND_DEVICES_SRCS = \
	nonce_nand.c \
	davinci_nand.c \
	lpc3180_nand_controller.c \
	mx3_nand.c \
	orion_nand.c \
	s3c24xx_nand.c \
	s3c2410_nand.c \
	s3c2412_nand.c \
	s3c2440_nand.c \
	s3c2443_nand.c


noinst_HEADERS = \
	arm_nandio.h \
	at91sam7.h \
	at91sam3.h \
	avrf.h \
	cfi.h \
	common.h \
	flash.h \
	lpc2000.h \
	lpc288x.h \
	lpc3180_nand_controller.h \
	mflash.h \
	mx3_nand.h \
	non_cfi.h \
	nand.h \
	ocl.h \
	pic32mx.h \
	stellaris.h \
	stm32x.h \
	str7x.h \
	str9x.h \
	str9xpec.h \
	tms470.h \
	s3c24xx_nand.h \
	s3c24xx_regs_nand.h

EXTRA_DIST = startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

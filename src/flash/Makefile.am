INCLUDES = -I$(top_srcdir)/src/helper -I$(top_srcdir)/src/jtag -I$(top_srcdir)/src/target $(all_includes)
AM_CPPFLAGS = -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
METASOURCES = AUTO
noinst_LIBRARIES = libflash.a
libflash_a_SOURCES = \
	flash.c lpc2000.c cfi.c non_cfi.c at91sam7.c at91sam7_old.c \
	str7x.c str9x.c aduc702x.c nand.c nand_ecc.c \
	lpc3180_nand_controller.c stellaris.c str9xpec.c stm32x.c tms470.c \
	ecos.c orion_nand.c s3c24xx_nand.c s3c2410_nand.c s3c2412_nand.c \
	s3c2440_nand.c s3c2443_nand.c lpc288x.c ocl.c mflash.c pic32mx.c
noinst_HEADERS = \
	flash.h lpc2000.h cfi.h non_cfi.h at91sam7.h at91sam7_old.h str7x.h \
	str9x.h nand.h lpc3180_nand_controller.h stellaris.h str9xpec.h \
	stm32x.h tms470.h s3c24xx_nand.h s3c24xx_regs_nand.h lpc288x.h \
	mflash.h ocl.h pic32mx.h
MAINTAINERCLEANFILES = Makefile.in

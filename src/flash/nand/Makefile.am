include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdflashnand.la

libocdflashnand_la_SOURCES = \
	ecc.c \
	ecc_kw.c \
	core.c \
	fileio.c \
	tcl.c \
	arm_io.c \
	$(NAND_DRIVERS) \
	driver.c

NAND_DRIVERS = \
	nonce.c \
	davinci.c \
	lpc3180.c \
	lpc32xx.c \
	mxc.c \
	mx3.c \
	orion.c \
	s3c24xx.c \
	s3c2410.c \
	s3c2412.c \
	s3c2440.c \
	s3c2443.c \
	s3c6400.c \
	at91sam9.c \
	nuc910.c

noinst_HEADERS = \
	arm_io.h \
	core.h \
	driver.h \
	fileio.h \
	imp.h \
	lpc3180.h \
	lpc32xx.h \
	mxc.h \
	mx3.h \
	s3c24xx.h \
	s3c24xx_regs.h \
	nuc910.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

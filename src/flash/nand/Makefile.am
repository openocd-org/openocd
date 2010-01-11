AM_CPPFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_builddir)/src

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
	mx3.c \
	orion.c \
	s3c24xx.c \
	s3c2410.c \
	s3c2412.c \
	s3c2440.c \
	s3c2443.c \
	s3c6400.c \
	at91sam9.c

noinst_HEADERS = \
	arm_io.h \
	core.h \
	driver.h \
	fileio.h \
	imp.h \
	lpc3180.h \
	mx3.h \
	s3c24xx.h \
	s3c24xx_regs.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

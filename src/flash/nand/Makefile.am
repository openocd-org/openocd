AM_CPPFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/src/flash \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/jtag \
	-I$(top_srcdir)/src/target

noinst_LTLIBRARIES = libocdflashnand.la

libocdflashnand_la_SOURCES = \
	nonce.c \
	davinci.c \
	lpc3180.c \
	mx3.c \
	orion.c \
	s3c24xx.c \
	s3c2410.c \
	s3c2412.c \
	s3c2440.c \
	s3c2443.c

noinst_HEADERS = \
	lpc3180.h \
	mx3.h \
	s3c24xx.h \
	s3c24xx_regs.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

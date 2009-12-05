SUBDIRS = \
	nor \
	nand

AM_CPPFLAGS = \
	-I$(top_srcdir)/src

METASOURCES = AUTO
noinst_LTLIBRARIES = libflash.la
libflash_la_SOURCES = \
	common.c \
	arm_nandio.c \
	nand_ecc.c \
	nand_ecc_kw.c \
	nand.c \
	mflash.c

libflash_la_LIBADD = \
	$(top_builddir)/src/flash/nor/libocdflashnor.la \
	$(top_builddir)/src/flash/nand/libocdflashnand.la

noinst_HEADERS = \
	arm_nandio.h \
	common.h \
	mflash.h \
	nand.h

EXTRA_DIST = startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

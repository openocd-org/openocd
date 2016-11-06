noinst_LTLIBRARIES += %D%/libflash.la
%C%_libflash_la_SOURCES = \
	%D%/common.c %D%/common.h \
	%D%/mflash.c %D%/mflash.h

%C%_libflash_la_LIBADD = \
	%D%/nor/libocdflashnor.la \
	%D%/nand/libocdflashnand.la

STARTUP_TCL_SRCS += %D%/startup.tcl

include %D%/nor/Makefile.am
include %D%/nand/Makefile.am

noinst_LTLIBRARIES += %D%/libjtag.la

%C%_libjtag_la_LIBADD =

if HLADAPTER
include %D%/hla/Makefile.am
%C%_libjtag_la_LIBADD += $(top_builddir)/%D%/hla/libocdhla.la
endif

if AICE
include %D%/aice/Makefile.am
%C%_libjtag_la_LIBADD += $(top_builddir)/%D%/aice/libocdaice.la
endif

include %D%/drivers/Makefile.am
%C%_libjtag_la_LIBADD += $(top_builddir)/%D%/drivers/libocdjtagdrivers.la

%C%_libjtag_la_SOURCES = \
	%D%/adapter.c \
	%D%/adapter.h \
	%D%/commands.c \
	%D%/core.c \
	%D%/interface.c \
	%D%/interfaces.c \
	%D%/tcl.c \
	%D%/swim.c \
	%D%/commands.h \
	%D%/interface.h \
	%D%/interfaces.h \
	%D%/minidriver.h \
	%D%/jtag.h \
	%D%/swd.h \
	%D%/swim.h \
	%D%/tcl.h

STARTUP_TCL_SRCS += %D%/startup.tcl

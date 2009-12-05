AM_CPPFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_builddir)/src

METASOURCES = AUTO
noinst_LTLIBRARIES = libjtag.la

SUBDIRS =
DRIVERFILES =
libjtag_la_LIBADD =

if MINIDRIVER

# for minidriver_imp.h
AM_CPPFLAGS += -I$(srcdir)/minidriver

if ZY1000
DRIVERFILES += zy1000/zy1000.c
AM_CPPFLAGS += -I$(srcdir)/zy1000
endif
if MINIDRIVER_DUMMY
DRIVERFILES += minidummy/minidummy.c commands.c
AM_CPPFLAGS += -I$(srcdir)/minidummy
endif

else

DRIVERFILES += commands.c

SUBDIRS += drivers
libjtag_la_LIBADD += $(top_builddir)/src/jtag/drivers/libocdjtagdrivers.la

# for minidriver_imp.h
AM_CPPFLAGS += -I$(srcdir)/drivers

endif
# endif // MINIDRIVER

libjtag_la_SOURCES = \
	core.c \
	interface.c \
	interfaces.c \
	tcl.c \
	$(DRIVERFILES)

noinst_HEADERS = \
	commands.h \
	interface.h \
	interfaces.h \
	minidriver.h \
	jtag.h \
	minidriver/minidriver_imp.h \
	minidummy/jtag_minidriver.h

EXTRA_DIST = startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

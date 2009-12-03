AM_CPPFLAGS = \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/target

METASOURCES = AUTO
noinst_LTLIBRARIES = libjtag.la

SUBDIRS =
DRIVERFILES =
libjtag_la_LIBADD =

if MINIDRIVER

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
	minidummy/jtag_minidriver.h

EXTRA_DIST = startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

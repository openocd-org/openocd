include $(top_srcdir)/common.mk

METASOURCES = AUTO
noinst_LTLIBRARIES = libjtag.la

SUBDIRS =
DRIVERFILES =
libjtag_la_LIBADD =

CLEANFILES =

BUILT_SOURCES =

BUILT_SOURCES += minidriver_imp.h
CLEANFILES += minidriver_imp.h

if MINIDRIVER

if ZY1000
DRIVERFILES += zy1000/zy1000.c
JTAG_MINIDRIVER_DIR = $(srcdir)/zy1000
endif
if MINIDRIVER_DUMMY
DRIVERFILES += minidummy/minidummy.c commands.c
JTAG_MINIDRIVER_DIR = $(srcdir)/minidummy
endif

MINIDRIVER_IMP_DIR = $(srcdir)/minidriver

jtag_minidriver.h: $(JTAG_MINIDRIVER_DIR)/jtag_minidriver.h
	cp $< $@

BUILT_SOURCES += jtag_minidriver.h

CLEANFILES += jtag_minidriver.h

else

MINIDRIVER_IMP_DIR = $(srcdir)/drivers
DRIVERFILES += commands.c

if HLADAPTER
SUBDIRS += hla
libjtag_la_LIBADD += $(top_builddir)/src/jtag/hla/libocdhla.la
endif

if AICE
SUBDIRS += aice
libjtag_la_LIBADD += $(top_builddir)/src/jtag/aice/libocdaice.la
endif

SUBDIRS += drivers
libjtag_la_LIBADD += $(top_builddir)/src/jtag/drivers/libocdjtagdrivers.la


endif

# endif // MINIDRIVER

minidriver_imp.h: $(MINIDRIVER_IMP_DIR)/minidriver_imp.h
	cp $< $@


libjtag_la_SOURCES = \
	adapter.c \
	core.c \
	interface.c \
	interfaces.c \
	tcl.c \
	$(DRIVERFILES)

noinst_HEADERS = \
	commands.h \
	driver.h \
	interface.h \
	interfaces.h \
	minidriver.h \
	jtag.h \
	minidriver/minidriver_imp.h \
	minidummy/jtag_minidriver.h \
	swd.h \
	tcl.h

EXTRA_DIST = startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

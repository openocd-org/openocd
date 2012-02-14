include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdstlink.la

libocdstlink_la_SOURCES = \
	$(STLINKFILES)

STLINKFILES =

if STLINK
STLINKFILES += stlink_transport.c
STLINKFILES += stlink_tcl.c
STLINKFILES += stlink_interface.c
STLINKFILES += stlink_layout.c
endif

noinst_HEADERS = \
	stlink_interface.h \
	stlink_layout.h \
	stlink_tcl.h \
	stlink_transport.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

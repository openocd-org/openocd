include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdhla.la

libocdhla_la_SOURCES = \
	$(HLFILES)

HLFILES =

if HLADAPTER
HLFILES += hla_transport.c
HLFILES += hla_tcl.c
HLFILES += hla_interface.c
HLFILES += hla_layout.c
endif

noinst_HEADERS = \
	hla_interface.h \
	hla_layout.h \
	hla_tcl.h \
	hla_transport.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

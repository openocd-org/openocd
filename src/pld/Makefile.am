AM_CPPFLAGS = \
	-I$(top_srcdir)/src/server \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/jtag

METASOURCES = AUTO
noinst_LIBRARIES = libpld.a
noinst_HEADERS = pld.h xilinx_bit.h virtex2.h
libpld_a_SOURCES = pld.c xilinx_bit.c virtex2.c

MAINTAINERCLEANFILES = Makefile.in

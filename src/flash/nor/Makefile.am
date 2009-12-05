AM_CPPFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_builddir)/src

noinst_LTLIBRARIES = libocdflashnor.la
libocdflashnor_la_SOURCES = \
	core.c \
	tcl.c \
	$(NOR_DRIVERS) \
	drivers.c

NOR_DRIVERS = \
	aduc702x.c \
	at91sam3.c \
	at91sam7.c \
	avrf.c \
	cfi.c \
	ecos.c \
	faux.c \
	lpc2000.c \
	lpc288x.c \
	lpc2900.c \
	non_cfi.c \
	ocl.c \
	pic32mx.c \
	stellaris.c \
	stm32x.c \
	str7x.c \
	str9x.c \
	str9xpec.c \
	tms470.c

noinst_HEADERS = \
	at91sam7.h \
	at91sam3.h \
	avrf.h \
	core.h \
	cfi.h \
	driver.h \
	imp.h \
	lpc2000.h \
	lpc288x.h \
	non_cfi.h \
	ocl.h \
	pic32mx.h \
	stellaris.h \
	stm32x.h \
	str7x.h \
	str9x.h \
	str9xpec.h \
	tms470.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

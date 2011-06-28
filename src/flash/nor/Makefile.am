include $(top_srcdir)/common.mk

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
	em357.c \
	faux.c \
	lpc2000.c \
	lpc288x.c \
	lpc2900.c \
	non_cfi.c \
	ocl.c \
	pic32mx.c \
	stmsmi.c \
	stellaris.c \
	stm32x.c \
	stm32f2xxx.c \
	str7x.c \
	str9x.c \
	str9xpec.c \
	tms470.c \
	virtual.c

# Disabled for now, it generates warnings
# dsp5680xx_flash.c 

noinst_HEADERS = \
	core.h \
	cfi.h \
	driver.h \
	imp.h \
	non_cfi.h \
	ocl.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

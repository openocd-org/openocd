include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdflashnor.la
libocdflashnor_la_SOURCES = \
	core.c \
	tcl.c \
	$(NOR_DRIVERS) \
	drivers.c

NOR_DRIVERS = \
	aduc702x.c \
	at91sam4.c \
	at91sam4l.c \
	at91samd.c \
	at91sam3.c \
	at91sam7.c \
	avrf.c \
	cfi.c \
	efm32.c \
	em357.c \
	faux.c \
	lpc2000.c \
	lpc288x.c \
	lpc2900.c \
	lpcspifi.c \
	mdr.c \
	non_cfi.c \
	ocl.c \
	pic32mx.c \
	spi.c \
	stmsmi.c \
	stellaris.c \
	stm32f1x.c \
	stm32f2x.c \
	stm32lx.c \
	str7x.c \
	str9x.c \
	str9xpec.c \
	tms470.c \
	virtual.c \
	fm3.c \
	dsp5680xx_flash.c \
	kinetis.c \
	mini51.c \
	nuc1x.c \
	nrf51.c

noinst_HEADERS = \
	core.h \
	cfi.h \
	driver.h \
	imp.h \
	non_cfi.h \
	ocl.h \
	spi.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

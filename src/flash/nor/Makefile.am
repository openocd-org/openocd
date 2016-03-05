include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdflashnor.la
libocdflashnor_la_SOURCES = \
	core.c \
	tcl.c \
	$(NOR_DRIVERS) \
	drivers.c

NOR_DRIVERS = \
	aduc702x.c \
	aducm360.c \
	at91sam4.c \
	at91sam4l.c \
	at91samd.c \
	at91sam3.c \
	at91sam7.c \
	atsamv.c \
	avrf.c \
	cfi.c \
	efm32.c \
	em357.c \
	faux.c \
	jtagspi.c \
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
	stm32l4x.c \
	str7x.c \
	str9x.c \
	str9xpec.c \
	tms470.c \
	virtual.c \
	fm3.c \
	fm4.c \
	dsp5680xx_flash.c \
	kinetis.c \
	kinetis_ke.c \
	numicro.c \
	nrf51.c \
	mrvlqspi.c \
	psoc4.c \
	sim3x.c \
	xmc4xxx.c \
	niietcm4.c


noinst_HEADERS = \
	core.h \
	cfi.h \
	driver.h \
	imp.h \
	non_cfi.h \
	ocl.h \
	spi.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

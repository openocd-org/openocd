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
	ambiqmicro.c \
	at91sam4.c \
	at91sam4l.c \
	at91samd.c \
	at91sam3.c \
	at91sam7.c \
	atsamv.c \
	avrf.c \
	cfi.c \
	dsp5680xx_flash.c \
	efm32.c \
	em357.c \
	fespi.c \
	faux.c \
	fm3.c \
	fm4.c \
	jtagspi.c \
	kinetis.c \
	kinetis_ke.c \
	lpc2000.c \
	lpc288x.c \
	lpc2900.c \
	lpcspifi.c \
	mdr.c \
	mrvlqspi.c \
	niietcm4.c \
	non_cfi.c \
	nrf51.c \
	numicro.c \
	ocl.c \
	pic32mx.c \
	psoc4.c \
	sim3x.c \
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
	xmc1xxx.c \
	xmc4xxx.c

noinst_HEADERS = \
	core.h \
	cfi.h \
	driver.h \
	imp.h \
	non_cfi.h \
	ocl.h \
	spi.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

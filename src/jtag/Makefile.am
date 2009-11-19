AM_CPPFLAGS = \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/target

METASOURCES = AUTO
noinst_LTLIBRARIES = libjtag.la

DRIVERFILES =

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

# Standard Driver: common files
DRIVERFILES += driver.c commands.c

if USB
DRIVERFILES += usb_common.c
endif

if BITBANG
DRIVERFILES += bitbang.c
endif
if PARPORT
DRIVERFILES += parport.c
endif
if DUMMY
DRIVERFILES += dummy.c
endif
if FT2232_DRIVER
DRIVERFILES += ft2232.c
endif
if AMTJTAGACCEL
DRIVERFILES += amt_jtagaccel.c
endif
if EP93XX
DRIVERFILES += ep93xx.c
endif
if AT91RM9200
DRIVERFILES += at91rm9200.c
endif
if GW16012
DRIVERFILES += gw16012.c
endif
if BITQ
DRIVERFILES += bitq.c
endif
if PRESTO_DRIVER
DRIVERFILES += presto.c
endif
if USBPROG
DRIVERFILES += usbprog.c
endif
if JLINK
DRIVERFILES += jlink.c
endif
if RLINK
DRIVERFILES += rlink/rlink.c rlink/rlink_speed_table.c
endif
if VSLLINK
DRIVERFILES += vsllink.c
endif
if ARMJTAGEW
DRIVERFILES += arm-jtag-ew.c
endif

endif
# endif // MINIDRIVER


libjtag_la_SOURCES = \
	core.c \
	interface.c \
	interfaces.c \
	tcl.c \
	$(DRIVERFILES)

noinst_HEADERS = \
	interface.h \
	interfaces.h \
	commands.h \
	minidriver.h \
	bitbang.h \
	jtag.h \
	bitq.h \
	rlink/dtc_cmd.h \
	rlink/ep1_cmd.h \
	rlink/rlink.h \
	rlink/st7.h \
	minidummy/jtag_minidriver.h \
	usb_common.h

EXTRA_DIST = startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

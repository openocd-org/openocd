

INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/helper  $(all_includes) -I$(top_srcdir)/src/target 
METASOURCES = AUTO
noinst_LIBRARIES = libjtag.a

if BITBANG
BITBANGFILES = bitbang.c
else
BITBANGFILES =
endif

if PARPORT
PARPORTFILES = parport.c
else
PARPORTFILES =
endif

if DUMMY
DUMMYFILES = dummy.c
else
DUMMYFILES =
endif

if FT2232_LIBFTDI
FT2232FILES = ft2232.c
else
if FT2232_FTD2XX
FT2232FILES = ft2232.c
else
FT2232FILES =
endif
endif

if AMTJTAGACCEL
AMTJTAGACCELFILES = amt_jtagaccel.c
else
AMTJTAGACCELFILES =
endif

if EP93XX
EP93XXFILES = ep93xx.c
else
EP93XXFILES =
endif

if ECOSBOARD
ECOSBOARDFILES = zy1000.c
else
ECOSBOARDFILES =
endif

if AT91RM9200
AT91RM9200FILES = at91rm9200.c
else
AT91RM9200FILES =
endif

if GW16012
GW16012FILES = gw16012.c
else
GW16012FILES =
endif

if BITQ
BITQFILES = bitq.c
else
BITQFILES =
endif

if PRESTO_LIBFTDI
PRESTOFILES = presto.c
else
if PRESTO_FTD2XX
PRESTOFILES = presto.c
else
PRESTOFILES =
endif
endif

if USBPROG
USBPROGFILES = usbprog.c
else
USBPROGFILES =
endif

if JLINK
JLINKFILES = jlink.c
else
JLINKFILES =
endif

if RLINK
RLINKFILES = rlink/rlink.c rlink/rlink_speed_table.c
else
RLINKFILES =
endif

if VSLLINK
VSLLINKFILES = vsllink.c
else
VSLLINKFILES =
endif

if ARMJTAGEW
ARMJTAGEWFILES = arm-jtag-ew.c
else
ARMJTAGEWFILES =
endif

libjtag_a_SOURCES = jtag.c $(BITBANGFILES) $(PARPORTFILES) $(DUMMYFILES) $(FT2232FILES) $(AMTJTAGACCELFILES) $(EP93XXFILES) \
	$(AT91RM9200FILES) $(GW16012FILES) $(BITQFILES) $(PRESTOFILES) $(USBPROGFILES) $(ECOSBOARDFILES) $(JLINKFILES) $(RLINKFILES) $(VSLLINKFILES) $(ARMJTAGEWFILES)

noinst_HEADERS = bitbang.h jtag.h bitq.h rlink/dtc_cmd.h rlink/ep1_cmd.h rlink/rlink.h rlink/st7.h

MAINTAINERCLEANFILES = Makefile.in

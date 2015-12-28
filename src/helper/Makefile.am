include $(top_srcdir)/common.mk

METASOURCES = AUTO
noinst_LTLIBRARIES = libhelper.la

CONFIGFILES = options.c time_support_common.c

libhelper_la_CPPFLAGS = $(AM_CPPFLAGS) $(LIBUSB1_CFLAGS)

libhelper_la_SOURCES = \
	binarybuffer.c \
	$(CONFIGFILES) \
	configuration.c \
	log.c \
	command.c \
	time_support.c \
	replacements.c \
	fileio.c \
	util.c \
	jep106.c \
	jim-nvp.c

if IOUTIL
libhelper_la_SOURCES += ioutil.c
else
libhelper_la_SOURCES += ioutil_stubs.c
endif

libhelper_la_CFLAGS =
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
libhelper_la_CFLAGS += -Wno-sign-compare
endif

noinst_HEADERS = \
	binarybuffer.h \
	configuration.h \
	ioutil.h \
	list.h \
	util.h \
	types.h \
	log.h \
	command.h \
	time_support.h \
	replacements.h \
	fileio.h \
	system.h \
	bin2char.sh \
	jep106.h \
	jep106.inc \
	update_jep106.pl \
	jim-nvp.h

EXTRA_DIST = startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

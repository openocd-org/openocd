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
	bin2char.c \
	jim-nvp.h

EXTRA_DIST = startup.tcl

BIN2C = bin2char$(EXEEXT_FOR_BUILD)

BUILT_SOURCES = $(BIN2C)

$(BIN2C): bin2char.c
	${CC_FOR_BUILD} ${CFLAGS_FOR_BUILD} $(srcdir)/bin2char.c -o $@

CLEANFILES = bin2char$(EXEEXT_FOR_BUILD)

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

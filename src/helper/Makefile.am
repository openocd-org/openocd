AM_CPPFLAGS = \
	-I$(top_srcdir)/src/server \
	-I$(top_srcdir)/src/target \
	-DPKGDATADIR=\"$(pkgdatadir)\"

METASOURCES = AUTO
noinst_LTLIBRARIES = libhelper.la

if ECOSBOARD
CONFIGFILES =
else
CONFIGFILES = options.c jim.c jim-eventloop.c
endif



libhelper_la_SOURCES = \
	binarybuffer.c \
	$(CONFIGFILES) \
	configuration.c \
	log.c \
	command.c \
	time_support.c \
	replacements.c \
	fileio.c \
	membuf.c \
	startup_tcl.c

if IOUTIL
libhelper_la_SOURCES += ioutil.c
endif

libhelper_la_CFLAGS =
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
libhelper_la_CFLAGS += -Wno-sign-compare
endif

noinst_HEADERS = \
	binarybuffer.h \
	configuration.h \
	types.h \
	log.h \
	command.h \
	membuf.h \
	time_support.h \
	replacements.h \
	fileio.h \
	jim.h \
	jim-eventloop.h \
	system.h \
	startup.tcl \
	bin2char.c

BIN2C = bin2char$(EXEEXT_FOR_BUILD)

BUILT_SOURCES = $(BIN2C)

$(BIN2C): bin2char.c
	${CC_FOR_BUILD} ${CFLAGS_FOR_BUILD} $(srcdir)/bin2char.c -o $@

# Convert .tcl to cfile
startup_tcl.c: startup.tcl $(BIN2C)
	./$(BIN2C) startup_tcl < $(srcdir)/startup.tcl > $@

# add startup_tcl.c to make clean list
CLEANFILES = startup_tcl.c bin2char$(EXEEXT_FOR_BUILD)

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

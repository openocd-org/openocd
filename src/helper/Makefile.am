INCLUDES = $(all_includes)
METASOURCES = AUTO
AM_CPPFLAGS = -DPKGDATADIR=\"$(pkgdatadir)\" -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
noinst_LIBRARIES = libhelper.a
libhelper_a_SOURCES = binarybuffer.c configuration.c options.c log.c interpreter.c command.c time_support.c \
	replacements.c fileio.c
noinst_HEADERS = binarybuffer.h configuration.h types.h log.h command.h \
	interpreter.h time_support.h replacements.h fileio.h

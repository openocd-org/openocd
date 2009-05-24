# not a GNU package. You can remove this line, if
# have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.6

nobase_dist_pkgdata_DATA = \
	contrib/libdcc/dcc_stdio.c \
	contrib/libdcc/dcc_stdio.h \
	contrib/libdcc/example.c \
	contrib/libdcc/README \
	contrib/openocd.udev

SUBDIRS = src doc

EXTRA_DIST = \
	Doxyfile.in \
	tools/logger.pl

docs: pdf html doxygen

Doxyfile: $(srcdir)/Doxyfile.in
	sed -e 's,@srcdir\@,$(srcdir),' $< > $@

doxygen::
	$(MAKE) Doxyfile
	doxygen Doxyfile 2>&1 | perl $(srcdir)/tools/logger.pl > doxygen.log

distclean-local:
	rm -rf Doxyfile doxygen

DISTCLEANFILES = doxygen.log

MAINTAINERCLEANFILES = \
	configure \
	Makefile.in \
	depcomp \
	config.guess \
	config.sub \
	config.h.in \
	config.h.in~ \
	compile \
	ltmain.sh \
	missing \
	aclocal.m4 \
	install-sh

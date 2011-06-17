info_TEXINFOS = openocd.texi
openocd_TEXINFOS = fdl.texi
man_MANS = openocd.1
EXTRA_DIST = openocd.1 \
	INSTALL.txt

dist-hook:
	mkdir $(distdir)/manual
	cp -p $(srcdir)/manual/*.txt $(distdir)/manual
	for i in $$(cd $(srcdir)/manual/ && ls -d */); do \
		mkdir $(distdir)/manual/$$i; \
		cp -p $(srcdir)/manual/$$i/* $(distdir)/manual/$$i/; \
	done

MAINTAINERCLEANFILES = \
	$(srcdir)/Makefile.in \
	$(srcdir)/mdate-sh \
	$(srcdir)/stamp-vti \
	$(srcdir)/version.texi \
	$(srcdir)/texinfo.tex

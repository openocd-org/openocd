info_TEXINFOS = openocd.texi
openocd_TEXINFOS = fdl.texi
man_MANS = openocd.1
EXTRA_DIST = openocd.1

dist-hook:
	mkdir $(distdir)/manual
	cp -p $(srcdir)/manual/*.txt $(distdir)/manual

MAINTAINERCLEANFILES = Makefile.in mdate-sh texinfo.tex

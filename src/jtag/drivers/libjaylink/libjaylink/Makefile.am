##
## This file is part of the libjaylink project.
##
## Copyright (C) 2014-2015 Marc Schink <jaylink-dev@marcschink.de>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

if SUBPROJECT_BUILD
noinst_LTLIBRARIES = libjaylink.la
else
lib_LTLIBRARIES = libjaylink.la

library_includedir = $(includedir)/libjaylink
library_include_HEADERS = libjaylink.h
nodist_library_include_HEADERS = version.h
endif

libjaylink_la_SOURCES = \
	buffer.c \
	core.c \
	device.c \
	discovery.c \
	discovery_tcp.c \
	emucom.c \
	error.c \
	fileio.c \
	jtag.c \
	list.c \
	log.c \
	socket.c \
	strutil.c \
	swd.c \
	swo.c \
	target.c \
	transport.c \
	transport_tcp.c \
	util.c \
	version.c

libjaylink_la_CFLAGS = $(JAYLINK_CFLAGS)
libjaylink_la_LDFLAGS = $(JAYLINK_LDFLAGS) -no-undefined
libjaylink_la_LIBADD = $(JAYLINK_LIBS)

if HAVE_LIBUSB
libjaylink_la_SOURCES += discovery_usb.c transport_usb.c
libjaylink_la_CFLAGS += $(libusb_CFLAGS)
libjaylink_la_LIBADD += $(libusb_LIBS)
endif

noinst_HEADERS = libjaylink-internal.h

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

AC_PREREQ([2.64])

AC_INIT([libjaylink], [0.2.0], [jaylink-dev@marcschink.de], [libjaylink],
	[http://git.zapb.de/libjaylink.git])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])

AC_CANONICAL_HOST

AM_INIT_AUTOMAKE([-Wall -Werror])

# Enable additional compiler warnings via -Wall and -Wextra. Use hidden
# visibility for all non-static symbols by default with -fvisibility=hidden.
JAYLINK_CFLAGS="-Wall -Wextra -Werror -fvisibility=hidden"

# Checks for programs.
AC_PROG_CC

# Automake >= 1.12 requires AM_PROG_AR when using options -Wall and -Werror.
# To be compatible with older versions of Automake use AM_PROG_AR if it's
# defined only. This line must occur before LT_INIT.
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

# Initialize libtool.
LT_INIT

# Initialize pkg-config.
PKG_PROG_PKG_CONFIG

# Checks for libraries.

# Check for libusb-1.0 which is always needed.
PKG_CHECK_MODULES([libusb], [libusb-1.0 >= 1.0.16],
	[HAVE_LIBUSB=yes], [HAVE_LIBUSB=no])

AS_IF([test "x$HAVE_LIBUSB" = "xyes"],
	[libusb_msg="yes"], [libusb_msg="no (missing: libusb-1.0)"])

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_C_BIGENDIAN

# Checks for library functions.

# Disable progress and informational output of libtool.
AC_SUBST([AM_LIBTOOLFLAGS], '--silent')

JAYLINK_SET_PACKAGE_VERSION([JAYLINK_VERSION_PACKAGE], [AC_PACKAGE_VERSION])

# Libtool interface version of libjaylink. This is not the same as the package
# version. For information about the versioning system of libtool, see:
# http://www.gnu.org/software/libtool/manual/libtool.html#Libtool-versioning
JAYLINK_SET_LIBRARY_VERSION([JAYLINK_VERSION_LIBRARY], [0:0:0])

AC_ARG_ENABLE([subproject-build], AS_HELP_STRING([--enable-subproject-build],
	[enable sub-project build [default=no]]))

AM_CONDITIONAL([SUBPROJECT_BUILD],
	[test "x$enable_subproject_build" = "xyes"])

AC_ARG_WITH([libusb], [AS_HELP_STRING([--without-libusb],
	[disable libusb support [default=detect]])])

AS_IF([test "x$with_libusb" != "xno"],
	[with_libusb="yes"])

AS_IF([test "x$with_libusb$HAVE_LIBUSB" = "xyesyes"],
	[AC_DEFINE([HAVE_LIBUSB], [1], [Define to 1 if libusb is available.])])

AS_IF([test "x$with_libusb" != "xyes"],
	[libusb_msg="no (disabled)"])

AS_IF([test "x$with_libusb$HAVE_LIBUSB" = "xyesyes"],
	[JAYLINK_PKG_LIBS="libusb-1.0"])

AM_CONDITIONAL([HAVE_LIBUSB],
	[test "x$with_libusb$HAVE_LIBUSB" = "xyesyes"])

# Libtool interface version is not used for sub-project build as libjaylink is
# built as libtool convenience library.
AS_IF([test "x$enable_subproject_build" != "xyes"],
	[JAYLINK_LDFLAGS="-version-info $JAYLINK_VERSION_LIBRARY"])

# Use C99 compatible stdio functions on MinGW instead of the incompatible
# functions provided by Microsoft.
AS_CASE([$host_os], [mingw*],
	[AC_DEFINE([__USE_MINGW_ANSI_STDIO], [1],
		[Define to 1 to use C99 compatible stdio functions on MinGW.])])

# Add the Winsock2 library on MinGW for socket and other network-related
# functions.
AS_CASE([$host_os], [mingw*], [JAYLINK_LIBS="$JAYLINK_LIBS -lws2_32"])

AC_SUBST([JAYLINK_CFLAGS])
AC_SUBST([JAYLINK_LDFLAGS])
AC_SUBST([JAYLINK_LIBS])
AC_SUBST([JAYLINK_PKG_LIBS])

AC_CONFIG_FILES([Makefile])
AC_CONFIG_FILES([libjaylink/Makefile])
AC_CONFIG_FILES([libjaylink/version.h])
AC_CONFIG_FILES([libjaylink.pc])
AC_CONFIG_FILES([Doxyfile])

AC_OUTPUT

echo
echo "libjaylink configuration summary:"
echo " - Package version ................ $JAYLINK_VERSION_PACKAGE"
echo " - Library version ................ $JAYLINK_VERSION_LIBRARY"
echo " - Installation prefix ............ $prefix"
echo " - Building on .................... $build"
echo " - Building for ................... $host"

echo
echo "Enabled transports:"
echo " - USB ............................ $libusb_msg"
echo " - TCP ............................ yes"
echo

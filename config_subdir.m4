dnl
dnl If needed, define the m4_ifblank and m4_ifnblank macros from autoconf 2.64
dnl This allows us to run with earlier Autoconfs as well.
ifdef([m4_ifblank],[],[
m4_define([m4_ifblank],
[m4_if(m4_translit([[$1]],  [ ][	][
]), [], [$2], [$3])])])
dnl
ifdef([m4_ifnblank],[],[
m4_define([m4_ifnblank],
[m4_if(m4_translit([[$1]],  [ ][	][
]), [], [$3], [$2])])])
dnl

dnl AC_CONFIG_SUBDIRS does not allow configure options to be passed
dnl to subdirs, this function allows that by creating a configure.gnu
dnl script that prepends configure options and then calls the real
dnl configure script
AC_DEFUN([AX_CONFIG_SUBDIR_OPTION],
[
AC_CONFIG_SUBDIRS([$1])

m4_ifblank([$2], [rm -f $srcdir/$1/configure.gnu],
[echo -e '#!/bin/sh\nexec "`dirname "'\$'0"`/configure" $2 "'\$'@"' > "$srcdir/$1/configure.gnu"
])
])

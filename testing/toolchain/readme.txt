Work-in-progress. Ignore for now.

Some build scripts for GCC toolchains. Notably the
Cortex CPUs require the very latest GCC version 4.3.x.

These are to be considered working notes for testers and
not a definitive source on how to build GCC toolchains.

1. get latest binutils, gcc, gdb and newlib

2. unzip source to src folder

2. fix libstc++/configure.ac


That works. After replacing AC_LIBTOOL_DLOPEN with

  if test "x${with_newlib}" != "xyes"; then
    AC_LIBTOOL_DLOPEN
  fi

and running autoconf I was able to build six different newlib targets.

http://gcc.gnu.org/ml/gcc/2008-03/msg00611.html

3. place newlib and libgloss into src/gcc

4. run cygwin.sh or linux.sh


Resources:

http://ecos.sourceware.org/build-toolchain.html


Results:

Build results from Zylin AS following the instructions above:

http://www.zylin.com/cortex-gcc-linux.tar.bz2

 
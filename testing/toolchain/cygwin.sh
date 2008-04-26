# Work in progress....
#
# CygWin hosted arm toolchain

set -e
rm -rf cygwin

rm -rf gcc
rm -rf gdb
rm -rf binutils


# here we need Linux hosted toolchain in the path
export PATH=`pwd`/install/bin:`pwd`/cygwin/bin:$PATH

mkdir cygwin

mkdir gcc
cd gcc
../src/binutils/configure --host=i686-pc-cygwin --target=arm-elf --build=i686-pc-linux-gnu --prefix=`pwd`/../cygwin
make
make install
cd ..

mkdir gcc
cd gcc
../src/configure --target=arm-elf  --enable-languages=c,c++ --with-gnu-as --with-gnu-ld --with-newlib --disable-shared --enable-newlib -v  --enable-multilib --disable-threads --enable-sjlj-exceptions --enable-libstdcxx-allocator=malloc --host=i686-pc-cygwin --build=i686-pc-linux-gnu --prefix=`pwd`/../cygwin  --disable-libssp
make 
make install
cd ..

mkdir gdb
cd gdb/
../src/gdb/configure --host=i686-pc-cygwin --target=arm-elf --build=i686-pc-linux-gnu --prefix=`pwd`/../cygwin
make
make install
cd ..

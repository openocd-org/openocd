# Build cygwin hosted cortex arm toolchain under Linux

set -e
rm -rf gcc
rm -rf binutils
rm -rf gdb

export PATH=`pwd`/install/bin:$PATH

#set HOST_NAME=i386-mingw32msvc
#set HOST_NAME=i686-pc-cygwin


mkdir binutils
cd binutils
../src/binutils/configure --host=$HOST_NAME --target=arm-elf  --prefix=`pwd`/../install
make
make install
cd ..

mkdir gcc
cd gcc
../src/gcc/configure --disable-libssp --target=arm-elf  --enable-languages=c,c++ --with-gnu-as --with-gnu-ld --with-newlib --disable-shared --enable-newlib -v  --disable-multilib --disable-threads --enable-sjlj-exceptions --enable-libstdcxx-allocator=malloc  --prefix=`pwd`/../install --disable-libssp
#../src/gcc/configure --target=arm-elf  --enable-languages=c --with-gnu-as --with-gnu-ld --with-newlib --disable-shared --enable-newlib -v  --disable-multilib --disable-threads    --prefix=`pwd`/../install  --disable-libssp
make 
make install
cd ..

mkdir gdb
cd gdb/
../src/gdb/configure --target=arm-elf  --prefix=`pwd`/../install
make
make install
cd ..

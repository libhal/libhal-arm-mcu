#!/usr/bin/sh

set -ex

BUILDDIR=build/rp2350-arm-s/RelWithDebInfo

conan build . -pr:h=rp2350 -pr:h=arm-gcc-12.3 --build=missing

source $BUILDDIR/generators/conanbuild.sh

cp $BUILDDIR/test_package $BUILDDIR/test_package.elf

file $BUILDDIR/test_package.elf

picotool uf2 convert $BUILDDIR/test_package.elf $BUILDDIR/test_package.uf2
picotool load $BUILDDIR/test_package.uf2 -f
sleep 1
plink -serial /dev/ttyACM0 -sercfg 115200,8,1,n,X


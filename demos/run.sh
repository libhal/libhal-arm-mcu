#!/usr/bin/sh

set -ex

BUILDDIR=build/rp2350-arm-s/RelWithDebInfo

conan build . -pr:h=rp2350 -pr:h=arm-gcc-12.3 --build=missing

source $BUILDDIR/generators/conanbuild.sh

picotool uf2 convert $BUILDDIR/blinker.elf $BUILDDIR/blinker.uf2
picotool load $BUILDDIR/blinker.uf2 -f
sleep 1
plink -serial /dev/ttyACM0 -sercfg 115200,8,1,n,X


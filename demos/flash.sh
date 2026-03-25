#!/usr/bin/sh

set -e

BUILDDIR=build/rp2350-arm-s/RelWithDebInfo
source $BUILDDIR/generators/conanbuild.sh

picotool uf2 convert $BUILDDIR/$1.elf $BUILDDIR/$1.uf2
picotool load $BUILDDIR/$1.uf2 -f
sleep 1
plink -serial /dev/ttyACM0 -sercfg 115200,8,1,n,X

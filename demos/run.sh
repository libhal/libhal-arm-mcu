#!/usr/bin/sh

source build/rp2350-arm-s/RelWithDebInfo/generators/conanbuild.sh

picotool load build/rp2350-arm-s/RelWithDebInfo/test_package.uf2 -f
sleep 1
plink -serial /dev/ttyACM0 -sercfg 115200,8,1,n,X


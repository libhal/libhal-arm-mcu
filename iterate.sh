#!/usr/bin/sh

set -ex

yes | conan remove libhal-arm-mcu
conan create . -pr:h=rp2350 -pr:h=arm-gcc-12.3 --version=latest
rm -rdf demos/build
conan build demos -pr:h=rp2350 -pr:h=arm-gcc-12.3 --build=missing
source demos/build/rp2350-arm-s/MinSizeRel/generators/conanbuild.sh
picotool load demos/build/rp2350-arm-s/MinSizeRel/test_package.uf2 -f
sleep 1
plink -serial /dev/ttyACM0 -sercfg 115200,8,1,n,X

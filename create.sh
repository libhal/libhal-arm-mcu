#!/usr/bin/sh

yes | conan remove libhal-arm-mcu
conan create . -pr:h=rp2350b -pr:h=arm-gcc-12.3 --version=latest

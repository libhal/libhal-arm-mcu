#!/usr/bin/sh

yes | conan remove libhal-arm-mcu
conan create . -pr:h=rp2350 -pr:h=arm-gcc-12.3 --version=latest

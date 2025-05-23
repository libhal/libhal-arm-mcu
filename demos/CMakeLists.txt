# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)

if(DEFINED ENV{PICO_SDK_PATH})
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(demos LANGUAGES CXX C ASM)

find_package(libhal-arm-mcu CONFIG REQUIRED)

if(DEFINED ENV{PICO_SDK_PATH})
pico_sdk_init()

add_executable(test_package main.cpp)
target_link_libraries(libhal::arm-mcu INTERFACE
  pico_stdlib
  hardware_gpio
  pico_stdio
)
target_link_libraries(test_package
  libhal::arm-mcu
)
pico_enable_stdio_usb(test_package 1)
pico_enable_stdio_uart(test_package 0)
endif()

set(ENV{LIBHAL_PLATFORM_LIBRARY} arm-mcu)

# libhal_build_demos(
#   DEMOS
#   adc
#   blinker
#   can
#   dac
#   gpio
#   i2c
#   interrupt_pin
#   uart
#   pwm
#   pwm16
#   spi
#   blank
#   # stream_dac

#   PACKAGES
#   minimp3

#   INCLUDES
#   .

#   LINK_LIBRARIES
#   minimp3::minimp3
# )

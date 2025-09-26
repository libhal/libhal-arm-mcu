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

pico_enable_stdio_usb(libhal::arm-mcu 1)
pico_enable_stdio_uart(libhal::arm-mcu 0)

set(demo_deps
  pico_stdlib
  hardware_gpio
  pico_stdio
  hardware_i2c
  hardware_adc
  hardware_spi
)

# not all of the demos work at the moment
set(demos
  adc
  blinker
  # can
  # dac
  gpio
  i2c
  interrupt_pin
  uart
  pwm
  pwm16
  spi
  blank
  # stream_dac

)

else()

set(demos
  adc
  blinker
  can
  dac
  gpio
  i2c
  interrupt_pin
  uart
  pwm
  pwm16
  spi
  blank
  stream_dac
)

set(demo_deps
  minimp3::minimp3
)
endif()

set(ENV{LIBHAL_PLATFORM_LIBRARY} arm-mcu)

libhal_build_demos(
  DEMOS
  ${demos}

  PACKAGES
  minimp3

  INCLUDES
  .

  LINK_LIBRARIES
  ${demo_deps}
)
